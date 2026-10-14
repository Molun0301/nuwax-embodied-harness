"""Mapping commits hand off completion; accepted bus history is never coalesced."""
import json
from pathlib import Path
import sqlite3
import tempfile
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

from contracts.geometry import Pose
from contracts.maps import OccupancyGrid
from contracts.occupancy_cells import PackedOccupancyCells
from contracts.events import RuntimeEventCategory
from contracts.streams import StreamFrame
from core import EventBus, StateNamespace, StateStore, StreamBus
from core.errors import GatewayError
from gateways.runtime import GatewayRuntime
from services.geometry.recording import MappingRecorder
from services.mapping.service import MappingService


class Interrupted(BaseException):
    pass


class MappingNotificationTests(unittest.TestCase):
    def setUp(self):
        self.stream, self.events, self.state = StreamBus(), EventBus(), StateStore()
        self.owner = NS(is_map_snapshot_source_current=lambda **kw: True)
        self.mapping = MappingService(provider_owner=self.owner, state_store=self.state,
                                      stream_bus=self.stream, event_bus=self.events)
        self.threads, self.releases, self.errors = [], [], []
        self.addCleanup(self.cleanup)

    def cleanup(self):
        for event in self.releases:
            event.set()
        self.mapping.close_notifications(3)
        self.events.shutdown()
        self.stream.shutdown()
        for thread in self.threads:
            thread.join(3)
            self.assertFalse(thread.is_alive())

    def spawn(self, action):
        def run():
            try:
                action()
            except BaseException as error:
                self.errors.append(error)
        thread = threading.Thread(target=run)
        self.threads.append(thread)
        thread.start()
        return thread

    def hold_navigation(self):
        entered, release = threading.Event(), threading.Event()
        self.releases.append(release)
        def observer(frame):
            entered.set()
            self.assertTrue(release.wait(5))
        self.stream.subscribe(observer, topics=('navigation.state',))
        self.spawn(lambda: self.stream.publish('navigation.state', source='navigation'))
        self.assertTrue(entered.wait(2))
        return release

    @staticmethod
    def grid():
        return OccupancyGrid(map_id='test-map', frame_id='map', width=2, height=1,
                             resolution_m=.1, origin=Pose(frame_id='map'),
                             data=PackedOccupancyCells(b'\x00\x64'))

    def accept(self, revision=1, epoch='first'):
        return self.mapping.ingest_external_snapshot(map_name='test-map', source_name='fixture',
            source_epoch=epoch, occupancy_grid=self.grid(), update_sequence_override=revision)

    def provider_update(self, revision=1, **metadata):
        return self.mapping._handle_provider_map_update({'map_name':'test-map', 'source_name':'fixture',
            'source_epoch':'first', 'occupancy_grid':self.grid(),
            'metadata': {'update_sequence':revision, **metadata}})

    def gateway(self, *, recorder=None, started=False):
        runtime=object.__new__(GatewayRuntime)
        runtime.mapping_service, runtime.stream_bus, runtime.event_bus = self.mapping, self.stream, self.events
        runtime.mapping_recorder=recorder
        runtime._recording_subscription_id = (self.stream.subscribe(recorder.record_stream_frame,
            topics=('map.frame','localization.pose','runtime.governance')) if recorder is not None else None)
        runtime.logged=[]
        runtime._event_log_subscription_id=self.events.subscribe(lambda event: runtime.logged.append(event.event_type))
        runtime._closing_event_log_subscription_id=None
        runtime._event_bridge_context=threading.local()
        runtime._runtime_event_log=NS(close=lambda timeout: True)
        runtime._owns_event_bus=runtime._owns_stream_bus=True
        runtime._dispatch_shutdown_complete=False
        runtime._dispatch_shutdown_blocker=None
        runtime._close_artifact_dependencies=lambda timeout: True
        runtime.scene_mapping_service=NS(close=lambda: None)
        runtime._close_memory_service=lambda: None
        runtime._lifecycle_lock=threading.RLock()
        runtime._started, runtime._closed=started,False
        runtime.task_manager=NS(shutdown=lambda **kwargs: None)
        runtime._join_exploration_finalizers=lambda: None
        runtime._stop_safety_watchdog_loop=lambda: None
        runtime.motion_guard=NS(has_connected=False,close=lambda: None)
        runtime._settle_relocalization_owner_before_geometry_stop=lambda **kw: {}
        runtime._stop_runtime_input_bridge=lambda: True
        runtime.platform_geometry_runtime=runtime.audio_service=runtime.robot=NS(stop=lambda: None)
        runtime._stop_data_frame_stream_sources=lambda: None
        runtime._begin_artifact_shutdown=lambda: None
        runtime.perception_video_runtime=NS(stop=lambda: None)
        runtime.person_follow_bridge=NS(close=lambda: None)
        return runtime

    def recorder(self):
        temp=tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        recorder=MappingRecorder(Path(temp.name), max_bytes=16*1024**2)
        self.assertTrue(recorder.start())
        self.addCleanup(recorder.close)
        return recorder

    def test_held_navigation_does_not_extend_provider_ticket_or_commit(self):
        self.stream.subscribe(lambda frame: None, topics=('map.frame','map.heartbeat'))
        release=self.hold_navigation()
        ticket=threading.Lock()
        returned=threading.Event()
        def provider():
            with ticket:
                snapshot=self.provider_update()
                self.assertIs(self.state.read(StateNamespace.MAP,'fixture').value,snapshot)
            returned.set()
        self.spawn(provider)
        self.assertTrue(returned.wait(1), 'notification held the provider delivery ticket')
        self.assertTrue(ticket.acquire(timeout=.1)); ticket.release()
        self.assertFalse(self.mapping.flush_notifications(.02))
        release.set()
        self.assertTrue(self.mapping.flush_notifications(2))
        self.assertEqual(self.errors,[])

    def test_heartbeat_only_suppressed_duplicate_rejected_do_not_wait(self):
        self.stream.subscribe(lambda frame: None, topics=('map.frame','map.heartbeat'))
        self.provider_update(2); self.assertTrue(self.mapping.flush_notifications(2))
        release=self.hold_navigation()
        done=threading.Event()
        def update():
            self.provider_update(event_type='heartbeat')
            self.provider_update(suppressed_live_map_update=True)
            self.provider_update(2)
            with self.assertRaises(GatewayError):
                self.provider_update(1)
            done.set()
        self.spawn(update)
        self.assertTrue(done.wait(1))
        frames=self.stream.replay(topics=('map.frame','map.heartbeat'))
        self.assertEqual([f.topic for f in frames].count('map.frame'),1)
        self.assertEqual([f.topic for f in frames].count('map.heartbeat'),5)
        release.set(); self.assertTrue(self.mapping.flush_notifications(2))
        self.assertEqual(self.errors,[])

    def test_async_frame_and_content_heartbeat_follow_state_store_commit(self):
        entered,release,observed=threading.Event(),threading.Event(),threading.Event()
        self.releases.append(release)
        seen=[]
        write=self.state.write
        def held_write(*args,**kwargs):
            entered.set(); self.assertTrue(release.wait(3))
            return write(*args,**kwargs)
        def observer(frame):
            seen.append((frame.topic,self.state.read(StateNamespace.MAP,'fixture').value.update_sequence))
            if frame.topic=='map.frame': observed.set()
        self.stream.subscribe(observer,topics=('map.frame','map.heartbeat'),async_dispatch=True)
        with patch.object(self.state,'write',side_effect=held_write):
            worker=self.spawn(lambda:self.provider_update())
            self.assertTrue(entered.wait(1))
            self.assertEqual(self.stream.replay(topics=('map.frame','map.heartbeat')),())
            release.set(); worker.join(2)
        self.assertTrue(observed.wait(1))
        self.assertEqual(seen,[('map.heartbeat',1),('map.frame',1)])
        self.assertEqual(self.errors,[])

    def test_ready_clear_new_epoch_retains_accepted_order_and_cache(self):
        release=self.hold_navigation(); seen=[]; events=[]
        self.stream.subscribe(lambda f:seen.append((f.cursor,f.status,getattr(f.payload,'source_epoch',None))),topics=('map.frame',))
        self.events.subscribe(lambda e:events.append(e.event_type))
        first=self.accept()
        self.mapping.clear_latest_snapshot(reason='switch')
        self.assertIsNone(self.stream.latest_frame('map.frame'))
        latest=self.accept(epoch='second')
        self.assertIs(self.stream.latest_frame('map.frame').payload,latest)
        release.set(); self.assertTrue(self.mapping.flush_notifications(2))
        self.assertEqual([(status,epoch) for _,status,epoch in seen],
                         [('available','first'),('cleared',None),('available','second')])
        self.assertEqual([cursor for cursor,_,_ in seen],sorted(cursor for cursor,_,_ in seen))
        self.assertEqual(events,['mapping.snapshot_ready','mapping.snapshot_cleared','mapping.snapshot_ready'])
        self.assertIs(self.mapping.get_latest_snapshot(),latest)
        self.assertIsNot(first,latest)

    def test_close_held_retains_subscriptions_then_exact_sqlite_order(self):
        recorder=self.recorder(); runtime=self.gateway(recorder=recorder)
        release=self.hold_navigation()
        self.accept(); self.mapping.clear_latest_snapshot(); self.accept(epoch='second')
        localization=self.stream.prepare_publication(StreamFrame(topic='localization.pose',source='fixture',payload={}))
        self.assertFalse(runtime._close_dispatch_resources(.03))
        self.assertEqual(runtime._dispatch_shutdown_blocker,'mapping_notifications_in_flight')
        self.assertIn(runtime._recording_subscription_id,self.stream._subscriptions)
        self.assertIn(runtime._event_log_subscription_id,self.events._subscriptions)
        self.assertFalse(recorder.status()['closed'])
        release.set(); self.stream.dispatch_prepared(localization)
        self.assertTrue(runtime._close_dispatch_resources(2))
        self.assertTrue(recorder.status()['complete'])
        with sqlite3.connect(recorder.status()['path']) as connection:
            rows=connection.execute('SELECT kind,payload FROM records ORDER BY id').fetchall()
        self.assertEqual([kind for kind,_ in rows],['mapping.state']*3+['localization.pose'])
        states=[json.loads(payload) for kind,payload in rows if kind=='mapping.state']
        self.assertEqual([(s['status'],s['source_epoch']) for s in states],
                         [('available','first'),('cleared',None),('available','second')])
        self.assertEqual(len({s['cursor'] for s in states}),3)

    def test_postcommit_queue_full_and_shutdown_mark_recording_incomplete(self):
        for failure in ('queue_full','shutdown'):
            with self.subTest(failure=failure):
                # Admission failure simulation is confined to the actual bus
                # boundary; authoritative state still goes through real code.
                recorder=self.recorder(); runtime=self.gateway(recorder=recorder)
                if failure=='queue_full':
                    context=patch('core.stream_bus._PREPARED_DISPATCH_QUEUE_MAXSIZE',0)
                else:
                    self.assertTrue(self.stream.shutdown())
                    from contextlib import nullcontext
                    context=nullcontext()
                with context:
                    snapshot=self.accept(2 if failure=='shutdown' else 1)
                self.assertIs(self.state.read(StateNamespace.MAP,'fixture').value,snapshot)
                self.assertGreater(self.mapping.notification_status()['map_frame_admission_gaps'],0)
                self.assertTrue(runtime._close_mapping_recording(2))
                status=recorder.status()
                self.assertTrue(status['incomplete']);self.assertFalse(status['complete'])
                self.assertEqual(status['dropped_records'],0)
                with sqlite3.connect(status['path']) as connection:
                    persisted=json.loads(connection.execute("SELECT value FROM info WHERE key='status'").fetchone()[0])
                self.assertTrue(persisted['incomplete']);self.assertFalse(persisted['complete'])
                # A separate service is needed after its terminal seal.
                if failure=='queue_full':
                    self.mapping=MappingService(provider_owner=self.owner,state_store=self.state,
                                               stream_bus=self.stream,event_bus=self.events)

    def test_event_tail_handoff_queues_every_reservation(self):
        seen=[]; self.events.subscribe(lambda event:seen.append(event.payload['n']))
        publications=[self.events.prepare(self.events.build_event('mapping.snapshot_ready',
            category=RuntimeEventCategory.ROBOT,source='fixture',payload={'n':n})) for n in range(3)]
        for publication in publications:self.events.enqueue_prepared(publication)
        self.events.dispatch_prepared(publications[-1])
        self.assertEqual(seen,[0,1,2])

    def test_owner_survives_baseexception_tail_is_not_replayed(self):
        seen,tail=[],[]
        def interrupt(frame):
            seen.append(frame.payload.update_sequence)
            if frame.payload.update_sequence==1:raise Interrupted('one')
        self.stream.subscribe(interrupt,topics=('map.frame',))
        self.stream.subscribe(lambda frame:tail.append(frame.payload.update_sequence),topics=('map.frame',))
        self.accept();self.assertTrue(self.mapping.flush_notifications(2))
        self.accept(2);self.assertTrue(self.mapping.flush_notifications(2))
        self.assertEqual(seen,[1,2]);self.assertEqual(tail,[2])
        status=self.mapping.notification_status()
        self.assertEqual(status['map_frame_interrupted_attempts'],1)
        self.assertEqual(status['dispatch_failures'],1)

    def test_other_drainer_interruption_does_not_claim_exact_missing_rows(self):
        recorder=self.recorder();runtime=self.gateway(recorder=recorder)
        release=self.hold_navigation()
        # Recorder precedes the interrupt: the row exists even though the map
        # attempt was truncated. The gap records uncertainty, not a missing row.
        self.stream.subscribe(lambda frame: (_ for _ in ()).throw(Interrupted()),topics=('map.frame',))
        self.accept();release.set()
        self.assertTrue(self.mapping.flush_notifications(2))
        self.assertTrue(runtime._close_dispatch_resources(2))
        status=recorder.status()
        self.assertFalse(status['complete']);self.assertTrue(status['incomplete'])
        self.assertEqual(status['dropped_records'],0)
        self.assertEqual(status['notification_gaps'],{'map_frame_interrupted_attempts':1})
        with sqlite3.connect(status['path']) as connection:
            self.assertEqual(connection.execute("SELECT count(*) FROM records WHERE kind='mapping.state'").fetchone()[0],1)
        self.assertEqual(len(self.errors),1);self.assertIsInstance(self.errors[0],Interrupted)

    def test_reentrant_map_acceptance_preserves_fifo(self):
        seen=[]
        def observe(frame):
            seen.append(frame.payload.update_sequence)
            if frame.payload.update_sequence==1:self.accept(2)
        self.stream.subscribe(observe,topics=('map.frame',))
        self.accept();self.assertTrue(self.mapping.flush_notifications(2))
        self.assertEqual(seen,[1,2])

    def test_callback_close_from_other_drainer_returns_incomplete_without_selfwait(self):
        runtime=self.gateway();release=self.hold_navigation();results=[]
        def close(frame):
            start=time.monotonic()
            results.append((runtime._close_dispatch_resources(.5),time.monotonic()-start))
        self.stream.subscribe(close,topics=('map.frame',))
        self.accept();release.set()
        self.assertTrue(self.mapping.flush_notifications(2))
        self.assertEqual(results[0][0],False);self.assertLess(results[0][1],.2)
        self.assertTrue(runtime._close_dispatch_resources(2))

    def test_no_sync_heartbeat_keeps_earlier_map_completion_fence(self):
        self.stream.subscribe(lambda frame: None, topics=('map.frame',))
        owner=self.mapping._notifications
        with patch.object(owner, '_start_locked', return_value=None):
            self.accept()
            old=owner._stream
            self.mapping._publish_map_heartbeat(source_name='fixture', metadata={})
            self.assertIs(owner._stream, old)
            self.assertEqual(old.frame.topic, 'map.frame')
        self.assertTrue(self.mapping.flush_notifications(2))

    def test_failed_worker_start_retains_commit_and_retries_fences(self):
        seen=[]
        self.stream.subscribe(lambda frame:seen.append(frame.payload.update_sequence),topics=('map.frame',))
        with patch('threading.Thread.start', side_effect=RuntimeError('temporary thread limit')):
            snapshot=self.accept()
        self.assertIs(self.state.read(StateNamespace.MAP,'fixture').value,snapshot)
        self.assertTrue(self.mapping.notification_status()['pending'])
        self.assertTrue(self.mapping.flush_notifications(2))
        self.assertEqual(seen,[1])
        self.assertEqual(self.mapping.notification_status()['map_frame_admission_gaps'],0)

    def test_event_callback_close_never_waits_for_its_own_fence(self):
        runtime=self.gateway(); results=[]
        def close(event):
            start=time.monotonic()
            results.append((runtime._close_dispatch_resources(.5), time.monotonic()-start))
        self.events.subscribe(close,event_types=('mapping.snapshot_ready',))
        self.accept();self.assertTrue(self.mapping.flush_notifications(2))
        self.assertFalse(results[0][0]);self.assertLess(results[0][1],.2)
        self.assertTrue(runtime._close_dispatch_resources(2))

    def test_recorder_writer_timeout_retains_event_bridge_and_can_finish(self):
        recorder=self.recorder();runtime=self.gateway(recorder=recorder)
        entered,release=threading.Event(),threading.Event()
        self.releases.append(release)
        write=recorder._write_batch
        def held_write(connection,batch):
            entered.set(); self.assertTrue(release.wait(3)); write(connection,batch)
        with patch.object(recorder,'_write_batch',side_effect=held_write):
            self.accept();self.assertTrue(self.mapping.flush_notifications(2))
            self.assertTrue(entered.wait(1))
            self.assertFalse(runtime._close_dispatch_resources(.02))
            self.assertEqual(runtime._dispatch_shutdown_blocker,'mapping_recording_in_flight')
            self.assertIn(runtime._event_log_subscription_id,self.events._subscriptions)
            self.assertFalse(recorder.status()['closed'])
            release.set();self.assertTrue(runtime._close_dispatch_resources(2))
        self.assertTrue(recorder.status()['complete'])
        with sqlite3.connect(recorder.status()['path']) as connection:
            self.assertEqual(connection.execute("SELECT count(*) FROM records WHERE kind='mapping.state'").fetchone()[0],1)

    def test_shutdown_cancellation_never_reports_recording_complete(self):
        recorder=self.recorder();runtime=self.gateway(recorder=recorder)
        release=self.hold_navigation()
        self.accept()
        self.assertFalse(self.stream.shutdown(.01))
        self.assertTrue(self.mapping.flush_notifications(1))
        self.assertTrue(runtime._close_mapping_recording(1))
        status=recorder.status()
        self.assertTrue(status['incomplete']);self.assertFalse(status['complete'])
        self.assertEqual(status['notification_gaps'],{'map_frame_cancelled_attempts':1})
        self.assertEqual(status['dropped_records'],0)
        with sqlite3.connect(status['path']) as connection:
            self.assertEqual(connection.execute("SELECT count(*) FROM records WHERE kind='mapping.state'").fetchone()[0],0)
        release.set()

    def test_shutdown_skipped_active_map_tail_records_potential_gap_once(self):
        entered,release=threading.Event(),threading.Event()
        self.releases.append(release)
        self.stream.subscribe(lambda frame:(entered.set(),release.wait(3)),topics=('map.frame',))
        recorder=self.recorder();runtime=self.gateway(recorder=recorder)
        self.stream.subscribe(lambda frame:None,topics=('map.frame',))
        self.accept();self.assertTrue(entered.wait(1))
        self.assertFalse(self.stream.shutdown(.01))
        release.set();self.assertTrue(self.mapping.flush_notifications(2))
        self.assertTrue(runtime._close_mapping_recording(2))
        self.assertEqual(recorder.status()['notification_gaps'],{'map_frame_cancelled_attempts':1})
        self.assertFalse(recorder.status()['complete'])

    def test_source_unsubscribe_finishes_before_notification_seal(self):
        # The provider retains its real callback ticket until commit/handoff,
        # then unsubscribe fences that ticket without observer completion.
        release=self.hold_navigation(); ticket=threading.Lock(); unsubscribed=[]
        entered=threading.Event()
        def unsubscribe(identity):
            with ticket:
                unsubscribed.append(identity)
                self.assertFalse(self.mapping._notification_admission_closed)
        self.owner.unsubscribe_map_updates=unsubscribe
        self.mapping._map_update_subscription_id=7
        self.stream.subscribe(lambda frame:None,topics=('map.frame',))
        with ticket:
            self.accept()
            closer=self.spawn(lambda:(entered.set(),self.mapping.close_notifications(.5)))
            self.assertTrue(entered.wait(1))
            self.assertFalse(self.mapping._notification_admission_closed)
        closer.join(1)
        self.assertEqual(unsubscribed,[7])
        self.assertFalse(self.mapping.notification_status()['accepting'])
        self.assertTrue(self.mapping.notification_status()['pending'])
        release.set();self.assertTrue(self.mapping.close_notifications(2))

    def test_both_gateway_stop_paths_flush_before_teardown(self):
        for started in (False,True):
            with self.subTest(started=started):
                # Use a fresh mapping owner; stop is terminal for every owner.
                if started:
                    self.events,self.stream=EventBus(),StreamBus()
                    self.mapping=MappingService(provider_owner=self.owner,state_store=self.state,
                        stream_bus=self.stream,event_bus=self.events)
                runtime=self.gateway(started=started)
                self.accept()
                runtime.stop()
                self.assertTrue(runtime._dispatch_shutdown_complete)
                self.assertIn('mapping.snapshot_ready',runtime.logged)
                self.assertIsNone(runtime._event_log_subscription_id)
                self.assertTrue(self.mapping.notification_status()['closed'])


if __name__=='__main__':unittest.main()
