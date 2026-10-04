#!/usr/bin/env python3
"""无设备地图修订、捕获、兼容性及有序发布回归，不运行内容摘要。"""
from __future__ import annotations
import json, struct, sys, tempfile, threading, unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from contracts.geometry import Pose
from contracts.maps import OccupancyGrid
from contracts.map_workspace import MapAssetBundle
from contracts.occupancy_cells import PackedOccupancyCells
from contracts.runtime_views import MapSnapshot
from contracts.streams import StreamFrame
from core import EventBus, StateStore, StreamBus
from core.errors import GatewayError
from services.mapping.service import MappingService, _snapshot_binding
from services.mapping.asset_store import MapAssetBundleRepository
from services.memory.scene_mapping_state import snapshot_binding
import core.stream_bus as streams
import services.mapping.service as mapping
from gateways.ws.data_streams import _encode_ws_frame, _format_sse_payload, build_data_stream_router

def grid(cells=None,packed=False):
    cells=[0,-1,100] if cells is None else cells
    data=PackedOccupancyCells(bytes(v&255 for v in cells)) if packed else list(cells)
    return OccupancyGrid(map_id='地图',frame_id='map',width=len(cells),height=1,resolution_m=.1,origin=Pose(frame_id='map'),data=data)
def snapshot(sequence=1,epoch='run-1'):
    return MapSnapshot(map_name='地图',source_name='owner',source_epoch=epoch,update_sequence=sequence,occupancy_grid=grid())

class SnapshotChecks(unittest.TestCase):
    def test_legacy_conversion_keeps_identity_ignores_old_digest_and_emits_v2(self):
        old=snapshot(7).model_dump(mode='json');old.pop('schema_version');old['content_sha256']='historical untrusted text'
        value=MapSnapshot.model_validate(old)
        self.assertEqual((value.schema_version,value.source_epoch,value.update_sequence),(2,'run-1',7))
        self.assertNotIn('content_sha256',value.model_dump_json())
        for version in (2,3,True,1.0):
            with self.assertRaises(ValueError):MapSnapshot.model_validate({**old,'schema_version':version})
    def test_current_and_legacy_snapshot_reopen_preserve_exact_binding(self):
        current=snapshot(9007199254740993);old=current.model_dump(mode='json');old.pop('schema_version');old['content_sha256']='old'
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)/'snapshot.json'
            for payload in (current.model_dump(mode='json'),old):
                p.write_text(json.dumps(payload));value=MapSnapshot.model_validate_json(p.read_text())
                self.assertEqual(snapshot_binding(value),snapshot_binding(current))
        self.assertEqual(snapshot_binding(current)['update_sequence'],'9007199254740993')
    def test_bundle_identity_does_not_copy_finalized_layers(self):
        bundle=MapAssetBundle(map_name='地图',source_name='owner',frame_id='map',snapshot=snapshot())
        object.__setattr__(bundle,'manifest_sha256','a'*64) # 旧格式字面输入，不计算或校验
        with patch('contracts.map_workspace.deepcopy',side_effect=AssertionError('identity copied')):
            self.assertEqual(bundle.snapshot_identity,snapshot_binding(snapshot()))
    def test_legacy_bundle_conversion_is_stable_and_preserves_canonical_identity(self):
        with tempfile.TemporaryDirectory() as directory:
            repo=MapAssetBundleRepository(directory);source=Path(directory)/'old-version'
            doc={'map_name':'地图','source_name':'owner','frame_id':'map','snapshot':snapshot(7).model_dump(mode='json')}
            first=repo._convert_legacy_bundle('地图',source,doc)
            self.assertEqual((first.snapshot.source_epoch,first.snapshot.update_sequence),('run-1',7))
            for key in ('source_epoch','update_sequence','schema_version'):doc['snapshot'].pop(key)
            doc['snapshot'].update(version_id='saved-one',revision=3,content_sha256='old')
            first=repo._convert_legacy_bundle('地图',source,doc);second=repo._convert_legacy_bundle('地图',source,doc)
            self.assertEqual(first.snapshot_identity,second.snapshot_identity)
            self.assertEqual(first.snapshot.update_sequence,3)

class OwnerChecks(unittest.TestCase):
    def setUp(self):
        self.bus=StreamBus();self.events=EventBus();self.addCleanup(self.bus.shutdown);self.addCleanup(self.events.shutdown)
        self.owner=NS(is_map_snapshot_source_current=lambda **kw:True)
        self.service=MappingService(provider_owner=self.owner,state_store=StateStore(),stream_bus=self.bus,event_bus=self.events)
    def admit(self,sequence=1,epoch='run-1',cells=None,metadata=None,value=None,**kw):
        return self.service.ingest_external_snapshot(map_name='地图',source_name='owner',source_epoch=epoch,
            update_sequence_override=sequence,occupancy_grid=value or grid(cells),metadata=metadata,**kw)
    def test_duplicate_timestamp_only_reopen_and_same_revision_conflict(self):
        first=self.admit();self.assertIs(self.admit(),first);self.assertEqual(self.bus.latest_cursor(),1)
        with self.assertRaises(GatewayError) as raised:self.admit(cells=[100,-1,0])
        self.assertEqual(raised.exception.error_code,'map_snapshot_revision_conflict')
        self.assertIs(self.service.get_latest_snapshot(),first)
    def test_owned_capture_detaches_source_and_reads_do_not_copy(self):
        original=grid();saved=self.admit(value=original);original.data[0]=100;original.origin.position.x=9
        self.assertEqual(saved.occupancy_grid.data[0],0);self.assertEqual(saved.occupancy_grid.origin.position.x,0)
        with patch.object(mapping,'_capture_map_layer',side_effect=AssertionError('read copied')):
            for _ in range(1000):self.assertIs(self.service.get_latest_snapshot(),saved)
        with self.assertRaises(GatewayError):self.admit(value=original)
    def test_packed_capture_reuses_immutable_bytes(self):
        original=grid(packed=True);saved=self.admit(value=original)
        self.assertIs(saved.occupancy_grid.data,original.data);self.assertIsNot(saved.occupancy_grid.origin.position,original.origin.position)
    def test_late_live_rejected_asset_rollback_and_new_epoch_allowed(self):
        self.admit(3)
        with self.assertRaises(GatewayError):self.admit(2)
        restored=self.admit(1,metadata={'source_kind':'map_asset_bundle'});self.assertEqual(restored.update_sequence,1)
        new=self.admit(1,epoch='replacement',cells=[100,0,0]);self.assertEqual(new.source_epoch,'replacement')
        self.owner.is_map_snapshot_source_current=lambda **kw:False
        with self.assertRaises(GatewayError):self.admit(5)
        self.assertIs(self.service.get_latest_snapshot(),new)
    def test_saved_reference_pins_and_explicit_clear_changes_only_small_binding(self):
        saved=self.admit(epoch='asset-1',metadata={'source_kind':'map_asset_bundle'});binding=_snapshot_binding(saved)
        live=self.admit(2,epoch='live',metadata={'semantic_reference_identity':binding})
        self.assertEqual(self.service.get_semantic_snapshot().source_epoch,'asset-1');self.assertEqual(live.metadata['semantic_map_binding'],binding)
        cleared=self.admit(2,epoch='live',metadata={'semantic_reference_identity':None})
        self.assertEqual(cleared.update_sequence,live.update_sequence);self.assertIs(self.service.get_semantic_snapshot(),cleared)
        replaced=self.admit(epoch='asset-2',metadata={'source_kind':'map_asset_bundle'})
        self.assertEqual(snapshot_binding(self.service.get_semantic_snapshot()),snapshot_binding(replaced))
    def test_display_view_has_separate_identity_and_semantic_binding(self):
        accepted=self.admit(stream_occupancy_grid=grid([100,-1,0]),stream_occupancy_source='physical-raw')
        display=self.bus.latest_frame('map.frame').payload
        self.assertNotEqual(display.source_epoch,accepted.source_epoch);self.assertEqual(display.metadata['semantic_map_binding'],_snapshot_binding(accepted))
    def test_display_only_updates_do_not_alias_the_unchanged_navigation_revision(self):
        self.admit(stream_occupancy_grid=grid([100,-1,0]),stream_occupancy_source='raw')
        first=self.bus.latest_frame('map.frame').payload
        self.admit(stream_occupancy_grid=grid([0,100,0]),stream_occupancy_source='raw')
        second=self.bus.latest_frame('map.frame').payload
        self.assertEqual(self.service.get_latest_snapshot().update_sequence,1)
        self.assertEqual(first.source_epoch,second.source_epoch)
        self.assertGreater(second.update_sequence,first.update_sequence)
        self.assertEqual(second.occupancy_grid.data[1],100)
        self.assertEqual(second.metadata['occupancy_content_revision'],second.update_sequence)
    def test_failed_prepare_does_not_advance_state_and_exact_retry_publishes(self):
        with patch.object(streams,'_PREPARED_DISPATCH_QUEUE_MAXSIZE',0):
            with self.assertRaises(RuntimeError):self.admit(7)
        self.assertIsNone(self.service.get_latest_snapshot());self.assertEqual(self.bus.latest_cursor(),0)
        accepted=self.admit(7);self.assertEqual(self.bus.latest_frame('map.frame').payload.update_sequence,7)
        with patch.object(streams,'_PREPARED_DISPATCH_QUEUE_MAXSIZE',0):
            with self.assertRaises(RuntimeError):self.service.clear_latest_snapshot()
        self.assertIs(self.service.get_latest_snapshot(),accepted)
        self.service.clear_latest_snapshot();self.assertIsNone(self.service.get_latest_snapshot())
    def test_native_display_revision_conflict_and_late_frame_are_rejected(self):
        options={'stream_occupancy_source':'raw','stream_occupancy_metadata':{'occupancy_content_revision':4}}
        self.admit(stream_occupancy_grid=grid([100,0,0]),**options)
        with self.assertRaises(GatewayError) as raised:self.admit(stream_occupancy_grid=grid([0,100,0]),**options)
        self.assertEqual(raised.exception.error_code,'map_display_revision_conflict')
        options['stream_occupancy_metadata']={'occupancy_content_revision':3}
        with self.assertRaises(GatewayError) as raised:self.admit(2,stream_occupancy_grid=grid([100,0,0]),**options)
        self.assertEqual(raised.exception.error_code,'map_display_stale_revision')
        self.assertEqual(self.service.get_latest_snapshot().update_sequence,1)
    def test_display_revision_reset_is_allowed_for_a_different_full_source_scope(self):
        self.admit(stream_occupancy_grid=grid(),stream_occupancy_source='raw',stream_occupancy_metadata={'occupancy_content_revision':5})
        accepted=self.service.ingest_external_snapshot(map_name='地图',source_name='other-owner',source_epoch='run-1',
            update_sequence_override=1,occupancy_grid=grid(),stream_occupancy_grid=grid(),
            stream_occupancy_source='raw',stream_occupancy_metadata={'occupancy_content_revision':1})
        self.assertEqual(accepted.source_name,'other-owner')
        self.assertEqual(self.bus.latest_frame('map.frame').payload.update_sequence,1)
    def test_retained_full_scope_highwater_rejects_late_old_owner_after_switch(self):
        self.admit(10,epoch='old');self.admit(1,epoch='new')
        with self.assertRaises(GatewayError):self.admit(9,epoch='old')
        self.assertEqual(self.service.get_latest_snapshot().source_epoch,'new')
    def test_content_revision_fallback_shares_display_conflict_rule(self):
        options={'stream_occupancy_source':'raw','stream_occupancy_metadata':{'content_revision':7}}
        self.admit(stream_occupancy_grid=grid(),**options)
        with self.assertRaises(GatewayError):self.admit(stream_occupancy_grid=grid([100,0,0]),**options)
        self.assertEqual(self.bus.latest_frame('map.frame').payload.update_sequence,7)
    def test_unseeded_same_name_replacement_retires_saved_pin(self):
        self.admit(epoch='asset-1',metadata={'source_kind':'map_asset_bundle'})
        accepted=self.admit(epoch='new-live')
        self.assertIs(self.service.get_semantic_snapshot(),accepted)
    def test_generic_refresh_conflicts_and_attested_owner_refresh_is_constant_work(self):
        current=[snapshot()];self.owner.get_map_snapshot=lambda:current[0]
        first=self.service.refresh()
        current[0]=snapshot();current[0].occupancy_grid.data[0]=100
        with self.assertRaises(GatewayError):self.service.refresh()
        current[0]=snapshot();current[0]._source_owner_id=id(self.owner)
        with patch.object(mapping,'_capture_map_layer',side_effect=AssertionError('trusted refresh copied')):
            self.assertIs(self.service.refresh(),first)
        self.assertNotIn('_source_owner_id',current[0].model_dump_json())
    def test_small_reference_identity_is_detached(self):
        saved=self.admit(epoch='asset',metadata={'source_kind':'map_asset_bundle'})
        binding=_snapshot_binding(saved);accepted=self.admit(2,epoch='live',metadata={'semantic_reference_identity':binding})
        binding['source_epoch']='changed-after-admission'
        self.assertEqual(accepted.metadata['semantic_reference_identity']['source_epoch'],'asset')
    def test_concurrent_commit_enqueue_order_and_read_progress(self):
        seen=[];errors=[];entered=threading.Event();release=threading.Event();self.addCleanup(release.set)
        self.bus.subscribe(lambda f:seen.append(f.payload.update_sequence),topics=('map.frame',));dispatch=self.bus.dispatch_prepared
        def blocked(publication):
            if publication.frame.payload.update_sequence==2:entered.set();self.assertTrue(release.wait(3))
            return dispatch(publication)
        def run():
            try:self.admit(2)
            except BaseException as error:errors.append(error)
        with patch.object(self.bus,'dispatch_prepared',side_effect=blocked):
            worker=threading.Thread(target=run);worker.start();self.assertTrue(entered.wait(2))
            self.assertEqual(self.service.get_latest_snapshot().update_sequence,2);self.admit(3)
            release.set();worker.join(2);self.assertFalse(worker.is_alive())
        self.assertFalse(errors);self.assertEqual(seen,[2,3]);self.assertEqual(self.bus.latest_frame('map.frame').payload.update_sequence,3)
    def test_callbacks_reenter_outside_mapping_locks_and_clear_is_ordered(self):
        seen=[]
        def callback(frame):
            self.assertFalse(self.service._publication_lock._is_owned());self.assertFalse(self.service._lock._is_owned())
            seen.append(frame.payload.update_sequence if frame.payload else None)
            if frame.payload and frame.payload.update_sequence==1:self.admit(2)
        self.bus.subscribe(callback,topics=('map.frame',));self.admit();self.service.clear_latest_snapshot(reason='test')
        self.assertEqual(seen,[1,2,None]);self.assertIsNone(self.service.get_latest_snapshot());self.assertIsNone(self.bus.latest_frame('map.frame'))

class RuntimeOwnershipChecks(unittest.TestCase):
    def test_captured_generation_cannot_cross_same_name_activation_replacement(self):
        from services.geometry.runtime import PlatformGeometryRuntime
        runtime=object.__new__(PlatformGeometryRuntime)
        runtime._runtime_state_lock=threading.RLock();runtime._workspace_map_name='地图';runtime._mapping_session_context=None
        generation=[1];runtime._map_activation_snapshot_locked=lambda:NS(generation=generation[0],bundle=None)
        runtime._map_activation_snapshot_read=lambda:NS(generation=generation[0],bundle=None)
        runtime._map_asset_activation_fence_snapshot=lambda:None
        payload={'map_name':'地图','occupancy_grid':grid(),'metadata':{}}
        accepted=runtime._attach_active_map_bundle_to_live_map_payload(payload)
        self.assertEqual(accepted['metadata']['map_activation_generation'],1)
        self.assertTrue(runtime.is_map_snapshot_source_current(map_name='地图',metadata=accepted['metadata']))
        generation[0]=2
        self.assertFalse(runtime.is_map_snapshot_source_current(map_name='地图',metadata=accepted['metadata']))

class PreparedChecks(unittest.TestCase):
    def setUp(self):
        self.bus=StreamBus();self.addCleanup(self.bus.shutdown);self.seen=[]
        self.bus.subscribe(lambda f:self.seen.append(f.revision),topics=('synthetic',))
    def frame(self,seq):return StreamFrame(topic='synthetic',source='owner',revision=str(seq))
    def test_no_prepare_callbacks_and_abandoned_work_adopted_by_next_dispatch(self):
        p=self.bus.prepare_publication(self.frame(1));self.assertEqual(self.seen,[])
        self.bus.publish_frame(self.frame(2));self.assertEqual(self.seen,['1','2'])
        self.bus.dispatch_prepared(p);self.assertEqual(self.seen,['1','2'])
    def test_shutdown_completes_pending_and_foreign_owner_rejected(self):
        p=self.bus.prepare_publication(self.frame(1));other=StreamBus();self.addCleanup(other.shutdown)
        with self.assertRaises(ValueError):other.dispatch_prepared(p)
        self.bus.shutdown();self.assertEqual(self.bus.dispatch_prepared(p).revision,'1');self.assertEqual(self.seen,[])
        with self.assertRaises(RuntimeError):self.bus.prepare_publication(self.frame(2))
    def test_unclaimed_preparations_bounded_before_failed_cursor_allocation(self):
        with patch.object(streams,'_PREPARED_DISPATCH_QUEUE_MAXSIZE',2):
            self.bus.prepare_publication(self.frame(1));self.bus.prepare_publication(self.frame(2))
            with self.assertRaises(RuntimeError):self.bus.prepare_publication(self.frame(3))
            self.assertEqual(self.bus.latest_cursor(),2)
    def test_cross_thread_dispatch_and_reentrant_publish_preserve_order(self):
        def callback(f):
            if f.revision=='1':self.bus.publish_frame(self.frame(3))
        self.bus.subscribe(callback,topics=('synthetic',));p=self.bus.prepare_publication(self.frame(1));self.bus.prepare_publication(self.frame(2))
        worker=threading.Thread(target=lambda:self.bus.dispatch_prepared(p));worker.start();worker.join(2)
        self.assertFalse(worker.is_alive());self.assertEqual(self.seen,['1','2','3'])

    def test_pending_shutdown_releases_waiters_without_running_their_callbacks(self):
        entered=threading.Event();release=threading.Event();finished=threading.Event();self.addCleanup(release.set)
        def blocker(frame):
            if frame.revision=='1':entered.set();release.wait(3)
        self.bus.subscribe(blocker,topics=('synthetic',))
        first=threading.Thread(target=lambda:self.bus.publish_frame(self.frame(1)));first.start();self.assertTrue(entered.wait(2))
        prepared=self.bus.prepare_publication(self.frame(2))
        second=threading.Thread(target=lambda:(self.bus.dispatch_prepared(prepared),finished.set()));second.start()
        self.bus.shutdown();self.assertTrue(finished.wait(1));release.set();first.join(2);second.join(2)
        self.assertEqual(self.seen,['1'])

    def test_async_latest_only_coalescing_and_gap_reporting_survive_prepare(self):
        entered=threading.Event();release=threading.Event();received=threading.Event();notices=[];seen=[];self.addCleanup(release.set)
        def callback(frame):
            seen.append(frame.revision)
            if frame.revision=='0':entered.set();release.wait(3)
            if frame.revision=='3':received.set()
        self.bus.subscribe(callback,topics=('async',),async_dispatch=True,latest_only_topics=('async',),on_gap=notices.append)
        self.bus.publish('async',source='owner',revision='0');self.assertTrue(entered.wait(2))
        for revision in ['1','2','3']:
            token=self.bus.prepare_publication(StreamFrame(topic='async',source='owner',revision=revision))
            self.bus.dispatch_prepared(token)
        release.set();self.assertTrue(received.wait(2));self.assertEqual(seen,['0','3']);self.assertEqual(notices,[])
        gap_entered=threading.Event();gap_release=threading.Event();gap_seen=threading.Event();self.addCleanup(gap_release.set)
        def event_callback(frame):
            if frame.revision=='0':gap_entered.set();gap_release.wait(3)
        def gap_callback(value):notices.append(value);gap_seen.set()
        self.bus.subscribe(event_callback,topics=('events',),async_dispatch=True,on_gap=gap_callback)
        self.bus.publish('events',source='owner',revision='0');self.assertTrue(gap_entered.wait(2))
        with patch.object(streams,'_ASYNC_DISPATCH_QUEUE_MAXSIZE',2):
            for revision in ['1','2','3']:self.bus.publish('events',source='owner',revision=revision)
        gap_release.set();self.assertTrue(gap_seen.wait(2));self.assertTrue(notices)

class TransportChecks(unittest.TestCase):
    def frame(self):
        value=snapshot(9007199254740993)
        return StreamFrame(topic='map.frame',source='mapping_service',frame_id='map',payload=value,revision=str(value.update_sequence),cursor=1)
    def test_json_sse_and_v3_binary_keep_exact_sequence(self):
        frame=self.frame();binary=_encode_ws_frame(frame,map_binary_version=3);self.assertEqual(binary[:9],b'NUWAXMAP\x03')
        size=struct.unpack('<I',binary[9:13])[0];payload=json.loads(binary[13:13+size])['frame']['payload']
        self.assertEqual((payload['schema_version'],payload['update_sequence']),(2,'9007199254740993'));self.assertNotIn('content_sha256',payload)
        for version in (None,2,4,'3'):
            payload=json.loads(_encode_ws_frame(frame,map_binary_version=version))['frame']['payload']
            self.assertEqual(payload['update_sequence'],'9007199254740993');self.assertEqual(payload['occupancy_grid']['data'],[0,-1,100])
        self.assertEqual(json.loads(_format_sse_payload(frame).split('data: ',1)[1])['payload']['update_sequence'],'9007199254740993')
    def test_two_clients_negotiate_independently_for_replay_live_and_clear(self):
        from fastapi import FastAPI
        from fastapi.testclient import TestClient
        bus=StreamBus();self.addCleanup(bus.shutdown)
        runtime=NS(stream_bus=bus,config=NS(gateway=NS(ws_ping_interval_sec=60)))
        app=FastAPI();app.include_router(build_data_stream_router(runtime,None));bus.publish_frame(self.frame())
        with TestClient(app) as client:
            with client.websocket_connect('/streams/ws?topics=map.frame') as old:
                self.assertEqual(old.receive_json()['frame']['payload']['schema_version'],2);self.assertEqual(old.receive_json()['type'],'stream.replay')
                with client.websocket_connect('/streams/ws?topics=map.frame&map_binary_version=3') as new:
                    self.assertEqual(new.receive_bytes()[:9],b'NUWAXMAP\x03');self.assertEqual(new.receive_json()['type'],'stream.replay')
                    bus.publish_frame(self.frame());self.assertEqual(old.receive_json()['type'],'stream.frame');self.assertEqual(new.receive_bytes()[:9],b'NUWAXMAP\x03')
                    bus.publish('map.frame',source='mapping_service',status='cleared',payload=None)
                    self.assertEqual(old.receive_json()['frame']['status'],'cleared');self.assertEqual(new.receive_json()['frame']['status'],'cleared')

    def test_reconnect_unknown_capability_and_identity_only_channel(self):
        from fastapi import FastAPI
        from fastapi.testclient import TestClient
        bus=StreamBus();self.addCleanup(bus.shutdown)
        runtime=NS(stream_bus=bus,config=NS(gateway=NS(ws_ping_interval_sec=60)))
        app=FastAPI();app.include_router(build_data_stream_router(runtime,None));bus.publish_frame(self.frame())
        with TestClient(app) as client:
            for capability in ['99','3']:
                with client.websocket_connect('/streams/ws?topics=map.frame&map_binary_version='+capability) as socket:
                    first=socket.receive()
                    self.assertEqual('bytes' in first,capability=='3');self.assertEqual(socket.receive_json()['type'],'stream.replay')
            with client.websocket_connect('/streams/ws?topics=localization.pose,map.frame&map_payload=identity&map_binary_version=3') as socket:
                first=socket.receive_json();self.assertEqual(first['type'],'stream.map_identity');self.assertEqual(first['source_epoch'],'run-1')
                self.assertEqual(socket.receive_json()['type'],'stream.replay')
                bus.publish('map.frame',source='mapping_service',status='cleared',payload=None)
                self.assertTrue(socket.receive_json()['cleared'])

if __name__=='__main__':unittest.main(verbosity=2)
