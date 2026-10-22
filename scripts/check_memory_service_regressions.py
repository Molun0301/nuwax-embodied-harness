#!/usr/bin/env python3
"""Offline service behavior/race regressions for durable memory integration."""
from __future__ import annotations
import json
import sys
import tempfile
import threading
import unittest
from datetime import timedelta
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import scripts.check_memory_durable_integration as helpers
from scripts.check_memory_storage_transactions import records
from contracts.geometry import Pose
from contracts.spatial_memory import InstanceDynamicState,InstanceLifecycleState
from services.memory.durable_index import TOKEN,INDEX_META
from services.memory.repository import MemoryRepository
from services.memory.vector_store import SQLiteNamedVectorStore,NamedVector
from services.memory.vectorization_outbox import OutboxConflict
from services.memory.vectorizer import TextEmbedder

class ExplicitFixtureEmbedder(TextEmbedder):
    """固定测试向量，不计算内容指纹，也不冒充历史向量空间。"""
    model_name='explicit-test-v1'
    model_revision='fixture-1'
    dimension=16
    def embed_text(self,text):
        vector=np.zeros(self.dimension,dtype=np.float32)
        if text.strip():vector[0]=1.0
        return vector

class ServiceRegressionChecks(unittest.TestCase):
    def setUp(self):
        self.thread_errors=[]
        hook=patch('threading.excepthook',side_effect=lambda event:self.thread_errors.append(event.exc_value))
        hook.start()
        self.addCleanup(lambda:self.assertFalse(self.thread_errors,repr(self.thread_errors)))
        self.addCleanup(hook.stop)

    fixture=helpers.IntegrationChecks.fixture
    memory=helpers.IntegrationChecks.memory
    accept=helpers.IntegrationChecks.accept
    claim=helpers.IntegrationChecks.claim
    query=helpers.IntegrationChecks.query

    def test_artifact_cleanup_failure_reports_committed_record_deletion(self):
        from core.errors import GatewayError
        service = self.fixture()
        artifact_id = "art_image_20261022T000000Z_deadbeef"
        record, _ = self.accept(service, self.memory(artifact_ids=[artifact_id]))
        def fail_delete(_artifact_id):
            raise PermissionError("测试制品目录不可写")
        with patch.object(service, "_artifact_store", SimpleNamespace(delete_artifact=fail_delete)):
            with self.assertRaises(GatewayError) as caught:
                service._delete_memory_library_records("library")
        error = caught.exception
        self.assertEqual(error.error_code, "memory_artifact_cleanup_incomplete")
        self.assertTrue(error.details["records_deleted"])
        self.assertEqual(error.details["deleted_counts"]["semantic_memory_count"], 1)
        self.assertEqual(error.details["deleted_counts"]["artifact_count"], 0)
        self.assertEqual(error.details["artifact_cleanup_errors"],
                         [{"artifact_id": artifact_id, "reason": "测试制品目录不可写"}])
        self.assertEqual(service._repository.load_semantic_memories(), [])

    def test_terminal_retry_budget_survives_restart_and_lazy_initialization(self):
        from services.memory.vectorization_outbox import RetryPolicy
        with tempfile.TemporaryDirectory() as tmp:
            path=str(Path(tmp)/'source.db')
            failing=helpers.FakeEmbedder();failing.fail=True
            first=self.fixture(path=path,embedder=failing)
            box=first._durable_index.outbox;box.retry_policy=RetryPolicy(max_attempts=1)
            record,receipt=self.accept(first);first._run_durable_vectorization(self.claim(first))
            failed=box.inspect(receipt.version.key)
            self.assertEqual(failed['status'],'failed_terminal');self.assertTrue(first.close())
            # Use the actual non-injected constructor with lazy provider setup.
            second=helpers.MemoryService(localization_service=None,mapping_service=None,
                observation_service=None,perception_service=None,state_store=SimpleNamespace(write=lambda *a,**kw:None),
                memory_db_path=path,text_embedding_model=failing.model_name,text_embedding_dimension=16)
            self.addCleanup(second.close)
            second._memory_enabled=True;second._active_library_name='library';second._durable_index.activate('library')
            self.assertIsNone(second._text_embedder);self.assertFalse(second._text_embedder_ready)
            loaded=second._repository.load_semantic_memories()[0]
            second._durable_index.ensure('memory',loaded)
            self.assertEqual(second._durable_index.outbox.current(receipt.version.key),receipt)
            with patch.object(second,'_build_text_embedder_with_fallback',return_value=helpers.FakeEmbedder()):
                second._ensure_embedders_initialized()
            second._durable_index.ensure('memory',loaded)
            after=second._durable_index.outbox.inspect(receipt.version.key)
            for key in ('status','attempts','next_attempt_at','receipt','source_revision','max_attempts'):
                self.assertEqual(after[key],failed[key],key)
            self.assertEqual(second._durable_index.outbox.snapshot()['accepted'],1)
            # A genuine in-process provider replacement remains new work.
            second._text_embedder=helpers.FakeEmbedder()
            second._durable_index.ensure('memory',loaded)
            self.assertEqual(second._durable_index.outbox.snapshot()['accepted'],2)
            self.assertEqual(second._durable_index.outbox.inspect(receipt.version.key)['attempts'],0)

    def test_declared_revision_change_survives_lazy_restart_gap(self):
        from services.memory.vectorization_outbox import RetryPolicy
        with tempfile.TemporaryDirectory() as tmp:
            path=str(Path(tmp)/'source.db');model=helpers.FakeEmbedder()
            model.fail=True;model.model_revision='revision-1'
            first=self.fixture(path=path,embedder=model);first._durable_index.outbox.retry_policy=RetryPolicy(max_attempts=1)
            record,receipt=self.accept(first);first._run_durable_vectorization(self.claim(first));first.close()
            second=self.fixture(path=path);second._text_embedder=None;second._text_embedder_ready=False
            loaded=second._repository.load_semantic_memories()[0]
            second._durable_index.ensure('memory',loaded)
            self.assertEqual(second._durable_index.outbox.current(receipt.version.key),receipt)
            revised=helpers.FakeEmbedder();revised.model_revision='revision-2'
            with patch.object(second,'_build_text_embedder_with_fallback',return_value=revised):
                second._ensure_embedders_initialized()
            second._durable_index.ensure('memory',loaded)
            after=second._durable_index.outbox.inspect(receipt.version.key)
            self.assertEqual(after['status'],'pending');self.assertEqual(after['attempts'],0)
            self.assertEqual(second._durable_index.outbox.snapshot()['accepted'],2)

    def test_partial_backoff_restart_retains_budget_but_revalidates_actual_binding(self):
        from services.memory.vectorization_outbox import RetryPolicy
        with tempfile.TemporaryDirectory() as tmp:
            path=str(Path(tmp)/'source.db');first=self.fixture(path=path)
            first._configured_image_model='synthetic-image'
            box=first._durable_index.outbox;box.retry_policy=RetryPolicy(base_delay=60,max_delay=60)
            record,receipt=self.accept(first,self.memory(artifact_ids=['missing-image']))
            first._run_durable_vectorization(self.claim(first));before=box.inspect(receipt.version.key)
            self.assertEqual(before['status'],'backoff');self.assertEqual(first._text_embedder.calls,1)
            self.assertTrue(first.close())
            second=self.fixture(path=path);second._configured_image_model='synthetic-image'
            second._load_persisted_memory(library_name='library')
            index=second._durable_index;loaded=second._repository.load_semantic_memories()[0]
            index.ensure('memory',loaded)
            after=index.outbox.inspect(receipt.version.key)
            for key in ('status','attempts','next_attempt_at','receipt','source_revision'):
                self.assertEqual(after[key],before[key],key)
            packets,_=index.query_snapshot([('memory',loaded)])
            self.assertNotIn('text_dense',packets[0][2].vectors)
            index.outbox._clock=lambda:float(before['next_attempt_at'])+1
            token=index.outbox.start_worker(recover=True)
            claim=index.outbox.claim_next(token);self.assertEqual(claim.attempt,2)
            self.assertFalse(index.current_vectors(claim))
            second._run_durable_vectorization(claim)
            self.assertEqual(second._text_embedder.calls,1)
            self.assertEqual(index.outbox.current(receipt.version.key),receipt)
            packets,_=index.query_snapshot([('memory',loaded)])
            self.assertIn('text_dense',packets[0][2].vectors)
            self.assertEqual(index.outbox.inspect(receipt.version.key)['attempts'],2)
            second._repository.upsert_semantic_memory(loaded.model_copy(update={'title':'Changed input'}))
            self.assertEqual(index.outbox.snapshot()['accepted'],2)

    def test_recovered_claim_refresh_does_not_complete_without_inference(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=str(Path(tmp)/'source.db');first=self.fixture(path=path)
            record,receipt=self.accept(first);self.assertTrue(first.close())
            second=self.fixture(path=path)
            # Claim before reconciliation exercises the runtime-binding mismatch
            # branch, which must refresh and actually infer the preserved claim.
            claim=self.claim(second);second._run_durable_vectorization(claim)
            self.assertEqual(second._text_embedder.calls,1)
            self.assertEqual(second._durable_index.outbox.current(receipt.version.key),receipt)
            self.assertEqual(second._durable_index.outbox.inspect(receipt.version.key)['status'],'completed_fresh')

    def instance(self,*,dynamic=False):
        return records()[2][3].model_copy(update={'dynamic_state':InstanceDynamicState.DYNAMIC if dynamic else InstanceDynamicState.STATIC})

    def stub_ingest(self,service,instance,associate=None):
        service._semantic_instances_by_id[instance.instance_id]=instance
        service._resolve_stable_map_name=lambda **kw:'library'
        service._refresh_semantic_map_context=lambda *a,**kw:None
        service._vision_projection_service.project=lambda *a,**kw:[object()]
        service._build_platform_semantic_metadata=lambda **kw:{}
        service._evaluate_navigation_metadata=lambda *a,**kw:{}
        if associate is None:
            def associate(*a,**kw):
                old=kw['existing_instances'][instance.instance_id]
                updated=old.model_copy(update={'observation_count':old.observation_count+1})
                return {instance.instance_id:updated},(SimpleNamespace(instance=updated),)
        service._instance_association_service.associate=associate
        return lambda persist=False:service.ingest_perception_observation(
            perception_context=object(),map_snapshot=None,localization_snapshot=None,
            current_pose=Pose(frame_id='map'),persist=persist)

    def test_association_block_does_not_hold_structural_lock_and_rebind_serializes(self):
        model=helpers.FakeEmbedder();model.release=threading.Event()
        service=self.fixture(embedder=model);instance=self.instance()
        def associate(*a,**kw):
            service._instance_association_service._label_similarity('desk','table')
            return {instance.instance_id:instance},(SimpleNamespace(instance=instance),)
        ingest=self.stub_ingest(service,instance,associate)
        errors=[]
        def run():
            try:ingest(False)
            except BaseException as exc:errors.append(exc)
        worker=threading.Thread(target=run);worker.start();self.assertTrue(model.entered.wait(2))
        self.assertTrue(service._instances_lock.acquire(blocking=False));service._instances_lock.release()
        replacement=helpers.FakeEmbedder();service._text_embedder=replacement
        rebinder=threading.Thread(target=service._ensure_embedders_initialized);rebinder.start()
        # Managed source paths take only the enabled/scope check while structural lock is held.
        with service._instances_lock:
            self.assertEqual(service._require_memory_enabled(initialize_embedders=False),'library')
        model.release.set();worker.join(2);rebinder.join(2)
        self.assertFalse(worker.is_alive());self.assertFalse(rebinder.is_alive());self.assertFalse(errors)
        self.assertIs(service._instance_association_service._text_embedder,replacement)
        self.assertFalse(service._instance_association_service._embedding_cache)
        self.assertEqual(service._durable_index.outbox.snapshot()['accepted'],0)

    def test_persist_false_rechecks_live_identity_and_stays_job_free(self):
        service=self.fixture();instance=self.instance();ingest=self.stub_ingest(service,instance)
        original=service._assert_observation_library;count=[0]
        newer=instance.model_copy(update={'confidence':.987})
        def interleave(*a,**kw):
            count[0]+=1
            if count[0]==2:service._semantic_instances_by_id[instance.instance_id]=newer
            return original(*a,**kw)
        with patch.object(service,'_assert_observation_library',side_effect=interleave):ingest(False)
        self.assertIs(service._semantic_instances_by_id[instance.instance_id],newer)
        self.assertFalse(service._repository.load_semantic_instances())
        self.assertEqual(service._durable_index.outbox.snapshot()['accepted'],0)

    def test_two_first_persistence_snapshots_cannot_borrow_newer_token(self):
        service=self.fixture();instance=self.instance();ingest=self.stub_ingest(service,instance)
        original=service._repository.upsert_semantic_instance;barrier=threading.Barrier(2);errors=[]
        def delayed(record):
            self.assertEqual(getattr(record,TOKEN),('raw',None))
            barrier.wait(2)
            return original(record)
        def run():
            try:ingest(True)
            except BaseException as exc:errors.append(exc)
        with patch.object(service._repository,'upsert_semantic_instance',side_effect=delayed):
            workers=[threading.Thread(target=run) for _ in range(2)]
            for worker in workers:worker.start()
            for worker in workers:worker.join(3)
        self.assertEqual(len(errors),1);self.assertIsInstance(errors[0],OutboxConflict)
        self.assertEqual(service._durable_index.outbox.snapshot()['accepted'],1)
        self.assertEqual(len(service._repository.load_semantic_instances()),1)

    def test_unmatched_instance_aging_is_retained_without_jobs(self):
        service=self.fixture();instance=self.instance()
        aged=instance.model_copy(update={'lifecycle_state':InstanceLifecycleState.STALE})
        ingest=self.stub_ingest(service,instance,lambda *a,**kw:({instance.instance_id:aged},()))
        ingest(False)
        self.assertEqual(service._semantic_instances_by_id[instance.instance_id].lifecycle_state,InstanceLifecycleState.STALE)
        self.assertEqual(service._durable_index.outbox.snapshot()['accepted'],0)

    def test_dynamic_graph_metadata_and_refinement_remain_live_only(self):
        service=self.fixture();instance=self.instance(dynamic=True)
        service._semantic_instances_by_id[instance.instance_id]=instance
        service._attach_graph_relation_to_node('semantic_instance',instance.instance_id,'synthetic_relation')
        current=service._semantic_instances_by_id[instance.instance_id]
        self.assertIn('synthetic_relation',current.graph_relation_ids)
        service._evaluate_navigation_metadata=lambda *a,**kw:{}
        result=SimpleNamespace(centroid=(3.,4.,0.),point_count=20,mean_range_m=1.5)
        service._apply_deferred_refine_success(instance.instance_id,result=result,expected_instance=current)
        self.assertEqual(service._semantic_instances_by_id[instance.instance_id].pose.position.x,3.)
        self.assertFalse(service._repository.load_semantic_instances())
        self.assertEqual(service._durable_index.outbox.snapshot()['accepted'],0)

    def test_dynamic_supporting_event_attachment_does_not_drop_instance(self):
        service=self.fixture();instance=self.instance(dynamic=True)
        service._semantic_instances_by_id[instance.instance_id]=instance
        event=records()[3][3].model_copy(update={'linked_instance_ids':[instance.instance_id]})
        service.ingest_perception_observation=lambda **kw:([object()],[],[instance.instance_id])
        service._build_projected_observation_events=lambda **kw:[event]
        service._record_observation_graph_relations=lambda **kw:None
        service._refresh_scene_spaces_from_observations=lambda **kw:None
        service._ingest_spatial_semantic_observation(memory_entry=self.memory(),perception_context=None,
            observation_context=None,map_snapshot=None,localization_snapshot=None,current_pose=None,camera_id=None)
        current=service._semantic_instances_by_id[instance.instance_id]
        self.assertIn(event.event_id,current.supporting_observation_event_ids)
        self.assertFalse(service._repository.load_semantic_instances())
        self.assertEqual(service._durable_index.outbox.snapshot()['accepted'],1) # event only

    def test_scene_delete_preserves_same_id_recreated_in_other_library(self):
        service=self.fixture();scene=records()[4][3];service._repository.upsert_scene_space(scene)
        original=service._repository.delete_scene_spaces
        def interleave(ids,**kw):
            original(ids)
            service._repository.upsert_scene_space(type(scene).model_validate({**scene.model_dump(mode='json'),'metadata':{'memory_library_name':'other'}}))
            return original(ids,**kw)
        with patch.object(service._repository,'delete_scene_spaces',side_effect=interleave):
            with self.assertRaises(OutboxConflict):service.delete_scene_spaces([scene.scene_space_id])
        self.assertEqual(service._repository.load_scene_space(scene.scene_space_id).metadata['memory_library_name'],'other')

    def test_ttl_delete_does_not_remove_concurrent_recreated_cache(self):
        service=self.fixture();event=records()[3][3];service._repository.upsert_observation_event(event)
        original=service._repository.delete_observation_events
        newer=type(event).model_validate({**event.model_dump(mode='json'),'title':'Recreated'})
        def interleave(ids,**kw):
            result=original(ids,**kw)
            service._repository.upsert_observation_event(newer)
            return result
        with patch.object(service._repository,'delete_observation_events',side_effect=interleave):
            service._cleanup_expired_observation_events(max_age_sec=0)
        self.assertEqual(service._observation_events_by_id[event.event_id].title,'Recreated')

    def test_terminal_close_fences_prepared_acceptance(self):
        service=self.fixture();index=service._durable_index
        prepared=index.prepare('memory',self.memory())
        index.closed=True
        with patch.object(index,'prepare',return_value=prepared):
            with self.assertRaises(RuntimeError):service._repository.upsert_semantic_memory(self.memory())
        self.assertEqual(index.outbox.snapshot()['accepted'],0)

    def test_shared_source_delete_removes_legacy_archive(self):
        with tempfile.TemporaryDirectory() as tmp:
            service=self.fixture(path=str(Path(tmp)/'source.db'))
            store=service._vector_store
            record=self.memory();payload=service._build_memory_payload(record)
            named=NamedVector('text_dense',service._text_embedder.model_name,16,np.ones(16,dtype=np.float32))
            store.upsert_point(point_id=record.memory_id,collection_name='episodic_observations',record_kind=record.kind.value,payload=payload,vectors={'text_dense':named})
            self.accept(service,record);service._run_durable_vectorization(self.claim(service))
            with store.storage.read_snapshot() as tx:self.assertEqual(tx.execute('SELECT count(*) FROM memory_index_legacy_archive').fetchone()[0],1)
            service._repository.delete_semantic_memories([record.memory_id])
            with store.storage.read_snapshot() as tx:self.assertEqual(tx.execute('SELECT count(*) FROM memory_index_legacy_archive').fetchone()[0],0)
            self.assertIsNone(store.get_point(record.memory_id))


    def test_crash_after_partial_commit_recovers_without_reencoding_text(self):
        service=self.fixture();service._image_embedder=service._text_embedder
        service._image_embedder.embed_image_bytes=lambda data:np.ones(16,dtype=np.float32)
        service._load_index_image=lambda ids:(b'x','digest',None)
        record,receipt=self.accept(service,self.memory(artifact_ids=['art_image_20260101T000000Z_00000001']))
        index=service._durable_index;token=index.outbox.start_worker();claim=index.outbox.claim_next(token)
        original=index.publish;cut=[False]
        class Crash(BaseException):pass
        def cut_after_text(*a,**kw):
            result=original(*a,**kw)
            if not cut[0]:cut[0]=True;raise Crash()
            return result
        with patch.object(index,'publish',side_effect=cut_after_text):
            with self.assertRaises(Crash):service._run_durable_vectorization(claim)
        self.assertEqual(index.query_snapshot([('memory',record)])[1]['validated_current'],1)
        # Physical fake call has ended before explicit ownership recovery.
        replacement=index.outbox.start_worker(recover=True)
        with index.storage.transaction() as tx:tx.execute("UPDATE memory_vectorization_state SET next_attempt_at=0 WHERE status='backoff'")
        service._run_durable_vectorization(index.outbox.claim_next(replacement))
        self.assertEqual(service._text_embedder.calls,1)
        self.assertEqual(index.outbox.receipt_outcome(receipt.token),'completed_fresh')

    def test_default_service_preserves_original_legacy_payload_and_map_fallback(self):
        import json
        from contracts.memory import MemoryPayloadFilter,MemoryRecordKind
        for library in ('library','legacy_default'):
            with self.subTest(library=library),tempfile.TemporaryDirectory() as tmp:
                path=str(Path(tmp)/'memory.db')
                repo=MemoryRepository(path)
                record=self.memory(map_name=None,metadata={'memory_library_name':library})
                repo.upsert_semantic_memory(record)
                store=SQLiteNamedVectorStore(path,preserve_legacy=True)
                payload={'record_id':record.memory_id,'record_kind':'scene','retrieval_text':'Desk Synthetic','map_name':None,
                         'title':'Desk','summary':'Synthetic','timestamp':record.timestamp.isoformat()}
                named=NamedVector('text_dense','fixture-constant-v1',16,np.ones(16,dtype=np.float32))
                store.upsert_point(point_id=record.memory_id,collection_name='episodic_observations',record_kind='scene',payload=payload,vectors={'text_dense':named})
                original=json.dumps(payload,ensure_ascii=False,indent=3)
                with store.storage.transaction() as tx:
                    tx.execute('UPDATE memory_vector_points SET payload_json=?,map_name=NULL',(original,))
                store.close();repo.close()
                service=self.fixture(path=path)
                service._active_library_name=library;service._durable_index.activate(library)
                service._load_persisted_memory(library_name=library)
                self.assertEqual(service._text_embedder.calls,0)
                with service._repository.storage.read_snapshot() as tx:
                    self.assertEqual(tx.execute('SELECT payload_json FROM memory_vector_points').fetchone()[0],original)
                result=service._query_records('Desk',allowed_record_kinds=(MemoryRecordKind.SCENE,),similarity_threshold=.1,
                    limit=5,payload_filter=MemoryPayloadFilter(map_name='library') if library=='library' else None)
                self.assertTrue(result.matches);self.assertTrue(result.metadata['includes_unverified_legacy_scores'])
                current=service._repository.load_semantic_memories(library_name=library)[0]
                service._repository.upsert_semantic_memory(current.model_copy(update={'observation_count':1}))
                with service._repository.storage.read_snapshot() as tx:
                    archived=tx.execute('SELECT point_payload_json,point_row_json,vector_payload FROM memory_index_legacy_archive').fetchone()
                    self.assertEqual(archived[0],original);self.assertEqual(bytes(archived[2]),named.vector.tobytes())
                    self.assertEqual(json.loads(archived[1])['payload_json'],original)

    def test_pending_source_recovers_after_reopen_without_sync_model_work(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=str(Path(tmp)/'memory.db')
            first=self.fixture(path=path);record,receipt=self.accept(first)
            first.close()
            reopened=self.fixture(path=path);reopened._load_persisted_memory(library_name='library')
            self.assertEqual(reopened._text_embedder.calls,0)
            index=reopened._durable_index;completed=threading.Event();actual=index.outbox.complete
            def complete(*a,**kw):
                result=actual(*a,**kw)
                if result.published:completed.set()
                return result
            with patch.object(index.outbox,'complete',side_effect=complete):
                reopened._start_vectorization_worker();self.assertTrue(completed.wait(3))
                worker=reopened._vectorization_worker
                reopened._start_vectorization_worker();self.assertIs(reopened._vectorization_worker,worker)
                self.assertTrue(self.query(reopened).matches)
                self.assertTrue(reopened.close())
            self.assertIsNotNone(index)

    def runtime_stub(self, started, close_results):
        from contextlib import ExitStack
        from tests.conftest import create_isolated_gateway
        temporary = tempfile.TemporaryDirectory(prefix="memory-runtime-regression-")
        self.addCleanup(temporary.cleanup)
        runtime = create_isolated_gateway(Path(temporary.name))
        self.addCleanup(runtime.stop)
        overrides = ExitStack()
        self.addCleanup(overrides.close)
        order, events = [], []
        runtime._started = started
        for owner, method, name in ((runtime.task_manager, "shutdown", "tasks"),
                                    (runtime.platform_geometry_runtime, "stop", "platform_geometry_runtime")):
            original = getattr(owner, method)
            def invoke(*args, _original=original, _name=name, **kwargs):
                order.append(_name)
                return _original(*args, **kwargs)
            overrides.enter_context(patch.object(owner, method, invoke))
        original_close = runtime.memory_service.close
        def close(*args, **kwargs):
            order.append("memory")
            if close_results and not close_results.pop(0):
                return False
            return original_close(*args, **kwargs)
        overrides.enter_context(patch.object(runtime.memory_service, "close", close))
        original_publish = runtime.event_bus.publish
        def publish(event, *args, **kwargs):
            events.append(event.model_dump(mode="python"))
            return original_publish(event, *args, **kwargs)
        overrides.enter_context(patch.object(runtime.event_bus, "publish", publish))
        return runtime, order, events

    def test_runtime_normal_and_never_started_stop_close_memory(self):
        for started in (False,True):
            runtime,order,events=self.runtime_stub(started,[True])
            runtime.stop()
            self.assertIn('memory',order);self.assertTrue(runtime._memory_shutdown_complete)
            if started:
                self.assertLess(order.index('tasks'),order.index('platform_geometry_runtime'))
                self.assertTrue(events[-1]['payload']['success'])

    def test_runtime_incomplete_memory_close_is_reported_and_retryable(self):
        for started in (False,True):
            runtime,order,events=self.runtime_stub(started,[False,True])
            with self.assertRaisesRegex(RuntimeError,'Memory shutdown incomplete'):runtime.stop()
            self.assertFalse(runtime._memory_shutdown_complete)
            if started:self.assertFalse(events[-1]['payload']['success'])
            runtime.stop();self.assertTrue(runtime._memory_shutdown_complete)
            self.assertEqual(order.count('memory'),2)

    def test_scope_change_rejects_old_geometry_context_publication(self):
        from services.memory.semantic_map_builder import SemanticMapBuildResult
        from core.errors import GatewayError
        service=self.fixture();entered=threading.Event();release=threading.Event();errors=[]
        service._observation_localization_session=lambda:None
        service._resolve_stable_map_name=lambda **kw:kw.get('map_name') or 'library'
        from contracts.runtime_views import MapSnapshot
        snapshot=MapSnapshot(map_name='library',source_name='synthetic',source_epoch='e',update_sequence=1,frame_id='map')
        service._mapping_service._latest_snapshot = snapshot
        def build(*a,**kw):
            entered.set();assert release.wait(3)
            return SemanticMapBuildResult(map_name='library',frame_id='map',semantic_regions=(),anchors_by_id={},topology_nodes_by_id={})
        service._semantic_map_builder.build=build
        def refresh():
            try:service._refresh_semantic_map_context(snapshot,map_name='library')
            except BaseException as exc:errors.append(exc)
        thread=threading.Thread(target=refresh);thread.start();self.assertTrue(entered.wait(2), repr(errors))
        service._mapping_service._latest_snapshot = snapshot.model_copy(update={'map_name':'other','source_epoch':'other'})
        with patch.object(service,'_start_vectorization_worker'),patch.object(service,'_start_deferred_refine_worker'):
            service.activate_memory_library(library_name='other',load_history=False)
        release.set();thread.join(2)
        self.assertEqual(len(errors),1);self.assertIsInstance(errors[0],GatewayError)
        self.assertNotEqual(service._latest_semantic_map_context.map_name,'library')
        self.assertEqual(service._durable_index.outbox.snapshot()['accepted'],0)

    def test_same_database_owner_cannot_steal_blocked_physical_worker(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = str(Path(temporary) / "memory.db")
            model = helpers.FakeEmbedder()
            model.release = threading.Event()
            self.addCleanup(model.release.set)
            first = self.fixture(path=path, embedder=model)
            self.accept(first)
            first._start_vectorization_worker()
            self.assertTrue(model.entered.wait(2))
            second = self.fixture(path=path)
            with self.assertRaises(OutboxConflict):
                second._start_vectorization_worker()
            self.assertEqual(second._text_embedder.calls, 0)
            self.assertTrue(first._vectorization_worker.is_alive())
            model.release.set()
            self.assertTrue(first.close())
            self.assertTrue(second.close())

    def test_managed_scene_acceptance_does_not_initialize_models_under_lock(self):
        service=self.fixture();scene=records()[4][3].model_copy(update={'map_name':'library'})
        with patch.object(service,'_ensure_embedders_initialized',side_effect=AssertionError('model init under lock')):
            with service._instances_lock:
                saved=service._persist_managed_scene_space(scene,expected_revision=0)
        self.assertEqual(saved.metadata['scene_revision'],1)
        self.assertEqual(service._durable_index.outbox.snapshot()['accepted'],1)
        self.assertEqual(service._text_embedder.calls,0)

    def test_busy_document_model_keeps_query_listing_and_metadata_available(self):
        model=helpers.FakeEmbedder();model.release=threading.Event()
        service=self.fixture(embedder=model);record,receipt=self.accept(service);claim=self.claim(service)
        document=threading.Thread(target=service._run_durable_vectorization,args=(claim,));document.start()
        self.assertTrue(model.entered.wait(2));done=threading.Event();results=[];errors=[]
        def foreground():
            try:
                service._repository.upsert_semantic_memory(record.model_copy(update={'observation_count':99}))
                self.assertTrue(service.list_semantic_memories(library_name='library'))
                self.assertEqual(service.get_summary().semantic_memory_count,1)
                results.append(self.query(service))
            except BaseException as error:errors.append(error)
            finally:done.set()
        reader=threading.Thread(target=foreground);reader.start()
        try:
            self.assertTrue(done.wait(2),'foreground waited behind blocked background inference')
            self.assertTrue(document.is_alive());self.assertFalse(errors)
            self.assertTrue(results[0].matches);self.assertTrue(results[0].metadata['fallback_used'])
            self.assertEqual(results[0].metadata['query_embedding_errors']['text_dense'],'model_busy')
            self.assertEqual(results[0].matches[0].semantic_memory.observation_count,99)
            self.assertEqual(service._durable_index.outbox.current(receipt.version.key),receipt)
        finally:model.release.set();document.join(3);reader.join(3)

    def test_independent_image_owner_does_not_block_text_query(self):
        for shared in (False,True):
            with self.subTest(shared_handle=shared):
                service=self.fixture();image_model=service._text_embedder if shared else helpers.FakeEmbedder()
                service._image_embedder=image_model
                entered=threading.Event();release=threading.Event();done=threading.Event();results=[];errors=[]
                def encode_image(data):
                    entered.set();assert release.wait(4)
                    return np.ones(16,dtype=np.float32)
                image_model.embed_image_bytes=encode_image
                service._load_index_image=lambda ids:(b'x','digest',None)
                self.accept(service,self.memory(artifact_ids=['art_image_20260101T000000Z_00000001']))
                claim=self.claim(service)
                document=threading.Thread(target=service._run_durable_vectorization,args=(claim,));document.start()
                self.assertTrue(entered.wait(2))
                def query():
                    try:results.append(self.query(service))
                    except BaseException as error:errors.append(error)
                    finally:done.set()
                foreground=threading.Thread(target=query);foreground.start()
                try:
                    self.assertTrue(done.wait(2));self.assertFalse(errors);self.assertTrue(document.is_alive())
                    self.assertTrue(results[0].matches)
                    self.assertEqual(results[0].metadata['query_embedding_errors']['image_dense'],'model_busy')
                    contributions=results[0].matches[0].metadata['query_vector_contributions']
                    self.assertEqual('text_dense' in contributions,not shared)
                finally:release.set();document.join(3);foreground.join(3)

    def test_corrupt_legacy_modality_is_suppressed_without_hiding_other_evidence(self):
        for broken in (b'x','not-json','[NaN]'):
            with self.subTest(payload=broken):
                service=self.fixture();service._image_embedder=service._text_embedder
                record,_=self.accept(service,self.memory(artifact_ids=['art_image_20260101T000000Z_00000001']))
                payload=service._build_memory_payload(record)
                vector=np.ones(16,dtype=np.float32)
                service._vector_store.upsert_point(point_id=record.memory_id,collection_name='episodic_observations',record_kind='scene',payload=payload,
                    vectors={name:NamedVector(name,service._text_embedder.model_name,16,vector) for name in ('text_dense','image_dense')})
                with service._repository.storage.transaction() as tx:
                    tx.execute("UPDATE memory_named_vectors SET vector_payload=? WHERE vector_name='text_dense'",(broken,))
                result=self.query(service);self.assertTrue(result.matches)
                evidence=result.matches[0].metadata['vector_provenance']
                self.assertEqual(evidence['text_dense']['reason'],'invalid_vector_bytes')
                self.assertEqual(evidence['image_dense']['status'],'legacy_unverified')
                self.assertEqual(service.get_summary().semantic_memory_count,1)
                service._load_index_image=lambda ids:(None,None,'image_missing')
                service._run_durable_vectorization(self.claim(service))
                with service._repository.storage.read_snapshot() as tx:
                    archived=tx.execute("SELECT vector_payload FROM memory_index_legacy_archive WHERE vector_name='text_dense'").fetchone()[0]
                    self.assertEqual(archived,broken)
                self.assertEqual(self.query(service).matches[0].metadata['vector_provenance']['text_dense']['status'],'validated_current')


    def test_deleted_dynamic_then_static_live_only_stays_lexical_and_can_repersist(self):
        service=self.fixture();static=self.instance();service._repository.upsert_semantic_instance(static)
        dynamic=static.model_copy(update={'dynamic_state':InstanceDynamicState.DYNAMIC})
        self.assertTrue(service._publish_live_instance(dynamic,expected=service._semantic_instances_by_id[static.instance_id]))
        service._repository.delete_semantic_instances([static.instance_id])
        revived=dynamic.model_copy(update={'dynamic_state':InstanceDynamicState.STATIC})
        self.assertTrue(service._publish_live_instance(revived,expected=dynamic))
        from contracts.memory import MemoryRecordKind
        result=service._query_records('desk',allowed_record_kinds=(MemoryRecordKind.OBJECT_INSTANCE,),similarity_threshold=.4,limit=5,payload_filter=None)
        self.assertTrue(result.matches);self.assertFalse(result.matches[0].metadata['query_vector_contributions'])
        ingest=self.stub_ingest(service,revived);ingest(True)
        persisted=service._repository.load_semantic_instances()[0]
        self.assertNotEqual(getattr(persisted,TOKEN)[1],getattr(static,TOKEN)[1])

    def test_query_local_lexical_cache_is_bounded_exact_and_discarded(self):
        service=self.fixture();texts=['Desk','Table','Semantic gallery item','Desk']
        expected=service._score_text_candidates('desk',texts,fallback_reason='test')
        service._query_score_state.cache={}
        try:
            self.assertEqual(service._score_text_candidates('desk',texts,fallback_reason='test'),expected)
            for number in range(3000):service._score_text_candidates('desk',['unique %d'%number],fallback_reason='test')
            self.assertLessEqual(len(service._query_score_state.cache),2048)
        finally:del service._query_score_state.cache
        self.accept(service);self.query(service)
        self.assertFalse(hasattr(service._query_score_state,'cache'))
        self.assertFalse(hasattr(service._query_score_state,'fuzzy_cache'))
        self.assertFalse(hasattr(service._query_score_state,'lexical_query'))

    def test_fuzzy_signatures_match_original_formula_and_preserve_token_identity(self):
        import random
        from difflib import SequenceMatcher
        service=self.fixture();randomizer=random.Random(3101)
        cases=[('desk',['Desk','table','desk 11','desk 99','']),
               ('',['','desk']),('  !?  ',['','!?','desk']),
               ('a b',['b a','a b']),('abc',['abcd','acbd','abcxxxxxxxxxx']),
               ('  CAFÉ\x00桌子  ',['café桌子','ＣＡＦÉ 桌子','cafe\u0301桌子','CAFÉ\x00桌子']),
               ('a b c d e',['a b c x x','a b c y z']),
               ('tide',['diet']),('diet',['tide']),
               ('前方红色桌子',['前方蓝色桌子123','前方绿色桌子789']),
               ('a'*205+'b',['a'*205+'c','a'*205+'d','a'*199+'b','a'*200+'b']),
               ('x y',['','  !?  ','x' * 201,'y' * 400])]
        for length in (199,200,201):
            left='b'+'a'*(length-1);right='a'*(length-1)+'b'
            cases.extend([(left,[right,'b'+'a'*(length-2)+'c']),
                          (right,[left,'c'+'a'*(length-1)])])
        alphabet='abcdefghi中女美98312 '
        for _ in range(800):
            query=''.join(randomizer.choices('abcde中女12 ',k=randomizer.randrange(1,60)))
            texts=[''.join(randomizer.choices(alphabet,k=randomizer.choice([5,20,100,199,200,201,400]))) for _ in range(3)]
            # Distinct actual strings sharing a signature exercise cache hits.
            texts += [text.translate(str.maketrans({'f':'g','h':'i','美':'女' if '女' not in query else '美','9':'8'})) for text in texts]
            cases.append((query,texts))
        def original(query,texts):
            if not query:return 0.0,'test'
            best=0.0
            for text in texts:
                text=service._normalize_text(text)
                if not text:continue
                if query==text:return .99,'文本完全匹配。'
                overlap=service._token_overlap_score(service._tokenize(query),service._tokenize(text))
                fuzzy=SequenceMatcher(None,query,text).ratio()
                best=max(best,fuzzy,overlap,.6*fuzzy+.4*overlap)
            return best,'test'
        def score(*args,**kwargs):
            for mode in ('disabled','score-only','query-local'):
                state=SimpleNamespace()
                if mode!='disabled':state.cache={}
                if mode=='query-local':state.fuzzy_cache={}
                with patch.object(service,'_query_score_state',state):
                    for number,(raw,texts) in enumerate(cases):
                        for name in ('cache','fuzzy_cache'):
                            if hasattr(state,name):getattr(state,name).clear()
                        query=service._normalize_text(raw)
                        with self.subTest(mode=mode,case=number):
                            # Compare the cold list before singleton calls warm either cache.
                            self.assertEqual(service._score_text_candidates(query,texts,fallback_reason='test'),original(query,texts))
                            for text in texts:
                                self.assertEqual(service._score_text_candidates(query,[text],fallback_reason='test'),original(query,[text]))
                            self.assertEqual(service._score_text_candidates(query,texts,fallback_reason='test'),original(query,texts))
                            self.assertEqual(service._score_text_candidates(query,list(reversed(texts)),fallback_reason='test'),original(query,list(reversed(texts))))
                    for name in ('cache','fuzzy_cache'):
                        self.assertLessEqual(len(getattr(state,name,{})),2048)
        with patch.object(service,'_query_records_impl',side_effect=score):service._query_records('test')

    def test_fuzzy_scoring_still_checks_later_exact_match_and_query_identity(self):
        from difflib import SequenceMatcher
        service=self.fixture();states=[]
        def score(*args,**kwargs):
            state=service._query_score_state
            self.assertEqual(state.cache,{});self.assertEqual(state.fuzzy_cache,{})
            states.append((state.cache,state.fuzzy_cache))
            self.assertEqual(service._score_text_candidates('a b',['b a'],fallback_reason='overlap'),(1.0,'overlap'))
            state.cache.clear();state.fuzzy_cache.clear()
            self.assertEqual(service._score_text_candidates('a b',['b a','a b'],fallback_reason='test'),(.99,'文本完全匹配。'))
            for query,text in (('tide','diet'),('diet','tide')):
                self.assertEqual(service._score_text_candidates(query,[text],fallback_reason=query),(SequenceMatcher(None,query,text).ratio(),query))
        with patch.object(service,'_query_records_impl',side_effect=score):
            service._query_records('first');service._query_records('second')
        self.assertIsNot(states[0][0],states[1][0]);self.assertIsNot(states[0][1],states[1][1])
        for name in ('cache','fuzzy_cache','lexical_query'):
            self.assertFalse(hasattr(service._query_score_state,name))

    def test_query_local_fuzzy_cache_reuses_only_exact_matching_structure(self):
        from difflib import SequenceMatcher
        service=self.fixture()
        def score(*args,**kwargs):
            with patch('services.memory.service.SequenceMatcher',wraps=SequenceMatcher) as matcher:
                for number in range(100):
                    text='Unique gallery item %04d'%number
                    service._score_text_candidates('beacon target',[text],fallback_reason='test')
                self.assertEqual(matcher.call_count,1)
            self.assertEqual(len(service._query_score_state.fuzzy_cache),1)
            for number in range(3000):
                service._score_text_candidates('0123456789',['item number %04d'%number],fallback_reason='test')
            self.assertEqual(len(service._query_score_state.fuzzy_cache),2048)
            self.assertEqual(len(service._query_score_state.cache),2048)
            raise RuntimeError('query failed')
        with patch.object(service,'_query_records_impl',side_effect=score):
            with self.assertRaisesRegex(RuntimeError,'query failed'):service._query_records('test')
        for name in ('cache','fuzzy_cache','lexical_query'):
            self.assertFalse(hasattr(service._query_score_state,name))

    def test_recall_snapshot_omits_only_unused_readiness_and_restores_survivors(self):
        service=self.fixture()
        healthy,_=self.accept(service,self.memory(title='Desk healthy'))
        service._run_durable_vectorization(self.claim(service))
        pending,_=self.accept(service,self.memory(memory_id='mem_pending_20260101T000000Z_00000001',title='Desk pending'))
        visible=[('memory',healthy),('memory',pending)]
        full,full_counts=service._durable_index.query_snapshot(visible)
        light,light_counts=service._durable_index.query_snapshot(visible,include_readiness=False)
        self.assertEqual(full_counts,light_counts)
        for left,right in zip(full,light):
            self.assertEqual(left[0],right[0]);self.assertEqual(left[1].model_dump(),right[1].model_dump())
            self.assertEqual(left[3],right[3])
            payload=dict(left[2].payload);self.assertTrue(payload.pop('modality_readiness'))
            self.assertEqual(payload,right[2].payload)
            self.assertEqual(set(left[2].vectors),set(right[2].vectors))
            for name in left[2].vectors:
                np.testing.assert_array_equal(left[2].vectors[name].vector,right[2].vectors[name].vector)
        original=service._durable_index.query_snapshot;phases=[]
        def capture(*args,**kwargs):
            phases.append(kwargs.get('include_readiness',True))
            return original(*args,**kwargs)
        with patch.object(service._durable_index,'query_snapshot',side_effect=capture):result=self.query(service)
        self.assertIn(False,phases);self.assertIn(True,phases);self.assertTrue(result.matches)
        for match in result.matches:
            self.assertEqual(set(match.metadata['modality_readiness']),{'text_dense','image_dense'})

    def test_model_object_binding_uses_monotonic_generation_not_address(self):
        service=self.fixture();generations=[]
        for _ in range(4):
            service._text_embedder=helpers.FakeEmbedder()
            generations.append(service._index_model_binding()['text']['binding_generation'])
        self.assertEqual(generations,sorted(set(generations)))
        self.assertEqual(service._index_model_binding()['text']['binding_generation'],generations[-1])

    def test_query_source_is_authoritative_owned_snapshot(self):
        service=self.fixture(embedder=ExplicitFixtureEmbedder())
        record,_=self.accept(service)
        service._run_durable_vectorization(self.claim(service))
        snapshots,counts=service._durable_index.query_snapshot([('memory',record)])
        self.assertIsNot(snapshots[0][1],record)
        self.assertEqual(snapshots[0][1].model_dump(),record.model_dump())
        self.assertEqual(counts['validated_current'],1)

    def test_query_source_preserves_legacy_json_without_content_proof(self):
        service=self.fixture(embedder=ExplicitFixtureEmbedder())
        record,_=self.accept(service,self.memory(metadata={
            'memory_library_name':'library','nested':{'name':'桌子','negative_zero':-0.0,'whole_float':1.0}}))
        service._run_durable_vectorization(self.claim(service))
        with service._repository.storage.transaction() as tx:
            raw=service._durable_index._raw(tx,'memory',record.memory_id)
            payload=dict(reversed(list(json.loads(raw['payload_json']).items())))
            payload['map_version_id']='legacy-unused'
            for name in ('scene_space_id','localization_session_id','observation_id'):
                if payload.get(name) is None:payload.pop(name,None)
            payload['metadata']['nested']['whole_float']=1
            tx.execute('UPDATE semantic_memories SET payload_json=? WHERE memory_id=?',
                       (json.dumps(payload,indent=2,ensure_ascii=True),record.memory_id))
        with patch.object(service._repository,'_load_record',wraps=service._repository._load_record) as loads:
            snapshots,counts=service._durable_index.query_snapshot([('memory',record)])
        self.assertEqual(loads.call_count,1)
        self.assertIsNot(snapshots[0][1],record)
        self.assertEqual(snapshots[0][1].title,record.title)
        self.assertEqual(counts['validated_current'],1)

    def test_query_source_preserves_normalization_and_mutation_isolation(self):
        service=self.fixture(embedder=ExplicitFixtureEmbedder())
        record,_=self.accept(service,self.memory(map_name=None,metadata={
            'memory_library_name':'library','nested':{'labels':['committed']}}))
        service._run_durable_vectorization(self.claim(service))
        loaded=service._durable_index.load('memory',record.memory_id)
        self.assertEqual(loaded.map_name,'library')
        with patch.object(service._repository,'_load_record',wraps=service._repository._load_record) as loads:
            snapshots,_=service._durable_index.query_snapshot([('memory',loaded)])
        self.assertEqual(loads.call_count,1)
        self.assertEqual(snapshots[0][1].model_dump(),loaded.model_dump())
        loaded.title='Uncommitted title'
        loaded.metadata['nested']['labels'].append('uncommitted')
        snapshots,counts=service._durable_index.query_snapshot([('memory',loaded)])
        self.assertEqual(snapshots[0][1].title,record.title)
        self.assertEqual(snapshots[0][1].metadata['nested']['labels'],['committed'])
        self.assertEqual(snapshots[0][1].map_name,'library')
        self.assertEqual(counts['validated_current'],1)

if __name__=='__main__':unittest.main(verbosity=2)
