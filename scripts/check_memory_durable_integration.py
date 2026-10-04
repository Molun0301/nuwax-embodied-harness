#!/usr/bin/env python3
"""Offline source/outbox/service integration controls. Synthetic temporary data only."""
from __future__ import annotations
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
from datetime import timedelta
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scripts.check_memory_storage_transactions import records, NOW
from contracts.geometry import Pose, Vector3
from contracts.memory import MemoryRecordKind, MemoryPayloadFilter
from contracts.spatial_memory import InstanceDynamicState
from services.memory.service import MemoryService
from services.memory.durable_index import TOKEN, INDEX_META
from scripts.memory_test_vectors import ExplicitTextEmbedder
from services.memory.vectorization_outbox import OutboxConflict, RetryPolicy
from services.memory.vector_store import SQLiteNamedVectorStore, NamedVector

class FakeEmbedder(ExplicitTextEmbedder):
    def __init__(self):
        super().__init__(dimension=16)
        self.calls = 0
        self.entered = threading.Event()
        self.release = None
        self.fail = False
    def embed_text(self, text):
        self.calls += 1
        self.entered.set()
        if self.release is not None:
            assert self.release.wait(5), 'test barrier expired'
        if self.fail:
            raise RuntimeError('synthetic')
        return super().embed_text(text)

class IntegrationChecks(unittest.TestCase):
    def fixture(self, *, path=':memory:', store=None, embedder=None):
        model = embedder or FakeEmbedder()
        service = MemoryService(localization_service=SimpleNamespace(get_current_pose=lambda: None, get_active_session=lambda: None),
            mapping_service=None, observation_service=None, perception_service=None,
            state_store=SimpleNamespace(write=lambda *a, **kw: None),
            memory_db_path=path, text_embedder=model, text_embedding_model=model.model_name,
            text_embedding_dimension=16, vector_store=store)
        service._get_current_pose = lambda: None
        service._memory_enabled = True
        service._active_library_name = 'library'
        service._durable_index.activate('library')
        self.addCleanup(service.close)
        return service
    def memory(self, **updates):
        return records()[1][3].model_copy(update=updates, deep=True)
    def accept(self, service, record=None):
        record = record or self.memory()
        service._repository.upsert_semantic_memory(record)
        service._semantic_entries_by_id[record.memory_id] = record
        index = service._durable_index
        with index.storage.read_snapshot() as tx:
            row = index._state(tx, 'memory', record.memory_id)
            receipt = index.outbox.current(index.key(row))
        return record, receipt
    def claim(self, service):
        box = service._durable_index.outbox
        token = box.start_worker()
        return box.claim_next(token)
    def query(self, service):
        return service._query_records('Desk', allowed_record_kinds=(MemoryRecordKind.SCENE,),
                                     similarity_threshold=.1, limit=5, payload_filter=None)
    def test_acceptance_atomic_and_private_token(self):
        service = self.fixture()
        record, receipt = self.accept(service)
        copied = record.model_copy(deep=True)
        self.assertEqual(getattr(copied, TOKEN), getattr(record, TOKEN))
        self.assertNotIn(TOKEN, copied.model_dump())
        self.assertEqual(record, copied)
        with self.assertRaises(RuntimeError):
            with service._repository.storage.transaction():
                service._repository.upsert_semantic_memory(record.model_copy(update={'title':'Changed'}))
                raise RuntimeError('rollback')
        current = service._durable_index.outbox.current(receipt.version.key)
        self.assertEqual(current, receipt)
        self.assertEqual(service._repository.load_semantic_memories()[0].title, 'Desk')
    def test_metadata_does_not_supersede_and_stale_rmw_conflicts(self):
        service = self.fixture()
        record, receipt = self.accept(service)
        claim = self.claim(service)
        stale = record.model_copy(deep=True)
        for n in range(30):
            record = record.model_copy(update={'observation_count':n})
            service._repository.upsert_semantic_memory(record)
        self.assertEqual(service._durable_index.outbox.current(receipt.version.key), receipt)
        self.assertEqual(service._durable_index.outbox.snapshot()['accepted'], 1)
        self.assertEqual(getattr(record,TOKEN)[2],31)
        with self.assertRaises(OutboxConflict):
            service._repository.upsert_semantic_memory(stale)
        service._run_durable_vectorization(claim)
        self.assertEqual(service._text_embedder.calls,1)
        result = self.query(service)
        self.assertTrue(result.matches)
        self.assertEqual(result.metadata['vector_provenance_counts']['validated_current'],1)
    def test_all_five_sources_are_durable(self):
        service=self.fixture()
        for name,_,_,record in records()[:5]:
            getattr(service._repository,'upsert_'+name)(record)
        self.assertEqual(service._durable_index.outbox.snapshot()['accepted'],5)
    def test_separate_injected_store_and_borrowed_ownership(self):
        with tempfile.TemporaryDirectory() as tmp:
            store=SQLiteNamedVectorStore(str(Path(tmp)/'derived.db'))
            self.addCleanup(store.close)
            service=self.fixture(path=str(Path(tmp)/'source.db'),store=store)
            record,receipt=self.accept(service)
            service._run_durable_vectorization(self.claim(service))
            self.assertTrue(self.query(service).matches)
            self.assertTrue(service.close())
            self.assertIsNotNone(store.get_point(record.memory_id))
    def test_legacy_positive_and_lexical_without_vectors(self):
        service=self.fixture()
        record=self.memory()
        # Simulate an existing pre-index source without sidecar/provenance.
        service._repository._index=None
        service._repository.upsert_semantic_memory(record)
        service._repository.attach_index(service._durable_index)
        service._semantic_entries_by_id[record.memory_id]=record
        payload=service._build_memory_payload(record)
        named=NamedVector('text_dense',service._text_embedder.model_name,16,service._text_embedder.embed_text(payload['retrieval_text']))
        service._vector_store.upsert_point(point_id=record.memory_id,collection_name='episodic_observations',
            record_kind=record.kind.value,payload=payload,vectors={'text_dense':named})
        result=self.query(service)
        self.assertTrue(result.matches)
        self.assertTrue(result.metadata['includes_unverified_legacy_scores'])
        service._vector_store.delete_points([record.memory_id])
        service._text_embedder.fail=True
        result=self.query(service)
        self.assertTrue(result.matches)
    def test_4101_jobs_are_not_silently_evicted(self):
        service=self.fixture()
        with service._repository.storage.transaction():
            for n in range(4101):
                self.accept(service,self.memory(memory_id='mem_synthetic_20260101T000000Z_%08d'%n))
        status=service._durable_index.outbox.snapshot()
        self.assertEqual(status['accepted'],4101)
        self.assertEqual(status['pending_ready'],4101)
        self.assertEqual(status['desired_accounted'],4101)
        self.assertFalse(hasattr(service,'_vectorization_queue'))
        self.assertFalse(hasattr(service,'_pending_vectorization'))
        box=service._durable_index.outbox; token=box.start_worker()
        processed=0
        while True:
            claim=box.claim_next(token)
            if claim is None:break
            service._run_durable_vectorization(claim);processed+=1
        self.assertEqual(processed,4101)
        self.assertEqual(service._text_embedder.calls,4101)
        self.assertEqual(box.snapshot()['completed_fresh'],4101)
        self.assertEqual(service._vector_store.count_vectors_by_name()['text_dense'],4101)
        self.assertEqual(box.snapshot()['desired_accounted'],4101)
        self.assertEqual(service._vector_store.count_points(),4101)

    def test_slow_inference_finishes_during_continuous_metadata_writes(self):
        model=FakeEmbedder(); model.release=threading.Event()
        service=self.fixture(embedder=model)
        record,receipt=self.accept(service)
        claim=self.claim(service)
        worker=threading.Thread(target=service._run_durable_vectorization,args=(claim,))
        worker.start(); self.assertTrue(model.entered.wait(2))
        for n in range(50):
            record=record.model_copy(update={'observation_count':n})
            service._repository.upsert_semantic_memory(record)
        self.assertEqual(service._durable_index.outbox.current(receipt.version.key),receipt)
        keep_writing=threading.Event(); writer_ready=threading.Event()
        def continuing_writer():
            current=service._repository.load_semantic_memories()[0]
            service._repository.upsert_semantic_memory(current.model_copy(update={'observation_count':100,'pose':Pose(frame_id='map',position=Vector3(x=100.,y=1.,z=0.)), 'timestamp':NOW+timedelta(seconds=100)}))
            writer_ready.set()
            assert keep_writing.wait(3)
            current=service._repository.load_semantic_memories()[0]
            service._repository.upsert_semantic_memory(current.model_copy(update={'observation_count':101}))
        metadata_writer=threading.Thread(target=continuing_writer);metadata_writer.start()
        self.assertTrue(writer_ready.wait(2));self.assertTrue(metadata_writer.is_alive())
        model.release.set(); worker.join(2); self.assertFalse(worker.is_alive())
        self.assertEqual(model.calls,1)
        result=self.query(service)
        self.assertTrue(result.matches)
        self.assertEqual(result.matches[0].semantic_memory.pose.position.x,100.)
        self.assertEqual(result.matches[0].semantic_memory.timestamp,NOW+timedelta(seconds=100))
        self.assertEqual(result.matches[0].metadata['vector_provenance']['text_dense']['status'],'validated_current')
        self.assertTrue(metadata_writer.is_alive())
        self.assertEqual(service._semantic_entries_by_id[record.memory_id].observation_count,100)
        keep_writing.set();metadata_writer.join(2)
        self.assertEqual(service._durable_index.outbox.receipt_outcome(receipt.token),'completed_fresh')
        point=service._vector_store.get_point(record.memory_id)
        self.assertIn('text_dense',point.vectors)
        self.assertEqual(service._semantic_entries_by_id[record.memory_id].observation_count,101)

    def test_partial_image_failure_preserves_text_through_retry_and_exhaustion(self):
        service=self.fixture()
        service._configured_image_model='synthetic-unavailable'
        service._load_index_image=lambda ids:(b'synthetic','digest',None)
        service._durable_index.outbox.retry_policy=RetryPolicy(max_attempts=2,base_delay=0,max_delay=0)
        record,receipt=self.accept(service,self.memory(artifact_ids=['art_image_20260101T000000Z_00000001']))
        box=service._durable_index.outbox;token=box.start_worker()
        service._run_durable_vectorization(box.claim_next(token))
        self.assertEqual(service._text_embedder.calls,1)
        self.assertEqual(box.inspect(receipt.version.key)['status'],'backoff')
        snapshots,counts=service._durable_index.query_snapshot([('memory',record)])
        self.assertEqual(counts['validated_current'],1)
        service._run_durable_vectorization(box.claim_next(token))
        self.assertEqual(service._text_embedder.calls,1)
        self.assertEqual(box.inspect(receipt.version.key)['status'],'failed_terminal')
        self.assertIn('text_dense',snapshots[0][2].vectors)
        retried=box.retry(receipt)
        self.assertEqual(retried.version,receipt.version)
        self.assertEqual(service._durable_index.query_snapshot([('memory',record)])[1]['validated_current'],1)

    def test_separate_cache_rejected_aba_publication_stays_invisible(self):
        with tempfile.TemporaryDirectory() as tmp:
            store=SQLiteNamedVectorStore(str(Path(tmp)/'vectors.db'));self.addCleanup(store.close)
            service=self.fixture(path=str(Path(tmp)/'source.db'),store=store)
            record,receipt=self.accept(service);claim=self.claim(service);index=service._durable_index
            index.activate('other');index.activate('library')
            named=NamedVector('text_dense',service._text_embedder.model_name,16,np.ones(16,dtype=np.float32))
            self.assertFalse(index.publish(claim,named,actual={'route':'text_only'}))
            snapshots,counts=index.query_snapshot([('memory',record)])
            self.assertNotIn('text_dense',snapshots[0][2].vectors)
            self.assertEqual(counts['validated_current'],0)
            self.assertEqual(counts['known_stale_or_incompatible'],1)

    def test_partial_publish_cannot_relabel_known_stale_legacy_other_modality(self):
        service=self.fixture();index=service._durable_index
        service._image_embedder=service._text_embedder
        old=self.memory();payload=service._build_memory_payload(old)
        named=NamedVector('text_dense',service._text_embedder.model_name,16,np.ones(16,dtype=np.float32))
        service._vector_store.upsert_point(point_id=old.memory_id,collection_name='episodic_observations',
            record_kind=old.kind.value,payload=payload,vectors={'text_dense':named})
        new,receipt=self.accept(service,self.memory(title='New title'))
        claim=self.claim(service)
        before=index.query_snapshot([('memory',new)])[0][0][2]
        self.assertNotIn('text_dense',before.vectors)
        image=NamedVector('image_dense',named.model_name,16,named.vector)
        self.assertTrue(index.publish(claim,image,actual={'route':'image_only','binding':service._index_model_binding()}))
        after=index.query_snapshot([('memory',new)])[0][0][2]
        self.assertNotIn('text_dense',after.vectors)
        self.assertIn('image_dense',after.vectors)
        with index.storage.read_snapshot() as tx:
            archive=tx.execute('SELECT vector_payload,point_payload_json FROM memory_index_legacy_archive WHERE point_id=? AND vector_name=?',
                               (new.memory_id,'text_dense')).fetchone()
        self.assertEqual(bytes(archive[0]),named.vector.tobytes())
        self.assertIn('Desk',archive[1])

    def test_input_change_fences_slow_result_and_source_aba(self):
        service=self.fixture();index=service._durable_index
        record,receipt=self.accept(service);claim=self.claim(service)
        changed=record.model_copy(update={'title':'Changed'})
        service._repository.upsert_semantic_memory(changed)
        named=NamedVector('text_dense',service._text_embedder.model_name,16,np.ones(16,dtype=np.float32))
        self.assertFalse(index.publish(claim,named,actual={'route':'text_only'}))
        service._repository.delete_semantic_memories([record.memory_id])
        recreated=self.memory();service._repository.upsert_semantic_memory(recreated)
        with self.assertRaises(OutboxConflict):service._repository.upsert_semantic_memory(changed)
        self.assertFalse(index.publish(claim,named,actual={'route':'text_only'}))

    def test_dependency_changes_are_value_specific_and_atomic(self):
        service=self.fixture();index=service._durable_index
        location=records()[0][3];service._repository.upsert_tagged_location(location)
        record,receipt=self.accept(service,self.memory(linked_location_id=location.location_id))
        updated=location.model_copy(update={'metadata':{**location.metadata,'pose_quality':'new'}})
        service._repository.upsert_tagged_location(updated)
        self.assertEqual(index.outbox.current(receipt.version.key),receipt)
        renamed=updated.model_copy(update={'name':'Other desk'})
        service._repository.upsert_tagged_location(renamed)
        self.assertNotEqual(index.outbox.current(receipt.version.key).version,receipt.version)
        before=index.outbox.current(receipt.version.key)
        with self.assertRaises(RuntimeError):
            with index.storage.transaction():
                service._repository.upsert_tagged_location(renamed.model_copy(update={'name':'Rolled back'}))
                raise RuntimeError('rollback')
        self.assertEqual(index.outbox.current(receipt.version.key),before)
        self.assertEqual(service._tagged_locations_by_id[location.location_id].name,'Other desk')

    def test_close_retains_storage_while_provider_is_uncooperative(self):
        model=FakeEmbedder();model.release=threading.Event()
        service=self.fixture(embedder=model);record,receipt=self.accept(service)
        service._start_vectorization_worker();self.assertTrue(model.entered.wait(2))
        worker=service._vectorization_worker
        self.assertFalse(service.close(timeout=.01))
        self.assertIs(service._vectorization_worker,worker)
        self.assertTrue(service._repository.load_semantic_memories())
        model.release.set();worker.join(2)
        self.assertTrue(service.close(timeout=.1));self.assertTrue(service.close())

    def test_hydration_aba_keeps_snapshot_business_identity(self):
        service=self.fixture();record,receipt=self.accept(service)
        original=service._repository._tag_loaded; once=[False]
        def replace_between_decode_and_tag(decoded, token=None):
            if not once[0]:
                once[0]=True
                service._repository.delete_semantic_memories([record.memory_id])
                service._repository.upsert_semantic_memory(self.memory())
            return original(decoded,token)
        with patch.object(service._repository,'_tag_loaded',side_effect=replace_between_decode_and_tag):
            stale=service._repository.load_semantic_memories()[0]
        self.assertEqual(getattr(stale,TOKEN)[1],receipt.version.record_incarnation)
        with self.assertRaises(OutboxConflict):
            service._repository.upsert_semantic_memory(stale.model_copy(update={'title':'stale'}))
        self.assertEqual(service._repository.load_semantic_memories()[0].title,'Desk')

    def test_dynamic_live_record_does_not_block_unrelated_query(self):
        service=self.fixture();record,receipt=self.accept(service)
        service._run_durable_vectorization(self.claim(service))
        instance=records()[2][3].model_copy(update={'dynamic_state':InstanceDynamicState.STATIC})
        service._repository.upsert_semantic_instance(instance)
        dynamic=instance.model_copy(update={'dynamic_state':InstanceDynamicState.DYNAMIC})
        service._semantic_instances_by_id[instance.instance_id]=dynamic
        service._repository.delete_semantic_instances([instance.instance_id])
        result=self.query(service)
        self.assertTrue(result.matches)
        self.assertNotIn('input_changed_during_query',result.metadata)

    def test_completed_missing_image_repairs_without_hiding_or_reembedding_text(self):
        service=self.fixture();service._image_embedder=service._text_embedder
        service._image_embedder.embed_image_bytes=lambda data:np.ones(16,dtype=np.float32)
        service._load_index_image=lambda ids:(b'x','digest',None)
        record,receipt=self.accept(service,self.memory(artifact_ids=['art_image_20260101T000000Z_00000001']))
        box=service._durable_index.outbox;token=box.start_worker()
        service._run_durable_vectorization(box.claim_next(token))
        self.assertEqual(service._text_embedder.calls,1)
        with service._repository.storage.transaction() as tx:
            tx.execute("DELETE FROM memory_named_vectors WHERE vector_name='image_dense'")
        for _ in range(2):
            points,counts=service._durable_index.query_snapshot([('memory',record)])
            self.assertIn('text_dense',points[0][2].vectors)
        service._run_durable_vectorization(box.claim_next(token))
        self.assertEqual(service._text_embedder.calls,1)
        self.assertIn('image_dense',service._vector_store.get_point(record.memory_id).vectors)

    def test_filter_and_arrival_lookup_use_current_eligibility(self):
        service=self.fixture();record,receipt=self.accept(service)
        service._run_durable_vectorization(self.claim(service))
        self.assertIn('text_dense',service._durable_index.validated_point(record.memory_id).vectors)
        newer=record.model_copy(update={'title':'New target','map_name':'new-map'})
        service._repository.upsert_semantic_memory(newer)
        point=service._durable_index.validated_point(record.memory_id)
        self.assertNotIn('text_dense',point.vectors)
        self.assertEqual(point.payload['map_name'],'new-map')
        self.assertTrue(service._point_matches_filter(point,MemoryPayloadFilter(map_name='new-map')))
        self.assertFalse(service._point_matches_filter(point,MemoryPayloadFilter(map_name='old-map')))

    def test_query_race_does_not_transfer_old_similarity(self):
        service=self.fixture();record,receipt=self.accept(service)
        service._run_durable_vectorization(self.claim(service))
        original=service._score_candidate;once=[False]
        def mutate_before_score(**kwargs):
            if not once[0]:
                once[0]=True
                service._repository.upsert_semantic_memory(record.model_copy(update={'title':'Completely unrelated','summary':'nothing'}))
            return original(**kwargs)
        with patch.object(service,'_score_candidate',side_effect=mutate_before_score):
            result=self.query(service)
        self.assertFalse(any(match.metadata.get('vector_provenance',{}).get('text_dense',{}).get('status')=='validated_current' for match in result.matches))

    def test_disable_activate_preserves_vectors_and_history_false_visibility(self):
        service=self.fixture();record,receipt=self.accept(service)
        service._run_durable_vectorization(self.claim(service))
        count=service._vector_store.count_vectors_by_name()
        with patch.object(service,'_start_vectorization_worker'),patch.object(service,'_start_deferred_refine_worker'):
            service.disable_memory_library()
            self.assertEqual(service._vector_store.count_vectors_by_name(),count)
            service.activate_memory_library(library_name='library',load_history=False)
            self.assertFalse(service._semantic_entries_by_id)
            self.assertEqual(service._vector_store.count_vectors_by_name(),count)
            service.activate_memory_library(library_name='library',load_history=True)
            self.assertIn(record.memory_id,service._semantic_entries_by_id)
            self.assertTrue(self.query(service).matches)

    def test_reset_new_library_incarnation_and_long_aliases(self):
        service=self.fixture();record,receipt=self.accept(service)
        service._delete_memory_library_records('library')
        service._durable_index.activate('library')
        _,second=self.accept(service)
        self.assertNotEqual(receipt.version.key.library_incarnation,second.version.key.library_incarnation)
        long_library='library-'+('中'*300)
        long_id='mem_'+('longkind'*40)+'_20260101T000000Z_00000001'
        long_record=self.memory(memory_id=long_id,metadata={'memory_library_name':long_library})
        service._repository.upsert_semantic_memory(long_record)
        self.assertEqual(service._repository.load_semantic_memories(library_name=long_library)[0].memory_id,long_id)
        with service._repository.storage.read_snapshot() as tx:
            state=service._durable_index._state(tx,'memory',long_id)
            self.assertLess(len(state['internal_id']),256)
            self.assertEqual(state['external_id'],long_id)

    def test_hot_migration_row_does_not_starve_later_legacy_rows(self):
        service=self.fixture();index=service._durable_index
        service._repository._index=None
        first=self.memory(memory_id='mem_synthetic_20260101T000000Z_00000001')
        second=self.memory(memory_id='mem_synthetic_20260101T000000Z_00000002')
        service._repository.upsert_semantic_memory(first);service._repository.upsert_semantic_memory(second)
        service._repository.attach_index(index)
        actual=index.ensure
        def race(kind,record):
            if record.memory_id==first.memory_id:
                service._repository.upsert_semantic_memory(record)
                raise OutboxConflict('synthetic hot writer')
            return actual(kind,record)
        with patch.object(index,'ensure',side_effect=race):
            for _ in range(5):index.reconcile_page(page_size=8)
        with index.storage.read_snapshot() as tx:
            self.assertIsNotNone(index._state(tx,'memory',second.memory_id))

if __name__=='__main__':
    unittest.main(verbosity=2)
