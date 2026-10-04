#!/usr/bin/env python3
"""Offline canonical scene/sample identity integration and publication races."""
from __future__ import annotations
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from scripts import check_memory_provider_modes as providers
from scripts.check_memory_builder_sample_identity import snapshot, region, event, SCENE, ANCHOR
from scripts import check_memory_scene_identity_registry as registry
from contracts.runtime_views import SceneSummary
from contracts.spatial_memory import SceneObservationSample, SceneSpaceReviewStatus
from services.memory.repository import MemoryRepository

class SceneIdentityServiceChecks(unittest.TestCase):
    def fixture(self, **kwargs):
        service=providers.ProviderModeChecks.fixture(self,**kwargs)
        service._localization_service=SimpleNamespace(get_active_session=lambda:None)
        service._semantic_map_builder._inspection_pose_planner=Mock(plan_for_target=lambda **kw:[])
        return service
    def source(self, **kwargs):
        return snapshot(regions=[region('r')]).model_copy(update=kwargs)
    def build(self, service, source=None):
        return service._refresh_semantic_map_context(source or self.source())
    def counts(self, service):
        with service._repository.storage.read_snapshot() as tx:
            return tuple(tx.execute('SELECT COUNT(*) FROM '+table).fetchone()[0] for table in
                ('scene_spaces','memory_scene_entity_ids','memory_scene_sample_ids'))
    def add_event(self,service):
        value=event(map_name='library',metadata={'memory_library_name':'library'})
        service._repository.upsert_observation_event(value)
        return value
    def sample(self,service,value):
        service._refresh_scene_spaces_from_observations(perception_context=SimpleNamespace(
            scene_summary=SceneSummary(headline='office',metadata={'semantic_tags':['office']})),observation_events=(value,))
    def test_restart_preserves_scene_anchor_and_no_provider(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=str(Path(tmp)/'source.db');one=self.fixture(path=path);a=self.build(one);one.close()
            two=self.fixture(path=path);b=self.build(two,self.source(update_sequence=2))
            self.assertEqual(a.scene_spaces[0].scene_space_id,b.scene_spaces[0].scene_space_id)
            self.assertEqual(set(a.anchors_by_id),set(b.anchors_by_id))
            self.assertEqual(self.counts(two),(1,2,0))
            self.assertIsNone(two._text_embedder)
    def test_warm_same_source_does_not_rebuild_geometry(self):
        service=self.fixture();source=self.source();a=self.build(service,source)
        with patch.object(service._semantic_map_builder,'build',side_effect=AssertionError('warm build')):
            self.assertIs(self.build(service,source),a)
    def test_source_epoch_replacement_keeps_old_source_and_separates_ids(self):
        service=self.fixture();a=self.build(service);b=self.build(service,self.source(source_epoch='new-map'))
        self.assertNotEqual(a.scene_spaces[0].scene_space_id,b.scene_spaces[0].scene_space_id)
        self.assertEqual(self.counts(service),(2,4,0))
    def test_frame_change_invalidates_equal_sequence_warm_context(self):
        service=self.fixture();a=self.build(service)
        source=self.source(frame_id='odom');source=source.model_copy(update={'semantic_map':source.semantic_map.model_copy(update={'frame_id':'odom'})})
        b=self.build(service,source)
        self.assertEqual(b.frame_id,'odom');self.assertIsNot(a,b)
        self.assertEqual(service._semantic_map_identity_plan.scope_metadata['frame_id'],'odom')
    def test_explicit_anchor_is_not_replaced(self):
        service=self.fixture();source=snapshot(nodes=[{'node_id':'n','label':'office','x':0,'y':0,'anchor_id':ANCHOR}])
        result=self.build(service,source)
        self.assertIn(ANCHOR,result.anchors_by_id)
        with service._repository.storage.read_snapshot() as tx:
            self.assertEqual(tx.execute("SELECT COUNT(*) FROM memory_scene_entity_ids WHERE entity_kind='anchor'").fetchone()[0],0)
    def test_proven_legacy_adoption_keeps_id_labels_samples_and_refs(self):
        service=self.fixture();helper=registry.SceneIdentityRegistryChecks();helper.plan=lambda s: s._repository.prepare_scene_identities('library',snapshot=self.source())
        helper.save_scene(service,source='r')
        stored=service._repository.load_scene_space(SCENE).model_copy(update={'primary_label':'Reviewed office','review_status':SceneSpaceReviewStatus.REVIEWED,
            'observation_samples':[SceneObservationSample(sample_id='old_sample')],'observation_sample_ids':['old_sample'],
            'graph_relation_ids':['rel_legacy_20260101T000000Z_deadbeef'],
            'summary_node_ids':['sum_legacy_20260101T000000Z_deadbeef'],
            'metadata':{**service._repository.load_scene_space(SCENE).metadata,'legacy_reference':'old-ref'}})
        service._repository.upsert_scene_space(stored)
        result=self.build(service).scene_spaces[0]
        self.assertEqual(result.scene_space_id,SCENE);self.assertEqual(result.primary_label,'Reviewed office')
        self.assertEqual(result.linked_anchor_ids,[ANCHOR]);self.assertEqual(result.observation_sample_ids,['old_sample'])
        self.assertIn('rel_legacy_20260101T000000Z_deadbeef',result.graph_relation_ids)
        self.assertIn('sum_legacy_20260101T000000Z_deadbeef',result.summary_node_ids)
        self.assertEqual(result.metadata['legacy_reference'],'old-ref')
    def test_session_change_keeps_owned_identity_and_reviewed_semantics(self):
        service=self.fixture();first=self.build(service).scene_spaces[0]
        source=service._repository.load_scene_space(first.scene_space_id).model_copy(update={
            'primary_label':'Reviewed room','review_status':SceneSpaceReviewStatus.REVIEWED})
        service._repository.upsert_scene_space(source)
        service._localization_service.get_active_session=lambda:SimpleNamespace(session_id='new-session',map_name='library')
        second=self.build(service).scene_spaces[0]
        self.assertEqual(second.scene_space_id,first.scene_space_id)
        self.assertEqual(second.linked_anchor_ids,first.linked_anchor_ids)
        self.assertEqual(second.primary_label,'Reviewed room')
        self.assertEqual(second.localization_session_id,'new-session')

    def test_unbound_legacy_data_is_preserved_and_diagnosed(self):
        service=self.fixture();helper=registry.SceneIdentityRegistryChecks();helper.save_scene(service,source='r',scope=False)
        result=self.build(service).scene_spaces[0]
        self.assertNotEqual(result.scene_space_id,SCENE);self.assertIsNotNone(service._repository.load_scene_space(SCENE))
        self.assertTrue(any(item['status']=='unbound' for item in service._scene_identity_diagnostics['map']))
    def test_concurrent_canonical_winner_remaps_once_without_geometry_repeat(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=str(Path(tmp)/'source.db');first=self.fixture(path=path);second=self.fixture(path=path)
            first._durable_index.activation_generation=second._durable_index.activation_generation
            original=first._semantic_map_builder.build;other=[]
            def build(*args,**kwargs):
                result=original(*args,**kwargs);other.append(self.build(second));return result
            with patch.object(first._semantic_map_builder,'build',side_effect=build) as observed:
                result=self.build(first)
            self.assertEqual(observed.call_count,1)
            self.assertEqual(result.scene_spaces[0].scene_space_id,other[0].scene_spaces[0].scene_space_id)
            self.assertEqual(set(result.anchors_by_id),set(other[0].anchors_by_id));self.assertEqual(self.counts(first),(1,2,0))
    def test_source_failure_rolls_back_entire_alias_and_scene_batch(self):
        service=self.fixture();original=service._repository._commit_prepared_scene_space;calls=[]
        def write(value):
            calls.append(value)
            if len(calls)==2:raise RuntimeError('injected')
            return original(value)
        with patch.object(service._repository,'_commit_prepared_scene_space',side_effect=write):
            with self.assertRaisesRegex(RuntimeError,'injected'):
                self.build(service,snapshot(regions=[region('one'),region('two')]))
        self.assertEqual(self.counts(service),(0,0,0));self.assertFalse(service._latest_semantic_map_context.scene_spaces)
    def test_outer_transaction_read_own_write_rollback_does_not_publish(self):
        service=self.fixture()
        with self.assertRaisesRegex(RuntimeError,'rollback'):
            with service._repository.storage.transaction():
                result=self.build(service)
                self.assertEqual(len(result.scene_spaces),1);self.assertEqual(self.counts(service),(1,2,0))
                self.assertFalse(service._latest_semantic_map_context.scene_spaces)
                raise RuntimeError('rollback')
        self.assertEqual(self.counts(service),(0,0,0));self.assertFalse(service._latest_semantic_map_context.scene_spaces)
    def test_deleted_scene_rebuild_returns_new_id_before_outer_commit(self):
        service=self.fixture();old=self.build(service);identifier=old.scene_spaces[0].scene_space_id
        with service._repository.storage.transaction():
            service._repository.delete_scene_spaces([identifier])
            new=self.build(service)
            self.assertNotEqual(new.scene_spaces[0].scene_space_id,identifier)
            self.assertIs(service._latest_semantic_map_context,old)
        self.assertEqual(service._latest_semantic_map_context.scene_spaces[0].scene_space_id,new.scene_spaces[0].scene_space_id)
    def test_scene_context_does_not_share_source_cache_objects(self):
        service=self.fixture();context=self.build(service);value=context.scene_spaces[0];identifier=value.scene_space_id
        value.primary_label='caller mutation'
        self.assertNotEqual(service._scene_spaces_by_id[identifier].primary_label,'caller mutation')
        self.assertNotEqual(service._repository.load_scene_space(identifier).primary_label,'caller mutation')
    def test_source_and_alias_batch_one_commit_before_derived_graph_work(self):
        directory=tempfile.TemporaryDirectory();self.addCleanup(directory.cleanup)
        service=self.fixture(path=str(Path(directory.name)/'source.db'));controls=[];original=service._repository.storage._control
        def control(connection,sql):controls.append(sql);return original(connection,sql)
        with patch.object(service,'_record_scene_space_graph'),patch.object(service._repository.storage,'_control',side_effect=control):
            self.build(service,snapshot(regions=[region('one'),region('two')]))
        self.assertEqual(controls.count('COMMIT'),1);self.assertEqual(self.counts(service),(2,4,0))
    def test_geometry_and_source_preparation_outside_writer_lock(self):
        service=self.fixture();original=service._repository._prepare_scene_space_write
        def prepare(value):
            self.assertFalse(service._repository.storage.in_transaction)
            self.assertFalse(service._instances_lock._is_owned())
            return original(value)
        with patch.object(service,'_record_scene_space_graph'),patch.object(service._repository,'_prepare_scene_space_write',side_effect=prepare):
            self.build(service)
    def test_delayed_old_map_callback_does_not_replace_new_epoch_context(self):
        service=self.fixture();entered=threading.Event();release=threading.Event();errors=[]
        def run():
            try:
                with service._repository.storage.transaction() as tx:
                    tx.after_commit(lambda:(entered.set(),release.wait(5)))
                    self.build(service)
            except BaseException as exc:errors.append(exc)
        worker=threading.Thread(target=run);worker.start();self.assertTrue(entered.wait(5))
        try:new=self.build(service,self.source(source_epoch='replacement'))
        finally:release.set();worker.join(5)
        self.assertFalse(errors);self.assertFalse(worker.is_alive())
        self.assertIs(service._latest_semantic_map_context,new)
    def test_sample_alias_and_source_one_commit(self):
        directory=tempfile.TemporaryDirectory();self.addCleanup(directory.cleanup)
        service=self.fixture(path=str(Path(directory.name)/'source.db'));context=self.build(service);value=self.add_event(service);controls=[];original=service._repository.storage._control
        def control(connection,sql):controls.append(sql);return original(connection,sql)
        with patch.object(service,'_record_scene_space_graph'),patch.object(service._repository.storage,'_control',side_effect=control):self.sample(service,value)
        self.assertEqual(controls.count('COMMIT'),1);self.assertEqual(self.counts(service),(1,2,1))
        stored=service._repository.load_scene_space(context.scene_spaces[0].scene_space_id)
        self.assertTrue(stored.observation_samples);self.assertEqual(stored.observation_sample_ids,stored.model_summary.evidence_observation_sample_ids)
    def test_sample_alias_survives_eviction_and_restart(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=str(Path(tmp)/'source.db');first=self.fixture(path=path);context=self.build(first);value=self.add_event(first);self.sample(first,value)
            stored=first._repository.load_scene_space(context.scene_spaces[0].scene_space_id);sample_id=stored.observation_samples[0].sample_id
            first._repository.upsert_scene_space(stored.model_copy(update={'observation_samples':[],'observation_sample_ids':[]}));first.close()
            second=self.fixture(path=path);self.build(second,self.source(update_sequence=2));value=second._repository.load_observation_events()[0];self.sample(second,value)
            self.assertEqual(second._repository.load_scene_space(stored.scene_space_id).observation_samples[0].sample_id,sample_id)
    def test_sample_legacy_id_seeded_even_when_sampler_reuses_it(self):
        service=self.fixture();context=self.build(service);value=self.add_event(service);stored=service._repository.load_scene_space(context.scene_spaces[0].scene_space_id)
        old=SceneObservationSample(sample_id='historical_sample',observation_event_id=value.event_id,pose=value.pose,timestamp=value.timestamp)
        service._repository.upsert_scene_space(stored.model_copy(update={'observation_samples':[old],'observation_sample_ids':[old.sample_id]}))
        self.sample(service,value)
        self.assertEqual(service._repository.load_scene_space(stored.scene_space_id).observation_samples[0].sample_id,old.sample_id)
        with service._repository.storage.read_snapshot() as tx:self.assertEqual(tx.execute('SELECT public_id FROM memory_scene_sample_ids').fetchone()[0],old.sample_id)
    def test_sample_outer_rollback_keeps_old_context_and_no_alias(self):
        service=self.fixture();old=self.build(service);value=self.add_event(service)
        with self.assertRaisesRegex(RuntimeError,'rollback'):
            with service._repository.storage.transaction():
                self.sample(service,value);self.assertEqual(self.counts(service),(1,2,1));self.assertIs(service._latest_semantic_map_context,old)
                raise RuntimeError('rollback')
        self.assertEqual(self.counts(service),(1,2,0));self.assertFalse(service._repository.load_scene_spaces()[0].observation_samples)
    def test_deleted_sample_owner_is_not_resurrected(self):
        service=self.fixture();old=self.build(service);value=self.add_event(service)
        service._repository.delete_scene_spaces([old.scene_spaces[0].scene_space_id]);self.sample(service,value)
        self.assertEqual(self.counts(service),(0,1,0))
    def test_sample_concurrent_alias_winner_remaps_all_evidence(self):
        service=self.fixture();context=self.build(service);value=self.add_event(service);original=service._scene_space_sampler.sample;winner=[]
        def sample(scene,**kwargs):
            result=original(scene,**kwargs)
            plan=service._repository.prepare_scene_identities('library',scene=scene);winner.append(plan.resolve_sample('observation_event',value.event_id))
            with service._repository.storage.transaction() as tx:plan.admit(tx)
            return result
        with patch.object(service._scene_space_sampler,'sample',side_effect=sample):self.sample(service,value)
        stored=service._repository.load_scene_space(context.scene_spaces[0].scene_space_id)
        self.assertEqual(stored.observation_sample_ids,winner)
        self.assertEqual([x.sample_id for x in stored.observation_samples],winner)
        self.assertEqual(stored.model_summary.evidence_observation_sample_ids,winner)
        self.assertEqual(stored.metadata['scene_space_understanding']['evidence_sample_ids'],winner)
        for label in stored.candidate_labels:self.assertEqual(label.evidence_observation_sample_ids,winner)
    def test_sample_standalone_source_change_preserves_newer_row(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=str(Path(tmp)/'source.db');service=self.fixture(path=path);context=self.build(service);value=self.add_event(service)
            other=MemoryRepository(path);self.addCleanup(other.close);original=service._scene_space_sampler.sample
            def sample(scene,**kwargs):
                result=original(scene,**kwargs);new=other.load_scene_space(scene.scene_space_id).model_copy(update={'primary_label':'new source'})
                other.upsert_scene_space(new);return result
            with patch.object(service._scene_space_sampler,'sample',side_effect=sample):self.sample(service,value)
            stored=service._repository.load_scene_space(context.scene_spaces[0].scene_space_id)
            self.assertEqual(stored.primary_label,'new source');self.assertFalse(stored.observation_samples);self.assertEqual(self.counts(service),(1,2,0))

if __name__=='__main__':unittest.main()
