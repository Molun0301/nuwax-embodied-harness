#!/usr/bin/env python3
"""Offline persistent scene identity preparation, admission, adoption and cleanup."""
from __future__ import annotations

import sys
import tempfile
import threading
import unittest
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from contracts.spatial_memory import SceneObservationSample
from scripts import check_memory_provider_modes as providers
from scripts.check_memory_builder_sample_identity import ANCHOR, EVENT, INSTANCE, SCENE, scene, snapshot
from scripts.check_memory_storage_transactions import records
from services.memory.repository import MemoryRepository
from services.memory.scene_identity import IdentityPlanChanged, IdentityScopeChanged


class SceneIdentityRegistryChecks(unittest.TestCase):
    fixture = providers.ProviderModeChecks.fixture

    def plan(self, service, **kwargs):
        return service._repository.prepare_scene_identities('library', snapshot=kwargs.get('snapshot', snapshot(regions=[])))

    def admit(self, plan):
        with plan.storage.transaction() as tx:
            return plan.admit(tx)

    def save_scene(self, service, *, identifier=SCENE, source='region-a', anchor=ANCHOR, scope=True, samples=(), source_kind='semantic_region'):
        payload=scene(observation_samples=list(samples),linked_anchor_ids=[anchor],
            linked_semantic_region_ids=[source] if source_kind=='semantic_region' else [],
            linked_topology_node_ids=[source] if source_kind=='topology_node' else [],
            attributes={'source':source_kind,'candidate_generation':'semantic_map_builder'},
            metadata={'memory_library_name':'library'}).model_copy(update={'scene_space_id':identifier,'map_name':'library'})
        if scope:
            payload.metadata['scene_identity_scope']=self.plan(service).scope_metadata
        return service._repository.upsert_scene_space(payload)

    def test_preparation_resolves_stably_without_writes_or_reservations(self):
        service=self.fixture();plan=self.plan(service)
        a=plan.resolve('anchor','region','A:B');b=plan.resolve('scene','region','A:B')
        self.assertEqual(a,plan.resolve('anchor','region','A:B'))
        self.assertEqual(len(a.rsplit('_',1)[1]),32);self.assertEqual(len(b.rsplit('_',1)[1]),32)
        with plan.storage.read_snapshot() as tx:
            self.assertEqual(tx.execute('SELECT COUNT(*) FROM memory_scene_identity_scopes').fetchone()[0],0)
            self.assertEqual(tx.execute('SELECT COUNT(*) FROM memory_scene_entity_ids').fetchone()[0],0)
        self.assertEqual(set(plan.scope_metadata),{'map_name','source_name','source_epoch','frame_id'})

    def test_unknown_library_cannot_be_created_by_preparation(self):
        repository=MemoryRepository();self.addCleanup(repository.close)
        with self.assertRaises(IdentityScopeChanged):repository.prepare_scene_identities('name',snapshot=snapshot())
        self.assertFalse(repository.load_memory_libraries())

    def test_map_keys_reopen_and_ignore_snapshot_update_sequence(self):
        with tempfile.TemporaryDirectory() as directory:
            path=str(Path(directory)/'memory.db');first=self.fixture(path=path)
            a=self.plan(first);identifier=a.resolve('scene','region','R');self.admit(a);first.close()
            second=self.fixture(path=path)
            b=self.plan(second,snapshot=snapshot().model_copy(update={'update_sequence':99}))
            self.assertEqual(b.resolve('scene','region','R'),identifier)
            self.admit(b)

    def test_changed_scope_components_never_alias_old_map_entities(self):
        service=self.fixture();base=self.plan(service);old=base.resolve('scene','region','R');self.admit(base)
        for field,value in [('map_name','other'),('source_name','new-owner'),('source_epoch','replacement'),('frame_id','other_frame')]:
            newer=self.plan(service,snapshot=snapshot().model_copy(update={field:value}))
            self.assertNotEqual(newer.resolve('scene','region','R'),old)
            self.admit(newer)
        with service._repository.storage.read_snapshot() as tx:
            self.assertEqual(tx.execute('SELECT COUNT(*) FROM memory_scene_identity_scopes').fetchone()[0],5)

    def test_topology_only_optional_frame_uses_the_builder_effective_frame(self):
        from services.memory.semantic_map_builder import SemanticMapBuilder
        service=self.fixture();builder=SemanticMapBuilder()
        source=snapshot(nodes=[{'node_id':'n','label':'office','x':0,'y':0}]).model_copy(update={'frame_id':None})
        plan=self.plan(service,snapshot=source)
        built=builder.build(source,map_name='library',identity_resolver=plan.resolve)
        self.assertEqual(plan.scope_metadata['frame_id'],'map');self.assertEqual(built.frame_id,'map')
        self.admit(plan)
        explicit=source.model_copy(update={'frame_id':'map'})
        same=self.plan(service,snapshot=explicit)
        self.assertEqual(same.resolve('scene','region','coarse_n'),built.scene_spaces[0].scene_space_id)
        other=source.model_copy(update={'frame_id':'odom'})
        different=self.plan(service,snapshot=other)
        second=builder.build(other,map_name='library',identity_resolver=different.resolve)
        self.assertEqual(second.frame_id,'odom');self.assertEqual(different.scope_metadata['frame_id'],'odom')
        self.assertNotEqual(second.scene_spaces[0].scene_space_id,built.scene_spaces[0].scene_space_id)

    def test_library_retirement_and_recreation_fence_old_plan(self):
        service=self.fixture();old=self.plan(service);old.resolve('anchor','region','r');self.admit(old)
        pending=self.plan(service);pending.resolve('scene','region','new')
        service._durable_index.retire_library('library');service._durable_index.activate('library')
        with self.assertRaises(IdentityScopeChanged):self.admit(pending)
        fresh=self.plan(service)
        self.assertNotEqual(old.library_incarnation,fresh.library_incarnation)
        with fresh.storage.read_snapshot() as tx:
            self.assertEqual(tx.execute('SELECT COUNT(*) FROM memory_scene_identity_scopes').fetchone()[0],0)
            self.assertEqual(tx.execute('SELECT COUNT(*) FROM memory_scene_identity_publication_scenes').fetchone()[0],0)

    def test_two_plans_converge_via_provisional_remap_without_partial_aliases(self):
        service=self.fixture();a=self.plan(service);b=self.plan(service)
        winner=a.resolve('scene','region','r');loser=b.resolve('scene','region','r')
        b.resolve('anchor','region','other');self.admit(a)
        with self.assertRaises(IdentityPlanChanged) as error:self.admit(b)
        self.assertEqual(error.exception.remap,{loser:winner})
        with b.storage.read_snapshot() as tx:
            self.assertEqual(tx.execute('SELECT COUNT(*) FROM memory_scene_entity_ids').fetchone()[0],1)
        b.refresh(error.exception.remap);self.admit(b)
        self.assertEqual(b.resolve('scene','region','r'),winner)

    def test_concurrent_library_owners_admit_one_winner(self):
        with tempfile.TemporaryDirectory() as directory:
            path=str(Path(directory)/'memory.db');a=self.fixture(path=path);b=self.fixture(path=path)
            plans=[self.plan(a),self.plan(b)];ids=[plan.resolve('scene','region','r') for plan in plans]
            barrier=threading.Barrier(2)
            def run(plan):
                barrier.wait(timeout=3)
                try:self.admit(plan)
                except IdentityPlanChanged as error:
                    plan.refresh(error.remap);self.admit(plan)
                return plan.resolve('scene','region','r')
            with ThreadPoolExecutor(max_workers=2) as pool:results=list(pool.map(run,plans))
            self.assertEqual(results[0],results[1]);self.assertIn(results[0],ids)

    def test_source_failure_rolls_back_scope_aliases_and_publication(self):
        service=self.fixture();plan=self.plan(service);plan.resolve('anchor','region','r')
        with self.assertRaisesRegex(RuntimeError,'source failed'),plan.storage.transaction() as tx:
            token=plan.admit(tx)
            self.assertTrue(plan.is_current(token))
            raise RuntimeError('source failed')
        self.assertFalse(plan.is_current(token))
        with plan.storage.read_snapshot() as tx:
            self.assertEqual(tx.execute('SELECT COUNT(*) FROM memory_scene_entity_ids').fetchone()[0],0)
            self.assertEqual(tx.execute('SELECT COUNT(*) FROM memory_scene_identity_scopes').fetchone()[0],0)
            self.assertEqual(tx.execute('SELECT COUNT(*) FROM memory_scene_identity_publication_scenes').fetchone()[0],0)

    def test_publication_token_rejects_delayed_same_scope_context(self):
        service=self.fixture();a=self.plan(service);a.resolve('scene','region','r');first=self.admit(a)
        b=self.plan(service);b.resolve('scene','region','r');second=self.admit(b)
        self.assertFalse(a.is_current(first));self.assertTrue(b.is_current(second))
        with self.assertRaises(RuntimeError),b.storage.transaction() as tx:
            failed=b.admit(tx);raise RuntimeError()
        self.assertTrue(b.is_current(second));self.assertFalse(b.is_current(failed))

    def test_warm_publication_validation_is_one_scalar_query(self):
        from services.memory.sqlite_storage import SQLiteTransaction
        service=self.fixture();plan=self.plan(service);plan.resolve('scene','region','r');token=self.admit(plan)
        statements=[];original=SQLiteTransaction.execute
        def execute(tx,sql,parameters=()):
            statements.append(sql)
            return original(tx,sql,parameters)
        with patch.object(SQLiteTransaction,'execute',execute):
            self.assertTrue(plan.is_current(token))
        self.assertEqual(len(statements),1)
        self.assertNotIn('sqlite_master',statements[0]);self.assertNotIn('payload_json',statements[0])

    def test_rolled_back_scope_id_is_never_reused_as_current(self):
        service=self.fixture();plan=self.plan(service);plan.resolve('scene','region','r')
        with self.assertRaises(RuntimeError),plan.storage.transaction() as tx:
            old=plan.admit(tx);raise RuntimeError()
        self.assertFalse(plan.is_current(old))
        new=self.admit(plan)
        self.assertTrue(plan.is_current(new));self.assertFalse(plan.is_current(old))

    def test_unrelated_memory_and_instance_writes_do_not_block_new_ids(self):
        service=self.fixture();plan=self.plan(service);plan.resolve('scene','region','new')
        service._repository.upsert_semantic_memory(records()[1][3]);service._repository.upsert_semantic_instance(records()[2][3])
        self.admit(plan)

    def test_uniquely_scoped_legacy_scene_and_anchor_are_adopted_without_rewriting(self):
        service=self.fixture();old=self.save_scene(service)
        before=old.model_dump_json();plan=self.plan(service)
        self.assertEqual(plan.resolve('scene','region','region-a'),SCENE)
        self.assertEqual(plan.resolve('anchor','region','region-a'),ANCHOR)
        self.admit(plan)
        self.assertEqual(service._repository.load_scene_space(SCENE).model_dump_json(),before)

    def test_topology_anchor_and_coarse_scene_keep_different_natural_kinds(self):
        service=self.fixture();topo='anc_topo_19700101T000000Z_abcdef01'
        coarse=self.save_scene(service,source='coarse_node',anchor=topo)
        coarse.linked_topology_node_ids=['node'];service._repository.upsert_scene_space(coarse)
        plan=self.plan(service)
        self.assertEqual(plan.resolve('scene','region','coarse_node'),SCENE)
        self.assertEqual(plan.resolve('anchor','topo','node'),topo)
        self.assertNotEqual(plan.resolve('anchor','region','coarse_node'),topo)

    def test_map_name_alone_and_other_epoch_never_authorize_legacy_adoption(self):
        service=self.fixture();self.save_scene(service,scope=False)
        plan=self.plan(service)
        self.assertNotEqual(plan.resolve('scene','region','region-a'),SCENE)
        self.assertTrue(any(item['status']=='unbound' for item in plan.diagnostics))
        changed=service._repository.load_scene_space(SCENE)
        changed.metadata['scene_identity_scope']={**plan.scope_metadata,'source_epoch':'different'}
        service._repository.upsert_scene_space(changed)
        self.assertNotEqual(self.plan(service).resolve('scene','region','region-a'),SCENE)

    def test_ambiguous_legacy_candidates_and_overflow_are_preserved(self):
        service=self.fixture();self.save_scene(service)
        self.save_scene(service,identifier=SCENE.replace('deadbeef','00000001'))
        plan=self.plan(service);new=plan.resolve('scene','region','region-a')
        self.assertNotIn(new,[SCENE,SCENE.replace('deadbeef','00000001')])
        self.assertTrue(any(item['status']=='ambiguous' for item in plan.diagnostics));self.admit(plan)
        self.assertEqual(len(service._repository.load_scene_spaces()),2)
        with patch('services.memory.scene_identity.MAX_CANDIDATES',1):
            bounded=self.plan(service);bounded.resolve('anchor','region','region-a')
            self.assertTrue(any(item['status']=='overflow' for item in bounded.diagnostics))

    def test_specific_adopted_witness_change_is_rejected(self):
        service=self.fixture();old=self.save_scene(service);plan=self.plan(service)
        plan.resolve('scene','region','region-a')
        service._repository.upsert_scene_space(old.model_copy(update={'primary_label':'changed source'}))
        with self.assertRaises(IdentityScopeChanged):self.admit(plan)

    def test_new_relevant_legacy_witness_cannot_make_pending_adoption_ambiguous(self):
        service=self.fixture();self.save_scene(service);plan=self.plan(service)
        self.assertEqual(plan.resolve('scene','region','region-a'),SCENE)
        self.save_scene(service,identifier=SCENE.replace('deadbeef','abcdef02'))
        with self.assertRaises(IdentityScopeChanged):self.admit(plan)

    def test_stored_scene_conflict_is_not_remapped_to_a_registered_id(self):
        service=self.fixture();first=self.plan(service)
        canonical=first.resolve('scene','region','region-a');self.admit(first)
        self.save_scene(service)
        conflicted=self.plan(service)
        self.assertEqual(conflicted.resolve('scene','region','region-a'),SCENE)
        self.admit(conflicted)
        self.assertTrue(any(item['status']=='ambiguous' for item in conflicted.diagnostics))
        with conflicted.storage.read_snapshot() as tx:
            self.assertEqual(tx.execute('SELECT public_id FROM memory_scene_entity_ids').fetchone()[0],canonical)
        self.assertIsNotNone(service._repository.load_scene_space(SCENE))

    def test_adoption_reads_only_bounded_identity_fields_not_scene_geometry(self):
        import services.memory.scene_identity as module
        service=self.fixture();old=self.save_scene(service)
        old.metadata['irrelevant_large_field']='x'*100000
        service._repository.upsert_scene_space(old)
        loads=module.json.loads
        seen=[]
        def capture(value,*args,**kwargs):
            seen.append(len(value));return loads(value,*args,**kwargs)
        with patch.object(module.json,'loads',side_effect=capture):
            plan=self.plan(service);self.assertEqual(plan.resolve('scene','region','region-a'),SCENE);self.admit(plan)
        self.assertTrue(seen);self.assertLess(max(seen),2000)

    def test_samples_seed_existing_ids_without_resolver_and_survive_selection_loss(self):
        service=self.fixture();existing=SceneObservationSample(sample_id='sample_legacy',observation_event_id=EVENT,
            metadata={'source_record_kind':'observation_event'})
        owner=self.save_scene(service,samples=[existing])
        plan=service._repository.prepare_scene_identities('library',scene=owner);self.admit(plan)
        # A later bounded selection drops the sample, but its alias remains.
        owner=service._repository.upsert_scene_space(owner.model_copy(update={'observation_samples':[]}))
        next_plan=service._repository.prepare_scene_identities('library',scene=owner)
        self.assertEqual(next_plan.resolve_sample('observation_event',EVENT),'sample_legacy')

    def test_over_1024_legacy_sample_ids_survive_window_eviction(self):
        from services.memory.scene_space_sampler import SceneSpaceSampler
        from services.memory.sqlite_storage import SQLiteTransaction
        service=self.fixture()
        samples=[SceneObservationSample(sample_id='legacy_sample_'+str(index),
            observation_event_id='oev_fixture_20260101T000000Z_'+format(index,'08x'),
            metadata={'source_record_kind':'observation_event'}) for index in range(1031)]
        owner=self.save_scene(service,samples=samples)
        source_reads=[];original=SQLiteTransaction.execute
        def execute(tx,sql,parameters=()):
            if 'FROM scene_spaces' in sql:source_reads.append(sql)
            return original(tx,sql,parameters)
        with patch.object(SQLiteTransaction,'execute',execute):
            plan=service._repository.prepare_scene_identities('library',scene=owner)
        self.assertEqual(len(source_reads),1)
        sampled=SceneSpaceSampler().sample(owner,identity_resolver=plan.resolve_sample)
        self.assertEqual(len(sampled.samples),8)
        with service._repository.storage.transaction() as tx:
            plan.admit(tx)
            service._repository.upsert_scene_space(sampled.scene_space)
        owner=service._repository.load_scene_space(owner.scene_space_id)
        self.assertEqual(len(owner.observation_samples),8)
        later=service._repository.prepare_scene_identities('library',scene=owner)
        for sample in samples:
            self.assertEqual(later.resolve_sample('observation_event',sample.observation_event_id),sample.sample_id)
        with service._repository.storage.read_snapshot() as tx:
            self.assertEqual(tx.execute('SELECT COUNT(*) FROM memory_scene_sample_ids').fetchone()[0],len(samples))

    def test_instance_sample_identity_uses_instance_id_not_last_event(self):
        service=self.fixture();existing=SceneObservationSample(sample_id='sample_instance',observation_event_id=EVENT,
            metadata={'source_record_kind':'semantic_instance','source_instance_id':INSTANCE})
        owner=self.save_scene(service,samples=[existing])
        plan=service._repository.prepare_scene_identities('library',scene=owner)
        self.assertEqual(plan.resolve_sample('semantic_instance',INSTANCE),'sample_instance')
        self.assertNotEqual(plan.resolve_sample('observation_event',EVENT),'sample_instance')
        self.admit(plan)

    def test_sample_owner_source_change_requires_new_plan(self):
        service=self.fixture();owner=self.save_scene(service)
        plan=service._repository.prepare_scene_identities('library',scene=owner);plan.resolve_sample('observation_event',EVENT)
        service._repository.upsert_scene_space(owner.model_copy(update={'primary_label':'changed'}))
        with self.assertRaises(IdentityScopeChanged):self.admit(plan)
        with self.assertRaises(IdentityScopeChanged):
            service._repository.prepare_scene_identities('library',scene=owner)

    def test_sample_ids_are_scoped_to_their_owning_scene(self):
        service=self.fixture();a=self.save_scene(service);b=self.save_scene(service,identifier=SCENE.replace('deadbeef','00000002'),source='b')
        ap=service._repository.prepare_scene_identities('library',scene=a);bp=service._repository.prepare_scene_identities('library',scene=b)
        ai=ap.resolve_sample('observation_event',EVENT);bi=bp.resolve_sample('observation_event',EVENT)
        self.assertNotEqual(ai,bi);self.admit(ap);self.admit(bp)

    def test_stored_sample_conflict_keeps_legacy_data_and_reports_ambiguity(self):
        service=self.fixture();owner=self.save_scene(service)
        first=service._repository.prepare_scene_identities('library',scene=owner)
        canonical=first.resolve_sample('observation_event',EVENT);self.admit(first)
        legacy=SceneObservationSample(sample_id='stored-original',observation_event_id=EVENT,metadata={'source_record_kind':'observation_event'})
        owner=service._repository.upsert_scene_space(owner.model_copy(update={'observation_samples':[legacy]}))
        conflict=service._repository.prepare_scene_identities('library',scene=owner)
        self.assertEqual(conflict.resolve_sample('observation_event',EVENT),'stored-original')
        self.admit(conflict)
        self.assertTrue(any(item['status']=='ambiguous' for item in conflict.diagnostics))
        self.assertEqual(service._repository.load_scene_space(SCENE).observation_samples[0].sample_id,'stored-original')
        with conflict.storage.read_snapshot() as tx:
            self.assertEqual(tx.execute('SELECT public_id FROM memory_scene_sample_ids').fetchone()[0],canonical)

    def test_scene_deletion_cleans_scene_and_sample_aliases_but_keeps_map_anchors(self):
        service=self.fixture();owner=self.save_scene(service)
        plan=self.plan(service);plan.resolve('scene','region','region-a');plan.resolve('anchor','region','region-a');token=self.admit(plan)
        sample=service._repository.prepare_scene_identities('library',scene=owner);sample.resolve_sample('observation_event',EVENT);self.admit(sample)
        service._repository.delete_scene_spaces([SCENE])
        self.assertFalse(plan.is_current(token))
        with plan.storage.read_snapshot() as tx:
            self.assertEqual(tx.execute('SELECT entity_kind FROM memory_scene_entity_ids').fetchall()[0][0],'anchor')
            self.assertEqual(tx.execute('SELECT COUNT(*) FROM memory_scene_sample_ids').fetchone()[0],0)
        with self.assertRaises(IdentityScopeChanged):self.admit(sample)

    def test_preserved_conflict_scene_deletion_invalidates_its_publication(self):
        service=self.fixture();canonical=self.plan(service)
        canonical_id=canonical.resolve('scene','region','region-a');self.admit(canonical)
        self.save_scene(service)
        conflict=self.plan(service)
        self.assertEqual(conflict.resolve('scene','region','region-a'),SCENE)
        token=self.admit(conflict)
        self.assertTrue(conflict.is_current(token))
        service._repository.delete_scene_spaces([SCENE])
        self.assertFalse(conflict.is_current(token))
        with conflict.storage.read_snapshot() as tx:
            self.assertEqual(tx.execute('SELECT public_id FROM memory_scene_entity_ids').fetchone()[0],canonical_id)
            self.assertEqual(tx.execute('SELECT COUNT(*) FROM memory_scene_identity_publication_scenes').fetchone()[0],0)

    def test_preserved_conflict_still_fences_its_specific_legacy_witness(self):
        service=self.fixture();canonical=self.plan(service)
        canonical.resolve('scene','region','region-a');self.admit(canonical)
        source=self.save_scene(service)
        conflict=self.plan(service)
        self.assertEqual(conflict.resolve('scene','region','region-a'),SCENE)
        service._repository.upsert_scene_space(source.model_copy(update={'linked_semantic_region_ids':['moved']}))
        with self.assertRaises(IdentityScopeChanged):self.admit(conflict)

    def test_deletion_does_not_invalidate_unrelated_map_scope_publication(self):
        service=self.fixture();self.save_scene(service)
        affected=self.plan(service);affected.resolve('scene','region','region-a');old=self.admit(affected)
        other=self.plan(service,snapshot=snapshot().model_copy(update={'source_epoch':'other'}))
        other.resolve('scene','region','region-a');current=self.admit(other)
        service._repository.delete_scene_spaces([SCENE])
        self.assertFalse(affected.is_current(old));self.assertTrue(other.is_current(current))

    def test_clear_all_retires_aliases_and_even_unadmitted_plans(self):
        service=self.fixture();old=self.plan(service);old.resolve('anchor','region','a');token=self.admit(old)
        pending=self.plan(service);pending.resolve('scene','region','not-yet-admitted')
        service._repository.clear_all()
        self.assertFalse(old.is_current(token))
        with self.assertRaises(IdentityScopeChanged):self.admit(pending)
        with pending.storage.read_snapshot() as tx:
            for table in ('memory_scene_identity_scopes','memory_scene_entity_ids','memory_scene_sample_ids',
                          'memory_scene_identity_publication_scenes'):
                self.assertEqual(tx.execute('SELECT COUNT(*) FROM '+table).fetchone()[0],0)

    def test_retired_registered_scene_alias_cannot_be_resurrected_by_old_plan(self):
        service=self.fixture();self.save_scene(service);a=self.plan(service);a.resolve('scene','region','region-a');self.admit(a)
        b=self.plan(service);b.resolve('scene','region','region-a')
        service._repository.delete_scene_spaces([SCENE])
        with self.assertRaises(IdentityScopeChanged):self.admit(b)

    def test_owner_transaction_is_required_and_refresh_is_outside_writer(self):
        service=self.fixture();plan=self.plan(service);plan.resolve('scene','region','r')
        other=self.fixture()
        with other._repository.storage.transaction() as tx, self.assertRaises(ValueError):plan.admit(tx)
        with plan.storage.transaction(), self.assertRaises(ValueError):plan.refresh({})


if __name__=='__main__':unittest.main(verbosity=2)
