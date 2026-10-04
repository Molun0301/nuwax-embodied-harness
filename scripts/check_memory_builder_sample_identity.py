#!/usr/bin/env python3
"""Offline resolver contracts for durable map entities and observation samples."""
from __future__ import annotations

import ast
import sys
import threading
import unittest
import uuid
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timedelta, timezone
from pathlib import Path
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from contracts.geometry import Pose, Vector3
from contracts.maps import SemanticMap
from contracts.maps import SemanticRegion as MapRegion
from contracts.naming import build_anchor_id, build_scene_space_id, validate_anchor_id, validate_scene_space_id
from contracts.runtime_views import MapSnapshot
from contracts.spatial_memory import (
    ObservationEvent,
    SceneObservationSample,
    SceneSpace,
    SceneSpaceBoundary,
    SemanticInstance,
)
from services.memory.scene_space_sampler import SceneSpaceSampler
from services.memory.semantic_map_builder import SemanticMapBuilder

NOW = datetime(2026, 1, 1, tzinfo=timezone.utc)
ANCHOR = 'anc_region_19700101T000000Z_deadbeef'
SCENE = 'ssp_region_19700101T000000Z_deadbeef'
EVENT = 'oev_fixture_20260101T000000Z_00000001'
INSTANCE = 'ins_fixture_20260101T000000Z_00000001'


def snapshot(*, nodes=(), regions=None):
    semantic = None if regions is None else SemanticMap(map_id='library', frame_id='map', regions=list(regions))
    return MapSnapshot(map_name='library', source_name='owner', source_epoch='persisted-map',
        update_sequence=1, frame_id='map', semantic_map=semantic, metadata={'topology_nodes': list(nodes)})


def region(identifier='Room:房间', **kwargs):
    return MapRegion(region_id=identifier, label='office', centroid=Pose(frame_id='map'), **kwargs)


def resolver():
    calls, identities = [], {}
    def resolve(entity, kind, key):
        calls.append((entity, kind, key))
        factory = build_anchor_id if entity == 'anchor' else build_scene_space_id
        return identities.setdefault((entity, kind, key), factory(kind, suffix=uuid.uuid4().hex))
    return resolve, calls, identities


def scene(**kwargs):
    return SceneSpace(scene_space_id=SCENE, frame_id='map',
        boundary=SceneSpaceBoundary(frame_id='map', centroid=Pose(frame_id='map'),
            min_corner=Vector3(x=-1, y=-1), max_corner=Vector3(x=1, y=1)), **kwargs)


def event(**kwargs):
    return ObservationEvent(event_id=EVENT, title='desk', summary='evidence', pose=Pose(frame_id='map'),
                            timestamp=NOW, **kwargs)


def instance(**kwargs):
    return SemanticInstance(instance_id=INSTANCE, anchor_id=ANCHOR, label='desk', pose=Pose(frame_id='map'),
                            first_seen_ts=NOW, last_seen_ts=NOW, **kwargs)


class BuilderIdentityChecks(unittest.TestCase):
    def setUp(self):
        self.builder = SemanticMapBuilder(inspection_pose_planner=Mock(plan_for_target=lambda **kwargs: []))

    def test_semantic_and_topology_inventory_preserves_source_ids(self):
        resolve, calls, ids = resolver()
        source = snapshot(nodes=[{'node_id': 'A:B', 'x': 0, 'y': 0}, {'node_id': 'A/中', 'x': 5, 'y': 0}],
            regions=[region(attributes={'topo_node_ids': ['A:B']})])
        built = self.builder.build(source, map_name='library', identity_resolver=resolve)
        self.assertEqual(set(calls), {('anchor','topo','A:B'), ('anchor','topo','A/中'),
            ('anchor','region','Room:房间'), ('scene','region','Room:房间'), ('scene','topo','A/中')})
        self.assertEqual(built.semantic_regions[0].region_id, 'Room:房间')
        self.assertEqual(set(built.topology_nodes_by_id), {'A:B','A/中'})
        self.assertEqual(built.semantic_regions[0].anchor_id, ids['anchor','region','Room:房间'])
        by_id = {space.scene_space_id: space for space in built.scene_spaces}
        self.assertEqual(by_id[ids['scene','region','Room:房间']].linked_anchor_ids,
                         [ids['anchor','region','Room:房间'], ids['anchor','topo','A:B']])
        for space in built.scene_spaces:
            self.assertTrue(set(space.linked_anchor_ids) <= set(built.anchors_by_id))
        self.assertEqual(source.metadata['topology_nodes'][0].get('anchor_id'), None)

    def test_coarse_topology_scene_is_region_key_but_anchor_is_topology(self):
        resolve, calls, ids = resolver()
        built = self.builder.build(snapshot(nodes=[{'id':'node:1','label':'Office','x':0,'y':0}]), identity_resolver=resolve)
        self.assertEqual(calls, [('anchor','topo','node:1'), ('scene','region','coarse_node:1')])
        self.assertEqual(built.semantic_regions[0].region_id, 'coarse_node:1')
        self.assertEqual(built.scene_spaces[0].linked_anchor_ids, [ids['anchor','topo','node:1']])
        self.assertEqual(built.scene_spaces[0].scene_space_id, ids['scene','region','coarse_node:1'])

    def test_explicit_topology_anchor_is_preserved_and_never_resolved(self):
        resolve, calls, _ = resolver()
        built = self.builder.build(snapshot(nodes=[{'node_id':'n','label':'Office','anchor_id':ANCHOR,'x':0,'y':0}]), identity_resolver=resolve)
        self.assertEqual(calls, [('scene','region','coarse_n')])
        self.assertIn(ANCHOR, built.anchors_by_id)
        self.assertEqual(built.topology_nodes_by_id['n']['anchor_id'], ANCHOR)
        self.assertEqual(built.scene_spaces[0].linked_anchor_ids, [ANCHOR])

    def test_missing_geometry_still_requests_anchor_but_not_scene(self):
        resolve, calls, _ = resolver()
        built = self.builder.build(snapshot(nodes=[{'node_id':'missing'}], regions=[MapRegion(region_id='no-center',label='room')]), identity_resolver=resolve)
        self.assertEqual(set(calls), {('anchor','topo','missing'), ('anchor','region','no-center')})
        self.assertFalse(built.scene_spaces)
        self.assertFalse(built.anchors_by_id)

    def test_empty_semantic_map_uses_topology_scene_not_coarse_region(self):
        resolve, calls, _ = resolver()
        built = self.builder.build(snapshot(nodes=[{'node_id':'n','label':'Office','x':0,'y':0}], regions=[]), identity_resolver=resolve)
        self.assertEqual(calls, [('anchor','topo','n'), ('scene','topo','n')])
        self.assertFalse(built.semantic_regions)

    def test_duplicate_keys_call_even_unstable_resolver_only_once_per_build(self):
        calls = []
        def unstable(entity, kind, key):
            calls.append((entity, kind, key))
            return (build_anchor_id if entity == 'anchor' else build_scene_space_id)(kind, suffix=uuid.uuid4().hex)
        built = self.builder.build(snapshot(nodes=[{'node_id':'n','x':0,'y':0}]*2,
            regions=[region(attributes={'topo_node_ids':['n']})]*2), identity_resolver=unstable)
        self.assertEqual(len(calls), len(set(calls)))
        self.assertEqual(built.semantic_regions[0].anchor_id, built.semantic_regions[1].anchor_id)
        self.assertEqual(len(built.scene_spaces), 1)

    def test_standalone_ids_have_full_entropy_and_only_per_build_lifetime(self):
        source = snapshot(regions=[region()])
        first, second = self.builder.build(source), self.builder.build(source)
        self.assertNotEqual(first.scene_spaces[0].scene_space_id, second.scene_spaces[0].scene_space_id)
        self.assertNotEqual(first.semantic_regions[0].anchor_id, second.semantic_regions[0].anchor_id)
        for identifier in [first.scene_spaces[0].scene_space_id, first.semantic_regions[0].anchor_id]:
            self.assertEqual(len(identifier.rsplit('_',1)[1]), 32)
        self.assertEqual(first.scene_spaces[0].linked_anchor_ids, [first.semantic_regions[0].anchor_id])

    def test_same_injected_mapping_reuses_ids_across_builder_instances(self):
        resolve, _, _ = resolver()
        source = snapshot(regions=[region()])
        a = self.builder.build(source, identity_resolver=resolve)
        b = SemanticMapBuilder().build(source, identity_resolver=resolve)
        self.assertEqual(a.scene_spaces[0].scene_space_id, b.scene_spaces[0].scene_space_id)
        self.assertEqual(a.semantic_regions[0].anchor_id, b.semantic_regions[0].anchor_id)

    def test_concurrent_shared_builder_has_no_resolver_state_leak(self):
        barrier = threading.Barrier(2)
        def build(suffix):
            def resolve(entity, kind, key):
                barrier.wait(timeout=2)
                return f'{"anc" if entity == "anchor" else "ssp"}_{kind}_20260101T000000Z_{suffix * 32}'
            return self.builder.build(snapshot(regions=[region()]), identity_resolver=resolve)
        with ThreadPoolExecutor(max_workers=2) as pool:
            a, b = list(pool.map(build, ['a','b']))
        self.assertTrue(a.semantic_regions[0].anchor_id.endswith('a'*32))
        self.assertTrue(b.semantic_regions[0].anchor_id.endswith('b'*32))
        self.assertTrue(a.scene_spaces[0].scene_space_id.endswith('a'*32))
        self.assertTrue(b.scene_spaces[0].scene_space_id.endswith('b'*32))

    def test_empty_snapshot_never_calls_resolver(self):
        resolve = Mock(side_effect=AssertionError('unexpected identity'))
        self.assertFalse(self.builder.build(None, identity_resolver=resolve).scene_spaces)
        resolve.assert_not_called()

    def test_anchor_and_scene_contracts_accept_legacy_and_full_uuid_only(self):
        for prefix, validate in [('anc',validate_anchor_id), ('ssp',validate_scene_space_id)]:
            for suffix in ('deadbeef',uuid.uuid4().hex):
                identifier=f'{prefix}_region_19700101T000000Z_{suffix}'
                self.assertEqual(validate(identifier),identifier)
            for suffix in ('a'*7, 'a'*9, 'a'*31, 'a'*33, 'z'*32):
                with self.assertRaises(ValueError):validate(f'{prefix}_region_19700101T000000Z_{suffix}')


class SampleIdentityChecks(unittest.TestCase):
    def setUp(self):
        self.sampler = SceneSpaceSampler()

    def test_resolver_uses_typed_event_and_instance_keys(self):
        calls = []
        def resolve(kind, key):
            calls.append((kind,key))
            return 'sample_'+kind
        result = self.sampler.sample(scene(), observation_events=[event()],
            semantic_instances=[instance(last_observation_event_id=EVENT)], identity_resolver=resolve)
        self.assertEqual(calls,[('observation_event',EVENT),('semantic_instance',INSTANCE)])
        self.assertEqual([sample.sample_id for sample in result.samples], ['sample_observation_event'])

    def test_existing_event_sample_keeps_actual_legacy_id_without_resolver(self):
        existing = SceneObservationSample(sample_id='sample_deadbeef', observation_event_id=EVENT,
            confidence=.1, timestamp=NOW, metadata={'source_record_kind':'observation_event'})
        resolve=Mock(side_effect=AssertionError('existing ID replacement'))
        result=self.sampler.sample(scene(observation_samples=[existing]),observation_events=[event()],identity_resolver=resolve)
        self.assertEqual([sample.sample_id for sample in result.samples], ['sample_deadbeef'])
        resolve.assert_not_called()

    def test_instance_identity_uses_instance_not_last_event_and_deduplicates_updates(self):
        existing=SceneObservationSample(sample_id='legacy-instance-sample',observation_event_id=EVENT,
            confidence=.1,timestamp=NOW,metadata={'source_record_kind':'semantic_instance','source_instance_id':INSTANCE})
        newer=instance(last_observation_event_id=EVENT.replace('00000001','00000002')).model_copy(update={'last_seen_ts':NOW+timedelta(seconds=1)})
        resolve=Mock(side_effect=AssertionError('existing ID replacement'))
        result=self.sampler.sample(scene(observation_samples=[existing]),semantic_instances=[newer],identity_resolver=resolve)
        self.assertEqual(len(result.samples),1)
        self.assertEqual(result.samples[0].sample_id,existing.sample_id)
        self.assertEqual(result.samples[0].observation_event_id,newer.last_observation_event_id)

    def test_legacy_instance_provenance_beats_last_event_even_without_source_kind(self):
        existing=SceneObservationSample(sample_id='legacy-instance',observation_event_id=EVENT,
            confidence=.1,metadata={'source_instance_id':INSTANCE})
        result=self.sampler.sample(scene(observation_samples=[existing]),semantic_instances=[instance()])
        self.assertEqual(result.samples[0].sample_id,'legacy-instance')

    def test_legacy_event_provenance_without_kind_is_reused(self):
        existing=SceneObservationSample(sample_id='legacy-event',observation_event_id=EVENT,confidence=.1)
        result=self.sampler.sample(scene(observation_samples=[existing]),observation_events=[event()])
        self.assertEqual(result.samples[0].sample_id,'legacy-event')

    def test_unknown_or_incomplete_kind_does_not_adopt_event_as_instance_identity(self):
        for metadata in ({'source_record_kind':'semantic_instance'}, {'source_record_kind':'unknown'}):
            with self.subTest(metadata=metadata):
                existing=SceneObservationSample(sample_id='unresolved',observation_event_id=EVENT,confidence=.1,metadata=metadata)
                resolve=Mock(return_value='resolved-event')
                result=self.sampler.sample(scene(observation_samples=[existing]),observation_events=[event()],identity_resolver=resolve)
                resolve.assert_called_once_with('observation_event',EVENT)
                self.assertIn('resolved-event',[sample.sample_id for sample in result.samples])
                self.assertEqual(existing.sample_id, 'unresolved')

    def test_ambiguous_existing_ids_are_not_arbitrarily_adopted(self):
        existing=[SceneObservationSample(sample_id=identifier,observation_event_id=EVENT,confidence=.1,
            metadata={'source_record_kind':'observation_event'}) for identifier in ('legacy-a','legacy-b')]
        resolve=Mock(return_value='canonical-from-owner')
        result=self.sampler.sample(scene(observation_samples=existing),observation_events=[event()],identity_resolver=resolve)
        resolve.assert_called_once_with('observation_event',EVENT)
        self.assertEqual(result.samples[0].sample_id,'canonical-from-owner')
        self.assertEqual([sample.sample_id for sample in existing],['legacy-a','legacy-b'])

    def test_repeated_source_calls_resolver_once_and_emits_one_sample(self):
        resolve=Mock(side_effect=lambda *args:'sample_'+uuid.uuid4().hex)
        result=self.sampler.sample(scene(),observation_events=[event(),event()],identity_resolver=resolve)
        self.assertEqual(len(result.samples),1)
        resolve.assert_called_once_with('observation_event',EVENT)

    def test_standalone_sample_ids_reuse_persisted_evidence_and_new_ids_are_full_uuid(self):
        first=self.sampler.sample(scene(),observation_events=[event()])
        identifier=first.samples[0].sample_id
        self.assertEqual(len(identifier[len('sample_'):]),32)
        second=SceneSpaceSampler().sample(first.scene_space,observation_events=[event()])
        self.assertEqual(second.samples[0].sample_id,identifier)
        unrelated=self.sampler.sample(scene(),observation_events=[event()])
        self.assertNotEqual(unrelated.samples[0].sample_id,identifier)

    def test_no_coverage_never_allocates_sample_identity(self):
        resolve=Mock(side_effect=AssertionError('no sample needed'))
        distant=event().model_copy(update={'pose':None})
        result=self.sampler.sample(scene(),observation_events=[distant],identity_resolver=resolve)
        self.assertFalse(result.samples)

    def test_shared_sampler_concurrent_calls_have_independent_resolvers(self):
        barrier=threading.Barrier(2)
        def run(identifier):
            def resolve(*args):
                barrier.wait(timeout=2)
                return identifier
            return self.sampler.sample(scene(),observation_events=[event()],identity_resolver=resolve)
        with ThreadPoolExecutor(max_workers=2) as pool:
            a,b=list(pool.map(run,['sample-a','sample-b']))
        self.assertEqual(a.samples[0].sample_id,'sample-a')
        self.assertEqual(b.samples[0].sample_id,'sample-b')

    def test_resolver_errors_propagate_without_mutating_input(self):
        original=scene()
        with self.assertRaisesRegex(RuntimeError,'ownership conflict'):
            self.sampler.sample(original,observation_events=[event()],identity_resolver=Mock(side_effect=RuntimeError('ownership conflict')))
        self.assertFalse(original.observation_samples)


class HashRemovalChecks(unittest.TestCase):
    def test_owned_producers_import_no_hash_library_and_have_no_digest_calls(self):
        root=Path(__file__).resolve().parents[1]
        for relative in ('services/memory/semantic_map_builder.py','services/memory/scene_space_sampler.py'):
            tree=ast.parse((root/relative).read_text())
            self.assertFalse(any(isinstance(node,ast.Import) and any(alias.name=='hashlib' for alias in node.names) for node in ast.walk(tree)))
            self.assertFalse(any(isinstance(node,ast.Attribute) and node.attr in ('hexdigest','digest','blake2b','sha256','sha1','md5') for node in ast.walk(tree)))


if __name__=='__main__':
    unittest.main(verbosity=2)
