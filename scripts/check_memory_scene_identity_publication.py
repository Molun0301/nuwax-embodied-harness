#!/usr/bin/env python3
"""Independent offline map-owner publication and transaction-local evidence checks."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scripts import check_memory_builder_sample_identity as producer
from scripts import check_memory_provider_modes as providers
from services.memory.service import MemoryService


class SceneIdentityPublicationChecks(unittest.TestCase):
    def fixture(self):
        service = providers.ProviderModeChecks.fixture(self)
        service._localization_service = SimpleNamespace(get_active_session=lambda: None)
        source = producer.snapshot(regions=[producer.region('r')])
        owner = SimpleNamespace(latest=source)
        owner.get_latest_snapshot = lambda: owner.latest
        owner.get_semantic_snapshot = lambda: owner.latest
        service._mapping_service = owner
        service._semantic_map_builder._inspection_pose_planner = Mock(plan_for_target=lambda **kw: [])
        # Identity and publication are independent of derived summary work or
        # a model. Keep these tests strictly about the source/alias boundary.
        service._record_scene_space_graph = Mock()
        service._scene_space_understanding_provider_metadata = lambda value: {}
        service._apply_scene_space_understanding = lambda value, **kw: value
        return service, owner

    @staticmethod
    def observation(service):
        value = producer.event(map_name='library', metadata={'memory_library_name': 'library'})
        service._repository.upsert_observation_event(value)
        return value

    @staticmethod
    def sample(service, observation):
        service._refresh_scene_spaces_from_observations(
            perception_context=SimpleNamespace(scene_summary=object()), observation_events=(observation,))

    @staticmethod
    def sample_alias_count(service):
        with service._repository.storage.read_snapshot() as tx:
            return tx.execute('SELECT COUNT(*) FROM memory_scene_sample_ids').fetchone()[0]

    def test_delayed_builder_callback_cannot_publish_after_owner_epoch_replacement(self):
        service, owner = self.fixture()
        before = service._latest_semantic_map_context
        with service._repository.storage.transaction():
            built = service._refresh_semantic_map_context(owner.latest)
            self.assertTrue(built.scene_spaces)
            owner.latest = owner.latest.model_copy(update={'source_epoch': 'replacement'})
        self.assertIs(service._latest_semantic_map_context, before)
        service._record_scene_space_graph.assert_not_called()
        self.assertEqual(len(service._repository.load_scene_spaces()), 1)

    def test_delayed_sample_callback_cannot_publish_after_owner_epoch_replacement(self):
        service, owner = self.fixture()
        before = service._refresh_semantic_map_context(owner.latest)
        observation = self.observation(service)
        service._record_scene_space_graph.reset_mock()
        with service._repository.storage.transaction():
            self.sample(service, observation)
            self.assertTrue(service._repository.load_scene_spaces()[0].observation_samples)
            owner.latest = owner.latest.model_copy(update={'source_epoch': 'replacement'})
        self.assertIs(service._latest_semantic_map_context, before)
        service._record_scene_space_graph.assert_not_called()

    def test_event_created_in_outer_writer_is_visible_to_sampling_and_rolls_back(self):
        service, owner = self.fixture()
        before = service._refresh_semantic_map_context(owner.latest)
        with self.assertRaisesRegex(RuntimeError, 'rollback'), service._repository.storage.transaction():
            observation = self.observation(service)
            self.sample(service, observation)
            current = service._repository.load_scene_spaces()[0]
            self.assertEqual([sample.observation_event_id for sample in current.observation_samples],
                             [observation.event_id])
            self.assertEqual(self.sample_alias_count(service), 1)
            self.assertIs(service._latest_semantic_map_context, before)
            raise RuntimeError('rollback')
        self.assertFalse(service._repository.load_scene_spaces()[0].observation_samples)
        self.assertFalse(service._repository.load_observation_events())
        self.assertEqual(self.sample_alias_count(service), 0)
        self.assertIs(service._latest_semantic_map_context, before)

    def test_initial_build_and_event_sample_share_outer_read_own_writes(self):
        service, owner = self.fixture()
        before = service._latest_semantic_map_context
        with service._repository.storage.transaction():
            context = service._refresh_semantic_map_context(owner.latest)
            observation = self.observation(service)
            self.sample(service, observation)
            stored = service._repository.load_scene_space(context.scene_spaces[0].scene_space_id)
            self.assertEqual([sample.observation_event_id for sample in stored.observation_samples],
                             [observation.event_id])
            self.assertEqual(self.sample_alias_count(service), 1)
            self.assertIs(service._latest_semantic_map_context, before)
        self.assertTrue(service._latest_semantic_map_context.scene_spaces[0].observation_samples)

    def test_outer_map_replacement_samples_new_scope_not_old_published_scope(self):
        service, owner = self.fixture()
        old = service._refresh_semantic_map_context(owner.latest)
        old_id = old.scene_spaces[0].scene_space_id
        owner.latest = owner.latest.model_copy(update={'source_epoch': 'replacement'})
        with service._repository.storage.transaction():
            new = service._refresh_semantic_map_context(owner.latest)
            observation = self.observation(service)
            self.sample(service, observation)
            self.assertFalse(service._repository.load_scene_space(old_id).observation_samples)
            self.assertTrue(service._repository.load_scene_space(new.scene_spaces[0].scene_space_id).observation_samples)
            self.assertIs(service._latest_semantic_map_context, old)
        self.assertEqual(service._latest_semantic_map_context.scene_spaces[0].scene_space_id,
                         new.scene_spaces[0].scene_space_id)

    def test_real_derived_graph_callback_does_not_overwrite_outer_sample_write(self):
        service, owner = self.fixture()
        service._record_scene_space_graph = MemoryService._record_scene_space_graph.__get__(service)
        with service._repository.storage.transaction():
            context = service._refresh_semantic_map_context(owner.latest)
            observation = self.observation(service)
            self.sample(service, observation)
            self.assertTrue(service._repository.load_scene_space(context.scene_spaces[0].scene_space_id).observation_samples)
        stored = service._repository.load_scene_space(context.scene_spaces[0].scene_space_id)
        self.assertEqual([sample.observation_event_id for sample in stored.observation_samples], [observation.event_id])
        self.assertTrue(service._latest_semantic_map_context.scene_spaces[0].observation_samples)
        self.assertFalse(service._repository.storage.notification_errors)

    def test_successful_nested_builder_context_remains_visible_in_outer_writer(self):
        service, owner = self.fixture()
        with service._repository.storage.transaction():
            with service._repository.storage.transaction():
                context = service._refresh_semantic_map_context(owner.latest)
            observation = self.observation(service)
            self.sample(service, observation)
            self.assertTrue(service._repository.load_scene_space(context.scene_spaces[0].scene_space_id).observation_samples)
        self.assertTrue(service._latest_semantic_map_context.scene_spaces[0].observation_samples)

    def test_inner_rollback_restores_prior_outer_context_for_sampling(self):
        service, owner = self.fixture()
        original = owner.latest
        with service._repository.storage.transaction():
            outer = service._refresh_semantic_map_context(owner.latest)
            with self.assertRaisesRegex(RuntimeError, 'inner rollback'), service._repository.storage.transaction():
                owner.latest = original.model_copy(update={'source_epoch': 'rolled-back'})
                service._refresh_semantic_map_context(owner.latest)
                raise RuntimeError('inner rollback')
            owner.latest = original
            observation = self.observation(service)
            self.sample(service, observation)
            self.assertTrue(service._repository.load_scene_space(outer.scene_spaces[0].scene_space_id).observation_samples)
            self.assertEqual(len(service._repository.load_scene_spaces()), 1)
        self.assertEqual(service._latest_semantic_map_context.scene_spaces[0].scene_space_id,
                         outer.scene_spaces[0].scene_space_id)
        self.assertTrue(service._latest_semantic_map_context.scene_spaces[0].observation_samples)

    def test_inner_rollback_restores_successful_nested_publication_at_outer_completion(self):
        service, owner = self.fixture()
        with service._repository.storage.transaction():
            with service._repository.storage.transaction():
                surviving = service._refresh_semantic_map_context(owner.latest)
            original = owner.latest
            with self.assertRaisesRegex(RuntimeError, 'inner rollback'), service._repository.storage.transaction():
                owner.latest = original.model_copy(update={'source_epoch': 'rolled-back'})
                service._refresh_semantic_map_context(owner.latest)
                raise RuntimeError('inner rollback')
            owner.latest = original
            # No transaction-local read repairs the request before completion.
            self.assertFalse(service._latest_semantic_map_context.scene_spaces)
        self.assertEqual(service._latest_semantic_map_context.scene_spaces[0].scene_space_id,
                         surviving.scene_spaces[0].scene_space_id)
        self.assertEqual(len(service._repository.load_scene_spaces()), 1)
        self.assertFalse(service._repository.storage.completion_errors)

    def test_warm_context_after_outer_sample_write_reads_current_scene_without_publication(self):
        service, owner = self.fixture()
        before = service._refresh_semantic_map_context(owner.latest)
        with service._repository.storage.transaction():
            observation = self.observation(service)
            self.sample(service, observation)
            with patch.object(service._semantic_map_builder, 'build', side_effect=AssertionError('warm geometry')):
                current = service._refresh_semantic_map_context(owner.latest)
            self.assertEqual([sample.observation_event_id for sample in current.scene_spaces[0].observation_samples],
                             [observation.event_id])
            self.assertIs(service._latest_semantic_map_context, before)
            self.assertFalse(before.scene_spaces[0].observation_samples)
        self.assertTrue(service._latest_semantic_map_context.scene_spaces[0].observation_samples)

    def test_linked_instance_created_in_outer_writer_is_visible_to_sampler(self):
        service, owner = self.fixture()
        service._refresh_semantic_map_context(owner.latest)
        original = service._scene_space_sampler.sample
        seen = []
        def sampling(*args, **kwargs):
            seen.extend(kwargs['semantic_instances'])
            return original(*args, **kwargs)
        with service._repository.storage.transaction():
            instance = producer.instance(map_name='library', metadata={'memory_library_name': 'library'})
            service._repository.upsert_semantic_instance(instance)
            observation = producer.event(map_name='library', linked_instance_ids=[instance.instance_id],
                                         metadata={'memory_library_name': 'library'})
            service._repository.upsert_observation_event(observation)
            self.assertNotIn(instance.instance_id, service._semantic_instances_by_id)
            with patch.object(service._scene_space_sampler, 'sample', side_effect=sampling):
                self.sample(service, observation)
            self.assertEqual([item.instance_id for item in seen], [instance.instance_id])
            self.assertEqual(seen[0].label, instance.label)
            stored = service._repository.load_scene_spaces()[0]
            self.assertIn(instance.instance_id, stored.linked_object_instance_ids)

    def test_deleted_linked_durable_instance_is_removed_from_outer_sampler_inputs(self):
        service, owner = self.fixture()
        service._refresh_semantic_map_context(owner.latest)
        instance = producer.instance(map_name='library', metadata={'memory_library_name': 'library'})
        service._repository.upsert_semantic_instance(instance)
        observation = producer.event(map_name='library', linked_instance_ids=[instance.instance_id],
                                     metadata={'memory_library_name': 'library'})
        service._repository.upsert_observation_event(observation)
        original, seen = service._scene_space_sampler.sample, []
        def sampling(*args, **kwargs):
            seen.extend(kwargs['semantic_instances'])
            return original(*args, **kwargs)
        with service._repository.storage.transaction():
            service._repository.delete_semantic_instances([instance.instance_id])
            self.assertIn(instance.instance_id, service._semantic_instances_by_id)
            with patch.object(service._scene_space_sampler, 'sample', side_effect=sampling):
                self.sample(service, observation)
            self.assertEqual(seen, [])

    def test_genuine_live_only_linked_instance_remains_in_outer_sampler_inputs(self):
        service, owner = self.fixture()
        service._refresh_semantic_map_context(owner.latest)
        instance = producer.instance(map_name='library', metadata={'memory_library_name': 'library'})
        object.__setattr__(instance, '_memory_live_only', True)
        service._semantic_instances_by_id[instance.instance_id] = instance
        observation = producer.event(map_name='library', linked_instance_ids=[instance.instance_id],
                                     metadata={'memory_library_name': 'library'})
        service._repository.upsert_observation_event(observation)
        original, seen = service._scene_space_sampler.sample, []
        def sampling(*args, **kwargs):
            seen.extend(kwargs['semantic_instances'])
            return original(*args, **kwargs)
        with service._repository.storage.transaction():
            with patch.object(service._scene_space_sampler, 'sample', side_effect=sampling):
                self.sample(service, observation)
            self.assertEqual([item.instance_id for item in seen], [instance.instance_id])
            self.assertTrue(seen[0]._memory_live_only)

    def test_deleted_event_is_removed_from_outer_sampler_inputs(self):
        service, owner = self.fixture()
        service._refresh_semantic_map_context(owner.latest)
        observation = self.observation(service)
        original, seen = service._scene_space_sampler.sample, []
        def sampling(*args, **kwargs):
            seen.extend(kwargs['observation_events'])
            return original(*args, **kwargs)
        with service._repository.storage.transaction():
            service._repository.delete_observation_events([observation.event_id])
            self.assertIn(observation.event_id, service._observation_events_by_id)
            with patch.object(service._scene_space_sampler, 'sample', side_effect=sampling):
                self.sample(service, observation)
            self.assertEqual(seen, [])
            self.assertFalse(service._repository.load_scene_spaces()[0].observation_samples)
            self.assertEqual(self.sample_alias_count(service), 0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
