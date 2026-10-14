#!/usr/bin/env python3
"""Settings reads stay available while an ordered hot-apply operation waits."""
from __future__ import annotations
import sys
import os
from contextlib import contextmanager
import threading
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import check_configuration_revisions as fixture
from core import config_io, config_registry, config_values, config_profiles
import gateways.settings as application_module
from gateways.settings import SettingsApplication

CONF = 'NUWAX_PERCEPTION_YOLO_WORLD_CONFIDENCE'
IOU = 'NUWAX_PERCEPTION_YOLO_WORLD_IOU'
MAX = 'NUWAX_PERCEPTION_YOLO_WORLD_MAX_DETECTIONS'
KEYS = (CONF, IOU, MAX)


class ReadIsolationChecks(unittest.TestCase):
    def setUp(self):
        fixture.ConfigurationRevisionChecks.setUp(self)
        fields = {key: fixture.field(key, kind='int' if key == MAX else 'float') for key in KEYS}
        self.stack.enter_context(patch.object(config_registry, 'get_field', side_effect=fields.get))
        self.stack.enter_context(patch.object(config_registry, 'build_registry', return_value=list(fields.values())))
        self.stack.enter_context(patch.object(application_module, 'HOT_LOG_KEYS', set()))
        self.stack.enter_context(patch.object(config_io, '_SAVED_SINCE_START', set()))
        self.stack.enter_context(patch.dict(config_values.RUNTIME_VALUES, {CONF: 0.2, IOU: 0.3, MAX: 10}, clear=True))
        self.path.write_text(CONF + '=0.2\n' + IOU + '=0.3\n' + MAX + '=10\n')
        self.other = self.path.with_name('other.env')
        self.other.write_text(CONF + '=0.6\n' + IOU + '=0.3\n' + MAX + '=10\n')
        self.path.parent.joinpath('.active').write_text('main\n')
        active = self.root / 'runtime_data/settings/active.env'
        active.parent.mkdir(parents=True)
        active.symlink_to(self.path)
        def config():
            return SimpleNamespace(robot=SimpleNamespace(active_robot='alpha'), perception=SimpleNamespace(yolo_world=SimpleNamespace(
                confidence_threshold=config_values.config_value(CONF, 0.2, 'float', {}),
                iou_threshold=config_values.config_value(IOU, 0.3, 'float', {}),
                max_detections=config_values.config_value(MAX, 10, 'int', {}))))
        self.stack.enter_context(patch.object(sys.modules['core.settings'], 'load_config', side_effect=config, create=True))
        self.calls = []
        self.apply_consumer = lambda candidate: None
        def consume(candidate):
            self.calls.append((candidate.confidence_threshold, candidate.iou_threshold, candidate.max_detections))
            self.apply_consumer(candidate)
        self.app = SettingsApplication(SimpleNamespace(config=config(), perception_service=SimpleNamespace(update_detector_thresholds=consume)))
        self.addCleanup(self.app.close)
        self.stack.enter_context(patch('gateways.settings_restart.restart_status', return_value={}))
        self.gates, self.workers = [], []
        self.addCleanup(self.finish_workers)

    def finish_workers(self):
        for gate in self.gates:
            gate.set()
        for thread, _ in self.workers:
            thread.join(3)
            self.assertFalse(thread.is_alive(), 'settings test left a blocked worker')

    def gate(self):
        event = threading.Event()
        self.gates.append(event)
        return event

    def spawn(self, fn):
        outcome = {}
        def run():
            try: outcome['result'] = fn()
            except BaseException as error: outcome['error'] = error
        thread = threading.Thread(target=run, daemon=True)
        self.workers.append((thread, outcome))
        thread.start()
        return thread, outcome

    def join(self, worker):
        worker[0].join(2)
        self.assertFalse(worker[0].is_alive(), 'read waited for a gated consumer')
        if 'error' in worker[1]: raise worker[1]['error']
        return worker[1].get('result')

    def schema(self, profile='main'):
        return self.app.schema('alpha', profile)

    def revision(self, profile='main'):
        return self.schema(profile)['profile_context']['revision']

    def fields(self, schema):
        return {item['key']: item for section in schema['sections'] for item in section['fields']}

    def save(self, value, revision, profile='main', **extra):
        return self.app.save({CONF: str(value), **extra}, robot='alpha', profile=profile, expected_revision=revision)

    def pause_consumer(self):
        entered, release = self.gate(), self.gate()
        def pause(candidate):
            entered.set()
            if not release.wait(5): raise AssertionError('consumer gate timed out')
        self.apply_consumer = pause
        return entered, release

    def test_schema_reports_saved_old_effective_and_pending_while_consumer_waits(self):
        token = self.revision()
        entered, release = self.pause_consumer()
        writer = self.spawn(lambda: self.save(0.6, token, **{IOU: '0.7', MAX: '20'}))
        self.assertTrue(entered.wait(2))
        during = self.join(self.spawn(self.schema))
        fields = self.fields(during)
        self.assertEqual([fields[k]['effective_value'] for k in KEYS], ['0.2', '0.3', '10'])
        self.assertEqual([fields[k]['saved_value'] for k in KEYS], ['0.6', '0.7', '20'])
        self.assertTrue(all(fields[k]['pending_restart'] for k in KEYS))
        self.assertEqual(set(during['application']['hot_apply_pending']), set(KEYS))
        self.assertNotEqual(during['profile_context']['revision'], token)
        release.set(); self.join(writer)
        after = self.schema()
        self.assertEqual([self.fields(after)[k]['effective_value'] for k in KEYS], ['0.6', '0.7', '20'])
        self.assertFalse(after['application']['hot_apply_pending'])
        self.assertFalse(any(self.fields(after)[k]['pending_restart'] for k in KEYS))

    def test_same_revision_second_save_waits_then_conflicts(self):
        token = self.revision()
        entered, release = self.pause_consumer()
        first = self.spawn(lambda: self.save(0.6, token))
        self.assertTrue(entered.wait(2))
        second = self.spawn(lambda: self.save(0.7, token))
        self.assertTrue(second[0].is_alive())
        self.assertEqual(len(self.calls), 1)
        release.set(); self.join(first)
        with self.assertRaises(config_profiles.ConfigurationConflict): self.join(second)
        self.assertEqual(len(self.calls), 1)

    def test_failed_then_new_success_clears_errors_without_publishing_failed_value(self):
        self.apply_consumer = lambda candidate: (_ for _ in ()).throw(RuntimeError('synthetic detector failure'))
        result = self.save(0.6, self.revision())
        self.assertIn(CONF, result['apply_errors'])
        failed = self.schema()
        self.assertEqual(self.fields(failed)[CONF]['effective_value'], '0.2')
        self.assertFalse(failed['application']['hot_apply_pending'])
        self.assertIn(CONF, failed['application']['apply_errors'])
        self.apply_consumer = lambda candidate: None
        self.save(0.7, failed['profile_context']['revision'])
        final = self.schema()
        self.assertEqual(self.fields(final)[CONF]['effective_value'], '0.7')
        self.assertNotIn(CONF, final['application']['apply_errors'])

    def test_error_formatting_cannot_hold_read_state_lock(self):
        entered, release = self.gate(), self.gate()
        class SlowError(RuntimeError):
            def __str__(self):
                entered.set()
                if not release.wait(5): raise AssertionError('formatting gate timed out')
                return 'synthetic slow formatting'
        self.apply_consumer = lambda candidate: (_ for _ in ()).throw(SlowError())
        token = self.revision()
        writer = self.spawn(lambda: self.save(0.6, token))
        self.assertTrue(entered.wait(2))
        during = self.join(self.spawn(self.schema))
        self.assertEqual(self.fields(during)[CONF]['effective_value'], '0.2')
        release.set(); self.join(writer)
        self.assertFalse(self.schema()['application']['hot_apply_pending'])

    def test_broken_error_formatting_retires_pending_and_propagates(self):
        class BrokenError(RuntimeError):
            def __str__(self): raise ValueError('synthetic formatting failure')
        self.apply_consumer = lambda candidate: (_ for _ in ()).throw(BrokenError())
        with self.assertRaisesRegex(ValueError, 'synthetic formatting failure'):
            self.save(0.6, self.revision())
        self.assertFalse(self.app.status()['hot_apply_pending'])
        self.assertEqual(self.fields(self.schema())[CONF]['effective_value'], '0.2')

    def test_activate_then_save_keeps_order(self):
        other_token = self.revision('other')
        entered, release = self.pause_consumer()
        first = self.spawn(lambda: self.app.activate(robot='alpha', profile='other', expected_revision=other_token))
        self.assertTrue(entered.wait(2))
        read = self.join(self.spawn(lambda: self.app.schema()))
        self.assertEqual(read['profile_context']['profile'], 'other')
        second = self.spawn(lambda: self.save(0.7, other_token, profile='other'))
        self.assertEqual(len(self.calls), 1)
        release.set(); self.join(first); self.join(second)
        self.assertEqual([call[0] for call in self.calls], [0.6, 0.7])
        self.assertEqual(self.fields(self.schema('other'))[CONF]['effective_value'], '0.7')

    def test_save_then_activate_keeps_order(self):
        main_token, other_token = self.revision(), self.revision('other')
        entered, release = self.pause_consumer()
        first = self.spawn(lambda: self.save(0.5, main_token))
        self.assertTrue(entered.wait(2))
        second = self.spawn(lambda: self.app.activate(robot='alpha', profile='other', expected_revision=other_token))
        self.join(self.spawn(self.schema))
        self.assertEqual(len(self.calls), 1)
        release.set(); self.join(first); self.join(second)
        self.assertEqual([call[0] for call in self.calls], [0.5, 0.6])
        self.assertEqual(self.fields(self.schema('other'))[CONF]['effective_value'], '0.6')

    def test_schema_uses_one_adopted_snapshot_through_field_render(self):
        initial = self.revision()
        entered, release = self.gate(), self.gate()
        original = config_io._robot_geometry_preview
        def pause(values):
            entered.set()
            if not release.wait(5): raise AssertionError('schema gate timed out')
            return original(values)
        with patch.object(config_io, '_robot_geometry_preview', side_effect=pause):
            reader = self.spawn(self.schema)
            self.assertTrue(entered.wait(2))
            # Simulate an already-running consumer committing after file capture.
            with self.app._state_lock:
                config_values.RUNTIME_VALUES.update({CONF: 0.8, IOU: 0.9, MAX: 40})
                self.app._apply_errors[CONF] = 'later error'
            release.set(); result = self.join(reader)
        self.assertEqual([self.fields(result)[k]['effective_value'] for k in KEYS], ['0.2', '0.3', '10'])
        self.assertNotIn(CONF, result['application']['apply_errors'])
        self.assertEqual(result['profile_context']['revision'], initial)

    def test_legacy_read_requires_no_new_runtime_directory_write(self):
        (self.root / 'runtime_data/settings/active.env').unlink()
        self.app.started_robot = ''
        with patch.dict(os.environ, {'NUWAX_USER_CONFIG_ENV_FILE': str(self.path)}), \
             patch.object(Path, 'mkdir', side_effect=PermissionError('read-only source')):
            result = self.app.schema()
        self.assertIsNone(result['profile_context'])
        self.assertEqual(self.fields(result)[CONF]['value'], '0.2')

    def test_profile_schema_never_acquires_write_sequence_lock(self):
        class ForbiddenLock:
            def __enter__(self): raise AssertionError('profile reader acquired write lock')
            def __exit__(self, *args): pass
        with patch.object(self.app, '_lock', ForbiddenLock()):
            self.assertEqual(self.fields(self.schema())[CONF]['effective_value'], '0.2')

    def test_mode_changes_release_one_lock_domain_before_taking_another(self):
        for to_profile in (False, True):
            with self.subTest(to_profile=to_profile):
                states = {'file': False, 'legacy': False}
                calls = []
                @contextmanager
                def file_context(path):
                    self.assertFalse(states['legacy'])
                    states['file'] = True; calls.append('file')
                    try: yield
                    finally: states['file'] = False
                @contextmanager
                def legacy_context():
                    self.assertFalse(states['file'])
                    states['legacy'] = True; calls.append('legacy')
                    try: yield
                    finally: states['legacy'] = False
                locations = iter([None, self.location, self.location, self.location] if to_profile
                                 else [self.location, None, None, None])
                with patch.object(config_io, '_target_lock', side_effect=file_context):
                    result = config_io.build_schema(revision_owner=self.app.revision_owner,
                        location_provider=lambda: next(locations), legacy_read_context=legacy_context,
                        runtime_state_provider=self.app._runtime_state)
                self.assertEqual(result['profile_context'] is not None, to_profile)
                self.assertEqual(calls, ['legacy', 'file'] if to_profile else ['file', 'legacy'])

    def test_service_manager_status_wait_does_not_hold_adopted_state(self):
        token = self.revision()
        entered, release = self.gate(), self.gate()
        def status(root):
            entered.set()
            if not release.wait(5): raise AssertionError('status gate timed out')
            return {}
        with patch('gateways.settings_restart.restart_status', side_effect=status):
            reader = self.spawn(self.app.status)
            self.assertTrue(entered.wait(2))
            result = self.join(self.spawn(lambda: self.save(0.6, token)))
            self.assertIn(CONF, result['hot_applied'])
            release.set(); self.join(reader)

    def test_fork_start_replaces_both_locks_and_pending_without_waiting(self):
        class Locked:
            def __enter__(self): raise AssertionError('inherited lock acquired')
            def __exit__(self, *args): pass
        old_owner = self.app.revision_owner
        token = self.revision()
        self.addCleanup(old_owner.close)
        self.app._lock = Locked(); self.app._state_lock = Locked()
        self.app._hot_apply_pending.add(CONF); self.app._pid = -1
        self.app.start()
        self.assertFalse(self.app.status()['hot_apply_pending'])
        with self.assertRaises(config_profiles.ConfigurationConflict): self.save(0.6, token)

    def test_close_during_schema_capture_does_not_revive_owner(self):
        entered, release = self.gate(), self.gate()
        original = config_io._robot_geometry_preview
        def pause(values):
            entered.set()
            if not release.wait(5): raise AssertionError('schema gate timed out')
            return original(values)
        with patch.object(config_io, '_robot_geometry_preview', side_effect=pause):
            reader = self.spawn(self.schema)
            self.assertTrue(entered.wait(2))
            self.app.close(); release.set()
            with self.assertRaises(config_profiles.ConfigurationConflict): self.join(reader)
        self.app.start()
        self.assertTrue(self.revision().startswith('config:'))


if __name__ == '__main__':
    unittest.main()
