#!/usr/bin/env python3
"""私有配置捕获、无摘要CAS与设置接线的隔离合成回归。"""
from __future__ import annotations

import ast
import asyncio
from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack, asynccontextmanager
import hashlib
import json
import os
from pathlib import Path
import stat
import sys
import tempfile
import threading
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from core import config_io, config_profiles, config_registry, config_values
from gateways.settings import SettingsApplication

WORK = Path(tempfile.gettempdir()) / 'neh-configuration-revisions-checks'
SECRET = 'synthetic-secret-never-a-real-credential'


def field(key, *, kind='str', sensitive=False, aliases=()):
    return config_registry.ConfigField(key=key, zh_name=key, description='', type=kind,
        section_id='synthetic', section_title='合成配置', advanced=False, sensitive=sensitive,
        target_file=config_registry.TARGET_CORE, default_repr="''", aliases=aliases)


class ConfigurationRevisionChecks(unittest.TestCase):
    def setUp(self):
        WORK.mkdir(parents=True, exist_ok=True)
        temporary = tempfile.TemporaryDirectory(prefix='fixture-', dir=WORK)
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.stack = ExitStack(); self.addCleanup(self.stack.close)
        spool_directory = tempfile.TemporaryDirectory(prefix='private-spool-', dir=WORK)
        self.addCleanup(spool_directory.cleanup)
        self.spool_directory = Path(spool_directory.name)
        self.stack.enter_context(patch.object(tempfile, 'tempdir', str(self.spool_directory)))
        self.stack.enter_context(patch.dict(os.environ, {'NUWAX_ACTIVE_ROBOT': 'alpha'}, clear=True))
        for robot in ('alpha', 'beta'):
            directory = self.root / 'drivers' / robot
            (directory / 'configs').mkdir(parents=True)
            (directory / 'manifest.py').write_text('# synthetic driver declaration\n')
            (directory / 'assembly.py').write_text('# no driver imports\n')
            (directory / 'data_plane.env').write_text('DEVICE_VALUE=synthetic\n')
        (self.root / '.env').write_text('NUWAX_ACTIVE_ROBOT=alpha\nLEGACY_VALUE=one\n')
        self.path = self.root / 'drivers/alpha/configs/main.env'
        self.path.write_text('# preserved comment\nVALUE=one\nSECRET=' + SECRET + '\nUNKNOWN=keep\n')
        self.location = config_values.configuration_location(root=self.root, robot='alpha', profile='main')
        self.owner = config_profiles.ProfileRevisionOwner(); self.addCleanup(self.owner.close)
        self.stack.enter_context(patch.object(config_io, '_CORE_ENV', self.root / '.env'))
        fields = {item.key: item for item in (field('VALUE', aliases=('OLD_VALUE',)), field('SECRET', sensitive=True), field('COUNT', kind='int'))}
        self.stack.enter_context(patch.object(config_registry, 'get_field', side_effect=fields.get))
        self.stack.enter_context(patch.object(config_registry, 'build_registry', return_value=list(fields.values())))
        self.stack.enter_context(patch.object(config_registry, 'SECTIONS', [('synthetic', '合成配置')]))
        self.stack.enter_context(patch.object(config_registry, 'structured_options', return_value={}))
        self.stack.enter_context(patch.object(config_io, 'geometry_backend_statuses', return_value=[]))
        self.stack.enter_context(patch.object(config_io, '_robot_geometry_preview', return_value={'summary': 'synthetic', 'warnings': []}))
        self.stack.enter_context(patch.object(config_io, '_collect_effective_values', side_effect=lambda values, *args, **kwargs: dict(values)))
        self.stack.enter_context(patch.object(config_io, '_default_values', return_value={}))
        self.stack.enter_context(patch.object(config_io.subprocess, 'run', side_effect=AssertionError('native subprocess forbidden')))
        # schema只需此只读字段；不导入会在模块初始化读取真实.env的生产settings。
        fake_settings = ModuleType('core.settings')
        fake_settings.LEGACY_CONFIG_FILE = self.root / '.config'; fake_settings._FILE_VALUES = {}
        self.stack.enter_context(patch.dict(sys.modules, {'core.settings': fake_settings}))
        import core
        self.stack.enter_context(patch.object(core, 'settings', fake_settings, create=True))

    def token(self, location=None, owner=None):
        return config_profiles.revision(location or self.location, root=self.root, revision_owner=owner or self.owner)

    def save(self, values, token=None):
        return config_io.save_values(values, robot='alpha', profile='main', expected_revision=token, revision_owner=self.owner)

    def schema(self):
        return config_io.build_schema('alpha', 'main', revision_owner=self.owner)

    def test_opaque_revision_reuses_actual_equal_sources_without_exposing_values(self):
        first = self.token(); second = self.token()
        self.assertEqual(first, second)
        self.assertRegex(first, r'^config:[0-9a-f]{32}:1$')
        self.assertNotIn(SECRET, first)
        self.assertEqual(len(self.owner._current), 1)
        self.assertIsNone(config_profiles.check_revision(self.location, first, root=self.root, revision_owner=self.owner))

    def test_same_snapshot_can_be_issued_twice_and_lease_survives_owner_close(self):
        with config_profiles.CapturedProfile(self.location, self.root) as snapshot:
            token = self.owner.issue(snapshot)
            self.assertEqual(self.owner.issue(snapshot), token)
            snapshot.retain()
            self.owner.close()
            self.assertEqual(snapshot.values()['VALUE'], 'one')
        self.assertIsNotNone(snapshot._stream)
        snapshot.close(); self.assertIsNone(snapshot._stream)

    def test_same_size_manual_change_and_restored_mtime_still_conflicts(self):
        token = self.token(); before = self.path.stat()
        self.path.write_text(self.path.read_text().replace('VALUE=one', 'VALUE=two'))
        os.utime(self.path, ns=(before.st_atime_ns, before.st_mtime_ns))
        self.assertEqual(self.path.stat().st_size, before.st_size)
        with self.assertRaises(config_profiles.ConfigurationConflict): self.save({'VALUE': 'new'}, token)
        self.assertEqual(config_values.read_env_values(self.path)['VALUE'], 'two')

    def test_two_editors_cannot_overwrite_each_other_and_noop_keeps_revision(self):
        first, second = self.token(), self.token()
        self.save({'VALUE': 'one'}, first)
        self.assertEqual(self.token(), first)
        self.save({'VALUE': 'two'}, first)
        with self.assertRaises(config_profiles.ConfigurationConflict): self.save({'VALUE': 'three'}, second)
        self.assertNotEqual(self.token(), first)

    def test_restart_foreign_profile_and_unknown_or_legacy_tokens_fail_closed(self):
        token = self.token()
        other = self.root / 'drivers/beta/configs/main.env'; other.write_bytes(self.path.read_bytes())
        location = config_values.configuration_location(root=self.root, robot='beta', profile='main')
        with self.assertRaises(config_profiles.ConfigurationConflict):
            config_profiles.check_revision(location, token, root=self.root, revision_owner=self.owner)
        restarted = config_profiles.ProfileRevisionOwner(); self.addCleanup(restarted.close)
        for expected, owner in [(token, restarted), ('a' * 64, self.owner), ('invented', self.owner), (42, self.owner)]:
            with self.assertRaises(config_profiles.ConfigurationConflict):
                config_profiles.check_revision(self.location, expected, root=self.root, revision_owner=owner)
        with self.assertRaises(config_profiles.ConfigurationConflict):
            config_profiles.check_revision(self.location, token, root=self.root)

    def test_inherited_owner_cannot_share_parent_spool_cursor_or_tokens_after_fork(self):
        token = self.token()
        with config_profiles.CapturedProfile(self.location, self.root) as snapshot:
            with patch.object(config_profiles.os, 'getpid', return_value=self.owner._pid + 1):
                with self.assertRaises(config_profiles.ConfigurationConflict): self.owner.issue(snapshot)
                with self.assertRaises(config_profiles.ConfigurationConflict): self.owner.check(snapshot, token)
                self.owner.close()
        self.assertFalse(self.owner._closed)

    def test_legacy_source_existence_path_and_order_are_part_of_cas(self):
        location = config_values.configuration_location(root=self.root, robot='alpha', profile='默认')
        token = self.token(location)
        (self.root / '.config').write_bytes(b'')
        with self.assertRaises(config_profiles.ConfigurationConflict):
            config_profiles.check_revision(location, token, root=self.root, revision_owner=self.owner)
        first, second = self.root / 'first.env', self.root / 'second.env'
        first.write_text('VALUE=same\n'); second.write_bytes(first.read_bytes())
        with patch.object(config_profiles, 'configuration_sources', return_value=[first, second]): token = self.token()
        with patch.object(config_profiles, 'configuration_sources', return_value=[second, first]):
            with self.assertRaises(config_profiles.ConfigurationConflict):
                config_profiles.check_revision(self.location, token, root=self.root, revision_owner=self.owner)

    def test_schema_values_and_revision_share_capture_if_file_changes_during_render(self):
        def change_after_capture(values):
            self.assertEqual(values['VALUE'], 'one')
            self.path.write_text(self.path.read_text().replace('VALUE=one', 'VALUE=two'))
            self.assertEqual(config_io._current_value('VALUE'), 'one')
            return {'summary': 'synthetic', 'warnings': []}
        with patch.object(config_io, '_robot_geometry_preview', side_effect=change_after_capture): schema = self.schema()
        fields = {item['key']: item for section in schema['sections'] for item in section['fields']}
        self.assertEqual(fields['VALUE']['value'], 'one')
        self.assertEqual(fields['SECRET']['value'], '********')
        self.assertNotIn(SECRET, json.dumps(schema))
        with self.assertRaises(config_profiles.ConfigurationConflict): self.save({'VALUE': 'new'}, schema['profile_context']['revision'])
        self.assertIsNone(config_io._PROFILE_VALUES.get())

    def test_masked_and_blank_secret_stays_unchanged_and_comments_aliases_survive(self):
        self.path.write_text(self.path.read_text() + 'OLD_VALUE=old\n')
        token = self.token()
        result = self.save({'VALUE': 'two', 'SECRET': '********'}, token)
        self.assertIn('SECRET', result.skipped)
        content = self.path.read_text()
        self.assertIn('# preserved comment', content); self.assertIn('UNKNOWN=keep', content)
        self.assertIn('SECRET=' + SECRET, content); self.assertNotIn('OLD_VALUE=', content)
        self.save({'SECRET': ''}, self.token())
        self.assertEqual(config_values.read_env_values(self.path)['SECRET'], SECRET)

    def test_type_rejection_does_not_write_or_consume_valid_revision(self):
        token = self.token(); before = self.path.read_bytes()
        result = self.save({'COUNT': 'not a number', 'NOT_REGISTERED': 'x'}, token)
        self.assertEqual(set(result.rejected), {'COUNT', 'NOT_REGISTERED'})
        self.assertEqual(self.path.read_bytes(), before); self.assertEqual(self.token(), token)

    def test_capture_to_write_gap_detects_external_target_change(self):
        token = self.token(); original = config_io._save_values
        def change(values, **kwargs):
            self.path.write_text(self.path.read_text().replace('VALUE=one', 'VALUE=two'))
            return original(values, **kwargs)
        with patch.object(config_io, '_save_values', side_effect=change):
            with self.assertRaises(config_profiles.ConfigurationConflict): self.save({'VALUE': 'new'}, token)
        self.assertEqual(config_values.read_env_values(self.path)['VALUE'], 'two')

    def test_atomic_failure_preserves_original_permission_and_leaves_no_temporary(self):
        self.path.chmod(0o640); before = self.path.read_bytes(); token = self.token(); original = os.replace
        def fail(source, target):
            if Path(target) == self.path: raise OSError('synthetic write failure')
            return original(source, target)
        with patch.object(config_io.os, 'replace', side_effect=fail):
            with self.assertRaises(OSError): self.save({'VALUE': 'two'}, token)
        self.assertEqual(self.path.read_bytes(), before)
        self.assertEqual(stat.S_IMODE(self.path.stat().st_mode), 0o640)
        self.assertEqual(list(self.path.parent.glob('.cfg_*')), [])
        self.save({'VALUE': 'two'}, token)
        self.assertEqual(stat.S_IMODE(self.path.stat().st_mode), 0o640)

    def test_create_profile_uses_same_checked_capture_and_rejects_stale_source(self):
        token = self.token()
        context = config_profiles.create_profile(root=self.root, robot='alpha', source='main', name='copy',
            expected_revision=token, revision_owner=self.owner)
        self.assertTrue(context['revision'].startswith('config:'))
        copied = self.path.parent / 'copy.env'
        self.assertEqual(config_values.read_env_values(copied)['VALUE'], 'one')
        self.save({'VALUE': 'two'}, token)
        with self.assertRaises(config_profiles.ConfigurationConflict):
            config_profiles.create_profile(root=self.root, robot='alpha', source='main', name='stale', expected_revision=token, revision_owner=self.owner)
        self.assertFalse((self.path.parent / 'stale.env').exists())

    def test_activate_validates_capture_and_stale_input_does_not_change_selection(self):
        token = self.token()
        self.save({'VALUE': 'two'}, token)
        with self.assertRaises(config_profiles.ConfigurationConflict):
            config_profiles.activate_profile(root=self.root, robot='alpha', profile='main', expected_revision=token, revision_owner=self.owner)
        self.assertFalse(self.location['selector'].exists())
        result = config_profiles.activate_profile(root=self.root, robot='alpha', profile='main', expected_revision=self.token(), revision_owner=self.owner)
        self.assertEqual(result['selected_profile'], 'main')
        self.assertEqual((self.root / 'runtime_data/settings/active.env').resolve(), self.path)
        self.path.write_text(self.path.read_text() + 'COUNT=invalid\n')
        with self.assertRaises(ValueError):
            config_profiles.activate_profile(root=self.root, robot='alpha', profile='main', expected_revision=self.token(), revision_owner=self.owner)

    def test_activation_detects_target_change_during_validation_before_selecting_it(self):
        token = self.token(); validate = config_io._validate
        def change_after_validation(key, value, kind):
            result = validate(key, value, kind)
            if key == 'VALUE':
                self.path.write_text(self.path.read_text().replace('VALUE=one', 'VALUE=two'))
            return result
        with patch.object(config_io, '_validate', side_effect=change_after_validation):
            with self.assertRaises(config_profiles.ConfigurationConflict):
                config_profiles.activate_profile(root=self.root, robot='alpha', profile='main', expected_revision=token, revision_owner=self.owner)
        self.assertFalse(self.location['selector'].exists())
        self.assertFalse((self.root / 'runtime_data/settings/active.env').is_symlink())

    def test_default_profile_migration_uses_captured_sources_and_refuses_stale_migration(self):
        location = config_values.configuration_location(root=self.root, robot='alpha', profile='默认')
        token = self.token(location)
        (self.root / '.env').write_text('NUWAX_ACTIVE_ROBOT=alpha\nLEGACY_VALUE=two\n')
        with self.assertRaises(config_profiles.ConfigurationConflict):
            config_io.save_values({'VALUE': 'new'}, robot='alpha', profile='默认', expected_revision=token, revision_owner=self.owner)
        self.assertFalse(location['path'].exists())
        config_io.save_values({'VALUE': 'new'}, robot='alpha', profile='默认', expected_revision=self.token(location), revision_owner=self.owner)
        self.assertEqual(config_values.read_env_values(location['path'])['LEGACY_VALUE'], 'two')

    def test_bounded_baselines_spill_privately_and_expired_versions_require_refresh(self):
        owner = config_profiles.ProfileRevisionOwner(max_profiles=1); self.addCleanup(owner.close)
        self.path.write_text('#' + 's' * (1024 * 1024) + '\nVALUE=large\n')
        old_token = self.token(owner=owner)
        baseline = next(iter(owner._current.values()))[1]
        stream = baseline._stream
        self.assertTrue(stream._rolled)
        self.assertEqual(stat.S_IMODE(os.fstat(stream.fileno()).st_mode), 0o600)
        other = self.path.parent / 'other.env'; other.write_text('VALUE=other\n')
        location = config_values.configuration_location(root=self.root, robot='alpha', profile='other')
        self.token(location, owner)
        self.assertEqual(len(owner._current), 1); self.assertIsNone(baseline._stream); self.assertTrue(stream.closed)
        with self.assertRaises(config_profiles.ConfigurationConflict):
            config_profiles.check_revision(self.location, old_token, root=self.root, revision_owner=owner)
        self.assertNotEqual(self.token(owner=owner), old_token)
        owner.close(); self.assertFalse(owner._current)
        self.assertEqual([p.name for p in (self.root / 'runtime_data/settings').iterdir() if not p.name.endswith('.lock')], [])

    def test_capture_failure_closes_private_spool(self):
        created = []; original = config_profiles.tempfile.SpooledTemporaryFile
        def remember(*args, **kwargs):
            result = original(*args, **kwargs); created.append(result); return result
        with patch.object(config_profiles.tempfile, 'SpooledTemporaryFile', side_effect=remember), \
             patch.object(config_profiles, 'configuration_sources', side_effect=ValueError('synthetic capture failure')):
            with self.assertRaises(ValueError): self.token()
        self.assertTrue(created[0].closed); self.assertFalse(self.owner._current)

    def test_read_only_sources_need_no_project_mkdir_for_capture_or_catalog(self):
        self.path.write_text('#' + 's' * (1024 * 1024) + '\nVALUE=one\n')
        with patch.object(Path, 'mkdir', side_effect=PermissionError('source project is read only')):
            token = self.token()
            context = config_profiles.profile_catalog(root=self.root, robot='alpha', profile='main', revision_owner=self.owner)
        self.assertEqual(context['revision'], token)
        self.assertFalse((self.root / 'runtime_data').exists())
        baseline = next(iter(self.owner._current.values()))[1]
        self.assertTrue(baseline._stream._rolled)

    def test_schema_get_uses_existing_runtime_lock_area_with_read_only_source_tree(self):
        runtime_directory = self.root / 'runtime_data/settings'; runtime_directory.mkdir(parents=True)
        (runtime_directory / 'transaction.lock').touch(mode=0o600)
        def only_existing_runtime_directory(path, *args, **kwargs):
            if path == runtime_directory and path.is_dir(): return
            raise PermissionError('source directories are read only')
        with patch.object(Path, 'mkdir', only_existing_runtime_directory):
            schema = self.schema()
        self.assertTrue(schema['profile_context']['revision'].startswith('config:'))
        fields = {item['key']: item for section in schema['sections'] for item in section['fields']}
        self.assertEqual(fields['VALUE']['value'], 'one')

    def test_concurrent_saves_have_exactly_one_winner(self):
        token = self.token(); gate = threading.Barrier(2)
        def write(value):
            gate.wait(3)
            try: self.save({'VALUE': value}, token); return True
            except config_profiles.ConfigurationConflict: return False
        with ThreadPoolExecutor(max_workers=2) as pool:
            self.assertEqual(sorted(pool.map(write, ('two', 'new'))), [False, True])

    def test_settings_owner_wires_schema_and_save_without_runtime_reload_side_effects(self):
        runtime = SimpleNamespace(config=SimpleNamespace(robot=SimpleNamespace(active_robot='alpha')))
        application = SettingsApplication(runtime); self.addCleanup(application.close)
        with patch.object(application, 'status', return_value={}), patch.object(application, '_is_active', return_value=False), \
             patch.object(application, '_apply_hot', side_effect=AssertionError('runtime mutation forbidden')):
            schema = application.schema('alpha', 'main')
            token = schema['profile_context']['revision']
            result = application.save({'VALUE': 'two'}, robot='alpha', profile='main', expected_revision=token)
            self.assertTrue(result['written']); self.assertFalse(result['is_active'])
            with self.assertRaises(config_profiles.ConfigurationConflict):
                application.save({'VALUE': 'new'}, robot='alpha', profile='main', expected_revision=token)
        application.close(); self.assertTrue(application.revision_owner._closed)

    def test_http_create_and_restart_hooks_use_the_existing_settings_owner(self):
        tree = ast.parse((ROOT / 'gateways/http/api.py').read_text())
        functions = {node.name: node for node in ast.walk(tree) if isinstance(node, ast.AsyncFunctionDef)}
        for name, target in [('api_config_create_profile', 'create_profile'), ('api_config_restart', 'check_revision')]:
            calls = [node for node in ast.walk(functions[name]) if isinstance(node, ast.Call)
                     and isinstance(node.func, ast.Name) and node.func.id == 'call_settings'
                     and node.args and isinstance(node.args[0], ast.Name) and node.args[0].id == target]
            self.assertEqual(len(calls), 1)
            keyword = next(item for item in calls[0].keywords if item.arg == 'revision_owner')
            self.assertEqual(ast.dump(keyword.value),
                         ast.dump(ast.parse('settings_application.revision_owner', mode='eval').body))

    def lifecycle_factory(self):
        tree = ast.parse((ROOT / 'gateways/http/api.py').read_text())
        node = next(item for item in tree.body if isinstance(item, ast.FunctionDef) and item.name == '_settings_lifespan')
        namespace = {'asynccontextmanager': asynccontextmanager}
        exec(compile(ast.Module(body=[node], type_ignores=[]), 'production-settings-lifespan', 'exec'), namespace)
        build = next(item for item in tree.body if isinstance(item, ast.FunctionDef) and item.name == 'build_http_router')
        self.assertTrue(any(isinstance(item, ast.Assign) and isinstance(item.value, ast.Call)
                            and isinstance(item.value.func, ast.Name) and item.value.func.id == '_settings_lifespan'
                            for item in build.body))
        return namespace['_settings_lifespan']

    async def run_lifespan(self, app, on_start):
        commands = iter([{'type': 'lifespan.startup'}, {'type': 'lifespan.shutdown'}]); messages = []
        async def receive(): return next(commands)
        async def send(message):
            messages.append(message)
            if message['type'] == 'lifespan.startup.complete': on_start()
        await app({'type': 'lifespan', 'asgi': {'version': '3.0'}, 'state': {}}, receive, send)
        self.assertEqual([item['type'] for item in messages], ['lifespan.startup.complete', 'lifespan.shutdown.complete'])

    def test_actual_asgi_shutdown_and_restart_release_spools_with_router_still_referenced(self):
        from fastapi import APIRouter, FastAPI
        application = SettingsApplication(SimpleNamespace(config=SimpleNamespace(robot=SimpleNamespace(active_robot='alpha'))))
        self.addCleanup(application.close)
        self.path.write_text('#' + 's' * (1024 * 1024) + '\nVALUE=one\n')
        events = []
        @asynccontextmanager
        async def outer(_):
            events.append('outer start')
            try: yield {'outer': True}
            finally: events.append('outer stop')
        router = APIRouter()
        router.add_event_handler('startup', lambda: events.append('existing start'))
        router.add_event_handler('shutdown', lambda: events.append('existing stop'))
        router.lifespan_context = self.lifecycle_factory()(router.lifespan_context, application)
        app = FastAPI(lifespan=outer); app.include_router(router)
        tokens, streams = [], []
        def opened():
            tokens.append(self.token(owner=application.revision_owner))
            baseline = next(iter(application.revision_owner._current.values()))[1]
            streams.append(baseline._stream)
            self.assertTrue(baseline._stream._rolled)
        for _ in range(2):
            asyncio.run(self.run_lifespan(app, opened))
            self.assertTrue(application.revision_owner.closed)
            self.assertTrue(streams[-1].closed)
        self.assertNotEqual(tokens[0], tokens[1])
        self.assertEqual(events, ['outer start', 'existing start', 'existing stop', 'outer stop'] * 2)

    def test_lifecycle_start_replaces_inherited_application_lock_before_acquiring_it(self):
        application = SettingsApplication(SimpleNamespace(config=SimpleNamespace(robot=SimpleNamespace(active_robot='alpha'))))
        self.addCleanup(application.close)
        old_owner = application.revision_owner; self.addCleanup(old_owner.close)
        token = self.token(owner=old_owner)
        class InheritedLockedMutex:
            def __enter__(self): raise AssertionError('inherited parent-thread lock was acquired')
            def __exit__(self, *args): pass
        application._lock = InheritedLockedMutex(); application._pid = -1
        application.start()
        self.assertIsNot(application.revision_owner, old_owner)
        with config_profiles.CapturedProfile(self.location, self.root) as snapshot:
            with self.assertRaises(config_profiles.ConfigurationConflict):
                application.revision_owner.check(snapshot, token)

    def test_asgi_shutdown_releases_owner_but_preserves_an_active_capture_lease(self):
        from fastapi import APIRouter, FastAPI
        application = SettingsApplication(SimpleNamespace(config=SimpleNamespace(robot=SimpleNamespace(active_robot='alpha'))))
        self.addCleanup(application.close)
        router = APIRouter(); router.lifespan_context = self.lifecycle_factory()(router.lifespan_context, application)
        app = FastAPI(); app.include_router(router); leases = []
        def opened():
            self.token(owner=application.revision_owner)
            leases.append(next(iter(application.revision_owner._current.values()))[1].retain())
        asyncio.run(self.run_lifespan(app, opened))
        self.assertTrue(application.revision_owner.closed)
        self.assertEqual(leases[0].values()['VALUE'], 'one')
        leases[0].close(); self.assertIsNone(leases[0]._stream)

    def test_asgi_cleanup_does_not_mask_an_existing_shutdown_exception(self):
        from fastapi import APIRouter, FastAPI
        application = SettingsApplication(SimpleNamespace(config=SimpleNamespace(robot=SimpleNamespace(active_robot='alpha'))))
        self.addCleanup(application.close)
        router = APIRouter()
        def earlier_failure(): raise RuntimeError('original shutdown failure')
        router.add_event_handler('shutdown', earlier_failure)
        router.lifespan_context = self.lifecycle_factory()(router.lifespan_context, application)
        app = FastAPI(); app.include_router(router); baselines = []
        def opened():
            self.token(owner=application.revision_owner)
            baselines.append(next(iter(application.revision_owner._current.values()))[1])
        original_close = application.close
        def close_then_fail():
            original_close()
            raise OSError('secondary cleanup failure')
        with patch.object(application, 'close', side_effect=close_then_fail):
            with self.assertRaisesRegex(RuntimeError, 'original shutdown failure'):
                asyncio.run(self.run_lifespan(app, opened))
        self.assertTrue(application.revision_owner.closed)
        self.assertIsNone(baselines[0]._stream)

    def test_explicit_revision_omission_retains_existing_internal_unconditional_behavior(self):
        with patch.object(config_profiles, 'CapturedProfile', side_effect=AssertionError('omitted CAS must not invent a capture')):
            self.assertIsNone(config_profiles.check_revision(self.location, None, root=self.root))
        result = config_io.save_values({'VALUE': 'two'}, robot='alpha', profile='main')
        self.assertTrue(result.written)
        self.assertIsNone(config_profiles.profile_catalog(root=self.root, robot='alpha', profile='main')['revision'])


if __name__ == '__main__':
    attempts = []
    def forbidden(*args, **kwargs):
        attempts.append(True)
        raise AssertionError('Application digest execution is forbidden')
    with ExitStack() as guards:
        for name in ('sha256', 'sha1', 'blake2b', 'blake2s'):
            guards.enter_context(patch.object(hashlib, name, side_effect=forbidden))
        outcome = unittest.main(verbosity=2, exit=False)
    print('Blocked digest constructor attempts:', len(attempts))
    raise SystemExit(0 if outcome.result.wasSuccessful() and not attempts else 1)
