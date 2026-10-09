#!/usr/bin/env python3
"""Exercise transient installer comparisons with temporary files and explicit doubles."""
import os
import io
import json
import platform
import sys
import tempfile
import unittest
from contextlib import ExitStack, nullcontext, redirect_stdout
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scripts.deploy_installer import cli, engine, state, operations as op  # noqa: E402
from scripts.deploy_installer.engine import DeploymentPlan, ExecutionContext  # noqa: E402
from scripts.deploy_installer.model import DeploymentOptions  # noqa: E402
from scripts.deploy_installer.profile import (  # noqa: E402
    DeploymentProfile, ProfileError, PythonProfile, RustProfile, RustBuild,
)


class PlanReached(Exception):
    """Stop CLI execution before any installation or persistent-state operation."""


def profile():
    return DeploymentProfile(1, 'fixture', 'custom', True, {}, (), (),
                             PythonProfile(()), RustProfile('', ()), ())


class ProfileComparisonTests(unittest.TestCase):
    def exercise(self, before, after, stage_error=None, load_error=None):
        with tempfile.TemporaryDirectory() as directory, ExitStack() as stack:
            root = Path(directory)
            options = replace(DeploymentOptions.for_tests(source_root=root),
                              allow_dirty_source=False)
            stage = stack.enter_context(patch.object(
                op, 'stage_source_snapshot', return_value=(root / 'snapshot', 'revision', 'tree'),
                side_effect=stage_error))
            load = stack.enter_context(patch.object(cli, 'load_named_profile',
                                                   return_value=after, side_effect=load_error))
            plan = stack.enter_context(patch.object(cli, 'build_deployment_plan', side_effect=PlanReached))
            stack.enter_context(patch.object(cli, '_options', return_value=options))
            stack.enter_context(patch.object(cli, '_load_profile_argument', return_value=before))
            stack.enter_context(patch.object(cli, 'InstallerLock', side_effect=lambda: nullcontext()))
            stack.enter_context(patch.object(cli.os, 'geteuid', return_value=0))
            stack.enter_context(patch.object(op, 'validate_protected_paths'))
            stack.enter_context(patch.object(cli, 'profile_sha256', side_effect=AssertionError('profile hash')))
            stack.enter_context(patch.object(op.hashlib, 'sha256', side_effect=AssertionError('application hash')))
            outcome = None
            try:
                cli._execute(SimpleNamespace(profile='fixture', release_id=''), dry_run=False)
            except (PlanReached, RuntimeError, ProfileError) as exc:
                outcome = exc
            self.assertFalse((root / '_state').exists())
            return outcome, stage, load, plan

    def test_equal_profiles_ignore_mapping_order_and_preserve_snapshot_options(self):
        before = replace(profile(), metadata={'nested': {'a': 1, 'b': ['é', None]}})
        after = replace(profile(), metadata={'nested': {'b': ['é', None], 'a': 1}})
        outcome, stage, load, plan = self.exercise(before, after)
        self.assertIsInstance(outcome, PlanReached)
        stage.assert_called_once()
        load.assert_called_once()
        self.assertIs(plan.call_args.args[0], after)
        options = plan.call_args.args[1]
        self.assertTrue(options.source_is_snapshot)
        self.assertEqual((options.source_revision, options.source_tree_id), ('revision', 'tree'))
        self.assertEqual(options.source_root, options.fingerprint_source_root)
        self.assertEqual(options.source_root, options.trust_root)

    def test_profile_changes_rejected_before_plan(self):
        for after in (replace(profile(), name='other'),
                      replace(profile(), python=PythonProfile(('new.txt',))),
                      replace(profile(), metadata={'nested': {'x': 'changed'}})):
            with self.subTest(after=after):
                outcome, _, _, plan = self.exercise(profile(), after)
                self.assertIsInstance(outcome, RuntimeError)
                self.assertEqual(str(outcome), 'deployment profile changed while creating the production source snapshot')
                plan.assert_not_called()

    def test_json_scalar_types_and_signed_zero_remain_distinct(self):
        for left, right in ((True, 1), (1, 1.0), (-0.0, 0.0)):
            with self.subTest(left=left, right=right):
                outcome, _, _, plan = self.exercise(replace(profile(), metadata={'value': left}),
                                                    replace(profile(), metadata={'value': right}))
                self.assertIsInstance(outcome, RuntimeError)
                plan.assert_not_called()

    def test_existing_json_normalization_of_sequences_is_preserved(self):
        outcome, _, _, _ = self.exercise(replace(profile(), metadata={'value': ('a', 'b')}),
                                         replace(profile(), metadata={'value': ['a', 'b']}))
        self.assertIsInstance(outcome, PlanReached)

    def test_stage_and_profile_load_failures_keep_their_order(self):
        error = RuntimeError('snapshot failed')
        outcome, _, load, plan = self.exercise(profile(), profile(), stage_error=error)
        self.assertIs(outcome, error)
        load.assert_not_called()
        plan.assert_not_called()
        error = ProfileError('profile parsing failed')
        outcome, _, load, plan = self.exercise(profile(), profile(), load_error=error)
        self.assertIs(outcome, error)
        load.assert_called_once()
        plan.assert_not_called()

    def test_signed_bundle_contract_still_uses_profile_digest(self):
        current = profile()
        metadata = dict(profile=current.name, profile_sha256='trusted', release_id='signed-id',
                        source_revision='git-id', supported_os={}, architectures=[])
        options = SimpleNamespace(release_id='signed-id', allow_unsupported_host=True)
        with patch.object(op, 'profile_sha256', return_value='trusted') as digest:
            op.validate_offline_bundle_contract(current, options, {'metadata': metadata})
            digest.assert_called_once_with(current)
            metadata['profile_sha256'] = 'different'
            with self.assertRaisesRegex(op.DeploymentError, 'profile digest does not match trusted profile'):
                op.validate_offline_bundle_contract(current, options, {'metadata': metadata})


class ActuatorComparisonTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.source = self.root / 'source'
        self.source.mkdir()
        self.manifest = self.root / 'actuator.json'
        self.original = b'{"schema_version":1,"symbols":["motor.stop"]}\n'
        self.write(self.original)
        self.script = self.source / 'scripts/install_motion_guard_deployment.sh'
        self.script.parent.mkdir()
        self.script.write_text('fixture only\n')
        self.options = replace(DeploymentOptions.for_tests(source_root=self.source),
                               actuator_manifest=self.manifest,
                               install_root=self.root / 'install', config_dir=self.root / 'config',
                               state_dir=self.root / 'state', cache_dir=self.root / 'cache')
        self.context = ExecutionContext(DeploymentPlan('fixture', 'opaque-existing-id', ()),
                                        replace(profile(), production=False), self.options, Mock())
        stack = ExitStack()
        self.addCleanup(stack.close)
        stack.enter_context(patch.object(op.hashlib, 'sha256', side_effect=AssertionError('application hash')))
        stack.enter_context(patch.dict(os.environ, {'SUDO_UID': str(os.getuid())}))

    def write(self, payload):
        self.manifest.write_bytes(payload)
        self.manifest.chmod(0o600)

    def preflight(self):
        with ExitStack() as stack:
            stack.enter_context(patch.object(op.os, 'geteuid', return_value=0))
            stack.enter_context(patch.object(op, '_os_release', return_value=('fixture', '1')))
            stack.enter_context(patch.object(op.shutil, 'disk_usage', return_value=SimpleNamespace(free=100 * 1024**3)))
            for name in ('_verify_python_runtime', '_verify_vendor_python_packages',
                         'validate_protected_paths', '_validate_jetson_compatibility'):
                stack.enter_context(patch.object(op, name))
            op.preflight(self.context)

    def provision(self):
        with patch.object(op, '_deployment_script', return_value=self.script), \
                patch.object(op, '_deployment_env', return_value={'FIXTURE': '1'}):
            op.provision_safety(self.context)

    @property
    def snapshot(self):
        return self.options.state_dir / 'backups' / self.options.transaction_id / 'inputs/actuator-symbols.json'

    def assert_no_publish(self):
        self.assertFalse(self.snapshot.exists())
        if self.snapshot.parent.exists():
            self.assertEqual(list(self.snapshot.parent.iterdir()), [])
        self.context.runner.run.assert_not_called()

    def test_validation_returns_same_read_bytes_without_another_read(self):
        with patch.object(Path, 'read_bytes', return_value=self.original) as read:
            result = op.validate_actuator_manifest(self.manifest)
        self.assertIs(result, self.original)
        read.assert_called_once_with()

    def test_real_preflight_to_provision_keeps_bytes_and_runner_contract(self):
        self.preflight()
        self.assertEqual(self.context.notes['actuator_manifest_payload'], self.original)
        self.assertNotIn('actuator_manifest_sha256', self.context.notes)
        self.provision()
        self.assertEqual(self.snapshot.read_bytes(), self.original)
        self.context.runner.run.assert_called_once_with(
            ('bash', str(self.script), 'provision', str(self.snapshot)), env={'FIXTURE': '1'}, timeout=300)

    def test_valid_content_and_whitespace_changes_rejected_before_publish(self):
        self.preflight()
        for changed in (b'{"symbols":["motor.start"]}', self.original + b' '):
            with self.subTest(changed=changed):
                self.write(changed)
                with self.assertRaisesRegex(op.DeploymentError, '^actuator manifest changed after preflight$'):
                    self.provision()
                self.assert_no_publish()

    def test_missing_preflight_note_preserves_existing_provision_behavior(self):
        self.provision()
        self.assertEqual(self.snapshot.read_bytes(), self.original)

    def test_invalid_input_keeps_validation_error_before_change_error(self):
        self.preflight()
        cases = ((b'', 'between 1 byte'), (b'x' * (1024 * 1024 + 1), 'between 1 byte'),
                 (b'\xff', 'invalid actuator manifest JSON'),
                 (b'{"symbols":[],"symbols":["a"]}', 'duplicate JSON key'),
                 (b'{}', 'top-level symbols'), (b'{"symbols":[]}', 'must not be empty'),
                 (b'{"symbols":["a","a"]}', 'must be unique'),
                 (b'{"symbols":["bad symbol"]}', 'invalid symbol'),
                 (b'{"symbols":["a"],"extra":1}', 'unknown field'),
                 (b'{"symbols":["a"],"schema_version":2}', 'schema_version must be 1'),
                 (b'{"symbols":["a"],"source":""}', 'non-empty string'))
        for payload, message in cases:
            with self.subTest(message=message):
                self.write(payload)
                with self.assertRaisesRegex(op.DeploymentError, message):
                    self.provision()
                self.assert_no_publish()

    def test_preflight_validation_failure_does_not_record_payload(self):
        self.write(b'{"symbols":[]}')
        with self.assertRaisesRegex(op.DeploymentError, 'must not be empty'):
            self.preflight()
        self.assertNotIn('actuator_manifest_payload', self.context.notes)
        self.assert_no_publish()

    def test_symlink_writable_and_unapproved_owner_rejections_remain(self):
        alias = self.root / 'alias'
        alias.symlink_to(self.manifest)
        with self.assertRaisesRegex(op.DeploymentError, 'non-symlink regular file'):
            op.validate_actuator_manifest(alias)
        self.manifest.chmod(0o666)
        with self.assertRaisesRegex(op.DeploymentError, 'group/world writable'):
            op.validate_actuator_manifest(self.manifest)
        self.manifest.chmod(0o600)
        actual_stat = self.manifest.stat()
        with patch.object(op.os, 'geteuid', return_value=0), \
                patch.object(Path, 'stat', return_value=SimpleNamespace(
                    st_mode=actual_stat.st_mode, st_uid=actual_stat.st_uid + 12345)):
            with self.assertRaisesRegex(op.DeploymentError, 'owned by root or the invoking sudo user'):
                op.validate_actuator_manifest(self.manifest)

    def test_copy_failure_cleans_temp_and_preserves_existing_snapshot(self):
        self.preflight()
        self.provision()
        self.context.runner.reset_mock()
        error = OSError('copy failed')
        with patch.object(op.shutil, 'copyfileobj', side_effect=error):
            with self.assertRaisesRegex(OSError, 'copy failed'):
                self.provision()
        self.assertEqual(self.snapshot.read_bytes(), self.original)
        self.assertEqual(list(self.snapshot.parent.iterdir()), [self.snapshot])
        self.context.runner.run.assert_not_called()

    def test_runner_failure_occurs_after_snapshot_publication(self):
        self.preflight()
        self.context.runner.run.side_effect = RuntimeError('provision failed')
        with self.assertRaisesRegex(RuntimeError, 'provision failed'):
            self.provision()
        self.assertEqual(self.snapshot.read_bytes(), self.original)
        self.assertEqual(list(self.snapshot.parent.iterdir()), [self.snapshot])


class PersistentIdentityTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory(prefix='installer-identity-')
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.source = self.root / 'source'
        self.source.mkdir()
        (self.source / 'app.py').write_text('original')
        self.python = self.root / 'python'
        self.python.write_text('fixture interpreter')
        self.profile = replace(profile(), production=False)
        self.options = replace(DeploymentOptions.for_tests(source_root=self.source),
                               install_root=self.root / 'install', config_dir=self.root / 'config',
                               state_dir=self.root / 'state', cache_dir=self.root / 'cache',
                               python_bin=str(self.python), persist_state=True, trust_root=self.source,
                               release_id='base', transaction_id='attempt')

    def capture(self, options=None, bundle=None):
        options = options or self.options
        with patch.object(engine.subprocess, 'run', return_value=SimpleNamespace(returncode=1)), \
                patch.object(engine, 'source_user_command', side_effect=list), \
                patch.object(engine, 'source_user_environment', return_value={}), \
                patch.object(op.hashlib, 'sha256', side_effect=AssertionError('input hash')), \
                patch.object(Path, 'open', side_effect=AssertionError('source payload read')):
            return engine.capture_installation_inputs(self.profile, options, bundle)

    def record(self, owner='owner', inputs=None, release='base'):
        return dict(schema_version=3, profile=self.profile.name, release_id=release,
                    release_identity={'version': 1, 'owner_id': owner},
                    installation_inputs=self.capture() if inputs is None else inputs)

    def write_release(self, record, marker=False):
        target = self.options.releases_dir / record['release_id']
        target.mkdir(parents=True, exist_ok=True)
        if marker:
            record = dict(record, schema_version=1)
        (target / ('.installing' if marker else 'release-manifest.json')).write_text(json.dumps(record))
        return target

    def test_inputs_json_roundtrip_file_add_edit_delete_and_no_payload_reads(self):
        original = self.capture()
        self.assertTrue(state.installation_inputs_equal(original, json.loads(json.dumps(original))))
        file = self.source / 'added.py'
        file.write_text('new')
        added = self.capture()
        self.assertFalse(state.installation_inputs_equal(original, added))
        file.write_text('edited longer')
        self.assertFalse(state.installation_inputs_equal(added, self.capture()))
        file.unlink()
        self.assertTrue(state.installation_inputs_equal(original, self.capture()))
        (self.source / 'app.py').unlink()
        self.assertFalse(state.installation_inputs_equal(original, self.capture()))

    def test_attempt_release_and_execution_flags_not_inputs_dirty_never_reused(self):
        original = self.capture()
        other = replace(self.options, transaction_id='another-attempt', release_id='another-release',
                        no_systemd=True, no_start=True, no_verify=True, require_update_readiness=True)
        self.assertTrue(state.installation_inputs_equal(original, self.capture(other)))
        dirty = self.capture(replace(other, allow_dirty_source=True))
        self.assertFalse(state.installation_inputs_equal(dirty, json.loads(json.dumps(dirty))))
        self.assertTrue(state.installation_inputs_match(dirty, json.loads(json.dumps(dirty))))

    def test_unknown_missing_inputs_and_typed_legacy_identity(self):
        current = self.capture()
        for invalid in (None, {}, {'version': 1}, dict(current, version=2), dict(current, version=True)):
            self.assertFalse(state.installation_inputs_equal(current, invalid))
            self.assertFalse(state.installation_inputs_equal(invalid, invalid))
        self.assertEqual(state.record_identity({'fingerprint': 'same'}), ('legacy-fingerprint', 'same'))
        self.assertNotEqual(state.record_identity({'fingerprint': 'same'}),
                            state.record_identity(self.record(owner='same')))
        self.assertIsNone(state.record_identity(dict(fingerprint='same', installation_inputs=None)))
        self.assertIsNone(state.record_identity({}))

    def test_snapshot_and_signed_bundle_freeze_metadata_not_inputs(self):
        snapshot = replace(self.options, source_is_snapshot=True, source_revision='git-head', source_tree_id='git-tree')
        before = self.capture(snapshot)
        (self.source / 'app.py').chmod(0o440)
        self.assertTrue(state.installation_inputs_equal(before, self.capture(snapshot)))
        bundle = self.root / 'bundle'
        (bundle / 'source').mkdir(parents=True)
        wheel = bundle / 'vendor' / 'sdk.whl'
        wheel.parent.mkdir()
        wheel.write_text('signed fixture')
        bundled = replace(self.options, offline_bundle=bundle, source_root=bundle / 'source',
                          go2_sdk_wheel=wheel)
        verified = {'metadata': {'release_id': 'signed'}, 'files': {'vendor/sdk.whl': {'sha256': 'external-pin'}}}
        before = self.capture(bundled, verified)
        wheel.chmod(0o440)
        (bundle / 'source').chmod(0o550)
        self.assertTrue(state.installation_inputs_equal(before, self.capture(bundled, verified)))
        changed = dict(verified, metadata={'release_id': 'other-signed'})
        self.assertFalse(state.installation_inputs_equal(before, self.capture(bundled, changed)))

    def test_owner_cross_inputs_and_activation_does_not_resample(self):
        inputs = self.capture()
        plan = DeploymentPlan('fixture', '', (), installation_inputs=inputs, release_owner_id='owner')
        context = ExecutionContext(plan, self.profile, self.options, Mock())
        for owner in ('owner', 'other'):
            for same in (True, False):
                record = self.record(owner, inputs if same else dict(inputs, options={'skip_models': True}))
                with self.subTest(owner=owner, same=same):
                    if owner == 'owner' and same:
                        op._require_release_identity(context, record)
                    else:
                        with self.assertRaises(op.DeploymentError):
                            op._require_release_identity(context, record)
        self.write_release(self.record(inputs=inputs))
        with patch.object(engine, 'capture_installation_inputs', side_effect=AssertionError('resample')), \
                patch.object(op, '_switch_current') as switch:
            op.activate_release(context)
        switch.assert_called_once()

    def test_auto_collision_then_latest_stabilizes_and_explicit_signed_conflict(self):
        inputs = self.capture()
        old = {'profile': 'fixture', 'release_id': 'base', 'fingerprint': 'opaque-old'}
        old_path = self.write_release(old)
        old_bytes = (old_path / 'release-manifest.json').read_bytes()
        first, owner = cli._select_release(self.options, self.profile, inputs, {}, explicit=False)
        self.assertNotEqual(first.release_id, 'base')
        self.assertEqual(owner, '')
        record = self.record(inputs=inputs, release=first.release_id)
        self.write_release(record)
        for status in ('committed', 'staged'):
            second, owner = cli._select_release(self.options, self.profile, inputs, dict(record, status=status), explicit=False)
            self.assertEqual(second.release_id, first.release_id)
            self.assertEqual(owner, 'owner')
        self.assertEqual((old_path / 'release-manifest.json').read_bytes(), old_bytes)
        with self.assertRaisesRegex(RuntimeError, 'immutable release id conflict'):
            cli._select_release(self.options, self.profile, inputs, {}, explicit=True)
        with self.assertRaisesRegex(RuntimeError, 'immutable release id conflict'):
            cli._select_release(replace(self.options, offline_bundle=self.root / 'signed'), self.profile, inputs, {}, explicit=True)

    def test_incomplete_owner_preserved_and_completed_manifest_wins_leftover_marker(self):
        record = self.record()
        target = self.write_release(record, marker=True)
        selected, owner = cli._select_release(self.options, self.profile, record['installation_inputs'], {}, explicit=True)
        self.assertEqual(owner, 'owner')
        self.assertEqual(selected.release_id, 'base')
        self.write_release(record)
        (target / '.installing').write_text('opaque-old-leftover')
        selected, owner = cli._select_release(self.options, self.profile, record['installation_inputs'], {}, explicit=True)
        self.assertEqual(owner, 'owner')
        context = ExecutionContext(DeploymentPlan('fixture', '', (), installation_inputs=record['installation_inputs'],
                                                  release_owner_id=owner), self.profile, self.options, Mock())
        with patch.object(op, '_prepare_backup_root'), patch.object(op, '_record_recovery'), \
                patch.object(op, '_require_file', return_value=self.source / 'script'):
            op.install_release(context)
        context.runner.run.assert_called_once()
        self.assertEqual((target / '.installing').read_text(), 'opaque-old-leftover')

    def history(self, *, legacy=True, status='running'):
        record = dict(schema_version=2, generation=1, profile='fixture', release_id='old-release',
                      transaction_id='old-attempt', status=status, started_at=1,
                      host={'hostname': platform.node(), 'machine': platform.machine()})
        if legacy:
            record['fingerprint'] = 'opaque-old'
        else:
            record.update(self.record(release='old-release'), transaction_id='old-attempt', status=status)
        cli._save_transaction_state(self.options.state_dir, record)
        return record

    def test_resume_uses_old_paths_and_never_preannounces_new_attempt(self):
        for legacy in (True, False):
            previous = self.history(legacy=legacy)
            seen = []
            def load(context):
                seen.append((context.options.release_id, context.options.transaction_id,
                             op._transaction_backup_dir(context)))
                return True
            with patch.object(op, 'load_persisted_recovery', side_effect=load), patch.object(op, 'rollback'):
                recovered = cli._recover_for_resume(SimpleNamespace(transaction_id='old-attempt'), self.options,
                                                    self.profile, Mock(), previous)
            self.assertEqual(seen, [('old-release', 'old-attempt', self.options.state_dir / 'backups/old-attempt')])
            self.assertEqual(recovered['status'], 'recovered')
            self.assertNotIn('resumed_by_transaction_id', recovered)
            self.assertEqual(state.record_identity(previous), state.record_identity(recovered))
            self.assertEqual('installation_inputs' in previous, 'installation_inputs' in recovered)

    def test_resume_profile_failure_and_recovery_failure_do_not_new_attempt(self):
        previous = self.history()
        with patch.object(op, 'load_persisted_recovery') as load:
            with self.assertRaisesRegex(RuntimeError, 'resume profile'):
                cli._recover_for_resume(SimpleNamespace(transaction_id='old-attempt'), self.options,
                                        replace(self.profile, name='wrong'), Mock(), previous)
            load.assert_not_called()
        with patch.object(op, 'load_persisted_recovery', side_effect=RuntimeError('restore failed')):
            with self.assertRaisesRegex(RuntimeError, 'restore failed'):
                cli._recover_for_resume(SimpleNamespace(transaction_id='old-attempt'), self.options,
                                        self.profile, Mock(), previous)
        saved = cli._newest_transaction_state(self.options.state_dir)
        self.assertEqual(saved['status'], 'rollback_failed')
        self.assertEqual([p.name for p in (self.options.state_dir / 'transactions').iterdir()], ['old-attempt.json'])

    def test_resume_without_material_supersedes(self):
        previous = self.history()
        with patch.object(op, 'load_persisted_recovery', return_value=False), patch.object(op, 'rollback') as rollback:
            saved = cli._recover_for_resume(SimpleNamespace(transaction_id='old-attempt'), self.options,
                                            self.profile, Mock(), previous)
        self.assertEqual(saved['status'], 'superseded')
        rollback.assert_not_called()
        self.assertNotIn('installation_inputs', saved)

    def test_latest_missing_or_mixed_identity_is_not_equal(self):
        base = dict(profile='fixture', release_id='base', transaction_id='tx')
        for record in (base, dict(base, installation_inputs=None), dict(base, release_identity={'version': 2, 'owner_id': 'x'})):
            with self.assertRaises(RuntimeError):
                cli._assert_latest_transaction(record, record.copy())
        with self.assertRaises(RuntimeError):
            cli._assert_latest_transaction(dict(base, fingerprint='same'), dict(base, **self.record(owner='same')))

    def test_state_owner_resumed_from_and_journal_write_failures(self):
        inputs = self.capture()
        store = state.StateStore(self.options.state_file)
        with self.assertRaisesRegex(RuntimeError, 'requires owner'):
            store.begin('fixture', '', 'base', 'new', [], installation_inputs=inputs)
        self.assertFalse(self.options.state_dir.exists())
        with patch.object(state, 'atomic_write', side_effect=OSError('journal failed')):
            with self.assertRaisesRegex(OSError, 'journal failed'):
                store.begin('fixture', '', 'base', 'new', [], installation_inputs=inputs,
                            release_owner_id='owner', resumed_from_transaction_id='old')
        self.assertFalse(self.options.state_dir.exists())
        store.begin('fixture', '', 'base', 'new', [], installation_inputs=inputs,
                    release_owner_id='owner', resumed_from_transaction_id='old')
        saved = cli._newest_transaction_state(self.options.state_dir)
        self.assertEqual(saved['resumed_from_transaction_id'], 'old')
        self.assertEqual(saved['release_identity']['owner_id'], 'owner')
        self.assertNotIn('fingerprint', saved)
        real = state.atomic_write
        def fail_latest(path, *args, **kwargs):
            if path == self.options.state_file:
                raise OSError('latest failed')
            return real(path, *args, **kwargs)
        with patch.object(state, 'atomic_write', side_effect=fail_latest):
            with self.assertRaisesRegex(OSError, 'latest failed'):
                store.begin('fixture', '', 'base', 'newer', [], installation_inputs=inputs, release_owner_id='owner')
        self.assertEqual(cli._newest_transaction_state(self.options.state_dir)['transaction_id'], 'newer')

    def cli_stack(self, build=None):
        stack = ExitStack()
        stack.enter_context(patch.object(cli, '_options', return_value=self.options))
        stack.enter_context(patch.object(cli, '_load_profile_argument', return_value=self.profile))
        stack.enter_context(patch.object(cli, 'load_named_profile', return_value=self.profile))
        stack.enter_context(patch.object(cli.os, 'geteuid', return_value=0))
        stack.enter_context(patch.object(op, 'validate_protected_paths'))
        stack.enter_context(patch.object(cli, 'InstallerLock', side_effect=lambda: nullcontext()))
        stack.enter_context(patch.object(cli, 'SubprocessRunner', return_value=Mock()))
        stack.enter_context(patch.object(cli, 'capture_installation_inputs', side_effect=lambda p, o, b: self.capture(o, b)))
        if build is not None:
            stack.enter_context(patch.object(cli, 'build_deployment_plan', side_effect=build))
        stack.enter_context(redirect_stdout(io.StringIO()))
        return stack

    def test_cli_recovered_then_conflict_or_new_journal_failure(self):
        self.history()
        self.write_release({'profile': 'fixture', 'release_id': 'base', 'fingerprint': 'old'})
        args = SimpleNamespace(profile='fixture', release_id='base', transaction_id='old-attempt', json=True)
        with self.cli_stack(), patch.object(op, 'load_persisted_recovery', return_value=True), patch.object(op, 'rollback'):
            with self.assertRaisesRegex(RuntimeError, 'old-attempt recovered; new installation not started: immutable'):
                cli._execute(args, dry_run=False, resume=True)
        saved = cli._newest_transaction_state(self.options.state_dir)
        self.assertEqual(saved['status'], 'recovered')
        self.assertNotIn('resumed_by_transaction_id', saved)
        self.history()
        args.release_id = ''
        def build(p, o, **kwargs):
            return DeploymentPlan(p.name, '', (), **kwargs)
        with self.cli_stack(build), patch.object(op, 'load_persisted_recovery', return_value=True), \
                patch.object(op, 'rollback'), patch.object(state, 'atomic_write', side_effect=OSError('new journal failed')):
            with self.assertRaisesRegex(OSError, 'new journal failed'):
                cli._execute(args, dry_run=False, resume=True)
        saved = cli._newest_transaction_state(self.options.state_dir)
        self.assertEqual(saved['status'], 'recovered')
        self.assertNotIn('resumed_by_transaction_id', saved)
        self.assertEqual(len(list((self.options.state_dir / 'transactions').iterdir())), 1)

    def test_cli_staged_to_active_keeps_owner_and_second_auto_deploy_is_unchanged(self):
        self.write_release({'profile': 'fixture', 'release_id': 'base', 'fingerprint': 'legacy'})
        seen = []
        def build(p, o, **kwargs):
            seen.append((o, kwargs))
            def publish(context):
                record = dict(profile=p.name, release_id=o.release_id, **state.plan_identity_fields(context.plan))
                target = self.write_release(record)
                if not o.no_start:
                    if o.current_link.is_symlink():
                        o.current_link.unlink()
                    o.current_link.symlink_to(target)
            return DeploymentPlan(p.name, '', (engine.Phase('release', 'fixture', (engine.Action.callback('publish', publish),)),), **kwargs)
        args = SimpleNamespace(profile='fixture', release_id='', json=True)
        self.options = replace(self.options, no_start=True)
        with self.cli_stack(build):
            self.assertEqual(cli._execute(args, dry_run=False), 0)
        staged = cli._newest_transaction_state(self.options.state_dir)
        self.assertEqual(staged['status'], 'staged')
        self.assertNotEqual(staged['release_id'], 'base')
        self.options = replace(self.options, no_start=False)
        with self.cli_stack(build):
            self.assertEqual(cli._execute(args, dry_run=False), 0)
        active = cli._newest_transaction_state(self.options.state_dir)
        self.assertEqual(active['status'], 'committed')
        self.assertEqual(active['release_identity'], staged['release_identity'])
        self.assertEqual(active['release_id'], staged['release_id'])
        self.assertNotEqual(active['transaction_id'], staged['transaction_id'])
        with self.cli_stack(build):
            self.assertEqual(cli._execute(args, dry_run=False), 0)
        self.assertEqual(cli._newest_transaction_state(self.options.state_dir), active)
        self.assertEqual({o.release_id for o, _ in seen}, {staged['release_id']})

    def test_final_plan_paths_use_selected_target(self):
        build = RustBuild('fixture', 'Cargo.toml', 'fixture', 'bin/fixture')
        current = replace(self.profile, rust=RustProfile('existing', (build,)))
        selected = replace(self.options, release_id='new-selected')
        runner = Mock()
        with patch.object(DeploymentOptions, 'rust_target_dir', lambda o, m: o.build_dir / 'existing-target'), \
                patch.object(engine, '_build_user_rust_paths', return_value=(self.root, self.root, self.root)):
            plan = engine.build_deployment_plan(current, selected, installation_inputs=self.capture(), release_owner_id='owner')
            context = ExecutionContext(plan, current, selected, runner)
            for action in next(p for p in plan.phases if p.name == 'rust_build').actions:
                action.execute(context)
        for call in runner.run.call_args_list:
            self.assertEqual(call.kwargs['env']['CARGO_TARGET_DIR'], str(selected.build_dir / 'existing-target'))
            self.assertEqual(call.kwargs['cwd'], self.source)
            self.assertEqual(call.args[0][0], str(selected.toolchain_bin / 'cargo'))

    def test_legacy_status_and_rollback_ignore_missing_inputs_without_rewriting_them(self):
        previous = self.history(status='failed')
        before = (self.options.state_dir / 'transactions/old-attempt.json').read_bytes()
        with redirect_stdout(io.StringIO()):
            self.assertEqual(cli._status(self.options.state_dir, self.options.install_root), 0)
        self.assertEqual((self.options.state_dir / 'transactions/old-attempt.json').read_bytes(), before)
        args = SimpleNamespace(state_dir=self.options.state_dir, install_root=self.options.install_root,
                               config_dir=self.options.config_dir, cache_dir=self.options.cache_dir,
                               source_root=self.source, transaction_id='old-attempt', to_release=None,
                               no_systemd=True, no_verify=True, json=True)
        contexts = []
        with self.cli_stack(), patch.object(op, 'load_persisted_recovery', side_effect=lambda c: contexts.append(c) or True), \
                patch.object(op, 'rollback'):
            self.assertEqual(cli._rollback_command(args), 0)
        self.assertEqual(contexts[0].options.release_id, 'old-release')
        self.assertEqual(state.record_identity(state.plan_identity_fields(contexts[0].plan)), ('legacy-fingerprint', 'opaque-old'))
        saved = cli._newest_transaction_state(self.options.state_dir)
        self.assertEqual(saved['status'], 'rolled_back')
        self.assertNotIn('installation_inputs', saved)
        self.assertEqual(saved['fingerprint'], previous['fingerprint'])

    def test_plan_and_verify_do_not_create_state_or_sample_inputs(self):
        args = SimpleNamespace(profile='fixture', release_id='', json=True)
        with self.cli_stack(), patch.object(cli, 'capture_installation_inputs', side_effect=AssertionError('input capture')):
            self.assertEqual(cli._execute(args, dry_run=True), 0)
            with patch.object(op, 'verify_deployment') as verify:
                self.assertEqual(cli._execute(args, dry_run=False, verify_only=True), 0)
                verify.assert_called_once()
        self.assertFalse(self.options.state_dir.exists())

    def test_current_switch_crash_after_prepared_and_after_link_recovered_from_facts(self):
        for fault in ('prepared', 'activated'):
            with self.subTest(fault=fault), tempfile.TemporaryDirectory(dir=str(self.root)) as directory:
                options = replace(self.options, install_root=Path(directory) / 'install', state_dir=Path(directory) / 'state')
                options.release_dir.mkdir(parents=True)
                previous = options.releases_dir / 'previous'
                previous.mkdir()
                options.current_link.symlink_to(previous)
                context = ExecutionContext(DeploymentPlan('fixture', 'legacy-owner', ()), self.profile, options, Mock())
                real_record = op._record_recovery
                def record(c, release, *, activated):
                    if fault == 'activated' and activated:
                        raise OSError('activated write interrupted')
                    real_record(c, release, activated=activated)
                real_replace = op.os.replace
                def replace_link(src, dst):
                    if fault == 'prepared' and Path(dst) == options.current_link:
                        raise OSError('link replace interrupted')
                    return real_replace(src, dst)
                with patch.object(op, '_validate_managed_release', return_value=previous), \
                        patch.object(op, '_record_recovery', side_effect=record), \
                        patch.object(op.os, 'replace', side_effect=replace_link):
                    with self.assertRaises(OSError):
                        op._switch_current(context, options.release_dir)
                persisted = json.loads((options.state_dir / 'backups/attempt/recovery.json').read_text())
                self.assertEqual(persisted['activation_state'], 'prepared')
                recovered = ExecutionContext(context.plan, self.profile, options, Mock())
                op.load_persisted_recovery(recovered)
                self.assertEqual(recovered.release_switched, fault == 'activated')
                self.assertEqual(recovered.previous_release, str(previous))

    def test_rolling_back_write_and_restored_link_can_be_retried(self):
        options = replace(self.options, no_systemd=True, no_verify=True)
        options.release_dir.mkdir(parents=True)
        previous = options.releases_dir / 'previous'
        previous.mkdir()
        options.current_link.symlink_to(previous)
        context = ExecutionContext(DeploymentPlan('fixture', 'legacy-owner', ()), self.profile, options, Mock())
        with patch.object(op, '_validate_managed_release', return_value=previous):
            op._switch_current(context, options.release_dir)
        real_fsync = op.fsync_directory
        def fail_after_restore(path):
            if path == options.current_link.parent:
                raise OSError('power loss after restored link')
            return real_fsync(path)
        with patch.object(op, '_validate_managed_release', return_value=previous), \
                patch.object(op, '_deployment_env', return_value={}), \
                patch.object(op, 'fsync_directory', side_effect=fail_after_restore):
            with self.assertRaisesRegex(OSError, 'power loss'):
                op.rollback(context)
        recovery = options.state_dir / 'backups/attempt/recovery.json'
        self.assertEqual(json.loads(recovery.read_text())['activation_state'], 'rolling_back')
        restarted = ExecutionContext(context.plan, self.profile, options, Mock())
        op.load_persisted_recovery(restarted)
        self.assertFalse(restarted.release_switched)
        self.assertEqual(options.current_link.resolve(), previous)

    def test_bundle_cache_returns_existing_verification_result_after_refreeze(self):
        bundle = self.root / 'signed-input'
        bundle.mkdir()
        cache = self.root / 'bundle-cache'
        target = cache / 'verified-bundles' / 'opaque-existing-cache-name'
        target.mkdir(parents=True)
        (target / 'bundle-manifest.json').write_text('fixture metadata')
        first = {'files': {}, 'metadata': {'source_revision': 'before'}}
        final = {'files': {}, 'metadata': {'source_revision': 'verified-cached'}}
        def metadata(_path):
            return SimpleNamespace(st_uid=0, st_mode=0o100440)
        with patch.object(op, 'verify_offline_bundle', side_effect=[first, final]) as verify, \
                patch.object(op, '_sha256', return_value='opaque-existing-cache-name'), \
                patch.object(op, '_prepare_root_managed_cache', side_effect=lambda p, gid: p), \
                patch.object(op, 'resolve_build_account', return_value=SimpleNamespace(pw_gid=os.getgid())), \
                patch.object(Path, 'lstat', metadata), patch.object(op.os, 'chown'), patch.object(op.os, 'chmod'):
            path, retained = op.stage_verified_bundle(bundle, self.root / 'fixture-key', cache)
        self.assertEqual(path, target)
        self.assertIs(retained, final)
        self.assertEqual(verify.call_count, 2)

    def test_current_unknown_input_version_does_not_block_historical_rollback(self):
        previous = self.history(legacy=False, status='failed')
        previous['installation_inputs'] = {'version': 999}
        cli._save_transaction_state(self.options.state_dir, previous)
        plan = cli._plan_for_history(previous)
        self.assertEqual(plan.release_owner_id, 'owner')
        cli._assert_latest_transaction(previous, cli._newest_transaction_state(self.options.state_dir))
        self.assertFalse(state.installation_inputs_equal(previous['installation_inputs'], self.capture()))


    def test_target_selection_does_not_adopt_changed_owner_for_same_latest_release(self):
        inputs = self.capture()
        record = self.record(inputs=inputs)
        self.write_release(record)
        latest = dict(self.record(owner='different-owner', inputs=inputs), status='committed')
        selected, owner = cli._select_release(self.options, self.profile, inputs, latest, explicit=False)
        self.assertNotEqual(selected.release_id, 'base')
        self.assertEqual(owner, '')
        with self.assertRaisesRegex(RuntimeError, 'immutable release id conflict'):
            cli._select_release(self.options, self.profile, inputs, latest, explicit=True)


    def test_resume_rechecks_latest_after_preparation_before_starting_attempt(self):
        self.history()
        args = SimpleNamespace(profile='fixture', release_id='', transaction_id='old-attempt', json=True)
        def prepare(*unused, **kwargs):
            newer = dict(self.record(), generation=2, transaction_id='other-attempt', status='committed', started_at=2)
            cli._save_transaction_state(self.options.state_dir, newer)
            return self.options, self.profile, None
        with self.cli_stack(), patch.object(op, 'load_persisted_recovery', return_value=True), \
                patch.object(op, 'rollback'), patch.object(cli, '_prepare_source', side_effect=prepare), \
                patch.object(cli, 'build_deployment_plan') as build:
            with self.assertRaisesRegex(RuntimeError, 'stale'):
                cli._execute(args, dry_run=False, resume=True)
            build.assert_not_called()
        self.assertEqual(cli._newest_transaction_state(self.options.state_dir)['transaction_id'], 'other-attempt')

    def test_empty_release_after_mkdir_before_marker_retries_same_target(self):
        for mode in ('explicit', 'signed', 'automatic'):
            with self.subTest(mode=mode):
                options = replace(self.options, release_id='empty-' + mode,
                                  offline_bundle=self.root / 'signed-fixture' if mode == 'signed' else None)
                inputs = self.capture(options)
                first = DeploymentPlan('fixture', '', (), installation_inputs=inputs, release_owner_id='interrupted-attempt')
                context = ExecutionContext(first, self.profile, options, Mock())
                def mkdir(c):
                    c.options.release_dir.mkdir(parents=True, exist_ok=True)
                    return c.options.release_dir
                with patch.object(op, 'resolve_build_account', return_value=SimpleNamespace(pw_uid=os.getuid(), pw_gid=os.getgid())), \
                        patch.object(op, '_prepare_build_root'), patch.object(op, '_prepare_release_root', side_effect=mkdir), \
                        patch.object(op, 'atomic_write', side_effect=OSError('power loss before marker write')):
                    with self.assertRaisesRegex(OSError, 'power loss before marker write'):
                        op.install_python_environment(context)
                self.assertTrue(options.release_dir.is_dir())
                self.assertEqual(list(options.release_dir.iterdir()), [])
                selected, owner = cli._select_release(options, self.profile, inputs, {}, explicit=mode != 'automatic')
                self.assertEqual(selected.release_id, options.release_id)
                self.assertEqual(owner, '')  # No artifact ownership was published before the crash.
                retry = ExecutionContext(replace(first, release_owner_id='retry-attempt'), self.profile, selected, Mock())
                with patch.object(op, 'resolve_build_account', return_value=SimpleNamespace(pw_uid=os.getuid(), pw_gid=os.getgid())), \
                        patch.object(op, '_prepare_build_root'), patch.object(op, '_prepare_release_root', side_effect=mkdir), \
                        patch.object(op, '_prepare_mutable_build_tree', side_effect=PlanReached):
                    with self.assertRaises(PlanReached):
                        op.install_python_environment(retry)
                marker = op._read_installing_record(selected.release_dir / '.installing')
                self.assertEqual(marker['release_identity']['owner_id'], 'retry-attempt')
                self.assertEqual(marker['installation_inputs'], inputs)
                retry.runner.run.assert_not_called()

    def test_unmarked_nonempty_and_non_directory_targets_remain_occupied(self):
        inputs = self.capture()
        self.options.release_dir.mkdir(parents=True)
        (self.options.release_dir / 'partial').write_text('partial artifact')
        with self.assertRaisesRegex(RuntimeError, 'immutable release id conflict'):
            cli._select_release(self.options, self.profile, inputs, {}, explicit=True)
        other, _ = cli._select_release(self.options, self.profile, inputs, {}, explicit=False)
        self.assertNotEqual(other.release_id, self.options.release_id)
        for mode in ('file', 'symlink'):
            options = replace(self.options, release_id=mode)
            if mode == 'file':
                options.release_dir.write_text('not a directory')
            else:
                options.release_dir.symlink_to(self.root / 'missing')
            with self.assertRaisesRegex(RuntimeError, 'immutable release id conflict'):
                cli._select_release(options, self.profile, inputs, {}, explicit=True)

    def rust_context(self, builds=None):
        builds = builds or (RustBuild('nav', 'Cargo.toml', 'nav', 'bin/nav'),)
        current = replace(self.profile, rust=RustProfile('existing', builds))
        options = replace(self.options, no_systemd=True)
        plan = DeploymentPlan('fixture', '', (), installation_inputs=self.capture(), release_owner_id='owner')
        return ExecutionContext(plan, current, options, Mock())

    def prepare_targets(self, context):
        with patch.object(op.os, 'chown'), patch.object(op.hashlib, 'sha256', side_effect=AssertionError('target hash')):
            op._prepare_rust_targets(context, os.getuid(), os.getgid())

    def test_legacy_target_plan_verify_status_and_query_are_read_only(self):
        self.profile = self.rust_context().profile
        args = SimpleNamespace(profile='fixture', release_id='', json=True)
        with self.cli_stack(), patch.object(Path, 'mkdir', side_effect=AssertionError('query created directory')), \
                patch.object(op, 'atomic_write', side_effect=AssertionError('query wrote record')), \
                patch.object(op.hashlib, 'sha256', side_effect=AssertionError('target hash')):
            self.assertEqual(cli._execute(args, dry_run=True), 0)
            with patch.object(op, 'verify_deployment'):
                self.assertEqual(cli._execute(args, dry_run=False, verify_only=True), 0)
            self.assertEqual(cli._status(self.options.state_dir, self.options.install_root), 1)
            with self.assertRaisesRegex(RuntimeError, 'has not been prepared'):
                self.options.rust_target_dir('Cargo.toml')
        self.assertFalse(self.options.cache_dir.exists())
        self.assertFalse(self.options.state_dir.exists())
        context = self.rust_context()
        self.prepare_targets(context)
        record = self.options.build_dir / 'rust-targets/target-paths-v1.json'
        saved = record.read_bytes()
        with patch.object(Path, 'mkdir', side_effect=AssertionError('query mkdir')), \
                patch.object(op, 'atomic_write', side_effect=AssertionError('query write')):
            self.assertTrue(self.options.rust_target_dir('Cargo.toml').is_dir())
        self.assertEqual(record.read_bytes(), saved)

    def test_legacy_targets_share_manifest_retry_stable_and_copy_actual_built_files(self):
        builds = (RustBuild('nav', 'Cargo.toml', 'nav', 'bin/nav'),
                  RustBuild('follow', 'Cargo.toml', 'follow', 'bin/follow'),
                  RustBuild('geometry', 'geometry/Cargo.toml', 'nuwax-geometry-runtime',
                            'services/geometry/nuwax_geometry_runtime/target/release/nuwax-geometry-runtime'))
        context = self.rust_context(builds)
        self.prepare_targets(context)
        workspace = self.options.rust_target_dir('Cargo.toml')
        geometry = self.options.rust_target_dir('geometry/Cargo.toml')
        self.assertNotEqual(workspace, geometry)
        with patch.object(op.uuid, 'uuid4', side_effect=AssertionError('reallocation')):
            self.prepare_targets(context)
        calls = []
        def run(argv, **kwargs):
            target = Path(kwargs['env']['CARGO_TARGET_DIR'])
            calls.append((argv, target))
            self.assertEqual(kwargs['env']['CARGO_INCREMENTAL'], '0')
            if 'build' in argv:
                binary = argv[argv.index('--bin') + 1]
                output = target / 'release' / binary
                output.parent.mkdir(parents=True, exist_ok=True)
                output.write_bytes(binary.encode())
                output.chmod(0o755)
        context.runner.run.side_effect = run
        with patch.object(engine, '_build_user_rust_paths', return_value=(self.root, self.root, self.root)):
            plan = engine.build_deployment_plan(context.profile, context.options,
                                                installation_inputs=context.plan.installation_inputs, release_owner_id='owner')
            for action in next(p for p in plan.phases if p.name == 'rust_build').actions:
                action.execute(context)
        self.assertEqual([target for argv, target in calls], [workspace, geometry, workspace, workspace, geometry])
        python = context.options.release_dir / '.venv/bin/python'
        python.parent.mkdir(parents=True)
        python.write_text('fixture')
        context.runner.reset_mock(side_effect=True)
        # The real source-copy step normalizes these directories; this fixture
        # substitutes that step but keeps the actual output-copy permission gate.
        for build in builds:
            parent = (context.options.release_dir / build.output).parent
            parent.mkdir(parents=True, exist_ok=True)
            parent.chmod(0o755)
        with patch.object(op, '_copy_release_source'), patch.object(op, '_source_revision', return_value='fixture'), \
                patch.object(op, '_validate_release_binary', return_value='fixture binary'), \
                patch.object(op, 'resolve_build_account', return_value=SimpleNamespace(pw_uid=os.getuid())):
            op.install_release(context)
        for build in builds:
            destination = context.options.release_dir / build.output
            self.assertEqual(destination.read_bytes(), build.binary.encode())
        self.assertEqual(context.runner.run.call_count, 0)

    def test_legacy_target_build_failure_reuses_published_path(self):
        context = self.rust_context()
        self.prepare_targets(context)
        target = self.options.rust_target_dir('Cargo.toml')
        context.runner.run.side_effect = RuntimeError('fixture build failure')
        with patch.object(engine, '_build_user_rust_paths', return_value=(self.root, self.root, self.root)):
            action = engine.build_deployment_plan(context.profile, context.options).phases[4].actions[-1]
            with self.assertRaisesRegex(RuntimeError, 'fixture build failure'):
                action.execute(context)
        with patch.object(op.uuid, 'uuid4', side_effect=AssertionError('reallocation')):
            self.prepare_targets(context)
        self.assertEqual(self.options.rust_target_dir('Cargo.toml'), target)

    def test_legacy_target_history_is_not_scanned_adopted_or_deleted(self):
        context = self.rust_context()
        parent = self.options.build_dir / 'rust-targets'
        old = parent / '0123456789abcdef'
        old.mkdir(parents=True)
        (old / 'user-data').write_text('keep exactly')
        real_iterdir = Path.iterdir
        def entries(path):
            if path == parent:
                raise AssertionError('scanned historical targets')
            return real_iterdir(path)
        with patch.object(Path, 'iterdir', entries), patch.object(op.shutil, 'rmtree', side_effect=AssertionError('deleted target')):
            self.prepare_targets(context)
        self.assertNotEqual(self.options.rust_target_dir('Cargo.toml'), old)
        self.assertEqual((old / 'user-data').read_text(), 'keep exactly')

    def test_legacy_target_missing_directory_recreated_same_path_and_empty_recreation_retries(self):
        context = self.rust_context()
        self.prepare_targets(context)
        target = self.options.rust_target_dir('Cargo.toml')
        record = target.parent / 'target-paths-v1.json'
        original = record.read_bytes()
        (target / 'neh-target-owner.json').unlink()
        target.rmdir()
        self.assertEqual(self.options.rust_target_dir('Cargo.toml'), target)
        self.assertFalse(target.exists())
        with patch.object(op.uuid, 'uuid4', side_effect=AssertionError('reallocation')), \
                patch.object(op, 'atomic_write', side_effect=OSError('owner publication failed')):
            with self.assertRaisesRegex(OSError, 'owner publication failed'):
                self.prepare_targets(context)
        self.assertEqual(list(target.iterdir()), [])
        with patch.object(op.uuid, 'uuid4', side_effect=AssertionError('reallocation')):
            self.prepare_targets(context)
        self.assertEqual(self.options.rust_target_dir('Cargo.toml'), target)
        self.assertEqual(record.read_bytes(), original)

    def test_legacy_target_atomic_owner_crash_retries_bound_directory(self):
        # A child exits without atomic_write's finally cleanup: actual mkstemp
        # leftovers, including an unfinished write, survive in the bound target.
        child = '''
import os, sys
from pathlib import Path
from scripts.deploy_installer import config
path, content, stage = sys.argv[1:]
original = config.tempfile.mkstemp
def interrupted_temp(*args, **kwargs):
    fd, name = original(*args, **kwargs)
    if stage == 'partial':
        os.write(fd, b'{"ver')
    os._exit(23)
if stage == 'before_replace':
    config.os.replace = lambda *args: os._exit(23)
else:
    config.tempfile.mkstemp = interrupted_temp
config.atomic_write(Path(path), content, mode=0o600)
'''
        for stage in ('created', 'partial', 'before_replace'):
            with self.subTest(stage=stage):
                context = self.rust_context()
                context.options = replace(self.options, release_id='crash-' + stage)
                self.prepare_targets(context)
                target = context.options.rust_target_dir('Cargo.toml')
                owner = target / 'neh-target-owner.json'
                record = target.parent / 'target-paths-v1.json'
                saved = record.read_bytes()
                owner.unlink()
                target.rmdir()
                def crash(path, content, **kwargs):
                    result = op.subprocess.run([sys.executable, '-c', child, str(path), content, stage],
                                               cwd=str(Path(__file__).resolve().parents[1]), check=False)
                    self.assertEqual(result.returncode, 23)
                    raise OSError('simulated process interruption')
                with patch.object(op, 'atomic_write', side_effect=crash):
                    with self.assertRaisesRegex(OSError, 'simulated process interruption'):
                        self.prepare_targets(context)
                leftovers = {p: p.read_bytes() for p in target.iterdir()}
                self.assertEqual(len(leftovers), 1)
                self.assertFalse(owner.exists())
                with self.assertRaisesRegex(RuntimeError, str(owner)):
                    context.options.rust_target_dir('Cargo.toml')  # Reads never repair.
                with patch.object(op.uuid, 'uuid4', side_effect=AssertionError('reallocation')):
                    self.prepare_targets(context)
                    self.prepare_targets(context)
                self.assertEqual(context.options.rust_target_dir('Cargo.toml'), target)
                self.assertEqual(record.read_bytes(), saved)
                self.assertEqual({p: p.read_bytes() for p in leftovers}, leftovers)

    def test_legacy_target_owner_retry_rejects_unknown_nonempty_layout(self):
        for kind in ('payload', 'directory', 'symlink', 'mode', 'foreign_uid', 'wrong_name', 'hardlink'):
            with self.subTest(kind=kind):
                context = self.rust_context()
                context.options = replace(self.options, release_id='conflict-' + kind)
                self.prepare_targets(context)
                target = context.options.rust_target_dir('Cargo.toml')
                owner = target / 'neh-target-owner.json'
                owner.unlink()
                fd, name = tempfile.mkstemp(prefix='.neh-target-owner.json.', dir=str(target))
                os.close(fd)
                pending = Path(name)
                if kind == 'payload':
                    (target / 'build-output').write_text('preserve')
                elif kind == 'directory':
                    pending.unlink()
                    pending.mkdir()
                elif kind == 'symlink':
                    pending.unlink()
                    pending.symlink_to(self.python)
                elif kind == 'mode':
                    pending.chmod(0o644)
                elif kind == 'wrong_name':
                    pending.rename(target / '.neh-target-owner.json.not-an-atomic-write')
                elif kind == 'hardlink':
                    os.link(pending, self.root / 'outside-link')
                original_lstat = Path.lstat
                def info(path):
                    result = original_lstat(path)
                    if kind == 'foreign_uid' and path == pending:
                        return SimpleNamespace(st_mode=result.st_mode, st_uid=result.st_uid + 1,
                                               st_nlink=result.st_nlink)
                    return result
                names = sorted(p.name for p in target.iterdir())
                with patch.object(Path, 'lstat', info), \
                        patch.object(op.uuid, 'uuid4', side_effect=AssertionError('reallocation')), \
                        patch.object(op.shutil, 'rmtree', side_effect=AssertionError('deletion')):
                    with self.assertRaisesRegex(RuntimeError, str(owner)):
                        self.prepare_targets(context)
                self.assertFalse(owner.exists())
                self.assertEqual(sorted(p.name for p in target.iterdir()), names)

    def test_legacy_target_corrupt_mapping_reports_path_without_reallocation(self):
        context = self.rust_context()
        self.prepare_targets(context)
        record = self.options.build_dir / 'rust-targets/target-paths-v1.json'
        original = record.read_text()
        identity = str((self.source / 'Cargo.toml').resolve())
        for payload in ('bad json', json.dumps({'version': True, 'slots': {}}),
                        json.dumps({'version': 1, 'slots': {identity: {'manifest': identity, 'path': '../outside'}}})):
            with self.subTest(payload=payload):
                record.write_text(payload)
                with patch.object(op.uuid, 'uuid4', side_effect=AssertionError('reallocation')):
                    with self.assertRaisesRegex(RuntimeError, str(record)):
                        self.prepare_targets(context)
                self.assertEqual(record.read_text(), payload)
        record.write_text(original)

    def test_legacy_target_owner_missing_or_wrong_and_uid_mismatch_preserve_directory(self):
        context = self.rust_context()
        self.prepare_targets(context)
        target = self.options.rust_target_dir('Cargo.toml')
        owner = target / 'neh-target-owner.json'
        original = owner.read_text()
        (target / 'build-output').write_text('keep')
        for value in (None, '{"version":1,"manifest":"other"}'):
            if value is None:
                owner.unlink()
            else:
                owner.write_text(value)
            with patch.object(op.shutil, 'rmtree', side_effect=AssertionError('delete')), \
                    patch.object(op.uuid, 'uuid4', side_effect=AssertionError('reallocation')):
                with self.assertRaisesRegex(RuntimeError, str(owner)):
                    self.prepare_targets(context)
            self.assertEqual((target / 'build-output').read_text(), 'keep')
        owner.write_text(original)
        actual_lstat = Path.lstat
        def info(path):
            result = actual_lstat(path)
            return SimpleNamespace(st_mode=result.st_mode, st_uid=result.st_uid + 1) if path == target else result
        with patch.object(Path, 'lstat', info), patch.object(op.shutil, 'rmtree', side_effect=AssertionError('delete')):
            with self.assertRaisesRegex(op.DeploymentError, 'another build user; preserved'):
                self.prepare_targets(context)
        self.assertEqual((target / 'build-output').read_text(), 'keep')

    def test_legacy_target_owner_and_mapping_publication_failures_leave_unbound_directory(self):
        for fail_name in ('neh-target-owner.json', 'target-paths-v1.json'):
            with self.subTest(fail_name=fail_name):
                options = replace(self.options, release_id=fail_name.replace('.', '-'))
                context = self.rust_context()
                context.options = options
                real_write = op.atomic_write
                def write(path, *args, **kwargs):
                    if path.name == fail_name:
                        raise OSError('publication failed')
                    return real_write(path, *args, **kwargs)
                with patch.object(op, 'atomic_write', side_effect=write):
                    with self.assertRaisesRegex(OSError, 'publication failed'):
                        self.prepare_targets(context)
                parent = options.build_dir / 'rust-targets'
                self.assertFalse((parent / 'target-paths-v1.json').exists())
                orphans = list(parent.iterdir())
                self.assertEqual(len(orphans), 1)
                self.prepare_targets(context)
                self.assertNotEqual(options.rust_target_dir('Cargo.toml'), orphans[0])
                self.assertTrue(orphans[0].is_dir())

    def test_legacy_target_mapping_write_reported_failure_after_replace_reuses_binding(self):
        context = self.rust_context()
        real_write = op.atomic_write
        def write(path, *args, **kwargs):
            real_write(path, *args, **kwargs)
            if path.name == 'target-paths-v1.json':
                raise OSError('reported fsync failure')
        with patch.object(op, 'atomic_write', side_effect=write):
            with self.assertRaisesRegex(OSError, 'reported fsync failure'):
                self.prepare_targets(context)
        target = self.options.rust_target_dir('Cargo.toml')
        with patch.object(op.uuid, 'uuid4', side_effect=AssertionError('reallocation')):
            self.prepare_targets(context)
        self.assertEqual(self.options.rust_target_dir('Cargo.toml'), target)

    def test_legacy_target_name_collision_never_adopts_even_empty_directory(self):
        context = self.rust_context()
        parent = self.options.build_dir / 'rust-targets'
        existing = parent / 'rust-fixed'
        existing.mkdir(parents=True)
        with patch.object(op.uuid, 'uuid4', return_value=SimpleNamespace(hex='fixed')):
            with self.assertRaises(FileExistsError):
                self.prepare_targets(context)
        self.assertEqual(list(existing.iterdir()), [])
        self.assertFalse((parent / 'target-paths-v1.json').exists())

    def test_legacy_target_toolchain_preparation_publishes_before_build_home(self):
        context = self.rust_context()
        account = SimpleNamespace(pw_uid=os.getuid(), pw_gid=os.getgid())
        with patch.object(op, 'resolve_build_account', return_value=account), \
                patch.object(op, '_prepare_build_root'), patch.object(op.os, 'chown'), \
                patch.object(op, '_prepare_mutable_build_tree', side_effect=PlanReached) as mutable:
            with self.assertRaises(PlanReached):
                op.ensure_rust_toolchain(context)
        self.assertEqual(mutable.call_args.args[0], context.options.build_home)
        self.assertTrue(self.options.rust_target_dir('Cargo.toml').is_dir())
        context.runner.run.assert_not_called()


    def test_legacy_target_new_manifest_failure_preserves_existing_binding(self):
        context = self.rust_context()
        self.prepare_targets(context)
        original_target = self.options.rust_target_dir('Cargo.toml')
        record = original_target.parent / 'target-paths-v1.json'
        original_record = record.read_bytes()
        context.profile = replace(context.profile, rust=RustProfile('existing', (
            *context.profile.rust.builds, RustBuild('geometry', 'geometry/Cargo.toml', 'geometry', 'bin/geometry'),
        )))
        real_write = op.atomic_write
        def write(path, *args, **kwargs):
            if path == record:
                raise OSError('new binding publication failed')
            return real_write(path, *args, **kwargs)
        with patch.object(op, 'atomic_write', side_effect=write):
            with self.assertRaisesRegex(OSError, 'new binding publication failed'):
                self.prepare_targets(context)
        self.assertEqual(record.read_bytes(), original_record)
        self.assertEqual(self.options.rust_target_dir('Cargo.toml'), original_target)
        self.assertTrue(original_target.is_dir())

    def test_legacy_target_offline_actions_use_signed_cargo_and_same_binding(self):
        context = self.rust_context()
        context.options = replace(context.options, offline_bundle=self.root / 'signed-fixture')
        self.prepare_targets(context)
        target = context.options.rust_target_dir('Cargo.toml')
        with patch.object(engine, '_build_user_rust_paths', return_value=(self.root, self.root, self.root)):
            plan = engine.build_deployment_plan(context.profile, context.options)
            for action in next(p for p in plan.phases if p.name == 'rust_build').actions:
                action.execute(context)
        self.assertEqual(context.runner.run.call_count, 2)
        for call in context.runner.run.call_args_list:
            self.assertEqual(call.args[0][0], str(context.options.offline_bundle / 'cargo-bin/cargo'))
            self.assertEqual(call.kwargs['env']['CARGO_TARGET_DIR'], str(target))
            self.assertEqual(call.kwargs['env']['CARGO_INCREMENTAL'], '0')
            self.assertTrue(call.kwargs['network_isolated'])


class SourceSnapshotTests(unittest.TestCase):
    def setUp(self):
        from scripts.deploy_installer import bundle
        self.bundle = bundle
        directory = tempfile.TemporaryDirectory(prefix='installer-snapshot-')
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.source = self.root / 'checkout'
        self.source.mkdir()
        self.cache = self.root / 'cache'
        self.env = dict(os.environ, GIT_CONFIG_GLOBAL='/dev/null', GIT_CONFIG_NOSYSTEM='1',
                        GIT_AUTHOR_NAME='Snapshot fixture', GIT_AUTHOR_EMAIL='fixture@example.invalid',
                        GIT_COMMITTER_NAME='Snapshot fixture', GIT_COMMITTER_EMAIL='fixture@example.invalid')
        self.git('init', '-q')
        (self.source / 'app.py').write_text('print("source")\n')
        (self.source / 'nested').mkdir()
        (self.source / 'nested/run.sh').write_text('#!/bin/sh\nexit 0\n')
        (self.source / 'nested/run.sh').chmod(0o755)
        self.commit()
        self.revision = self.git('rev-parse', 'HEAD').strip()
        self.tree = self.git('rev-parse', self.revision + '^{tree}').strip()
        self.target = self.cache / 'source-snapshots/v2' / (self.revision + '-' + self.tree[:16])
        self.account = SimpleNamespace(pw_gid=os.getgid())
        self.deleted = []
        stack = ExitStack()
        self.addCleanup(stack.close)
        def prepare_cache(path, gid):
            path.mkdir(parents=True, exist_ok=True)
            return path
        real_lstat = Path.lstat
        def cache_stat(path, *args, **kwargs):
            result = real_lstat(path, *args, **kwargs)
            if path == self.cache or self.cache in path.parents:
                values = list(result)
                values[4] = 0  # Root ownership double; real modes/layout are exercised.
                return os.stat_result(values)
            return result
        real_remove = op.shutil.rmtree
        def remove(path, *args, **kwargs):
            path = Path(path)
            self.assertTrue(self.root in path.parents)
            self.deleted.append(path)
            # Production is root. Let this unprivileged fixture remove its own
            # genuinely read-only staging directories after a failed seal.
            for current, directories, files in os.walk(str(path)):
                os.chmod(current, 0o700)
            return real_remove(path, *args, **kwargs)
        stack.enter_context(patch.object(op, '_prepare_root_managed_cache', side_effect=prepare_cache))
        stack.enter_context(patch.object(op, 'resolve_build_account', return_value=self.account))
        stack.enter_context(patch.object(op.os, 'chown'))
        self.fchown = stack.enter_context(patch.object(op.os, 'fchown'))
        stack.enter_context(patch.object(Path, 'lstat', cache_stat))
        stack.enter_context(patch.object(op.shutil, 'rmtree', side_effect=remove))
        stack.enter_context(patch.object(op, '_tree_digest', side_effect=AssertionError('local tree digest')))
        stack.enter_context(patch.object(op, '_sha256', side_effect=AssertionError('local payload digest')))

    def git(self, *args):
        return op.subprocess.check_output(['git', '-c', 'core.hooksPath=/dev/null', *args],
                                          cwd=str(self.source), env=self.env, text=True)

    def commit(self):
        self.git('add', '-A')
        self.git('commit', '-qm', 'fixture')

    def stage(self):
        return op.stage_source_snapshot(self.source, self.cache)

    def test_snapshot_archive_layout_fixed_git_identity_and_external_record(self):
        with patch.object(op.subprocess, 'run', wraps=op.subprocess.run) as run, \
                patch.object(Path, 'rglob', side_effect=AssertionError('extra creation walk')):
            source, revision, tree = self.stage()
        self.assertEqual((source, revision, tree), (self.target / 'source', self.revision, self.tree))
        commands = [call.args[0] for call in run.call_args_list]
        self.assertIn(['git', 'rev-parse', self.revision + '^{tree}'], commands)
        self.assertNotIn(['git', 'rev-parse', 'HEAD^{tree}'], commands)
        self.assertEqual((source / 'app.py').read_bytes(), (self.source / 'app.py').read_bytes())
        self.assertFalse((source / 'snapshot.json').exists())
        record = json.loads((self.target / 'snapshot.json').read_text())
        self.assertEqual(record, {'schema_version': 2, 'revision': revision, 'tree_id': tree, 'layout': {
            'app.py': {'type': 'file', 'size': (self.source / 'app.py').stat().st_size, 'executable': False},
            'nested': {'type': 'directory'},
            'nested/run.sh': {'type': 'file', 'size': (self.source / 'nested/run.sh').stat().st_size, 'executable': True},
        }})
        self.assertEqual(source.stat().st_mode & 0o777, 0o550)
        self.assertEqual((source / 'app.py').stat().st_mode & 0o777, 0o440)
        self.assertEqual((source / 'nested/run.sh').stat().st_mode & 0o777, 0o550)

    def test_snapshot_hit_has_one_existing_walk_no_payload_read_or_refreeze(self):
        source, revision, tree = self.stage()
        original_open = Path.open
        original_walk = Path.rglob
        walks = []
        def guarded_open(path, *args, **kwargs):
            if path == source or source in path.parents or path == self.source or self.source in path.parents:
                raise AssertionError('source payload opened on hit')
            return original_open(path, *args, **kwargs)
        def walk(path, *args, **kwargs):
            walks.append(path)
            return original_walk(path, *args, **kwargs)
        with patch.object(Path, 'open', guarded_open), patch.object(Path, 'rglob', walk), \
                patch.object(op, '_freeze_root_tree', side_effect=AssertionError('redundant freeze')), \
                patch.object(self.bundle, '_copy_source_snapshot', side_effect=AssertionError('archive on hit')):
            self.assertEqual(self.stage(), (source, revision, tree))
        self.assertEqual(walks, [self.target])

    def test_snapshot_modes_are_repaired_by_existing_freeze(self):
        source, revision, tree = self.stage()
        (source / 'app.py').chmod(0o400)
        (source / 'nested/run.sh').chmod(0o500)
        (source / 'nested').chmod(0o500)
        (self.target / 'snapshot.json').chmod(0o400)
        self.target.chmod(0o500)
        with patch.object(op, '_freeze_root_tree', wraps=op._freeze_root_tree) as freeze:
            self.assertEqual(self.stage(), (source, revision, tree))
            freeze.assert_called_once_with(self.target, self.account.pw_gid)
        self.assertEqual((source / 'app.py').stat().st_mode & 0o777, 0o440)
        self.assertEqual((source / 'nested/run.sh').stat().st_mode & 0o777, 0o550)
        self.assertEqual((source / 'nested').stat().st_mode & 0o777, 0o550)
        self.assertEqual(self.target.stat().st_mode & 0o777, 0o550)
        # Owner-writable and changed executable modes were repairable before;
        # archive executability must not add a new rejection policy.
        (source / 'app.py').chmod(0o700)
        self.stage()
        self.assertEqual((source / 'app.py').stat().st_mode & 0o777, 0o550)

    def test_snapshot_changed_build_gid_uses_existing_permission_path(self):
        source, revision, tree = self.stage()
        self.account.pw_gid += 1000
        self.fchown.reset_mock()
        with patch.object(op, '_freeze_root_tree', wraps=op._freeze_root_tree) as freeze:
            self.assertEqual(self.stage(), (source, revision, tree))
            freeze.assert_called_once_with(self.target, self.account.pw_gid)
        self.assertTrue(self.fchown.call_args_list)
        self.assertTrue(all(call.args[1:] == (0, self.account.pw_gid) for call in self.fchown.call_args_list))

    def test_snapshot_tree_is_pinned_when_head_moves_before_export(self):
        previous = self.revision
        (self.source / 'app.py').write_text('print("changed")\n')
        self.commit()
        real_run = op.subprocess.run
        first = True
        commands = []
        def run(argv, *args, **kwargs):
            nonlocal first
            commands.append(argv)
            if first and argv == ['git', 'rev-parse', 'HEAD']:
                first = False
                return SimpleNamespace(returncode=0, stdout=previous + '\n')
            return real_run(argv, *args, **kwargs)
        with patch.object(op.subprocess, 'run', side_effect=run):
            with self.assertRaisesRegex(op.DeploymentError, 'revision does not match Git HEAD'):
                self.stage()
        self.assertIn(['git', 'rev-parse', previous + '^{tree}'], commands)
        self.assertNotIn(['git', 'rev-parse', 'HEAD^{tree}'], commands)
        self.assertFalse(self.target.exists())

    def test_snapshot_legacy_directories_and_sidecars_are_untouched(self):
        parent = self.cache / 'source-snapshots'
        legacy = parent / (self.revision + '-' + self.tree[:16])
        legacy.mkdir(parents=True)
        (legacy / 'unrecorded-user-file').write_text('keep without sidecar')
        other = parent / 'old-snapshot'
        other.mkdir()
        (other / 'original').write_text('keep with sidecar')
        sidecar = parent / 'old-snapshot.json'
        sidecar.write_text('{"schema_version":1,"tree_sha256":"opaque historical value"}')
        saved = sidecar.read_bytes()
        source, _, _ = self.stage()
        self.assertNotEqual(source, legacy)
        self.assertEqual((legacy / 'unrecorded-user-file').read_text(), 'keep without sidecar')
        self.assertFalse((parent / (legacy.name + '.json')).exists())
        self.assertEqual(sidecar.read_bytes(), saved)
        self.assertEqual((other / 'original').read_text(), 'keep with sidecar')
        self.assertEqual(self.deleted, [])

    def test_snapshot_generated_names_rejected_only_for_local_layout(self):
        for index, relative in enumerate(('__pycache__/tracked', 'compiled.pyc', 'package.egg-info/PKG-INFO')):
            with self.subTest(relative=relative):
                item = self.source / relative
                item.parent.mkdir(parents=True, exist_ok=True)
                item.write_text('tracked fixture')
                self.commit()
                revision = self.git('rev-parse', 'HEAD').strip()
                with self.assertRaisesRegex(op.DeploymentError, 'generated build/import state'):
                    self.stage()
                destination = self.root / ('bundle-default-' + str(index))
                self.assertIsNone(self.bundle._copy_source_snapshot(self.source, destination, revision))
                self.assertEqual((destination / relative).read_text(), 'tracked fixture')
                self.git('rm', '-q', relative)
                self.commit()

    def assert_staging_failure(self, point):
        self.target.parent.mkdir(parents=True)
        orphan = self.target.parent / '.unrelated-old.staging-leftover'
        orphan.mkdir()
        (orphan / 'keep').write_text('historical')
        original_copy = self.bundle._copy_source_snapshot
        original_freeze = op._freeze_root_tree
        original_replace = op.os.replace
        def copy(*args, **kwargs):
            original_copy(*args, **kwargs)
            raise self.bundle.BundleError('fixture export failure')
        def freeze(*args, **kwargs):
            original_freeze(*args, **kwargs)
            raise OSError('fixture seal failure')
        def publish(src, dst):
            if Path(dst) == self.target:
                raise OSError('fixture publish failure')
            return original_replace(src, dst)
        with ExitStack() as stack:
            if point == 'export':
                stack.enter_context(patch.object(self.bundle, '_copy_source_snapshot', side_effect=copy))
            elif point == 'seal':
                stack.enter_context(patch.object(op, '_freeze_root_tree', side_effect=freeze))
            elif point == 'record':
                stack.enter_context(patch.object(op, 'atomic_write', side_effect=OSError('fixture record failure')))
            else:
                stack.enter_context(patch.object(op.os, 'replace', side_effect=publish))
            with self.assertRaisesRegex((op.DeploymentError, OSError), 'fixture'):
                self.stage()
        self.assertFalse(self.target.exists())
        self.assertEqual((orphan / 'keep').read_text(), 'historical')
        self.assertEqual(list(self.target.parent.iterdir()), [orphan])
        self.assertEqual(len(self.deleted), 1)
        self.assertTrue(self.deleted[0].name.startswith('.' + self.target.name + '.staging-'))
        source, _, _ = self.stage()
        self.assertTrue((source / 'app.py').is_file())

    def test_snapshot_export_failure_only_cleans_current_staging(self):
        self.assert_staging_failure('export')

    def test_snapshot_seal_failure_only_cleans_current_staging(self):
        self.assert_staging_failure('seal')

    def test_snapshot_record_failure_only_cleans_current_staging(self):
        self.assert_staging_failure('record')

    def test_snapshot_publish_failure_only_cleans_current_staging(self):
        self.assert_staging_failure('publish')

    def test_snapshot_publication_contains_sealed_source_and_record(self):
        real_replace = op.os.replace
        def publish(src, dst):
            if Path(dst) == self.target:
                staging = Path(src)
                record = json.loads((staging / 'snapshot.json').read_text())
                self.assertEqual(record['revision'], self.revision)
                self.assertEqual((staging / 'source/app.py').read_text(), 'print("source")\n')
                self.assertEqual(staging.stat().st_mode & 0o777, 0o550)
                self.assertEqual((staging / 'snapshot.json').stat().st_mode & 0o777, 0o440)
                self.assertFalse(self.target.exists())
            return real_replace(src, dst)
        with patch.object(op.os, 'replace', side_effect=publish):
            self.stage()

    def test_snapshot_post_publish_error_retries_without_export(self):
        real_replace = op.os.replace
        def publish(src, dst):
            real_replace(src, dst)
            if Path(dst) == self.target:
                raise OSError('fixture post-publish failure')
        with patch.object(op.os, 'replace', side_effect=publish):
            with self.assertRaisesRegex(OSError, 'post-publish'):
                self.stage()
        self.assertTrue((self.target / 'source/app.py').is_file())
        self.assertTrue((self.target / 'snapshot.json').is_file())
        self.assertEqual(self.deleted, [])
        with patch.object(self.bundle, '_copy_source_snapshot', side_effect=AssertionError('re-export')):
            self.assertEqual(self.stage()[0], self.target / 'source')

    def test_snapshot_post_publish_fsync_error_preserves_complete_container(self):
        with patch.object(op, 'fsync_directory', side_effect=OSError('fixture parent fsync failure')):
            with self.assertRaisesRegex(OSError, 'parent fsync'):
                self.stage()
        with patch.object(self.bundle, '_copy_source_snapshot', side_effect=AssertionError('re-export')):
            self.assertEqual(self.stage()[0], self.target / 'source')
        self.assertEqual(self.deleted, [])

    def test_snapshot_unknown_missing_or_corrupt_record_is_not_adopted(self):
        self.stage()
        record = self.target / 'snapshot.json'
        original = record.read_bytes()
        for content in (None, b'[]', b'{"schema_version":true}', original.replace(self.revision.encode(), b'other')):
            with self.subTest(content=content):
                self.target.chmod(0o750)
                if record.exists():
                    record.chmod(0o640)
                    record.unlink()
                if content is not None:
                    record.write_bytes(content)
                    record.chmod(0o440)
                with patch.object(self.bundle, '_copy_source_snapshot', side_effect=AssertionError('adoption')):
                    with self.assertRaisesRegex(op.DeploymentError, str(record)):
                        self.stage()
                self.assertTrue((self.target / 'source/app.py').is_file())
                self.assertEqual(self.deleted, [])

    def test_snapshot_layout_mismatch_preserves_files_and_reports_record(self):
        source, _, _ = self.stage()
        item = source / 'app.py'
        item.chmod(0o640)
        item.write_text('changed size')
        with self.assertRaisesRegex(op.DeploymentError, 'provenance/layout'):
            self.stage()
        self.assertEqual(item.read_text(), 'changed size')
        self.assertEqual(self.deleted, [])

    def test_snapshot_unknown_entries_are_not_adopted_or_deleted(self):
        source, _, _ = self.stage()
        for relative in ('unknown', 'source/unknown-directory'):
            with self.subTest(relative=relative):
                item = self.target / relative
                item.parent.chmod(0o750)
                item.mkdir()
                with patch.object(self.bundle, '_copy_source_snapshot', side_effect=AssertionError('adoption')):
                    with self.assertRaises(op.DeploymentError):
                        self.stage()
                self.assertTrue(item.is_dir())
                self.assertEqual(self.deleted, [])
                item.rmdir()
        (source / 'nested').chmod(0o750)
        (source / 'nested/run.sh').unlink()
        with self.assertRaisesRegex(op.DeploymentError, 'provenance/layout'):
            self.stage()
        self.assertFalse((source / 'nested/run.sh').exists())
        self.assertEqual(self.deleted, [])

    def test_snapshot_archive_rejects_links_and_closes_child_pipes(self):
        (self.source / 'link').symlink_to('app.py')
        self.commit()
        actual_popen = op.subprocess.Popen
        archives = []
        def popen(argv, *args, **kwargs):
            process = actual_popen(argv, *args, **kwargs)
            if 'archive' in argv:
                archives.append(process)
            return process
        with patch.object(op.subprocess, 'Popen', side_effect=popen):
            with self.assertRaisesRegex(op.DeploymentError, 'link or special file'):
                self.stage()
        self.assertEqual(len(archives), 1)
        self.assertIsNotNone(archives[0].returncode)
        self.assertTrue(archives[0].stdout.closed)
        self.assertTrue(archives[0].stderr.closed)

    def test_snapshot_layout_does_not_claim_content_authentication(self):
        source, _, _ = self.stage()
        item = source / 'app.py'
        original = item.read_bytes()
        item.chmod(0o640)
        item.write_bytes(b'X' * len(original))
        item.chmod(0o440)
        self.assertEqual(self.stage()[0], source)
        self.assertEqual(item.read_bytes(), b'X' * len(original))

    def test_snapshot_cli_roots_and_input_identity_follow_returned_source(self):
        options = replace(DeploymentOptions.for_tests(source_root=self.source), cache_dir=self.cache,
                          allow_dirty_source=False)
        current = profile()
        with patch.object(cli, 'InstallerLock', return_value=nullcontext()), \
                patch.object(cli, 'load_named_profile', return_value=current):
            prepared, actual_profile, manifest = cli._prepare_source(
                SimpleNamespace(profile='fixture'), options, current, materialize=True,
            )
        self.assertEqual((prepared.source_root, prepared.trust_root, prepared.fingerprint_source_root),
                         (self.target / 'source',) * 3)
        self.assertEqual(engine._path_input_record(prepared.source_root, prepared),
                         {'kind': 'git-snapshot', 'revision': self.revision, 'tree_id': self.tree})
        self.assertEqual(actual_profile, current)
        self.assertIsNone(manifest)


if __name__ == '__main__':
    unittest.main()
