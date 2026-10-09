#!/usr/bin/env python3
"""Exercise transient installer comparisons with temporary files and explicit doubles."""
import os
import sys
import tempfile
import unittest
from contextlib import ExitStack, nullcontext
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scripts.deploy_installer import cli, operations as op  # noqa: E402
from scripts.deploy_installer.engine import DeploymentPlan, ExecutionContext  # noqa: E402
from scripts.deploy_installer.model import DeploymentOptions  # noqa: E402
from scripts.deploy_installer.profile import (  # noqa: E402
    DeploymentProfile, ProfileError, PythonProfile, RustProfile,
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
                cli._execute(SimpleNamespace(profile='fixture'), dry_run=False)
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


if __name__ == '__main__':
    unittest.main()
