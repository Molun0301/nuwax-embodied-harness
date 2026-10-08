#!/usr/bin/env python3
"""In-memory Git identity equality, without application hashes or host Git calls."""
import subprocess
import sys
import tempfile
import unittest
from contextlib import ExitStack
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scripts.deploy_installer import operations as op  # noqa: E402


class GitSourceIdentityTests(unittest.TestCase):
    def query(self, head=b'commit-id\n', index=b'100644 object-id 0\tfile.py\0',
              dirty=b'', failures=()):
        values = [subprocess.CompletedProcess([], int(i in failures), output, b'')
                  for i, output in enumerate((head, index, dirty))]
        with ExitStack() as stack:
            stack.enter_context(patch.object(op, 'source_user_environment', return_value={}))
            stack.enter_context(patch.object(op, 'source_user_command', side_effect=tuple))
            stack.enter_context(patch.object(op.hashlib, 'sha256', side_effect=AssertionError('application hash')))
            run = stack.enter_context(patch.object(op.subprocess, 'run', side_effect=values))
            result = op._git_source_identity(Path('/fixture-only'))
        self.assertEqual([call.args[0] for call in run.call_args_list], [
            ('git', 'rev-parse', 'HEAD'), ('git', 'ls-files', '--stage', '-z'),
            ('git', 'status', '--porcelain', '--untracked-files=normal')])
        self.assertEqual([call.kwargs['timeout'] for call in run.call_args_list], [30, 60, 30])
        return result

    def test_retains_exact_existing_bytes_without_copy_or_hash(self):
        head, index = b'commit-id\n', b'100644 object-id 0\t\xff name.py\0'
        result = self.query(head, index)
        self.assertIs(result[0], head)
        self.assertIs(result[1], index)
        self.assertEqual(result, self.query(head, index))

    def test_head_and_index_changes_remain_distinct(self):
        baseline = self.query()
        self.assertNotEqual(baseline, self.query(head=b'other-id\n'))
        self.assertNotEqual(baseline, self.query(index=b'100755 other-id 0\tfile.py\0'))

    def test_dirty_and_unreadable_states_keep_existing_rejection(self):
        for dirty in (b' M file.py\n', b'?? new.py\n'):
            with self.subTest(dirty=dirty), self.assertRaisesRegex(op.DeploymentError, 'dirty or unreadable'):
                self.query(dirty=dirty)
        for index in range(3):
            with self.subTest(failure=index), self.assertRaisesRegex(op.DeploymentError, 'dirty or unreadable'):
                self.query(failures=(index,))
        self.query(dirty=b' \n')

    def test_install_rejects_changed_pair_before_release_write(self):
        original = self.query()
        with tempfile.TemporaryDirectory() as directory:
            release = Path(directory) / 'release'
            context = SimpleNamespace(
                options=SimpleNamespace(source_root=Path(directory), release_dir=release),
                notes={'source_identity_at_preflight': original})
            with patch.object(op, '_git_source_identity', return_value=(b'changed', original[1])):
                with self.assertRaisesRegex(op.DeploymentError, 'source changed'):
                    op.install_release(context)
            self.assertFalse(release.exists())


if __name__ == '__main__':
    unittest.main()
