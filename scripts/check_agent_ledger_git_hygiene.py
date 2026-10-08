#!/usr/bin/env python3
"""Real Git/SQLite check: default ledgers must not block source updates."""
from pathlib import Path
import os
import sqlite3
import subprocess
import tempfile
import unittest

import self_update

ROOT = Path(__file__).resolve().parents[1]


class LedgerGitHygieneChecks(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='neh-ledger-git-')
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.env = dict(os.environ, GIT_CONFIG_GLOBAL=os.devnull, GIT_CONFIG_NOSYSTEM='1')
        self.git('init', '--quiet')
        self.git('config', 'user.name', 'Fixture')
        self.git('config', 'user.email', 'fixture@example.invalid')
        rules = Path(os.environ.get('NEH_IGNORE_RULES', str(ROOT / '.gitignore')))
        (self.root / '.gitignore').write_bytes(rules.read_bytes())
        (self.root / 'source.py').write_text('value = 1\n')
        self.git('add', '.gitignore', 'source.py')
        self.git('commit', '--quiet', '-m', 'fixture')

    def git(self, *args):
        return subprocess.check_output(['git', '-C', str(self.root)] + list(args),
                                       env=self.env, text=True)

    def test_live_wal_databases_do_not_dirty_source(self):
        for name in ('agent_calls.sqlite3', 'agent_calls.terminals.sqlite3'):
            connection = sqlite3.connect(str(self.root / name))
            self.addCleanup(connection.close)
            self.assertEqual(connection.execute('PRAGMA journal_mode=WAL').fetchone()[0], 'wal')
            connection.execute('CREATE TABLE receipt (value TEXT)')
            connection.execute("INSERT INTO receipt VALUES ('keep history')")
            connection.commit()
            self.assertTrue((self.root / (name + '-wal')).exists())
            self.assertTrue((self.root / (name + '-shm')).exists())
            self_update.require_clean(self.root)
            self.assertEqual(connection.execute('SELECT value FROM receipt').fetchall(), [('keep history',)])

    def test_rollback_journals_do_not_dirty_source(self):
        for name in ('agent_calls.sqlite3', 'agent_calls.terminals.sqlite3'):
            connection = sqlite3.connect(str(self.root / name))
            self.addCleanup(connection.close)
            connection.execute('CREATE TABLE receipt (value TEXT)')
            connection.commit()
            connection.execute("INSERT INTO receipt VALUES ('uncommitted')")
            self.assertTrue((self.root / (name + '-journal')).exists())
            self_update.require_clean(self.root)

    def test_unrelated_untracked_files_still_rejected(self):
        for name in ('unrelated.sqlite3', 'agent_calls.sqlite3.backup', 'new_source.py',
                     'sample_package/agent_calls.sqlite3'):
            with self.subTest(name=name):
                path = self.root / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text('preserve me')
                with self.assertRaises(RuntimeError):
                    self_update.require_clean(self.root)
                self.assertEqual(path.read_text(), 'preserve me')
                path.unlink()

    def test_tracked_edits_still_rejected(self):
        (self.root / 'source.py').write_text('value = 2\n')
        with self.assertRaises(RuntimeError):
            self_update.require_clean(self.root)

    def test_already_tracked_database_is_not_hidden(self):
        path = self.root / 'agent_calls.sqlite3'
        path.write_text('tracked fixture')
        self.git('add', '--force', path.name)
        self.git('commit', '--quiet', '-m', 'tracked fixture')
        path.write_text('changed fixture')
        with self.assertRaises(RuntimeError):
            self_update.require_clean(self.root)


if __name__ == '__main__':
    unittest.main(verbosity=2)
