#!/usr/bin/env python3
"""Focused real-SQLite fallback regressions, including Python 3.8 runtimes."""
from __future__ import annotations

import importlib.util
import os
from pathlib import Path
import sqlite3
import sys
import tempfile
import unittest
from unittest.mock import patch

# Reuse the existing isolated protocol import and application digest guard.
import check_log_index_hashfree as harness

protocol = harness.protocol
index_module = harness.index_module
baseline = os.environ.get('NEH_LOG_INDEX_MODULE_PATH')
if baseline:
    spec = importlib.util.spec_from_file_location(harness.PACKAGE + '.compat_baseline', baseline)
    index_module = importlib.util.module_from_spec(spec)
    with harness.digest_guard():
        spec.loader.exec_module(index_module)


class SQLiteCompatibilityChecks(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='neh-sqlite-compat-')
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.log = self.root / 'main.log'
        self.original = ''.join(harness.line(harness.entry(n, message='token=FixtureSecret123 ' + str(n)))
                                for n in range(4)).encode('utf-8')
        self.log.write_bytes(self.original)
        self.reader = index_module.LogHistoryReader()
        self.addCleanup(self.reader.close)
        with protocol._LOG_QUERY_CACHE_LOCK:
            protocol._LOG_QUERY_CACHE.clear()

    def tearDown(self):
        self.assertEqual(self.log.read_bytes(), self.original)

    def assert_scan(self, **options):
        options.setdefault('limit', 2)
        expected = protocol._scan_logs(str(self.log), **options)
        actual = self.reader.query(str(self.log), **options)
        self.assertEqual(actual, expected)
        self.assertNotIn('FixtureSecret123', str(actual))
        return actual

    def assert_recovers(self):
        self.assertIsNone(self.reader.failed_scope)
        self.assertIsNone(self.reader.index)
        first = self.assert_scan()
        self.assertIsNotNone(self.reader.index)
        self.assert_scan(before=first['next_before'])

    def test_native_database_lock_on_index_creation_recovers(self):
        path = index_module._database_path(str(self.log), '')
        holder = sqlite3.connect(str(path), isolation_level=None)
        self.addCleanup(holder.close)
        holder.execute('BEGIN EXCLUSIVE')
        try:
            self.assert_scan()
            self.assertIsNone(self.reader.failed_scope)
        finally:
            holder.rollback()
        self.assert_recovers()

    def test_native_database_lock_on_warm_query_recovers(self):
        self.assert_scan()
        old = self.reader.index
        holder = sqlite3.connect(str(old.path), isolation_level=None)
        self.addCleanup(holder.close)
        holder.execute('BEGIN EXCLUSIVE')
        try:
            self.assert_scan()
            self.assertIsNone(old.connection)
        finally:
            holder.rollback()
        self.assert_recovers()

    def native_shared_lock(self, schema):
        uri = (self.root / 'shared.sqlite3').as_uri() + '?cache=shared'
        first = sqlite3.connect(uri, uri=True, isolation_level=None)
        second = sqlite3.connect(uri, uri=True, isolation_level=None)
        try:
            first.execute('CREATE TABLE fixture(value INTEGER)')
            first.execute('BEGIN IMMEDIATE')
            if schema:
                first.execute('CREATE TABLE uncommitted(value INTEGER)')
                statement = 'SELECT * FROM fixture'
            else:
                first.execute('INSERT INTO fixture VALUES(1)')
                statement = 'SELECT * FROM fixture'
            try:
                second.execute(statement).fetchall()
            except sqlite3.OperationalError as error:
                print('Native shared-lock:', repr(str(error)),
                      'code=', getattr(error, 'sqlite_errorcode', None))
                return error
            self.fail('Fixture did not produce a native SQLite lock')
        finally:
            first.rollback()
            first.close()
            second.close()

    def test_native_table_lock_recovers(self):
        error = self.native_shared_lock(False)
        self.assertEqual(str(error), 'database table is locked: fixture')
        with patch.object(index_module, 'PersistentLogIndex', side_effect=error):
            self.assert_scan()
        self.assert_recovers()

    def test_native_schema_lock_recovers(self):
        error = self.native_shared_lock(True)
        self.assertEqual(str(error), 'database schema is locked: main')
        with patch.object(index_module, 'PersistentLogIndex', side_effect=error):
            self.assert_scan()
        self.assert_recovers()

    def test_native_malformed_index_stays_failed_scope(self):
        path = index_module._database_path(str(self.log), '')
        path.write_bytes(b'not a sqlite database\n')
        self.assert_scan()
        self.assertEqual(self.reader.failed_scope, (str(self.log), ''))
        path.write_bytes(b'')
        with patch.object(index_module, 'PersistentLogIndex', side_effect=AssertionError('must not retry')):
            self.assert_scan()

    def test_nontransient_operational_errors_stay_failed_scope(self):
        messages = ('no such table: entries', 'database is locked unexpectedly',
                    'application database is locked', 'database table is lockedness',
                    'database schema is lockedness')
        for message in messages:
            with self.subTest(message=message):
                self.reader.close()
                self.reader = index_module.LogHistoryReader()
                self.addCleanup(self.reader.close)
                with patch.object(index_module, 'PersistentLogIndex', side_effect=sqlite3.OperationalError(message)) as constructor:
                    self.assert_scan()
                    self.assert_scan()
                    self.assertEqual(constructor.call_count, 1)
                self.assertEqual(self.reader.failed_scope, (str(self.log), ''))

    def test_extended_busy_and_locked_codes_recovers(self):
        for code in (5, 6, 261, 262, 517):
            with self.subTest(code=code):
                self.reader.close()
                self.reader = index_module.LogHistoryReader()
                self.addCleanup(self.reader.close)
                error = sqlite3.OperationalError('native-message-not-used-for-code')
                error.sqlite_errorcode = code
                with patch.object(index_module, 'PersistentLogIndex', side_effect=error):
                    self.assert_scan()
                self.assert_recovers()

    def test_unsupported_projection_stays_failed_scope(self):
        with patch.object(index_module, 'PersistentLogIndex', side_effect=index_module._UnsupportedProjection('fixture')) as constructor:
            self.assert_scan()
            self.assert_scan()
            self.assertEqual(constructor.call_count, 1)
        self.assertEqual(self.reader.failed_scope, (str(self.log), ''))

    def test_snapshot_change_recovers(self):
        with patch.object(index_module, 'PersistentLogIndex', side_effect=protocol._LogSnapshotChanged('fixture')):
            self.assert_scan()
        self.assert_recovers()


if __name__ == '__main__':
    print('Python:', sys.version)
    print('SQLite:', sqlite3.sqlite_version)
    print('Module:', index_module.__file__)
    with harness.digest_guard():
        outcome = unittest.main(verbosity=2, exit=False)
    print('Blocked digest constructor attempts:', len(harness._digest_attempts))
    raise SystemExit(0 if outcome.result.wasSuccessful() and not harness._digest_attempts else 1)
