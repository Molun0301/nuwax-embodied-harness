#!/usr/bin/env python3
"""持久日志无摘要索引：脱敏精确筛选、作用域、轮转和可变末尾回归。"""
from __future__ import annotations

from contextlib import ExitStack, contextmanager
from datetime import datetime, timedelta, timezone
import hashlib
import importlib
import json
import os
from pathlib import Path
import sqlite3
import stat
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

# 先装载既有框架：Starlette导入时有自己的MD5能力探测，不属于应用内容摘要。
# 随后的本仓库模块导入和全部查询测试仍由所有hashlib构造入口的守卫覆盖。
import fastapi

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
_digest_attempts = []


@contextmanager
def digest_guard():
    def forbidden(*args, **kwargs):
        _digest_attempts.append(True)
        raise AssertionError('Application digest execution is forbidden')
    with ExitStack() as guards:
        for name in ('new', 'md5', 'sha1', 'sha224', 'sha256', 'sha384', 'sha512',
                     'sha3_224', 'sha3_256', 'sha3_384', 'sha3_512', 'shake_128', 'shake_256', 'blake2b', 'blake2s'):
            guards.enter_context(patch.object(hashlib, name, side_effect=forbidden))
        yield


# 不执行gateways.http.__init__，避免它导入真实网关/settings及读取.env。
PACKAGE = '_isolated_log_checks'
package = types.ModuleType(PACKAGE); package.__path__ = [str(ROOT / 'gateways/http')]
sys.modules[PACKAGE] = package
with digest_guard():
    protocol = importlib.import_module(PACKAGE + '.logs')
    index_module = importlib.import_module(PACKAGE + '.log_index')
PersistentLogIndex = index_module.PersistentLogIndex
WORK = Path(tempfile.gettempdir()) / 'neh-log-index-hashfree-checks'
SECRET_A = 'token=AlphaSyntheticPrivate123'
SECRET_B = 'token=BravoSyntheticPrivate456'
MASKED = protocol._redact_entry(SECRET_A)


def entry(number, **values):
    result = {'timestamp': (datetime(2026, 10, 2, tzinfo=timezone.utc) + timedelta(seconds=number)).isoformat(),
        'id': 'synthetic-' + str(number), 'level': 'info', 'source': 'python', 'component': 'synthetic.worker',
        'message': 'message ' + str(number), 'request_id': 'request-' + str(number % 3), 'task_id': 'task-safe'}
    result.update(values)
    return result


def line(value): return json.dumps(value, ensure_ascii=False, separators=(',', ':')) + '\n'


class LogIndexChecks(unittest.TestCase):
    def setUp(self):
        WORK.mkdir(parents=True, exist_ok=True)
        temporary = tempfile.TemporaryDirectory(prefix='fixture-', dir=WORK)
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.log = self.root / 'main.log'
        self.log.write_text('')
        self.index = PersistentLogIndex(str(self.log), str(self.root)); self.addCleanup(self.index.close)
        with protocol._LOG_QUERY_CACHE_LOCK: protocol._LOG_QUERY_CACHE.clear()

    def write(self, entries, *, path=None, append=False):
        with (path or self.log).open('a' if append else 'w', encoding='utf-8') as stream:
            stream.writelines(line(item) for item in entries)

    def query(self, **options):
        return self.index.query(str(self.log), log_dir=str(self.root), **options)

    def assert_scan(self, **options):
        expected = protocol._scan_logs(str(self.log), log_dir=str(self.root), **options)
        actual = self.query(**options)
        self.assertEqual(actual, expected)
        return actual

    def test_safe_indexed_queries_match_scanner_and_warm_reuse_parses_nothing(self):
        self.write([entry(n, level='warning' if n % 2 else 'info', message='混合Unicode STRASSE ' + str(n)) for n in range(30)])
        for options in ({'limit': 7}, {'limit': 7, 'request_id': 'request-1'}, {'limit': 7, 'task_id': 'task-safe'},
                        {'limit': 7, 'level': 'warn'}, {'limit': 7, 'component': 'WORKER'},
                        {'limit': 7, 'query': 'straße'}, {'limit': 7, 'source': 'python'}):
            self.assert_scan(**options)
        self.assertEqual(self.index.stats['parsed_entries'], 0)
        self.assertEqual(self.index.stats['read_bytes'], 0)
        self.assertEqual(self.index.stats['raw_candidates'], 0)

    def test_json_rust_spans_and_legacy_text_formats_keep_exact_filter_semantics(self):
        rust = {'timestamp': '2026-10-02T04:00:01Z', 'level': 'INFO',
                'fields': {'message': 'rust structured message', 'request_id': SECRET_A},
                'spans': [{'task_id': SECRET_B}], 'target': 'synthetic.crate'}
        self.log.write_text(line(entry(1)) + line(rust)
            + '2026-10-02 04:00:02,000 [WARN] [synthetic] legacy text\n'
            + '2026-10-02T04:00:03Z INFO plain rust message\n')
        self.assert_scan(limit=20)
        self.assert_scan(limit=20, source='rust')
        self.assert_scan(limit=20, request_id=SECRET_A, task_id=SECRET_B)

    def test_export_body_and_cursor_headers_match_original_rendering(self):
        self.write([entry(n, request_id=SECRET_A if n % 2 else SECRET_B) for n in range(10)])
        options = {'log_dir': str(self.root), 'limit': 2, 'request_id': SECRET_A}
        for format_name in (None, 'jsonl', 'text'):
            indexed = protocol._render_log_response(str(self.log), options, format_name, reader=self.index.query)
            scanned = protocol._render_log_response(str(self.log), options, format_name, reader=protocol._scan_logs)
            self.assertEqual(indexed, scanned)
            self.assertNotIn(SECRET_A.encode(), indexed[0])

    def test_sensitive_filters_distinguish_collisions_and_literal_masks_without_false_counts(self):
        self.assertEqual(protocol._redact_entry(SECRET_B), MASKED)
        self.write([entry(1, request_id=SECRET_A), entry(2, request_id=SECRET_A),
                    entry(3, request_id=SECRET_B), entry(4, request_id=MASKED)])
        result = self.assert_scan(limit=1, request_id=SECRET_A)
        self.assertEqual(result['entries'][0]['id'], 'synthetic-2'); self.assertTrue(result['has_more'])
        self.assert_scan(limit=10, request_id=SECRET_B)
        literal = self.assert_scan(limit=10, request_id=MASKED)
        self.assertEqual([item['id'] for item in literal['entries']], ['synthetic-4'])
        self.assertNotIn(SECRET_A, json.dumps(result)); self.assertNotIn(SECRET_B, json.dumps(result))
        self.assert_scan(limit=1)
        self.assertEqual(self.index.stats['raw_candidates'], 0)

    def test_sensitive_task_only_verified_when_that_field_is_filtered(self):
        self.write([entry(1, request_id='safe', task_id=SECRET_A), entry(2, request_id='safe', task_id=SECRET_B)])
        self.assert_scan(limit=10, request_id='safe')
        self.assertEqual(self.index.stats['raw_candidates'], 0)
        self.assert_scan(limit=10, request_id='safe', task_id=SECRET_A)
        self.assertEqual(self.index.stats['raw_candidates'], 2)

    def test_one_ordered_candidate_cursor_does_not_skip_duplicate_sorting_keys(self):
        values = [entry(1, id='same-explicit-id', request_id=SECRET_A) for _ in range(2)]
        values += [entry(1, id='same-explicit-id', request_id=SECRET_B) for _ in range(150)]
        self.write(values)
        result = self.assert_scan(limit=1, request_id=SECRET_A)
        self.assertEqual(len(result['entries']), 1); self.assertTrue(result['has_more'])
        self.assertEqual(self.index.stats['raw_candidates'], 152)

    def test_typed_record_ids_are_not_coerced_to_http_filter_strings(self):
        values = [1, True, 1.0, None, ['1'], {'a': 1}, '1', 'true', '{"a":1}']
        self.write([entry(n, request_id=value) for n, value in enumerate(values)])
        for value in ('1', 'true', '{"a":1}', 'null'): self.assert_scan(limit=30, request_id=value)

    def test_sensitive_originals_and_raw_anchors_never_enter_database_or_journal(self):
        self.write([entry(1, request_id=SECRET_A, task_id=SECRET_B, message=SECRET_A)])
        self.assert_scan(limit=10, request_id=SECRET_A)
        for path in self.index.path.parent.iterdir():
            if path.is_file():
                data = path.read_bytes()
                self.assertNotIn(SECRET_A.encode(), data); self.assertNotIn(SECRET_B.encode(), data)
            self.assertNotIn(SECRET_A, path.name)
        columns = {row[1] for row in self.index.connection.execute('PRAGMA table_info(files)')}
        self.assertFalse({'prefix', 'suffix', 'sha256'} & columns)
        self.assertTrue(any(SECRET_A.encode() in part for _version, anchors in self.index._anchor_cache.values() for part in anchors))

    def test_inflight_sqlite_journal_also_contains_no_sensitive_originals(self):
        self.write([entry(1, request_id=SECRET_A)]); self.query()
        self.write([entry(2, task_id=SECRET_B)], append=True)
        insert = self.index._insert; checked = []
        def inspect_journal(rows):
            insert(rows)
            for path in self.index.path.parent.iterdir():
                if path.is_file():
                    raw = path.read_bytes()
                    self.assertNotIn(SECRET_A.encode(), raw); self.assertNotIn(SECRET_B.encode(), raw)
                    if path.name.endswith('-journal'): checked.append(path.name)
        with patch.object(self.index, '_insert', side_effect=inspect_journal): self.query()
        self.assertTrue(checked)

    def test_fixed_database_has_isolated_literal_scopes_and_scoped_deletion(self):
        other = self.root / 'other.log'; self.write([entry(1, source='first')]); self.write([entry(2, source='second')], path=other)
        second = PersistentLogIndex(str(other), str(self.root)); self.addCleanup(second.close)
        self.assertEqual(second.path, self.index.path)
        self.assertNotEqual(second.scope_id, self.index.scope_id)
        self.assertEqual(self.query()['sources'], ['first'])
        self.assertEqual(second.query(str(other), log_dir=str(self.root))['sources'], ['second'])
        self.log.unlink()
        self.assertEqual(self.query()['entries'], [])
        self.assertEqual(second.query(str(other), log_dir=str(self.root))['sources'], ['second'])
        self.assertEqual(second.stats['parsed_entries'], 0)

    def test_scope_keys_handle_unicode_long_main_names_and_relative_cwd_changes(self):
        main = self.root / ('长' * 65 + '.log'); self.write([entry(1)], path=main)
        other = PersistentLogIndex(str(main)); self.addCleanup(other.close)
        self.assertEqual(other.path.name, 'history-v2.sqlite3')
        self.assertEqual(other.query(str(main))['entries'][0]['id'], 'synthetic-1')
        original = Path.cwd()
        elsewhere = self.root / 'elsewhere'; elsewhere.mkdir()
        try:
            os.chdir(self.root)
            relative = PersistentLogIndex(main.name); self.addCleanup(relative.close)
            self.assertEqual(relative.scope_id, other.scope_id)
            os.chdir(elsewhere)
            self.assertEqual(relative.query(main.name)['entries'][0]['id'], 'synthetic-1')
        finally: os.chdir(original)

    def test_append_reparses_only_mutable_tail_and_new_bytes(self):
        self.write([entry(n) for n in range(100)])
        self.assert_scan(limit=10); old_size = self.log.stat().st_size
        self.write([entry(100)], append=True)
        self.assert_scan(limit=10)
        self.assertLess(self.index.stats['read_bytes'], old_size)
        self.assertLessEqual(self.index.stats['parsed_entries'], 2)

    def test_reopen_unchanged_reuses_index_but_changed_without_old_anchors_rebuilds_one_file(self):
        self.write([entry(n) for n in range(40)]); self.query()
        self.index.close(); self.assertFalse(self.index._anchor_cache)
        self.index = PersistentLogIndex(str(self.log), str(self.root)); self.addCleanup(self.index.close)
        self.assert_scan(limit=10); self.assertEqual(self.index.stats['parsed_entries'], 0)
        self.index.close(); self.write([entry(40)], append=True)
        self.index = PersistentLogIndex(str(self.log), str(self.root)); self.addCleanup(self.index.close)
        self.assert_scan(limit=10); self.assertEqual(self.index.stats['parsed_entries'], 41)
        self.write([entry(41)], append=True)
        self.assert_scan(limit=10); self.assertLessEqual(self.index.stats['parsed_entries'], 2)

    def test_reopen_anchor_seed_cannot_tag_rewritten_boundaries_with_old_version(self):
        self.write([entry(1), entry(2)]); self.query(); self.index.close()
        self.index = PersistentLogIndex(str(self.log), str(self.root)); self.addCleanup(self.index.close)
        anchors = self.index._anchors; changed = False
        def rewrite_during_seed(handle, size):
            nonlocal changed
            if not changed:
                changed = True
                self.write([entry(n, message='replacement-' + str(n)) for n in range(10, 20)])
            return anchors(handle, size)
        with patch.object(self.index, '_anchors', side_effect=rewrite_during_seed):
            with self.assertRaises(protocol._LogSnapshotChanged): self.query()
        self.assertFalse(self.index._anchor_cache)
        self.assertEqual(self.index.connection.execute('SELECT count(*) FROM entries WHERE scope_id=?', (self.index.scope_id,)).fetchone()[0], 2)
        result = self.assert_scan(limit=30)
        self.assertEqual(len(result['entries']), 10)
        self.assertFalse({'synthetic-1', 'synthetic-2'} & {item['id'] for item in result['entries']})

    def test_anchor_lru_is_bounded_and_evicted_changed_file_rebuilds_without_losing_entries(self):
        second = self.root / 'nuwax-z.log'; self.write([entry(n) for n in range(10)]); self.write([entry(20)], path=second)
        with patch.object(index_module, '_ANCHOR_CACHE_FILES', 1):
            self.query(); self.assertEqual(len(self.index._anchor_cache), 1)
            self.write([entry(10)], append=True)
            self.assert_scan(limit=30)
            self.assertEqual(len(self.index._anchor_cache), 1)
            self.assertLessEqual(sum(len(part) for _state, anchors in self.index._anchor_cache.values() for part in anchors), 512)
            self.assertGreaterEqual(self.index.stats['parsed_entries'], 11)

    def test_rename_rotation_new_inode_and_missing_current_file_preserve_identity(self):
        self.write([entry(1, request_id=SECRET_A)]); before = self.query()['entries'][0]['id']
        rotated = self.root / 'main.log.1'; self.log.rename(rotated)
        self.write([entry(2)])
        result = self.assert_scan(limit=10)
        self.assertIn(before, [value['id'] for value in result['entries']])
        self.assertEqual(self.index.stats['parsed_entries'], 1)
        self.assert_scan(limit=10, request_id=SECRET_A)
        self.log.unlink(); self.assert_scan(limit=10)

    def test_same_size_overwrite_and_truncate_regrow_replace_stale_rows(self):
        self.write([entry(1)]); self.query(); previous = self.log.stat()
        self.write([entry(2)]); os.utime(self.log, ns=(previous.st_atime_ns, previous.st_mtime_ns))
        self.assertEqual(self.log.stat().st_size, previous.st_size)
        self.assertEqual(self.assert_scan(limit=10)['entries'][0]['id'], 'synthetic-2')
        self.write([entry(n, message='replacement-' + str(n)) for n in range(20, 30)])
        self.assert_scan(limit=20); self.assertEqual(self.index.stats['parsed_entries'], 10)

    def test_partial_json_newline_completion_and_traceback_continuations_match_scanner(self):
        self.write([entry(1)]); self.log.write_text(self.log.read_text() + '{"timestamp":"2026-10-02T01:00:00Z","message":"part')
        self.assert_scan(limit=10)
        with self.log.open('a') as stream: stream.write('ial","request_id":"safe"}')
        self.assert_scan(limit=10)
        with self.log.open('a') as stream: stream.write('\n  Traceback continuation\n  final frame\n')
        self.assert_scan(limit=10)
        self.write([entry(10000)], append=True); self.assert_scan(limit=10)

    def test_mtime_derived_timestamps_require_changed_file_reparse(self):
        self.log.write_text('plain process message\n' + line(entry(1)))
        self.assert_scan(limit=10)
        with self.log.open('a') as stream: stream.write('another plain message\n')
        self.assert_scan(limit=10)
        self.assertGreaterEqual(self.index.stats['parsed_entries'], 3)

    def test_raw_verification_uses_identical_fallback_time_at_nanosecond_boundaries(self):
        value = entry(1, request_id=SECRET_A); value.pop('timestamp')
        self.write([value])
        for nanos in (123456499, 123456500, 123456789, 999999499, 999999500):
            stamp = 1790000000 * 1_000_000_000 + nanos
            os.utime(self.log, ns=(stamp, stamp))
            self.assert_scan(limit=10, request_id=SECRET_A)

    def test_cursor_pages_invalid_cursor_and_prefilter_source_list_match_scanner(self):
        self.write([entry(n, timestamp='2026-10-02T00:00:00Z', source='first' if n % 2 else 'second') for n in range(12)])
        first = self.assert_scan(limit=4, source='first')
        self.assertEqual(first['sources'], ['first', 'second'])
        self.assert_scan(limit=4, before=first['next_before'], source='first')
        self.assert_scan(limit=4, before='not a cursor')

    def test_missing_or_replaced_source_after_refresh_rejects_stale_offsets(self):
        self.write([entry(1, request_id=SECRET_A)]); self.query()
        opened = index_module._RawCandidateReader._opened
        def replace_source(reader, row):
            self.log.rename(self.root / 'moved-away.log')
            self.write([entry(2, request_id=SECRET_A)])
            return opened(reader, row)
        with patch.object(index_module._RawCandidateReader, '_opened', replace_source):
            with self.assertRaises(protocol._LogSnapshotChanged): self.query(request_id=SECRET_A)
        self.assertFalse(self.index.connection.in_transaction)
        self.assertEqual(self.assert_scan(request_id=SECRET_A)['entries'][0]['id'], 'synthetic-2')

    def test_source_mutation_during_raw_verification_rolls_back_without_false_matches(self):
        self.write([entry(1, request_id=SECRET_A)]); self.query()
        original = index_module._RawCandidateReader.read
        def mutate(reader, row):
            value = original(reader, row)
            self.write([entry(2, request_id=SECRET_A)], append=True)
            return value
        with patch.object(index_module._RawCandidateReader, 'read', mutate):
            with self.assertRaises(protocol._LogSnapshotChanged): self.query(request_id=SECRET_A)
        self.assertFalse(self.index.connection.in_transaction)
        self.assert_scan(request_id=SECRET_A)

    def test_raw_candidate_ordering_identity_mismatch_fails_instead_of_skipping(self):
        self.write([entry(1, request_id=SECRET_A)]); self.query()
        self.index.connection.execute('UPDATE entries SET identity=? WHERE scope_id=?', ('wrong', self.index.scope_id))
        with self.assertRaises(protocol._LogSnapshotChanged): self.query(request_id=SECRET_A)

    def test_rollback_anchor_versions_are_not_used_for_older_database_snapshot(self):
        self.write([entry(1)]); self.query(); old = self.index.connection.execute('SELECT size FROM files WHERE scope_id=?', (self.index.scope_id,)).fetchone()[0]
        self.write([entry(2)], append=True)
        with patch.object(index_module._RawCandidateReader, '__enter__', side_effect=ValueError('synthetic query failure')):
            with self.assertRaises(ValueError): self.query()
        self.assertEqual(self.index.connection.execute('SELECT size FROM files WHERE scope_id=?', (self.index.scope_id,)).fetchone()[0], old)
        self.assert_scan(limit=10); self.assertEqual(self.index.stats['parsed_entries'], 2)

    def test_large_records_keep_existing_semantics_without_a_new_cap(self):
        self.write([entry(1, request_id=SECRET_A, message='large-' + 'x' * (2 * 1024 * 1024))])
        result = self.assert_scan(limit=1, request_id=SECRET_A)
        self.assertGreater(len(result['entries'][0]['message']), 2 * 1024 * 1024)
        self.assertGreater(self.index.stats['raw_candidate_bytes'], 2 * 1024 * 1024)

    def test_legacy_cache_is_ignored_without_rehash_or_rewrite(self):
        old = self.index.path.parent / ('history-v1-' + 'a' * 24 + '.sqlite3')
        old.write_bytes(b'synthetic historical cache'); self.write([entry(1)])
        before = old.read_bytes(); self.query()
        self.assertEqual(old.read_bytes(), before)
        self.assertEqual(self.index.path.name, 'history-v2.sqlite3')
        self.assertEqual(self.index.connection.execute('PRAGMA user_version').fetchone()[0], 2)

    def test_private_permissions_and_symlink_cache_file_are_enforced(self):
        self.assertEqual(stat.S_IMODE(self.index.path.stat().st_mode), 0o600)
        self.assertEqual(stat.S_IMODE(self.index.path.parent.stat().st_mode), 0o700)
        self.index.close(); self.index.path.unlink()
        target = self.root / 'unrelated'; target.write_text('keep')
        self.index.path.symlink_to(target)
        with self.assertRaises(OSError): PersistentLogIndex(str(self.log), str(self.root))
        self.assertEqual(target.read_text(), 'keep')

    def test_incompatible_cache_and_unsupported_projection_use_existing_fallback(self):
        self.index.close()
        with sqlite3.connect(self.index.path) as connection: connection.execute('PRAGMA user_version=999')
        self.write([entry(1)])
        reader = index_module.LogHistoryReader(); self.addCleanup(reader.close)
        self.assertEqual(reader.query(str(self.log), log_dir=str(self.root)), protocol.query_logs(str(self.log), log_dir=str(self.root)))
        self.assertIsNone(reader.index)

    def test_corrupt_derived_database_falls_back_without_rewriting_authoritative_logs(self):
        self.write([entry(1)]); before = self.log.read_bytes(); self.index.close()
        self.index.path.write_bytes(b'synthetic malformed SQLite data')
        reader = index_module.LogHistoryReader(); self.addCleanup(reader.close)
        self.assertEqual(reader.query(str(self.log), log_dir=str(self.root)), protocol.query_logs(str(self.log), log_dir=str(self.root)))
        self.assertEqual(self.log.read_bytes(), before)
        self.assertIsNone(reader.index)

    def test_transient_shared_database_contention_is_retried_on_next_query(self):
        self.write([entry(1)]); self.query()
        reader = index_module.LogHistoryReader(); self.addCleanup(reader.close)
        self.index.connection.execute('BEGIN IMMEDIATE')
        try:
            result = reader.query(str(self.log), log_dir=str(self.root))
            self.assertEqual(result['entries'][0]['id'], 'synthetic-1')
            self.assertIsNone(reader.failed_scope)
        finally: self.index.connection.execute('ROLLBACK')
        self.assertEqual(reader.query(str(self.log), log_dir=str(self.root))['entries'][0]['id'], 'synthetic-1')
        self.assertIsNotNone(reader.index)

    def test_raw_file_handle_lru_is_bounded_and_all_handles_close(self):
        for number in range(12):
            self.write([entry(number, request_id=SECRET_A)], path=self.root / ('nuwax-' + str(number) + '.log'))
        handles = []; opened = index_module._RawCandidateReader._opened
        def observe(reader, row):
            handle = opened(reader, row); handles.append(handle)
            self.assertLessEqual(len(reader.files), index_module._RAW_READER_FILES)
            return handle
        with patch.object(index_module._RawCandidateReader, '_opened', observe):
            self.assert_scan(limit=20, request_id=SECRET_A)
        self.assertEqual(len(handles), 12); self.assertTrue(all(handle.closed for handle in handles))

    def test_sensitive_component_is_not_persisted_via_unsupported_projection(self):
        self.write([entry(1, component=SECRET_A)])
        with self.assertRaises(index_module._UnsupportedProjection): self.query()
        self.assertEqual(self.index.connection.execute('SELECT count(*) FROM entries WHERE scope_id=?', (self.index.scope_id,)).fetchone()[0], 0)
        self.assertNotIn(SECRET_A.encode(), self.index.path.read_bytes())


if __name__ == '__main__':
    with digest_guard():
        outcome = unittest.main(verbosity=2, exit=False)
    print('Blocked digest constructor attempts:', len(_digest_attempts))
    raise SystemExit(0 if outcome.result.wasSuccessful() and not _digest_attempts else 1)
