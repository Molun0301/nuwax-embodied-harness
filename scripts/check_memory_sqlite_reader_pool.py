#!/usr/bin/env python3
"""Deterministic offline qualification of two-reader SQLite ownership.

Only synthetic temporary databases; Event/Barrier waits are deadlock watchdogs,
not timing assertions. Optional --baseline PATH runs only the concurrency oracle.
"""
from __future__ import annotations
import importlib.util
import sqlite3
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from services.memory.sqlite_storage import SQLiteStorage

if '--baseline' in sys.argv:
    i = sys.argv.index('--baseline')
    path = sys.argv[i + 1]
    del sys.argv[i:i + 2]
    spec = importlib.util.spec_from_file_location('baseline_sqlite_storage', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    SQLiteStorage = module.SQLiteStorage
    sys.argv.append('PoolChecks.test_two_readers_enter_before_either_releases')

WAIT = 8
class BodyError(Exception): pass
class Abort(BaseException): pass
class BeginError(Exception): pass
class CleanupError(Exception): pass
class CloseError(Exception): pass

class Proxy:
    def __init__(self, connection):
        self.connection = connection
        self.close_errors = 0
        self.close_calls = 0
        self.setup_error = False
    def __getattr__(self, key): return getattr(self.connection, key)
    def execute(self, sql, *args):
        if self.setup_error and sql == 'PRAGMA query_only=ON': raise Abort('setup')
        return self.connection.execute(sql, *args)
    def close(self):
        self.close_calls += 1
        if self.close_errors:
            self.close_errors -= 1
            raise CloseError('close')
        return self.connection.close()

class ObservedCondition(threading.Condition):
    def __init__(self):
        super().__init__(threading.Lock())
        self.waiting = threading.Event()
    def wait(self, timeout=None):
        self.waiting.set()
        return super().wait(timeout)

class PoolChecks(unittest.TestCase):
    def fixture(self, memory=False):
        temp = tempfile.TemporaryDirectory(prefix='neh-reader-pool-')
        self.addCleanup(temp.cleanup)
        owner = SQLiteStorage(':memory:' if memory else str(Path(temp.name) / 'synthetic.db'))
        self.addCleanup(owner.close)
        with owner.transaction() as tx:
            tx.execute('CREATE TABLE t (id INTEGER PRIMARY KEY, generation INTEGER)')
            tx.executemany('INSERT INTO t VALUES (?, ?)', [(1, 0), (2, 0)])
        return owner
    def await_event(self, event): self.assertTrue(event.wait(WAIT), 'deadlock watchdog expired')
    def launch(self, fn):
        errors = []
        def run():
            try: fn()
            except BaseException as e: errors.append(e)
        thread = threading.Thread(target=run, daemon=True)
        thread.start()
        return thread, errors
    def join(self, worker):
        thread, errors = worker
        thread.join(WAIT)
        self.assertFalse(thread.is_alive(), 'thread deadlock watchdog expired')
        if errors: raise errors[0]
    def proxy_slot(self, owner):
        original = owner._idle_readers[-1]
        proxy = Proxy(original)
        owner._idle_readers[-1] = proxy
        owner._readers[owner._readers.index(original)] = proxy
        return proxy
    def test_two_readers_enter_before_either_releases(self):
        owner = self.fixture()
        release = threading.Event()
        entered = [threading.Event(), threading.Event()]
        def reader(i):
            with owner.read_snapshot() as tx:
                self.assertEqual(tx.execute('SELECT COUNT(*) FROM t').fetchone()[0], 2)
                entered[i].set()
                self.assertTrue(release.wait(WAIT * 3), 'release watchdog expired')
        workers = [self.launch(lambda i=i: reader(i)) for i in range(2)]
        try:
            for event in entered: self.await_event(event)
        finally:
            release.set()
            for worker in workers: self.join(worker)
    def test_prebuilt_lifo_third_waits_nested_and_writer_own_writes(self):
        owner = self.fixture()
        self.assertEqual(len(owner._readers), 2)
        for reader in owner._readers:
            self.assertEqual(reader.execute('PRAGMA query_only').fetchone()[0], 1)
            self.assertEqual(reader.execute('PRAGMA journal_mode').fetchone()[0], 'wal')
        with owner.read_snapshot() as tx: last = tx._connection
        with owner.read_snapshot() as tx: self.assertIs(tx._connection, last)
        condition = owner._reader_condition = ObservedCondition()
        release = threading.Event(); entered = threading.Barrier(3); third_entered = threading.Event()
        connections = []
        def reader():
            with owner.read_snapshot() as outer:
                with owner.read_snapshot() as inner: self.assertIs(inner, outer)
                connections.append(outer._connection)
                entered.wait(WAIT)
                self.await_event(release)
        with patch.object(owner, '_connect', side_effect=AssertionError('unexpected connection')):
            workers = [self.launch(reader) for _ in range(2)]
            entered.wait(WAIT)
            def third():
                with owner.read_snapshot(): third_entered.set()
            waiter = self.launch(third)
            try:
                self.await_event(condition.waiting)
                self.assertFalse(third_entered.is_set())
                self.assertEqual(len(set(connections)), 2)
                with owner.transaction() as tx:
                    tx.execute('UPDATE t SET generation=1')
                    with owner.read_snapshot() as own:
                        self.assertIs(own, tx)
                        self.assertEqual([r[0] for r in own.execute('SELECT generation FROM t')], [1, 1])
            finally:
                release.set()
                for worker in workers: self.join(worker)
                self.join(waiter)
            self.assertTrue(third_entered.is_set())
    def test_condition_not_held_across_begin_or_rollback(self):
        owner = self.fixture(); original = owner._control
        barriers = {'BEGIN': threading.Barrier(2), 'ROLLBACK': threading.Barrier(2)}
        def control(connection, sql):
            if connection in owner._readers and sql in barriers:
                barriers[sql].wait(WAIT)
            return original(connection, sql)
        def read():
            with owner.read_snapshot() as tx:
                self.assertEqual(tx.execute('SELECT COUNT(*) FROM t').fetchone()[0], 2)
        with patch.object(owner, '_control', side_effect=control):
            workers = [self.launch(read) for _ in range(2)]
            for worker in workers: self.join(worker)
    def test_old_new_snapshots_remain_internally_consistent(self):
        owner = self.fixture(); old_ready = threading.Event(); new_ready = threading.Event(); release = threading.Event()
        def values(tx): return [row[0] for row in tx.execute('SELECT generation FROM t ORDER BY id')]
        def old_reader():
            with owner.read_snapshot() as tx:
                self.assertEqual(values(tx), [0, 0]); old_ready.set(); self.await_event(new_ready)
                self.assertEqual(values(tx), [0, 0]); release.set()
        old = self.launch(old_reader); self.await_event(old_ready)
        with owner.transaction() as tx: tx.execute('UPDATE t SET generation=1')
        with owner.read_snapshot() as tx:
            self.assertEqual(values(tx), [1, 1]); new_ready.set(); self.await_event(release)
            self.assertEqual(values(tx), [1, 1])
        self.join(old)
    def test_body_exception_and_baseexception_return_slots(self):
        for error in (BodyError('body'), Abort('abort')):
            with self.subTest(error=type(error).__name__):
                owner = self.fixture(); slots = list(owner._readers)
                with self.assertRaises(type(error)) as caught:
                    with owner.read_snapshot(): raise error
                self.assertIs(caught.exception, error)
                self.assertEqual(owner._active_readers, 0)
                self.assertEqual(set(owner._idle_readers), set(slots))
                self.assertTrue(all(not slot.in_transaction for slot in slots))
    def test_begin_failures_reuse_when_clean_and_preserve_original(self):
        for after in (False, True):
            for rollback_failure in (False, True):
                with self.subTest(after=after, rollback_failure=rollback_failure):
                    owner = self.fixture(); target = owner._idle_readers[-1]; original = owner._control
                    error = BeginError('begin')
                    def control(connection, sql):
                        if connection is target and sql == 'BEGIN':
                            if after: original(connection, sql)
                            raise error
                        if connection is target and sql == 'ROLLBACK' and rollback_failure: raise CleanupError('rollback')
                        return original(connection, sql)
                    with patch.object(owner, '_control', side_effect=control):
                        with self.assertRaises(BeginError) as caught:
                            with owner.read_snapshot(): self.fail('BEGIN should fail')
                    self.assertIs(caught.exception, error)
                    retired = after and rollback_failure
                    self.assertEqual(len(owner._readers), 1 if retired else 2)
                    self.assertEqual(owner._active_readers, 0)
                    with owner.read_snapshot() as tx:
                        if not retired: self.assertIs(tx._connection, target)
                        self.assertEqual(tx.execute('SELECT COUNT(*) FROM t').fetchone()[0], 2)
    def test_begin_original_survives_rollback_and_close_failures(self):
        owner = self.fixture(); proxy = self.proxy_slot(owner); proxy.close_errors = 1
        original = owner._control; error = Abort('begin after execution')
        def control(connection, sql):
            if connection is proxy and sql == 'BEGIN':
                original(connection, sql)
                raise error
            if connection is proxy and sql == 'ROLLBACK': raise CleanupError('rollback')
            return original(connection, sql)
        with patch.object(owner, '_control', side_effect=control):
            with self.assertRaises(Abort) as caught:
                with owner.read_snapshot(): self.fail('BEGIN should fail')
        self.assertIs(caught.exception, error)
        self.assertIn(proxy, owner._readers)
        self.assertNotIn(proxy, owner._idle_readers)
        self.assertEqual(owner._active_readers, 0)
        owner.close(); self.assertEqual(proxy.close_calls, 2)
    def test_cleanup_error_precedence_including_unrelated_except(self):
        for body in (None, BodyError('body'), Abort('abort')):
            for unrelated in (False, True):
                with self.subTest(body=type(body).__name__, unrelated=unrelated):
                    owner = self.fixture(); target = owner._idle_readers[-1]; original = owner._control
                    cleanup = CleanupError('rollback')
                    def control(connection, sql):
                        if connection is target and sql == 'ROLLBACK': raise cleanup
                        return original(connection, sql)
                    def run():
                        with patch.object(owner, '_control', side_effect=control):
                            with owner.read_snapshot():
                                if body is not None: raise body
                    with self.assertRaises(type(body or cleanup)) as caught:
                        if unrelated:
                            try: raise ValueError('unrelated caller error')
                            except ValueError: run()
                        else: run()
                    self.assertIs(caught.exception, body or cleanup)
                    self.assertEqual(len(owner._readers), 1)
                    self.assertEqual(owner._active_readers, 0)
    def test_failed_retirement_close_owned_zero_slots_unavailable_and_retry(self):
        owner = self.fixture(); proxies = []
        for _ in range(2):
            proxy = self.proxy_slot(owner); proxies.append(proxy); proxy.close_errors = 1
            original = owner._control
            def control(connection, sql):
                if connection is proxy and sql == 'ROLLBACK': raise CleanupError('rollback')
                return original(connection, sql)
            with patch.object(owner, '_control', side_effect=control):
                with self.assertRaises(CleanupError):
                    with owner.read_snapshot(): pass
            self.assertIn(proxy, owner._readers)
            self.assertNotIn(proxy, owner._idle_readers)
        self.assertEqual(owner._active_readers, 0)
        with self.assertRaisesRegex(RuntimeError, 'unavailable'):
            with owner.read_snapshot(): pass
        owner.close()
        self.assertTrue(owner._closed)
        self.assertTrue(all(p.close_calls == 2 for p in proxies))
    def test_close_announces_rejects_outer_drains_allows_nested_and_diagnostics(self):
        owner = self.fixture(); condition = owner._reader_condition = ObservedCondition()
        entered = threading.Event(); release = threading.Event(); nested_done = threading.Event()
        def reader():
            with owner.read_snapshot() as tx:
                entered.set(); self.await_event(release)
                with owner.read_snapshot() as inner: self.assertIs(inner, tx)
                self.assertEqual(owner.rollback_errors, ())
                tx.execute('SELECT * FROM t').fetchall(); nested_done.set()
        worker = self.launch(reader); self.await_event(entered)
        closer = self.launch(owner.close); self.await_event(condition.waiting)
        try:
            with condition: self.assertTrue(owner._closing)
            for context in (owner.read_snapshot, owner.transaction):
                with self.assertRaisesRegex(RuntimeError, 'closed'):
                    with context(): pass
        finally: release.set()
        self.join(worker); self.join(closer)
        self.assertTrue(nested_done.is_set()); self.assertTrue(owner._closed)
    def test_close_wakes_queued_third_and_drains_both_active_readers(self):
        owner = self.fixture()
        queued = threading.Event(); closing = threading.Event(); waiting_for_b = threading.Event()
        class DrainCondition(threading.Condition):
            def __init__(self): super().__init__(threading.Lock())
            def wait(self, timeout=None):
                if owner._closing:
                    closing.set()
                    if owner._active_readers == 1: waiting_for_b.set()
                else:
                    queued.set()
                return super().wait(timeout)
        condition = owner._reader_condition = DrainCondition()
        entered = threading.Barrier(3)
        release_a = threading.Event(); check_b = threading.Event(); checked_b = threading.Event()
        release_b = threading.Event(); third_rejected = threading.Event(); close_done = threading.Event()
        def reader_a():
            with owner.read_snapshot():
                entered.wait(WAIT)
                self.await_event(release_a)
        def reader_b():
            with owner.read_snapshot() as tx:
                entered.wait(WAIT)
                self.await_event(check_b)
                self.assertEqual(tx.execute('SELECT COUNT(*) FROM t').fetchone()[0], 2)
                checked_b.set()
                self.await_event(release_b)
        readers = [self.launch(reader_a), self.launch(reader_b)]
        entered.wait(WAIT)
        def third():
            with self.assertRaisesRegex(RuntimeError, 'closed'):
                with owner.read_snapshot(): self.fail('queued reader admitted during close')
            third_rejected.set()
        waiter = self.launch(third)
        closer = None
        try:
            self.await_event(queued)
            with condition:
                self.assertEqual(owner._active_readers, 2)
                self.assertFalse(owner._closing)
            def close():
                owner.close()
                close_done.set()
            closer = self.launch(close)
            self.await_event(closing)
            with condition: self.assertTrue(owner._closing)
            self.await_event(third_rejected)
            self.join(waiter)
            release_a.set()
            self.join(readers[0])
            self.await_event(waiting_for_b)
            with condition:
                self.assertEqual(owner._active_readers, 1)
                self.assertFalse(owner._closed)
            check_b.set()
            self.await_event(checked_b)
            self.assertFalse(close_done.is_set())
        finally:
            release_a.set(); check_b.set(); release_b.set()
            for worker in readers: self.join(worker)
            self.join(waiter)
            if closer is not None: self.join(closer)
        self.assertTrue(close_done.is_set())
        self.assertTrue(owner._closed)
    def test_concurrent_closers_and_retry_after_reader_close_failure(self):
        owner = self.fixture(); proxy = self.proxy_slot(owner); proxy.close_errors = 1
        with self.assertRaises(CloseError): owner.close()
        self.assertTrue(owner._closing); self.assertFalse(owner._closed); self.assertIn(proxy, owner._readers)
        with self.assertRaisesRegex(RuntimeError, 'closed'):
            with owner.read_snapshot(): pass
        barrier = threading.Barrier(3)
        def close(): barrier.wait(WAIT); owner.close()
        workers = [self.launch(close) for _ in range(2)]
        barrier.wait(WAIT)
        for worker in workers: self.join(worker)
        self.assertTrue(owner._closed); self.assertEqual(proxy.close_calls, 2)
    def test_interrupted_close_stays_terminal_and_retry(self):
        owner = self.fixture(); entered = threading.Event(); release = threading.Event()
        def reader():
            with owner.read_snapshot(): entered.set(); self.await_event(release)
        worker = self.launch(reader); self.await_event(entered)
        with patch.object(owner._reader_condition, 'wait', side_effect=Abort('close interrupted')):
            with self.assertRaises(Abort): owner.close()
        self.assertTrue(owner._closing); self.assertFalse(owner._closed)
        with self.assertRaisesRegex(RuntimeError, 'closed'):
            with owner.transaction(): pass
        release.set(); self.join(worker); owner.close(); self.assertTrue(owner._closed)
    def test_same_thread_close_readonly_and_facade_lifetime_thread(self):
        owner = self.fixture()
        with owner.read_snapshot() as tx:
            with self.assertRaises(RuntimeError): owner.close()
            with self.assertRaises(sqlite3.OperationalError): tx.execute('UPDATE t SET generation=8')
            with self.assertRaises(RuntimeError):
                with owner.transaction(): pass
            def misuse():
                with self.assertRaises(RuntimeError): tx.execute('SELECT 1')
            self.join(self.launch(misuse))
        with self.assertRaises(RuntimeError): tx.execute('SELECT 1')
        with owner.transaction():
            with self.assertRaises(RuntimeError): owner.close()
    def test_memory_single_connection_and_nested_writer_identity(self):
        owner = self.fixture(memory=True)
        self.assertEqual(owner._readers, [])
        with owner.transaction() as tx:
            tx.execute('UPDATE t SET generation=2')
            with owner.read_snapshot() as read: self.assertIs(read, tx)
        with owner.read_snapshot() as read: self.assertIs(read._connection, owner._connection)
    def test_active_writer_nested_then_callbacks_outside_lock_before_close_completes(self):
        owner = self.fixture(); writer_entered = threading.Event(); close_attempt = threading.Event(); allow_writer = threading.Event()
        callback_entered = threading.Event(); finish_callback = threading.Event(); events = []
        def callback(committed):
            self.assertTrue(committed)
            self.assertFalse(owner._lock._is_owned())
            events.append('callback'); callback_entered.set(); self.await_event(finish_callback)
        def writer():
            with owner.transaction() as tx:
                tx.after_completion(callback); writer_entered.set(); self.await_event(allow_writer)
                with owner.transaction() as inner: inner.execute('UPDATE t SET generation=3')
                with owner.read_snapshot() as own: self.assertIs(own, tx)
                events.append('nested')
        worker = self.launch(writer); self.await_event(writer_entered)
        def close(): close_attempt.set(); owner.close(); events.append('closed')
        closer = self.launch(close); self.await_event(close_attempt)
        # Holding the writer lock prevents close from announcing closing.
        self.assertFalse(owner._closing)
        allow_writer.set(); self.await_event(callback_entered)
        self.join(closer); finish_callback.set(); self.join(worker)
        self.assertLess(events.index('nested'), events.index('closed'))
    def test_second_reader_creation_and_setup_failure_closes_returned_connections(self):
        for fail_setup in (False, True):
            with self.subTest(fail_setup=fail_setup):
                temp = tempfile.TemporaryDirectory(prefix='neh-reader-constructor-'); self.addCleanup(temp.cleanup)
                made = []; original = SQLiteStorage._connect
                error = Abort('constructor')
                def connect(owner):
                    if len(made) == 2 and not fail_setup: raise error
                    proxy = Proxy(original(owner)); made.append(proxy)
                    if len(made) == 3: proxy.setup_error = True
                    return proxy
                with patch.object(SQLiteStorage, '_connect', connect):
                    with self.assertRaises(Abort): SQLiteStorage(str(Path(temp.name) / 'db'))
                self.assertEqual(len(made), 3 if fail_setup else 2)
                self.assertTrue(all(c.close_calls == 1 for c in made))
                for c in made:
                    with self.assertRaises(sqlite3.ProgrammingError): c.execute('SELECT 1')

if __name__ == '__main__': unittest.main(verbosity=2)
