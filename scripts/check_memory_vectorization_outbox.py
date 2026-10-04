#!/usr/bin/env python3
"""Offline foundation checks using real SQL source/vector rows and crash cuts.

Run .nuwax-test-env/bin/python scripts/check_memory_vectorization_outbox.py.
Only temporary databases and this script's own subprocesses are used. No memory
service integration, model calls, hardware, network, or throughput claims.
"""

from __future__ import annotations

import ast
import os
import random
import signal
import sqlite3
import subprocess
import sys
import tempfile
import threading
import tracemalloc
import unittest
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from services.memory.sqlite_storage import SQLiteStorage
from services.memory.vectorization_outbox import (
    OutboxConflict,
    RetryPolicy,
    VectorizationOutbox,
    WorkKey,
)


class Clock:
    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now


class Fixture:
    """Actual SQL participants; no mocked source commits or vector publication."""

    def __init__(self, path=":memory:", *, clock=None, retry=None, history_limit=16, wake=None):
        self.storage = SQLiteStorage(path)
        self.clock = clock or Clock()
        self.outbox = VectorizationOutbox(
            self.storage, clock=self.clock, retry_policy=retry, history_limit=history_limit, wake=wake
        )
        with self.storage.transaction() as tx:
            tx.execute("""CREATE TABLE IF NOT EXISTS probe_source (
                library TEXT, kind TEXT, id TEXT, incarnation TEXT, revision INTEGER,
                payload TEXT, PRIMARY KEY(library, kind, id))""")
            tx.execute("""CREATE TABLE IF NOT EXISTS probe_vectors (
                library TEXT, kind TEXT, id TEXT, incarnation TEXT, revision INTEGER,
                receipt_token TEXT, vector TEXT, PRIMARY KEY(library, kind, id))""")
        self.library = self.outbox.create_library("A")

    def put(self, point_id, *, expected=None, library=None, kind="semantic", payload="same timestamp/text"):
        key = WorkKey((library or self.library).incarnation, kind, str(point_id))

        def write(tx, version):
            if expected is None:
                tx.execute(
                    "INSERT INTO probe_source VALUES (?, ?, ?, ?, ?, ?)",
                    key.sql() + (version.record_incarnation, version.revision, payload),
                )
            else:
                changed = tx.execute(
                    """UPDATE probe_source SET revision=?, payload=?
                    WHERE library=? AND kind=? AND id=? AND incarnation=? AND revision=?""",
                    (version.revision, payload) + key.sql() + (expected.record_incarnation, expected.revision),
                ).rowcount
                if changed != 1:
                    raise OutboxConflict("Actual SQL source CAS failed")
            # Conservative invalidation is atomic with the authoritative write.
            tx.execute("DELETE FROM probe_vectors WHERE library=? AND kind=? AND id=?", key.sql())

        return self.outbox.accept(key, expected=expected, write_source=write)

    @staticmethod
    def publish(tx, claim):
        version = claim.receipt.version
        key = version.key
        row = tx.execute(
            "SELECT incarnation, revision FROM probe_source WHERE library=? AND kind=? AND id=?", key.sql()
        ).fetchone()
        if row is None or tuple(row) != (version.record_incarnation, version.revision):
            raise OutboxConflict("Actual SQL source does not match publication")
        tx.execute(
            "INSERT OR REPLACE INTO probe_vectors VALUES (?, ?, ?, ?, ?, ?, ?)",
            key.sql() + (version.record_incarnation, version.revision, claim.receipt.token, "[1.0,0.0]"),
        )

    def delete(self, receipt):
        def remove(tx, version):
            changed = tx.execute(
                "DELETE FROM probe_source WHERE library=? AND kind=? AND id=? AND incarnation=? AND revision=?",
                version.key.sql() + (version.record_incarnation, version.revision),
            ).rowcount
            if changed != 1:
                raise OutboxConflict("Actual SQL delete CAS failed")
            tx.execute("DELETE FROM probe_vectors WHERE library=? AND kind=? AND id=?", version.key.sql())

        self.outbox.delete(receipt.version.key, expected=receipt.version, delete_source=remove)

    def drop(self, library):
        def remove(tx, identity):
            tx.execute("DELETE FROM probe_source WHERE library=?", (identity.incarnation,))
            tx.execute("DELETE FROM probe_vectors WHERE library=?", (identity.incarnation,))

        self.outbox.delete_library(library, delete_source=remove)

    def count(self, table):
        assert table in ("probe_source", "probe_vectors", "memory_vectorization_state")
        with self.storage.read_snapshot() as tx:
            return tx.execute("SELECT count(*) FROM " + table).fetchone()[0]

    def start(self, *, recover=False):
        self.outbox.activate(self.library)
        return self.outbox.start_worker(recover=recover)

    def conserved(self):
        snapshot = self.outbox.snapshot()
        assert snapshot["accepted"] == snapshot["desired_accounted"], snapshot
        assert snapshot["attempts_started"] == snapshot["attempts_accounted"], snapshot
        assert snapshot["occupied_claim_slots"] <= 1, snapshot
        assert snapshot["receipt_details"] <= self.outbox.history_limit, snapshot
        return snapshot


class StorageChecks(unittest.TestCase):
    def test_nested_savepoints_real_rollback_and_notification_isolation(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(str(Path(directory) / "atomic.sqlite"))
            self.addCleanup(fixture.storage.close)
            storage = fixture.storage
            hooks = []
            with self.assertRaisesRegex(RuntimeError, "outer abort"), storage.transaction() as outer:
                first = fixture.put("outer")
                with storage.transaction() as inner:
                    fixture.put("inner")
                    inner.after_commit(lambda: hooks.append("inner"))
                with self.assertRaisesRegex(RuntimeError, "inner abort"), storage.transaction() as inner:
                    fixture.put("rolledback")
                    inner.after_commit(lambda: hooks.append("bad"))
                    raise RuntimeError("inner abort")
                self.assertEqual(fixture.count("probe_source"), 2)
                self.assertIs(storage.current_transaction, outer)
                with storage.read_snapshot() as own:
                    self.assertIs(own, outer)
                other = sqlite3.connect(storage.db_path)
                try:
                    self.assertEqual(other.execute("SELECT count(*) FROM probe_source").fetchone()[0], 0)
                finally:
                    other.close()
                self.assertFalse(hooks)
                self.assertEqual(first.version.revision, 1)
                raise RuntimeError("outer abort")
            self.assertEqual(fixture.count("probe_source"), 0)
            self.assertEqual(fixture.conserved()["accepted"], 0)
            self.assertFalse(hooks)
            with storage.transaction() as outer:
                fixture.put("committed")
                outer.after_commit(lambda: hooks.append("outer"))
                with storage.transaction() as inner:
                    inner.after_commit(lambda: hooks.append("inner"))
                with self.assertRaises(ValueError):
                    outer.execute("/* accidental helper commit */ COMMIT")
                self.assertFalse(hooks)
            self.assertEqual(hooks, ["outer", "inner"])
            with self.assertRaises(RuntimeError):
                outer.execute("SELECT 1")
            fixture.conserved()

    def test_implicit_full_rollback_poisoning_prevents_autocommit_escape(self):
        fixture = Fixture()
        self.addCleanup(fixture.storage.close)
        hooks = []
        with fixture.storage.transaction() as tx:
            tx.execute("CREATE TABLE rollback_probe (id INTEGER PRIMARY KEY)")
        with self.assertRaisesRegex(RuntimeError, "rolled back"), fixture.storage.transaction() as outer:
            outer.execute("INSERT INTO rollback_probe VALUES (1)")
            outer.after_commit(lambda: hooks.append("must not run"))
            with self.assertRaises(sqlite3.IntegrityError), fixture.storage.transaction() as inner:
                inner.execute("INSERT OR ROLLBACK INTO rollback_probe VALUES (1)")
            with self.assertRaisesRegex(RuntimeError, "rolled back"):
                outer.execute("INSERT INTO rollback_probe VALUES (2)")
            with self.assertRaisesRegex(RuntimeError, "rolled back"), fixture.storage.transaction():
                pass
            with self.assertRaisesRegex(RuntimeError, "rolled back"):
                outer.after_commit(lambda: hooks.append("also must not run"))
            # Catching EVERY inner error still cannot make outer exit succeed.
        with fixture.storage.read_snapshot() as tx:
            self.assertEqual(tx.execute("SELECT count(*) FROM rollback_probe").fetchone()[0], 0)
        self.assertFalse(hooks)
        with fixture.storage.transaction() as tx:
            tx.execute("INSERT INTO rollback_probe VALUES (3)")
        with fixture.storage.read_snapshot() as tx:
            self.assertEqual(tx.execute("SELECT id FROM rollback_probe").fetchone()[0], 3)

    def test_close_inside_writer_or_reader_is_rejected_and_no_lock_cycle(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(str(Path(directory) / "close.sqlite"))
            storage = fixture.storage
            self.addCleanup(storage.close)
            with storage.transaction(), self.assertRaises(RuntimeError):
                storage.close()
            close_started = threading.Event()

            def closer():
                close_started.set()
                storage.close()

            with ThreadPoolExecutor(max_workers=1) as pool:
                with storage.read_snapshot():
                    future = pool.submit(closer)
                    self.assertTrue(close_started.wait(10))
                    with self.assertRaises(RuntimeError), storage.transaction():
                        pass
                    with self.assertRaises(RuntimeError):
                        storage.close()
                future.result(timeout=10)
            with self.assertRaises(RuntimeError), storage.read_snapshot():
                pass
            with self.assertRaises(RuntimeError), storage.transaction():
                pass

    def test_read_reuse_thread_isolation_and_no_writer_lock_during_wal_read(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(str(Path(directory) / "wal.sqlite"))
            self.addCleanup(fixture.storage.close)
            storage = fixture.storage
            fixture.put("before")
            entered, release = threading.Event(), threading.Event()

            def writer():
                with storage.transaction():
                    fixture.put("during")
                    entered.set()
                    if not release.wait(10):
                        raise RuntimeError("test reader failed to release writer")

            with ThreadPoolExecutor(max_workers=1) as pool:
                future = pool.submit(writer)
                self.assertTrue(entered.wait(10))
                try:
                    self.assertFalse(storage.in_transaction)
                    self.assertIsNone(storage.current_transaction)
                    with storage.read_snapshot() as reader:
                        connection = reader._connection
                        self.assertEqual(reader.execute("SELECT count(*) FROM probe_source").fetchone()[0], 1)
                        with storage.read_snapshot() as nested:
                            self.assertIs(reader, nested)
                        with self.assertRaises(RuntimeError), storage.transaction():
                            pass
                        with self.assertRaises(sqlite3.OperationalError):
                            reader.execute("DELETE FROM probe_source")
                finally:
                    release.set()
                future.result(timeout=10)
            with storage.read_snapshot() as reader:
                self.assertIs(reader._connection, connection)
                self.assertEqual(reader.execute("SELECT count(*) FROM probe_source").fetchone()[0], 2)
            fixture.conserved()

    def test_post_commit_errors_do_not_undo_or_report_failed_admission(self):
        def broken_wakeup():
            raise RuntimeError("synthetic wake failure")

        fixture = Fixture(wake=broken_wakeup)
        self.addCleanup(fixture.storage.close)
        for index in range(20):
            receipt = fixture.put(index)
            self.assertEqual(fixture.outbox.current(receipt.version.key), receipt)
        self.assertEqual(fixture.storage.notification_failure_count, 20)
        self.assertEqual(len(fixture.storage.notification_errors), 16)
        self.assertEqual(fixture.count("probe_source"), 20)
        fixture.conserved()

    def test_deduplicated_parent_hook_survives_descendant_rollback(self):
        fixture = Fixture()
        self.addCleanup(fixture.storage.close)
        calls = []

        def wake():
            calls.append("wake")

        for rollback in (True, False):
            calls.clear()
            with fixture.storage.transaction() as outer:
                try:
                    with fixture.storage.transaction() as inner:
                        inner.after_commit(wake, deduplicate=True)
                        outer.after_commit(wake, deduplicate=True)
                        if rollback:
                            raise RuntimeError("child rollback")
                except RuntimeError:
                    pass
            self.assertEqual(calls, ["wake"])
        calls.clear()
        with fixture.storage.transaction() as outer:
            outer.after_commit(wake)
            with fixture.storage.transaction() as inner:
                inner.after_commit(wake)
                outer.after_commit(wake)
        self.assertEqual(calls, ["wake", "wake", "wake"])

    def test_bulk_transaction_coalesces_wakeup_hooks_and_discards_rollback(self):
        calls = []
        fixture = Fixture(wake=lambda: calls.append("wake"))
        self.addCleanup(fixture.storage.close)
        with fixture.storage.transaction() as outer:
            for index in range(4101):
                fixture.put(index)
                self.assertEqual(len(outer._callbacks), 1)
            self.assertFalse(calls)
        self.assertEqual(calls, ["wake"])
        with self.assertRaises(RuntimeError), fixture.storage.transaction():
            fixture.put("rolledback wake")
            raise RuntimeError("rollback")
        fixture.put("new wake")
        self.assertEqual(calls, ["wake", "wake"])
        self.assertEqual(fixture.conserved()["accepted"], 4102)

    def test_callbacks_execute_after_lock_release(self):
        fixture = Fixture()
        self.addCleanup(fixture.storage.close)
        completed = threading.Event()

        def notification():
            def other_thread():
                with fixture.storage.transaction():
                    completed.set()

            thread = threading.Thread(target=other_thread)
            thread.start()
            self.assertTrue(completed.wait(10))
            thread.join(timeout=10)

        with fixture.storage.transaction() as tx:
            tx.after_commit(notification)
        self.assertEqual(fixture.storage.notification_failure_count, 0)


class OutboxChecks(unittest.TestCase):
    def fixture(self, **kwargs):
        fixture = Fixture(**kwargs)
        self.addCleanup(fixture.storage.close)
        return fixture

    def test_4101_unique_jobs_drain_and_bounded_scheduler_state(self):
        fixture = self.fixture(history_limit=7)
        outbox = fixture.outbox
        worker = fixture.start()
        tracemalloc.start()
        before, _ = tracemalloc.get_traced_memory()
        for index in range(4101):
            fixture.put(index)
        after_admission, peak = tracemalloc.get_traced_memory()
        tracemalloc.stop()
        snapshot = fixture.conserved()
        self.assertEqual(snapshot["accepted"], 4101)
        self.assertEqual(snapshot["current_rows"], 4101)
        self.assertEqual(snapshot["pending_ready"], 4101)
        self.assertEqual(snapshot["receipt_details"], 7)
        self.assertEqual(fixture.count("probe_source"), 4101)
        # No list/deque/dict of jobs is retained on the scheduler. This does not
        # claim a bound on the existing MemoryService's unrelated caches/RSS.
        self.assertFalse(any(isinstance(value, (list, dict, set)) for value in vars(outbox).values()))
        for index in range(4101):
            claim = outbox.claim_next(worker)
            self.assertIsNotNone(claim)
            self.assertEqual(claim.receipt.version.key.point_id, str(index))
            self.assertTrue(outbox.complete(claim, publish=fixture.publish).published)
            if index % 127 == 0:
                fixture.conserved()
        self.assertIsNone(outbox.claim_next(worker))
        snapshot = fixture.conserved()
        self.assertEqual(snapshot["completed_fresh"], 4101)
        self.assertEqual(snapshot["attempts_completed"], 4101)
        self.assertEqual(snapshot["pending_ready"], 0)
        self.assertEqual(fixture.count("probe_vectors"), 4101)
        print(
            f"4101-job synthetic correctness: live Python allocation delta={after_admission - before} B; "
            f"traced peak={peak} B (diagnostic only, no throughput/RSS claim)"
        )

    def test_4101_file_jobs_survive_reopen_and_all_drain(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "4101.sqlite")
            fixture = Fixture(path, history_limit=7)
            fixture.start()
            for index in range(4101):
                fixture.put(index)
            self.assertEqual(fixture.conserved()["accepted"], 4101)
            fixture.storage.close()
            fixture = Fixture(path, history_limit=7)
            self.addCleanup(fixture.storage.close)
            self.assertEqual(fixture.conserved()["pending_ready"], 4101)
            worker = fixture.outbox.start_worker(recover=True)
            for index in range(4101):
                claim = fixture.outbox.claim_next(worker)
                self.assertEqual(claim.receipt.version.key.point_id, str(index))
                self.assertTrue(fixture.outbox.complete(claim, publish=fixture.publish).published)
            self.assertEqual(fixture.conserved()["completed_fresh"], 4101)
            self.assertEqual(fixture.count("probe_vectors"), 4101)
            with fixture.storage.read_snapshot() as tx:
                self.assertEqual(
                    tx.execute("""SELECT count(*) FROM probe_source s JOIN probe_vectors v
                    ON (s.library,s.kind,s.id)=(v.library,v.kind,v.id)
                    WHERE s.incarnation=v.incarnation AND s.revision=v.revision""").fetchone()[0],
                    4101,
                )

    def test_indexed_wakeup_hint_matches_ready_backoff_and_claim(self):
        fixture = self.fixture(retry=RetryPolicy(3, 5, 5))
        worker = fixture.start()
        self.assertIsNone(fixture.outbox.next_wakeup_at(worker))
        fixture.put("key")
        self.assertEqual(fixture.outbox.next_wakeup_at(worker), fixture.clock.now)
        claim = fixture.outbox.claim_next(worker, lease_seconds=10)
        self.assertEqual(fixture.outbox.next_wakeup_at(worker), fixture.clock.now + 10)
        fixture.outbox.fail(claim, category="temporary")
        self.assertEqual(fixture.outbox.next_wakeup_at(worker), fixture.clock.now + 5)
        fixture.clock.now += 6
        self.assertEqual(fixture.outbox.next_wakeup_at(worker), fixture.clock.now)
        fixture.outbox.activate(None)
        self.assertIsNone(fixture.outbox.next_wakeup_at(worker))
        fixture.conserved()

    def test_indexed_ready_and_bounded_backoff_queries(self):
        fixture = self.fixture()
        with fixture.storage.read_snapshot() as tx:
            ready = tx.execute(
                """EXPLAIN QUERY PLAN SELECT * FROM memory_vectorization_state
                WHERE library_incarnation=? AND status='pending' ORDER BY ticket LIMIT 1""",
                (fixture.library.incarnation,),
            ).fetchall()
            due = tx.execute(
                """EXPLAIN QUERY PLAN SELECT receipt FROM memory_vectorization_state
                WHERE library_incarnation=? AND status='backoff' AND next_attempt_at<=?
                ORDER BY next_attempt_at, ticket LIMIT 64""",
                (fixture.library.incarnation, 1000),
            ).fetchall()
        ready_plan, due_plan = " ".join(row[3] for row in ready), " ".join(row[3] for row in due)
        self.assertIn("USING INDEX memory_vectorization_ready", ready_plan)
        self.assertIn("USING INDEX memory_vectorization_backoff", due_plan)
        self.assertNotIn("TEMP B-TREE", ready_plan + due_plan)
        self.assertNotIn("SCAN memory_vectorization_state", ready_plan + due_plan)
        print("Claim plans:", ready_plan, ";", due_plan)

    def test_coalescing_first_dirty_age_and_hot_cold_fairness(self):
        fixture = self.fixture()
        outbox = fixture.outbox
        worker = fixture.start()
        hot = fixture.put("hot")
        first = outbox.inspect(hot.version.key)
        cold = fixture.put("cold")
        for index in range(50):
            fixture.clock.now += 1
            hot = fixture.put("hot", expected=hot.version, payload=str(index))
            state = outbox.inspect(hot.version.key)
            self.assertEqual(state["ticket"], first["ticket"])
            self.assertEqual(state["first_dirty_at"], first["first_dirty_at"])
            fixture.conserved()
        claim = outbox.claim_next(worker)
        self.assertEqual(claim.receipt, hot)
        newest = fixture.put("hot", expected=hot.version, payload="new while running")
        self.assertIsNone(outbox.claim_next(worker))  # obsolete physical attempt still owns slot
        self.assertFalse(outbox.complete(claim, publish=fixture.publish).published)
        self.assertEqual(outbox.current(newest.version.key), newest)
        claim = outbox.claim_next(worker)
        self.assertEqual(claim.receipt, cold)
        outbox.complete(claim, publish=fixture.publish)
        fixture.conserved()
        # An endless supply of newer keys/hot updates cannot jump the pending hot
        # key's finite predecessors. Producer activity does not duplicate slots.
        for index in range(20):
            fixture.put("new" + str(index))
            newest = fixture.put("hot", expected=newest.version, payload="again" + str(index))
        claim = outbox.claim_next(worker)
        self.assertEqual(claim.receipt, newest)
        outbox.complete(claim, publish=fixture.publish)
        snapshot = fixture.conserved()
        self.assertEqual(snapshot["attempts_superseded"], 1)
        self.assertEqual(snapshot["superseded"], 71)

    def test_source_cas_and_new_revision_fence_real_payload(self):
        fixture = self.fixture()
        outbox = fixture.outbox
        worker = fixture.start()
        old = fixture.put("same")
        claim = outbox.claim_next(worker)
        latest = fixture.put("same", expected=old.version, payload="new filter; same timestamp")
        with self.assertRaises(OutboxConflict):
            fixture.put("same", expected=old.version, payload="stale whole snapshot")
        self.assertFalse(outbox.complete(claim, publish=fixture.publish).published)
        self.assertEqual(fixture.count("probe_vectors"), 0)
        self.assertEqual(outbox.current(latest.version.key), latest)
        new_claim = outbox.claim_next(worker)
        self.assertTrue(outbox.complete(new_claim, publish=fixture.publish).published)
        with fixture.storage.read_snapshot() as tx:
            self.assertEqual(tx.execute("SELECT payload FROM probe_source").fetchone()[0], "new filter; same timestamp")
            self.assertEqual(tx.execute("SELECT revision FROM probe_vectors").fetchone()[0], 2)
        snapshot = fixture.conserved()
        self.assertEqual(snapshot["accepted"], 2)
        self.assertEqual(snapshot["superseded"], 1)

    def test_admission_and_publication_failure_roll_back_actual_sql(self):
        fixture = self.fixture()
        outbox = fixture.outbox
        worker = fixture.start()
        for table in ("memory_vectorization_state", "memory_vectorization_receipts"):
            with fixture.storage.transaction() as tx:
                tx.execute(f"""CREATE TRIGGER reject_probe BEFORE INSERT ON {table}
                    BEGIN SELECT RAISE(ABORT, 'synthetic full/rejected write'); END""")
            with self.assertRaises(sqlite3.IntegrityError):
                fixture.put("failed")
            self.assertEqual(fixture.count("probe_source"), 0)
            self.assertEqual(fixture.count("memory_vectorization_state"), 0)
            self.assertEqual(fixture.conserved()["accepted"], 0)
            with fixture.storage.transaction() as tx:
                tx.execute("DROP TRIGGER reject_probe")
        fixture.put("ok")
        claim = outbox.claim_next(worker)
        hooks = []

        def failed_publish(tx, job):
            fixture.publish(tx, job)
            tx.after_commit(lambda: hooks.append("should never run"))
            tx.execute("INSERT INTO table_that_does_not_exist VALUES (1)")

        with self.assertRaises(sqlite3.OperationalError):
            outbox.complete(claim, publish=failed_publish)
        self.assertEqual(fixture.count("probe_vectors"), 0)
        self.assertEqual(fixture.conserved()["in_flight_current"], 1)
        self.assertFalse(hooks)
        self.assertTrue(outbox.complete(claim, publish=fixture.publish).published)
        self.assertFalse(outbox.complete(claim, publish=fixture.publish).published)
        self.assertEqual(fixture.conserved()["completed_fresh"], 1)

    def test_callback_reentry_cannot_orphan_receipts_or_library_work(self):
        fixture = self.fixture()
        outbox = fixture.outbox
        worker = fixture.start()
        old = fixture.put("key")

        def reenter(tx, version):
            tx.execute("UPDATE probe_source SET payload='must roll back'")
            outbox.accept(
                old.version.key, expected=old.version, write_source=lambda tx, version: None
            )

        with self.assertRaisesRegex(OutboxConflict, "reenter"):
            outbox.accept(old.version.key, expected=old.version, write_source=reenter)
        self.assertEqual(outbox.current(old.version.key), old)
        self.assertEqual(fixture.conserved()["accepted"], 1)
        with fixture.storage.read_snapshot() as tx:
            self.assertEqual(tx.execute("SELECT payload FROM probe_source").fetchone()[0], "same timestamp/text")
            self.assertEqual(tx.execute("SELECT count(*) FROM memory_vectorization_receipts").fetchone()[0], 1)

        def delete_library_reentry(tx, version):
            fixture.drop(fixture.library)

        with self.assertRaisesRegex(OutboxConflict, "reenter"):
            outbox.accept(
                old.version.key, expected=old.version, write_source=delete_library_reentry
            )
        with self.assertRaisesRegex(OutboxConflict, "reenter"):
            outbox.delete(old.version.key, expected=old.version, delete_source=reenter)
        with self.assertRaisesRegex(OutboxConflict, "reenter"):
            outbox.delete_library(fixture.library, delete_source=lambda tx, library: fixture.put("nested"))
        claim = outbox.claim_next(worker)
        with self.assertRaisesRegex(OutboxConflict, "reenter"):
            outbox.complete(claim, publish=lambda tx, claim: outbox.activate(None))
        self.assertTrue(outbox.complete(claim, publish=fixture.publish).published)
        fixture.conserved()
        # The supported dependency route nests complete operations in one owner
        # transaction, rather than mutating the scheduler inside its SQL callback.
        with fixture.storage.transaction():
            updated = fixture.put("key", expected=old.version, payload="primary")
            dependent = fixture.put("dependent")
        self.assertEqual(outbox.current(updated.version.key), updated)
        self.assertEqual(outbox.current(dependent.version.key), dependent)
        fixture.conserved()

    def test_backoff_promotion_is_bounded_and_starvation_free_not_global_fifo(self):
        fixture = self.fixture(retry=RetryPolicy(5, 1, 100))
        outbox = fixture.outbox
        worker = fixture.start()
        first = fixture.put("slow-deadline")
        for index in range(128):
            fixture.put("fast" + str(index))
        for index in range(129):
            claim = outbox.claim_next(worker)
            outbox.fail(claim, category="temporary")
            if index == 0:
                # A different per-version policy is legal. Make the oldest
                # retry's deadline later than two promotion pages of followers.
                with fixture.storage.transaction() as tx:
                    tx.execute(
                        "UPDATE memory_vectorization_state SET next_attempt_at=? WHERE receipt=?",
                        (fixture.clock.now + 100, first.token),
                    )
        fixture.clock.now += 200
        served = []
        for index in range(3):
            fixture.put("new arrival" + str(index))
            claim = outbox.claim_next(worker)
            served.append(claim.receipt.token)
            outbox.complete(claim, publish=fixture.publish)
            fixture.conserved()
        self.assertNotEqual(served[0], first.token)
        self.assertEqual(served[2], first.token)

    def test_expiry_during_sql_publication_rolls_back_vector_write(self):
        fixture = self.fixture()
        worker = fixture.start()
        fixture.put("key")
        claim = fixture.outbox.claim_next(worker, lease_seconds=5)

        def publish(tx, job):
            fixture.publish(tx, job)
            fixture.clock.now += 6

        with self.assertRaises(OutboxConflict):
            fixture.outbox.complete(claim, publish=publish)
        self.assertEqual(fixture.count("probe_vectors"), 0)
        self.assertEqual(fixture.conserved()["in_flight_current"], 1)
        self.assertFalse(fixture.outbox.complete(claim, publish=fixture.publish).published)
        self.assertEqual(fixture.conserved()["pending_ready"], 1)

    def test_proven_metadata_only_cas_keeps_slow_claim_and_latest_metadata(self):
        fixture = self.fixture()
        worker = fixture.start()
        receipt = fixture.put("key")
        claim = fixture.outbox.claim_next(worker)
        with fixture.storage.transaction() as tx:
            tx.execute("ALTER TABLE probe_source ADD COLUMN business_revision INTEGER NOT NULL DEFAULT 0")
            tx.execute("ALTER TABLE probe_source ADD COLUMN pose_metadata TEXT")
        for revision in range(50):
            with fixture.storage.transaction() as tx:
                # This synthetic field is explicitly absent from the embedding
                # input. Real projection/dependency/model equivalence is owned by
                # integration, not inferred by the outbox from arbitrary fields.
                self.assertEqual(fixture.outbox.current(receipt.version.key), receipt)
                changed = tx.execute(
                    """UPDATE probe_source SET business_revision=?, pose_metadata=?
                    WHERE library=? AND kind=? AND id=? AND incarnation=? AND revision=? AND business_revision=?""",
                    (revision + 1, "pose " + str(revision))
                    + receipt.version.key.sql()
                    + (receipt.version.record_incarnation, receipt.version.revision, revision),
                ).rowcount
                self.assertEqual(changed, 1)
            fixture.conserved()
        # An old whole-record/business snapshot cannot overwrite newer metadata
        # merely because its indexing-input token is still unchanged.
        with fixture.storage.transaction() as tx:
            changed = tx.execute("UPDATE probe_source SET pose_metadata='stale' WHERE business_revision=0").rowcount
            self.assertEqual(changed, 0)
        self.assertTrue(fixture.outbox.complete(claim, publish=fixture.publish).published)
        with fixture.storage.read_snapshot() as tx:
            self.assertEqual(tx.execute("SELECT pose_metadata FROM probe_source").fetchone()[0], "pose 49")
        snapshot = fixture.conserved()
        self.assertEqual(snapshot["accepted"], 1)
        self.assertEqual(snapshot["superseded"], 0)
        self.assertEqual(snapshot["completed_fresh"], 1)

    def test_partial_publication_survives_failure_then_retry_and_completion(self):
        fixture = self.fixture(retry=RetryPolicy(3, 2, 2))
        worker = fixture.start()
        receipt = fixture.put("key")
        claim = fixture.outbox.claim_next(worker)
        result = fixture.outbox.publish_partial(claim, publish=fixture.publish)
        self.assertTrue(result.published)
        self.assertEqual(result.reason, "partial_published")
        snapshot = fixture.conserved()
        self.assertEqual(snapshot["completed_fresh"], 0)
        self.assertEqual(snapshot["attempts_completed"], 0)
        self.assertEqual(snapshot["in_flight_current"], 1)
        self.assertEqual(snapshot["occupied_claim_slots"], 1)
        self.assertEqual(fixture.count("probe_vectors"), 1)
        fixture.outbox.fail(claim, category="image_io")
        self.assertEqual(fixture.count("probe_vectors"), 1)
        self.assertEqual(fixture.conserved()["pending_backoff"], 1)
        fixture.clock.now += 2
        new_claim = fixture.outbox.claim_next(worker)
        self.assertEqual(new_claim.receipt, receipt)
        fixture.outbox.fail(new_claim, category="permanent_image_input", permanent=True)
        self.assertEqual(fixture.count("probe_vectors"), 1)
        retry = fixture.outbox.retry(receipt)
        final = fixture.outbox.claim_next(worker)
        self.assertEqual(final.receipt, retry)
        self.assertTrue(fixture.outbox.complete(final, publish=fixture.publish).published)
        snapshot = fixture.conserved()
        self.assertEqual(snapshot["accepted"], 2)
        self.assertEqual(snapshot["completed_fresh"], 1)
        self.assertEqual(snapshot["failed_terminal"], 1)

    def test_partial_publication_fences_stale_expired_and_newer_input(self):
        fixture = self.fixture()
        worker = fixture.start()
        original = fixture.put("key")
        old = fixture.outbox.claim_next(worker)
        self.assertTrue(fixture.outbox.publish_partial(old, publish=fixture.publish).published)
        newer = fixture.put("key", expected=original.version, payload="new version")
        self.assertFalse(fixture.outbox.publish_partial(old, publish=fixture.publish).published)
        self.assertEqual(fixture.count("probe_vectors"), 0)
        self.assertEqual(fixture.conserved()["occupied_claim_slots"], 1)
        self.assertIsNone(fixture.outbox.claim_next(worker))
        self.assertFalse(fixture.outbox.complete(old, publish=fixture.publish).published)
        claim = fixture.outbox.claim_next(worker, lease_seconds=5)
        invalid = replace(claim, receipt=replace(newer, token="wrong"))
        self.assertFalse(fixture.outbox.publish_partial(invalid, publish=fixture.publish).published)
        self.assertEqual(fixture.conserved()["occupied_claim_slots"], 1)
        fixture.clock.now += 6
        self.assertFalse(fixture.outbox.publish_partial(claim, publish=fixture.publish).published)
        self.assertEqual(fixture.count("probe_vectors"), 0)
        self.assertEqual(fixture.conserved()["occupied_claim_slots"], 1)
        self.assertEqual(fixture.conserved()["attempts_abandoned"], 0)
        fixture.outbox.fail(claim, category="expired attempt ended")
        next_claim = fixture.outbox.claim_next(worker)
        fixture.outbox.activate(None)
        self.assertFalse(fixture.outbox.publish_partial(next_claim, publish=fixture.publish).published)
        self.assertEqual(fixture.conserved()["occupied_claim_slots"], 1)
        self.assertFalse(fixture.outbox.complete(next_claim, publish=fixture.publish).published)
        snapshot = fixture.conserved()
        self.assertEqual(snapshot["completed_fresh"], 0)
        self.assertEqual(snapshot["suspended"], 1)

    def test_partial_callback_failure_and_mid_publication_expiry_roll_back(self):
        fixture = self.fixture()
        worker = fixture.start()
        fixture.put("key")
        claim = fixture.outbox.claim_next(worker, lease_seconds=5)
        callbacks = []

        def failed(tx, job):
            fixture.publish(tx, job)
            tx.after_commit(lambda: callbacks.append("must not run"))
            tx.execute("INSERT INTO absent_table VALUES (1)")

        with self.assertRaises(sqlite3.OperationalError):
            fixture.outbox.publish_partial(claim, publish=failed)
        self.assertFalse(callbacks)
        self.assertEqual(fixture.count("probe_vectors"), 0)
        self.assertEqual(fixture.conserved()["in_flight_current"], 1)
        with self.assertRaisesRegex(OutboxConflict, "reenter"):
            fixture.outbox.publish_partial(claim, publish=lambda tx, job: fixture.outbox.activate(None))
        self.assertTrue(fixture.outbox.publish_partial(claim, publish=fixture.publish).published)

        def expired(tx, job):
            tx.execute("UPDATE probe_vectors SET vector='must roll back'")
            fixture.clock.now += 6

        with self.assertRaises(OutboxConflict):
            fixture.outbox.publish_partial(claim, publish=expired)
        with fixture.storage.read_snapshot() as tx:
            self.assertEqual(tx.execute("SELECT vector FROM probe_vectors").fetchone()[0], "[1.0,0.0]")
        self.assertEqual(fixture.conserved()["completed_fresh"], 0)

    def test_retry_backoff_exhaustion_reopen_and_explicit_retry(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "retry.sqlite")
            clock = Clock()
            fixture = Fixture(path, clock=clock, retry=RetryPolicy(3, 2, 3))
            worker = fixture.start()
            first = fixture.put("failure")
            fixture.put("healthy")
            claim = fixture.outbox.claim_next(worker)
            fixture.outbox.fail(claim, category="provider_timeout", message="x" * 5000)
            self.assertEqual(len(fixture.outbox.inspect(first.version.key)["error_message"]), 1024)
            self.assertEqual(fixture.conserved()["pending_backoff"], 1)
            healthy = fixture.outbox.claim_next(worker)
            self.assertEqual(healthy.receipt.version.key.point_id, "healthy")
            fixture.outbox.complete(healthy, publish=fixture.publish)
            self.assertIsNone(fixture.outbox.claim_next(worker))
            fixture.storage.close()
            # Constructor retry policy does not reset a persisted version's budget.
            fixture = Fixture(path, clock=clock, retry=RetryPolicy(99, 99, 99))
            self.addCleanup(fixture.storage.close)
            worker = fixture.outbox.start_worker(recover=True)
            clock.now += 2
            claim = fixture.outbox.claim_next(worker)
            self.assertEqual(claim.attempt, 2)
            fixture.outbox.fail(claim, category="provider_timeout")
            state = fixture.outbox.inspect(first.version.key)
            self.assertEqual(state["next_attempt_at"], clock.now + 3)
            self.assertEqual(state["max_attempts"], 3)
            clock.now += 3
            claim = fixture.outbox.claim_next(worker)
            fixture.outbox.fail(claim, category="provider_timeout")
            snapshot = fixture.conserved()
            self.assertEqual(snapshot["failed_terminal"], 1)
            self.assertEqual(snapshot["blocked_current"], 1)
            self.assertIsNone(fixture.outbox.claim_next(worker))
            retry = fixture.outbox.retry(first)
            self.assertNotEqual(retry.token, first.token)
            self.assertEqual(retry.version, first.version)
            with self.assertRaises(OutboxConflict):
                fixture.outbox.retry(first)
            new_claim = fixture.outbox.claim_next(worker)
            fixture.outbox.fail(claim, category="stale failure")
            self.assertEqual(fixture.outbox.inspect(first.version.key)["attempts"], 1)
            self.assertTrue(fixture.outbox.complete(new_claim, publish=fixture.publish).published)
            snapshot = fixture.conserved()
            self.assertEqual(snapshot["accepted"], 3)
            self.assertEqual(snapshot["failed_terminal"], 1)
            self.assertEqual(snapshot["completed_fresh"], 2)

    def test_permanent_failure_updates_and_receipt_retention(self):
        fixture = self.fixture(history_limit=1)
        worker = fixture.start()
        old = fixture.put("key")
        claim = fixture.outbox.claim_next(worker)
        fixture.outbox.fail(claim, category="invalid_input", permanent=True)
        self.assertEqual(fixture.outbox.receipt_outcome(old.token), "failed_terminal")
        new = fixture.put("key", expected=old.version, payload="fixed")
        self.assertEqual(fixture.outbox.receipt_outcome(old.token), "detail_expired_or_unknown")
        self.assertEqual(fixture.outbox.receipt_outcome("never issued"), "detail_expired_or_unknown")
        current = fixture.outbox.claim_next(worker)
        fixture.outbox.complete(current, publish=fixture.publish)
        fixture.put("another")
        self.assertEqual(fixture.outbox.receipt_outcome(new.token), "completed_fresh")
        fixture.conserved()

    def test_delete_recreate_and_library_aba(self):
        fixture = self.fixture()
        outbox = fixture.outbox
        worker = fixture.start()
        old = fixture.put("same")
        claim = outbox.claim_next(worker)
        fixture.delete(old)
        new = fixture.put("same")
        self.assertEqual(new.version.revision, old.version.revision)
        self.assertNotEqual(new.version.record_incarnation, old.version.record_incarnation)
        self.assertFalse(outbox.complete(claim, publish=fixture.publish).published)
        new_claim = outbox.claim_next(worker)
        self.assertEqual(new_claim.receipt, new)
        other_kind = fixture.put("same", kind="scene")
        old_library = fixture.library
        fixture.drop(old_library)
        self.assertEqual(fixture.count("probe_source"), 0)
        fixture.library = outbox.create_library("A")
        self.assertNotEqual(old_library.incarnation, fixture.library.incarnation)
        recreated = fixture.put("same")
        with self.assertRaises(OutboxConflict):
            outbox.activate(old_library)
        with self.assertRaises(OutboxConflict):
            fixture.put("ghost", library=old_library)
        outbox.activate(fixture.library)
        self.assertFalse(outbox.complete(new_claim, publish=fixture.publish).published)
        self.assertEqual(outbox.current(recreated.version.key), recreated)
        self.assertIsNone(outbox.current(other_kind.version.key))
        snapshot = fixture.conserved()
        self.assertEqual(snapshot["cancelled_deleted"], 3)
        self.assertEqual(snapshot["pending_ready"], 1)

    def test_activation_aba_all_kinds_and_suspended_work(self):
        fixture = self.fixture()
        outbox = fixture.outbox
        worker = fixture.start()
        for kind in ("location", "semantic", "instance", "observation", "scene"):
            fixture.put("same", kind=kind)
        old_claim = outbox.claim_next(worker)
        other = outbox.create_library("B")
        fixture.put("b", library=other)
        outbox.activate(other)
        self.assertEqual(fixture.conserved()["suspended"], 5)
        outbox.activate(fixture.library)
        self.assertIsNone(outbox.claim_next(worker))
        self.assertFalse(outbox.complete(old_claim, publish=fixture.publish).published)
        for _ in range(5):
            claim = outbox.claim_next(worker)
            self.assertNotEqual(claim.activation_generation, old_claim.activation_generation)
            outbox.complete(claim, publish=fixture.publish)
            fixture.conserved()
        self.assertEqual(fixture.conserved()["suspended"], 1)
        self.assertEqual(fixture.conserved()["attempts_cancelled"], 1)
        outbox.activate(other)
        claim = outbox.claim_next(worker)
        self.assertEqual(claim.receipt.version.key.library_incarnation, other.incarnation)
        outbox.complete(claim, publish=fixture.publish)
        self.assertEqual(fixture.conserved()["completed_fresh"], 6)

    def test_each_identity_guard_rejects_before_publication(self):
        fixture = self.fixture()
        worker = fixture.start()
        receipt = fixture.put("key")
        claim = fixture.outbox.claim_next(worker)
        version = receipt.version
        variants = (
            replace(claim, token="wrong"),
            replace(claim, worker_token="wrong"),
            replace(claim, activation_generation=claim.activation_generation + 1),
            replace(claim, attempt=claim.attempt + 1),
            replace(claim, receipt=replace(receipt, token="wrong")),
            replace(claim, receipt=replace(receipt, token="wrong")),
            replace(claim, receipt=replace(receipt, version=replace(version, revision=version.revision + 1))),
            replace(claim, receipt=replace(receipt, version=replace(version, record_incarnation="wrong"))),
            replace(
                claim, receipt=replace(receipt, version=replace(version, key=replace(version.key, source_kind="wrong")))
            ),
            replace(
                claim,
                receipt=replace(
                    receipt, version=replace(version, key=replace(version.key, library_incarnation="wrong"))
                ),
            ),
        )
        published = []
        for invalid in variants:
            self.assertFalse(fixture.outbox.complete(invalid, publish=lambda tx, c: published.append(c)).published)
            self.assertEqual(fixture.conserved()["occupied_claim_slots"], 1)
        self.assertFalse(published)
        fixture.outbox.activate(fixture.library)
        invalid = replace(claim, receipt=replace(receipt, token="wrong"), attempt=999)
        self.assertFalse(fixture.outbox.complete(invalid, publish=fixture.publish).published)
        self.assertEqual(fixture.conserved()["occupied_claim_slots"], 1)
        self.assertFalse(fixture.outbox.complete(claim, publish=fixture.publish).published)
        self.assertEqual(fixture.conserved()["occupied_claim_slots"], 0)

    def test_lease_renewal_takeover_old_results_do_not_release_new_slot(self):
        fixture = self.fixture(retry=RetryPolicy(3, 1, 1))
        worker = fixture.start()
        fixture.put("key")
        old = fixture.outbox.claim_next(worker, lease_seconds=5)
        fixture.clock.now += 4
        self.assertTrue(fixture.outbox.renew(old, lease_seconds=5))
        fixture.clock.now += 2
        self.assertIsNone(fixture.outbox.claim_next(worker))
        fixture.clock.now += 4
        self.assertFalse(fixture.outbox.renew(old))
        new = fixture.outbox.claim_next(worker)
        self.assertIsNotNone(new)
        self.assertNotEqual(new.token, old.token)
        self.assertFalse(fixture.outbox.complete(old, publish=fixture.publish).published)
        self.assertEqual(fixture.conserved()["occupied_claim_slots"], 1)
        newer_worker = fixture.outbox.start_worker(recover=True)
        latest = fixture.outbox.claim_next(newer_worker)
        fixture.outbox.fail(new, category="late")
        self.assertEqual(fixture.conserved()["occupied_claim_slots"], 1)
        self.assertTrue(fixture.outbox.complete(latest, publish=fixture.publish).published)
        self.assertEqual(fixture.conserved()["attempts_abandoned"], 2)

    def test_stop_is_logical_and_does_not_allow_uncooperative_overlap(self):
        fixture = self.fixture()
        worker = fixture.start()
        fixture.put("key")
        claimed, release = threading.Event(), threading.Event()

        def fake_uncooperative_worker():
            claim = fixture.outbox.claim_next(worker)
            self.assertFalse(fixture.storage.in_transaction)
            claimed.set()
            if not release.wait(10):
                raise RuntimeError("test did not release fake inference")
            return fixture.outbox.complete(claim, publish=fixture.publish)

        with ThreadPoolExecutor(max_workers=1) as pool:
            future = pool.submit(fake_uncooperative_worker)
            self.assertTrue(claimed.wait(10))
            try:
                self.assertTrue(fixture.outbox.stop_worker(worker))
                self.assertFalse(future.done())
                self.assertFalse(fixture.outbox.stop_worker(worker))
                with self.assertRaises(OutboxConflict):
                    fixture.outbox.start_worker()
                fixture.put("unrelated-write")  # no inference-held DB lock
                fixture.conserved()
            finally:
                release.set()
            self.assertFalse(future.result(timeout=10).published)
        new_worker = fixture.outbox.start_worker()
        claim = fixture.outbox.claim_next(new_worker)
        self.assertIsNotNone(claim)
        fixture.outbox.complete(claim, publish=fixture.publish)
        fixture.conserved()

    def test_racing_claimers_share_one_slot_and_separate_file_owner(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "race.sqlite")
            fixture = Fixture(path)
            self.addCleanup(fixture.storage.close)
            other_storage = SQLiteStorage(path)
            self.addCleanup(other_storage.close)
            other = VectorizationOutbox(other_storage, clock=fixture.clock, history_limit=16)
            worker = fixture.start()
            for index in range(8):
                fixture.put(index)
            barrier = threading.Barrier(8)

            def attempt(index):
                barrier.wait(timeout=10)
                return (fixture.outbox if index % 2 else other).claim_next(worker)

            with ThreadPoolExecutor(max_workers=8) as pool:
                claims = list(pool.map(attempt, range(8)))
            self.assertEqual(sum(claim is not None for claim in claims), 1)
            fixture.conserved()

    def test_clock_reversal_delays_instead_of_premature_retry(self):
        fixture = self.fixture(retry=RetryPolicy(3, 3, 3))
        worker = fixture.start()
        fixture.put("key")
        claim = fixture.outbox.claim_next(worker)
        fixture.outbox.fail(claim, category="temporary")
        fixture.clock.now -= 100
        self.assertIsNone(fixture.outbox.claim_next(worker))
        fixture.clock.now += 102
        self.assertIsNone(fixture.outbox.claim_next(worker))
        fixture.clock.now += 1
        self.assertIsNotNone(fixture.outbox.claim_next(worker))
        fixture.conserved()

    def test_randomized_state_transitions_conserve_each_event(self):
        fixture = self.fixture(retry=RetryPolicy(3, 1, 2), history_limit=3)
        outbox = fixture.outbox
        worker = fixture.start()
        randomizer = random.Random(23)
        held = None
        for _ in range(500):
            event = randomizer.randrange(8)
            point = str(randomizer.randrange(7))
            key = WorkKey(fixture.library.incarnation, "semantic", point)
            current = outbox.current(key)
            if event <= 2:
                fixture.put(point, expected=current.version if current else None, payload=str(randomizer.randrange(10)))
            elif event == 3 and current:
                fixture.delete(current)
            elif event == 4 and held is None:
                held = outbox.claim_next(worker, lease_seconds=5)
            elif event == 5 and held is not None:
                outbox.complete(held, publish=fixture.publish)
                held = None
            elif event == 6 and held is not None:
                outbox.fail(held, category="synthetic", permanent=randomizer.randrange(3) == 0)
                held = None
            elif event == 7:
                outbox.activate(fixture.library if randomizer.randrange(3) else None)
            fixture.clock.now += randomizer.randrange(2)
            fixture.conserved()


_CRASH_CUTS = (
    "before_source_commit",
    "after_source_commit_before_wake",
    "after_claim",
    "after_inference_before_publish",
    "during_publish_before_commit",
    "after_publish_commit_before_notification",
)


def crash_child(path, cut):
    fixture = Fixture(path)
    worker = fixture.start()

    def crash():
        os.kill(os.getpid(), signal.SIGKILL)

    if cut == "before_source_commit":
        with fixture.storage.transaction():
            fixture.put("crash")
            crash()
    if cut == "after_source_commit_before_wake":
        fixture.outbox._wake = crash
    fixture.put("crash")
    claim = fixture.outbox.claim_next(worker)
    if cut == "after_claim":
        crash()
    synthetic_vector = (1.0, 0.0)  # computed without any DB lock/model
    assert not fixture.storage.in_transaction and synthetic_vector
    if cut == "after_inference_before_publish":
        crash()

    def publish(tx, job):
        fixture.publish(tx, job)
        if cut == "during_publish_before_commit":
            crash()

    if cut == "after_publish_commit_before_notification":
        fixture.outbox._wake = crash
    fixture.outbox.complete(claim, publish=publish)
    raise AssertionError("crash cut was not reached")


class CrashChecks(unittest.TestCase):
    def test_real_process_crash_cutpoints_disk_reopen(self):
        for cut in _CRASH_CUTS:
            with self.subTest(cut=cut), tempfile.TemporaryDirectory() as directory:
                path = str(Path(directory) / "crash.sqlite")
                process = subprocess.run(
                    [sys.executable, __file__, "--crash-child", path, cut],
                    capture_output=True,
                    text=True,
                    timeout=30,
                    check=False,
                )
                self.assertEqual(process.returncode, -signal.SIGKILL, process.stderr)
                fixture = Fixture(path)
                try:
                    snapshot = fixture.conserved()
                    expected = 0 if cut == "before_source_commit" else 1
                    self.assertEqual(snapshot["accepted"], expected)
                    self.assertEqual(fixture.count("probe_source"), expected)
                    committed = cut == "after_publish_commit_before_notification"
                    self.assertEqual(snapshot["completed_fresh"], int(committed))
                    self.assertEqual(fixture.count("probe_vectors"), int(committed))
                    worker = fixture.outbox.start_worker(recover=True)
                    fixture.conserved()
                    claim = fixture.outbox.claim_next(worker)
                    if expected and not committed:
                        self.assertIsNotNone(claim)
                        self.assertTrue(fixture.outbox.complete(claim, publish=fixture.publish).published)
                    else:
                        self.assertIsNone(claim)
                    snapshot = fixture.conserved()
                    self.assertEqual(snapshot["completed_fresh"], expected)
                    self.assertEqual(fixture.count("probe_vectors"), expected)
                finally:
                    fixture.storage.close()
                reopened = Fixture(path)
                try:
                    self.assertEqual(reopened.conserved()["completed_fresh"], expected)
                    reopened.outbox.start_worker(recover=True)
                    self.assertEqual(reopened.conserved()["completed_fresh"], expected)
                finally:
                    reopened.storage.close()

    def test_python38_syntax(self):
        root = Path(__file__).resolve().parents[1]
        for path in (
            root / "services/memory/sqlite_storage.py",
            root / "services/memory/vectorization_outbox.py",
            Path(__file__),
        ):
            ast.parse(path.read_text(encoding="utf-8"), feature_version=(3, 8))


if __name__ == "__main__":
    if len(sys.argv) == 4 and sys.argv[1] == "--crash-child":
        crash_child(sys.argv[2], sys.argv[3])
    else:
        unittest.main(verbosity=2)
