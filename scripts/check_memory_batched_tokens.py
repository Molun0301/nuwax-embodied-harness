#!/usr/bin/env python3
"""离线验证有界令牌批读；真实临时 SQLite、固定模型与事件屏障，无外部调用。"""
from __future__ import annotations

import ast
import json
import sqlite3
import sys
import tempfile
import threading
import unittest
from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
from pathlib import Path
from unittest.mock import patch

from pydantic import ValidationError

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from contracts.memory import SemanticMemoryEntry
from contracts.spatial_memory import NavigationTrajectoryEvent
from scripts import check_memory_durable_integration as integration
from scripts.check_memory_storage_transactions import records
from services.memory.durable_index import SOURCES, TOKEN
from services.memory.repository import MemoryRepository
from services.memory.sqlite_storage import SQLiteTransaction
from services.memory.vectorization_outbox import OutboxConflict


class BatchTokenChecks(unittest.TestCase):
    fixture = integration.IntegrationChecks.fixture
    memory = integration.IntegrationChecks.memory

    def rows(self, repo, tx, kind):
        table, column, _ = SOURCES[kind]
        return tx.execute(
            f"SELECT payload_json,map_name,library_name,{column} AS index_record_id "
            f"FROM {table} ORDER BY {column} DESC"
        ).fetchall()

    @contextmanager
    def statements(self, owner):
        found = []
        original = SQLiteTransaction.execute

        def capture(tx, sql, parameters=()):
            if tx.owner is owner:
                found.append((sql, tuple(parameters)))
            return original(tx, sql, parameters)

        with patch.object(SQLiteTransaction, "execute", capture):
            yield found

    def oracle(self, repo, tx, model, rows):
        return [repo._capture_loaded_token(tx, model, row) for row in rows]

    def populate(self, repo, count, kind="memory"):
        names = {"memory": "semantic_memory", "instance": "semantic_instance", "scene": "scene_space"}
        base = next(record for _, _, _, record in records() if type(record) is SOURCES[kind][2])
        column = SOURCES[kind][1]
        prefix = {"memory": "mem", "instance": "ins", "scene": "ssp"}[kind]
        with repo.storage.transaction():
            for index in range(count):
                record = base.model_copy(update={column: f"{prefix}_batch_20260101T000000Z_{index:08x}"}, deep=True)
                getattr(repo, "upsert_" + names[kind])(record)

    def test_all_kinds_match_single_row_oracle_without_unused_projection(self):
        service = self.fixture()
        repo = service._repository
        for name, _, _, record in records()[:5]:
            getattr(repo, "upsert_" + name)(record)
        with repo.storage.transaction() as tx:
            tx.execute("UPDATE memory_index_sources SET projection_json='not JSON',fingerprint='unused'")
        with repo.storage.read_snapshot() as tx:
            for kind, (_, _, model) in SOURCES.items():
                source_rows = self.rows(repo, tx, kind)
                expected = self.oracle(repo, tx, model, source_rows)
                with self.statements(repo.storage) as sql:
                    actual = repo._capture_loaded_tokens(tx, model, source_rows)
                self.assertEqual(actual, expected)
                self.assertEqual(len(sql), 1)
                self.assertTrue(sql[0][0].startswith("SELECT external_id,record_incarnation,business_revision "))
                self.assertNotIn("projection_json", sql[0][0])

    def test_chunks_empty_duplicates_order_missing_state_and_alias(self):
        service = self.fixture()
        repo = service._repository
        self.populate(repo, 501)
        with repo.storage.transaction() as tx:
            tx.execute("DELETE FROM memory_index_sources WHERE external_id LIKE '%00000001'")
        with repo.storage.read_snapshot() as tx:
            source_rows = self.rows(repo, tx, "memory")
            for count in (0, 1, 249, 250, 251, 501):
                selected = source_rows[:count]
                with self.subTest(count=count), self.statements(repo.storage) as sql:
                    actual = repo._capture_loaded_tokens(tx, SemanticMemoryEntry, selected)
                self.assertEqual(len(sql), (count + 249) // 250)
                self.assertTrue(all(len(params) <= 251 for _, params in sql))
                self.assertEqual(actual, self.oracle(repo, tx, SemanticMemoryEntry, selected))
            # 返回顺序来自源记录，重复 ID 不得被 SQL 的集合语义丢弃。
            mixed = [source_rows[-1], source_rows[0], source_rows[-2], source_rows[0], dict(source_rows[1])]
            mixed[-1].pop("index_record_id")
            actual = repo._capture_loaded_tokens(tx, SemanticMemoryEntry, mixed)
            self.assertEqual(actual, self.oracle(repo, tx, SemanticMemoryEntry, mixed))
            self.assertEqual(actual[2][0], "raw")
            self.assertIsNone(actual[-1])
            no_alias = [{"payload_json": "{}", "map_name": None, "library_name": None}] * 501
            with self.statements(repo.storage) as sql:
                self.assertEqual(repo._capture_loaded_tokens(tx, SemanticMemoryEntry, no_alias), [None] * 501)
            self.assertFalse(sql)

    def test_sqlite_parameter_budget_is_bounded(self):
        service = self.fixture()
        repo = service._repository
        repo.upsert_semantic_memory(self.memory())
        with repo.storage.read_snapshot() as tx:
            rows = self.rows(repo, tx, "memory") * 501
            connection = tx._connection
            if not hasattr(connection, "setlimit"):
                self.skipTest("SQLite runtime limit API requires Python 3.11")
            previous = connection.setlimit(sqlite3.SQLITE_LIMIT_VARIABLE_NUMBER, 251)
            try:
                with self.statements(repo.storage) as sql:
                    actual = repo._capture_loaded_tokens(tx, SemanticMemoryEntry, rows)
                self.assertEqual(actual, self.oracle(repo, tx, SemanticMemoryEntry, rows))
                self.assertEqual([len(params) for _, params in sql], [251, 251, 2])
            finally:
                connection.setlimit(sqlite3.SQLITE_LIMIT_VARIABLE_NUMBER, previous)

    def test_no_index_and_unsupported_models_issue_no_token_sql(self):
        service = self.fixture()
        repo = service._repository
        repo.upsert_semantic_memory(self.memory())
        with repo.storage.read_snapshot() as tx:
            source_rows = self.rows(repo, tx, "memory")
            with self.statements(repo.storage) as sql:
                self.assertEqual(repo._capture_loaded_tokens(tx, NavigationTrajectoryEvent, source_rows), [None])
                with patch.object(repo, "_index", None):
                    self.assertEqual(repo._capture_loaded_tokens(tx, SemanticMemoryEntry, source_rows), [None])
            self.assertFalse(sql)

    def test_token_values_are_not_coerced_and_source_kind_is_exact(self):
        service = self.fixture()
        repo = service._repository
        record = self.memory()
        repo.upsert_semantic_memory(record)
        with repo.storage.transaction() as tx:
            tx.execute("UPDATE memory_index_sources SET record_incarnation=?,business_revision=?",
                       (b"opaque-owner", "not-an-integer"))
            tx.execute("""INSERT INTO memory_index_sources
                SELECT 'instance',external_id,library_name,library_incarnation,internal_id || '-other',
                       'wrong-kind-owner',99,source_digest,projection_json,fingerprint
                FROM memory_index_sources WHERE source_kind='memory'""")
        with repo.storage.read_snapshot() as tx:
            source_rows = self.rows(repo, tx, "memory")
            actual = repo._capture_loaded_tokens(tx, SemanticMemoryEntry, source_rows)
            self.assertEqual(actual, self.oracle(repo, tx, SemanticMemoryEntry, source_rows))
            self.assertEqual(actual, [("business", b"opaque-owner", "not-an-integer")])

    def test_raw_fallback_keeps_exact_bytes_and_normalization_columns(self):
        service = self.fixture()
        repo = service._repository
        with patch.object(repo, "_index", None):
            repo.upsert_semantic_memory(self.memory(map_name=None))
        with repo.storage.transaction() as tx:
            tx.execute("UPDATE semantic_memories SET map_name=' map-column ',library_name=' library-column '")
        with repo.storage.read_snapshot() as tx:
            source_rows = self.rows(repo, tx, "memory")
            actual = repo._capture_loaded_tokens(tx, SemanticMemoryEntry, source_rows)
            self.assertEqual(actual, self.oracle(repo, tx, SemanticMemoryEntry, source_rows))
            self.assertEqual(actual[0], ("raw", (source_rows[0]["payload_json"], " map-column ", " library-column ")))

    def test_sql_errors_propagate_and_no_repair_is_attempted(self):
        service = self.fixture()
        repo = service._repository
        repo.upsert_semantic_memory(self.memory())
        with repo.storage.read_snapshot() as tx:
            source_rows = self.rows(repo, tx, "memory")
            failure = sqlite3.OperationalError("synthetic token read failure")
            original = SQLiteTransaction.execute

            def fail(frame, sql, parameters=()):
                if sql.startswith("SELECT external_id,record_incarnation,business_revision"):
                    raise failure
                return original(frame, sql, parameters)

            with patch.object(SQLiteTransaction, "execute", fail), self.assertRaises(sqlite3.OperationalError) as caught:
                repo._capture_loaded_tokens(tx, SemanticMemoryEntry, source_rows)
            self.assertIs(caught.exception, failure)

    def test_public_lists_execute_two_selects_and_validate_every_source(self):
        service = self.fixture()
        repo = service._repository
        self.populate(repo, 32, "instance")
        self.populate(repo, 32, "scene")
        for call in (service.list_semantic_instances, lambda: service.list_scene_spaces(limit=32)):
            with self.statements(repo.storage) as sql, patch.object(repo, "_load_record", wraps=repo._load_record) as load:
                actual = call()
            self.assertEqual(len(actual), 32)
            self.assertEqual(len(sql), 2)
            self.assertEqual(load.call_count, 32)
            with patch.object(repo, "_capture_loaded_tokens", side_effect=lambda tx, model, rows: self.oracle(repo, tx, model, rows)), self.statements(repo.storage) as baseline_sql:
                expected = call()
            self.assertEqual(actual, expected)
            self.assertEqual(len(baseline_sql), 33)

    def test_source_validation_errors_match_original_bulk_path(self):
        service = self.fixture()
        repo = service._repository
        record = self.memory()
        repo.upsert_semantic_memory(record)
        for update in ({"unused_unknown": True}, {"observation_count": -1}, {"pose": {"frame_id": "invalid frame"}}):
            payload = record.model_dump(mode="json")
            payload.update(update)
            with repo.storage.transaction() as tx:
                tx.execute("UPDATE semantic_memories SET payload_json=?", (json.dumps(payload),))
            errors = []
            for baseline in (True, False):
                helper = (lambda tx, model, rows: self.oracle(repo, tx, model, rows)) if baseline else repo._capture_loaded_tokens
                with patch.object(repo, "_capture_loaded_tokens", side_effect=helper):
                    try:
                        repo.load_semantic_memories()
                    except ValidationError as error:
                        errors.append((type(error), str(error), error.json() if hasattr(error, "json") else None))
            self.assertEqual(len(errors), 2)
            self.assertEqual(errors[0], errors[1])

    def test_all_bulk_callers_preserve_tokens_order_and_graph_revision(self):
        service = self.fixture()
        repo = service._repository
        for name, _, _, record in records()[:5]:
            if name == "scene_space":
                record.metadata["scene_mapping_managed"] = True
            getattr(repo, "upsert_" + name)(record)
        calls = []
        original = repo._capture_loaded_tokens

        def checked(tx, model, rows):
            expected = self.oracle(repo, tx, model, rows)
            actual = original(tx, model, rows)
            self.assertEqual(actual, expected)
            calls.append(model)
            return actual

        def signature(value):
            if isinstance(value, dict):
                return {key: signature(item) for key, item in value.items()}
            if isinstance(value, (tuple, list)):
                return [signature(item) for item in value]
            if hasattr(value, "model_dump"):
                return (value.model_dump(), getattr(value, TOKEN, None),
                        getattr(value, "_scene_graph_source_revision", None),
                        getattr(value, "_scene_decision_source_revision", None))
            return value

        operations = [repo.load_tagged_locations, repo.load_semantic_memories, repo.load_semantic_instances,
                      repo.load_observation_events, repo.load_scene_spaces,
                      lambda: repo.list_scene_spaces(library_name="library", limit=1),
                      lambda: repo.capture_scene_decision_entries("library", include_spaces=True),
                      lambda: repo.capture_scene_graph_sources("library")]
        for call in operations:
            with patch.object(repo, "_capture_loaded_tokens", checked):
                actual = call()
            with patch.object(repo, "_capture_loaded_tokens", side_effect=lambda tx, model, rows: self.oracle(repo, tx, model, rows)):
                expected = call()
            self.assertEqual(signature(actual), signature(expected))
        self.assertEqual(len(calls), 11)

    def test_paged_wal_snapshot_cannot_mix_new_token_with_old_source(self):
        for pause_batch in (1, 2):
            with self.subTest(pause_batch=pause_batch), tempfile.TemporaryDirectory() as directory:
                service = self.fixture(path=str(Path(directory) / "memory.db"))
                repo = service._repository
                record = self.memory()
                repo.upsert_semantic_memory(record)
                expected_token = getattr(record, TOKEN)
                entered, resume = threading.Event(), threading.Event()
                original = SQLiteTransaction.execute
                read_thread = []
                batch_count = []

                def pause(tx, sql, parameters=(), original=original, read_thread=read_thread,
                          batch_count=batch_count, pause_batch=pause_batch, entered=entered, resume=resume):
                    cursor = original(tx, sql, parameters)
                    if threading.get_ident() in read_thread and sql.startswith("SELECT external_id,record_incarnation,business_revision"):
                        batch_count.append(1)
                        if len(batch_count) == pause_batch:
                            entered.set()
                            if not resume.wait(5):
                                raise AssertionError("writer did not release token reader")
                    return cursor

                def reader(repo=repo, read_thread=read_thread):
                    read_thread.append(threading.get_ident())
                    with repo.storage.read_snapshot() as tx:
                        rows = self.rows(repo, tx, "memory") * 501
                        return rows[0]["payload_json"], repo._capture_loaded_tokens(tx, SemanticMemoryEntry, rows)

                with ThreadPoolExecutor(max_workers=1) as pool, patch.object(SQLiteTransaction, "execute", pause):
                    future = pool.submit(reader)
                    try:
                        self.assertTrue(entered.wait(5))
                        payload = record.model_dump(mode="json")
                        payload["title"] = "Committed after source capture"
                        with repo.storage.transaction() as tx:
                            tx.execute("UPDATE semantic_memories SET payload_json=?", (json.dumps(payload),))
                            tx.execute("UPDATE memory_index_sources SET business_revision=business_revision+1")
                    finally:
                        resume.set()
                    raw, tokens = future.result(timeout=5)
                self.assertEqual(json.loads(raw)["title"], record.title)
                self.assertEqual(tokens, [expected_token] * 501)
                fresh = repo.load_semantic_memories()[0]
                self.assertEqual(fresh.title, "Committed after source capture")
                self.assertEqual(getattr(fresh, TOKEN), (*expected_token[:2], expected_token[2] + 1))
                service.close()

    def test_wal_recreation_between_source_and_tokens_keeps_old_owner(self):
        with tempfile.TemporaryDirectory() as directory:
            service = self.fixture(path=str(Path(directory) / "memory.db"))
            repo = service._repository
            record = self.memory()
            repo.upsert_semantic_memory(record)
            expected_token = getattr(record, TOKEN)
            entered, resume = threading.Event(), threading.Event()
            original = repo._capture_loaded_tokens
            thread_ids = []

            def paused(tx, model, rows):
                if threading.get_ident() in thread_ids:
                    entered.set()
                    if not resume.wait(5):
                        raise AssertionError("writer did not release recreation reader")
                return original(tx, model, rows)

            def reader():
                thread_ids.append(threading.get_ident())
                return repo.load_semantic_memories()[0]

            with ThreadPoolExecutor(max_workers=1) as pool, patch.object(repo, "_capture_loaded_tokens", paused):
                future = pool.submit(reader)
                try:
                    self.assertTrue(entered.wait(5))
                    with repo.storage.transaction() as writer:
                        # 提交完成先放行读者；提交后的缓存通知可能另行读取数据库。
                        writer.after_completion(lambda committed: resume.set())
                        repo.delete_semantic_memories([record.memory_id])
                        fresh = SemanticMemoryEntry.model_validate(record.model_dump())
                        fresh.title = "Recreated source"
                        repo.upsert_semantic_memory(fresh)
                finally:
                    resume.set()
                old = future.result(timeout=5)
            self.assertEqual(old.title, record.title)
            self.assertEqual(getattr(old, TOKEN), expected_token)
            current = repo.load_semantic_memories()[0]
            self.assertEqual(current.title, "Recreated source")
            self.assertNotEqual(getattr(current, TOKEN)[1], expected_token[1])
            with self.assertRaises(OutboxConflict):
                repo.upsert_semantic_memory(old.model_copy(update={"title": "Stale write"}))
            service.close()

    def test_read_own_writes_rollback_and_recreated_owner_reject_stale_mutation(self):
        service = self.fixture()
        repo = service._repository
        record = self.memory()
        repo.upsert_semantic_memory(record)
        original = repo.load_semantic_memories()[0]
        with self.assertRaisesRegex(RuntimeError, "rollback"), repo.storage.transaction():
            repo.upsert_semantic_memory(original.model_copy(update={"title": "Uncommitted"}))
            with repo.storage.read_snapshot() as tx:
                rows = self.rows(repo, tx, "memory")
                self.assertEqual(repo._capture_loaded_tokens(tx, SemanticMemoryEntry, rows),
                                 self.oracle(repo, tx, SemanticMemoryEntry, rows))
            self.assertEqual(repo.load_semantic_memories()[0].title, "Uncommitted")
            raise RuntimeError("rollback")
        self.assertEqual(repo.load_semantic_memories()[0].title, original.title)
        repo.delete_semantic_memories([record.memory_id])
        fresh = SemanticMemoryEntry.model_validate(record.model_dump())
        repo.upsert_semantic_memory(fresh)
        recreated = repo.load_semantic_memories()[0]
        self.assertNotEqual(getattr(recreated, TOKEN)[1], getattr(original, TOKEN)[1])
        with self.assertRaises(OutboxConflict):
            repo.upsert_semantic_memory(original.model_copy(update={"title": "Stale write"}))

    def test_python38_syntax(self):
        for path in (Path(__file__), Path(MemoryRepository.__module__.replace(".", "/") + ".py")):
            ast.parse(path.read_text(), filename=str(path), feature_version=(3, 8))


if __name__ == "__main__":
    unittest.main(verbosity=2)
