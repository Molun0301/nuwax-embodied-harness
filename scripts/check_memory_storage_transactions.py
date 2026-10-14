#!/usr/bin/env python3
"""离线验证共享 SQLite、嵌套事务、兼容迁移及向量缓存快照。

运行 .nuwax-test-env/bin/python scripts/check_memory_storage_transactions.py。
仅使用合成记录、固定向量、临时数据库及事件屏障；不调用模型、网络或硬件。
"""
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
from datetime import datetime, timezone
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from contracts.geometry import Pose
from contracts.memory import MemoryPayloadFilter, SemanticMemoryEntry, TaggedLocation
from contracts.spatial_memory import (
    NavigationTrajectoryEvent,
    ObservationEvent,
    SceneSpace,
    SemanticGraphRelation,
    SemanticInstance,
    SemanticSummaryNode,
)
from services.memory.repository import MemoryRepository
from services.memory.sqlite_storage import SQLiteStorage, SQLiteTransaction
from services.memory.vector_store import NamedVector, SQLiteNamedVectorStore
from services.memory.vectorization_outbox import VectorizationOutbox, WorkKey

STAMP = "2026-01-01T00:00:00+00:00"
NOW = datetime(2026, 1, 1, tzinfo=timezone.utc)
IDS = {kind: kind + "_synthetic_20260101T000000Z_00000001" for kind in ("loc", "mem", "ins", "anc", "oev", "ssp")}


def put(store, point_id="point", vector=(1.0, 0.0), *, title="old", vectors=None, payload=None):
    if vectors is None:
        array = np.asarray(vector, dtype=np.float32)
        vectors = {"text_dense": NamedVector("text_dense", "fake2", array.size, array)}
    store.upsert_point(
        point_id=point_id, collection_name="place_nodes", record_kind="semantic_memory",
        payload=payload or {"title": title, "timestamp": STAMP}, vectors=vectors, updated_at=STAMP,
    )


def query(store, **kwargs):
    return store.query(vector_name="text_dense", query_vector=np.array([1.0, 0.0]), limit=5, **kwargs)


def records():
    common = {"timestamp": NOW, "map_name": "synthetic_map", "metadata": {"memory_library_name": "library"}}
    return [
        ("tagged_location", "tagged_locations", "location_id", TaggedLocation(
            location_id=IDS["loc"], name="Desk", normalized_name="desk", pose=Pose(frame_id="map"), **common)),
        ("semantic_memory", "semantic_memories", "memory_id", SemanticMemoryEntry(
            memory_id=IDS["mem"], title="Desk", summary="Synthetic", **common)),
        ("semantic_instance", "semantic_instances", "instance_id", SemanticInstance(
            instance_id=IDS["ins"], anchor_id=IDS["anc"], label="desk", pose=Pose(frame_id="map"),
            first_seen_ts=NOW, last_seen_ts=NOW, **common)),
        ("observation_event", "observation_events", "event_id", ObservationEvent(
            event_id=IDS["oev"], title="Desk", summary="Synthetic", **common)),
        ("scene_space", "scene_spaces", "scene_space_id", SceneSpace(
            scene_space_id=IDS["ssp"], frame_id="map", primary_label="office", **common)),
        ("navigation_trajectory_event", "navigation_trajectory_events", "trajectory_event_id", NavigationTrajectoryEvent(
            trajectory_event_id="nte_synthetic_000000000001", event_kind="visited_pose", **common)),
        ("semantic_graph_relation", "semantic_graph_relations", "relation_id", SemanticGraphRelation(
            relation_id="synthetic_relation", relation_kind="nearby", source_node_kind="scene_space",
            source_node_id=IDS["ssp"], target_node_kind="semantic_instance", target_node_id=IDS["ins"], **common)),
        ("semantic_summary_node", "semantic_summary_nodes", "summary_id", SemanticSummaryNode(
            summary_id="synthetic_summary", summary_kind="region", title="Office", summary="Synthetic", **common)),
    ]


class StorageChecks(unittest.TestCase):
    def fixture(self, *, file=False):
        temporary = tempfile.TemporaryDirectory(prefix="neh-storage-")
        self.addCleanup(temporary.cleanup)
        path = str(Path(temporary.name) / "memory.db") if file else ":memory:"
        owner = SQLiteStorage(path)
        self.addCleanup(owner.close)
        repo = MemoryRepository(storage=owner)
        store = SQLiteNamedVectorStore(storage=owner)
        self.addCleanup(repo.close)
        self.addCleanup(store.close)
        return owner, repo, store

    def test_standalone_constructor_and_owned_close(self):
        for cls in (MemoryRepository, SQLiteNamedVectorStore):
            with self.subTest(cls=cls.__name__):
                wrapper = cls(":memory:")
                owner = wrapper.storage
                wrapper.close()
                wrapper.close()
                with self.assertRaisesRegex(RuntimeError, "closed"), owner.read_snapshot():
                    pass

    def test_borrowed_wrappers_close_without_closing_owner(self):
        owner, repo, store = self.fixture()
        repo.close()
        store.close()
        with owner.transaction() as tx:
            self.assertEqual(tx.execute("SELECT 1").fetchone()[0], 1)
        for call in (repo.load_memory_libraries, store.count_points):
            with self.assertRaisesRegex(RuntimeError, "closed"):
                call()
        other = SQLiteNamedVectorStore(storage=owner)
        self.addCleanup(other.close)
        put(other)
        self.assertEqual(other.count_points(), 1)

    def test_failed_constructor_does_not_close_borrowed_owner(self):
        owner, _, _ = self.fixture()
        for cls in (MemoryRepository, SQLiteNamedVectorStore):
            with patch.object(cls, "_init_schema", side_effect=ValueError("synthetic schema failure")):  # noqa: SIM117
                with self.assertRaisesRegex(ValueError, "synthetic"):
                    cls(storage=owner)
            with owner.read_snapshot() as tx:
                self.assertEqual(tx.execute("SELECT 1").fetchone()[0], 1)

    def test_separate_memory_owners_are_not_shared(self):
        repo = MemoryRepository(":memory:")
        store = SQLiteNamedVectorStore(":memory:")
        self.addCleanup(repo.close)
        self.addCleanup(store.close)
        self.assertIsNot(repo.storage, store.storage)
        with self.assertRaises(sqlite3.OperationalError), repo.storage.read_snapshot() as tx:
            tx.execute("SELECT * FROM memory_vector_points")

    def test_full_durability_policy_is_owner_policy(self):
        owner, _, _ = self.fixture(file=True)
        with owner.transaction() as tx:
            self.assertEqual(tx.execute("PRAGMA synchronous").fetchone()[0], 2)
            self.assertEqual(tx.execute("PRAGMA journal_mode").fetchone()[0], "wal")

    def test_source_vector_outbox_commit_and_rollback_are_atomic(self):
        owner, repo, store = self.fixture()
        outbox = VectorizationOutbox(owner)
        library = outbox.create_library("library")
        key = WorkKey(library.incarnation, "semantic_memory", IDS["mem"])
        entry = records()[1][3]
        with self.assertRaisesRegex(ValueError, "rollback"), owner.transaction():
            repo.upsert_semantic_memory(entry)
            put(store)
            receipt = outbox.accept(key, expected=None, write_source=lambda tx, version: None)
            self.assertEqual(len(repo.load_semantic_memories()), 1)
            self.assertEqual(store.count_points(), 1)
            self.assertEqual(outbox.current(key), receipt)
            raise ValueError("rollback")
        self.assertEqual(repo.load_semantic_memories(), [])
        self.assertEqual(store.count_points(), 0)
        self.assertIsNone(outbox.current(key))
        with owner.transaction():
            repo.upsert_semantic_memory(entry)
            put(store)
            outbox.accept(key, expected=None, write_source=lambda tx, version: None)
        self.assertEqual(len(repo.load_semantic_memories()), 1)
        self.assertEqual(store.count_points(), 1)
        self.assertIsNotNone(outbox.current(key))

    def test_every_repository_write_family_nests_without_commit(self):
        owner, repo, _ = self.fixture()
        for singular, plural, key, entry in records():
            with self.subTest(kind=singular):
                write = getattr(repo, "upsert_" + singular)
                load = getattr(repo, "load_" + plural)
                with self.assertRaises(ValueError), owner.transaction():
                    write(entry)
                    loaded = load(library_name="library")
                    self.assertEqual(getattr(loaded[0], key), getattr(entry, key))
                    raise ValueError("rollback")
                self.assertEqual(load(), [])
                write(entry)
                self.assertEqual(load()[0], entry)
        self.assertEqual(sum(sum(counts.values()) for counts in repo.count_records_by_library().values()), 8)

    def test_every_repository_delete_family_and_clear_roll_back(self):
        owner, repo, _ = self.fixture()
        for singular, plural, key, entry in records():
            getattr(repo, "upsert_" + singular)(entry)
        for singular, plural, key, entry in records():
            with self.subTest(kind=singular):
                remove = getattr(repo, "delete_" + plural)
                load = getattr(repo, "load_" + plural)
                with self.assertRaises(ValueError), owner.transaction():
                    self.assertEqual(remove([getattr(entry, key)]), 1)
                    self.assertEqual(load(), [])
                    raise ValueError("rollback")
                self.assertEqual(len(load()), 1)
        with self.assertRaises(ValueError), owner.transaction():
            repo.clear_all()
            self.assertEqual(repo.count_records_by_library(), {})
            raise ValueError("rollback")
        self.assertEqual(len(repo.load_scene_spaces()), 1)

    def test_directory_legacy_vectors_and_source_delete_semantics(self):
        owner, repo, _ = self.fixture()
        entry = records()[0][3]
        with self.assertRaises(ValueError), owner.transaction():
            repo.upsert_memory_library("library")
            repo.upsert_vector_record(namespace="tagged_location", record_id=entry.location_id,
                                      embedding_model="fake2", embedding_dim=2, source_text="Desk",
                                      vector_payload="[1,0]", updated_at=STAMP)
            self.assertEqual(len(repo.load_memory_libraries()), 1)
            self.assertEqual(repo.load_vector_records()[entry.location_id].vector_payload, "[1,0]")
            raise ValueError("rollback")
        self.assertEqual(repo.load_memory_libraries(), [])
        self.assertEqual(repo.load_vector_records(), {})
        repo.upsert_memory_library("library")
        repo.upsert_tagged_location(entry)
        repo.upsert_vector_record(namespace="tagged_location", record_id=entry.location_id,
                                  embedding_model="fake2", embedding_dim=2, source_text="Desk",
                                  vector_payload="[1,0]", updated_at=STAMP)
        self.assertEqual(repo.delete_tagged_locations([entry.location_id, "missing"]), 2)
        self.assertEqual(repo.load_vector_records(), {})
        repo.clear_all()
        self.assertEqual(len(repo.load_memory_libraries()), 1)
        self.assertTrue(repo.delete_memory_library("library"))
        self.assertFalse(repo.delete_memory_library("library"))

    def test_inner_failure_preserves_outer_transaction(self):
        owner, repo, store = self.fixture()
        with owner.transaction():
            repo.upsert_memory_library("outer")
            with self.assertRaises(ValueError), owner.transaction():
                repo.upsert_memory_library("inner")
                put(store)
                raise ValueError("inner")
            self.assertEqual([r.library_name for r in repo.load_memory_libraries()], ["outer"])
            self.assertEqual(store.count_points(), 0)
            put(store, "outer_point")
        self.assertEqual(store.count_points(), 1)

    def test_read_own_writes_all_vector_read_paths_and_scene_filters(self):
        owner, repo, store = self.fixture(file=True)
        with owner.transaction():
            put(store)
            repo.upsert_scene_space(records()[4][3])
            self.assertEqual(store.get_point("point").payload["title"], "old")
            self.assertEqual(list(store.get_points_batch(["point"])), ["point"])
            self.assertEqual(list(store.load_points()), ["point"])
            self.assertEqual(store.count_points(), 1)
            self.assertEqual(store.count_points_by_collection(), {"place_nodes": 1})
            self.assertEqual(store.count_vectors_by_name(), {"text_dense": 1})
            self.assertAlmostEqual(query(store)[0].score, 1.0)
            self.assertIsNotNone(repo.load_scene_space(IDS["ssp"]))
            self.assertEqual(len(repo.list_scene_spaces(map_name="synthetic_map")), 1)
            self.assertEqual(len(repo.query_scene_spaces(MemoryPayloadFilter(map_name="synthetic_map"))), 1)

    def test_vector_full_replacement_delete_clear_and_duplicate_counts(self):
        owner, _, store = self.fixture()
        both = {name: NamedVector(name, "fake2", 2, np.array([1, 0])) for name in ("text_dense", "image_dense")}
        put(store, vectors=both)
        self.assertEqual(len(store.get_point("point").vectors), 2)
        put(store, title="replacement")
        self.assertEqual(set(store.get_point("point").vectors), {"text_dense"})
        with self.assertRaises(ValueError), owner.transaction():
            put(store, vectors={})
            self.assertEqual(store.get_point("point").vectors, {})
            self.assertEqual(query(store), ())
            raise ValueError("rollback")
        self.assertEqual(set(store.get_point("point").vectors), {"text_dense"})
        with self.assertRaises(ValueError), owner.transaction():
            store.clear_all()
            self.assertEqual(store.count_points(), 0)
            raise ValueError("rollback")
        self.assertEqual(store.delete_points(["point", "point", "missing"]), 3)
        self.assertEqual(query(store), ())
        self.assertEqual(store.count_vectors_by_name(), {})

    def test_cache_same_timestamp_update_rollback_and_aba(self):
        owner, _, store = self.fixture()
        put(store)
        self.assertAlmostEqual(query(store)[0].score, 1.0)
        old_token = store._cached_vectors["point"]["text_dense"][0]
        with self.assertRaises(ValueError), owner.transaction():
            put(store, vector=(0, 1), title="provisional")
            self.assertAlmostEqual(query(store)[0].score, 0.0)
            raise ValueError("rollback")
        hit = query(store)[0]
        self.assertEqual(hit.point.payload["title"], "old")
        self.assertAlmostEqual(hit.score, 1.0)
        self.assertEqual(store._cached_vectors["point"]["text_dense"][0], old_token)
        put(store, vector=(0, 1), title="new")
        self.assertAlmostEqual(query(store)[0].score, 0.0)
        self.assertNotEqual(store._cached_vectors["point"]["text_dense"][0], old_token)
        store.delete_points(["point"])
        put(store)
        self.assertAlmostEqual(query(store)[0].score, 1.0)
        self.assertNotEqual(store._cached_vectors["point"]["text_dense"][0], old_token)

    def test_cache_does_not_borrow_mutable_input_or_output_buffers(self):
        _, _, store = self.fixture()
        array = np.array([1.0, 0.0], dtype=np.float32)
        put(store, vector=array)
        array[:] = [0, 1]
        hit = query(store)[0]
        self.assertAlmostEqual(hit.score, 1.0)
        hit.point.vectors["text_dense"].vector[:] = [0, 1]
        cached = store._cached_vectors["point"]["text_dense"][1]
        with self.assertRaises(ValueError):
            cached[0] = 0
        with self.assertRaises(ValueError):
            cached.setflags(write=True)
        self.assertIsInstance(cached.base, bytes)
        self.assertAlmostEqual(query(store)[0].score, 1.0)
        np.testing.assert_array_equal(store.get_point("point").vectors["text_dense"].vector, [1, 0])

    def test_shared_and_independent_store_cache_tokens(self):
        owner, _, store = self.fixture(file=True)
        shared = SQLiteNamedVectorStore(storage=owner)
        independent = SQLiteNamedVectorStore(owner.db_path)
        self.addCleanup(shared.close)
        self.addCleanup(independent.close)
        put(store)
        query(store)
        for writer, value in ((shared, (0, 1)), (independent, (1, 0))):
            put(writer, vector=value, title=str(value))
            hit = query(store)[0]
            self.assertEqual(hit.point.payload["title"], str(value))
            self.assertAlmostEqual(hit.score, value[0])
        independent.delete_points(["point"])
        self.assertEqual(query(store), ())

    def test_wal_reader_finishes_while_writer_is_uncommitted(self):
        owner, repo, store = self.fixture(file=True)
        put(store)
        repo.upsert_memory_library("committed")
        with ThreadPoolExecutor(max_workers=1) as pool, owner.transaction():
            put(store, vector=(0, 1), title="uncommitted")
            repo.upsert_memory_library("uncommitted")
            future = pool.submit(lambda: (query(store)[0], repo.load_memory_libraries()))
            hit, libraries = future.result(timeout=3)
            self.assertEqual(hit.point.payload["title"], "old")
            self.assertAlmostEqual(hit.score, 1.0)
            self.assertEqual([row.library_name for row in libraries], ["committed"])
        self.assertAlmostEqual(query(store)[0].score, 0.0)

    def test_query_snapshot_cannot_pair_old_payload_with_new_vector(self):
        _, _, store = self.fixture(file=True)
        put(store)
        selected, resume = threading.Event(), threading.Event()
        original = store._select_candidate_rows

        def pause(*args, **kwargs):
            result = original(*args, **kwargs)
            selected.set()
            if not resume.wait(3):
                raise AssertionError("writer did not resume reader")
            return result

        with ThreadPoolExecutor(max_workers=1) as pool, patch.object(store, "_select_candidate_rows", side_effect=pause):
            future = pool.submit(query, store)
            try:
                self.assertTrue(selected.wait(3))
                put(store, vector=(0, 1), title="new")
            finally:
                resume.set()
            hit = future.result(timeout=3)[0]
        self.assertEqual(hit.point.payload["title"], "old")
        self.assertAlmostEqual(hit.score, 1.0)
        hit = query(store)[0]
        self.assertEqual(hit.point.payload["title"], "new")
        self.assertAlmostEqual(hit.score, 0.0)

    def test_post_commit_delay_and_late_cache_install_are_safe(self):
        owner, _, store = self.fixture(file=True)
        put(store)
        query(store)
        with owner.read_snapshot() as tx:
            old_row = tx.execute("SELECT * FROM memory_named_vectors").fetchone()
        committed, release = threading.Event(), threading.Event()

        def writer():
            with owner.transaction() as tx:
                put(store, vector=(0, 1), title="new")
                def notification():
                    committed.set()
                    if not release.wait(3):
                        raise AssertionError("reader did not release notification")
                tx.after_commit(notification)

        with ThreadPoolExecutor(max_workers=1) as pool:
            future = pool.submit(writer)
            try:
                self.assertTrue(committed.wait(3))
                self.assertAlmostEqual(query(store)[0].score, 0.0)
                store._snapshot_vector(old_row, cache_allowed=True)
                self.assertAlmostEqual(query(store)[0].score, 0.0)
            finally:
                release.set()
            future.result(timeout=3)

    def test_read_snapshot_cannot_upgrade_to_writer(self):
        owner, repo, store = self.fixture(file=True)
        put(store)
        with owner.read_snapshot():
            self.assertEqual(store.count_points(), 1)
            with self.assertRaisesRegex(RuntimeError, "upgrade"):
                repo.upsert_memory_library("not_written")
            with self.assertRaisesRegex(RuntimeError, "upgrade"):
                put(store, title="not_written")
        self.assertEqual(repo.load_memory_libraries(), [])
        self.assertEqual(store.get_point("point").payload["title"], "old")

    def test_borrowed_close_rejects_operation_waiting_for_owner(self):
        owner, repo, store = self.fixture()
        for wrapper, method in ((repo, repo.load_memory_libraries), (store, store.count_points)):
            entered, resume = threading.Event(), threading.Event()
            original = owner.read_snapshot

            @contextmanager
            def paused_scope(entered=entered, resume=resume, original=original):
                entered.set()
                if not resume.wait(3):
                    raise AssertionError("close did not release reader")
                with original() as tx:
                    yield tx

            with ThreadPoolExecutor(max_workers=1) as pool, patch.object(owner, "read_snapshot", paused_scope):
                future = pool.submit(method)
                try:
                    self.assertTrue(entered.wait(3))
                    wrapper.close()
                finally:
                    resume.set()
                with self.assertRaisesRegex(RuntimeError, "closed"):
                    future.result(timeout=3)

    def test_fault_mid_replacement_rolls_back_sql_and_cache(self):
        _, _, store = self.fixture()
        put(store)
        query(store)
        execute = SQLiteTransaction.execute

        def fail(tx, sql, parameters=()):
            if "INSERT INTO memory_named_vectors" in sql:
                raise sqlite3.OperationalError("synthetic disk full")
            return execute(tx, sql, parameters)

        with patch.object(SQLiteTransaction, "execute", fail), self.assertRaises(sqlite3.OperationalError):
            put(store, vector=(0, 1), title="failed")
        self.assertEqual(store.get_point("point").payload["title"], "old")
        self.assertAlmostEqual(query(store)[0].score, 1.0)

    def test_repository_backfill_is_nested_and_preserves_concurrent_payload(self):
        owner, repo, _ = self.fixture()
        entry = records()[1][3]
        repo.upsert_semantic_memory(entry)
        with owner.transaction() as tx:
            tx.execute("UPDATE semantic_memories SET library_name=NULL")
        with self.assertRaises(ValueError), owner.transaction():
            self.assertEqual(repo.backfill_library_names(batch_size=1), {"semantic_memories": 1})
            self.assertEqual(len(repo.load_semantic_memories(library_name="library")), 1)
            raise ValueError("rollback")
        with owner.read_snapshot() as tx:
            rows = tx.execute("SELECT memory_id AS pk, payload_json FROM semantic_memories").fetchall()
            self.assertIsNone(tx.execute("SELECT library_name FROM semantic_memories").fetchone()[0])
        prepared = repo._prepare_library_name_backfill(rows)
        changed = entry.model_copy(update={"metadata": {"memory_library_name": "new_library"}}).model_dump_json()
        with owner.transaction() as tx:
            tx.execute("UPDATE semantic_memories SET payload_json=?", (changed,))
            self.assertEqual(repo._backfill_library_names_batch_tx(tx, "semantic_memories", "memory_id", prepared), 0)
        self.assertEqual(repo.backfill_library_names(batch_size=1), {"semantic_memories": 1})
        self.assertEqual(len(repo.load_semantic_memories(library_name="new_library")), 1)
        self.assertEqual(repo.backfill_library_names(batch_size=1), {})

    def test_legacy_json_vectors_map_migration_and_reopen(self):
        owner, _, store = self.fixture(file=True)
        put(store, payload={"map_name": "already", "title": "canonical"})
        canonical = '{"map_name":"already","title":"canonical"}'
        legacy = json.dumps({"metadata": {"memory_library_name": " office "}, "map_version_id": "deprecated"})
        with owner.transaction() as tx:
            tx.execute("UPDATE memory_vector_points SET payload_json=? WHERE point_id='point'", (canonical,))
            tx.execute("UPDATE memory_named_vectors SET vector_payload='[1.0, 0.0]', cache_token=NULL")
            tx.execute("INSERT INTO memory_vector_points(point_id,record_kind,payload_json,updated_at) VALUES(?,?,?,?)",
                       ("legacy", "semantic_memory", legacy, STAMP))
            tx.execute("INSERT INTO memory_named_vectors(point_id,vector_name,model_name,dimension,vector_payload,updated_at)"
                       " VALUES(?,?,?,?,?,?)", ("legacy", "text_dense", "fake2", 2, "[1, 0]", STAMP))
        reopened = SQLiteNamedVectorStore(owner.db_path)
        self.addCleanup(reopened.close)
        self.assertEqual(reopened.get_point("legacy").payload["map_name"], "office")
        self.assertNotIn("map_version_id", reopened.get_point("legacy").payload)
        self.assertEqual([hit.point.point_id for hit in query(reopened, payload_filter=MemoryPayloadFilter(map_name="office"))],
                         ["legacy"])
        with owner.read_snapshot() as tx:
            self.assertEqual(tx.execute("SELECT payload_json FROM memory_vector_points WHERE point_id='point'").fetchone()[0],
                             canonical)
            rows = tx.execute("SELECT vector_payload, cache_token, typeof(vector_payload) FROM memory_named_vectors"
                              " ORDER BY point_id").fetchall()
        self.assertEqual([tuple(row) for row in rows], [("[1, 0]", None, "text"), ("[1.0, 0.0]", None, "text")])
        self.assertEqual(len(reopened.load_points()), 2)
        self.assertEqual(len(reopened.get_points_batch(["legacy", "point"])), 2)

    def test_schema_migrations_and_optional_blob_migration_never_commit_outer_scope(self):
        owner, repo, store = self.fixture()
        put(store)
        with owner.transaction() as tx:
            tx.execute("UPDATE memory_named_vectors SET vector_payload='[1,0]', cache_token=NULL")
        with self.assertRaises(ValueError):  # noqa: SIM117 - 显式保留被测试的外层事务边界
            with owner.transaction():
                repo.upsert_memory_library("provisional")
                repo._init_schema()
                store._init_schema()
                store._migrate_vector_payload_to_blob()
                with owner.read_snapshot() as tx:
                    self.assertEqual(tx.execute("SELECT typeof(vector_payload) FROM memory_named_vectors").fetchone()[0], "blob")
                raise ValueError("rollback")
        self.assertEqual(repo.load_memory_libraries(), [])
        with owner.read_snapshot() as tx:
            self.assertEqual(tx.execute("SELECT typeof(vector_payload) FROM memory_named_vectors").fetchone()[0], "text")

    def test_payload_filter_excludes_malformed_legacy_vector_before_decoding(self):
        owner, _, store = self.fixture()
        put(store, "keep", payload={"semantic_labels": ["keep"]})
        put(store, "reject", payload={"semantic_labels": ["reject"]})
        with owner.transaction() as tx:
            tx.execute("UPDATE memory_named_vectors SET vector_payload='not json', cache_token=NULL"
                       " WHERE point_id='reject'")
        hits = query(store, payload_filter=MemoryPayloadFilter(semantic_labels_any=["keep"]))
        self.assertEqual([hit.point.point_id for hit in hits], ["keep"])

    def test_all_point_loaders_use_one_snapshot(self):
        _, _, store = self.fixture(file=True)
        execute = SQLiteTransaction.execute
        calls = (
            lambda: store.get_point("point"),
            lambda: store.get_points_batch(["point"])["point"],
            lambda: store.load_points()["point"],
        )
        for load in calls:
            put(store)
            selected, resume = threading.Event(), threading.Event()

            def pause(tx, sql, parameters=(), selected=selected, resume=resume):
                result = execute(tx, sql, parameters)
                if "SELECT point_id, collection_name" in sql:
                    selected.set()
                    if not resume.wait(3):
                        raise AssertionError("writer did not release loader")
                return result

            with ThreadPoolExecutor(max_workers=1) as pool, patch.object(SQLiteTransaction, "execute", pause):
                future = pool.submit(load)
                try:
                    self.assertTrue(selected.wait(3))
                    put(store, vector=(0, 1), title="new")
                finally:
                    resume.set()
                point = future.result(timeout=3)
            self.assertEqual(point.payload["title"], "old")
            np.testing.assert_array_equal(point.vectors["text_dense"].vector, [1, 0])

    def test_old_schema_adds_columns_without_reencoding_vectors_or_source_payload(self):
        temporary = tempfile.TemporaryDirectory(prefix="neh-legacy-")
        self.addCleanup(temporary.cleanup)
        path = str(Path(temporary.name) / "old.db")
        entry = records()[1][3].model_copy(update={"map_name": None})
        source_payload = entry.model_dump_json()
        point_payload = json.dumps({"metadata": {"memory_library_name": "library"}})
        with sqlite3.connect(path) as connection:
            connection.executescript("""
                CREATE TABLE semantic_memories(memory_id TEXT PRIMARY KEY, kind TEXT, title TEXT,
                    linked_location_id TEXT, payload_json TEXT NOT NULL, created_at TEXT, updated_at TEXT);
                CREATE TABLE memory_vector_points(point_id TEXT PRIMARY KEY, record_kind TEXT,
                    payload_json TEXT NOT NULL, updated_at TEXT);
                CREATE TABLE memory_named_vectors(point_id TEXT, vector_name TEXT, model_name TEXT,
                    dimension INTEGER, vector_payload TEXT, updated_at TEXT, PRIMARY KEY(point_id,vector_name));
                CREATE TRIGGER forbid_vector_migration BEFORE UPDATE OF vector_payload ON memory_named_vectors
                    BEGIN SELECT RAISE(ABORT, 'legacy vectors must not be rewritten on open'); END;
            """)
            connection.execute("INSERT INTO semantic_memories VALUES(?,?,?,?,?,?,?)",
                               (entry.memory_id, "scene", "Desk", None, source_payload, STAMP, STAMP))
            connection.execute("INSERT INTO memory_vector_points VALUES(?,?,?,?)",
                               ("legacy", "semantic_memory", point_payload, STAMP))
            connection.execute("INSERT INTO memory_named_vectors VALUES(?,?,?,?,?,?)",
                               ("legacy", "text_dense", "fake2", 2, "[1, 0]", STAMP))
        repo = MemoryRepository(path)
        store = SQLiteNamedVectorStore(storage=repo.storage)
        self.addCleanup(repo.close)
        self.addCleanup(store.close)
        self.assertEqual(repo.load_semantic_memories(library_name="library")[0].map_name, "library")
        self.assertAlmostEqual(query(store, payload_filter=MemoryPayloadFilter(map_name="library"))[0].score, 1.0)
        with repo.storage.read_snapshot() as tx:
            self.assertEqual(tx.execute("SELECT payload_json FROM semantic_memories").fetchone()[0], source_payload)
            self.assertEqual(tuple(tx.execute("SELECT vector_payload, cache_token FROM memory_named_vectors").fetchone()),
                             ("[1, 0]", None))

    def test_map_migration_pages_and_preserves_explicit_and_legacy_default_names(self):
        owner, _, store = self.fixture()
        rows = [(str(i), "semantic_memory", json.dumps({"metadata": {"memory_library_name": "library"}}), STAMP)
                for i in range(1003)]
        rows.extend([
            ("explicit", "semantic_memory", '{"map_name":"explicit","memory_library_name":"other"}', STAMP),
            ("default", "semantic_memory", '{"metadata":{"memory_library_name":"legacy_default"}}', STAMP),
        ])
        with owner.transaction() as tx:
            tx.executemany("INSERT INTO memory_vector_points(point_id,record_kind,payload_json,updated_at) VALUES(?,?,?,?)",
                           rows)
        store._migrate_point_map_names()
        with owner.read_snapshot() as tx:
            self.assertEqual(tx.execute("SELECT COUNT(*) FROM memory_vector_points WHERE map_name='library'").fetchone()[0],
                             1003)
        self.assertEqual(store.get_point("explicit").payload["map_name"], "explicit")
        self.assertIsNone(store.get_point("default").payload["map_name"])
        with owner.transaction() as tx:
            tx.execute("CREATE TRIGGER forbid_point_rewrite BEFORE UPDATE ON memory_vector_points"
                       " BEGIN SELECT RAISE(ABORT, 'normalized payload must not be rewritten'); END")
        store._migrate_point_map_names()

    def test_legacy_writer_retaining_new_token_cannot_select_stale_cache(self):
        owner, _, store = self.fixture()
        put(store)
        query(store)
        old_token = store._cached_vectors["point"]["text_dense"][0]
        for payload in (np.array([0, 1], dtype=np.float32).tobytes(), "[0, 1]"):
            with owner.transaction() as tx:
                tx.execute("UPDATE memory_named_vectors SET vector_payload=?", (payload,))
            self.assertAlmostEqual(query(store)[0].score, 0.0)
            self.assertEqual(store._cached_vectors["point"]["text_dense"][0], old_token)

    def test_unbounded_legacy_names_ids_and_unicode_payload_survive(self):
        owner, repo, store = self.fixture()
        library = "中文库" + "x" * 300
        point_id = "mem_" + "long" * 80 + "_20260101T000000Z_00000001"
        entry = SemanticMemoryEntry(memory_id=point_id, title="桌子", summary="抽屉里的地图",
                                    metadata={"memory_library_name": library})
        with owner.transaction():
            repo.upsert_semantic_memory(entry)
            repo.upsert_memory_library(library)
            put(store, point_id, payload={"title": "桌子", "memory_library_name": library})
        self.assertEqual(repo.load_semantic_memories(library_name=library)[0].memory_id, point_id)
        self.assertEqual(store.get_point(point_id).payload["map_name"], library)
        self.assertEqual(query(store)[0].point.payload["title"], "桌子")

    def test_real_store_publication_rolls_back_with_outbox_completion(self):
        owner, repo, store = self.fixture()
        outbox = VectorizationOutbox(owner)
        library = outbox.create_library("library")
        outbox.activate(library)
        entry = records()[1][3]
        key = WorkKey(library.incarnation, "semantic_memory", entry.memory_id)
        outbox.accept(key, expected=None,
                      write_source=lambda tx, version: repo.upsert_semantic_memory(entry))
        worker = outbox.start_worker()
        claim = outbox.claim_next(worker)
        self.assertIsNotNone(claim)

        def fail(tx, current):
            put(store, entry.memory_id)
            raise ValueError("synthetic publication failure")

        with self.assertRaises(ValueError):
            outbox.complete(claim, publish=fail)
        self.assertEqual(store.count_points(), 0)
        self.assertEqual(outbox.inspect(key)["status"], "in_flight")
        result = outbox.complete(claim, publish=lambda tx, current: put(store, entry.memory_id))
        self.assertTrue(result.published)
        self.assertEqual(store.count_points(), 1)
        self.assertEqual(outbox.inspect(key)["status"], "completed_fresh")

    def test_python38_syntax(self):
        for name in ("services/memory/repository.py", "services/memory/vector_store.py", __file__):
            ast.parse(Path(name).read_text(), filename=name, feature_version=(3, 8))


class CompletionChecks(unittest.TestCase):
    """完成通知必须跨保存点保留资源持有，并在真实外层退出后释放。"""

    def fixture(self):
        owner = SQLiteStorage()
        self.addCleanup(owner.close)
        with owner.transaction() as tx:
            tx.execute("CREATE TABLE completion_probe(value TEXT PRIMARY KEY)")
        return owner

    def rows(self, owner):
        with owner.read_snapshot() as tx:
            return [row[0] for row in tx.execute("SELECT value FROM completion_probe ORDER BY value")]

    def test_successful_nesting_completes_before_unchanged_commit_order(self):
        owner = self.fixture()
        calls = []
        with owner.transaction() as outer:
            outer.execute("INSERT INTO completion_probe VALUES ('outer')")
            outer.after_commit(lambda: calls.append("commit_outer"))
            outer.after_completion(lambda committed: calls.append(("outer", committed)))
            with owner.transaction() as inner:
                inner.execute("INSERT INTO completion_probe VALUES ('inner')")
                inner.after_completion(lambda committed: calls.append(("inner", committed)))
                inner.after_commit(lambda: calls.append("commit_inner"))
            outer.after_completion(lambda committed: calls.append(("last", committed)))
            self.assertEqual(calls, [])
        self.assertEqual(calls, [("outer", True), ("inner", True), ("last", True),
                                 "commit_outer", "commit_inner"])
        self.assertEqual(self.rows(owner), ["inner", "outer"])

    def test_caught_child_rollback_waits_for_outer_commit_including_grandchild(self):
        owner = self.fixture()
        calls = []
        with owner.transaction() as outer:
            outer.after_completion(lambda committed: calls.append(("outer", committed)))
            with self.assertRaisesRegex(ValueError, "child"), owner.transaction() as inner:
                inner.execute("INSERT INTO completion_probe VALUES ('inner')")
                inner.after_completion(lambda committed: calls.append(("inner", committed)))
                inner.after_commit(lambda: calls.append("forbidden_commit"))
                with owner.transaction() as grandchild:
                    grandchild.after_completion(lambda committed: calls.append(("grandchild", committed)))
                self.assertEqual(calls, [])
                raise ValueError("child")
            self.assertEqual(calls, [])
            outer.execute("INSERT INTO completion_probe VALUES ('outer')")
        self.assertEqual(calls, [("outer", True), ("inner", False), ("grandchild", False)])
        self.assertEqual(self.rows(owner), ["outer"])

    def test_rolled_back_grandchild_stays_false_through_successful_parent(self):
        owner = self.fixture()
        calls = []
        with owner.transaction() as outer:
            with owner.transaction() as inner:
                inner.after_completion(lambda committed: calls.append(("inner", committed)))
                with self.assertRaises(ValueError), owner.transaction() as grandchild:
                    grandchild.after_completion(lambda committed: calls.append(("grandchild", committed)))
                    raise ValueError("grandchild")
                self.assertEqual(calls, [])
            self.assertEqual(calls, [])
            outer.after_completion(lambda committed: calls.append(("outer", committed)))
        self.assertEqual(calls, [("inner", True), ("grandchild", False), ("outer", True)])

    def test_outer_baseexception_rolls_back_successful_children_exactly_once(self):
        for error in (ValueError("body"), KeyboardInterrupt("body"), SystemExit("body")):
            with self.subTest(error=type(error).__name__):
                owner = self.fixture()
                calls = []
                with self.assertRaises(type(error)) as caught:  # noqa: SIM117 - 显式保留被测事务边界
                    with owner.transaction() as outer:
                        outer.after_completion(lambda committed, calls=calls: calls.append(("outer", committed)))
                        with owner.transaction() as inner:
                            inner.execute("INSERT INTO completion_probe VALUES ('inner')")
                            inner.after_completion(lambda committed, calls=calls: calls.append(("inner", committed)))
                            inner.after_commit(lambda calls=calls: calls.append("forbidden_commit"))
                        raise error
                self.assertIs(caught.exception, error)
                self.assertEqual(calls, [("outer", False), ("inner", False)])
                self.assertEqual(self.rows(owner), [])

    def test_completion_does_not_hold_writer_lock_and_can_reenter(self):
        for rollback in (False, True):
            with self.subTest(rollback=rollback):
                owner = self.fixture()
                observations = []
                with ThreadPoolExecutor(max_workers=1) as pool:
                    def acquire_from_other_thread(owner=owner):
                        # 使用真实竞争线程，避免 RLock 在同线程重入导致假阳性。
                        acquired = owner._lock.acquire(timeout=1)
                        if acquired:
                            owner._lock.release()
                        return acquired

                    def completed(committed, owner=owner, observations=observations):
                        observations.append((committed, owner.current_transaction, frame._active,
                                             pool.submit(acquire_from_other_thread).result(timeout=3)))
                        with owner.transaction() as tx:
                            tx.execute("INSERT INTO completion_probe VALUES ('notification')")

                    try:
                        with owner.transaction() as frame:
                            frame.after_completion(completed)
                            if rollback:
                                raise ValueError("rollback")
                    except ValueError:
                        if not rollback:
                            raise
                self.assertEqual(observations, [(not rollback, None, False, True)])
                self.assertEqual(self.rows(owner), ["notification"])
                self.assertEqual(owner.completion_failure_count, 0)

    def test_aborted_child_notification_waits_for_outer_writer_lock_release(self):
        owner = self.fixture()
        calls = []
        with ThreadPoolExecutor(max_workers=1) as pool:
            def completed(committed):
                # 通知时另一线程必须能打开自己的写事务。
                calls.append((committed, pool.submit(self.rows, owner).result(timeout=3)))

            with owner.transaction() as outer:
                with self.assertRaises(ValueError), owner.transaction() as inner:
                    inner.after_completion(completed)
                    raise ValueError("child")
                self.assertEqual(calls, [])
                outer.execute("INSERT INTO completion_probe VALUES ('outer')")
        self.assertEqual(calls, [(False, ["outer"])])
        self.assertEqual(owner.completion_failure_count, 0)

    def test_commit_failure_with_live_transaction_rolls_back_before_completion(self):
        owner = self.fixture()
        with owner.transaction() as tx:
            tx.execute("CREATE TABLE parent_probe(id INTEGER PRIMARY KEY)")
            tx.execute("CREATE TABLE child_probe(id INTEGER REFERENCES parent_probe(id)"
                       " DEFERRABLE INITIALLY DEFERRED)")
        calls = []
        with self.assertRaises(sqlite3.IntegrityError), owner.transaction() as tx:
            tx.execute("INSERT INTO child_probe VALUES (123)")
            tx.after_completion(lambda committed: calls.append((committed, owner._connection.in_transaction)))
            tx.after_commit(lambda: calls.append("forbidden_commit"))
        self.assertEqual(calls, [(False, False)])
        with owner.read_snapshot() as tx:
            self.assertEqual(tx.execute("SELECT COUNT(*) FROM child_probe").fetchone()[0], 0)

    def test_error_after_actual_commit_is_unconfirmed_not_claimed_rollback(self):
        owner = self.fixture()
        control = owner._control
        failure = sqlite3.OperationalError("unconfirmed COMMIT response")
        calls = []

        def fail(connection, statement):
            control(connection, statement)
            if statement == "COMMIT":
                raise failure

        with patch.object(owner, "_control", side_effect=fail), self.assertRaises(sqlite3.OperationalError) as caught:  # noqa: SIM117
            with owner.transaction() as tx:
                tx.execute("INSERT INTO completion_probe VALUES ('committed')")
                tx.after_completion(calls.append)
                tx.after_commit(lambda: calls.append("forbidden_commit"))
        self.assertIs(caught.exception, failure)
        self.assertEqual(calls, [False])
        self.assertTrue(tx.completion.settled)
        self.assertEqual(self.rows(owner), ["committed"])
        self.assertEqual(owner.rollback_failure_count, 0)

    def test_nested_scopes_share_read_only_outer_completion_proof(self):
        owner = self.fixture()
        calls = []
        with owner.transaction() as outer:
            proof = outer.completion
            self.assertFalse(proof.settled)
            with owner.transaction() as inner:
                self.assertIs(inner.completion, proof)
                inner.after_completion(lambda committed: calls.append((committed, proof.settled)))
            with self.assertRaises(ValueError), owner.transaction() as aborted:
                self.assertIs(aborted.completion, proof)
                aborted.after_completion(lambda committed: calls.append((committed, proof.settled)))
                raise ValueError("aborted")
            self.assertFalse(proof.settled)
            self.assertEqual(calls, [])
            with self.assertRaises(AttributeError):
                proof.settled = True
            with self.assertRaises(AttributeError):
                outer.completion = None
        self.assertTrue(proof.settled)
        self.assertEqual(calls, [(True, True), (False, True)])

    def test_failed_rollback_stays_unsettled_until_actual_storage_close(self):
        owner = self.fixture()
        control = owner._control
        failure = ValueError("original error")
        calls = []

        def fail_rollback(connection, statement):
            if statement == "ROLLBACK":
                raise sqlite3.OperationalError("rollback never executed")
            control(connection, statement)

        with patch.object(owner, "_control", side_effect=fail_rollback), self.assertRaises(ValueError) as caught:  # noqa: SIM117
            with owner.transaction() as tx:
                proof = tx.completion
                tx.execute("INSERT INTO completion_probe VALUES ('still_uncommitted')")
                tx.after_completion(lambda committed: calls.append((committed, proof.settled)))
                raise failure
        self.assertIs(caught.exception, failure)
        self.assertEqual(calls, [(False, False)])
        self.assertTrue(owner._connection.in_transaction)
        self.assertFalse(proof.settled)
        with self.assertRaises(sqlite3.OperationalError), owner.transaction():
            self.fail("unresolved transaction cannot admit another outer BEGIN")
        self.assertIs(owner._unsettled_completion, proof)
        with patch.object(owner, "_connection") as unavailable:
            unavailable.close.side_effect = sqlite3.OperationalError("close failed")
            with self.assertRaises(sqlite3.OperationalError):
                owner.close()
        self.assertFalse(proof.settled)
        owner.close()
        self.assertTrue(proof.settled)
        self.assertIsNone(owner._unsettled_completion)
        self.assertEqual(calls, [(False, False)])

    def test_late_commit_or_rollback_settles_old_proof_at_next_owner_boundary(self):
        for resolution in ("COMMIT", "ROLLBACK"):
            with self.subTest(resolution=resolution):
                owner = self.fixture()
                control = owner._control
                calls = []

                def fail_rollback(connection, statement, control=control):
                    if statement == "ROLLBACK":
                        raise sqlite3.OperationalError("rollback never executed")
                    control(connection, statement)

                with patch.object(owner, "_control", side_effect=fail_rollback), self.assertRaises(ValueError):  # noqa: SIM117
                    with owner.transaction() as tx:
                        proof = tx.completion
                        tx.execute("INSERT INTO completion_probe VALUES ('late')")
                        tx.after_completion(calls.append)
                        raise ValueError("body")
                self.assertFalse(proof.settled)
                self.assertEqual(calls, [False])
                # 注入连接恢复结果；证明只有 owner 实际观察结束后才改变。
                owner._connection.execute(resolution)
                self.assertFalse(proof.settled)
                with owner.transaction() as next_tx:
                    self.assertTrue(proof.settled)
                    self.assertIsNone(owner._unsettled_completion)
                    self.assertIsNot(next_tx.completion, proof)
                    self.assertFalse(next_tx.completion.settled)
                    rows = [row[0] for row in next_tx.execute("SELECT value FROM completion_probe")]
                self.assertEqual(rows, ["late"] if resolution == "COMMIT" else [])
                self.assertTrue(next_tx.completion.settled)
                self.assertEqual(calls, [False])

    def test_commit_baseexception_dispatches_after_rollback(self):
        owner = self.fixture()
        control = owner._control
        failure = KeyboardInterrupt("COMMIT interrupted")
        calls = []

        def fail(connection, statement):
            if statement == "COMMIT":
                raise failure
            control(connection, statement)

        with patch.object(owner, "_control", side_effect=fail), self.assertRaises(KeyboardInterrupt) as caught:  # noqa: SIM117
            with owner.transaction() as tx:
                tx.execute("INSERT INTO completion_probe VALUES ('uncommitted')")
                tx.after_completion(calls.append)
        self.assertIs(caught.exception, failure)
        self.assertEqual(calls, [False])
        self.assertEqual(self.rows(owner), [])

    def test_failed_savepoint_release_defers_false_until_outer_commit(self):
        owner = self.fixture()
        control = owner._control
        calls = []
        failed = []

        def fail_once(connection, statement):
            if statement.startswith("RELEASE ") and not failed:
                failed.append(True)
                raise sqlite3.OperationalError("release failed")
            control(connection, statement)

        with patch.object(owner, "_control", side_effect=fail_once), owner.transaction() as outer:
            outer.after_completion(lambda committed: calls.append(("outer", committed)))
            with self.assertRaisesRegex(sqlite3.OperationalError, "release failed"), owner.transaction() as inner:
                inner.execute("INSERT INTO completion_probe VALUES ('inner')")
                inner.after_completion(lambda committed: calls.append(("inner", committed)))
            self.assertEqual(calls, [])
            outer.execute("INSERT INTO completion_probe VALUES ('outer')")
        self.assertEqual(calls, [("outer", True), ("inner", False)])
        self.assertEqual(self.rows(owner), ["outer"])

    def test_sqlite_whole_transaction_rollback_keeps_all_completion_hooks(self):
        for swallow_inside_child in (False, True):
            with self.subTest(swallow_inside_child=swallow_inside_child):
                owner = self.fixture()
                calls = []
                with self.assertRaisesRegex(RuntimeError, "rolled back"), owner.transaction() as outer:
                    outer.execute("INSERT INTO completion_probe VALUES ('duplicate')")
                    outer.after_completion(lambda committed, calls=calls: calls.append(("outer", committed)))
                    with self.assertRaises((sqlite3.IntegrityError, RuntimeError)), owner.transaction() as inner:
                        inner.after_completion(lambda committed, calls=calls: calls.append(("inner", committed)))
                        try:
                            inner.execute("INSERT OR ROLLBACK INTO completion_probe VALUES ('duplicate')")
                        except sqlite3.IntegrityError:
                            if not swallow_inside_child:
                                raise
                    self.assertEqual(calls, [])
                self.assertEqual(calls, [("outer", False), ("inner", False)])
                self.assertEqual(self.rows(owner), [])

    def test_completion_errors_are_bounded_separate_and_do_not_skip_notifications(self):
        owner = self.fixture()
        calls = []

        def completion_failure(committed):
            calls.append(committed)
            raise SystemExit("sensitive completion message must not be retained")

        def commit_failure():
            raise ValueError("sensitive commit message must not be retained")

        with owner.transaction() as tx:
            tx.execute("INSERT INTO completion_probe VALUES ('committed')")
            for _ in range(20):
                tx.after_completion(completion_failure)
                tx.after_commit(commit_failure)
            tx.after_completion(lambda committed: calls.append(("last", committed)))
            tx.after_commit(lambda: calls.append("commit_last"))
        self.assertEqual(calls, [True] * 20 + [("last", True), "commit_last"])
        self.assertEqual(owner.completion_failure_count, 20)
        self.assertEqual(owner.completion_errors, ("SystemExit",) * 16)
        self.assertEqual(owner.notification_failure_count, 20)
        self.assertEqual(owner.notification_errors, ("ValueError",) * 16)
        self.assertEqual(self.rows(owner), ["committed"])

    def test_completion_baseexception_cannot_replace_original_body_exception(self):
        owner = self.fixture()
        failure = ValueError("original body exception")
        calls = []

        def completion_failure(committed):
            raise KeyboardInterrupt("notification interrupted")

        with self.assertRaises(ValueError) as caught:  # noqa: SIM117 - 显式保留被测事务边界
            with owner.transaction() as tx:
                tx.after_completion(completion_failure)
                tx.after_completion(calls.append)
                raise failure
        self.assertIs(caught.exception, failure)
        self.assertEqual(calls, [False])
        self.assertEqual(owner.completion_errors, ("KeyboardInterrupt",))
        self.assertEqual(owner.notification_failure_count, 0)

    def test_completion_error_diagnostics_do_not_wait_for_a_competing_writer(self):
        owner = self.fixture()
        start_writer, writer_acquired, released = threading.Event(), threading.Event(), threading.Event()
        calls = []

        def writer():
            if not start_writer.wait(3):
                raise AssertionError("completion did not start the competing writer")
            with owner.transaction():
                writer_acquired.set()
                if not released.wait(3):
                    raise AssertionError("completion diagnostics blocked the remaining resource release")

        def failure(committed):
            start_writer.set()
            if not writer_acquired.wait(3):
                raise AssertionError("writer did not acquire the lock during completion")
            raise ValueError("synthetic completion failure")

        def release(committed):
            calls.append((committed, owner.completion_errors))
            released.set()

        with ThreadPoolExecutor(max_workers=1) as pool:
            future = pool.submit(writer)
            try:
                with owner.transaction() as tx:
                    tx.after_completion(failure)
                    tx.after_completion(release)
            finally:
                released.set()
            future.result(timeout=3)
        self.assertEqual(calls, [(True, ("ValueError",))])
        self.assertEqual(owner.completion_failure_count, 1)

    def test_rollback_cleanup_failure_does_not_mask_original_or_lose_completion(self):
        owner = self.fixture()
        control = owner._control
        failure = ValueError("original body exception")
        calls = []

        def fail(connection, statement):
            control(connection, statement)
            if statement == "ROLLBACK":
                raise KeyboardInterrupt("rollback response interrupted")

        with patch.object(owner, "_control", side_effect=fail), self.assertRaises(ValueError) as caught:  # noqa: SIM117
            with owner.transaction() as tx:
                tx.after_completion(calls.append)
                raise failure
        self.assertIs(caught.exception, failure)
        self.assertEqual(calls, [False])
        self.assertEqual(owner.rollback_failure_count, 1)
        self.assertEqual(owner.rollback_errors, ("KeyboardInterrupt",))
        self.assertFalse(owner.in_transaction)
        self.assertFalse(owner._connection.in_transaction)

    def test_commit_hook_baseexception_behavior_is_unchanged_after_all_completions(self):
        owner = self.fixture()
        failure = KeyboardInterrupt("ordinary commit hook")
        calls = []

        def commit_failure():
            raise failure

        with self.assertRaises(KeyboardInterrupt) as caught:  # noqa: SIM117 - 显式保留被测事务边界
            with owner.transaction() as tx:
                tx.after_commit(commit_failure)
                tx.after_completion(calls.append)
                tx.after_completion(calls.append)
                tx.after_commit(lambda: calls.append("not_reached"))
        self.assertIs(caught.exception, failure)
        self.assertEqual(calls, [True, True])
        self.assertEqual(owner.notification_failure_count, 0)
        self.assertEqual(owner.completion_failure_count, 0)

    def test_completion_registrations_are_independent_of_commit_deduplication(self):
        owner = self.fixture()
        calls = []

        def notification(*args):
            calls.append(args)

        with owner.transaction() as outer:
            outer.after_commit(notification, deduplicate=True)
            outer.after_completion(notification)
            with owner.transaction() as inner:
                inner.after_commit(notification, deduplicate=True)
                inner.after_completion(notification)
                inner.after_completion(notification)
            with self.assertRaises(ValueError), owner.transaction() as inner:
                inner.after_completion(notification)
                inner.after_commit(notification)
                raise ValueError("rollback")
            outer.after_commit(notification)
        self.assertEqual(calls, [(True,), (True,), (True,), (False,), (), ()])

    def test_parent_deduplicated_commit_hook_survives_child_rollback(self):
        owner = self.fixture()
        calls = []

        def wake():
            calls.append("wake")

        with owner.transaction() as outer:
            with self.assertRaises(ValueError), owner.transaction() as inner:
                inner.after_commit(wake, deduplicate=True)
                outer.after_commit(wake, deduplicate=True)
                inner.after_completion(lambda committed: calls.append(committed))
                raise ValueError("rollback")
            with owner.transaction() as inner:
                inner.after_commit(wake, deduplicate=True)
        self.assertEqual(calls, [False, "wake"])

    def test_completion_registration_rejects_wrong_thread_expired_and_reader_scopes(self):
        owner = self.fixture()
        with owner.transaction() as tx, ThreadPoolExecutor(max_workers=1) as pool:  # noqa: SIM117
            with self.assertRaisesRegex(RuntimeError, "thread/scope"):
                pool.submit(tx.after_completion, lambda committed: None).result(timeout=3)
        with self.assertRaisesRegex(RuntimeError, "thread/scope"):
            tx.after_completion(lambda committed: None)
        with tempfile.TemporaryDirectory(prefix="neh-completion-reader-") as directory:
            reader_owner = SQLiteStorage(str(Path(directory) / "memory.db"))
            try:
                with reader_owner.read_snapshot() as reader:  # noqa: SIM117 - 断言仅覆盖注册操作
                    with self.assertRaisesRegex(RuntimeError, "writer transaction"):
                        reader.after_completion(lambda committed: None)
            finally:
                reader_owner.close()

    def test_python38_completion_syntax(self):
        for name in ("services/memory/sqlite_storage.py", __file__):
            ast.parse(Path(name).read_text(), filename=name, feature_version=(3, 8))


if __name__ == "__main__":
    unittest.main(verbosity=2)
