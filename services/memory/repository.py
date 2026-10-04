from __future__ import annotations

import json
import sqlite3
import uuid
from contextlib import contextmanager
from dataclasses import dataclass
from typing import Dict, Iterable, Iterator, List, Optional, Sequence

from contracts.base import utc_now
from contracts.memory import MemoryPayloadFilter, SemanticMemoryEntry, TaggedLocation
from contracts.spatial_memory import (
    NavigationTrajectoryEvent,
    ObservationEvent,
    SceneSpace,
    SemanticGraphRelation,
    SemanticInstance,
    SemanticSummaryNode,
)
from services.memory.sqlite_storage import SQLiteStorage, SQLiteTransaction
from services.memory.scene_revisions import geometry_value, graph_geometry_value

TAGGED_LOCATION_VECTOR_NAMESPACE = "tagged_location"
SEMANTIC_MEMORY_VECTOR_NAMESPACE = "semantic_memory"
SCENE_SPACE_VECTOR_NAMESPACE = "scene_space"


@dataclass(frozen=True)
class VectorRecord:
    """持久化的向量记录。"""

    namespace: str
    record_id: str
    embedding_model: str
    embedding_dim: int
    source_text: str
    vector_payload: str
    updated_at: str


@dataclass(frozen=True)
class MemoryLibraryRecord:
    """命名记忆库目录记录。"""

    library_name: str
    created_at: str
    updated_at: str


@dataclass(frozen=True)
class _PreparedSceneWrite:
    origin: SceneSpace
    record: SceneSpace
    payload_json: str
    values: tuple
    source_preparation: object
    has_expected_revision: bool
    expected_revision: Optional[str]


class MemoryRepository:
    """记忆库的轻量 SQLite 仓储。"""

    def __init__(self, db_path: str = ":memory:", *, storage: Optional[SQLiteStorage] = None) -> None:
        self._closed = False
        self._index = None
        self._artifact_references = None
        self._owns_storage = storage is None
        self._storage = SQLiteStorage(db_path) if storage is None else storage
        self._db_path = self._storage.db_path
        try:
            self._init_schema()
        except BaseException:
            self.close()
            raise

    def attach_index(self, index) -> None:
        """Attach the service coordinator; standalone repositories stay unchanged."""
        if index.storage is not self._storage:
            raise ValueError("Index and repository must share a storage owner")
        self._index = index

    def prepare_scene_identities(self, library_name, *, snapshot=None, scene=None):
        from services.memory.scene_identity import SceneIdentityPlan
        self._check_open()
        return SceneIdentityPlan(self, library_name, snapshot=snapshot, scene=scene)

    def attach_artifact_references(self, adapter) -> None:
        """可选引用桥与仓储共享事务拥有方，不改变独立仓储的默认行为。"""
        if adapter.storage is not self._storage:
            raise ValueError("Artifact references and repository must share a storage owner")
        if self._artifact_references is not None and self._artifact_references is not adapter:
            raise ValueError("Artifact reference adapter is already attached")
        self._artifact_references = adapter

    @contextmanager
    def _source_mutation(self, kind, record, *, payload_json=None, origin=None, source_preparation=None):
        self._check_open()
        context = self._index.mutation(kind, record, origin=origin, prepared=source_preparation) if self._index is not None else self._transaction()
        if self._artifact_references is not None:
            context = self._artifact_references.mutation(kind, record, context, payload_json=payload_json)
        with context as transaction:
            yield transaction

    @contextmanager
    def _source_deletion(self, kind, ids, expected_tokens=None):
        self._check_open()
        context = self._index.deletion(kind, ids, expected_tokens=expected_tokens) if self._index is not None else self._transaction()
        if self._artifact_references is not None:
            context = self._artifact_references.deletion(kind, ids, context)
        with context as transaction:
            yield transaction

    @property
    def db_path(self) -> str:
        return self._db_path

    @property
    def storage(self) -> SQLiteStorage:
        """Shared transaction owner; borrowed owners outlive this wrapper."""
        return self._storage

    def close(self) -> None:
        if self._closed:
            return
        if self._owns_storage:
            self._storage.close()
        self._closed = True

    def _check_open(self) -> None:
        if self._closed:
            raise RuntimeError("Memory repository is closed")

    @contextmanager
    def _transaction(self) -> Iterator[SQLiteTransaction]:
        self._check_open()
        with self._storage.transaction() as transaction:
            self._check_open()
            yield transaction

    @contextmanager
    def _read_snapshot(self) -> Iterator[SQLiteTransaction]:
        self._check_open()
        with self._storage.read_snapshot() as transaction:
            self._check_open()
            yield transaction

    def _load_library_records(self, table_name, model_type, order_by, library_name):
        """先按库索引筛选，再解析记录；表名和排序仅由内部固定调用提供。"""

        index_columns = {"tagged_locations": "location_id", "semantic_memories": "memory_id",
                         "semantic_instances": "instance_id", "observation_events": "event_id", "scene_spaces": "scene_space_id"}
        column = index_columns.get(table_name)
        extra = f", {column} AS index_record_id" if column and self._index is not None else ""
        if table_name in ("semantic_instances", "scene_spaces"):
            extra += ",graph_source_revision"
        query = f"SELECT payload_json, map_name, library_name{extra} FROM {table_name}"
        params = ()
        if library_name is not None:
            query += " WHERE library_name = ?"
            params = (str(library_name).strip(),)
        query += f" ORDER BY {order_by}"
        with self._read_snapshot() as transaction:
            rows = transaction.execute(query, params).fetchall()
            tokens = [self._capture_loaded_token(transaction, model_type, row) for row in rows]
        return [self._tag_loaded(self._load_record(model_type, row), token) for row, token in zip(rows, tokens)]

    def load_tagged_locations(self, *, library_name: Optional[str] = None) -> List[TaggedLocation]:
        return self._load_library_records(
            "tagged_locations", TaggedLocation, "created_at ASC, location_id ASC", library_name
        )

    def load_semantic_memories(self, *, library_name: Optional[str] = None) -> List[SemanticMemoryEntry]:
        return self._load_library_records(
            "semantic_memories", SemanticMemoryEntry, "created_at ASC, memory_id ASC", library_name
        )

    def load_semantic_instances(self, *, library_name: Optional[str] = None) -> List[SemanticInstance]:
        return self._load_library_records(
            "semantic_instances", SemanticInstance, "updated_at ASC, instance_id ASC", library_name
        )

    def load_observation_events(self, *, library_name: Optional[str] = None) -> List[ObservationEvent]:
        return self._load_library_records(
            "observation_events", ObservationEvent, "created_at ASC, event_id ASC", library_name
        )

    def load_scene_spaces(self, *, library_name: Optional[str] = None) -> List[SceneSpace]:
        return self._load_library_records(
            "scene_spaces", SceneSpace, "updated_at ASC, scene_space_id ASC", library_name
        )

    def load_navigation_trajectory_events(self, *, library_name: Optional[str] = None) -> List[NavigationTrajectoryEvent]:
        return self._load_library_records(
            "navigation_trajectory_events", NavigationTrajectoryEvent, "created_at ASC, trajectory_event_id ASC", library_name
        )

    def load_semantic_graph_relations(self, *, library_name: Optional[str] = None) -> List[SemanticGraphRelation]:
        return self._load_library_records(
            "semantic_graph_relations", SemanticGraphRelation, "updated_at ASC, relation_id ASC", library_name
        )

    def load_semantic_summary_nodes(self, *, library_name: Optional[str] = None) -> List[SemanticSummaryNode]:
        return self._load_library_records(
            "semantic_summary_nodes", SemanticSummaryNode, "updated_at ASC, summary_id ASC", library_name
        )

    def load_scene_space(self, scene_space_id: str) -> Optional[SceneSpace]:
        resolved_id = str(scene_space_id or "").strip()
        if not resolved_id:
            return None
        with self._read_snapshot() as transaction:
            row = transaction.execute(
                """
                SELECT payload_json, map_name, library_name, graph_source_revision, scene_space_id AS index_record_id
                FROM scene_spaces
                WHERE scene_space_id = ?
                """,
                (resolved_id,),
            ).fetchone()
            token = self._capture_loaded_token(transaction, SceneSpace, row) if row is not None else None
        if row is None:
            return None
        return self._tag_loaded(self._load_record(SceneSpace, row), token)

    def list_scene_spaces(
        self,
        *,
        scene_space_ids: Optional[Sequence[str]] = None,
        library_name: Optional[str] = None,
        map_name: Optional[str] = None,
        localization_session_id: Optional[str] = None,
        review_statuses: Optional[Sequence[object]] = None,
        limit: Optional[int] = None,
    ) -> List[SceneSpace]:
        where_clauses: List[str] = []
        params: List[object] = []
        if library_name is not None:
            where_clauses.append("library_name = ?")
            params.append(str(library_name).strip())
        resolved_ids = [str(item).strip() for item in scene_space_ids or [] if str(item).strip()]
        if resolved_ids:
            placeholders = ", ".join("?" for _ in resolved_ids)
            where_clauses.append(f"scene_space_id IN ({placeholders})")
            params.extend(resolved_ids)
        if map_name:
            where_clauses.append("map_name = ?")
            params.append(map_name)
        if localization_session_id:
            where_clauses.append("localization_session_id = ?")
            params.append(localization_session_id)
        resolved_statuses = self._enum_values(review_statuses or [])
        if resolved_statuses:
            placeholders = ", ".join("?" for _ in resolved_statuses)
            where_clauses.append(f"review_status IN ({placeholders})")
            params.extend(resolved_statuses)
        query = "SELECT payload_json, map_name, library_name, graph_source_revision, scene_space_id AS index_record_id FROM scene_spaces"
        if where_clauses:
            query = f"{query} WHERE {' AND '.join(where_clauses)}"
        query = f"{query} ORDER BY updated_at DESC, scene_space_id ASC"
        if limit is not None and limit > 0:
            query = f"{query} LIMIT ?"
            params.append(int(limit))
        with self._read_snapshot() as transaction:
            rows = transaction.execute(query, tuple(params)).fetchall()
            tokens = [self._capture_loaded_token(transaction, SceneSpace, row) for row in rows]
        return [self._tag_loaded(self._load_record(SceneSpace, row), token) for row, token in zip(rows, tokens)]

    def query_scene_spaces(
        self,
        payload_filter: Optional[MemoryPayloadFilter] = None,
        *,
        library_name: Optional[str] = None,
        limit: Optional[int] = None,
    ) -> List[SceneSpace]:
        if payload_filter is None:
            return self.list_scene_spaces(library_name=library_name, limit=limit)
        if payload_filter.record_kinds:
            record_kinds = self._enum_values(payload_filter.record_kinds)
            if SCENE_SPACE_VECTOR_NAMESPACE not in record_kinds:
                return []
        spaces = self.list_scene_spaces(
            library_name=library_name,
            scene_space_ids=payload_filter.scene_space_ids,
            map_name=payload_filter.map_name,
            localization_session_id=payload_filter.localization_session_id,
            review_statuses=payload_filter.review_statuses,
        )
        matched = [space for space in spaces if self._scene_space_matches_filter(space, payload_filter)]
        if limit is not None and limit > 0:
            return matched[: int(limit)]
        return matched

    def load_vector_records(self, namespace: Optional[str] = None) -> Dict[str, VectorRecord]:
        if namespace is None:
            query = """
                SELECT namespace, record_id, embedding_model, embedding_dim, source_text, vector_payload, updated_at
                FROM memory_vectors
            """
            params: Iterable[object] = ()
        else:
            query = """
                SELECT namespace, record_id, embedding_model, embedding_dim, source_text, vector_payload, updated_at
                FROM memory_vectors
                WHERE namespace = ?
            """
            params = (namespace,)
        with self._read_snapshot() as transaction:
            rows = transaction.execute(query, tuple(params)).fetchall()
        return {
            str(row["record_id"]): VectorRecord(
                namespace=str(row["namespace"]),
                record_id=str(row["record_id"]),
                embedding_model=str(row["embedding_model"]),
                embedding_dim=int(row["embedding_dim"]),
                source_text=str(row["source_text"]),
                vector_payload=str(row["vector_payload"]),
                updated_at=str(row["updated_at"]),
            )
            for row in rows
        }

    def load_memory_libraries(self) -> List[MemoryLibraryRecord]:
        with self._read_snapshot() as transaction:
            rows = transaction.execute(
                """
                SELECT library_name, created_at, updated_at
                FROM memory_libraries
                ORDER BY updated_at DESC, library_name ASC
                """
            ).fetchall()
        return [
            MemoryLibraryRecord(
                library_name=str(row["library_name"]),
                created_at=str(row["created_at"]),
                updated_at=str(row["updated_at"]),
            )
            for row in rows
        ]

    def upsert_memory_library(self, library_name: str) -> None:
        resolved_name = str(library_name or "").strip()
        if not resolved_name:
            return
        timestamp = utc_now().isoformat()
        with self._transaction() as transaction:
            transaction.execute(
                """
                INSERT INTO memory_libraries (
                    library_name,
                    created_at,
                    updated_at
                )
                VALUES (?, ?, ?)
                ON CONFLICT(library_name) DO UPDATE SET
                    updated_at = excluded.updated_at
                """,
                (
                    resolved_name,
                    timestamp,
                    timestamp,
                ),
            )

    def delete_memory_library(self, library_name: str) -> bool:
        resolved_name = str(library_name or "").strip()
        if not resolved_name:
            return False
        with self._transaction() as transaction:
            cursor = transaction.execute(
                "DELETE FROM memory_libraries WHERE library_name = ?",
                (resolved_name,),
            )
        return int(cursor.rowcount or 0) > 0

    def upsert_tagged_location(self, location: TaggedLocation) -> None:
        origin = location
        expected = getattr(origin, "_memory_business_token", None)
        payload_json = origin.model_dump_json()
        # SQL 标量、索引输入、制品引用和回调共用同一个已序列化快照。
        location = type(origin).model_validate_json(payload_json)
        if expected is not None:
            object.__setattr__(location, "_memory_business_token", expected)
        created_at = location.timestamp.isoformat()
        library_name = self._resolve_record_library_name(location)
        with self._source_mutation("location", location, payload_json=payload_json, origin=origin) as transaction:
            transaction.execute(
                """
                INSERT INTO tagged_locations (
                    location_id,
                    normalized_name,
                    map_name,
                    payload_json,
                    created_at,
                    updated_at,
                    library_name
                )
                VALUES (?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(location_id) DO UPDATE SET
                    normalized_name = excluded.normalized_name,
                    map_name = excluded.map_name,
                    payload_json = excluded.payload_json,
                    updated_at = excluded.updated_at,
                    library_name = excluded.library_name
                """,
                (
                    location.location_id,
                    location.normalized_name,
                    location.map_name,
                    payload_json,
                    created_at,
                    created_at,
                    library_name,
                ),
            )

    def upsert_semantic_memory(self, entry: SemanticMemoryEntry) -> None:
        origin = entry
        expected = getattr(origin, "_memory_business_token", None)
        payload_json = origin.model_dump_json()
        # SQL 标量、索引输入、制品引用和回调共用同一个已序列化快照。
        entry = type(origin).model_validate_json(payload_json)
        if expected is not None:
            object.__setattr__(entry, "_memory_business_token", expected)
        created_at = entry.timestamp.isoformat()
        library_name = self._resolve_record_library_name(entry)
        with self._source_mutation("memory", entry, payload_json=payload_json, origin=origin) as transaction:
            transaction.execute(
                """
                INSERT INTO semantic_memories (
                    memory_id,
                    kind,
                    title,
                    summary,
                    map_name,
                    payload_json,
                    created_at,
                    updated_at,
                    library_name
                )
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(memory_id) DO UPDATE SET
                    kind = excluded.kind,
                    title = excluded.title,
                    summary = excluded.summary,
                    map_name = excluded.map_name,
                    payload_json = excluded.payload_json,
                    updated_at = excluded.updated_at,
                    library_name = excluded.library_name
                """,
                (
                    entry.memory_id,
                    entry.kind.value,
                    entry.title,
                    entry.summary,
                    entry.map_name,
                    payload_json,
                    created_at,
                    created_at,
                    library_name,
                ),
            )

    def upsert_semantic_instance(self, entry: SemanticInstance) -> None:
        origin = entry
        expected = getattr(origin, "_memory_business_token", None)
        payload_json = origin.model_dump_json()
        # SQL 标量、索引输入、制品引用和回调共用同一个已序列化快照。
        entry = type(origin).model_validate_json(payload_json)
        if expected is not None:
            object.__setattr__(entry, "_memory_business_token", expected)
        source_revision = uuid.uuid4().hex
        object.__setattr__(entry, "_scene_graph_source_revision", source_revision)
        created_at = entry.first_seen_ts.isoformat()
        updated_at = entry.last_seen_ts.isoformat()
        library_name = self._resolve_record_library_name(entry)
        with self._source_mutation("instance", entry, payload_json=payload_json, origin=origin) as transaction:
            transaction.execute(
                """
                INSERT INTO semantic_instances (
                    instance_id,
                    label,
                    lifecycle_state,
                    map_name,
                    localization_session_id,
                    payload_json,
                    created_at,
                    updated_at,
                    library_name,
                    graph_source_revision
                )
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(instance_id) DO UPDATE SET
                    label = excluded.label,
                    lifecycle_state = excluded.lifecycle_state,
                    map_name = excluded.map_name,
                    localization_session_id = excluded.localization_session_id,
                    payload_json = excluded.payload_json,
                    updated_at = excluded.updated_at,
                    library_name = excluded.library_name,
                    graph_source_revision = excluded.graph_source_revision
                """,
                (
                    entry.instance_id,
                    entry.label,
                    entry.lifecycle_state.value,
                    entry.map_name,
                    entry.localization_session_id,
                    payload_json,
                    created_at,
                    updated_at,
                    library_name,
                    source_revision,
                ),
            )

    def upsert_observation_event(self, entry: ObservationEvent) -> None:
        origin = entry
        expected = getattr(origin, "_memory_business_token", None)
        payload_json = origin.model_dump_json()
        # SQL 标量、索引输入、制品引用和回调共用同一个已序列化快照。
        entry = type(origin).model_validate_json(payload_json)
        if expected is not None:
            object.__setattr__(entry, "_memory_business_token", expected)
        created_at = entry.timestamp.isoformat()
        library_name = self._resolve_record_library_name(entry)
        with self._source_mutation("event", entry, payload_json=payload_json, origin=origin) as transaction:
            transaction.execute(
                """
                INSERT INTO observation_events (
                    event_id,
                    anchor_id,
                    map_name,
                    localization_session_id,
                    payload_json,
                    created_at,
                    updated_at,
                    library_name
                )
                VALUES (?, ?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(event_id) DO UPDATE SET
                    anchor_id = excluded.anchor_id,
                    map_name = excluded.map_name,
                    localization_session_id = excluded.localization_session_id,
                    payload_json = excluded.payload_json,
                    updated_at = excluded.updated_at,
                    library_name = excluded.library_name
                """,
                (
                    entry.event_id,
                    entry.anchor_id,
                    entry.map_name,
                    entry.localization_session_id,
                    payload_json,
                    created_at,
                    created_at,
                    library_name,
                ),
            )

    def scene_graph_generations(self, library_name, *, transaction=None):
        if transaction is None:
            with self._read_snapshot() as snapshot:
                return self.scene_graph_generations(library_name, transaction=snapshot)
        row = transaction.execute("SELECT source_generation,geometry_generation FROM memory_scene_graph_inputs WHERE library_name=?", (library_name,)).fetchone()
        return tuple(row) if row is not None else (0, 0)

    def capture_scene_decision_entries(self, library_name, *, visible_ids=None, include_spaces=False, visible_space_ids=None):
        with self._read_snapshot() as transaction:
            row = transaction.execute("SELECT memory_generation FROM memory_scene_graph_inputs WHERE library_name=?", (library_name,)).fetchone()
            generation = row[0] if row is not None else 0
            identity = transaction.execute("""SELECT l.incarnation FROM memory_index_aliases a
                JOIN memory_vectorization_libraries l ON l.name=a.internal_key
                WHERE a.alias_kind='lib' AND a.namespace='' AND a.external_key=?""", (library_name,)).fetchone()
            incarnation = identity[0] if identity is not None else None
            where, params = "library_name=?", [library_name]
            if visible_ids is not None:
                where += " AND memory_id IN (SELECT value FROM json_each(?))"
                params.append(json.dumps(tuple(visible_ids)))
            rows = transaction.execute(f"""SELECT payload_json,map_name,library_name,memory_id AS index_record_id
                FROM semantic_memories WHERE {where}
                ORDER BY CAST(json_extract(payload_json,'$.timestamp') AS TEXT) COLLATE NEH_SCENE_TIMESTAMP DESC,memory_id
                LIMIT 2000""", tuple(params)).fetchall()
            tokens = [self._capture_loaded_token(transaction, SemanticMemoryEntry, row) for row in rows]
            scene_rows, scene_tokens = [], []
            if include_spaces:
                scene_where = "library_name=? AND json_extract(payload_json,'$.metadata.scene_mapping_managed')=1"
                scene_params = [library_name]
                if visible_space_ids is not None:
                    scene_where += " AND scene_space_id IN (SELECT value FROM json_each(?))"
                    scene_params.append(json.dumps(tuple(visible_space_ids)))
                scene_rows = transaction.execute(f"SELECT payload_json,map_name,library_name,graph_source_revision,scene_space_id AS index_record_id FROM scene_spaces WHERE {scene_where} ORDER BY scene_space_id", tuple(scene_params)).fetchall()
                scene_tokens = [self._capture_loaded_token(transaction, SceneSpace, row) for row in scene_rows]
        records = []
        for row, token in zip(rows, tokens):
            entry = self._tag_loaded(self._load_record(SemanticMemoryEntry,row),token)
            object.__setattr__(entry, "_scene_decision_source_revision", ("memory_source", incarnation, generation))
            records.append(entry)
        if include_spaces:
            scenes = tuple(self._tag_loaded(self._load_record(SceneSpace,row),token) for row,token in zip(scene_rows,scene_tokens))
            return scenes, tuple(records)
        return tuple(records)

    def capture_scene_graph_sources(self, library_name, *, binding=None, visible_ids=None, live_records=None):
        """Capture bounded source rows/counters together; decode outside reader ownership."""
        specs = (("scene_spaces", SceneSpace, "scene_space_id", "updated_at DESC,scene_space_id", 513),
                 ("semantic_memories", SemanticMemoryEntry, "memory_id", "CAST(json_extract(payload_json,'$.timestamp') AS TEXT) COLLATE NEH_SCENE_TIMESTAMP DESC,memory_id", 2000),
                 ("semantic_instances", SemanticInstance, "instance_id", "CAST(json_extract(payload_json,'$.last_seen_ts') AS TEXT) COLLATE NEH_SCENE_TIMESTAMP DESC,instance_id DESC", 1000))
        captured = []
        with self._read_snapshot() as transaction:
            generations = self.scene_graph_generations(library_name, transaction=transaction)
            valid_live_ids, extra_live = [], 0
            for point_id, record in (live_records or {}).items():
                version = transaction.execute("SELECT graph_source_revision,library_name FROM semantic_instances WHERE instance_id=?", (point_id,)).fetchone()
                if version is None:
                    valid_live_ids.append(point_id)
                    extra_live += 1
                    continue
                if version["library_name"] != library_name:
                    continue
                current_revision = version["graph_source_revision"] or None
                expected_revision = getattr(record, "_scene_graph_source_revision", None) or None
                if current_revision is not None or expected_revision is not None:
                    valid = current_revision == expected_revision
                else:
                    state = self._index._state(transaction, 'instance', point_id) if self._index is not None else None
                    raw = self._index._raw(transaction, 'instance', point_id) if self._index is not None else None
                    token = self._index.business_token(state) if state is not None else self._index.raw_token(raw) if self._index is not None else None
                    valid = token == getattr(record, "_memory_business_token", None)
                if valid:
                    valid_live_ids.append(point_id)
            for table, model, column, order, limit in specs:
                where = "library_name=?"
                params = [library_name]
                if visible_ids is not None:
                    ids = tuple(visible_ids.get(table, ()))
                    where += f" AND {column} IN (SELECT value FROM json_each(?))"
                    params.append(json.dumps(ids))
                if table == "scene_spaces":
                    where += " AND json_extract(payload_json,'$.metadata.scene_mapping_managed')=1"
                total = transaction.execute(f"SELECT COUNT(*) FROM {table} WHERE {where}", tuple(params)).fetchone()[0]
                selected_total = total
                if table == "scene_spaces" and binding is not None:
                    where += " AND review_status NOT IN ('stale','rejected')"
                    for field in ("map_name", "source_name", "source_epoch", "frame_id", "update_sequence"):
                        where += f" AND CAST(json_extract(payload_json,'$.metadata.map_binding.{field}') AS TEXT)=?"
                        params.append(str(binding[field]))
                    selected_total = transaction.execute(f"SELECT COUNT(*) FROM {table} WHERE {where}", tuple(params)).fetchone()[0]
                if table == "semantic_instances" and valid_live_ids:
                    # Overlay before LIMIT: demoted live records must not crowd
                    # newer durable candidates out of the bounded source page.
                    where += " AND instance_id NOT IN (SELECT value FROM json_each(?))"
                    params.append(json.dumps(valid_live_ids))
                extra = ",graph_source_revision" if table in ("semantic_instances", "scene_spaces") else ""
                rows = transaction.execute(f"SELECT payload_json,map_name,library_name,{column} AS index_record_id{extra} FROM {table} WHERE {where} ORDER BY {order} LIMIT ?", (*params,limit)).fetchall()
                tokens = [self._capture_loaded_token(transaction,model,row) for row in rows]
                captured.append((model,rows,tokens,total,selected_total))
        result = {"generations": generations, "valid_live_ids": valid_live_ids, "extra_live": extra_live}
        for name, (model, rows, tokens, total, selected_total) in zip(("spaces","entries","instances"),captured):
            result[name] = [self._tag_loaded(self._load_record(model,row),token) for row,token in zip(rows,tokens)]
            result[name+"_total"] = total
            result[name+"_selected_total"] = selected_total
        return result

    def upsert_scene_space(self, entry: SceneSpace) -> SceneSpace:
        prepared = self._prepare_scene_space_write(entry)
        return self._commit_prepared_scene_space(prepared).model_copy(deep=True)

    def _prepare_scene_space_write(self, entry: SceneSpace) -> _PreparedSceneWrite:
        """Serialize/decode/project before entering a batch writer transaction."""
        self._check_open()
        origin = entry
        expected = getattr(origin, "_memory_business_token", None)
        payload_json = origin.model_dump_json()
        # SQL 标量、索引输入、制品引用和回调共用同一个已序列化快照。
        entry = type(origin).model_validate_json(payload_json)
        if expected is not None:
            object.__setattr__(entry, "_memory_business_token", expected)
        with self._read_snapshot() as snapshot:
            row = snapshot.execute("SELECT payload_json,map_name,library_name,graph_geometry_revision FROM scene_spaces WHERE scene_space_id=?", (entry.scene_space_id,)).fetchone()
        previous = self._load_record(SceneSpace, row) if row is not None else None
        previous_revision = previous.metadata.get("geometry_revision") if previous is not None else None
        revision = (previous_revision if previous_revision and geometry_value(previous) == geometry_value(entry) else uuid.uuid4().hex)
        if entry.metadata.get("scene_mapping_managed"):
            entry.metadata = {**entry.metadata, "geometry_revision": revision}
        graph_value = graph_geometry_value(entry)
        graph_revision = ((row["graph_geometry_revision"] if row is not None and row["graph_geometry_revision"]
            and graph_geometry_value(previous) == graph_value else uuid.uuid4().hex) if graph_value is not None else "")
        payload_json = entry.model_dump_json()
        source_revision = uuid.uuid4().hex
        object.__setattr__(entry, "_scene_graph_source_revision", source_revision)
        created_at = entry.timestamp.isoformat()
        semantic_updated_at = entry.semantic_updated_at.isoformat() if entry.semantic_updated_at else None
        updated_at = semantic_updated_at or created_at
        library_name = self._resolve_record_library_name(entry)
        source_preparation = None
        if self._index is not None:
            source = self._index.prepare("scene", entry)
            dependents = self._index._dependents("scene", source.point_id, entry)
            source_preparation = (source, dependents)
        return _PreparedSceneWrite(origin, entry, payload_json,
            (entry.scene_space_id, entry.map_name, entry.localization_session_id, entry.frame_id,
             entry.primary_label, entry.review_status.value, payload_json, created_at, updated_at,
             semantic_updated_at, library_name, graph_revision, source_revision),
            source_preparation, hasattr(origin, "_scene_expected_source_revision"),
            getattr(origin, "_scene_expected_source_revision", None))

    def _commit_prepared_scene_space(self, prepared: _PreparedSceneWrite) -> SceneSpace:
        """Commit an owned scene plan; outer batch rollback discards all callbacks."""
        entry = prepared.record
        with self._source_mutation("scene", entry, payload_json=prepared.payload_json,
                origin=prepared.origin, source_preparation=prepared.source_preparation) as transaction:
            if prepared.has_expected_revision:
                current = transaction.execute("SELECT graph_source_revision FROM scene_spaces WHERE scene_space_id=?", (entry.scene_space_id,)).fetchone()
                current_revision = current[0] if current is not None else None
                if current_revision != prepared.expected_revision:
                    from services.memory.vectorization_outbox import OutboxConflict
                    raise OutboxConflict("Scene source changed during geometry preparation")
            transaction.execute("""
                INSERT INTO scene_spaces (
                    scene_space_id,
                    map_name,
                    localization_session_id,
                    frame_id,
                    primary_label,
                    review_status,
                    payload_json,
                    created_at,
                    updated_at,
                    semantic_updated_at,
                    library_name,
                    graph_geometry_revision,
                    graph_source_revision
                )
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(scene_space_id) DO UPDATE SET
                    map_name = excluded.map_name,
                    localization_session_id = excluded.localization_session_id,
                    frame_id = excluded.frame_id,
                    primary_label = excluded.primary_label,
                    review_status = excluded.review_status,
                    payload_json = excluded.payload_json,
                    updated_at = excluded.updated_at,
                    semantic_updated_at = excluded.semantic_updated_at,
                    library_name = excluded.library_name,
                    graph_geometry_revision = excluded.graph_geometry_revision,
                    graph_source_revision = excluded.graph_source_revision
                """, prepared.values)
        return entry

    def upsert_navigation_trajectory_event(self, entry: NavigationTrajectoryEvent) -> NavigationTrajectoryEvent:
        entry = NavigationTrajectoryEvent.model_validate_json(entry.model_dump_json())
        payload_json = entry.model_dump_json()
        created_at = entry.timestamp.isoformat()
        library_name = self._resolve_record_library_name(entry)
        publication_revision = uuid.uuid4().hex
        with self._transaction() as transaction:
            transaction.execute(
                """
                INSERT INTO navigation_trajectory_events (
                    trajectory_event_id,
                    event_kind,
                    map_name,
                    localization_session_id,
                    source_record_id,
                    payload_json,
                    created_at,
                    updated_at,
                    library_name,
                    publication_revision
                )
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(trajectory_event_id) DO UPDATE SET
                    event_kind = excluded.event_kind,
                    map_name = excluded.map_name,
                    localization_session_id = excluded.localization_session_id,
                    source_record_id = excluded.source_record_id,
                    payload_json = excluded.payload_json,
                    updated_at = excluded.updated_at,
                    library_name = excluded.library_name,
                    publication_revision = excluded.publication_revision
                """,
                (
                    entry.trajectory_event_id,
                    entry.event_kind.value,
                    entry.map_name,
                    entry.localization_session_id,
                    entry.source_record_id,
                    payload_json,
                    created_at,
                    created_at,
                    library_name,
                    publication_revision,
                ),
            )
        object.__setattr__(entry, "_memory_publication_revision", publication_revision)
        return entry

    def auxiliary_record_current(self, kind, record_id, publication_revision, library_name):
        table, column = {
            "trajectory": ("navigation_trajectory_events", "trajectory_event_id"),
            "relation": ("semantic_graph_relations", "relation_id"),
            "summary": ("semantic_summary_nodes", "summary_id"),
        }[kind]
        with self._read_snapshot() as transaction:
            return transaction.execute(
                f"SELECT 1 FROM {table} WHERE {column}=? AND publication_revision=? AND library_name=?",
                (record_id, publication_revision, library_name),
            ).fetchone() is not None

    def _natural_record_identity(self, transaction, entry, identity_scope, *, kind, table, column, fields):
        """Resolve one library-owned natural key; existing duplicates stay intact."""
        library = self._resolve_record_library_name(entry)
        key = json.dumps([value for _, value in fields], ensure_ascii=False, separators=(",", ":"))
        clauses = " AND ".join(("COALESCE(r.%s, '') = ?" if name in ("label", "scene_space_id", "semantic_region_id") else "r.%s = ?") % name for name, _ in fields)
        values = tuple(value for _, value in fields)
        row = transaction.execute(
            f"""SELECT r.{column} FROM memory_record_natural_ids n
                JOIN {table} r ON r.{column}=n.record_id
                WHERE n.library_incarnation=? AND n.record_kind=? AND n.natural_key=?
                AND r.library_name=? AND {clauses}""",
            (identity_scope, kind, key, library, *values),
        ).fetchone()
        if row is None:
            # Adopt only matching candidates, without rewriting existing identities.
            row = transaction.execute(
                f"SELECT r.{column} FROM {table} r WHERE r.library_name=? AND {clauses} ORDER BY r.created_at,r.{column} LIMIT 1",
                (library, *values),
            ).fetchone()
        record_id = row[0] if row else ("sgr_" if kind == "relation" else "ssn_") + uuid.uuid4().hex
        transaction.execute(
            """INSERT INTO memory_record_natural_ids VALUES (?,?,?,?)
               ON CONFLICT(library_incarnation,record_kind,natural_key) DO UPDATE SET record_id=excluded.record_id""",
            (identity_scope, kind, key, record_id),
        )
        return entry.model_copy(update={column: record_id}, deep=True)

    @staticmethod
    def _summary_identity_fields(entry):
        kind = entry.summary_kind.value
        fields = [("summary_kind", kind), ("label", entry.label or "")]
        if kind != "target_category_cluster":
            fields.append(("scene_space_id", entry.scene_space_id or ""))
        if kind == "region":
            fields.append(("semantic_region_id", entry.semantic_region_id or ""))
        return tuple(fields)

    def upsert_semantic_graph_relation(self, entry: SemanticGraphRelation, *, natural_identity: Optional[str] = None) -> SemanticGraphRelation:
        entry = SemanticGraphRelation.model_validate_json(entry.model_dump_json())
        publication_revision = uuid.uuid4().hex
        with self._transaction() as transaction:
            if natural_identity is not None:
                entry = self._natural_record_identity(transaction, entry, natural_identity,
                    kind="relation", table="semantic_graph_relations", column="relation_id", fields=(("relation_kind", entry.relation_kind.value), ("source_node_kind", entry.source_node_kind),
                    ("source_node_id", entry.source_node_id), ("target_node_kind", entry.target_node_kind),
                    ("target_node_id", entry.target_node_id)))
            # Explicit-ID updates retain valid aliases. The mapped-row join
            # checks changed ownership/key only when that identity is requested.
            payload_json = entry.model_dump_json()
            created_at = entry.timestamp.isoformat()
            library_name = self._resolve_record_library_name(entry)
            transaction.execute(
                """
                INSERT INTO semantic_graph_relations (
                    relation_id,
                    relation_kind,
                    source_node_kind,
                    source_node_id,
                    target_node_kind,
                    target_node_id,
                    map_name,
                    localization_session_id,
                    payload_json,
                    created_at,
                    updated_at,
                    library_name,
                    publication_revision
                )
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(relation_id) DO UPDATE SET
                    relation_kind = excluded.relation_kind,
                    source_node_kind = excluded.source_node_kind,
                    source_node_id = excluded.source_node_id,
                    target_node_kind = excluded.target_node_kind,
                    target_node_id = excluded.target_node_id,
                    map_name = excluded.map_name,
                    localization_session_id = excluded.localization_session_id,
                    payload_json = excluded.payload_json,
                    updated_at = excluded.updated_at,
                    library_name = excluded.library_name,
                    publication_revision = excluded.publication_revision
                """,
                (
                    entry.relation_id,
                    entry.relation_kind.value,
                    entry.source_node_kind,
                    entry.source_node_id,
                    entry.target_node_kind,
                    entry.target_node_id,
                    entry.map_name,
                    entry.localization_session_id,
                    payload_json,
                    created_at,
                    created_at,
                    library_name,
                    publication_revision,
                ),
            )
        object.__setattr__(entry, "_memory_publication_revision", publication_revision)
        return entry

    def upsert_semantic_summary_node(self, entry: SemanticSummaryNode, *, natural_identity: Optional[str] = None) -> SemanticSummaryNode:
        entry = SemanticSummaryNode.model_validate_json(entry.model_dump_json())
        publication_revision = uuid.uuid4().hex
        with self._transaction() as transaction:
            if natural_identity is not None:
                entry = self._natural_record_identity(transaction, entry, natural_identity,
                    kind="summary", table="semantic_summary_nodes", column="summary_id", fields=self._summary_identity_fields(entry))
            # Explicit-ID updates retain valid aliases. The mapped-row join
            # checks changed ownership/key only when that identity is requested.
            payload_json = entry.model_dump_json()
            created_at = entry.timestamp.isoformat()
            library_name = self._resolve_record_library_name(entry)
            transaction.execute(
                """
                INSERT INTO semantic_summary_nodes (
                    summary_id,
                    summary_kind,
                    map_name,
                    localization_session_id,
                    scene_space_id,
                    semantic_region_id,
                    label,
                    payload_json,
                    created_at,
                    updated_at,
                    library_name,
                    publication_revision
                )
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(summary_id) DO UPDATE SET
                    summary_kind = excluded.summary_kind,
                    map_name = excluded.map_name,
                    localization_session_id = excluded.localization_session_id,
                    scene_space_id = excluded.scene_space_id,
                    semantic_region_id = excluded.semantic_region_id,
                    label = excluded.label,
                    payload_json = excluded.payload_json,
                    updated_at = excluded.updated_at,
                    library_name = excluded.library_name,
                    publication_revision = excluded.publication_revision
                """,
                (
                    entry.summary_id,
                    entry.summary_kind.value,
                    entry.map_name,
                    entry.localization_session_id,
                    entry.scene_space_id,
                    entry.semantic_region_id,
                    entry.label,
                    payload_json,
                    created_at,
                    created_at,
                    library_name,
                    publication_revision,
                ),
            )
        object.__setattr__(entry, "_memory_publication_revision", publication_revision)
        return entry

    def delete_tagged_locations(self, location_ids: Sequence[str], *, expected_tokens=None) -> int:
        normalized_ids = [str(item).strip() for item in location_ids if str(item).strip()]
        if not normalized_ids:
            return 0
        with self._source_deletion("location", normalized_ids, expected_tokens) as transaction:
            transaction.executemany(
                "DELETE FROM tagged_locations WHERE location_id = ?",
                [(location_id,) for location_id in normalized_ids],
            )
            transaction.executemany(
                "DELETE FROM memory_vectors WHERE record_id = ?",
                [(location_id,) for location_id in normalized_ids],
            )
        return len(normalized_ids)

    def delete_semantic_memories(self, memory_ids: Sequence[str], *, expected_tokens=None) -> int:
        normalized_ids = [str(item).strip() for item in memory_ids if str(item).strip()]
        if not normalized_ids:
            return 0
        with self._source_deletion("memory", normalized_ids, expected_tokens) as transaction:
            transaction.executemany(
                "DELETE FROM semantic_memories WHERE memory_id = ?",
                [(memory_id,) for memory_id in normalized_ids],
            )
            transaction.executemany(
                "DELETE FROM memory_vectors WHERE record_id = ?",
                [(memory_id,) for memory_id in normalized_ids],
            )
        return len(normalized_ids)

    def delete_semantic_instances(self, instance_ids: Sequence[str], *, expected_tokens=None) -> int:
        normalized_ids = [str(item).strip() for item in instance_ids if str(item).strip()]
        if not normalized_ids:
            return 0
        with self._source_deletion("instance", normalized_ids, expected_tokens) as transaction:
            transaction.executemany(
                "DELETE FROM semantic_instances WHERE instance_id = ?",
                [(instance_id,) for instance_id in normalized_ids],
            )
            transaction.executemany(
                "DELETE FROM memory_vectors WHERE record_id = ?",
                [(instance_id,) for instance_id in normalized_ids],
            )
        return len(normalized_ids)

    def delete_observation_events(self, event_ids: Sequence[str], *, expected_tokens=None) -> int:
        normalized_ids = [str(item).strip() for item in event_ids if str(item).strip()]
        if not normalized_ids:
            return 0
        with self._source_deletion("event", normalized_ids, expected_tokens) as transaction:
            transaction.executemany(
                "DELETE FROM observation_events WHERE event_id = ?",
                [(event_id,) for event_id in normalized_ids],
            )
            transaction.executemany(
                "DELETE FROM memory_vectors WHERE record_id = ?",
                [(event_id,) for event_id in normalized_ids],
            )
        return len(normalized_ids)

    def delete_scene_spaces(self, scene_space_ids: Sequence[str], *, expected_tokens=None) -> int:
        normalized_ids = [str(item).strip() for item in scene_space_ids if str(item).strip()]
        if not normalized_ids:
            return 0
        with self._source_deletion("scene", normalized_ids, expected_tokens) as transaction:
            transaction.executemany(
                "DELETE FROM scene_spaces WHERE scene_space_id = ?",
                [(scene_space_id,) for scene_space_id in normalized_ids],
            )
            transaction.executemany(
                "DELETE FROM memory_vectors WHERE record_id = ?",
                [(scene_space_id,) for scene_space_id in normalized_ids],
            )
        return len(normalized_ids)

    def delete_navigation_trajectory_events(self, trajectory_event_ids: Sequence[str]) -> int:
        normalized_ids = [str(item).strip() for item in trajectory_event_ids if str(item).strip()]
        if not normalized_ids:
            return 0
        with self._transaction() as transaction:
            transaction.executemany(
                "DELETE FROM navigation_trajectory_events WHERE trajectory_event_id = ?",
                [(trajectory_event_id,) for trajectory_event_id in normalized_ids],
            )
        return len(normalized_ids)

    def delete_semantic_graph_relations(self, relation_ids: Sequence[str]) -> int:
        normalized_ids = [str(item).strip() for item in relation_ids if str(item).strip()]
        if not normalized_ids:
            return 0
        with self._transaction() as transaction:
            transaction.executemany("DELETE FROM memory_record_natural_ids WHERE record_kind=? AND record_id=?",
                                    [("relation", item) for item in normalized_ids])
            transaction.executemany(
                "DELETE FROM semantic_graph_relations WHERE relation_id = ?",
                [(relation_id,) for relation_id in normalized_ids],
            )
        return len(normalized_ids)

    def delete_semantic_summary_nodes(self, summary_ids: Sequence[str]) -> int:
        normalized_ids = [str(item).strip() for item in summary_ids if str(item).strip()]
        if not normalized_ids:
            return 0
        with self._transaction() as transaction:
            transaction.executemany("DELETE FROM memory_record_natural_ids WHERE record_kind=? AND record_id=?",
                                    [("summary", item) for item in normalized_ids])
            transaction.executemany(
                "DELETE FROM semantic_summary_nodes WHERE summary_id = ?",
                [(summary_id,) for summary_id in normalized_ids],
            )
        return len(normalized_ids)

    def clear_all(self) -> None:
        with self._transaction() as transaction:
            from services.memory.scene_identity import clear_all as clear_scene_identities
            clear_scene_identities(transaction)
            if self._index is not None or self._artifact_references is not None:
                for table, column, method in (
                    ("tagged_locations", "location_id", self.delete_tagged_locations),
                    ("semantic_memories", "memory_id", self.delete_semantic_memories),
                    ("semantic_instances", "instance_id", self.delete_semantic_instances),
                    ("observation_events", "event_id", self.delete_observation_events),
                    ("scene_spaces", "scene_space_id", self.delete_scene_spaces),
                ):
                    method([row[0] for row in transaction.execute(f"SELECT {column} FROM {table}")])
            transaction.execute("DELETE FROM tagged_locations")
            transaction.execute("DELETE FROM semantic_memories")
            transaction.execute("DELETE FROM semantic_instances")
            transaction.execute("DELETE FROM observation_events")
            transaction.execute("DELETE FROM scene_spaces")
            transaction.execute("DELETE FROM navigation_trajectory_events")
            transaction.execute("DELETE FROM semantic_graph_relations")
            transaction.execute("DELETE FROM semantic_summary_nodes")
            transaction.execute("DELETE FROM memory_vectors")
            transaction.execute("DELETE FROM memory_record_natural_ids")

    def upsert_vector_record(
        self,
        *,
        namespace: str,
        record_id: str,
        embedding_model: str,
        embedding_dim: int,
        source_text: str,
        vector_payload: str,
        updated_at: str,
    ) -> None:
        with self._transaction() as transaction:
            transaction.execute(
                """
                INSERT INTO memory_vectors (
                    namespace,
                    record_id,
                    embedding_model,
                    embedding_dim,
                    source_text,
                    vector_payload,
                    updated_at
                )
                VALUES (?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(namespace, record_id) DO UPDATE SET
                    embedding_model = excluded.embedding_model,
                    embedding_dim = excluded.embedding_dim,
                    source_text = excluded.source_text,
                    vector_payload = excluded.vector_payload,
                    updated_at = excluded.updated_at
                """,
                (
                    namespace,
                    record_id,
                    embedding_model,
                    embedding_dim,
                    source_text,
                    vector_payload,
                    updated_at,
                ),
            )

    def _capture_loaded_token(self, transaction, model_type, row):
        if self._index is None or "index_record_id" not in row.keys():
            return None
        from services.memory.durable_index import SOURCES
        kind = next((key for key, value in SOURCES.items() if value[2] is model_type), None)
        if kind is None:
            return None
        state = self._index._state(transaction, kind, row["index_record_id"])
        return self._index.business_token(state) if state else self._index.raw_token(row)

    def _tag_loaded(self, record, token=None):
        if token is not None:
            from services.memory.durable_index import TOKEN
            object.__setattr__(record, TOKEN, token)
        return record

    @staticmethod
    def _load_record(model_type, row):
        payload = json.loads(str(row["payload_json"]))
        if not isinstance(payload, dict):
            raise TypeError("memory payload_json must contain an object")

        payload.pop("map_version_id", None)
        resolved_map_name = str(payload.get("map_name") or row["map_name"] or "").strip()
        if not resolved_map_name:
            library_name = str(row["library_name"] or "").strip()
            metadata = payload.get("metadata")
            if not library_name and isinstance(metadata, dict):
                library_name = str(metadata.get("memory_library_name") or "").strip()
            if library_name and library_name != "legacy_default":
                resolved_map_name = library_name
        payload["map_name"] = resolved_map_name or None
        record = model_type.model_validate(payload)
        if model_type in (SemanticInstance, SceneSpace) and "graph_source_revision" in row.keys():
            object.__setattr__(record, "_scene_graph_source_revision", row["graph_source_revision"])
        return record

    @staticmethod
    def _enum_values(values: Sequence[object]) -> List[str]:
        resolved: List[str] = []
        for item in values:
            value = getattr(item, "value", item)
            normalized = str(value or "").strip()
            if normalized:
                resolved.append(normalized)
        return resolved

    @staticmethod
    def _matches_any(values: Sequence[str], expected_values: Sequence[str]) -> bool:
        if not expected_values:
            return True
        normalized_values = {str(value).strip().lower() for value in values if str(value).strip()}
        for item in expected_values:
            if str(item).strip().lower() in normalized_values:
                return True
        return False

    @staticmethod
    def _scene_space_labels(space: SceneSpace) -> List[str]:
        labels: List[str] = []
        if space.primary_label:
            labels.append(space.primary_label)
        labels.extend(space.human_aliases)
        for candidate in space.candidate_labels:
            labels.append(candidate.label)
            if candidate.normalized_label:
                labels.append(candidate.normalized_label)
        return labels

    @staticmethod
    def _scene_space_visual_labels(space: SceneSpace) -> List[str]:
        labels: List[str] = []
        for sample in space.observation_samples:
            labels.extend(sample.visual_labels)
        return labels

    @staticmethod
    def _scene_space_vision_tags(space: SceneSpace) -> List[str]:
        tags: List[str] = []
        for sample in space.observation_samples:
            tags.extend(sample.vision_tags)
        return tags

    @staticmethod
    def _scene_space_camera_ids(space: SceneSpace) -> List[str]:
        camera_ids: List[str] = []
        for sample in space.observation_samples:
            if sample.camera_id:
                camera_ids.append(sample.camera_id)
        return camera_ids

    @staticmethod
    def _scene_space_reference_poses(space: SceneSpace) -> List[object]:
        poses: List[object] = []
        if space.boundary and space.boundary.centroid:
            poses.append(space.boundary.centroid)
        poses.extend(space.entry_poses)
        poses.extend(space.inspection_poses)
        for sample in space.observation_samples:
            if sample.pose:
                poses.append(sample.pose)
        return poses

    @staticmethod
    def _pose_distance_m(first: object, second: object) -> float:
        first_position = first.position
        second_position = second.position
        dx = float(first_position.x) - float(second_position.x)
        dy = float(first_position.y) - float(second_position.y)
        dz = float(first_position.z) - float(second_position.z)
        return (dx * dx + dy * dy + dz * dz) ** 0.5

    def _scene_space_matches_filter(self, space: SceneSpace, payload_filter: MemoryPayloadFilter) -> bool:
        if payload_filter.scene_space_ids and space.scene_space_id not in payload_filter.scene_space_ids:
            return False
        if payload_filter.map_name and space.map_name != payload_filter.map_name:
            return False
        if payload_filter.localization_session_id and space.localization_session_id != payload_filter.localization_session_id:
            return False
        if payload_filter.review_statuses:
            review_statuses = self._enum_values(payload_filter.review_statuses)
            if space.review_status.value not in review_statuses:
                return False
        if payload_filter.anchor_ids and not self._matches_any(space.linked_anchor_ids, payload_filter.anchor_ids):
            return False
        if payload_filter.topo_node_ids and not self._matches_any(space.linked_topology_node_ids, payload_filter.topo_node_ids):
            return False
        if payload_filter.semantic_labels_any and not self._matches_any(
            self._scene_space_labels(space),
            payload_filter.semantic_labels_any,
        ):
            return False
        if payload_filter.visual_labels_any and not self._matches_any(
            self._scene_space_visual_labels(space),
            payload_filter.visual_labels_any,
        ):
            return False
        if payload_filter.vision_tags_any and not self._matches_any(
            self._scene_space_vision_tags(space),
            payload_filter.vision_tags_any,
        ):
            return False
        if payload_filter.camera_ids and not self._matches_any(
            self._scene_space_camera_ids(space),
            payload_filter.camera_ids,
        ):
            return False
        if payload_filter.created_after and space.timestamp < payload_filter.created_after:
            return False
        if payload_filter.created_before and space.timestamp > payload_filter.created_before:
            return False
        reference_ts = space.semantic_updated_at or space.timestamp
        if payload_filter.last_seen_after and reference_ts < payload_filter.last_seen_after:
            return False
        if payload_filter.last_seen_before and reference_ts > payload_filter.last_seen_before:
            return False
        if payload_filter.max_age_sec is not None:
            age_sec = (utc_now() - reference_ts).total_seconds()
            if age_sec > payload_filter.max_age_sec:
                return False
        if payload_filter.near_pose and payload_filter.max_distance_m is not None:
            reference_poses = self._scene_space_reference_poses(space)
            if not reference_poses:
                return False
            min_distance = min(self._pose_distance_m(payload_filter.near_pose, pose) for pose in reference_poses)
            if min_distance > payload_filter.max_distance_m:
                return False
        return True

    def _ensure_columns(self, table_name: str, columns: Dict[str, str]) -> None:
        with self._transaction() as transaction:
            existing = {str(row["name"]) for row in transaction.execute(f"PRAGMA table_info({table_name})").fetchall()}
            for column_name, column_type in columns.items():
                if column_name not in existing:
                    transaction.execute(f"ALTER TABLE {table_name} ADD COLUMN {column_name} {column_type}")

    # —— speed-up-list-maps-summary ——
    # 8 张记忆表与 library_name 列 / 索引的常量映射；供 _ensure_library_name_columns /
    # _ensure_library_name_indexes / count_records_by_library / backfill_library_names 共用。
    _LIBRARY_NAME_TABLES: Sequence[str] = (
        "tagged_locations",
        "semantic_memories",
        "semantic_instances",
        "observation_events",
        "scene_spaces",
        "navigation_trajectory_events",
        "semantic_graph_relations",
        "semantic_summary_nodes",
    )

    def _ensure_library_name_columns(self) -> None:
        """确保 8 张记忆表都有 library_name 列（不存在则 ALTER）。"""

        with self._transaction():
            for table_name in self._LIBRARY_NAME_TABLES:
                self._ensure_columns(table_name, {"library_name": "TEXT"})

    def _ensure_map_name_columns(self) -> None:
        with self._transaction():
            for table_name in self._LIBRARY_NAME_TABLES:
                self._ensure_columns(table_name, {"map_name": "TEXT"})

    def _backfill_map_names_from_library_names(self) -> None:
        with self._transaction() as transaction:
            for table_name in self._LIBRARY_NAME_TABLES:
                transaction.execute(
                    f"""
                    UPDATE {table_name}
                    SET map_name = TRIM(library_name)
                    WHERE (map_name IS NULL OR TRIM(map_name) = '')
                      AND library_name IS NOT NULL
                      AND TRIM(library_name) != ''
                      AND TRIM(library_name) != 'legacy_default'
                    """
                )

    @staticmethod
    def _resolve_record_library_name(record: object) -> str:
        """从 record.metadata 提取 MEMORY_LIBRARY_METADATA_KEY；缺失则 LEGACY_DEFAULT。

        与 services/memory/service.py::_record_library_name 行为保持一致：
        - metadata[MEMORY_LIBRARY_METADATA_KEY] 非空字符串 → 用之；
        - 否则返回 LEGACY_DEFAULT（"legacy_default"），与旧全量实现兜底语义一致。
        """

        MEMORY_LIBRARY_METADATA_KEY = "memory_library_name"
        LEGACY_DEFAULT = "legacy_default"
        metadata = getattr(record, "metadata", None)
        if isinstance(metadata, dict):
            value = metadata.get(MEMORY_LIBRARY_METADATA_KEY)
            if isinstance(value, str) and value.strip():
                return value.strip()
        return LEGACY_DEFAULT

    def _ensure_library_name_indexes(self) -> None:
        """为每张记忆表的 library_name 列加索引（按库聚合 O(logN)）。"""

        with self._transaction() as transaction:
            for table_name in self._LIBRARY_NAME_TABLES:
                index_name = f"idx_{table_name}_library_name"
                transaction.execute(
                    f"CREATE INDEX IF NOT EXISTS {index_name} ON {table_name}(library_name)"
                )

    def has_library_name_columns(self) -> bool:
        """检测所有记忆表是否都已具备 library_name 列（用于轻量快路径就绪判定）。"""

        with self._read_snapshot() as transaction:
            for table_name in self._LIBRARY_NAME_TABLES:
                columns = {
                    str(row["name"])
                    for row in transaction.execute(f"PRAGMA table_info({table_name})").fetchall()
                }
                if "library_name" not in columns:
                    return False
        return True

    def count_records_by_library(self) -> Dict[str, Dict[str, int]]:
        """按库聚合各记忆表的轻量计数（不反序列化 payload）。

        返回结构：{ library_name: { record_kind: count, ... }, ... }。
        record_kind 取值：tagged_location / semantic_memory / semantic_instance /
        observation_event / scene_space / navigation_trajectory /
        semantic_graph_relation / semantic_summary。
        """

        table_to_kind = (
            ("tagged_locations", "tagged_location"),
            ("semantic_memories", "semantic_memory"),
            ("semantic_instances", "semantic_instance"),
            ("observation_events", "observation_event"),
            ("scene_spaces", "scene_space"),
            ("navigation_trajectory_events", "navigation_trajectory"),
            ("semantic_graph_relations", "semantic_graph_relation"),
            ("semantic_summary_nodes", "semantic_summary"),
        )
        result: Dict[str, Dict[str, int]] = {}
        with self._read_snapshot() as transaction:
            for table_name, record_kind in table_to_kind:
                rows = transaction.execute(
                    f"""
                    SELECT COALESCE(library_name, '') AS library_name, COUNT(*) AS cnt
                    FROM {table_name}
                    GROUP BY library_name
                    """
                ).fetchall()
                for row in rows:
                    name = str(row["library_name"] or "").strip()
                    if not name:
                        # 列虽存在但未回填；按 legacy_default 计入以与旧全量实现语义一致。
                        name = "legacy_default"
                    bucket = result.setdefault(name, {})
                    bucket[record_kind] = int(row["cnt"])
        return result

    def library_last_updated_at(self) -> Dict[str, str]:
        """按库返回最新更新时间（取各表 updated_at/created_at 最大值）。"""

        table_to_kind: tuple = ()  # 仅用于按表选 updated 列名
        del table_to_kind
        table_to_updated = (
            ("tagged_locations", "updated_at"),
            ("semantic_memories", "updated_at"),
            ("semantic_instances", "updated_at"),
            ("observation_events", "updated_at"),
            ("scene_spaces", "updated_at"),
            ("navigation_trajectory_events", "updated_at"),
            ("semantic_graph_relations", "updated_at"),
            ("semantic_summary_nodes", "updated_at"),
        )
        merged: Dict[str, str] = {}
        with self._read_snapshot() as transaction:
            for table_name, updated_col in table_to_updated:
                rows = transaction.execute(
                    f"""
                    SELECT COALESCE(library_name, '') AS library_name, MAX({updated_col}) AS last_ts
                    FROM {table_name}
                    GROUP BY library_name
                    """
                ).fetchall()
                for row in rows:
                    name = str(row["library_name"] or "").strip() or "legacy_default"
                    last_ts = str(row["last_ts"] or "") if row["last_ts"] is not None else ""
                    if not last_ts:
                        continue
                    existing = merged.get(name)
                    if existing is None or last_ts > existing:
                        merged[name] = last_ts
        return merged

    def backfill_library_names(self, *, batch_size: int = 500) -> Dict[str, int]:
        """Resume NULL library-name migration in independently committed batches.

        When called inside an owner transaction (including schema setup), every
        batch instead nests a savepoint and remains provisional until its caller
        commits. Standalone decoding runs outside the writer lock; guarded SQL
        leaves a concurrently changed payload/ownership for a later batch.
        """
        self._check_open()
        table_to_pk = (
            ("tagged_locations", "location_id"),
            ("semantic_memories", "memory_id"),
            ("semantic_instances", "instance_id"),
            ("observation_events", "event_id"),
            ("scene_spaces", "scene_space_id"),
            ("navigation_trajectory_events", "trajectory_event_id"),
            ("semantic_graph_relations", "relation_id"),
            ("semantic_summary_nodes", "summary_id"),
        )
        updated_totals: Dict[str, int] = {}
        for table_name, pk_col in table_to_pk:
            updated = 0
            while True:
                with self._read_snapshot() as transaction:
                    rows = transaction.execute(
                        f"""
                        SELECT {pk_col} AS pk, payload_json AS payload_json
                        FROM {table_name}
                        WHERE library_name IS NULL
                        LIMIT ?
                        """,
                        (batch_size,),
                    ).fetchall()
                if not rows:
                    break
                updates = self._prepare_library_name_backfill(rows)
                with self._transaction() as transaction:
                    updated += self._backfill_library_names_batch_tx(transaction, table_name, pk_col, updates)
            if updated:
                updated_totals[table_name] = updated
        return updated_totals

    @staticmethod
    def _prepare_library_name_backfill(rows: Sequence[sqlite3.Row]) -> List[tuple]:
        updates: List[tuple] = []
        for row in rows:
            payload_str = str(row["payload_json"] or "")
            library_name = "legacy_default"
            if payload_str:
                try:
                    payload = json.loads(payload_str)
                    metadata = payload.get("metadata") if isinstance(payload, dict) else None
                    value = metadata.get("memory_library_name") if isinstance(metadata, dict) else None
                    if isinstance(value, str) and value.strip():
                        library_name = value.strip()
                except (ValueError, TypeError):
                    # Preserve the legacy fallback for damaged payloads.
                    library_name = "legacy_default"
            updates.append((library_name, str(row["pk"]), row["payload_json"]))
        return updates

    @staticmethod
    def _backfill_library_names_batch_tx(
        transaction: SQLiteTransaction,
        table_name: str,
        pk_col: str,
        updates: Sequence[tuple],
    ) -> int:
        """Apply one prepared batch without controlling the caller's boundary."""
        cursor = transaction.executemany(
            f"UPDATE {table_name} SET library_name = ? WHERE {pk_col} = ? AND library_name IS NULL AND payload_json IS ?",
            updates,
        )
        return int(cursor.rowcount or 0)

    def _init_schema(self) -> None:
        with self._transaction() as transaction:
            transaction.execute(
                """
                CREATE TABLE IF NOT EXISTS tagged_locations (
                    location_id TEXT PRIMARY KEY,
                    normalized_name TEXT NOT NULL,
                    map_name TEXT,
                    payload_json TEXT NOT NULL,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                )
                """
            )
            transaction.execute(
                "CREATE INDEX IF NOT EXISTS idx_tagged_locations_normalized_name ON tagged_locations(normalized_name)"
            )
            transaction.execute(
                """
                CREATE TABLE IF NOT EXISTS semantic_memories (
                    memory_id TEXT PRIMARY KEY,
                    kind TEXT NOT NULL,
                    title TEXT NOT NULL,
                    summary TEXT NOT NULL,
                    map_name TEXT,
                    payload_json TEXT NOT NULL,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                )
                """
            )
            transaction.execute(
                "CREATE INDEX IF NOT EXISTS idx_semantic_memories_kind ON semantic_memories(kind)"
            )
            transaction.execute(
                """
                CREATE TABLE IF NOT EXISTS memory_libraries (
                    library_name TEXT PRIMARY KEY,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                )
                """
            )
            transaction.execute(
                "CREATE INDEX IF NOT EXISTS idx_memory_libraries_updated_at ON memory_libraries(updated_at)"
            )
            transaction.execute(
                """
                CREATE TABLE IF NOT EXISTS memory_vectors (
                    namespace TEXT NOT NULL,
                    record_id TEXT NOT NULL,
                    embedding_model TEXT NOT NULL,
                    embedding_dim INTEGER NOT NULL,
                    source_text TEXT NOT NULL,
                    vector_payload TEXT NOT NULL,
                    updated_at TEXT NOT NULL,
                    PRIMARY KEY(namespace, record_id)
                )
                """
            )
            transaction.execute(
                "CREATE INDEX IF NOT EXISTS idx_memory_vectors_namespace ON memory_vectors(namespace)"
            )
            transaction.execute(
                """
                CREATE TABLE IF NOT EXISTS scene_spaces (
                    scene_space_id TEXT PRIMARY KEY,
                    map_name TEXT,
                    localization_session_id TEXT,
                    frame_id TEXT NOT NULL,
                    primary_label TEXT,
                    review_status TEXT NOT NULL,
                    payload_json TEXT NOT NULL,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL,
                    semantic_updated_at TEXT
                )
                """
            )
            self._ensure_columns(
                "scene_spaces",
                {
                    "map_name": "TEXT",
                    "localization_session_id": "TEXT",
                },
            )
            transaction.execute(
                "CREATE INDEX IF NOT EXISTS idx_scene_spaces_map_name ON scene_spaces(map_name)"
            )
            transaction.execute(
                "CREATE INDEX IF NOT EXISTS idx_scene_spaces_localization_session_id ON scene_spaces(localization_session_id)"
            )
            transaction.execute(
                "CREATE INDEX IF NOT EXISTS idx_scene_spaces_frame_id ON scene_spaces(frame_id)"
            )
            transaction.execute(
                "CREATE INDEX IF NOT EXISTS idx_scene_spaces_primary_label ON scene_spaces(primary_label)"
            )
            transaction.execute(
                "CREATE INDEX IF NOT EXISTS idx_scene_spaces_review_status ON scene_spaces(review_status)"
            )
            transaction.execute(
                "CREATE INDEX IF NOT EXISTS idx_scene_spaces_updated_at ON scene_spaces(updated_at)"
            )
            transaction.execute(
                """
                CREATE TABLE IF NOT EXISTS semantic_instances (
                    instance_id TEXT PRIMARY KEY,
                    label TEXT NOT NULL,
                    lifecycle_state TEXT NOT NULL,
                    map_name TEXT,
                    localization_session_id TEXT,
                    payload_json TEXT NOT NULL,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                )
                """
            )
            self._ensure_columns(
                "semantic_instances",
                {
                    "map_name": "TEXT",
                    "localization_session_id": "TEXT",
                },
            )
            transaction.execute(
                "CREATE INDEX IF NOT EXISTS idx_semantic_instances_label ON semantic_instances(label)"
            )
            transaction.execute(
                "CREATE INDEX IF NOT EXISTS idx_semantic_instances_lifecycle_state ON semantic_instances(lifecycle_state)"
            )
            transaction.execute(
                "CREATE INDEX IF NOT EXISTS idx_semantic_instances_map_name ON semantic_instances(map_name)"
            )
            transaction.execute(
                "CREATE INDEX IF NOT EXISTS idx_semantic_instances_localization_session_id ON semantic_instances(localization_session_id)"
            )
            transaction.execute(
                """
                CREATE TABLE IF NOT EXISTS observation_events (
                    event_id TEXT PRIMARY KEY,
                    anchor_id TEXT,
                    map_name TEXT,
                    localization_session_id TEXT,
                    payload_json TEXT NOT NULL,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                )
                """
            )
            self._ensure_columns(
                "observation_events",
                {
                    "map_name": "TEXT",
                    "localization_session_id": "TEXT",
                },
            )
            transaction.execute(
                "CREATE INDEX IF NOT EXISTS idx_observation_events_anchor_id ON observation_events(anchor_id)"
            )
            transaction.execute(
                "CREATE INDEX IF NOT EXISTS idx_observation_events_created_at ON observation_events(created_at)"
            )
            transaction.execute(
                "CREATE INDEX IF NOT EXISTS idx_observation_events_map_name ON observation_events(map_name)"
            )
            transaction.execute(
                "CREATE INDEX IF NOT EXISTS idx_observation_events_localization_session_id ON observation_events(localization_session_id)"
            )
            transaction.execute(
                """
                CREATE TABLE IF NOT EXISTS navigation_trajectory_events (
                    trajectory_event_id TEXT PRIMARY KEY,
                    event_kind TEXT NOT NULL,
                    map_name TEXT,
                    localization_session_id TEXT,
                    source_record_id TEXT,
                    payload_json TEXT NOT NULL,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                )
                """
            )
            self._ensure_columns(
                "navigation_trajectory_events",
                {
                    "map_name": "TEXT",
                    "localization_session_id": "TEXT",
                    "source_record_id": "TEXT",
                },
            )
            transaction.execute(
                "CREATE INDEX IF NOT EXISTS idx_navigation_trajectory_events_kind ON navigation_trajectory_events(event_kind)"
            )
            transaction.execute(
                "CREATE INDEX IF NOT EXISTS idx_navigation_trajectory_events_map_name ON navigation_trajectory_events(map_name)"
            )
            transaction.execute(
                "CREATE INDEX IF NOT EXISTS idx_navigation_trajectory_events_localization_session_id ON navigation_trajectory_events(localization_session_id)"
            )
            transaction.execute(
                "CREATE INDEX IF NOT EXISTS idx_navigation_trajectory_events_source_record_id ON navigation_trajectory_events(source_record_id)"
            )
            transaction.execute(
                """
                CREATE TABLE IF NOT EXISTS semantic_graph_relations (
                    relation_id TEXT PRIMARY KEY,
                    relation_kind TEXT NOT NULL,
                    source_node_kind TEXT NOT NULL,
                    source_node_id TEXT NOT NULL,
                    target_node_kind TEXT NOT NULL,
                    target_node_id TEXT NOT NULL,
                    map_name TEXT,
                    localization_session_id TEXT,
                    payload_json TEXT NOT NULL,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                )
                """
            )
            self._ensure_columns(
                "semantic_graph_relations",
                {
                    "map_name": "TEXT",
                    "localization_session_id": "TEXT",
                },
            )
            transaction.execute(
                "CREATE INDEX IF NOT EXISTS idx_semantic_graph_relations_kind ON semantic_graph_relations(relation_kind)"
            )
            transaction.execute(
                "CREATE INDEX IF NOT EXISTS idx_semantic_graph_relations_source ON semantic_graph_relations(source_node_kind, source_node_id)"
            )
            transaction.execute(
                "CREATE INDEX IF NOT EXISTS idx_semantic_graph_relations_target ON semantic_graph_relations(target_node_kind, target_node_id)"
            )
            transaction.execute(
                "CREATE INDEX IF NOT EXISTS idx_semantic_graph_relations_map_name ON semantic_graph_relations(map_name)"
            )
            transaction.execute(
                "CREATE INDEX IF NOT EXISTS idx_semantic_graph_relations_localization_session_id ON semantic_graph_relations(localization_session_id)"
            )
            transaction.execute(
                """
                CREATE TABLE IF NOT EXISTS semantic_summary_nodes (
                    summary_id TEXT PRIMARY KEY,
                    summary_kind TEXT NOT NULL,
                    map_name TEXT,
                    localization_session_id TEXT,
                    scene_space_id TEXT,
                    semantic_region_id TEXT,
                    label TEXT,
                    payload_json TEXT NOT NULL,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                )
                """
            )
            self._ensure_columns(
                "semantic_summary_nodes",
                {
                    "map_name": "TEXT",
                    "localization_session_id": "TEXT",
                    "scene_space_id": "TEXT",
                    "semantic_region_id": "TEXT",
                    "label": "TEXT",
                },
            )
            transaction.execute(
                "CREATE INDEX IF NOT EXISTS idx_semantic_summary_nodes_kind ON semantic_summary_nodes(summary_kind)"
            )
            transaction.execute(
                "CREATE INDEX IF NOT EXISTS idx_semantic_summary_nodes_map_name ON semantic_summary_nodes(map_name)"
            )
            transaction.execute(
                "CREATE INDEX IF NOT EXISTS idx_semantic_summary_nodes_localization_session_id ON semantic_summary_nodes(localization_session_id)"
            )
            transaction.execute(
                "CREATE INDEX IF NOT EXISTS idx_semantic_summary_nodes_scene_space_id ON semantic_summary_nodes(scene_space_id)"
            )
            transaction.execute(
                "CREATE INDEX IF NOT EXISTS idx_semantic_summary_nodes_label ON semantic_summary_nodes(label)"
            )
            # —— speed-up-list-maps-summary ——
            # 给 8 张记忆表补齐库归属与稳定地图主键。旧 map_version_id 列可能仍在
            # 旧数据库中，但不再参与新写入、过滤或归属推导。
            self._ensure_library_name_columns()
            self._ensure_columns("scene_spaces", {"graph_geometry_revision": "TEXT NOT NULL DEFAULT ''", "graph_source_revision": "TEXT NOT NULL DEFAULT ''"})
            from services.memory.scene_identity import install_schema as install_scene_identity_schema
            install_scene_identity_schema(transaction)
            self._ensure_columns("semantic_instances", {"graph_source_revision": "TEXT NOT NULL DEFAULT ''"})
            transaction.execute("""CREATE TABLE IF NOT EXISTS memory_scene_graph_inputs (
                library_name TEXT PRIMARY KEY, source_generation INTEGER NOT NULL DEFAULT 0,
                geometry_generation INTEGER NOT NULL DEFAULT 0, memory_generation INTEGER NOT NULL DEFAULT 0)""")
            self._ensure_columns("memory_scene_graph_inputs", {"memory_generation": "INTEGER NOT NULL DEFAULT 0"})
            for table in ("semantic_memories", "semantic_instances", "scene_spaces"):
                memory_change = 1 if table == "semantic_memories" else 0
                def eligible(row):
                    return (f"CASE WHEN json_valid({row}.payload_json) THEN CASE WHEN "
                            f"json_extract({row}.payload_json,'$.metadata.scene_mapping_managed')=1 "
                            f"AND {row}.review_status NOT IN ('stale','rejected') THEN 1 ELSE 0 END ELSE 0 END")
                geometry = eligible("OLD") if table == "scene_spaces" else "0"
                changed = (f"CASE WHEN COALESCE(OLD.library_name,'legacy_default') != COALESCE(NEW.library_name,'legacy_default') "
                           f"THEN {eligible('NEW')} WHEN ({eligible('OLD')}) != ({eligible('NEW')}) THEN 1 "
                           f"WHEN ({eligible('NEW')})=1 AND OLD.graph_geometry_revision != NEW.graph_geometry_revision THEN 1 ELSE 0 END") if table == "scene_spaces" else "0"
                for operation, row, amount in (("insert", "NEW", geometry.replace("OLD.", "NEW.")), ("delete", "OLD", geometry)):
                    transaction.execute(f"""CREATE TRIGGER IF NOT EXISTS scene_graph_{table}_{operation}
                        AFTER {operation.upper()} ON {table} BEGIN
                        INSERT INTO memory_scene_graph_inputs VALUES(COALESCE({row}.library_name,'legacy_default'),1,{amount},{memory_change})
                        ON CONFLICT(library_name) DO UPDATE SET source_generation=source_generation+1,
                            geometry_generation=geometry_generation+{amount},memory_generation=memory_generation+{memory_change}; END""")
                transaction.execute(f"""CREATE TRIGGER IF NOT EXISTS scene_graph_{table}_update
                    AFTER UPDATE ON {table} BEGIN
                    INSERT INTO memory_scene_graph_inputs VALUES(COALESCE(NEW.library_name,'legacy_default'),1,{changed},{memory_change})
                    ON CONFLICT(library_name) DO UPDATE SET source_generation=source_generation+1,
                        geometry_generation=geometry_generation+{changed},memory_generation=memory_generation+{memory_change};
                    INSERT INTO memory_scene_graph_inputs SELECT COALESCE(OLD.library_name,'legacy_default'),1,{geometry},{memory_change}
                        WHERE COALESCE(OLD.library_name,'legacy_default') != COALESCE(NEW.library_name,'legacy_default')
                    ON CONFLICT(library_name) DO UPDATE SET source_generation=source_generation+1,
                        geometry_generation=geometry_generation+{geometry},memory_generation=memory_generation+{memory_change}; END""")
            for table in ("navigation_trajectory_events", "semantic_graph_relations", "semantic_summary_nodes"):
                self._ensure_columns(table, {"publication_revision": "TEXT NOT NULL DEFAULT ''"})
            transaction.execute("""CREATE TABLE IF NOT EXISTS memory_record_natural_ids (
                library_incarnation TEXT NOT NULL, record_kind TEXT NOT NULL,
                natural_key TEXT NOT NULL, record_id TEXT NOT NULL,
                PRIMARY KEY(library_incarnation,record_kind,natural_key))""")
            transaction.execute("CREATE INDEX IF NOT EXISTS idx_memory_record_natural_ids_record ON memory_record_natural_ids(record_kind,record_id)")
            transaction.execute("CREATE INDEX IF NOT EXISTS idx_semantic_graph_relations_natural ON semantic_graph_relations(library_name,relation_kind,source_node_kind,source_node_id,target_node_kind,target_node_id)")
            transaction.execute("CREATE INDEX IF NOT EXISTS idx_semantic_summary_nodes_natural ON semantic_summary_nodes(library_name,summary_kind,COALESCE(label,''),COALESCE(scene_space_id,''),COALESCE(semantic_region_id,''))")
            self._ensure_map_name_columns()
            self.backfill_library_names()
            self._backfill_map_names_from_library_names()
            self._ensure_library_name_indexes()
            for table_name in self._LIBRARY_NAME_TABLES:
                transaction.execute(
                    f"CREATE INDEX IF NOT EXISTS idx_{table_name}_map_name ON {table_name}(map_name)"
                )

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:  # noqa: BLE001 - 析构时尽力清理，包括未完成的初始化
            return
