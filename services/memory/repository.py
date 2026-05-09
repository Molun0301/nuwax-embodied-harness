from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sqlite3
import threading
from typing import Dict, Iterable, List, Optional, Sequence

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


class MemoryRepository:
    """记忆库的轻量 SQLite 仓储。"""

    def __init__(self, db_path: str) -> None:
        self._db_path = db_path.strip() or ":memory:"
        self._lock = threading.RLock()
        if self._db_path != ":memory:":
            db_file = Path(self._db_path)
            db_file.parent.mkdir(parents=True, exist_ok=True)
        self._conn = sqlite3.connect(self._db_path, check_same_thread=False)
        self._conn.row_factory = sqlite3.Row
        with self._lock:
            if self._db_path != ":memory:":
                self._conn.execute("PRAGMA journal_mode=WAL")
            self._conn.execute("PRAGMA synchronous=NORMAL")
        self._init_schema()

    @property
    def db_path(self) -> str:
        return self._db_path

    def close(self) -> None:
        with self._lock:
            self._conn.close()

    def load_tagged_locations(self) -> List[TaggedLocation]:
        with self._lock:
            rows = self._conn.execute(
                """
                SELECT payload_json
                FROM tagged_locations
                ORDER BY created_at ASC, location_id ASC
                """
            ).fetchall()
        return [TaggedLocation.model_validate_json(str(row["payload_json"])) for row in rows]

    def load_semantic_memories(self) -> List[SemanticMemoryEntry]:
        with self._lock:
            rows = self._conn.execute(
                """
                SELECT payload_json
                FROM semantic_memories
                ORDER BY created_at ASC, memory_id ASC
                """
            ).fetchall()
        return [SemanticMemoryEntry.model_validate_json(str(row["payload_json"])) for row in rows]

    def load_semantic_instances(self) -> List[SemanticInstance]:
        with self._lock:
            rows = self._conn.execute(
                """
                SELECT payload_json
                FROM semantic_instances
                ORDER BY updated_at ASC, instance_id ASC
                """
            ).fetchall()
        return [SemanticInstance.model_validate_json(str(row["payload_json"])) for row in rows]

    def load_observation_events(self) -> List[ObservationEvent]:
        with self._lock:
            rows = self._conn.execute(
                """
                SELECT payload_json
                FROM observation_events
                ORDER BY created_at ASC, event_id ASC
                """
            ).fetchall()
        return [ObservationEvent.model_validate_json(str(row["payload_json"])) for row in rows]

    def load_scene_spaces(self) -> List[SceneSpace]:
        with self._lock:
            rows = self._conn.execute(
                """
                SELECT payload_json
                FROM scene_spaces
                ORDER BY updated_at ASC, scene_space_id ASC
                """
            ).fetchall()
        return [SceneSpace.model_validate_json(str(row["payload_json"])) for row in rows]

    def load_navigation_trajectory_events(self) -> List[NavigationTrajectoryEvent]:
        with self._lock:
            rows = self._conn.execute(
                """
                SELECT payload_json
                FROM navigation_trajectory_events
                ORDER BY created_at ASC, trajectory_event_id ASC
                """
            ).fetchall()
        return [NavigationTrajectoryEvent.model_validate_json(str(row["payload_json"])) for row in rows]

    def load_semantic_graph_relations(self) -> List[SemanticGraphRelation]:
        with self._lock:
            rows = self._conn.execute(
                """
                SELECT payload_json
                FROM semantic_graph_relations
                ORDER BY updated_at ASC, relation_id ASC
                """
            ).fetchall()
        return [SemanticGraphRelation.model_validate_json(str(row["payload_json"])) for row in rows]

    def load_semantic_summary_nodes(self) -> List[SemanticSummaryNode]:
        with self._lock:
            rows = self._conn.execute(
                """
                SELECT payload_json
                FROM semantic_summary_nodes
                ORDER BY updated_at ASC, summary_id ASC
                """
            ).fetchall()
        return [SemanticSummaryNode.model_validate_json(str(row["payload_json"])) for row in rows]

    def load_scene_space(self, scene_space_id: str) -> Optional[SceneSpace]:
        resolved_id = str(scene_space_id or "").strip()
        if not resolved_id:
            return None
        with self._lock:
            row = self._conn.execute(
                """
                SELECT payload_json
                FROM scene_spaces
                WHERE scene_space_id = ?
                """,
                (resolved_id,),
            ).fetchone()
        if row is None:
            return None
        return SceneSpace.model_validate_json(str(row["payload_json"]))

    def list_scene_spaces(
        self,
        *,
        scene_space_ids: Optional[Sequence[str]] = None,
        map_version_id: Optional[str] = None,
        localization_session_id: Optional[str] = None,
        review_statuses: Optional[Sequence[object]] = None,
        limit: Optional[int] = None,
    ) -> List[SceneSpace]:
        where_clauses: List[str] = []
        params: List[object] = []
        resolved_ids = [str(item).strip() for item in scene_space_ids or [] if str(item).strip()]
        if resolved_ids:
            placeholders = ", ".join("?" for _ in resolved_ids)
            where_clauses.append(f"scene_space_id IN ({placeholders})")
            params.extend(resolved_ids)
        if map_version_id:
            where_clauses.append("map_version_id = ?")
            params.append(map_version_id)
        if localization_session_id:
            where_clauses.append("localization_session_id = ?")
            params.append(localization_session_id)
        resolved_statuses = self._enum_values(review_statuses or [])
        if resolved_statuses:
            placeholders = ", ".join("?" for _ in resolved_statuses)
            where_clauses.append(f"review_status IN ({placeholders})")
            params.extend(resolved_statuses)
        query = "SELECT payload_json FROM scene_spaces"
        if where_clauses:
            query = f"{query} WHERE {' AND '.join(where_clauses)}"
        query = f"{query} ORDER BY updated_at DESC, scene_space_id ASC"
        if limit is not None and limit > 0:
            query = f"{query} LIMIT ?"
            params.append(int(limit))
        with self._lock:
            rows = self._conn.execute(query, tuple(params)).fetchall()
        return [SceneSpace.model_validate_json(str(row["payload_json"])) for row in rows]

    def query_scene_spaces(
        self,
        payload_filter: Optional[MemoryPayloadFilter] = None,
        *,
        limit: Optional[int] = None,
    ) -> List[SceneSpace]:
        if payload_filter is None:
            return self.list_scene_spaces(limit=limit)
        if payload_filter.record_kinds:
            record_kinds = self._enum_values(payload_filter.record_kinds)
            if SCENE_SPACE_VECTOR_NAMESPACE not in record_kinds:
                return []
        spaces = self.list_scene_spaces(
            scene_space_ids=payload_filter.scene_space_ids,
            map_version_id=payload_filter.map_version_id,
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
        with self._lock:
            rows = self._conn.execute(query, tuple(params)).fetchall()
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
        with self._lock:
            rows = self._conn.execute(
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
        with self._lock:
            self._conn.execute(
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
            self._conn.commit()

    def delete_memory_library(self, library_name: str) -> bool:
        resolved_name = str(library_name or "").strip()
        if not resolved_name:
            return False
        with self._lock:
            cursor = self._conn.execute(
                "DELETE FROM memory_libraries WHERE library_name = ?",
                (resolved_name,),
            )
            self._conn.commit()
        return int(cursor.rowcount or 0) > 0

    def upsert_tagged_location(self, location: TaggedLocation) -> None:
        payload_json = location.model_dump_json()
        created_at = location.timestamp.isoformat()
        with self._lock:
            self._conn.execute(
                """
                INSERT INTO tagged_locations (
                    location_id,
                    normalized_name,
                    payload_json,
                    created_at,
                    updated_at
                )
                VALUES (?, ?, ?, ?, ?)
                ON CONFLICT(location_id) DO UPDATE SET
                    normalized_name = excluded.normalized_name,
                    payload_json = excluded.payload_json,
                    updated_at = excluded.updated_at
                """,
                (
                    location.location_id,
                    location.normalized_name,
                    payload_json,
                    created_at,
                    created_at,
                ),
            )
            self._conn.commit()

    def upsert_semantic_memory(self, entry: SemanticMemoryEntry) -> None:
        payload_json = entry.model_dump_json()
        created_at = entry.timestamp.isoformat()
        with self._lock:
            self._conn.execute(
                """
                INSERT INTO semantic_memories (
                    memory_id,
                    kind,
                    title,
                    summary,
                    payload_json,
                    created_at,
                    updated_at
                )
                VALUES (?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(memory_id) DO UPDATE SET
                    kind = excluded.kind,
                    title = excluded.title,
                    summary = excluded.summary,
                    payload_json = excluded.payload_json,
                    updated_at = excluded.updated_at
                """,
                (
                    entry.memory_id,
                    entry.kind.value,
                    entry.title,
                    entry.summary,
                    payload_json,
                    created_at,
                    created_at,
                ),
            )
            self._conn.commit()

    def upsert_semantic_instance(self, entry: SemanticInstance) -> None:
        payload_json = entry.model_dump_json()
        created_at = entry.first_seen_ts.isoformat()
        updated_at = entry.last_seen_ts.isoformat()
        with self._lock:
            self._conn.execute(
                """
                INSERT INTO semantic_instances (
                    instance_id,
                    label,
                    lifecycle_state,
                    map_version_id,
                    localization_session_id,
                    payload_json,
                    created_at,
                    updated_at
                )
                VALUES (?, ?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(instance_id) DO UPDATE SET
                    label = excluded.label,
                    lifecycle_state = excluded.lifecycle_state,
                    map_version_id = excluded.map_version_id,
                    localization_session_id = excluded.localization_session_id,
                    payload_json = excluded.payload_json,
                    updated_at = excluded.updated_at
                """,
                (
                    entry.instance_id,
                    entry.label,
                    entry.lifecycle_state.value,
                    entry.map_version_id,
                    entry.localization_session_id,
                    payload_json,
                    created_at,
                    updated_at,
                ),
            )
            self._conn.commit()

    def upsert_observation_event(self, entry: ObservationEvent) -> None:
        payload_json = entry.model_dump_json()
        created_at = entry.timestamp.isoformat()
        with self._lock:
            self._conn.execute(
                """
                INSERT INTO observation_events (
                    event_id,
                    anchor_id,
                    map_version_id,
                    localization_session_id,
                    payload_json,
                    created_at,
                    updated_at
                )
                VALUES (?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(event_id) DO UPDATE SET
                    anchor_id = excluded.anchor_id,
                    map_version_id = excluded.map_version_id,
                    localization_session_id = excluded.localization_session_id,
                    payload_json = excluded.payload_json,
                    updated_at = excluded.updated_at
                """,
                (
                    entry.event_id,
                    entry.anchor_id,
                    entry.map_version_id,
                    entry.localization_session_id,
                    payload_json,
                    created_at,
                    created_at,
                ),
            )
            self._conn.commit()

    def upsert_scene_space(self, entry: SceneSpace) -> None:
        payload_json = entry.model_dump_json()
        created_at = entry.timestamp.isoformat()
        semantic_updated_at = entry.semantic_updated_at.isoformat() if entry.semantic_updated_at else None
        updated_at = semantic_updated_at or created_at
        with self._lock:
            self._conn.execute(
                """
                INSERT INTO scene_spaces (
                    scene_space_id,
                    map_version_id,
                    localization_session_id,
                    frame_id,
                    primary_label,
                    review_status,
                    payload_json,
                    created_at,
                    updated_at,
                    semantic_updated_at
                )
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(scene_space_id) DO UPDATE SET
                    map_version_id = excluded.map_version_id,
                    localization_session_id = excluded.localization_session_id,
                    frame_id = excluded.frame_id,
                    primary_label = excluded.primary_label,
                    review_status = excluded.review_status,
                    payload_json = excluded.payload_json,
                    updated_at = excluded.updated_at,
                    semantic_updated_at = excluded.semantic_updated_at
                """,
                (
                    entry.scene_space_id,
                    entry.map_version_id,
                    entry.localization_session_id,
                    entry.frame_id,
                    entry.primary_label,
                    entry.review_status.value,
                    payload_json,
                    created_at,
                    updated_at,
                    semantic_updated_at,
                ),
            )
            self._conn.commit()

    def upsert_navigation_trajectory_event(self, entry: NavigationTrajectoryEvent) -> None:
        payload_json = entry.model_dump_json()
        created_at = entry.timestamp.isoformat()
        with self._lock:
            self._conn.execute(
                """
                INSERT INTO navigation_trajectory_events (
                    trajectory_event_id,
                    event_kind,
                    map_version_id,
                    localization_session_id,
                    source_record_id,
                    payload_json,
                    created_at,
                    updated_at
                )
                VALUES (?, ?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(trajectory_event_id) DO UPDATE SET
                    event_kind = excluded.event_kind,
                    map_version_id = excluded.map_version_id,
                    localization_session_id = excluded.localization_session_id,
                    source_record_id = excluded.source_record_id,
                    payload_json = excluded.payload_json,
                    updated_at = excluded.updated_at
                """,
                (
                    entry.trajectory_event_id,
                    entry.event_kind.value,
                    entry.map_version_id,
                    entry.localization_session_id,
                    entry.source_record_id,
                    payload_json,
                    created_at,
                    created_at,
                ),
            )
            self._conn.commit()

    def upsert_semantic_graph_relation(self, entry: SemanticGraphRelation) -> None:
        payload_json = entry.model_dump_json()
        created_at = entry.timestamp.isoformat()
        with self._lock:
            self._conn.execute(
                """
                INSERT INTO semantic_graph_relations (
                    relation_id,
                    relation_kind,
                    source_node_kind,
                    source_node_id,
                    target_node_kind,
                    target_node_id,
                    map_version_id,
                    localization_session_id,
                    payload_json,
                    created_at,
                    updated_at
                )
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(relation_id) DO UPDATE SET
                    relation_kind = excluded.relation_kind,
                    source_node_kind = excluded.source_node_kind,
                    source_node_id = excluded.source_node_id,
                    target_node_kind = excluded.target_node_kind,
                    target_node_id = excluded.target_node_id,
                    map_version_id = excluded.map_version_id,
                    localization_session_id = excluded.localization_session_id,
                    payload_json = excluded.payload_json,
                    updated_at = excluded.updated_at
                """,
                (
                    entry.relation_id,
                    entry.relation_kind.value,
                    entry.source_node_kind,
                    entry.source_node_id,
                    entry.target_node_kind,
                    entry.target_node_id,
                    entry.map_version_id,
                    entry.localization_session_id,
                    payload_json,
                    created_at,
                    created_at,
                ),
            )
            self._conn.commit()

    def upsert_semantic_summary_node(self, entry: SemanticSummaryNode) -> None:
        payload_json = entry.model_dump_json()
        created_at = entry.timestamp.isoformat()
        with self._lock:
            self._conn.execute(
                """
                INSERT INTO semantic_summary_nodes (
                    summary_id,
                    summary_kind,
                    map_version_id,
                    localization_session_id,
                    scene_space_id,
                    semantic_region_id,
                    label,
                    payload_json,
                    created_at,
                    updated_at
                )
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(summary_id) DO UPDATE SET
                    summary_kind = excluded.summary_kind,
                    map_version_id = excluded.map_version_id,
                    localization_session_id = excluded.localization_session_id,
                    scene_space_id = excluded.scene_space_id,
                    semantic_region_id = excluded.semantic_region_id,
                    label = excluded.label,
                    payload_json = excluded.payload_json,
                    updated_at = excluded.updated_at
                """,
                (
                    entry.summary_id,
                    entry.summary_kind.value,
                    entry.map_version_id,
                    entry.localization_session_id,
                    entry.scene_space_id,
                    entry.semantic_region_id,
                    entry.label,
                    payload_json,
                    created_at,
                    created_at,
                ),
            )
            self._conn.commit()

    def delete_tagged_locations(self, location_ids: Sequence[str]) -> int:
        normalized_ids = [str(item).strip() for item in location_ids if str(item).strip()]
        if not normalized_ids:
            return 0
        with self._lock:
            self._conn.executemany(
                "DELETE FROM tagged_locations WHERE location_id = ?",
                [(location_id,) for location_id in normalized_ids],
            )
            self._conn.executemany(
                "DELETE FROM memory_vectors WHERE record_id = ?",
                [(location_id,) for location_id in normalized_ids],
            )
            self._conn.commit()
        return len(normalized_ids)

    def delete_semantic_memories(self, memory_ids: Sequence[str]) -> int:
        normalized_ids = [str(item).strip() for item in memory_ids if str(item).strip()]
        if not normalized_ids:
            return 0
        with self._lock:
            self._conn.executemany(
                "DELETE FROM semantic_memories WHERE memory_id = ?",
                [(memory_id,) for memory_id in normalized_ids],
            )
            self._conn.executemany(
                "DELETE FROM memory_vectors WHERE record_id = ?",
                [(memory_id,) for memory_id in normalized_ids],
            )
            self._conn.commit()
        return len(normalized_ids)

    def delete_semantic_instances(self, instance_ids: Sequence[str]) -> int:
        normalized_ids = [str(item).strip() for item in instance_ids if str(item).strip()]
        if not normalized_ids:
            return 0
        with self._lock:
            self._conn.executemany(
                "DELETE FROM semantic_instances WHERE instance_id = ?",
                [(instance_id,) for instance_id in normalized_ids],
            )
            self._conn.executemany(
                "DELETE FROM memory_vectors WHERE record_id = ?",
                [(instance_id,) for instance_id in normalized_ids],
            )
            self._conn.commit()
        return len(normalized_ids)

    def delete_observation_events(self, event_ids: Sequence[str]) -> int:
        normalized_ids = [str(item).strip() for item in event_ids if str(item).strip()]
        if not normalized_ids:
            return 0
        with self._lock:
            self._conn.executemany(
                "DELETE FROM observation_events WHERE event_id = ?",
                [(event_id,) for event_id in normalized_ids],
            )
            self._conn.executemany(
                "DELETE FROM memory_vectors WHERE record_id = ?",
                [(event_id,) for event_id in normalized_ids],
            )
            self._conn.commit()
        return len(normalized_ids)

    def delete_scene_spaces(self, scene_space_ids: Sequence[str]) -> int:
        normalized_ids = [str(item).strip() for item in scene_space_ids if str(item).strip()]
        if not normalized_ids:
            return 0
        with self._lock:
            self._conn.executemany(
                "DELETE FROM scene_spaces WHERE scene_space_id = ?",
                [(scene_space_id,) for scene_space_id in normalized_ids],
            )
            self._conn.executemany(
                "DELETE FROM memory_vectors WHERE record_id = ?",
                [(scene_space_id,) for scene_space_id in normalized_ids],
            )
            self._conn.commit()
        return len(normalized_ids)

    def delete_navigation_trajectory_events(self, trajectory_event_ids: Sequence[str]) -> int:
        normalized_ids = [str(item).strip() for item in trajectory_event_ids if str(item).strip()]
        if not normalized_ids:
            return 0
        with self._lock:
            self._conn.executemany(
                "DELETE FROM navigation_trajectory_events WHERE trajectory_event_id = ?",
                [(trajectory_event_id,) for trajectory_event_id in normalized_ids],
            )
            self._conn.commit()
        return len(normalized_ids)

    def delete_semantic_graph_relations(self, relation_ids: Sequence[str]) -> int:
        normalized_ids = [str(item).strip() for item in relation_ids if str(item).strip()]
        if not normalized_ids:
            return 0
        with self._lock:
            self._conn.executemany(
                "DELETE FROM semantic_graph_relations WHERE relation_id = ?",
                [(relation_id,) for relation_id in normalized_ids],
            )
            self._conn.commit()
        return len(normalized_ids)

    def delete_semantic_summary_nodes(self, summary_ids: Sequence[str]) -> int:
        normalized_ids = [str(item).strip() for item in summary_ids if str(item).strip()]
        if not normalized_ids:
            return 0
        with self._lock:
            self._conn.executemany(
                "DELETE FROM semantic_summary_nodes WHERE summary_id = ?",
                [(summary_id,) for summary_id in normalized_ids],
            )
            self._conn.commit()
        return len(normalized_ids)

    def clear_all(self) -> None:
        with self._lock:
            self._conn.execute("DELETE FROM tagged_locations")
            self._conn.execute("DELETE FROM semantic_memories")
            self._conn.execute("DELETE FROM semantic_instances")
            self._conn.execute("DELETE FROM observation_events")
            self._conn.execute("DELETE FROM scene_spaces")
            self._conn.execute("DELETE FROM navigation_trajectory_events")
            self._conn.execute("DELETE FROM semantic_graph_relations")
            self._conn.execute("DELETE FROM semantic_summary_nodes")
            self._conn.execute("DELETE FROM memory_vectors")
            self._conn.commit()

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
        with self._lock:
            self._conn.execute(
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
            self._conn.commit()

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
        first_position = getattr(first, "position")
        second_position = getattr(second, "position")
        dx = float(first_position.x) - float(second_position.x)
        dy = float(first_position.y) - float(second_position.y)
        dz = float(first_position.z) - float(second_position.z)
        return (dx * dx + dy * dy + dz * dz) ** 0.5

    def _scene_space_matches_filter(self, space: SceneSpace, payload_filter: MemoryPayloadFilter) -> bool:
        if payload_filter.scene_space_ids and space.scene_space_id not in payload_filter.scene_space_ids:
            return False
        if payload_filter.map_version_id and space.map_version_id != payload_filter.map_version_id:
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
        existing = {str(row["name"]) for row in self._conn.execute("PRAGMA table_info(%s)" % table_name).fetchall()}
        for column_name, column_type in columns.items():
            if column_name not in existing:
                self._conn.execute("ALTER TABLE %s ADD COLUMN %s %s" % (table_name, column_name, column_type))

    def _init_schema(self) -> None:
        with self._lock:
            self._conn.execute(
                """
                CREATE TABLE IF NOT EXISTS tagged_locations (
                    location_id TEXT PRIMARY KEY,
                    normalized_name TEXT NOT NULL,
                    payload_json TEXT NOT NULL,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                )
                """
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_tagged_locations_normalized_name ON tagged_locations(normalized_name)"
            )
            self._conn.execute(
                """
                CREATE TABLE IF NOT EXISTS semantic_memories (
                    memory_id TEXT PRIMARY KEY,
                    kind TEXT NOT NULL,
                    title TEXT NOT NULL,
                    summary TEXT NOT NULL,
                    payload_json TEXT NOT NULL,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                )
                """
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_semantic_memories_kind ON semantic_memories(kind)"
            )
            self._conn.execute(
                """
                CREATE TABLE IF NOT EXISTS memory_libraries (
                    library_name TEXT PRIMARY KEY,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                )
                """
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_memory_libraries_updated_at ON memory_libraries(updated_at)"
            )
            self._conn.execute(
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
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_memory_vectors_namespace ON memory_vectors(namespace)"
            )
            self._conn.execute(
                """
                CREATE TABLE IF NOT EXISTS scene_spaces (
                    scene_space_id TEXT PRIMARY KEY,
                    map_version_id TEXT,
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
                    "map_version_id": "TEXT",
                    "localization_session_id": "TEXT",
                },
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_scene_spaces_map_version_id ON scene_spaces(map_version_id)"
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_scene_spaces_localization_session_id ON scene_spaces(localization_session_id)"
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_scene_spaces_frame_id ON scene_spaces(frame_id)"
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_scene_spaces_primary_label ON scene_spaces(primary_label)"
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_scene_spaces_review_status ON scene_spaces(review_status)"
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_scene_spaces_updated_at ON scene_spaces(updated_at)"
            )
            self._conn.execute(
                """
                CREATE TABLE IF NOT EXISTS semantic_instances (
                    instance_id TEXT PRIMARY KEY,
                    label TEXT NOT NULL,
                    lifecycle_state TEXT NOT NULL,
                    map_version_id TEXT,
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
                    "map_version_id": "TEXT",
                    "localization_session_id": "TEXT",
                },
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_semantic_instances_label ON semantic_instances(label)"
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_semantic_instances_lifecycle_state ON semantic_instances(lifecycle_state)"
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_semantic_instances_map_version_id ON semantic_instances(map_version_id)"
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_semantic_instances_localization_session_id ON semantic_instances(localization_session_id)"
            )
            self._conn.execute(
                """
                CREATE TABLE IF NOT EXISTS observation_events (
                    event_id TEXT PRIMARY KEY,
                    anchor_id TEXT,
                    map_version_id TEXT,
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
                    "map_version_id": "TEXT",
                    "localization_session_id": "TEXT",
                },
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_observation_events_anchor_id ON observation_events(anchor_id)"
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_observation_events_created_at ON observation_events(created_at)"
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_observation_events_map_version_id ON observation_events(map_version_id)"
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_observation_events_localization_session_id ON observation_events(localization_session_id)"
            )
            self._conn.execute(
                """
                CREATE TABLE IF NOT EXISTS navigation_trajectory_events (
                    trajectory_event_id TEXT PRIMARY KEY,
                    event_kind TEXT NOT NULL,
                    map_version_id TEXT,
                    localization_session_id TEXT,
                    source_record_id TEXT,
                    payload_json TEXT NOT NULL,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                )
                """
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_navigation_trajectory_events_kind ON navigation_trajectory_events(event_kind)"
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_navigation_trajectory_events_map_version_id ON navigation_trajectory_events(map_version_id)"
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_navigation_trajectory_events_localization_session_id ON navigation_trajectory_events(localization_session_id)"
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_navigation_trajectory_events_source_record_id ON navigation_trajectory_events(source_record_id)"
            )
            self._conn.execute(
                """
                CREATE TABLE IF NOT EXISTS semantic_graph_relations (
                    relation_id TEXT PRIMARY KEY,
                    relation_kind TEXT NOT NULL,
                    source_node_kind TEXT NOT NULL,
                    source_node_id TEXT NOT NULL,
                    target_node_kind TEXT NOT NULL,
                    target_node_id TEXT NOT NULL,
                    map_version_id TEXT,
                    localization_session_id TEXT,
                    payload_json TEXT NOT NULL,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                )
                """
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_semantic_graph_relations_kind ON semantic_graph_relations(relation_kind)"
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_semantic_graph_relations_source ON semantic_graph_relations(source_node_kind, source_node_id)"
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_semantic_graph_relations_target ON semantic_graph_relations(target_node_kind, target_node_id)"
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_semantic_graph_relations_map_version_id ON semantic_graph_relations(map_version_id)"
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_semantic_graph_relations_localization_session_id ON semantic_graph_relations(localization_session_id)"
            )
            self._conn.execute(
                """
                CREATE TABLE IF NOT EXISTS semantic_summary_nodes (
                    summary_id TEXT PRIMARY KEY,
                    summary_kind TEXT NOT NULL,
                    map_version_id TEXT,
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
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_semantic_summary_nodes_kind ON semantic_summary_nodes(summary_kind)"
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_semantic_summary_nodes_map_version_id ON semantic_summary_nodes(map_version_id)"
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_semantic_summary_nodes_localization_session_id ON semantic_summary_nodes(localization_session_id)"
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_semantic_summary_nodes_scene_space_id ON semantic_summary_nodes(scene_space_id)"
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_semantic_summary_nodes_label ON semantic_summary_nodes(label)"
            )
            self._conn.commit()

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            return
