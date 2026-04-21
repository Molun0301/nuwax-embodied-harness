from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import json
import sqlite3
import threading
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from contracts.base import utc_now
from contracts.geometry import Pose
from contracts.memory import MemoryPayloadFilter


TEXT_VECTOR_NAME = "text_dense"
IMAGE_VECTOR_NAME = "image_dense"

PLACE_NODES_COLLECTION = "place_nodes"
OBJECT_INSTANCES_COLLECTION = "object_instances"
EPISODIC_OBSERVATIONS_COLLECTION = "episodic_observations"


def _encode_vector(vector: np.ndarray) -> str:
    return json.dumps([round(float(item), 8) for item in vector.tolist()], ensure_ascii=False)


def _decode_vector(payload: str) -> np.ndarray:
    return np.asarray(json.loads(payload), dtype=np.float32)


def _cosine_similarity(left: np.ndarray, right: np.ndarray) -> float:
    if left.size == 0 or right.size == 0:
        return 0.0
    left_norm = float(np.linalg.norm(left))
    right_norm = float(np.linalg.norm(right))
    if left_norm == 0.0 or right_norm == 0.0:
        return 0.0
    return float(np.dot(left, right) / (left_norm * right_norm))


def _normalize_string_list(values: Optional[Sequence[str]]) -> Tuple[str, ...]:
    if not values:
        return ()
    return tuple(str(item).strip() for item in values if str(item).strip())


@dataclass(frozen=True)
class NamedVector:
    """单个命名向量。"""

    vector_name: str
    model_name: str
    dimension: int
    vector: np.ndarray


@dataclass(frozen=True)
class VectorPoint:
    """多模态向量点。"""

    point_id: str
    collection_name: str
    record_kind: str
    payload: Dict[str, object]
    vectors: Dict[str, NamedVector]
    updated_at: str


@dataclass(frozen=True)
class VectorSearchHit:
    """向量检索结果。"""

    point: VectorPoint
    vector_name: str
    score: float


class SQLiteNamedVectorStore:
    """基于 SQLite 的 named vectors + payload filters 存储层。"""

    backend_name = "sqlite_named_vectors"

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

    def upsert_point(
        self,
        *,
        point_id: str,
        collection_name: str,
        record_kind: str,
        payload: Dict[str, object],
        vectors: Dict[str, NamedVector],
        updated_at: Optional[str] = None,
    ) -> None:
        resolved_updated_at = updated_at or utc_now().isoformat()
        payload_json = json.dumps(payload, ensure_ascii=False)
        timestamp_ts = str(payload.get("timestamp") or resolved_updated_at)
        last_seen_ts = str(payload.get("last_seen_ts") or "")
        with self._lock:
            self._conn.execute(
                """
                INSERT INTO memory_vector_points (
                    point_id,
                    collection_name,
                    record_kind,
                    map_version_id,
                    localization_session_id,
                    linked_location_id,
                    camera_id,
                    topo_node_id,
                    semantic_region_id,
                    anchor_id,
                    instance_type,
                    movability,
                    last_seen_ts,
                    timestamp_ts,
                    payload_json,
                    updated_at
                )
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(point_id) DO UPDATE SET
                    collection_name = excluded.collection_name,
                    record_kind = excluded.record_kind,
                    map_version_id = excluded.map_version_id,
                    localization_session_id = excluded.localization_session_id,
                    linked_location_id = excluded.linked_location_id,
                    camera_id = excluded.camera_id,
                    topo_node_id = excluded.topo_node_id,
                    semantic_region_id = excluded.semantic_region_id,
                    anchor_id = excluded.anchor_id,
                    instance_type = excluded.instance_type,
                    movability = excluded.movability,
                    last_seen_ts = excluded.last_seen_ts,
                    timestamp_ts = excluded.timestamp_ts,
                    payload_json = excluded.payload_json,
                    updated_at = excluded.updated_at
                """,
                (
                    point_id,
                    collection_name,
                    record_kind,
                    self._string_or_none(payload.get("map_version_id")),
                    self._string_or_none(payload.get("localization_session_id")),
                    self._string_or_none(payload.get("linked_location_id")),
                    self._string_or_none(payload.get("camera_id")),
                    self._string_or_none(payload.get("topo_node_id")),
                    self._string_or_none(payload.get("semantic_region_id")),
                    self._string_or_none(payload.get("anchor_id")),
                    self._string_or_none(payload.get("instance_type")),
                    self._string_or_none(payload.get("movability")),
                    last_seen_ts,
                    timestamp_ts,
                    payload_json,
                    resolved_updated_at,
                ),
            )
            existing_rows = self._conn.execute(
                "SELECT vector_name FROM memory_named_vectors WHERE point_id = ?",
                (point_id,),
            ).fetchall()
            existing_names = {str(row["vector_name"]) for row in existing_rows}
            incoming_names = set(vectors.keys())
            names_to_delete = existing_names - incoming_names
            if names_to_delete:
                self._conn.executemany(
                    "DELETE FROM memory_named_vectors WHERE point_id = ? AND vector_name = ?",
                    [(point_id, vector_name) for vector_name in sorted(names_to_delete)],
                )
            for vector_name, vector in vectors.items():
                self._conn.execute(
                    """
                    INSERT INTO memory_named_vectors (
                        point_id,
                        vector_name,
                        model_name,
                        dimension,
                        vector_payload,
                        updated_at
                    )
                    VALUES (?, ?, ?, ?, ?, ?)
                    ON CONFLICT(point_id, vector_name) DO UPDATE SET
                        model_name = excluded.model_name,
                        dimension = excluded.dimension,
                        vector_payload = excluded.vector_payload,
                        updated_at = excluded.updated_at
                    """,
                    (
                        point_id,
                        vector_name,
                        vector.model_name,
                        int(vector.dimension),
                        _encode_vector(vector.vector),
                        resolved_updated_at,
                    ),
                )
            self._conn.commit()

    def get_point(self, point_id: str) -> Optional[VectorPoint]:
        with self._lock:
            point_row = self._conn.execute(
                """
                SELECT point_id, collection_name, record_kind, payload_json, updated_at
                FROM memory_vector_points
                WHERE point_id = ?
                """,
                (point_id,),
            ).fetchone()
            if point_row is None:
                return None
            vector_rows = self._conn.execute(
                """
                SELECT vector_name, model_name, dimension, vector_payload
                FROM memory_named_vectors
                WHERE point_id = ?
                """,
                (point_id,),
            ).fetchall()
        return self._build_point(point_row, vector_rows)

    def load_points(self) -> Dict[str, VectorPoint]:
        with self._lock:
            point_rows = self._conn.execute(
                """
                SELECT point_id, collection_name, record_kind, payload_json, updated_at
                FROM memory_vector_points
                ORDER BY updated_at ASC, point_id ASC
                """
            ).fetchall()
            vector_rows = self._conn.execute(
                """
                SELECT point_id, vector_name, model_name, dimension, vector_payload
                FROM memory_named_vectors
                ORDER BY point_id ASC, vector_name ASC
                """
            ).fetchall()
        grouped_vectors: Dict[str, List[sqlite3.Row]] = {}
        for row in vector_rows:
            grouped_vectors.setdefault(str(row["point_id"]), []).append(row)
        return {
            str(row["point_id"]): self._build_point(row, grouped_vectors.get(str(row["point_id"]), []))
            for row in point_rows
        }

    def query(
        self,
        *,
        vector_name: str,
        query_vector: np.ndarray,
        limit: int,
        payload_filter: Optional[MemoryPayloadFilter] = None,
        candidate_ids: Optional[Iterable[str]] = None,
        collection_names: Optional[Iterable[str]] = None,
    ) -> Tuple[VectorSearchHit, ...]:
        selected_rows = self._select_candidate_rows(
            payload_filter=payload_filter,
            candidate_ids=candidate_ids,
            collection_names=collection_names,
        )
        if not selected_rows:
            return ()
        point_ids = [str(row["point_id"]) for row in selected_rows]
        vector_rows = self._select_vector_rows(vector_name, point_ids)
        vector_map = {str(row["point_id"]): row for row in vector_rows}

        hits: List[VectorSearchHit] = []
        for point_row in selected_rows:
            point_id = str(point_row["point_id"])
            vector_row = vector_map.get(point_id)
            if vector_row is None:
                continue
            point = self._build_point(point_row, [vector_row])
            if not self._payload_matches(point.payload, payload_filter):
                continue
            stored_vector = point.vectors[vector_name].vector
            score = max(0.0, _cosine_similarity(query_vector, stored_vector))
            hits.append(VectorSearchHit(point=point, vector_name=vector_name, score=score))
        hits.sort(key=lambda item: item.score, reverse=True)
        return tuple(hits[: max(1, limit)])

    def count_points(self) -> int:
        with self._lock:
            row = self._conn.execute("SELECT COUNT(*) AS count FROM memory_vector_points").fetchone()
        return int(row["count"] if row is not None else 0)

    def count_points_by_collection(self) -> Dict[str, int]:
        with self._lock:
            rows = self._conn.execute(
                """
                SELECT collection_name, COUNT(*) AS count
                FROM memory_vector_points
                GROUP BY collection_name
                ORDER BY collection_name ASC
                """
            ).fetchall()
        return {str(row["collection_name"]): int(row["count"]) for row in rows}

    def count_vectors_by_name(self) -> Dict[str, int]:
        with self._lock:
            rows = self._conn.execute(
                """
                SELECT vector_name, COUNT(*) AS count
                FROM memory_named_vectors
                GROUP BY vector_name
                ORDER BY vector_name ASC
                """
            ).fetchall()
        return {str(row["vector_name"]): int(row["count"]) for row in rows}

    def delete_points(self, point_ids: Sequence[str]) -> int:
        normalized_ids = [str(item).strip() for item in point_ids if str(item).strip()]
        if not normalized_ids:
            return 0
        with self._lock:
            self._conn.executemany(
                "DELETE FROM memory_named_vectors WHERE point_id = ?",
                [(point_id,) for point_id in normalized_ids],
            )
            self._conn.executemany(
                "DELETE FROM memory_vector_points WHERE point_id = ?",
                [(point_id,) for point_id in normalized_ids],
            )
            self._conn.commit()
        return len(normalized_ids)

    def clear_all(self) -> None:
        with self._lock:
            self._conn.execute("DELETE FROM memory_named_vectors")
            self._conn.execute("DELETE FROM memory_vector_points")
            self._conn.commit()

    def _select_candidate_rows(
        self,
        *,
        payload_filter: Optional[MemoryPayloadFilter],
        candidate_ids: Optional[Iterable[str]],
        collection_names: Optional[Iterable[str]],
    ) -> List[sqlite3.Row]:
        query = [
            "SELECT point_id, collection_name, record_kind, payload_json, updated_at",
            "FROM memory_vector_points",
            "WHERE 1 = 1",
        ]
        params: List[object] = []

        normalized_collections = _normalize_string_list(collection_names)
        if normalized_collections:
            query.append("AND collection_name IN (%s)" % ",".join("?" for _ in normalized_collections))
            params.extend(normalized_collections)

        normalized_candidate_ids = _normalize_string_list(tuple(candidate_ids or ()))
        if normalized_candidate_ids:
            query.append("AND point_id IN (%s)" % ",".join("?" for _ in normalized_candidate_ids))
            params.extend(normalized_candidate_ids)

        if payload_filter is not None:
            self._append_sql_filters(query, params, payload_filter)

        query.append("ORDER BY updated_at DESC, point_id ASC")
        with self._lock:
            rows = self._conn.execute(" ".join(query), tuple(params)).fetchall()
        return list(rows)

    def _select_vector_rows(self, vector_name: str, point_ids: List[str]) -> List[sqlite3.Row]:
        if not point_ids:
            return []
        query = """
            SELECT point_id, vector_name, model_name, dimension, vector_payload
            FROM memory_named_vectors
            WHERE vector_name = ? AND point_id IN (%s)
            ORDER BY point_id ASC
        """ % ",".join("?" for _ in point_ids)
        params: List[object] = [vector_name]
        params.extend(point_ids)
        with self._lock:
            rows = self._conn.execute(query, tuple(params)).fetchall()
        return list(rows)

    def _append_sql_filters(
        self,
        query: List[str],
        params: List[object],
        payload_filter: MemoryPayloadFilter,
    ) -> None:
        record_kinds = _normalize_string_list([item.value if hasattr(item, "value") else str(item) for item in payload_filter.record_kinds])
        if record_kinds:
            query.append("AND record_kind IN (%s)" % ",".join("?" for _ in record_kinds))
            params.extend(record_kinds)
        if payload_filter.map_version_id:
            query.append("AND map_version_id = ?")
            params.append(payload_filter.map_version_id)
        if payload_filter.localization_session_id:
            query.append("AND localization_session_id = ?")
            params.append(payload_filter.localization_session_id)
        if payload_filter.linked_location_id:
            query.append("AND linked_location_id = ?")
            params.append(payload_filter.linked_location_id)
        camera_ids = _normalize_string_list(payload_filter.camera_ids)
        if camera_ids:
            query.append("AND camera_id IN (%s)" % ",".join("?" for _ in camera_ids))
            params.extend(camera_ids)
        topo_node_ids = _normalize_string_list(payload_filter.topo_node_ids)
        if topo_node_ids:
            query.append("AND topo_node_id IN (%s)" % ",".join("?" for _ in topo_node_ids))
            params.extend(topo_node_ids)
        anchor_ids = _normalize_string_list(payload_filter.anchor_ids)
        if anchor_ids:
            query.append("AND anchor_id IN (%s)" % ",".join("?" for _ in anchor_ids))
            params.extend(anchor_ids)
        instance_types = _normalize_string_list(payload_filter.instance_types)
        if instance_types:
            query.append("AND instance_type IN (%s)" % ",".join("?" for _ in instance_types))
            params.extend(instance_types)
        movabilities = _normalize_string_list(payload_filter.movabilities)
        if movabilities:
            query.append("AND movability IN (%s)" % ",".join("?" for _ in movabilities))
            params.extend(movabilities)
        if payload_filter.last_seen_after is not None:
            query.append("AND last_seen_ts >= ?")
            params.append(payload_filter.last_seen_after.isoformat())
        if payload_filter.last_seen_before is not None:
            query.append("AND last_seen_ts <= ?")
            params.append(payload_filter.last_seen_before.isoformat())
        if payload_filter.created_after is not None:
            query.append("AND timestamp_ts >= ?")
            params.append(payload_filter.created_after.isoformat())
        if payload_filter.created_before is not None:
            query.append("AND timestamp_ts <= ?")
            params.append(payload_filter.created_before.isoformat())

    def _build_point(self, point_row: sqlite3.Row, vector_rows: List[sqlite3.Row]) -> VectorPoint:
        payload = json.loads(str(point_row["payload_json"]))
        vectors = {
            str(row["vector_name"]): NamedVector(
                vector_name=str(row["vector_name"]),
                model_name=str(row["model_name"]),
                dimension=int(row["dimension"]),
                vector=_decode_vector(str(row["vector_payload"])),
            )
            for row in vector_rows
        }
        return VectorPoint(
            point_id=str(point_row["point_id"]),
            collection_name=str(point_row["collection_name"]),
            record_kind=str(point_row["record_kind"]),
            payload=payload,
            vectors=vectors,
            updated_at=str(point_row["updated_at"]),
        )

    def _payload_matches(
        self,
        payload: Dict[str, object],
        payload_filter: Optional[MemoryPayloadFilter],
    ) -> bool:
        if payload_filter is None:
            return True
        if payload_filter.localization_session_id:
            localization_session_id = str(payload.get("localization_session_id") or "").strip()
            if localization_session_id != str(payload_filter.localization_session_id).strip():
                return False
        if payload_filter.semantic_labels_any:
            labels = {str(item).strip().lower() for item in (payload.get("semantic_labels") or []) if str(item).strip()}
            expected = {str(item).strip().lower() for item in payload_filter.semantic_labels_any if str(item).strip()}
            if not labels.intersection(expected):
                return False
        if payload_filter.visual_labels_any:
            visual_labels = {str(item).strip().lower() for item in (payload.get("visual_labels") or []) if str(item).strip()}
            expected = {str(item).strip().lower() for item in payload_filter.visual_labels_any if str(item).strip()}
            if not visual_labels.intersection(expected):
                return False
        if payload_filter.vision_tags_any:
            vision_tags = {str(item).strip().lower() for item in (payload.get("vision_tags") or []) if str(item).strip()}
            expected = {str(item).strip().lower() for item in payload_filter.vision_tags_any if str(item).strip()}
            if not vision_tags.intersection(expected):
                return False
        if payload_filter.max_age_sec is not None:
            created_at = self._extract_timestamp(payload)
            if created_at is None:
                return False
            age_sec = (utc_now() - created_at).total_seconds()
            if age_sec > float(payload_filter.max_age_sec):
                return False
        if payload_filter.near_pose is not None and payload_filter.max_distance_m is not None:
            distance_m = self._distance_to_pose(payload, payload_filter.near_pose)
            if distance_m is None or distance_m > float(payload_filter.max_distance_m):
                return False
        return True

    def _extract_timestamp(self, payload: Dict[str, object]) -> Optional[datetime]:
        raw_value = str(payload.get("timestamp") or "").strip()
        if not raw_value:
            return None
        try:
            return datetime.fromisoformat(raw_value)
        except ValueError:
            return None

    def _distance_to_pose(self, payload: Dict[str, object], pose: Pose) -> Optional[float]:
        if payload.get("pose_x") is None or payload.get("pose_y") is None:
            return None
        try:
            point_x = float(payload.get("pose_x"))
            point_y = float(payload.get("pose_y"))
            point_z = float(payload.get("pose_z") or 0.0)
        except (TypeError, ValueError):
            return None
        delta_x = point_x - float(pose.position.x)
        delta_y = point_y - float(pose.position.y)
        delta_z = point_z - float(pose.position.z)
        return float(np.sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z))

    def _init_schema(self) -> None:
        with self._lock:
            self._conn.execute(
                """
                CREATE TABLE IF NOT EXISTS memory_vector_points (
                    point_id TEXT PRIMARY KEY,
                    collection_name TEXT NOT NULL DEFAULT 'legacy',
                    record_kind TEXT NOT NULL,
                    map_version_id TEXT,
                    localization_session_id TEXT,
                    linked_location_id TEXT,
                    camera_id TEXT,
                    topo_node_id TEXT,
                    semantic_region_id TEXT,
                    anchor_id TEXT,
                    instance_type TEXT,
                    movability TEXT,
                    last_seen_ts TEXT,
                    timestamp_ts TEXT,
                    payload_json TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                )
                """
            )
            self._ensure_point_columns()
            self._conn.execute(
                """
                CREATE TABLE IF NOT EXISTS memory_named_vectors (
                    point_id TEXT NOT NULL,
                    vector_name TEXT NOT NULL,
                    model_name TEXT NOT NULL,
                    dimension INTEGER NOT NULL,
                    vector_payload TEXT NOT NULL,
                    updated_at TEXT NOT NULL,
                    PRIMARY KEY(point_id, vector_name)
                )
                """
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_memory_vector_points_collection ON memory_vector_points(collection_name)"
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_memory_vector_points_record_kind ON memory_vector_points(record_kind)"
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_memory_vector_points_map_version ON memory_vector_points(map_version_id)"
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_memory_vector_points_localization_session ON memory_vector_points(localization_session_id)"
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_memory_vector_points_topo_node ON memory_vector_points(topo_node_id)"
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_memory_vector_points_semantic_region ON memory_vector_points(semantic_region_id)"
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_memory_vector_points_anchor ON memory_vector_points(anchor_id)"
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_memory_vector_points_instance_type ON memory_vector_points(instance_type)"
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_memory_vector_points_movability ON memory_vector_points(movability)"
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_memory_vector_points_last_seen ON memory_vector_points(last_seen_ts)"
            )
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_memory_named_vectors_name ON memory_named_vectors(vector_name)"
            )
            self._conn.commit()

    def _ensure_point_columns(self) -> None:
        rows = self._conn.execute("PRAGMA table_info(memory_vector_points)").fetchall()
        existing = {str(row["name"]) for row in rows}
        required_columns = {
            "collection_name": "TEXT NOT NULL DEFAULT 'legacy'",
            "map_version_id": "TEXT",
            "localization_session_id": "TEXT",
            "linked_location_id": "TEXT",
            "camera_id": "TEXT",
            "topo_node_id": "TEXT",
            "semantic_region_id": "TEXT",
            "anchor_id": "TEXT",
            "instance_type": "TEXT",
            "movability": "TEXT",
            "last_seen_ts": "TEXT",
            "timestamp_ts": "TEXT",
        }
        for column_name, ddl in required_columns.items():
            if column_name in existing:
                continue
            self._conn.execute("ALTER TABLE memory_vector_points ADD COLUMN %s %s" % (column_name, ddl))

    def _string_or_none(self, value: object) -> Optional[str]:
        normalized = str(value or "").strip()
        return normalized or None

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            return
