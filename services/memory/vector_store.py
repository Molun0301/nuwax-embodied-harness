from __future__ import annotations

import json
import sqlite3
import threading
from contextlib import contextmanager
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, Iterable, Iterator, List, Optional, Sequence, Tuple
from uuid import uuid4

import numpy as np

from contracts.base import utc_now
from contracts.geometry import Pose
from contracts.memory import MemoryPayloadFilter
from services.memory.sqlite_storage import SQLiteStorage, SQLiteTransaction

TEXT_VECTOR_NAME = "text_dense"
IMAGE_VECTOR_NAME = "image_dense"

PLACE_NODES_COLLECTION = "place_nodes"
OBJECT_INSTANCES_COLLECTION = "object_instances"
EPISODIC_OBSERVATIONS_COLLECTION = "episodic_observations"


def _encode_vector(vector: np.ndarray) -> bytes:
    """将向量编码为 float32 二进制 BLOB，比 JSON 快 5-10x 且节省约 40% 空间。"""
    return vector.astype(np.float32).tobytes()


def _decode_vector(payload: object) -> np.ndarray:
    """解码向量，支持新 BLOB 格式和旧 JSON 格式（向后兼容）。"""
    if isinstance(payload, (bytes, bytearray, memoryview)):
        return np.frombuffer(payload, dtype=np.float32).copy()
    # 旧格式：JSON 字符串，自动 fallback
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
    storage_revision: Optional[str] = None


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

    def __init__(self, db_path: str = ":memory:", *, storage: Optional[SQLiteStorage] = None, preserve_legacy: bool = False) -> None:
        self._preserve_legacy = bool(preserve_legacy)
        self._owns_storage = storage is None
        self._storage = storage if storage is not None else SQLiteStorage(db_path)
        self._db_path = self._storage.db_path
        self._closed = False
        self._cache_lock = threading.RLock()
        # One immutable, versioned entry per point/vector. Tokens describe stored
        # bytes only, not model/input provenance or service indexing readiness.
        self._cached_vectors: Dict[str, Dict[str, Tuple[str, np.ndarray]]] = {}
        self._cache_threshold: int = 50
        try:
            self._init_schema()
        except BaseException:
            self.close()
            raise

    @property
    def db_path(self) -> str:
        return self._db_path

    @property
    def storage(self) -> SQLiteStorage:
        return self._storage

    @contextmanager
    def _transaction(self) -> Iterator[SQLiteTransaction]:
        if self._closed:
            raise RuntimeError("SQLite vector store is closed")
        with self._storage.transaction() as tx:
            if self._closed:
                raise RuntimeError("SQLite vector store is closed")
            yield tx

    @contextmanager
    def _read_snapshot(self) -> Iterator[SQLiteTransaction]:
        if self._closed:
            raise RuntimeError("SQLite vector store is closed")
        with self._storage.read_snapshot() as tx:
            if self._closed:
                raise RuntimeError("SQLite vector store is closed")
            yield tx

    def close(self) -> None:
        if self._closed:
            return
        if self._owns_storage:
            self._storage.close()
        self._closed = True
        with self._cache_lock:
            self._cached_vectors.clear()

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
        payload = dict(payload)
        payload.pop("map_version_id", None)
        resolved_map_name = str(payload.get("map_name") or "").strip()
        if not resolved_map_name:
            library_name = str(payload.get("memory_library_name") or "").strip()
            metadata = payload.get("metadata")
            if not library_name and isinstance(metadata, dict):
                library_name = str(metadata.get("memory_library_name") or "").strip()
            if library_name and library_name != "legacy_default":
                resolved_map_name = library_name
        payload["map_name"] = resolved_map_name or None
        payload_json = json.dumps(payload, ensure_ascii=False)
        timestamp_ts = str(payload.get("timestamp") or resolved_updated_at)
        last_seen_ts = str(payload.get("last_seen_ts") or "")
        encoded_vectors = {name: (named, _encode_vector(named.vector), uuid4().hex) for name, named in vectors.items()}
        point_values = (
            point_id,
            collection_name,
            record_kind,
            self._string_or_none(payload.get("map_name")),
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
        )
        with self._transaction() as tx:
            self._upsert_point_tx(tx, point_id, point_values, encoded_vectors, resolved_updated_at)

    def _upsert_point_tx(
        self,
        tx: SQLiteTransaction,
        point_id: str,
        point_values: Tuple[object, ...],
        encoded_vectors: Dict[str, Tuple[NamedVector, bytes, str]],
        updated_at: str,
    ) -> None:
        """SQL-only full replacement; payload/vector encoding precedes the lock."""
        tx.execute(
            """
            INSERT INTO memory_vector_points (
                point_id,
                collection_name,
                record_kind,
                map_name,
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
                map_name = excluded.map_name,
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
            point_values,
        )
        existing_rows = tx.execute(
            "SELECT vector_name FROM memory_named_vectors WHERE point_id = ?",
            (point_id,),
        ).fetchall()
        existing_names = {str(row["vector_name"]) for row in existing_rows}
        incoming_names = set(encoded_vectors)
        names_to_delete = existing_names - incoming_names
        if names_to_delete:
            tx.executemany(
                "DELETE FROM memory_named_vectors WHERE point_id = ? AND vector_name = ?",
                [(point_id, vector_name) for vector_name in sorted(names_to_delete)],
            )
        for vector_name, (vector, encoded, cache_token) in encoded_vectors.items():
            tx.execute(
                """
                INSERT INTO memory_named_vectors (
                    point_id,
                    vector_name,
                    model_name,
                    dimension,
                    vector_payload,
                    updated_at,
                    cache_token
                )
                VALUES (?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(point_id, vector_name) DO UPDATE SET
                    model_name = excluded.model_name,
                    dimension = excluded.dimension,
                    vector_payload = excluded.vector_payload,
                    updated_at = excluded.updated_at,
                    cache_token = excluded.cache_token
                """,
                (
                    point_id,
                    vector_name,
                    vector.model_name,
                    int(vector.dimension),
                    encoded,
                    updated_at,
                    cache_token,
                ),
            )
        # Invalidation is safe even if this savepoint later rolls back.
        # No uncommitted replacement is ever installed in the shared cache.
        self._invalidate_cache((point_id,))

    def _invalidate_cache(self, point_ids: Optional[Iterable[str]] = None) -> None:
        with self._cache_lock:
            if point_ids is None:
                self._cached_vectors.clear()
            else:
                for point_id in point_ids:
                    self._cached_vectors.pop(point_id, None)

    def _snapshot_vector(self, row: sqlite3.Row, *, cache_allowed: bool) -> np.ndarray:
        """Resolve bytes selected by this read snapshot, never by point ID alone.

        Old rows have no token and stay readable without a synchronous rewrite.
        Writer reads bypass the cache so provisional bytes cannot outlive rollback.
        A late reader can at worst replace a newer cache entry with an older token:
        every later read validates the token again against its own selected row.
        """
        token = row["cache_token"]
        raw = row["vector_payload"]
        point_id, name = str(row["point_id"]), str(row["vector_name"])
        if cache_allowed and token is not None:
            with self._cache_lock:
                cached = self._cached_vectors.get(point_id, {}).get(name)
                # 旧版写入者可能保留新列 token；同时校验快照中的实际字节。
                if cached is not None and cached[0] == token and raw == cached[1].base:
                    return cached[1]
        # SQLite BLOB results are immutable owned bytes. JSON legacy rows use the
        # same bytes-backed representation after decoding, without changing SQL.
        owned_bytes = raw if isinstance(raw, bytes) else _encode_vector(_decode_vector(raw))
        vector = np.frombuffer(owned_bytes, dtype=np.float32)
        if cache_allowed and token is not None:
            with self._cache_lock:
                self._cached_vectors.setdefault(point_id, {})[name] = (str(token), vector)
        return vector

    def get_point(self, point_id: str) -> Optional[VectorPoint]:
        with self._read_snapshot() as tx:
            point_row = tx.execute(
                """
                SELECT point_id, collection_name, record_kind, map_name, payload_json, updated_at
                FROM memory_vector_points
                WHERE point_id = ?
                """,
                (point_id,),
            ).fetchone()
            if point_row is None:
                return None
            vector_rows = tx.execute(
                """
                SELECT vector_name, model_name, dimension, vector_payload, cache_token
                FROM memory_named_vectors
                WHERE point_id = ?
                """,
                (point_id,),
            ).fetchall()
        return self._build_point(point_row, vector_rows)

    def get_points_batch(self, point_ids: Iterable[str]) -> Dict[str, VectorPoint]:
        """批量获取多个向量点，减少逐条查询的开销。"""
        id_list = list({str(pid).strip() for pid in point_ids if str(pid).strip()})
        if not id_list:
            return {}
        result: Dict[str, VectorPoint] = {}
        batch_size = 200
        with self._read_snapshot() as tx:
            for i in range(0, len(id_list), batch_size):
                batch = id_list[i : i + batch_size]
                placeholders = ",".join("?" * len(batch))
                point_rows = tx.execute(
                    f"SELECT point_id, collection_name, record_kind, map_name, payload_json, updated_at "
                    f"FROM memory_vector_points WHERE point_id IN ({placeholders})",
                    batch,
                ).fetchall()
                vector_rows = tx.execute(
                    f"SELECT point_id, vector_name, model_name, dimension, vector_payload, cache_token "
                    f"FROM memory_named_vectors WHERE point_id IN ({placeholders})",
                    batch,
                ).fetchall()
                grouped_vectors: Dict[str, List[sqlite3.Row]] = {}
                for vrow in vector_rows:
                    grouped_vectors.setdefault(str(vrow["point_id"]), []).append(vrow)
                for row in point_rows:
                    pid = str(row["point_id"])
                    point = self._build_point(row, grouped_vectors.get(pid, []))
                    result[pid] = point
        return result

    def load_points(self) -> Dict[str, VectorPoint]:
        with self._read_snapshot() as tx:
            point_rows = tx.execute(
                """
                SELECT point_id, collection_name, record_kind, map_name, payload_json, updated_at
                FROM memory_vector_points
                ORDER BY updated_at ASC, point_id ASC
                """
            ).fetchall()
            vector_rows = tx.execute(
                """
                SELECT point_id, vector_name, model_name, dimension, vector_payload, cache_token
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
        model_name: Optional[str] = None,
    ) -> Tuple[VectorSearchHit, ...]:
        query_vector = np.asarray(query_vector, dtype=np.float32).reshape(-1)
        if not query_vector.size or not np.all(np.isfinite(query_vector)):
            return ()
        # 一次快照捕获载荷和向量行，筛选、解码及评分均在 SQL 锁外进行。
        # 先做载荷筛选，避免不相关的旧损坏向量影响已筛选的查询。
        cache_allowed = self._storage.current_transaction is None
        with self._read_snapshot() as tx:
            selected_rows = self._select_candidate_rows(
                tx,
                payload_filter=payload_filter,
                candidate_ids=candidate_ids,
                collection_names=collection_names,
            )
            if not selected_rows:
                return ()
            point_ids = [str(row["point_id"]) for row in selected_rows]
            vector_rows = self._select_vector_rows(tx, vector_name, point_ids)
            vector_map = {
                str(row["point_id"]): row
                for row in vector_rows
                if int(row["dimension"]) == query_vector.size
                and (model_name is None or str(row["model_name"]) == model_name)
            }

        rows_by_id = {}
        vectors_by_id = {}
        for point_row in selected_rows:
            point_id = str(point_row["point_id"])
            vector_row = vector_map.get(point_id)
            if vector_row is None:
                continue
            if payload_filter is not None:
                payload = json.loads(str(point_row["payload_json"]))
                if not self._payload_matches(payload, payload_filter):
                    continue
            vector = self._snapshot_vector(vector_row, cache_allowed=cache_allowed)
            if vector.size != query_vector.size or not np.all(np.isfinite(vector)):
                continue
            rows_by_id[point_id] = point_row
            vectors_by_id[point_id] = vector
        if not vectors_by_id:
            return ()

        ordered_ids = list(vectors_by_id)
        if len(ordered_ids) > self._cache_threshold:
            matrix = np.stack(list(vectors_by_id.values()))
            scores = self._batch_cosine_similarity(query_vector, matrix)
        else:
            scores = [_cosine_similarity(query_vector, vector) for vector in vectors_by_id.values()]
        ranked = sorted(
            ((point_id, max(0.0, float(score))) for point_id, score in zip(ordered_ids, scores)),
            key=lambda item: item[1],
            reverse=True,
        )
        # 只为最终命中构造对象，避免全部候选重复解码向量和载荷。
        return tuple(
            VectorSearchHit(
                point=self._build_point(rows_by_id[point_id], [vector_map[point_id]]),
                vector_name=vector_name,
                score=score,
            )
            for point_id, score in ranked[: max(1, limit)]
        )

    def count_points(self) -> int:
        with self._read_snapshot() as tx:
            row = tx.execute("SELECT COUNT(*) AS count FROM memory_vector_points").fetchone()
        return int(row["count"] if row is not None else 0)

    def count_points_by_collection(self) -> Dict[str, int]:
        with self._read_snapshot() as tx:
            rows = tx.execute(
                """
                SELECT collection_name, COUNT(*) AS count
                FROM memory_vector_points
                GROUP BY collection_name
                ORDER BY collection_name ASC
                """
            ).fetchall()
        return {str(row["collection_name"]): int(row["count"]) for row in rows}

    def count_vectors_by_name(self) -> Dict[str, int]:
        with self._read_snapshot() as tx:
            rows = tx.execute(
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
        with self._transaction() as tx:
            tx.executemany(
                "DELETE FROM memory_named_vectors WHERE point_id = ?",
                [(point_id,) for point_id in normalized_ids],
            )
            tx.executemany(
                "DELETE FROM memory_vector_points WHERE point_id = ?",
                [(point_id,) for point_id in normalized_ids],
            )
            self._invalidate_cache(normalized_ids)
        return len(normalized_ids)

    def clear_all(self) -> None:
        with self._transaction() as tx:
            tx.execute("DELETE FROM memory_named_vectors")
            tx.execute("DELETE FROM memory_vector_points")
            self._invalidate_cache()

    def _batch_cosine_similarity(self, query_vector: np.ndarray, matrix: np.ndarray) -> np.ndarray:
        """批量计算 query_vector 与矩阵每行的余弦相似度，利用 numpy BLAS 加速。"""
        query_norm = query_vector / (np.linalg.norm(query_vector) + 1e-10)
        norms = np.linalg.norm(matrix, axis=1, keepdims=True) + 1e-10
        matrix_norm = matrix / norms
        return matrix_norm @ query_norm

    def _select_candidate_rows(
        self,
        tx: SQLiteTransaction,
        *,
        payload_filter: Optional[MemoryPayloadFilter],
        candidate_ids: Optional[Iterable[str]],
        collection_names: Optional[Iterable[str]],
    ) -> List[sqlite3.Row]:
        query = [
            "SELECT point_id, collection_name, record_kind, map_name, payload_json, updated_at",
            "FROM memory_vector_points",
            "WHERE 1 = 1",
        ]
        params: List[object] = []

        normalized_collections = _normalize_string_list(collection_names)
        if normalized_collections:
            query.append("AND collection_name IN ({})".format(",".join("?" for _ in normalized_collections)))
            params.extend(normalized_collections)

        normalized_candidate_ids = _normalize_string_list(tuple(candidate_ids) if candidate_ids is not None else ())
        if candidate_ids is not None and not normalized_candidate_ids:
            return []
        if normalized_candidate_ids:
            query.append("AND point_id IN ({})".format(",".join("?" for _ in normalized_candidate_ids)))
            params.extend(normalized_candidate_ids)

        if payload_filter is not None:
            self._append_sql_filters(query, params, payload_filter)

        query.append("ORDER BY updated_at DESC, point_id ASC")
        rows = tx.execute(" ".join(query), tuple(params)).fetchall()
        return list(rows)

    def _select_vector_rows(self, tx: SQLiteTransaction, vector_name: str, point_ids: List[str]) -> List[sqlite3.Row]:
        if not point_ids:
            return []
        query = """
            SELECT point_id, vector_name, model_name, dimension, vector_payload, cache_token
            FROM memory_named_vectors
            WHERE vector_name = ? AND point_id IN ({})
            ORDER BY point_id ASC
        """.format(",".join("?" for _ in point_ids))
        params: List[object] = [vector_name]
        params.extend(point_ids)
        rows = tx.execute(query, tuple(params)).fetchall()
        return list(rows)

    def _append_sql_filters(
        self,
        query: List[str],
        params: List[object],
        payload_filter: MemoryPayloadFilter,
    ) -> None:
        record_kinds = _normalize_string_list(
            [item.value if hasattr(item, "value") else str(item) for item in payload_filter.record_kinds]
        )
        if record_kinds:
            query.append("AND record_kind IN ({})".format(",".join("?" for _ in record_kinds)))
            params.extend(record_kinds)
        if payload_filter.map_name:
            query.append("AND map_name = ?")
            params.append(payload_filter.map_name)
        if payload_filter.localization_session_id:
            query.append("AND localization_session_id = ?")
            params.append(payload_filter.localization_session_id)
        if payload_filter.linked_location_id:
            query.append("AND linked_location_id = ?")
            params.append(payload_filter.linked_location_id)
        camera_ids = _normalize_string_list(payload_filter.camera_ids)
        if camera_ids:
            query.append("AND camera_id IN ({})".format(",".join("?" for _ in camera_ids)))
            params.extend(camera_ids)
        topo_node_ids = _normalize_string_list(payload_filter.topo_node_ids)
        if topo_node_ids:
            query.append("AND topo_node_id IN ({})".format(",".join("?" for _ in topo_node_ids)))
            params.extend(topo_node_ids)
        anchor_ids = _normalize_string_list(payload_filter.anchor_ids)
        if anchor_ids:
            query.append("AND anchor_id IN ({})".format(",".join("?" for _ in anchor_ids)))
            params.extend(anchor_ids)
        instance_types = _normalize_string_list(payload_filter.instance_types)
        if instance_types:
            query.append("AND instance_type IN ({})".format(",".join("?" for _ in instance_types)))
            params.extend(instance_types)
        movabilities = _normalize_string_list(payload_filter.movabilities)
        if movabilities:
            query.append("AND movability IN ({})".format(",".join("?" for _ in movabilities)))
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
        payload.pop("map_version_id", None)
        resolved_map_name = str(payload.get("map_name") or point_row["map_name"] or "").strip()
        payload["map_name"] = resolved_map_name or None
        vectors = {
            str(row["vector_name"]): NamedVector(
                vector_name=str(row["vector_name"]),
                model_name=str(row["model_name"]),
                dimension=int(row["dimension"]),
                vector=_decode_vector(row["vector_payload"]),
                storage_revision=row["cache_token"],
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
        if payload_filter.map_name:
            map_name = str(payload.get("map_name") or "").strip()
            if map_name != str(payload_filter.map_name).strip():
                return False
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
            visual_labels = {
                str(item).strip().lower() for item in (payload.get("visual_labels") or []) if str(item).strip()
            }
            expected = {str(item).strip().lower() for item in payload_filter.visual_labels_any if str(item).strip()}
            if not visual_labels.intersection(expected):
                return False
        if payload_filter.vision_tags_any:
            vision_tags = {
                str(item).strip().lower() for item in (payload.get("vision_tags") or []) if str(item).strip()
            }
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
        with self._transaction() as tx:
            tx.execute(
                """
                CREATE TABLE IF NOT EXISTS memory_vector_points (
                    point_id TEXT PRIMARY KEY,
                    collection_name TEXT NOT NULL DEFAULT 'legacy',
                    record_kind TEXT NOT NULL,
                    map_name TEXT,
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
            if not self._preserve_legacy:
                self._migrate_point_map_names()
            tx.execute(
                """
                CREATE TABLE IF NOT EXISTS memory_named_vectors (
                    point_id TEXT NOT NULL,
                    vector_name TEXT NOT NULL,
                    model_name TEXT NOT NULL,
                    dimension INTEGER NOT NULL,
                    vector_payload BLOB NOT NULL,
                    updated_at TEXT NOT NULL,
                    cache_token TEXT,
                    PRIMARY KEY(point_id, vector_name)
                )
                """
            )
            vector_columns = {
                str(row["name"]) for row in tx.execute("PRAGMA table_info(memory_named_vectors)").fetchall()
            }
            if "cache_token" not in vector_columns:
                tx.execute("ALTER TABLE memory_named_vectors ADD COLUMN cache_token TEXT")
            # 所有旧写入路径更新内容而未更换版本时撤销旧证明，不扫描或改写历史行。
            tx.execute("""CREATE TRIGGER IF NOT EXISTS memory_vector_revision_invalidate
                AFTER UPDATE OF vector_payload,model_name,dimension ON memory_named_vectors
                WHEN NEW.cache_token IS OLD.cache_token AND
                     (NEW.vector_payload IS NOT OLD.vector_payload OR NEW.model_name IS NOT OLD.model_name
                      OR NEW.dimension IS NOT OLD.dimension)
                BEGIN
                    UPDATE memory_named_vectors SET cache_token=NULL
                    WHERE point_id=NEW.point_id AND vector_name=NEW.vector_name;
                END""")
            # Legacy JSON and BLOB rows remain byte-for-byte intact. Decode both
            # formats lazily; cache tokens are allocated only on actual writes.
            tx.execute(
                "CREATE INDEX IF NOT EXISTS idx_memory_vector_points_collection ON memory_vector_points(collection_name)"
            )
            tx.execute(
                "CREATE INDEX IF NOT EXISTS idx_memory_vector_points_record_kind ON memory_vector_points(record_kind)"
            )
            tx.execute("CREATE INDEX IF NOT EXISTS idx_memory_vector_points_map_name ON memory_vector_points(map_name)")
            tx.execute(
                "CREATE INDEX IF NOT EXISTS idx_memory_vector_points_localization_session ON memory_vector_points(localization_session_id)"
            )
            tx.execute(
                "CREATE INDEX IF NOT EXISTS idx_memory_vector_points_topo_node ON memory_vector_points(topo_node_id)"
            )
            tx.execute(
                "CREATE INDEX IF NOT EXISTS idx_memory_vector_points_semantic_region ON memory_vector_points(semantic_region_id)"
            )
            tx.execute("CREATE INDEX IF NOT EXISTS idx_memory_vector_points_anchor ON memory_vector_points(anchor_id)")
            tx.execute(
                "CREATE INDEX IF NOT EXISTS idx_memory_vector_points_instance_type ON memory_vector_points(instance_type)"
            )
            tx.execute(
                "CREATE INDEX IF NOT EXISTS idx_memory_vector_points_movability ON memory_vector_points(movability)"
            )
            tx.execute(
                "CREATE INDEX IF NOT EXISTS idx_memory_vector_points_last_seen ON memory_vector_points(last_seen_ts)"
            )
            tx.execute("CREATE INDEX IF NOT EXISTS idx_memory_named_vectors_name ON memory_named_vectors(vector_name)")

    def _ensure_point_columns(self) -> None:
        with self._transaction() as tx:
            rows = tx.execute("PRAGMA table_info(memory_vector_points)").fetchall()
            existing = {str(row["name"]) for row in rows}
            required_columns = {
                "collection_name": "TEXT NOT NULL DEFAULT 'legacy'",
                "map_name": "TEXT",
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
                tx.execute(f"ALTER TABLE memory_vector_points ADD COLUMN {column_name} {ddl}")

    def _migrate_point_map_names(self) -> None:
        """分页规范化旧地图字段；不重写已规范化的载荷或命名向量。"""
        with self._transaction() as tx:
            last_rowid = None
            while True:
                query = "SELECT rowid, point_id, map_name, payload_json FROM memory_vector_points"
                params: Tuple[object, ...] = ()
                if last_rowid is not None:
                    query += " WHERE rowid > ?"
                    params = (last_rowid,)
                rows = tx.execute(query + " ORDER BY rowid LIMIT 500", params).fetchall()
                if not rows:
                    break
                last_rowid = rows[-1]["rowid"]
                updates = []
                for row in rows:
                    try:
                        payload = json.loads(str(row["payload_json"]))
                    except (TypeError, ValueError):
                        continue
                    if not isinstance(payload, dict):
                        continue
                    original = dict(payload)
                    payload.pop("map_version_id", None)
                    resolved_map_name = str(payload.get("map_name") or row["map_name"] or "").strip()
                    if not resolved_map_name:
                        library_name = str(payload.get("memory_library_name") or "").strip()
                        metadata = payload.get("metadata")
                        if not library_name and isinstance(metadata, dict):
                            library_name = str(metadata.get("memory_library_name") or "").strip()
                        if library_name and library_name != "legacy_default":
                            resolved_map_name = library_name
                    payload["map_name"] = resolved_map_name or None
                    if payload == original and row["map_name"] == payload["map_name"]:
                        continue
                    updates.append((
                        payload["map_name"],
                        json.dumps(payload, ensure_ascii=False) if payload != original else row["payload_json"],
                        str(row["point_id"]),
                    ))
                if updates:
                    tx.executemany(
                        "UPDATE memory_vector_points SET map_name = ?, payload_json = ? WHERE point_id = ?",
                        updates,
                    )

    def _migrate_vector_payload_to_blob(self) -> None:
        """将旧 JSON 格式的 vector_payload 迁移为 BLOB 格式（仅处理仍为 TEXT 的行）。"""
        with self._transaction() as tx:
            rows = tx.execute("SELECT rowid, vector_payload FROM memory_named_vectors").fetchall()
            updates = []
            for row in rows:
                payload = row["vector_payload"]
                if isinstance(payload, (bytes, bytearray, memoryview)):
                    continue  # 已是 BLOB，跳过
                try:
                    blob = np.asarray(json.loads(payload), dtype=np.float32).tobytes()
                    updates.append((blob, row["rowid"]))
                except (TypeError, ValueError, OverflowError):
                    continue  # 无法解析的行保持原样
            if updates:
                tx.executemany(
                    "UPDATE memory_named_vectors SET vector_payload = ? WHERE rowid = ?",
                    updates,
                )

    def _string_or_none(self, value: object) -> Optional[str]:
        normalized = str(value or "").strip()
        return normalized or None

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:  # noqa: BLE001 - best-effort destructor, including partial initialization
            return
