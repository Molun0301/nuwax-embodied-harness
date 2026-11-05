"""每库一个 SQLite 文件；所有写入由记忆服务的唯一写线程调用。"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sqlite3
import json
from urllib.parse import quote
from types import MappingProxyType

import numpy as np

from contracts.memory import SemanticMemoryEntry, TaggedLocation
from contracts.spatial_memory import (
    NavigationTrajectoryEvent, ObservationEvent, SceneSpace, SemanticGraphRelation,
    SemanticInstance, SemanticSummaryNode,
)

MODELS = {
    "location": (TaggedLocation, "location_id", "tagged_locations"),
    "memory": (SemanticMemoryEntry, "memory_id", "semantic_memories"),
    "instance": (SemanticInstance, "instance_id", "semantic_instances"),
    "observation": (ObservationEvent, "event_id", "observation_events"),
    "scene_space": (SceneSpace, "scene_space_id", "scene_spaces"),
    "trajectory": (NavigationTrajectoryEvent, "trajectory_event_id", "navigation_trajectory_events"),
    "relation": (SemanticGraphRelation, "relation_id", "semantic_graph_relations"),
    "summary": (SemanticSummaryNode, "summary_id", "semantic_summary_nodes"),
}


@dataclass(frozen=True)
class EmbeddingVector:
    model: str
    dimension: int
    data: bytes

    def array(self):
        return np.frombuffer(self.data, dtype="<f4")


@dataclass(frozen=True)
class MemorySnapshot:
    name: str | None
    directory: Path | None
    records: object
    vectors: object

    @classmethod
    def empty(cls):
        return cls(None, None, MappingProxyType({kind: () for kind in MODELS}), MappingProxyType({}))

    def read(self, kind):
        model = MODELS[kind][0]
        return tuple(model.model_validate_json(value) for value in self.records[kind])

    @classmethod
    def from_directory(cls, directory, *, name):
        directory = Path(directory)
        if not (directory / "memory.db").is_file():
            return cls.empty()
        connection = sqlite3.connect((directory / "memory.db").resolve().as_uri()+"?mode=ro", uri=True)
        try:
            with connection:
                connection.execute("BEGIN")
                records = {kind: tuple(row[0] for row in connection.execute(
                    f"SELECT payload FROM {table} ORDER BY rowid")) for kind, (_, _, table) in MODELS.items()}
                vectors = {(kind, identity, vector_name): EmbeddingVector(model, dimension, bytes(data))
                           for kind, identity, vector_name, model, dimension, data in connection.execute(
                               "SELECT kind,record_id,name,model,dimension,data FROM vectors")}
            return cls(name, directory, MappingProxyType(records), MappingProxyType(vectors))
        finally:
            connection.close()


class MemoryStore:
    def __init__(self, directory, *, name):
        self.name = name
        self.changed = set()
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.photos = self.directory / "photos"
        self.photos.mkdir(exist_ok=True)
        self.connection = sqlite3.connect(self.directory / "memory.db")
        for kind, (_, _, table) in MODELS.items():
            extra = (", relation_kind TEXT, source_node_kind TEXT, source_node_id TEXT, target_node_kind TEXT, target_node_id TEXT"
                     if kind == "relation" else ", summary_kind TEXT, label TEXT, scene_space_id TEXT, semantic_region_id TEXT"
                     if kind == "summary" else "")
            self.connection.execute(f"CREATE TABLE IF NOT EXISTS {table} (id TEXT PRIMARY KEY, payload TEXT NOT NULL{extra})")
        self.connection.execute("CREATE INDEX IF NOT EXISTS relation_business_key ON semantic_graph_relations(relation_kind,source_node_kind,source_node_id,target_node_kind,target_node_id)")
        self.connection.execute("CREATE INDEX IF NOT EXISTS summary_business_key ON semantic_summary_nodes(summary_kind,label,scene_space_id,semantic_region_id)")
        self.connection.execute("""CREATE TABLE IF NOT EXISTS vectors (
            kind TEXT NOT NULL, record_id TEXT NOT NULL, name TEXT NOT NULL,
            model TEXT NOT NULL, dimension INTEGER NOT NULL, data BLOB NOT NULL,
            PRIMARY KEY(kind,record_id,name))""")
        self.connection.commit()
        self._instances = {item.instance_id: item for item in self.records('instance')}

    def put(self, kind, record):
        _, identity, table = MODELS[kind]
        fields = self._business_key_fields(kind, record)
        if fields:
            where = " AND ".join(f"{field}=?" for field, _ in fields)
            previous = self.connection.execute(f"SELECT id FROM {table} WHERE {where} ORDER BY rowid LIMIT 1",
                                               tuple(value for _, value in fields)).fetchone()
            if previous is not None:
                record = record.model_copy(update={identity: previous[0]})
        columns = (("relation_kind", "source_node_kind", "source_node_id", "target_node_kind", "target_node_id")
                   if kind == "relation" else ("summary_kind", "label", "scene_space_id", "semantic_region_id")
                   if kind == "summary" else ())
        values = [getattr(record, identity), record.model_dump_json(),
                  *((getattr(record, field).value if hasattr(getattr(record, field), "value")
                     else getattr(record, field) or "") for field in columns)]
        self.connection.execute(f"INSERT OR REPLACE INTO {table}(id,payload{''.join(','+name for name in columns)}) "
                                f"VALUES ({','.join('?' for _ in values)})", tuple(values))
        self.connection.execute('DELETE FROM vectors WHERE kind=? AND record_id=?', (kind, getattr(record, identity)))
        self.changed.add((kind, getattr(record, identity)))
        if kind == "instance":
            self._instances[record.instance_id] = record
        return record

    @staticmethod
    def _business_key_fields(kind, record):
        if kind == "relation":
            return tuple((field, getattr(record, field).value if field == "relation_kind" else getattr(record, field))
                         for field in ("relation_kind", "source_node_kind", "source_node_id", "target_node_kind", "target_node_id"))
        if kind == "summary":
            fields = [("summary_kind", record.summary_kind.value), ("label", record.label or "")]
            if record.summary_kind.value != "target_category_cluster":
                fields.append(("scene_space_id", record.scene_space_id or ""))
            if record.summary_kind.value == "region":
                fields.append(("semantic_region_id", record.semantic_region_id or ""))
            return tuple(fields)
        return ()

    def delete(self, kind, identity):
        table = MODELS[kind][2]
        self.connection.execute(f"DELETE FROM {table} WHERE id=?", (identity,))
        if kind == 'instance':
            self._instances.pop(identity, None)
        self.connection.execute("DELETE FROM vectors WHERE kind=? AND record_id=?", (kind, identity))

    def has(self, kind, identity):
        table = MODELS[kind][2]
        return self.connection.execute(f"SELECT 1 FROM {table} WHERE id=?", (identity,)).fetchone() is not None

    def records(self, kind):
        if kind == "instance" and hasattr(self, "_instances"):
            return tuple(item.model_copy(deep=True) for item in self._instances.values())
        model, _, table = MODELS[kind]
        return tuple(model.model_validate_json(row[0]) for row in
                     self.connection.execute(f"SELECT payload FROM {table} ORDER BY rowid"))

    def put_vector(self, kind, identity, name, model, vector):
        data = np.asarray(vector, dtype="<f4")
        self.connection.execute("INSERT OR REPLACE INTO vectors VALUES (?,?,?,?,?,?)",
                                (kind, identity, name, model, data.size, data.tobytes()))

    def put_live_instance(self, record, *, persist):
        self._instances[record.instance_id] = record
        if persist:
            self.put('instance', record)
        else:
            self.connection.execute('DELETE FROM semantic_instances WHERE id=?', (record.instance_id,))
            self.connection.execute('DELETE FROM vectors WHERE kind=? AND record_id=?', ('instance', record.instance_id))

    def put_photo(self, artifact_id, content):
        (self.photos / artifact_id).write_bytes(content)

    def snapshot(self, name):
        records = {}
        for kind, (_, _, table) in MODELS.items():
            records[kind] = tuple(row[0] for row in self.connection.execute(
                f"SELECT payload FROM {table} ORDER BY rowid"))
        records['instance'] = tuple(item.model_dump_json() for item in self._instances.values())
        vectors = {(kind, identity, vector_name): EmbeddingVector(model, dimension, bytes(data))
                   for kind, identity, vector_name, model, dimension, data in self.connection.execute(
                       "SELECT kind,record_id,name,model,dimension,data FROM vectors")}
        return MemorySnapshot(name, self.directory, MappingProxyType(records), MappingProxyType(vectors))

    def rename_records(self, old, new, *, instances=None):
        old_uri = f"/api/memory/photos/{quote(old, safe='')}/"
        new_uri = f"/api/memory/photos/{quote(new, safe='')}/"
        def renamed(value):
            if isinstance(value, dict):
                return {key: (new if key in {"map_name", "memory_library_name", "library_name"} and item == old
                              else renamed(item)) for key, item in value.items()}
            if isinstance(value, list):
                return [renamed(item) for item in value]
            if isinstance(value, str) and value.startswith(old_uri):
                return new_uri + value[len(old_uri):]
            return value
        with self.connection:
            for _, _, table in MODELS.values():
                for identity, payload in self.connection.execute(f"SELECT id,payload FROM {table}").fetchall():
                    self.connection.execute(f"UPDATE {table} SET payload=? WHERE id=?",
                        (json.dumps(renamed(json.loads(payload)), ensure_ascii=False), identity))
        values = self._disk_instances() if instances is None else instances
        self._instances = {record.instance_id: SemanticInstance.model_validate(renamed(record.model_dump(mode="json")))
                           for record in values}

    def _disk_instances(self):
        return tuple(SemanticInstance.model_validate_json(row[0]) for row in
                     self.connection.execute("SELECT payload FROM semantic_instances ORDER BY rowid"))

    def close(self):
        self.connection.close()
