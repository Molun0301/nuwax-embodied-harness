"""每库一个 SQLite 文件；所有写入由记忆服务的唯一写线程调用。"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional
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
    name: Optional[str]
    directory: Optional[Path]
    records: object
    vectors: object
    objects: object = None
    transient: object = None

    @classmethod
    def empty(cls):
        return cls(None, None, MappingProxyType({kind: () for kind in MODELS}), MappingProxyType({}))

    def read(self, kind):
        if self.objects is not None:
            return tuple(value.model_copy(deep=True) for value in self.objects[kind].values())
        model = MODELS[kind][0]
        return tuple(model.model_validate_json(value) for value in self.records[kind])

    def get(self, kind, identity):
        if self.objects is not None:
            value = self.objects[kind].get(identity)
            return value.model_copy(deep=True) if value is not None else None
        return next((value for value in self.read(kind) if getattr(value, MODELS[kind][1]) == identity), None)

    def model_map(self, kind):
        if self.objects is not None:
            return self.objects[kind]
        return MappingProxyType({getattr(value, MODELS[kind][1]): value for value in self.read(kind)})

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
    def __init__(self, directory, *, name, indexer=None):
        self.name, self.indexer = name, indexer
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.photos = self.directory/'photos'
        self.photos.mkdir(exist_ok=True)
        self.connection = sqlite3.connect(self.directory/'memory.db')
        try:
            self._initialize()
        except Exception:
            self.connection.close()
            raise

    def _initialize(self):
        self.connection.execute("BEGIN")
        rebuild = set()
        for kind, (_, _, table) in MODELS.items():
            extra = (", relation_kind TEXT, source_node_kind TEXT, source_node_id TEXT, target_node_kind TEXT, target_node_id TEXT"
                if kind == 'relation' else ", summary_kind TEXT, label TEXT, scene_space_id TEXT, semantic_region_id TEXT"
                if kind == 'summary' else '')
            self.connection.execute('CREATE TABLE IF NOT EXISTS '+table+' (id TEXT PRIMARY KEY, payload TEXT NOT NULL'+extra+')')
            columns = {row[1] for row in self.connection.execute('PRAGMA table_info('+table+')')}
            for column in ('search_text','lexical_json','exact_json','filter_json'):
                if column not in columns:
                    self.connection.execute('ALTER TABLE '+table+' ADD COLUMN '+column+' TEXT')
                    rebuild.add(kind)
            for field in ('map_name','camera_id','scene_space_id','anchor_id','source_memory_id','source_record_id','label','timestamp','last_seen_ts','semantic_updated_at'):
                self.connection.execute('CREATE INDEX IF NOT EXISTS '+table+'_'+field+" ON "+table+"(json_extract(filter_json,'$."+field+"'))")
            for field in ('timestamp','last_seen_ts','semantic_updated_at'):
                self.connection.execute('CREATE INDEX IF NOT EXISTS '+table+'_'+field+"_date ON "+table+"(julianday(json_extract(filter_json,'$."+field+"')))")
        self.connection.execute('CREATE INDEX IF NOT EXISTS relation_business_key ON semantic_graph_relations(relation_kind,source_node_kind,source_node_id,target_node_kind,target_node_id)')
        self.connection.execute('CREATE INDEX IF NOT EXISTS summary_business_key ON semantic_summary_nodes(summary_kind,label,scene_space_id,semantic_region_id)')
        self.connection.execute('CREATE TABLE IF NOT EXISTS vectors (kind TEXT NOT NULL,record_id TEXT NOT NULL,name TEXT NOT NULL,model TEXT NOT NULL,dimension INTEGER NOT NULL,data BLOB NOT NULL,PRIMARY KEY(kind,record_id,name))')
        self.connection.execute('CREATE INDEX IF NOT EXISTS vectors_model ON vectors(name,model,dimension,kind)')
        self._json = {kind: dict(self.connection.execute('SELECT id,payload FROM '+table+' ORDER BY rowid')) for kind,(_,_,table) in MODELS.items()}
        self._objects = {kind: {identity:model.model_validate_json(payload) for identity,payload in self._json[kind].items()}
                         for kind,(model,_,table) in MODELS.items()}
        self._texts = {(kind,identity): text for kind,(_,_,table) in MODELS.items()
                       for identity,text in self.connection.execute('SELECT id,search_text FROM '+table)}
        self._vectors = {(kind,identity,name): EmbeddingVector(model,dimension,bytes(data))
            for kind,identity,name,model,dimension,data in self.connection.execute('SELECT kind,record_id,name,model,dimension,data FROM vectors')}
        self._snapshot = MemorySnapshot.empty()
        self.changed = set()
        self._pending = {}
        self._pending_vectors = {}
        self._pending_texts = {}
        self._pending_changed = set()
        self._nonpersistent = set()
        self._pending_persistence = {}
        for kind in rebuild:
            for record in self._objects[kind].values():
                indexed = self._index(kind,record)
                self.connection.execute('UPDATE '+MODELS[kind][2]+' SET search_text=?,lexical_json=?,exact_json=?,filter_json=? WHERE id=?',
                    (*indexed,getattr(record,MODELS[kind][1])))
                self._texts[kind,getattr(record,MODELS[kind][1])] = indexed[0]
        self.connection.commit()
        self._refresh_snapshot(set(MODELS))

    def _index(self, kind, record):
        if self.indexer is not None:
            text, lexical, exact, fields = self.indexer(kind,record,self)
        else:
            text,lexical,exact,fields = '',[],[],record.model_dump(mode='json')
        return text,json.dumps(lexical,ensure_ascii=False),json.dumps(exact,ensure_ascii=False),json.dumps(fields,ensure_ascii=False)

    def get(self, kind, identity):
        value = self._pending.get((kind,identity),self._objects[kind].get(identity))
        return value.model_copy(deep=True) if value is not None else None

    def model_map(self, kind):
        values = self._objects[kind]
        if any(item_kind == kind for item_kind,_ in self._pending):
            values = dict(values)
            for (item_kind,identity),record in self._pending.items():
                if item_kind == kind:
                    if record is None: values.pop(identity,None)
                    else:
                        values.pop(identity,None); values[identity] = record
        return values

    def records(self, kind):
        return tuple(record.model_copy(deep=True) for record in self.model_map(kind).values())

    def has(self, kind, identity):
        return self.connection.execute("SELECT 1 FROM "+MODELS[kind][2]+" WHERE id=?",(identity,)).fetchone() is not None

    def put(self, kind, record):
        _,identity,table = MODELS[kind]
        fields = self._business_key_fields(kind,record)
        if fields:
            where = ' AND '.join(field+'=?' for field,_ in fields)
            previous = self.connection.execute('SELECT id FROM '+table+' WHERE '+where+' ORDER BY rowid LIMIT 1',tuple(value for _,value in fields)).fetchone()
            if previous is not None:
                record = record.model_copy(update={identity:previous[0]})
        identifier = getattr(record,identity)
        old = self.get(kind,identifier)
        if old == record and self.connection.execute('SELECT 1 FROM '+table+' WHERE id=?',(identifier,)).fetchone():
            return record
        indexed = self._index(kind,record)
        columns = ('relation_kind','source_node_kind','source_node_id','target_node_kind','target_node_id') if kind == 'relation' else ('summary_kind','label','scene_space_id','semantic_region_id') if kind == 'summary' else ()
        values = [identifier,record.model_dump_json(),*indexed,
            *(getattr(record,field).value if hasattr(getattr(record,field),'value') else getattr(record,field) or '' for field in columns)]
        self.connection.execute('INSERT OR REPLACE INTO '+table+' (id,payload,search_text,lexical_json,exact_json,filter_json'+''.join(','+field for field in columns)+') VALUES ('+','.join('?' for _ in values)+')',tuple(values))
        self._pending[kind,identifier] = record.model_copy(deep=True)
        if kind == "instance":
            self._pending_persistence[identifier] = True
        previous_text = self._pending_texts.get((kind,identifier),self._texts.get((kind,identifier)))
        self._pending_texts[kind,identifier] = indexed[0]
        if old is None or previous_text != indexed[0]:
            self._delete_vectors(kind,identifier,'text')
        if old is None or self._photos(old) != self._photos(record):
            self._delete_vectors(kind,identifier,'image')
        if kind in {'location','memory','instance','observation','scene_space'}:
            self._pending_changed.add((kind,identifier))
        return record

    @staticmethod
    def _photos(record):
        return tuple(getattr(record,'artifact_ids',record.metadata.get('artifact_ids',())))

    @staticmethod
    def _business_key_fields(kind, record):
        if kind == 'relation':
            return tuple((field,getattr(record,field).value if field == 'relation_kind' else getattr(record,field))
                         for field in ('relation_kind','source_node_kind','source_node_id','target_node_kind','target_node_id'))
        if kind == 'summary':
            fields = [('summary_kind',record.summary_kind.value),('label',record.label or '')]
            if record.summary_kind.value != 'target_category_cluster': fields.append(('scene_space_id',record.scene_space_id or ''))
            if record.summary_kind.value == 'region': fields.append(('semantic_region_id',record.semantic_region_id or ''))
            return tuple(fields)
        return ()

    def _delete_vectors(self, kind, identity, name=None):
        sql = 'DELETE FROM vectors WHERE kind=? AND record_id=?'
        params = (kind,identity)
        if name is not None:
            sql += ' AND name=?'; params += (name,)
        self.connection.execute(sql,params)
        for vector_name in ((name,) if name is not None else ('text','image')):
            key = (kind,identity,vector_name)
            if key in self._vectors or key in self._pending_vectors:
                self._pending_vectors[key] = None

    def delete(self, kind, identity):
        self.connection.execute('DELETE FROM '+MODELS[kind][2]+' WHERE id=?',(identity,))
        self._pending[kind,identity] = None
        if kind == "instance":
            self._pending_persistence[identity] = True
        self._pending_texts[kind,identity] = None
        self._delete_vectors(kind,identity)
        self._pending_changed.discard((kind,identity))

    def put_live_instance(self, record, *, persist):
        if persist:
            return self.put('instance',record)
        self.connection.execute('DELETE FROM semantic_instances WHERE id=?',(record.instance_id,))
        self._pending['instance',record.instance_id] = record.model_copy(deep=True)
        self._pending_persistence[record.instance_id] = False
        self._delete_vectors('instance',record.instance_id)
        return record

    def put_vector(self, kind, identity, name, model, vector):
        data = np.asarray(vector,dtype='<f4')
        self.connection.execute('INSERT OR REPLACE INTO vectors VALUES (?,?,?,?,?,?)',(kind,identity,name,model,data.size,data.tobytes()))
        self._pending_vectors[kind,identity,name] = EmbeddingVector(model,data.size,data.tobytes())

    def put_photo(self, artifact_id, content):
        (self.photos/artifact_id).write_bytes(content)

    def commit_cache(self):
        kinds = {kind for kind,_ in self._pending}
        for kind in kinds:
            values = dict(self._objects[kind])
            encoded = dict(self._json[kind])
            for (item_kind,identity),record in self._pending.items():
                if item_kind == kind:
                    if record is None:
                        values.pop(identity,None); encoded.pop(identity,None)
                    else:
                        values.pop(identity,None); values[identity] = record
                        encoded.pop(identity,None); encoded[identity] = record.model_dump_json()
            self._objects[kind] = values
            self._json[kind] = encoded
        if self._pending_vectors:
            self._vectors = dict(self._vectors)
            for key,value in self._pending_vectors.items():
                if value is None: self._vectors.pop(key,None)
                else: self._vectors[key] = value
        for identity,persisted in self._pending_persistence.items():
            if persisted: self._nonpersistent.discard(identity)
            else: self._nonpersistent.add(identity)
        self._texts.update(self._pending_texts)
        self.changed.update(self._pending_changed)
        self._refresh_snapshot(kinds)
        self.rollback_cache()

    def rollback_cache(self):
        self._pending.clear(); self._pending_vectors.clear(); self._pending_texts.clear(); self._pending_changed.clear(); self._pending_persistence.clear()

    def _refresh_snapshot(self, kinds):
        records = dict(self._snapshot.records)
        objects = dict(self._snapshot.objects or {})
        for kind in kinds:
            records[kind] = tuple(self._json[kind].values())
            objects[kind] = MappingProxyType(self._objects[kind])
        transient = (MappingProxyType({identity:self._objects["instance"][identity] for identity in self._nonpersistent})
                     if "instance" in kinds else self._snapshot.transient)
        self._snapshot = MemorySnapshot(self.name,self.directory,MappingProxyType(records),MappingProxyType(self._vectors),MappingProxyType(objects),transient)

    def snapshot(self, name=None):
        return self._snapshot

    def rename_records(self, old, new, *, instances=None):
        old_uri = '/api/memory/photos/'+quote(old,safe='')+'/'
        new_uri = '/api/memory/photos/'+quote(new,safe='')+'/'
        def renamed(value):
            if isinstance(value,dict):
                return {key:new if key in {'map_name','memory_library_name','library_name'} and item == old else renamed(item) for key,item in value.items()}
            if isinstance(value,list): return [renamed(item) for item in value]
            if isinstance(value,str) and value.startswith(old_uri): return new_uri+value[len(old_uri):]
            return value
        persisted_instances = set(self._objects["instance"])
        with self.connection:
            for kind,(_,identity,table) in MODELS.items():
                source = instances if kind == 'instance' and instances is not None else self.records(kind)
                for record in source:
                    value = type(record).model_validate(renamed(record.model_dump(mode='json')))
                    if kind == "instance" and getattr(value,identity) not in persisted_instances:
                        self.put_live_instance(value,persist=False)
                    else:
                        self.put(kind,value)
        self.commit_cache()

    def close(self):
        self.connection.close()
        self._objects.clear(); self._json.clear(); self._vectors = {}
        self._texts.clear(); self._nonpersistent.clear()
        self.rollback_cache()
        self._snapshot = MemorySnapshot.empty()
