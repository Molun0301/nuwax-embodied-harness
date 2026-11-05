"""按库目录进行只读SQL查询，不常驻非当前库的记录。"""
from __future__ import annotations

from contextlib import contextmanager
from pathlib import Path
import json
import sqlite3

from services.memory.store import MODELS


@contextmanager
def read_connection(directory):
    connection = sqlite3.connect((Path(directory)/'memory.db').resolve().as_uri()+'?mode=ro', uri=True)
    try:
        connection.execute('BEGIN')
        yield connection
    finally:
        connection.close()


class MemoryReader:
    def __init__(self, directory, *, name):
        self.directory = Path(directory) if directory is not None else None
        self.name = name

    def exists(self):
        return self.directory is not None and (self.directory/'memory.db').is_file()

    def read(self, kind, *, where='', params=(), order='rowid', limit=None):
        if not self.exists():
            return ()
        model, _, table = MODELS[kind]
        sql = 'SELECT payload FROM '+table
        if where:
            sql += ' WHERE '+where
        if order:
            sql += ' ORDER BY '+order
        if limit is not None:
            sql += ' LIMIT ?'
            params = (*params, limit)
        with read_connection(self.directory) as connection:
            return tuple(model.model_validate_json(row[0]) for row in connection.execute(sql, params))

    def get(self, kind, identity):
        values = self.read(kind, where='id=?', params=(identity,), order=None, limit=1)
        return values[0] if values else None

    def counts(self):
        if not self.exists():
            return {kind: 0 for kind in MODELS}
        with read_connection(self.directory) as connection:
            return {kind: connection.execute('SELECT COUNT(*) FROM '+table).fetchone()[0]
                    for kind, (_, _, table) in MODELS.items()}

    def summary(self):
        counts = self.counts()
        if not self.exists():
            return counts, None, None, None
        with read_connection(self.directory) as connection:
            latest = []
            for _, _, table in MODELS.values():
                for field in ('timestamp','last_seen_ts','semantic_updated_at'):
                    value = connection.execute("SELECT MAX(json_extract(filter_json,'$."+field+"')) FROM "+table).fetchone()[0]
                    if value:
                        latest.append(value)
            location = connection.execute('SELECT id FROM tagged_locations ORDER BY rowid DESC LIMIT 1').fetchone()
            memory = connection.execute('SELECT id FROM semantic_memories ORDER BY rowid DESC LIMIT 1').fetchone()
        return counts, location[0] if location else None, memory[0] if memory else None, max(latest) if latest else None


def filter_sql(value, *, alias='r'):
    """与公开MemoryPayloadFilter相同的条件；数值评分仍由原算法计算。"""
    if value is None:
        return '', ()
    expressions, params = [], []
    column = alias+'.filter_json'
    def field(name):
        return "json_extract("+column+",'$."+name+"')"
    for name in ('map_name','linked_location_id'):
        wanted = getattr(value, name)
        if wanted is not None:
            expressions.append(field(name)+'=?'); params.append(wanted)
    for name, key in (('camera_ids','camera_id'),('topo_node_ids','topo_node_id'),('anchor_ids','anchor_id'),
                      ('scene_space_ids','scene_space_id'),('instance_types','instance_type'),
                      ('movabilities','movability'),('review_statuses','review_status')):
        wanted = getattr(value, name)
        if wanted:
            marks = ','.join('?' for _ in wanted)
            clause = '('+field(key)+' IN ('+marks+') OR EXISTS (SELECT 1 FROM json_each('+column+",'$."+name+"') WHERE value IN ("+marks+'))'
            params.extend(item.value if hasattr(item,'value') else item for item in wanted)
            params.extend(item.value if hasattr(item,'value') else item for item in wanted)
            # 缺少显式锚点时，最终载荷仍可按原空间上下文解析锚点。
            if key == 'anchor_id':
                clause += ' OR '+field(key)+' IS NULL'
            expressions.append(clause+')')
    for name,key in (('semantic_labels_any','semantic_labels'),('visual_labels_any','visual_labels'),('vision_tags_any','vision_tags')):
        wanted = [str(item).strip().lower() for item in getattr(value,name) if str(item).strip()]
        if wanted:
            expressions.append('EXISTS (SELECT 1 FROM json_each('+column+",'$."+key+"') WHERE lower(value) IN ("+','.join('?' for _ in wanted)+'))')
            params.extend(wanted)
    for name,key,operator in (('created_after','timestamp','>='),('created_before','timestamp','<='),
                              ('last_seen_after','last_seen_ts','>='),('last_seen_before','last_seen_ts','<=')):
        wanted = getattr(value,name)
        if wanted is not None:
            expressions.append('julianday('+field(key)+')'+operator+'julianday(?)'); params.append(wanted.isoformat())
    if value.max_age_sec is not None:
        from contracts.base import utc_now
        expressions.append('(julianday(?) - julianday('+field('record_timestamp')+'))*86400<=?')
        params.extend((utc_now().isoformat(),value.max_age_sec))
    if value.near_pose is not None and value.max_distance_m is not None:
        point=value.near_pose.position
        expressions.append('('+field('pose_x')+'-?)*('+field('pose_x')+'-?)+('
            +field('pose_y')+'-?)*('+field('pose_y')+'-?)+(COALESCE('+field('pose_z')+',0)-?)*(COALESCE('
            +field('pose_z')+',0)-?)<=?')
        params.extend((point.x,point.x,point.y,point.y,point.z,point.z,value.max_distance_m**2))
    return ' AND '.join(expressions), tuple(params)
