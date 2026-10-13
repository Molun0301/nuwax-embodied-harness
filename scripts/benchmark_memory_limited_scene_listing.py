#!/usr/bin/env python3
"""Offline 1k/10k scene listing benchmark; median of three calls, no model/network.

Counts source rows fetched into Python, not SQLite internal sorting visits.
"""
import sys,json,time,statistics
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scripts.check_memory_durable_integration import IntegrationChecks
from scripts.check_memory_storage_transactions import records
from contracts.memory import MemoryPayloadFilter
from contracts.spatial_memory import SceneSpace
from services.memory.sqlite_storage import SQLiteTransaction

class Cursor:
    def __init__(self,c,counts,key):self.c,self.counts,self.key=c,counts,key
    def fetchall(self):
        rows=self.c.fetchall();self.counts[self.key]+=len(rows);return rows
    def fetchmany(self,*a):
        rows=self.c.fetchmany(*a);self.counts[self.key]+=len(rows);return rows
    def __iter__(self):
        for row in self.c:self.counts[self.key]+=1;yield row
    def close(self):self.c.close()

h=IntegrationChecks();s=h.fixture();r=s._repository
base=next(record for _,_,_,record in records() if isinstance(record,SceneSpace));r.upsert_scene_space(base)
with r.storage.transaction() as tx:
    row=dict(tx.execute('SELECT * FROM scene_spaces').fetchone()); state=dict(tx.execute("SELECT * FROM memory_index_sources WHERE source_kind='scene'").fetchone())
    tx.execute('DELETE FROM scene_spaces');tx.execute("DELETE FROM memory_index_sources WHERE source_kind='scene'")
    values=[];states=[]
    for i in range(10000):
        rid='ssp_limit_20260101T000000Z_%08x'%i
        p=base.model_dump(mode='json');p['scene_space_id']=rid;p['primary_label']='match' if i%100==99 else 'other'
        a=dict(row,scene_space_id=rid,payload_json=json.dumps(p),primary_label=p['primary_label']);values.append(tuple(a.values()))
        b=dict(state,external_id=rid,internal_id='limit-'+str(i),record_incarnation='inc-'+str(i));states.append(tuple(b.values()))
    tx.executemany('INSERT INTO scene_spaces ('+','.join(row)+') VALUES ('+','.join('?' for _ in row)+')',values)
    tx.executemany('INSERT INTO memory_index_sources ('+','.join(state)+') VALUES ('+','.join('?' for _ in state)+')',states)
original=SQLiteTransaction.execute;load=r._load_record
for n in (10000,1000):
    if n==1000:
        with r.storage.transaction() as tx:tx.execute("DELETE FROM scene_spaces WHERE scene_space_id>=?",('ssp_limit_20260101T000000Z_%08x'%n,))
    for select in ('broad','selective','none'):
        f=MemoryPayloadFilter(semantic_labels_any=[] if select=='broad' else ['match' if select=='selective' else 'missing'])
        for limit in (5,20):
            times=[]
            for _ in range(3):
                counts=dict(models=0,scene_rows=0,token_rows=0)
                def execute(tx,sql,parameters=()):
                    c=original(tx,sql,parameters)
                    if sql.startswith('SELECT') and 'FROM scene_spaces' in sql:return Cursor(c,counts,'scene_rows')
                    if sql.startswith('SELECT external_id,record_incarnation,business_revision'):return Cursor(c,counts,'token_rows')
                    return c
                def hydrate(*a,**k):counts['models']+=1;return load(*a,**k)
                with patch.object(SQLiteTransaction,'execute',execute),patch.object(r,'_load_record',hydrate):
                    start=time.perf_counter();out=r.query_scene_spaces(f,library_name='library',limit=limit);times.append((time.perf_counter()-start)*1000)
            print(json.dumps(dict(n=n,filter=select,limit=limit,returned=len(out),ms=round(statistics.median(times),3),**counts)),flush=True)
h.doCleanups()
