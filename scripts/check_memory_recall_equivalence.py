#!/usr/bin/env python3
"""Offline mixed-source recall, stale suppression and current-filter oracle."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from scripts.check_memory_durable_integration import IntegrationChecks, FakeEmbedder
from scripts.check_memory_storage_transactions import records
from contracts.memory import MemoryRecordKind
from contracts.spatial_memory import InstanceDynamicState
from services.memory.vector_store import NamedVector

class SemanticFake(FakeEmbedder):
    def embed_text(self, text):
        self.calls += 1
        v=np.zeros(16,dtype=np.float32)
        v[0],v[1]=(1.,0.) if text=='needle' else (.9,.1)
        return v

c=IntegrationChecks();s=c.fixture(embedder=SemanticFake())
try:
    healthy,_=c.accept(s,c.memory(memory_id='mem_valid_20260101T000000Z_00000001',title='Semantic destination',summary='Meaningful item'))
    s._run_durable_vectorization(c.claim(s))
    old_payload=s._build_memory_payload(c.memory(title='needle historical',summary='Old evidence'))
    high=np.zeros(16,dtype=np.float32);high[0]=1
    old=NamedVector('text_dense',s._text_embedder.model_name,16,high)
    for i in range(120):
        rid='mem_stale_20260101T000000Z_%08x'%i
        p={**old_payload,'record_id':rid}
        s._vector_store.upsert_point(point_id=rid,collection_name='episodic_observations',record_kind='scene',payload=p,vectors={'text_dense':old})
        c.accept(s,c.memory(memory_id=rid,title='Unrelated current item %d'%i,summary='Current evidence'))
    lexical,_=c.accept(s,c.memory(memory_id='mem_lexical_20260101T000000Z_00000001',title='needle lexical fallback'))
    live=records()[2][3].model_copy(update={'dynamic_state':InstanceDynamicState.STATIC,'label':'needle ephemeral','display_name':'needle ephemeral'})
    s._semantic_instances_by_id[live.instance_id]=live
    result=s._query_records('needle',allowed_record_kinds=(MemoryRecordKind.SCENE,MemoryRecordKind.OBJECT_INSTANCE),similarity_threshold=.4,limit=5,payload_filter=None)
    ids={m.record_id for m in result.matches}
    print('MATCHES',sorted(ids))
    print([(m.record_id,m.score,m.metadata.get("vector_provenance")) for m in result.matches])
    assert healthy.memory_id in ids,'stale high-score rows crowded out valid semantic match'
    assert lexical.memory_id in ids,'current lexical source was lost'
    assert live.instance_id in ids,'live-only lexical instance was lost'
    assert not any(x.startswith('mem_stale_') for x in ids),'known-stale similarity leaked'
    print('PASS recall, lexical, live-only, stale suppression')
finally:c.doCleanups()

# 同库元数据更新不重做向量；筛选继续使用已提交的当前源。
import tempfile
from pathlib import Path
from contracts.memory import MemoryPayloadFilter
with tempfile.TemporaryDirectory() as d:
    s=c.fixture(path=str(Path(d)/'source.db'),embedder=SemanticFake())
    store=s._vector_store
    try:
        record,receipt=c.accept(s,c.memory(title='Semantic destination',summary='Meaningful item',map_name='old-map'))
        s._run_durable_vectorization(c.claim(s))
        s._repository.upsert_semantic_memory(record.model_copy(update={'map_name':'new-map'}))
        assert s._durable_index.outbox.current(receipt.version.key)==receipt
        assert store.get_point(record.memory_id).payload['map_name']=='new-map'
        result=s._query_records('needle',allowed_record_kinds=(MemoryRecordKind.SCENE,),similarity_threshold=.4,limit=5,payload_filter=MemoryPayloadFilter(map_name='new-map'))
        assert [m.record_id for m in result.matches]==[record.memory_id],'derived cache old metadata hid current filter match'
        result=s._query_records('needle',allowed_record_kinds=(MemoryRecordKind.SCENE,),similarity_threshold=.4,limit=5,payload_filter=MemoryPayloadFilter(map_name='old-map'))
        assert not result.matches,'stale cache payload passed old filter'
        print('PASS shared-cache current metadata filtering')
    finally:c.doCleanups()

# A committed source must remain searchable while its after-commit cache callback
# is delayed. Candidate narrowing cannot rely solely on the older live renderer.
import threading
from unittest.mock import patch
s=c.fixture(embedder=SemanticFake())
record,_=c.accept(s,c.memory(title='zzzz qqqq',summary='yyyy'))
s._run_durable_vectorization(c.claim(s))
entered=threading.Event();release=threading.Event();errors=[]
original=s._publish_index_record

def delayed(*a,**kw):
    entered.set()
    assert release.wait(3)
    return original(*a,**kw)

def writer():
    try:s._repository.upsert_semantic_memory(record.model_copy(update={'title':'needle','summary':'needle'}))
    except BaseException as exc:errors.append(exc)

try:
    with patch.object(s,'_publish_index_record',side_effect=delayed):
        t=threading.Thread(target=writer);t.start();assert entered.wait(2)
        try:
            assert s._semantic_entries_by_id[record.memory_id].title=='zzzz qqqq'
            result=s._query_records('needle',allowed_record_kinds=(MemoryRecordKind.SCENE,),similarity_threshold=.4,limit=5,payload_filter=None)
            assert len(result.matches)==1,'new committed lexical match lost behind delayed cache callback'
            assert result.matches[0].semantic_memory.title=='needle','old source payload attached to current match'
            assert result.matches[0].metadata['vector_provenance']['text_dense']['status']=='known_stale_or_incompatible'
        finally:release.set();t.join(3)
    assert not errors,errors
    print('PASS committed source before cache callback')
finally:release.set();c.doCleanups()


# Input changes between ranking and survivor decoding must not freeze old cutoffs.
from unittest.mock import patch
import numpy as np
from scripts.check_memory_durable_integration import IntegrationChecks,FakeEmbedder
from contracts.memory import MemoryRecordKind
class ConstantFake(FakeEmbedder):
 def embed_text(self,text):self.calls+=1;return np.ones(16,dtype=np.float32)
c=IntegrationChecks();s=c.fixture(embedder=ConstantFake())
try:
 records=[]
 for i,title in enumerate(['needle','needle','zzzz qqqq']):
  record,_=c.accept(s,c.memory(memory_id='mem_cutover_20260101T000000Z_%08x'%i,title=title,summary='yyyy'))
  records.append(record)
 token=s._durable_index.outbox.start_worker()
 while True:
  claim=s._durable_index.outbox.claim_next(token)
  if claim is None:break
  s._run_durable_vectorization(claim)
 original=s._durable_index.query_snapshot;first=[True]
 def change_after_snapshot(*a,**kw):
  result=original(*a,**kw)
  if first[0]:
   first[0]=False
   for record in records[:2]:s._repository.upsert_semantic_memory(record.model_copy(update={'title':'needle secondary stale'}))
  return result
 def query():return s._query_records('needle',allowed_record_kinds=(MemoryRecordKind.SCENE,),similarity_threshold=.4,limit=1,payload_filter=None)
 with patch.object(s._durable_index,'query_snapshot',side_effect=change_after_snapshot):raced=query()
 current=query()
 print('RACED',[(m.record_id,m.score,m.semantic_memory.title) for m in raced.matches])
 print('CURRENT',[(m.record_id,m.score,m.semantic_memory.title) for m in current.matches])
 assert [m.record_id for m in raced.matches]==[m.record_id for m in current.matches],'superseded first-phase ranks crowded unchanged stronger valid candidate'
finally:c.doCleanups()
