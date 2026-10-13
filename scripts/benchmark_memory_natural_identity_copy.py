#!/usr/bin/env python3
"""Offline paired natural-identity write benchmark; JSON output path is optional argv[1]."""
import inspect
import json
import statistics
import sys
import textwrap
import time
import unittest
from unittest.mock import patch
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from services.memory.repository import MemoryRepository
from scripts.check_memory_natural_identity import NaturalIdentityChecks
from scripts.check_memory_storage_transactions import IDS

candidate=MemoryRepository._natural_record_identity
source=textwrap.dedent(inspect.getsource(candidate))
needle='return entry.model_copy(update={column: record_id})'
assert source.count(needle)==1
namespace={}
exec(source.replace(needle,'return entry.model_copy(update={column: record_id}, deep=True)'),candidate.__globals__,namespace)
original=namespace['_natural_record_identity']

def record(kind,n):
    entry=NaturalIdentityChecks.summary() if kind=='summary' else NaturalIdentityChecks.relation()
    updates={'evidence_observation_event_ids':[IDS['oev'].rsplit('_',1)[0]+'_%08x'%i for i in range(n)],
             'metadata':{'memory_library_name':'library','nested':{'values':[1,{'value':'owned'}]}}}
    if kind=='summary': updates.update(member_node_ids=['instance-%d'%i for i in range(n)], relation_ids=['relation-%d'%i for i in range(n)])
    return entry.model_copy(update=updates,deep=True)

def fixture(kind,n):
    case=NaturalIdentityChecks()
    service=case.fixture()
    entry=record(kind,n)
    suffix='semantic_summary_node' if kind=='summary' else 'semantic_graph_relation'
    raw=getattr(service._repository,'upsert_'+suffix)
    # A fixed existing legacy ID makes exact output comparison possible.
    raw(entry)
    return case,service,entry,raw,getattr(service,'_upsert_'+suffix)

def ownership(impl,kind):
    with patch.object(MemoryRepository,'_natural_record_identity',impl):
        case,s,entry,raw,method=fixture(kind,4)
        try:
            column='summary_id' if kind=='summary' else 'relation_id'
            cache=s._semantic_summary_nodes_by_id if kind=='summary' else s._semantic_graph_relations_by_id
            expected=entry.model_dump()
            with s._repository.storage.transaction():
                result=method(entry)
                result.metadata['nested']['values'][1]['value']='returned mutation'
                result.evidence_observation_event_ids.clear()
                entry.metadata['nested']['values'][1]['value']='input mutation'
                entry.evidence_observation_event_ids.clear()
                assert not cache
            published=cache[getattr(result,column)]
            assert published.model_dump()==expected
            rows=(s._repository.load_semantic_summary_nodes() if kind=='summary' else s._repository.load_semantic_graph_relations())
            assert rows[0].model_dump()==expected
            rows[0].metadata['nested']['values'][1]['value']='loaded mutation'
            assert published.model_dump()==expected
            # Each committed upsert obtains a separate private model.
            second=method(record(kind,4))
            second.metadata['nested']['values'].clear()
            assert published.model_dump()==expected
            return expected
        finally: case.doCleanups()

for kind in ('summary','relation'):
    assert ownership(original,kind)==ownership(candidate,kind)
print(json.dumps({'ownership':'passed','python':sys.version}),flush=True)
for name,impl in [('original',original),('candidate',candidate)]:
    with patch.object(MemoryRepository,'_natural_record_identity',impl):
        suite=unittest.defaultTestLoader.loadTestsFromTestCase(NaturalIdentityChecks)
        result=unittest.TextTestRunner(verbosity=1).run(suite)
        assert result.wasSuccessful()
        print(json.dumps({'qualification':name,'tests':result.testsRun,'passed':True}),flush=True)

results=[]
for kind in ('summary','relation'):
  for n in (4,256,2048):
    for layer in ('repository','service'):
      fixtures=[fixture(kind,n),fixture(kind,n)]
      try:
        data={'original':[],'candidate':[]}
        cpu={'original':[],'candidate':[]}
        outputs={}
        for round_ in range(7):
          for label,index,impl in ([('original',0,original),('candidate',1,candidate)] if round_%2==0 else [('candidate',1,candidate),('original',0,original)]):
            case,s,entry,raw,method=fixtures[index]
            operation=(lambda:raw(entry,natural_identity='bench')) if layer=='repository' else lambda:method(entry)
            with patch.object(MemoryRepository,'_natural_record_identity',impl):
              operation()
              start_cpu=time.process_time(); start=time.perf_counter()
              for _ in range(30): result=operation()
              elapsed=time.perf_counter()-start; elapsed_cpu=time.process_time()-start_cpu
            data[label].append(elapsed/30); cpu[label].append(elapsed_cpu/30)
            outputs[label]=result.model_dump()
        assert outputs['original']==outputs['candidate']
        row={'kind':kind,'size':n,'layer':layer,'seconds_per_operation':data,'cpu_seconds_per_operation':cpu,
             'median_reduction_pct':100*(1-statistics.median(data['candidate'])/statistics.median(data['original'])), 'output_equal':True}
        results.append(row); print(json.dumps(row),flush=True)
      finally:
        for value in fixtures:value[0].doCleanups()
Path(sys.argv[1] if len(sys.argv)>1 else '/tmp/neh-natural-identity-copy-results.json').write_text(json.dumps(results,indent=2))
