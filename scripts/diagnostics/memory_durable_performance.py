#!/usr/bin/env python3
"""Offline memory service comparison; explicit constant fixture vectors, no workers/network.

Run each revision in a fresh process using --repo. Admission measures source SQL
only on the historical revision (which had no atomic index admission); the new
revision includes durable outbox/provenance acceptance. This is not an equivalent
durability claim. File-backed FULL versus historical NORMAL is reported explicitly.
"""
from __future__ import annotations
import argparse
import math
import json
import resource
import random
import statistics
import sys
import tempfile
import threading
import time
import tracemalloc
from datetime import datetime,timezone
from pathlib import Path
from types import SimpleNamespace

class TimedLock:
    def __init__(self,lock):self.lock=lock;self.depth=0;self.samples=[];self.start=0;self.on_query_acquire=None
    def acquire(self,*a,**kw):
        result=self.lock.acquire(*a,**kw)
        if result:
            if self.depth==0:self.start=time.perf_counter()
            self.depth+=1
            if self.depth==1 and self.on_query_acquire is not None and threading.current_thread().name=='benchmark-query':
                self.on_query_acquire()
        return result
    def release(self):
        self.depth-=1
        if self.depth==0:self.samples.append(time.perf_counter()-self.start)
        return self.lock.release()
    def __enter__(self):self.acquire();return self
    def __exit__(self,*args):self.release()
    def _is_owned(self):return getattr(self.lock,'_is_owned',lambda:False)()

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--repo',required=True);parser.add_argument('--counts',default='1000,5000')
    parser.add_argument('--profile');parser.add_argument('--unique-text',action='store_true');parser.add_argument('--varied-text',action='store_true');parser.add_argument('--mixed-trials',type=int,default=0);parser.add_argument('--samples',type=int,default=3);parser.add_argument('--backend',choices=['memory','file'],default='memory')
    args=parser.parse_args();sys.path.insert(0,args.repo)
    from contracts.memory import SemanticMemoryEntry,MemoryRecordKind
    from services.memory.service import MemoryService
    from services.memory.vectorizer import TextEmbedder
    import numpy as np
    class ExplicitFixtureEmbedder(TextEmbedder):
        model_name='explicit-constant-fixture-v2'
        model_revision='fixture-v2'
        dimension=128
        def embed_text(self,text):
            vector=np.zeros(self.dimension,dtype=np.float32)
            if text.strip():vector[0]=1.0
            return vector
    from services.memory.vector_store import NamedVector
    output=[]
    for count in map(int,args.counts.split(',')):
        with tempfile.TemporaryDirectory(prefix='neh-memory-perf-') as temporary:
            path=':memory:' if args.backend=='memory' else str(Path(temporary)/'memory.db')
            model=ExplicitFixtureEmbedder()
            service=MemoryService(localization_service=None,mapping_service=None,observation_service=None,perception_service=None,
                state_store=SimpleNamespace(write=lambda *a,**kw:None),memory_db_path=path,text_embedder=model,
                text_embedding_model=model.model_name,text_embedding_dimension=128)
            service._get_current_pose=lambda:None;service._memory_enabled=True;service._active_library_name='synthetic'
            durable=hasattr(service,'_durable_index')
            if durable:service._durable_index.activate('synthetic')
            records=[];admission=[];timestamp=datetime(2026,1,1,tzinfo=timezone.utc)
            randomizer=random.Random(3101)
            vocabulary=('red blue green yellow wooden metal glass plastic small large round narrow tall low '
                'chair desk table cabinet shelf counter door window hallway office kitchen workshop laboratory '
                'entrance corridor storage charger toolbox notebook monitor keyboard bottle basket cup lamp '
                'north south east west left right beside behind opposite near under above corner wall floor').split()
            for number in range(count):
                title='beacon target unique' if number==0 else 'Object item %d'%number
                summary='Synthetic gallery item %d'%number if args.unique_text else 'Synthetic gallery item'
                if args.varied_text:
                    if number:title=' '.join(randomizer.sample(vocabulary,5))
                    summary=' '.join(randomizer.sample(vocabulary,12)) if number else title
                record=SemanticMemoryEntry(memory_id='mem_synthetic_20260101T000000Z_%08d'%number,
                    title=title,summary=summary,timestamp=timestamp,metadata={'memory_library_name':'synthetic'})
                start=time.perf_counter();service._repository.upsert_semantic_memory(record);admission.append(time.perf_counter()-start)
                service._semantic_entries_by_id[record.memory_id]=record;records.append(record)
            start=time.perf_counter()
            if durable:
                box=service._durable_index.outbox;token=box.start_worker()
                while True:
                    claim=box.claim_next(token)
                    if claim is None:break
                    service._run_durable_vectorization(claim)
                box.stop_worker(token)
            else:
                for record in records:
                    payload=service._build_memory_payload(record);vector=model.embed_text(payload['retrieval_text'])
                    service._vector_store.upsert_point(point_id=record.memory_id,collection_name='episodic_observations',record_kind='scene',
                        payload=payload,vectors={'text_dense':NamedVector('text_dense',model.model_name,model.dimension,vector)})
            index_seconds=time.perf_counter()-start
            locks=[];instrumented_locks=[]
            owners=[service._repository.storage] if durable else [service._repository,service._vector_store]
            for owner in owners:
                for attr in ('_lock','_reader_lock'):
                    if hasattr(owner,attr):
                        wrapper=TimedLock(getattr(owner,attr));setattr(owner,attr,wrapper);locks.append(wrapper)
                        instrumented_locks.append(type(owner).__name__+'.'+attr)
            def query():return service._query_records('beacon target unique',allowed_record_kinds=(MemoryRecordKind.SCENE,),
                similarity_threshold=.1,limit=5,payload_filter=None)
            query();samples=[];result=None;all_lock_samples=[]
            if args.profile:
                import cProfile
                profiler=cProfile.Profile();profiler.enable();query();profiler.disable()
                profiler.dump_stats(args.profile+'.'+str(count))
            for _ in range(args.samples):
                for lock in locks:lock.samples.clear()
                start=time.perf_counter();result=query();samples.append(time.perf_counter()-start)
                all_lock_samples.append([value*1000 for lock in locks for value in lock.samples])
            lock_samples=[value for lock in locks for value in lock.samples]
            tracemalloc.start();query();_,peak=tracemalloc.get_traced_memory();tracemalloc.stop()
            assert result.matches and result.matches[0].record_id==records[0].memory_id
            overlap=[]
            for _ in range(args.mixed_trials):
                entered=threading.Event();errors=[]
                owner_lock=service._repository.storage._lock if durable else service._vector_store._lock
                owner_lock.on_query_acquire=entered.set
                def concurrent_query():
                    try:query()
                    except BaseException as error:errors.append(error)
                thread=threading.Thread(target=concurrent_query,name='benchmark-query');thread.start()
                assert entered.wait(5),'query never acquired storage'
                start=time.perf_counter();service._vector_store.get_point(records[-1].memory_id);overlap.append(time.perf_counter()-start)
                thread.join(10);assert not thread.is_alive();assert not errors,errors
                owner_lock.on_query_acquire=None
            # Separate untimed invocation makes fuzzy cache hit rate observable
            # without perturbing measured queries, allocations, or overlap reads.
            # This is a fixture property, not a throughput metric.
            from unittest.mock import patch
            import services.memory.service as service_module
            with patch.object(service_module,'SequenceMatcher',wraps=service_module.SequenceMatcher) as matcher:
                query();fuzzy_evaluations=matcher.call_count
            output.append({'count':count,'fixture_version':'explicit-constant-v2','workload_seed':3101,'unique_text':args.unique_text,'varied_text':args.varied_text,'durable':durable,'backend':args.backend,'dimension':128,
                'admission_median_ms':statistics.median(admission)*1000,'admission_total_s':sum(admission),'index_seed_s':index_seconds,
                'query_median_ms':statistics.median(samples)*1000,'query_samples_ms':[v*1000 for v in samples],
                'query_lock_max_ms':max(lock_samples,default=0)*1000,'query_lock_total_ms':sum(lock_samples)*1000,
                'query_lock_hold_samples_ms':all_lock_samples,
                'query_lock_sources':instrumented_locks,
                'query_lock_scope_note':'Only listed locks are measured; pooled-reader admission and lease duration are not included.',
                'query_all_samples_lock_max_ms':max((value for sample in all_lock_samples for value in sample),default=0),
                'untimed_fuzzy_evaluations':fuzzy_evaluations,
                'query_tracemalloc_peak_mb':peak/1024/1024,'process_peak_rss_mb':resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,
                'returned_ids':[m.record_id for m in result.matches],'returned_scores':[m.score for m in result.matches],
                'overlap_point_read_samples_ms':[v*1000 for v in overlap],
                'overlap_point_read_p95_ms':sorted(overlap)[max(0,math.ceil(len(overlap)*.95)-1)]*1000 if overlap else None,
                'durability':'FULL' if durable else 'NORMAL historical source-only admission'})
            if durable:service.close()
            else:service._repository.close();service._vector_store.close()
    source_paths=['services/memory/'+name+'.py' for name in ('service','durable_index','repository','vector_store','scene_mapping_state')]
    print(json.dumps({'repo':str(Path(args.repo).resolve()),'source_paths':source_paths,'provider_scope':'constant fixture vectors; not comparable to prior embedding-space score results','results':output},indent=2))
if __name__=='__main__':main()
