"""只读历史住宅源，在独立库实测现有区域模型、统一提交及显式名称保护。"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import shutil
import sqlite3
import sys
from datetime import datetime
from types import SimpleNamespace

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from contracts.memory import SemanticMemoryEntry
from contracts.runtime_views import MapSnapshot
from core.capability_registry import CapabilityRegistry
from core.settings import SceneMappingConfig
from services.artifact_store import LocalArtifactStore
from services.memory.service import MemoryService
from services.memory.scene_mapping import SceneMappingService
from services.scene_decision import create_scene_decision_backend
from gateways.scene_mapping import register_scene_mapping_capabilities


def iso(value):
    return datetime.fromisoformat(str(value).replace('Z','+00:00'))


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--base-url',default='http://127.0.0.1:8791')
    args=parser.parse_args();source=args.source.resolve();output=args.output.resolve()
    if output.exists():raise ValueError('派生目录已存在；拒绝覆盖本次或现场成果')
    output.mkdir(parents=True);evidence=output/'证据';evidence.mkdir();artifacts=output/'artifacts';artifacts.mkdir()
    bundle_path=source/'maps/住宅演示地图/current/bundle.json';bundle=json.loads(bundle_path.read_text())
    snapshot=MapSnapshot.model_validate(bundle['snapshot']);map_name=snapshot.map_name
    (evidence/'原规范地图快照.json').write_text(json.dumps(bundle['snapshot'],ensure_ascii=False,indent=2))
    source_db=sqlite3.connect((source/'memory.db').as_uri()+'?mode=ro',uri=True)
    rows=source_db.execute('select payload_json from semantic_memories where map_name=?',(map_name,)).fetchall();source_db.close()
    provenance={'mode':'历史已存地图语义重放；不是当前现场','source':str(source),'map_bundle_path':str(bundle_path),'observations':[]}
    selected=[]
    for (raw,) in rows:
        entry=SemanticMemoryEntry.model_validate_json(raw);reasons=[];matching=[]
        if entry.pose is None:reasons.append('missing_observer_pose')
        frame_time=entry.metadata.get('frame_timestamp');camera=entry.metadata.get('camera_id')
        if not frame_time:reasons.append('missing_frame_time')
        if entry.pose is not None and frame_time and abs((entry.pose.timestamp-iso(frame_time)).total_seconds())>1:reasons.append('pose_frame_delta_exceeds_1s')
        if entry.metadata.get('map_source_name')!=snapshot.source_name or entry.metadata.get('map_source_epoch')!=snapshot.source_epoch:reasons.append('different_canonical_source')
        for artifact_id in entry.artifact_ids:
            meta=next(iter((source/'artifacts').glob('**/'+artifact_id+'.meta.json')),None)
            if meta is None:continue
            ref=json.loads(meta.read_text());md=ref.get('metadata',{});image=next((p for p in meta.parent.glob(artifact_id+'.*') if not p.name.endswith('.meta.json')),None)
            if (image and camera and md.get('camera_id')==camera and frame_time and md.get('frame_timestamp')
                    and abs((iso(md['frame_timestamp'])-iso(frame_time)).total_seconds())<=.001):
                matching.append({'artifact_id':artifact_id,'path':str(image),'size_bytes':image.stat().st_size,'frame_timestamp':md['frame_timestamp']})
                for file in (image,meta):
                    dest=artifacts/file.relative_to(source/'artifacts');dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(file,dest)
        if not matching:reasons.append('no_matching_historical_image')
        provenance['observations'].append({'memory_id':entry.memory_id,'observation_id':entry.observation_id,'pose':entry.pose.model_dump(mode='json') if entry.pose else None,
            'frame_timestamp':frame_time,'images':matching,'excluded':reasons})
        if not reasons:selected.append(entry)
    (evidence/'历史观察来源与排除原因.json').write_text(json.dumps(provenance,ensure_ascii=False,indent=2))
    config=SceneMappingConfig(enabled=True,base_url=args.base_url,max_evidence_age_sec=86400,timeout_sec=180)
    (evidence/'实际采纳参数.json').write_text(json.dumps({k:v for k,v in vars(config).items() if k!='api_key'},ensure_ascii=False,indent=2))
    calls=[];events=[]
    def construct():
        memory=MemoryService(localization_service=SimpleNamespace(get_active_session=lambda:None,get_latest_snapshot=lambda:None),
            mapping_service=SimpleNamespace(get_latest_snapshot=lambda:snapshot),observation_service=SimpleNamespace(),perception_service=SimpleNamespace(),
            state_store=SimpleNamespace(write=lambda *a,**k:None),memory_db_path=str(output/'memory.db'),
            text_embedding_model="disabled",image_embedding_model="disabled",artifact_store=LocalArtifactStore(str(artifacts),'http://historical-replay.invalid'))
        memory.activate_memory_library(library_name=map_name)
        actual=create_scene_decision_backend(backend=config.backend,base_url=config.base_url,model=config.model,api_key='',timeout_sec=config.timeout_sec)
        class RecordedBackend:
            def decide(self,request):
                index=len(calls)+1;(evidence/f'模型请求_{index}.json').write_text(request.model_dump_json(indent=2))
                result=actual.decide(request);calls.append({'request':index,'result':result.model_dump(mode='json')})
                (evidence/f'模型真实结果_{index}.json').write_text(result.model_dump_json(indent=2));return result
            def close(self):actual.close()
            def describe(self):return actual.describe()
        service=SceneMappingService(memory_service=memory,config=config,backend=RecordedBackend(),
            stream_bus=SimpleNamespace(publish=lambda topic,**values:events.append({'topic':topic,'payload':values})))
        return memory,service
    def close(memory,service):
        service.close()
        if not memory.close():raise RuntimeError('记忆关闭未完成，保留派生证据并报告失败')
    memory,service=construct()
    try:
        for entry in selected:memory._repository.upsert_semantic_memory(entry)
        memory._load_persisted_memory(library_name=map_name)
        initial=service.run_once();automatic=service.get_map();graph=service.graph.query()
        for name,value in [('自动命名回执',initial),('自动候选与区域',automatic),('自动提交后关系图',graph)]:
            (evidence/(name+'.json')).write_text(json.dumps(value,ensure_ascii=False,indent=2,default=str))
    finally:close(memory,service)
    memory,service=construct()
    try:
        cold=service.get_map();(evidence/'自动提交冷恢复.json').write_text(json.dumps(cold,ensure_ascii=False,indent=2))
        candidates=[r for r in cold['regions'] if r['metadata'].get('last_decision')]
        if candidates:
            chosen=candidates[0]
            runtime=SimpleNamespace(scene_mapping_service=service,capability_registry=CapabilityRegistry())
            register_scene_mapping_capabilities(runtime)
            args={'scene_space_id':chosen['scene_space_id'],'map_name':map_name,'expected_revision':chosen['metadata']['scene_revision'],
                'label':'重放验证显式名称','request_id':'historical-replay-explicit-1'}
            reply=runtime.capability_registry.get_registration('update_scene_space_label').handler(args,requested_by='isolated-replay-test')
            (evidence/'同公开工具显式覆盖.json').write_text(json.dumps({'arguments':args,'scene_space':reply['scene_space'].model_dump(mode='json')},ensure_ascii=False,indent=2))
            before=len(calls);protected=service.run_once(background=True)
            (evidence/'自动保护回执.json').write_text(json.dumps({'response':protected,'model_calls_before':before,'model_calls_after':len(calls)},ensure_ascii=False,indent=2))
    finally:close(memory,service)
    memory,service=construct()
    try:
        final=service.get_map();final_graph=service.graph.query()
        (evidence/'最终冷恢复与关系图.json').write_text(json.dumps({'map':final,'graph':final_graph},ensure_ascii=False,indent=2))
        assert not candidates or any(r['primary_label']=='重放验证显式名称' and r['metadata']['label_locked'] for r in final['regions'])
    finally:close(memory,service)
    summary={'mode':provenance['mode'],'map_name':map_name,'source_entries':len(rows),'valid_photo_entries':len(selected),'model_calls':len(calls),
        'region_count':initial['region_count'],'processed_ids':initial['processed_ids'],'applied_ids':initial['applied_ids'],
        'decisions':[{'region_id':r['scene_space_id'],'label':r['primary_label'],'decision':r['metadata']['last_decision']} for r in automatic['regions'] if r['metadata'].get('last_decision')],
        'explicit_protection_cold_restore_verified':bool(candidates),'production_code_modified':False}
    (output/'验证汇总.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
    (evidence/'统一提交通知.json').write_text(json.dumps(events,ensure_ascii=False,indent=2,default=str))
    print(json.dumps({k:v for k,v in summary.items() if k!='decisions'},ensure_ascii=False,indent=2))


if __name__=='__main__':main()
