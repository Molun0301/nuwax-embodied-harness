#!/usr/bin/env python3
"""只读历史观察摘要，对照基线与现行关联；不修改记录，不伪造缺失的原始几何。"""
from __future__ import annotations

import argparse
from collections import defaultdict
from datetime import datetime
import json
from pathlib import Path
import sqlite3
import subprocess
import sys
import types
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from contracts.geometry import Pose
from contracts.spatial_memory import SemanticInstance,InstanceConfirmationState
from services.memory.instance_association_service import InstanceAssociationService
from services.memory.vision_to_map_projection_service import ProjectedObservation
from services.memory.instance_evidence import (cross_label_geometry_diagnostics,cooccurrence_evidence,
    instance_identity_ids,measurement_sigma_m,geometry_overlap_diagnostics)

DEFAULT_DB='/home/ml/Deeprob/NEH_sim_test_platform/data/导航探索_20261027/20261029_100924/memory.db'
TARGETS={'bed_bench_suitcase':(('bed','bench','suitcase'),(1.,-1.42)),
         'cup_frisbee':(('cup','frisbee'),(2.35,-.79)),
         'book_remote':(('book','remote'),(1.66,-1.72))}


def load_observations(path):
    with sqlite3.connect('file:'+str(path.resolve())+'?mode=ro&immutable=1',uri=True) as connection:
        rows=connection.execute('SELECT payload_json FROM observation_events ORDER BY created_at,event_id').fetchall()
    records=[]
    for raw, in rows:
        event=json.loads(raw);meta=event.get('metadata') or {}
        label=event['title'][3:] if event['title'].startswith('观察到') else (event.get('visual_labels') or ['unknown'])[0]
        attrs={key:meta[key] for key in ('projection_method','projection_confidence','support_point_count',
            'measurement_sigma_m','depth_m','triangulation_residual_m',
            'ray_origin_map','ray_dir_map','ray_cone_half_angle_rad') if key in meta}
        sizes=[meta.get('extent_'+axis+'_m') for axis in 'xyz']
        pose=Pose.model_validate(event['pose'])
        if all(isinstance(value,(int,float)) and value>=0 for value in sizes) and max(sizes)>0:
            center=[meta.get('centroid_map_'+axis,getattr(pose.position,axis)) for axis in 'xyz']
            attrs['bbox_3d_map']={'frame_id':pose.frame_id,
                'min':{axis:center[i]-sizes[i]/2 for i,axis in enumerate('xyz')},
                'max':{axis:center[i]+sizes[i]/2 for i,axis in enumerate('xyz')}}
        item=ProjectedObservation(label=label,score=float(meta.get('projection_confidence') or .5),pose=pose,
            camera_id=event.get('camera_id'),track_id=None,instance_type='object',
            source_observation_id=event['source_observation_id'],source_artifact_ids=(),semantic_region_id=None,
            topo_node_id=None,semantic_labels=(),visual_labels=(label,),attributes=attrs)
        stamp=datetime.fromisoformat(event['timestamp'].replace('Z','+00:00'))
        records.append((stamp,event.get('map_name'),item))
    return sorted(records,key=lambda row:row[0])


def pair_diagnostics(service,left,right,*,anchors=(),now=None):
    """解释实际空间门限及跨标签拒绝；不改判定、不推造摘要缺失的深度。"""
    left_geometry,right_geometry=service._spatial_geometry(left),service._spatial_geometry(right)
    details={"left_label":left.label,"right_label":right.label,
        "left_id":getattr(left,"instance_id",None),"right_id":getattr(right,"instance_id",None),
        "cross_label":left.label.strip().lower()!=right.label.strip().lower()}
    if left_geometry is None or right_geometry is None:
        return {**details,"reason":"spatial_geometry_unavailable"}
    deltas=[abs(left_geometry[0][i]-right_geometry[0][i]) for i in range(3)]
    gates=[.5*max(left_geometry[1][i],right_geometry[1][i])
        +service._association_config.uncertainty_gate_sigma*float(np.hypot(left_geometry[2],right_geometry[2])) for i in range(3)]
    details.update(left_extent_center=list(left_geometry[0]),right_extent_center=list(right_geometry[0]),
        left_size_m=list(left_geometry[1]),right_size_m=list(right_geometry[1]),
        delta_xyz_m=deltas,gate_xyz_m=gates,left_sigma_m=left_geometry[2],right_sigma_m=right_geometry[2],
        failed_gate_axes=[axis for i,axis in enumerate('xyz') if deltas[i]>gates[i] and not np.isclose(deltas[i],gates[i],rtol=1e-12,atol=0.)])
    geometry=cross_label_geometry_diagnostics(left,right,service._association_config)
    details['cross_geometry']=geometry
    veto=cooccurrence_evidence(left,right,service._association_config)
    if not any(instance_identity_ids(right) & instance_identity_ids(anchor) for anchor in anchors):
        veto=next(({'anchor_id':anchor.instance_id,**proof} for anchor in anchors
                   for proof in (cooccurrence_evidence(anchor,right,service._association_config),) if proof is not None),veto)
    if veto:
        return {**details,'reason':veto['reason'],'cooccurrence':veto}
    if service._conflicting_appearance(left,right):
        return {**details,'reason':'appearance_conflict'}
    if details['failed_gate_axes']:
        return {**details,'reason':'measurement_uncertainty_gate'}
    if (getattr(left,'confirmation_state',None)==getattr(right,'confirmation_state',None)==InstanceConfirmationState.TENTATIVE):
        return {**details,'reason':'both_tentative'}
    if details['cross_label'] and not geometry['compatible']:
        return {**details,'reason':geometry['reason']}
    return {**details,'reason':'compatible'}


def target_records(instances,name):
    labels,center=TARGETS[name]
    return [instance for instance in instances.values()
        if set(labels) & {instance.label,*instance.semantic_labels,*instance.visual_labels}
        and np.linalg.norm(np.array([instance.pose.position.x,instance.pose.position.y])-center)<.3]


def frame_pair_statistics(records):
    """只按原摘要的实测范围分类同帧异标签对；缺几何单列，不反推真值。"""
    policy=InstanceAssociationService(text_embedder=None)
    groups={}
    for _stamp,map_name,item in records:
        groups.setdefault((item.source_observation_id,map_name,item.camera_id),[]).append(item)
    counts=dict(overlapping=0,nonoverlapping=0,appearance_conflict=0,unknown=0)
    rows=[]
    for (source,map_name,camera),items in groups.items():
        for i,left in enumerate(items):
            for right in items[i+1:]:
                if left.label.strip().lower()==right.label.strip().lower():
                    continue
                details=geometry_overlap_diagnostics(left,right,policy._association_config)
                if policy._spatial_geometry(left) is None or policy._spatial_geometry(right) is None:
                    kind='unknown'
                elif details['compatible']:
                    kind='appearance_conflict' if policy._conflicting_appearance(left,right) else 'overlapping'
                else:
                    kind='nonoverlapping' if details['reason'] in ('cross_label_size_ratio','cross_label_containment') else 'unknown'
                counts[kind]+=1
                rows.append({'source_observation_id':source,'map_name':map_name,'camera_id':camera,
                    'labels':[left.label,right.label],'classification':kind,'geometry':details,
                    'delta_xyz_m':[abs(getattr(left.pose.position,axis)-getattr(right.pose.position,axis)) for axis in 'xyz']})
    return {'counts':counts,'total_pairs':len(rows),'pairs':rows,
        'note':'几何来自历史摘要范围，非原始主体点；重合不等于已证明真实物体相同，缺外观信息不能证明无外观冲突。'}


def replay(service,records,admission=None):
    groups={}
    for stamp,map_name,item in records:
        groups.setdefault((item.source_observation_id,map_name),[]).append((stamp,item))
    instances={};assignments=[];merge_count=0
    rejections={name:[] for name in TARGETS}
    for (_source,map_name),items in groups.items():
        now=max(stamp for stamp,item in items)
        kwargs={'admission':admission} if admission else {}
        if admission:
            for _stamp,item in items:
                for name,(labels,center) in TARGETS.items():
                    if item.label not in labels or np.linalg.norm(np.array([item.pose.position.x,item.pose.position.y])-center)>=.3:
                        continue
                    candidates=target_records(instances,name)
                    anchors=service._identity_anchors(item,tuple(candidates),now,None,service._spatial_geometry)
                    for candidate in candidates:
                        details=pair_diagnostics(service,item,candidate,anchors=anchors,now=now)
                        if details['reason']!='compatible':
                            rejections[name].append({'source_observation_id':item.source_observation_id,'timestamp':now.isoformat(),**details})
        instances,outcomes=service.associate([item for stamp,item in items],existing_instances=instances,map_name=map_name,now=now,**kwargs)
        aliases={}
        if admission:
            instances,merges=service.consolidate(instances,candidate_ids=[outcome.instance.instance_id for outcome in outcomes if outcome.instance])
            merge_count+=sum(len(merge.merged_ids) for merge in merges)
            aliases={key:merge.survivor_id for merge in merges for key in merge.merged_ids}
            for entry in assignments:
                entry['id']=aliases.get(entry['id'],entry['id'])
        for (_stamp,item),outcome in zip(items,outcomes):
            if outcome.instance:
                key=aliases.get(outcome.instance.instance_id,outcome.instance.instance_id)
                p=instances[key].pose.position
                assignments.append({'label':item.label,'raw':[getattr(item.pose.position,k) for k in 'xyz'],
                    'id':key,'fused':[getattr(p,k) for k in 'xyz'],
                    'source_observation_id':item.source_observation_id,'camera_id':item.camera_id})
    result={'final_instances':len(instances),'confirmed_instances':sum(getattr(instance,'confirmation_state','confirmed')=='confirmed' for instance in instances.values()),
        'assigned_observations':len(assignments),'unique_ever_ids':len({row['id'] for row in assignments}),'merged_instances':merge_count,'targets':{}}
    for name,(labels,center) in TARGETS.items():
        relevant=[row for row in assignments if row['label'] in labels and np.linalg.norm(np.asarray(row['raw'][:2])-center)<.3]
        keys={row['id'] for row in relevant}
        raw=np.asarray([row['raw'] for row in relevant]);fused=np.asarray([row['fused'] for row in relevant])
        live=[instances[key] for key in keys if key in instances]
        result['targets'][name]={'observations':len(relevant),'ever_ids':len(keys),'final_ids':len(live),
            'confirmed_ids':sum(getattr(instance,'confirmation_state','confirmed')=='confirmed' for instance in live),
            'raw_position_std_m':np.std(raw,axis=0).tolist() if len(raw) else None,
            'accepted_position_std_m':np.std(fused,axis=0).tolist() if len(fused) else None,
            'labels':[instance.label for instance in live]}
        if admission:
            pairs=[pair_diagnostics(service,instances[a],instances[b]) for i,a in enumerate(sorted(keys))
                   if a in instances for b in sorted(keys)[i+1:] if b in instances]
            for pair in pairs:
                if pair['reason'].startswith('coobserved_'):
                    sources={}
                    for row in assignments:
                        if row['id'] in (pair['left_id'],pair['right_id']):
                            source=(row['source_observation_id'],row['camera_id'])
                            sources.setdefault(source,set()).add(row['id'])
                    pair['cooccurrence_examples']=[{'source_observation_id':source,'camera_id':camera}
                        for (source,camera),ids in sources.items() if len(ids)==2]
            result['targets'][name]['remaining_pair_diagnostics']=pairs
            result['targets'][name]['association_rejections']=rejections[name]

    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--database',type=Path,default=Path(DEFAULT_DB))
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    source=subprocess.check_output(['git','show','52e54406:services/memory/instance_association_service.py'],cwd=str(ROOT),text=True)
    module=types.ModuleType('spatial_identity_baseline');sys.modules[module.__name__]=module
    exec(compile(source,'baseline_instance_association.py','exec'),module.__dict__)
    records=load_observations(args.database)
    result={'database':str(args.database),'source_event_count':len(records),'baseline_commit':'52e54406',
        'limitations':['只使用已落盘观察摘要；没有补造支持点、关节、检测分数或跟踪号。',
            '尺寸以记录的跨度、中心重建轴向范围；缺少实际边界中点时只能使用原摘要中心。',
            '标签投票分数沿用记录中的投影置信度，原检测分数未保存。',
            '新规则不能修正旧最近点投影的位置偏差；此回放只验关联层。']}
    sigmas=[measurement_sigma_m(item.attributes) for _stamp,_map,item in records
            if item.attributes.get('projection_method')!='current_pose_fallback']
    result['measurement_uncertainty']={'reliable_observations':len(sigmas),
        'at_5mm_floor':sum(abs(sigma-.005)<1e-12 for sigma in sigmas),
        'with_depth':sum('depth_m' in item.attributes for _stamp,_map,item in records),
        'with_sigma':sum('measurement_sigma_m' in item.attributes for _stamp,_map,item in records),
        'with_view_cone':sum('ray_cone_half_angle_rad' in item.attributes for _stamp,_map,item in records),
        'note':'摘要缺少测量离散度、深度和视锥，回退下限不是传感器精度标定；同标签门限可能偏紧，不能代表新感知管线。'}
    result['same_frame_cross_label_pairs']=frame_pair_statistics(records)
    result['baseline']=replay(module.InstanceAssociationService(text_embedder=None),records)
    result['new_explicit']=replay(InstanceAssociationService(text_embedder=None),records,'explicit')
    result['new_automatic']=replay(InstanceAssociationService(text_embedder=None),records,'automatic')
    with sqlite3.connect('file:'+str(args.database.resolve())+'?mode=ro&immutable=1',uri=True) as connection:
        legacy={}
        for raw, in connection.execute('SELECT payload_json FROM semantic_instances'):
            instance=SemanticInstance.model_validate_json(raw);legacy[instance.instance_id]=instance
    consolidated,merges=InstanceAssociationService(text_embedder=None).consolidate(legacy)
    result['persisted_activation']={'before':len(legacy),'after':len(consolidated),
        'merged_label_groups':[[legacy[key].label for key in [merge.survivor_id,*merge.merged_ids]] for merge in merges]}

    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(result,ensure_ascii=False,indent=2))
    return 0

if __name__=='__main__':raise SystemExit(main())
