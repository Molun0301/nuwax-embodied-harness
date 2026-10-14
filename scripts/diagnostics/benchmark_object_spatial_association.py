#!/usr/bin/env python3
"""离线关联全调用成本：同标签、明确分离场景，仅比较双方均正确的输入。"""
from __future__ import annotations
import argparse
import json
import statistics
import subprocess
import sys
import time
import types
from dataclasses import replace
from datetime import datetime, timedelta, timezone
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from contracts.geometry import Pose, Vector3
from services.memory.instance_association_service import InstanceAssociationService
from services.memory.vision_to_map_projection_service import ProjectedObservation
from services.memory.spatial_candidate_index import SpatialCandidateIndex

NOW = datetime(2026, 10, 14, tzinfo=timezone.utc)


def old_service(baseline_ref):
    source = subprocess.check_output(['git','show',baseline_ref+':services/memory/instance_association_service.py'], cwd=ROOT, text=True)
    module = types.ModuleType('neh_prior_object_association')
    sys.modules[module.__name__] = module
    exec(compile(source, baseline_ref+':instance_association_service.py','exec'), module.__dict__)
    return module.InstanceAssociationService(text_embedder=None)


def fixture(count, detections):
    observation = ProjectedObservation('table',.98,Pose(frame_id='map',position=Vector3(x=1,y=2,z=.5)),
        'front',None,'object','seed',(),None,None,(),('table',),
        {'projection_method':'detection3d_pose','projection_confidence':.99,'size_x_m':.4,'size_y_m':.4,'size_z_m':.6})
    service = InstanceAssociationService(text_embedder=None)
    base = service._create_instance(observation,map_name='house',localization_session_id='session',
        occupancy_grid=None,cost_map=None,now=NOW)
    instances = {}
    for i in range(count):
        identifier = base.instance_id[:-8] + f'{i:08x}'
        pose = Pose(frame_id='map',position=Vector3(x=1+(i%64)*3,y=2+(i//64)*3,z=.5))
        instances[identifier] = base.model_copy(update={'instance_id':identifier,'pose':pose,'observation_window':[]})
    values = list(instances.values())
    chosen = [values[i*count//detections] for i in range(detections)]
    observations = [replace(observation,pose=item.pose,source_observation_id='new') for item in chosen]
    return instances, observations, [item.instance_id for item in chosen]


def run(service, instances, observations):
    return service.associate(observations,existing_instances=instances,map_name='house',
        localization_session_id='session',now=NOW+timedelta(seconds=1))


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--iterations',type=int,default=7)
    parser.add_argument('--baseline-ref',default='e1873185084b5bb51cec9328e57fae0493ceefc3')
    args=parser.parse_args()
    baseline_ref=subprocess.check_output(['git','rev-parse',args.baseline_ref],cwd=ROOT,text=True).strip()
    baseline=old_service(baseline_ref);candidate=InstanceAssociationService(text_embedder=None)
    results=[]
    for history in (32,2048):
        for detections in (1,16):
            instances,observations,expected=fixture(history,detections)
            samples={'prior':[],'extent_index':[]}
            for i in range(args.iterations+1):
                pairs=[('prior',baseline),('extent_index',candidate)]
                for name,service in pairs if i % 2 == 0 else reversed(pairs):
                    start=time.perf_counter();updated,outcomes=run(service,instances,observations)
                    elapsed=time.perf_counter()-start
                    assert len(updated)==history and [item.instance.instance_id for item in outcomes]==expected
                    assert all(not item.created and item.instance.observation_count==2 for item in outcomes)
                    if i:samples[name].append(elapsed*1000)
            timing={name:{'median_total_ms':statistics.median(values),'min_total_ms':min(values),
                         'correct_associations':detections,'history_size':history} for name,values in samples.items()}
            profile={}
            def wrap(owner,name,label):
                original=getattr(owner,name)
                def measured(*a,**kw):
                    start=time.perf_counter()
                    try:return original(*a,**kw)
                    finally:
                        item=profile.setdefault(label,{'calls':0,'ms':0.0})
                        item['calls']+=1;item['ms']+=(time.perf_counter()-start)*1000
                return patch.object(owner,name,side_effect=measured)
            # 单独测量诊断分量；不把有计时钩子的调用混进上面的总耗时中位数。
            constructor=SpatialCandidateIndex.__init__
            def measured_init(obj,*a,**kw):
                start=time.perf_counter()
                constructor(obj,*a,**kw)
                profile['interval_index_build']={'calls':1,'ms':(time.perf_counter()-start)*1000}
            with wrap(candidate,'_spatial_bounds','metric_bounds'),wrap(candidate,'_association_score','candidate_scores'),\
                 wrap(candidate,'_mark_stale','aging'),patch.object(SpatialCandidateIndex,'__init__',measured_init):
                run(candidate,instances,observations)
            with patch.object(baseline,'_association_score',wraps=baseline._association_score) as scores:
                run(baseline,instances,observations)
                profile['prior_candidate_scores']={'calls':scores.call_count}
            results.append({'history':history,'detections':detections,'timing':timing,'profile':profile})
    print(json.dumps({'python':sys.version.split()[0],'iterations':args.iterations,'baseline_ref':baseline_ref,'order':'alternating',
        'scope':'Synthetic unambiguous same-label geometry; full associate includes index, shallow output map, model updates and lifecycle pass. No hardware/model/network.',
        'results':results},indent=2))

if __name__=='__main__':main()
