#!/usr/bin/env python3
"""只用本地模型和已保存投影区域，比较实跑修正前后的每帧自体判定开销。"""
from __future__ import annotations
import argparse
from datetime import datetime,timezone
import json
from pathlib import Path
import subprocess
import sys
import types
from time import perf_counter
import numpy as np

ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from contracts.geometry import FrameTree,Transform,Vector3
from contracts.image import ImageFrame,CameraInfo
from contracts.perception import Detection2D
from core.robot_kinematics import load_robot_kinematic_model
from services.localization.frame_tree_ops import _matrix_to_quaternion
from services.perception import self_body as current


def measure(callback,iterations):
    for _ in range(10):callback()
    samples=[]
    for _ in range(iterations):
        start=perf_counter();callback();samples.append((perf_counter()-start)*1000.)
    return {'iterations':iterations,'median_ms':float(np.median(samples)),
        'p95_ms':float(np.quantile(samples,.95)),'min_ms':min(samples),'max_ms':max(samples)}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--iterations',type=int,default=200)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    source=subprocess.check_output(['git','show','bff266c7:services/perception/self_body.py'],cwd=str(ROOT),text=True)
    baseline=types.ModuleType('g1_self_body_before_live_fix');sys.modules[baseline.__name__]=baseline
    exec(compile(source,'baseline_self_body.py','exec'),baseline.__dict__)
    sample=ROOT/'docs/开发文档/任务记录/2026-10-29_空间记忆去重与自体识别过滤/实跑帧样本_20261029_1409.json'
    context=json.loads(sample.read_text())['result']['perception_context']
    payload=context['metadata']['detection_metadata']['self_body_region']
    detections=tuple(Detection2D.model_validate(item) for item in context['observation']['detections_2d'])
    def frame_judgment(module):
        region=module.SelfBodyRegion.from_metadata(payload)
        for detection in detections:
            coverage=region.coverage_of_detection(detection)
            if module is current:
                region.bbox_iou_of_detection(detection)
    result={'baseline_commit':'bff266c7','notes':'冷区域缓存每帧重建；同一实跑四框；完整G1另用固定真实模型/关节，均不含通信、推理或首次模型读取。',
        'sample_before':measure(lambda:frame_judgment(baseline),args.iterations),
        'sample_after':measure(lambda:frame_judgment(current),args.iterations)}
    model=load_robot_kinematic_model('/home/ml/Deeprob/NEH_sim_test_platform/data/robots/g1_dex3/robot.urdf',base_link='imu_in_torso')
    joints={key:0. for key in model.actuated_joint_names};joints.update(left_elbow_joint=1.1,right_elbow_joint=1.1,left_shoulder_roll_joint=-.15,right_shoulder_roll_joint=.15)
    camera=model.link_transforms(joints)['d435_link'].copy()
    camera[:3,:3]=camera[:3,:3] @ np.array(((0,0,1),(-1,0,0),(0,-1,0)))
    tree=FrameTree(root_frame_id='base_link',transforms=[Transform(parent_frame_id='base_link',child_frame_id='camera',
        translation=Vector3(**dict(zip('xyz',camera[:3,3]))),rotation=_matrix_to_quaternion(camera[:3,:3]))])
    now=datetime.now(timezone.utc)
    image=ImageFrame(camera_id=context['camera_id'],frame_id='camera',width_px=640,height_px=480,encoding='jpeg',data=b'x',timestamp=now)
    info=CameraInfo(camera_id=context['camera_id'],frame_id='camera',width_px=640,height_px=480,fx=320,fy=320,cx=320,cy=240)
    for name,module in (('before',baseline),('after',current)):
        filter_=module.KinematicSelfBodyFilter(model=model,model_error=None,
            joint_state_source=lambda:current.JointStateSample(now,joints),coverage_threshold=.5,
            dilation_px=6,max_joint_age_sec=.5,static_regions={})
        result['g1_complete_'+name]=measure(lambda:filter_.apply(detections,image_frame=image,camera_info=info,frame_tree=tree),args.iterations)
    result['sample_added_median_ms']=result['sample_after']['median_ms']-result['sample_before']['median_ms']
    result['g1_complete_added_median_ms']=result['g1_complete_after']['median_ms']-result['g1_complete_before']['median_ms']
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(result,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
