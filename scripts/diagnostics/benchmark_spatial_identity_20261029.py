#!/usr/bin/env python3
"""离线测量G1自体过滤、主体支持选择和500实例的一批关联，不启动网关或设备。"""
from __future__ import annotations
import argparse
from dataclasses import replace
from datetime import datetime,timezone
import json
from pathlib import Path
import sys
from time import perf_counter
import numpy as np

ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from core.robot_kinematics import load_robot_kinematic_model
from contracts.geometry import FrameTree,Transform,Vector3,Pose
from contracts.image import ImageFrame,CameraInfo
from contracts.perception import Detection2D,BoundingBox2D
from services.localization.frame_tree_ops import _matrix_to_quaternion
from services.perception.self_body import KinematicSelfBodyFilter,JointStateSample
from services.perception.support_points import select_object_support
from services.memory.vision_to_map_projection_service import ProjectedObservation
from services.memory.instance_association_service import InstanceAssociationService


def measure(callback,iterations):
    for _ in range(10):callback()
    samples=[]
    for _ in range(iterations):
        start=perf_counter();callback();samples.append((perf_counter()-start)*1000)
    return {'iterations':iterations,'median_ms':float(np.median(samples)),'p95_ms':float(np.quantile(samples,.95)),
        'min_ms':min(samples),'max_ms':max(samples)}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--iterations',type=int,default=100)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();now=datetime.now(timezone.utc)
    model=load_robot_kinematic_model('/home/ml/Deeprob/NEH_sim_test_platform/data/robots/g1_dex3/robot.urdf',base_link='imu_in_torso')
    joints={key:0. for key in model.actuated_joint_names}
    joints.update(left_elbow_joint=1.1,right_elbow_joint=1.1,left_shoulder_roll_joint=-.15,right_shoulder_roll_joint=.15)
    camera=model.link_transforms(joints)['d435_link'].copy()
    camera[:3,:3]=camera[:3,:3] @ np.array(((0,0,1),(-1,0,0),(0,-1,0)))
    tree=FrameTree(root_frame_id='base_link',transforms=[Transform(parent_frame_id='base_link',child_frame_id='camera',
        translation=Vector3(**dict(zip('xyz',camera[:3,3]))),rotation=_matrix_to_quaternion(camera[:3,:3]))])
    image=ImageFrame(camera_id='cam',frame_id='camera',width_px=640,height_px=480,encoding='jpeg',data=b'x',timestamp=now)
    info=CameraInfo(camera_id='cam',frame_id='camera',width_px=640,height_px=480,fx=320,fy=320,cx=320,cy=240)
    detection=Detection2D(label='person',score=.9,bbox=BoundingBox2D(x_px=180,y_px=180,width_px=280,height_px=260))
    filtering=KinematicSelfBodyFilter(model=model,model_error=None,joint_state_source=lambda:JointStateSample(now,joints),
        coverage_threshold=.5,dilation_px=6,max_joint_age_sec=.5,static_regions={})
    result={'g1_model':{'links':len(model.link_elements),'actuated_joints':len(joints),'collision_hulls':len(model.hulls)},
        'self_filter_frame':measure(lambda:filtering.apply((detection,),image_frame=image,camera_info=info,frame_tree=tree),args.iterations)}
    rng=np.random.default_rng(7);points=rng.normal([0,0,2],[.2,.2,.02],size=(20000,3));pixels=rng.uniform(0,640,(20000,2))
    result['support_selection_20000_points']=measure(lambda:select_object_support(candidate_indices=np.arange(len(points)),
        camera_points=points,projected_pixels=pixels,bbox_xyxy=(0,0,640,640),from_mask=False,max_points=24),args.iterations)
    policy=InstanceAssociationService(text_embedder=None)
    obs=ProjectedObservation('cup',.9,Pose(frame_id='map',position=Vector3(x=0,y=0,z=1)),
        'cam',None,'object','benchmark',(),None,None,(),('cup',),
        {'projection_method':'detection3d_pose','size_x_m':.08,'size_y_m':.08,'size_z_m':.12})
    initial,outcomes=policy.associate([obs],existing_instances={},map_name='house',now=now,admission='explicit')
    first=outcomes[0].instance
    instances={first.instance_id[:-8]+('%08x'%i):first.model_copy(update={'instance_id':first.instance_id[:-8]+('%08x'%i),
        'pose':Pose(frame_id='map',position=Vector3(x=i*2.,y=0,z=1))}) for i in range(500)}
    for count in (1,16):
        batch=[replace(obs,pose=Pose(frame_id='map',position=Vector3(x=i*2.,y=0,z=1)),source_observation_id='next') for i in range(count)]
        def associate_batch():
            updated,outcomes=policy.associate(batch,existing_instances=instances,map_name='house',now=now,admission='explicit')
            policy.consolidate(updated,candidate_ids=[outcome.instance.instance_id for outcome in outcomes if outcome.instance])
        result['association_500_instances_'+str(count)+'_detections']=measure(associate_batch,args.iterations)
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(result,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
