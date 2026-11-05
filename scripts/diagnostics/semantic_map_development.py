"""把显式离线占据图接入当前语义工作台；Geometry输入是诊断夹具。"""
from pathlib import Path
import argparse
import json
import sys
import numpy as np
import uvicorn
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from core.settings import load_config,SceneMappingConfig
from services.geometry.state import OccupancyGrid
from scripts.diagnostics.current_fixtures import offline_platform
from scripts.diagnostics.recording_inputs import recording_artifact
from contracts.artifacts import ArtifactKind
from gateways.app import create_gateway_app


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--occupancy",type=Path,required=True);parser.add_argument("--output-dir",type=Path,required=True)
    parser.add_argument("--pointcloud",type=Path);parser.add_argument("--port",type=int,default=18792)
    parser.add_argument("--model-url",default="http://127.0.0.1:18791");parser.add_argument("--max-cells",type=int,default=3000000)
    args=parser.parse_args();args.output_dir.mkdir(parents=True,exist_ok=True)
    raw_bytes=args.occupancy.read_bytes();raw=json.loads(raw_bytes)
    if raw["origin_orientation"]!=[0.,0.,0.,1.]:raise ValueError("本诊断要求轴对齐占据图，不隐式旋转源资产")
    config=load_config();config.base_dir=args.output_dir.resolve();config.robot.active_robot="customrobot"
    config.runtime_data.map_catalog_root=str(args.output_dir/"maps");config.runtime_data.memory_embedding_model="lexical-only";config.runtime_data.memory_image_embedding_model="disabled"
    config.gateway.artifact_dir=str(args.output_dir/"artifacts");config.gateway.public_base_url=f"http://127.0.0.1:{args.port}"
    config.perception.yolo_world.enabled=False;config.perception.openai_vision.enabled=False;config.perception.segmentation.enabled=False
    config.scene_mapping=SceneMappingConfig(enabled=True,base_url=args.model_url,max_cells=args.max_cells)
    name="实录离线验证（无相机记录）"
    grid=OccupancyGrid(name,raw["width"],raw["height"],raw["resolution"],raw["origin"][0],raw["origin"][1],raw["frame_id"],round(raw["ts"]*1e9),np.asarray(raw["data"],dtype=np.int8).tobytes())
    with offline_platform(config,grid) as platform:
        store=platform.artifacts._store
        recording_artifact(store,source_path=args.occupancy,kind=ArtifactKind.MAP_SNAPSHOT,mime_type="application/json",data=raw_bytes,extension=".json",map_name=name,frame_id=raw["frame_id"])
        if args.pointcloud is not None:
            cloud=np.fromfile(args.pointcloud,dtype="<f4").reshape(-1,4)[:,:3]
            if not np.isfinite(cloud).all():raise ValueError("展示点云包含非有限坐标")
            artifact=recording_artifact(store,source_path=args.pointcloud,kind=ArtifactKind.POINTCLOUD,mime_type="application/octet-stream",data=cloud.astype("<f4").tobytes(),extension=".xyz",map_name=name,frame_id=raw["frame_id"])
            platform.streams.stream_bus.publish("pointcloud_map.meta",source="offline_recording",frame_id=raw["frame_id"],payload={"url":"/artifacts/"+artifact.artifact_id,"point_count":len(cloud),"map_name":name,"offline_recording":True})
        result=platform.scene_mapping.run_once(apply_labels=False)
        (args.output_dir/"启动结果.json").write_text(json.dumps(result,ensure_ascii=False,indent=2))
        uvicorn.run(create_gateway_app(platform,config,start_runtime_on_lifespan=False),host="127.0.0.1",port=args.port,access_log=False)

if __name__=="__main__":main()
