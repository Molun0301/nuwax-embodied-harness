"""合成地图和观察经过当前装配根与独立语义模型服务，不连接机器人。"""
from pathlib import Path
import argparse
import json
import sys
import time
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from fastapi.testclient import TestClient
from contracts.base import utc_now
from contracts.geometry import Pose,Vector3
from contracts.naming import build_observation_id
from contracts.perception import Observation
from contracts.runtime_views import PerceptionContext,SceneObjectSummary,SceneSummary
from core.settings import load_config,SceneMappingConfig
from gateways.app import create_gateway_app
from services.geometry.state import OccupancyGrid
from scripts.diagnostics.current_fixtures import offline_platform


def run(output_dir:Path,model_url:str):
    output_dir.mkdir(parents=True,exist_ok=True)
    config=load_config();config.base_dir=output_dir;config.robot.active_robot="customrobot"
    config.runtime_data.map_catalog_root=str(output_dir/"maps");config.runtime_data.memory_embedding_model="lexical-only";config.runtime_data.memory_image_embedding_model="disabled"
    config.gateway.artifact_dir=str(output_dir/"artifacts")
    config.perception.yolo_world.enabled=False;config.perception.openai_vision.enabled=False;config.perception.segmentation.enabled=False
    config.scene_mapping=SceneMappingConfig(enabled=True,backend="openjev_http",base_url=model_url)
    cells=np.full((60,60),100,dtype=np.int8);cells[5:55,5:55]=0
    name="语义双后端合成验证";stamp=time.time_ns();grid=OccupancyGrid(name,60,60,.1,0.,0.,"map",stamp,cells.tobytes())
    with offline_platform(config,grid) as platform:
        for x in (2,3):
            now=utc_now();pose=Pose(frame_id="map",position=Vector3(x=x,y=2),timestamp=now)
            context=PerceptionContext(camera_id="front",observation=Observation(observation_id=build_observation_id("front"),frame_id="camera",summary="合成办公观察"),
                scene_summary=SceneSummary(headline="房间内有四组独立办公工位、办公桌、显示器和办公椅，主要用于日常办公。",objects=[SceneObjectSummary(label="desk",count=4),SceneObjectSummary(label="monitor",count=4)]),
                pipeline_name="synthetic_evidence",detector_backend="synthetic",tracker_backend="synthetic",metadata={"frame_timestamp":now.isoformat(),"frame_timestamp_source":"synthetic.image_time","source_timestamp_ns":time.time_ns()})
            platform.memory.remember_current_scene(map_name=name,title="合成办公室证据",perception_context=context,observation_pose=pose,metadata={"synthetic":True})
        with TestClient(create_gateway_app(platform,config,start_runtime_on_lifespan=False)) as client:
            def invoke(name,args=None):
                response=client.post(f"/api/capabilities/{name}/invoke",json={"arguments":args or {}});response.raise_for_status();return response.json()
            catalog=client.post("/mcp",json={"jsonrpc":"2.0","id":1,"method":"tools/list","params":{}})
            assert "refresh_semantic_map" in json.dumps(catalog.json())
            task=invoke("refresh_semantic_map",{"apply_labels":True})["task"];deadline=time.monotonic()+180
            while task["state"] in ("running","cancelling"):
                if time.monotonic()>deadline:raise RuntimeError("语义任务未在验证期限内返回")
                time.sleep(.1);response=client.get("/api/tasks/"+task["task_id"]);response.raise_for_status();task=response.json()["task"]
            assert task["state"]=="succeeded",task
            semantic=invoke("get_semantic_map");assert semantic["region_count"]==1
            region=semantic["regions"][0];assert region["metadata"].get("last_decision"),platform.scene_mapping.status()
            renamed=invoke("update_scene_space_label",{"scene_space_id":region["scene_space_id"],"map_name":name,"expected_revision":region["metadata"]["scene_revision"],"label":"开发机验证办公室","request_id":"smoke-rename"})
            assert renamed["scene_space"]["primary_label"]=="开发机验证办公室"
            report={"evidence_kind":"synthetic_map_and_observations_with_real_model","model_endpoint":model_url,"task_state":task["state"],
                "http_capabilities":["refresh_semantic_map","get_semantic_map","update_scene_space_label"],"mcp_discovery":True,
                "model_result":region["metadata"]["last_decision"],"initial_label":region["primary_label"],"region_count":semantic["region_count"],
                "rename_persisted":platform.memory.get_scene_space(region["scene_space_id"]).primary_label,"no_robot_connection":True}
            (output_dir/"验证结果.json").write_text(json.dumps(report,ensure_ascii=False,indent=2))
            return report


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument("--model-url",default="http://127.0.0.1:8791");parser.add_argument("--output-dir",type=Path,required=True)
    args=parser.parse_args();print(json.dumps(run(args.output_dir.resolve(),args.model_url),ensure_ascii=False,indent=2))

if __name__=="__main__":main()
