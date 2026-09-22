#!/usr/bin/env python3
"""把离线实录栅格接入开发机真实语义工作台，运行隔离网关且不连接机器人。"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import uvicorn
from contracts.geometry import Pose, Quaternion, Vector3
from contracts.artifacts import ArtifactKind
import numpy as np
from contracts.maps import OccupancyGrid
from core.settings import SceneMappingConfig, load_config
from drivers.customrobot import create_customrobot_assembly
from gateways.app import create_gateway_app
from gateways.runtime import GatewayRuntime


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--occupancy", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--pointcloud", type=Path, help="已核对同源的四列float32展示点云，第四列不作时间使用")
    parser.add_argument("--port", type=int, default=18792)
    parser.add_argument("--model-url", default="http://127.0.0.1:18791")
    parser.add_argument("--max-cells", type=int, default=3000000)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    config = load_config()
    config.robot.active_robot = "customrobot"
    config.geometry.enabled = False
    config.runtime_data.map_active_restore_on_start = False
    config.runtime_data.mapping_recording_enabled = False
    config.runtime_data.map_catalog_root = str(args.output_dir / "maps")
    config.runtime_data.memory_db_path = str(args.output_dir / "memory.sqlite")
    config.runtime_data.memory_embedding_model = "hashing-v1"
    config.runtime_data.memory_image_embedding_model = "disabled"
    config.gateway.artifact_dir = str(args.output_dir / "artifacts")
    config.gateway.public_base_url = "http://127.0.0.1:%d" % args.port
    config.perception.yolo_world.enabled = False
    config.perception.openai_vision.enabled = False
    config.perception.stream_runtime.enabled = False
    config.perception.stream_runtime.auto_start = False
    config.scene_mapping = SceneMappingConfig(enabled=True, base_url=args.model_url, max_cells=args.max_cells)
    robot = create_customrobot_assembly(config)
    runtime = GatewayRuntime(config=config, robot=robot)
    app = create_gateway_app(runtime, config, start_runtime_on_lifespan=False)
    raw_bytes = args.occupancy.read_bytes()
    raw = json.loads(raw_bytes)
    map_name = "9月18日15:12实录离线验证（无相机记录）"
    timestamp = datetime.fromtimestamp(raw["ts"], timezone.utc)
    origin = Pose(frame_id=raw["frame_id"], position=Vector3(**dict(zip(("x", "y", "z"), raw["origin"]))),
                  orientation=Quaternion(**dict(zip(("x", "y", "z", "w"), raw["origin_orientation"]))))
    try:
        runtime.memory_service.activate_memory_library(library_name=map_name)
        runtime.mapping_service.ingest_external_snapshot(map_name=map_name, source_name="offline_recording_production_horizontal",
            source_epoch="offline-" + hashlib.sha256(raw_bytes).hexdigest()[:24], update_sequence_override=10434,
            occupancy_grid=OccupancyGrid(map_id=map_name, frame_id=raw["frame_id"], width=raw["width"],
                height=raw["height"], resolution_m=raw["resolution"], origin=origin, data=raw["data"], timestamp=timestamp),
            metadata={"offline_validation": True, "camera_recording_available": False, "map_snapshot_kind": "offline_recording", "map_name": map_name})
        if args.pointcloud is not None:
            cloud = np.fromfile(args.pointcloud, dtype="<f4").reshape(-1, 4)[:, :3]
            if not np.isfinite(cloud).all():
                raise ValueError("展示点云包含非有限坐标")
            cloud_bytes = cloud.astype("<f4").tobytes()
            cloud_sha = hashlib.sha256(cloud_bytes).hexdigest()
            artifact = next((ref for ref in runtime.artifact_store.list_refs(kind=ArtifactKind.POINTCLOUD)
                             if ref.sha256 == cloud_sha), None)
            if artifact is None:
                artifact = runtime.artifact_store.save_bytes(kind=ArtifactKind.POINTCLOUD,
                    mime_type="application/octet-stream", data=cloud_bytes, extension=".xyz",
                    metadata={"map_name": map_name, "frame_id": raw["frame_id"], "offline_recording": True,
                              "source_path": str(args.pointcloud.resolve())})
            runtime.stream_bus.publish("pointcloud_map.meta", source="saved",
                frame_id=raw["frame_id"], status="available", payload={
                    "url": "/artifacts/" + artifact.artifact_id, "lod": "preview", "point_count": len(cloud),
                    "map_name": map_name, "frame_id": raw["frame_id"], "offline_recording": True})
        result = runtime.scene_mapping_service.run_once(apply_labels=False)
        (args.output_dir / "启动结果.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
        uvicorn.run(app, host="127.0.0.1", port=args.port, access_log=False)
    finally:
        runtime.scene_mapping_service.close()
        runtime.memory_service.disable_memory_library()
        runtime.task_manager.shutdown(wait=True)
        runtime.memory_service._repository.close()
        runtime.memory_service._vector_store.close()
        runtime.stop()


if __name__ == "__main__":
    main()
