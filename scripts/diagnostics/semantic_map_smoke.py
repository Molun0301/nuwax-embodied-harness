#!/usr/bin/env python3
"""以合成地图和观察验证真实网关、记忆仓储及独立OpenJev服务，不连接机器人。"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from fastapi.testclient import TestClient
from contracts.base import utc_now
from contracts.geometry import Pose, Vector3
from contracts.maps import OccupancyGrid
from contracts.naming import build_observation_id
from contracts.perception import Observation
from contracts.runtime_views import LocalizationSnapshot, PerceptionContext, SceneObjectSummary, SceneSummary
from core.settings import SceneMappingConfig, load_config
from drivers.customrobot import create_customrobot_assembly
from gateways.app import create_gateway_app
from gateways.runtime import GatewayRuntime


def run(output_dir: Path, model_url: str) -> dict:
    output_dir.mkdir(parents=True, exist_ok=True)
    config = load_config()
    config.robot.active_robot = "customrobot"
    config.geometry.enabled = False
    config.runtime_data.map_active_restore_on_start = False
    config.runtime_data.mapping_recording_enabled = False
    config.runtime_data.map_catalog_root = str(output_dir / "maps")
    config.runtime_data.memory_db_path = str(output_dir / "memory.sqlite")
    config.runtime_data.memory_embedding_model = "lexical-only"
    config.runtime_data.memory_image_embedding_model = "disabled"
    config.gateway.artifact_dir = str(output_dir / "artifacts")
    config.perception.yolo_world.enabled = False
    config.perception.openai_vision.enabled = False
    config.perception.stream_runtime.enabled = False
    config.perception.stream_runtime.auto_start = False
    config.scene_mapping = SceneMappingConfig(enabled=True, backend="openjev_http", base_url=model_url)
    robot = create_customrobot_assembly(config)
    runtime = GatewayRuntime(config=config, robot=robot)
    app = create_gateway_app(runtime, config, start_runtime_on_lifespan=False)
    map_name = "语义双后端合成验证"
    try:
        runtime.memory_service.activate_memory_library(library_name=map_name, reset_library=True)
        cells = [100] * 3600
        for y in range(5, 55):
            for x in range(5, 55):
                cells[y * 60 + x] = 0
        snapshot = runtime.mapping_service.ingest_external_snapshot(map_name=map_name, source_name="semantic_smoke",
            source_epoch="synthetic-scene-v1", update_sequence_override=1,
            occupancy_grid=OccupancyGrid(map_id=map_name, frame_id="map", width=60, height=60,
                resolution_m=.1, origin=Pose(frame_id="map"), data=cells), metadata={"synthetic": True})
        for x in (2, 3):
            now = utc_now()
            pose = Pose(frame_id="map", position=Vector3(x=x, y=2), timestamp=now)
            context = PerceptionContext(camera_id="front", observation=Observation(
                observation_id=build_observation_id("front"), frame_id="camera", summary="合成办公观察"),
                scene_summary=SceneSummary(headline="房间内有四组独立办公工位、办公桌、显示器和办公椅，主要用于日常办公。",
                    objects=[SceneObjectSummary(label="desk", count=4), SceneObjectSummary(label="monitor", count=4)]),
                pipeline_name="synthetic_evidence", detector_backend="synthetic", tracker_backend="synthetic",
                metadata={"frame_timestamp": now.isoformat(), "frame_timestamp_source": "synthetic.image_time"})
            runtime.memory_service._create_scene_memory_entry(title="合成办公室证据", summary=None, tags=[],
                linked_location=None, camera_id="front", metadata={"synthetic": True},
                perception_context=context, use_observation_snapshot=True, observation_pose=pose,
                use_spatial_snapshot=True, map_snapshot=snapshot,
                localization_snapshot=LocalizationSnapshot(source_name="synthetic", current_pose=pose),
                expected_library_name=map_name)
        with TestClient(app) as client:
            def invoke(name, arguments=None):
                response = client.post("/api/capabilities/%s/invoke" % name, json={"arguments": arguments or {}})
                response.raise_for_status()
                envelope = response.json()
                return envelope.get("result", envelope)

            catalog = client.post("/mcp", json={"jsonrpc": "2.0", "id": 1, "method": "tools/list", "params": {}})
            catalog.raise_for_status()
            assert "refresh_semantic_map" in json.dumps(catalog.json())
            accepted = invoke("refresh_semantic_map", {"apply_labels": True})
            task_id = accepted["task_id"]
            deadline = time.monotonic() + 180
            while True:
                state = runtime.task_manager.get_task_status(task_id).state.value
                if state in ("succeeded", "failed", "cancelled", "timeout"):
                    break
                if time.monotonic() > deadline:
                    raise RuntimeError("语义任务未在验证期限内返回")
                time.sleep(.1)
            if state != "succeeded":
                raise RuntimeError(str(runtime.task_manager.get_task_error_payload(task_id)))
            semantic_map = invoke("get_semantic_map")
            assert semantic_map["region_count"] == 1
            region = semantic_map["regions"][0]
            assert region["metadata"].get("last_decision"), runtime.scene_mapping_service.status()
            renamed = invoke("update_scene_space_label", {"scene_space_id": region["scene_space_id"],
                "map_name": map_name, "expected_revision": region["metadata"]["scene_revision"],
                "label": "开发机验证办公室", "request_id": "smoke-rename"})
            assert renamed["scene_space"]["primary_label"] == "开发机验证办公室"
            report = {"evidence_kind": "synthetic_map_and_observations_with_real_model",
                "model_endpoint": model_url, "task_state": state,
                "http_capabilities": ["refresh_semantic_map", "get_semantic_map", "update_scene_space_label"],
                "mcp_discovery": True, "model_result": region["metadata"]["last_decision"],
                "initial_label": region["primary_label"], "region_count": semantic_map["region_count"],
                "rename_persisted": runtime.memory_service._repository.load_scene_space(region["scene_space_id"]).primary_label,
                "no_robot_connection": True}
            (output_dir / "验证结果.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
            return report
    finally:
        runtime.scene_mapping_service.close()
        runtime.memory_service.disable_memory_library()
        runtime.task_manager.shutdown(wait=True)
        runtime.memory_service._repository.close()
        runtime.memory_service._vector_store.close()
        runtime.stop()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-url", default="http://127.0.0.1:8791")
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    result = run(args.output_dir.resolve(), args.model_url)
    print(json.dumps({key: result[key] for key in ("evidence_kind", "task_state", "initial_label", "region_count", "rename_persisted", "no_robot_connection")}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
