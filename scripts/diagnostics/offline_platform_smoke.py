#!/usr/bin/env python3
"""无机器人验证标准数据接入、真实 HTTP/MCP 调用及网关资源关闭。"""

from __future__ import annotations

import argparse
import json
import logging
from pathlib import Path
import socket
import sys
from tempfile import TemporaryDirectory, mkdtemp
import threading
import time

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import cv2
import httpx
import numpy as np
import uvicorn

from contracts.image import ImageEncoding, ImageFrame
from contracts.robot_state import RobotState
from core.settings import load_config
from drivers.customrobot import create_customrobot_assembly
from gateways.app import create_gateway_app
from gateways.runtime import GatewayRuntime


def run(output_dir: Path) -> dict:
    """只发布显式模拟的静止状态和测试图像，不连接硬件、不加载识别模型。"""

    config = load_config()
    config.robot.active_robot = "customrobot"
    config.geometry.enabled = False
    config.runtime_data.map_active_restore_on_start = False
    config.runtime_data.map_catalog_root = str(output_dir / "maps")
    config.runtime_data.memory_db_path = str(output_dir / "memory.db")
    config.runtime_data.memory_embedding_model = "hashing-v1"
    config.runtime_data.memory_embedding_dimension = 128
    config.runtime_data.memory_image_embedding_model = "disabled"
    config.gateway.artifact_dir = str(output_dir / "artifacts")
    config.perception.yolo_world.enabled = False
    config.perception.openai_vision.enabled = False
    config.perception.segmentation.enabled = False
    config.perception.stream_runtime.enabled = False
    config.perception.stream_runtime.auto_start = False

    startup_errors = []
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        listener.listen(128)
        config.gateway.public_base_url = "http://127.0.0.1:%d" % listener.getsockname()[1]
        robot = create_customrobot_assembly(config)
        runtime = GatewayRuntime(config=config, robot=robot)
        app = create_gateway_app(runtime, config)
        server = uvicorn.Server(uvicorn.Config(app, log_level="error", access_log=False))

        def serve() -> None:
            try:
                server.run(sockets=[listener])
            except BaseException as exc:
                startup_errors.append(exc)

        thread = threading.Thread(target=serve, name="offline-gateway")
        thread.start()
        try:
            deadline = time.monotonic() + 15
            while not server.started:
                if not thread.is_alive() or time.monotonic() >= deadline:
                    raise RuntimeError("离线网关启动失败：%s" % startup_errors)
                time.sleep(0.02)
            robot.providers.publish_robot_state(RobotState(
                robot_id="offline_robot", frame_id=robot.defaults.frame_ids["base"],
                metadata={"synthetic": True},
            ))
            encoded, pixels = cv2.imencode(".png", np.zeros((16, 16, 3), dtype=np.uint8))
            if not encoded:
                raise RuntimeError("测试图像编码失败")
            robot.providers.publish_image(ImageFrame(
                camera_id="front_camera", frame_id=robot.defaults.frame_ids["camera_front"],
                width_px=16, height_px=16, encoding=ImageEncoding.PNG,
                data=pixels.tobytes(), metadata={"synthetic": True},
            ))
            results = []
            with httpx.Client(base_url=config.gateway.public_base_url, timeout=15, trust_env=False) as client:
                def rpc(method, params):
                    response = client.post("/mcp", json={
                        "jsonrpc": "2.0", "id": len(results) + 1, "method": method, "params": params,
                    })
                    response.raise_for_status()
                    payload = response.json()
                    if "error" in payload or payload.get("result", {}).get("isError"):
                        raise RuntimeError(json.dumps(payload, ensure_ascii=False))
                    results.append({"method": method, "function": params.get("arguments", {}).get("function"), "passed": True})
                    return payload["result"]

                rpc("initialize", {})
                catalog = rpc("tools/list", {})
                collections = {
                    function: tool["name"]
                    for tool in catalog["tools"]
                    for function in tool.get("inputSchema", {}).get("properties", {}).get("function", {}).get("enum", [])
                }
                for name, arguments in (
                    ("get_robot_status", {}),
                    ("capture_image", {"camera_id": "front_camera"}),
                    ("perceive_current_scene", {"camera_id": "front_camera", "detector_backend": "metadata_detector"}),
                    ("create_map", {"map_name": "离线验证地图"}),
                    ("list_maps", {}),
                    ("enable_memory_library", {"library_name": "离线验证地图"}),
                ):
                    result = rpc("tools/call", {"name": collections[name], "arguments": {"function": name, "parameters": arguments}})
                    if name == "capture_image":
                        artifact = result["structuredContent"]["result"]["artifact"]
                        response = client.get(artifact["uri"])
                        response.raise_for_status()
                        if response.content != pixels.tobytes():
                            raise RuntimeError("图像制品下载与发布数据不一致")
                        results.append({"method": "GET", "function": "artifact_download", "passed": True})
                if runtime.motion_guard.has_connected:
                    raise RuntimeError("只读验证意外建立了运动连接")
                report = {"passed": True, "synthetic_input": True, "vision_backend": "metadata_detector", "tool_count": len(catalog["tools"]), "checks": results}
                memory_workers = [
                    runtime.memory_service._vectorization_worker,
                    runtime.memory_service._deferred_refine_worker,
                ]
        finally:
            server.should_exit = True
            thread.join(timeout=20)
            if thread.is_alive():
                raise RuntimeError("离线网关未正常退出")
    if runtime._started or runtime.event_bus._consumer_thread.is_alive() or runtime.stream_bus._consumer_thread.is_alive():
        raise RuntimeError("离线网关仍有运行资源未关闭")
    if runtime._log_stream_handler in logging.getLogger().handlers:
        raise RuntimeError("离线网关未释放日志处理器")
    if any(worker is not None and worker.is_alive() for worker in memory_workers):
        raise RuntimeError("离线网关未停止记忆后台工作线程")
    report["shutdown_passed"] = True
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, help="保留验证生成的地图和图像；缺省使用自动清理的临时目录。")
    args = parser.parse_args()
    if args.output_dir:
        args.output_dir.mkdir(parents=True, exist_ok=True)
        run_dir = Path(mkdtemp(prefix="验证-", dir=args.output_dir.resolve()))
        report = run(run_dir)
        report["output_dir"] = str(run_dir)
        (args.output_dir / "离线验证结果.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    else:
        with TemporaryDirectory(prefix="nuwax-offline-") as directory:
            report = run(Path(directory))
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
