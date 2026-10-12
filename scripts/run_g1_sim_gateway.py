"""用独立数据目录启动G1动作仿真接入；只使用外部设备HTTP接口。"""
from dataclasses import replace
from pathlib import Path
import argparse
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device-url", default="http://127.0.0.1:18910")
    parser.add_argument("--port", type=int, default=18911)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    from core.settings import load_config, ManipulationConfig
    from drivers.customrobot import create_customrobot_assembly
    from gateways.runtime import GatewayRuntime
    from gateways.app import create_gateway_app
    import uvicorn

    config = load_config()
    config.robot.active_robot = "customrobot"
    config.geometry.enabled = False
    config.runtime_data.map_active_restore_on_start = False
    config.runtime_data.map_catalog_root = str(args.output / "maps")
    config.runtime_data.memory_db_path = str(args.output / "memory.db")
    config.runtime_data.memory_embedding_model = "disabled"
    config.runtime_data.memory_embedding_dimension = 128
    config.runtime_data.memory_image_embedding_model = "disabled"
    config.gateway.artifact_dir = str(args.output / "artifacts")
    config.gateway.host = "127.0.0.1"
    config.gateway.port = args.port
    config.gateway.public_base_url = f"http://127.0.0.1:{args.port}"
    config.perception.yolo_world.enabled = False
    config.perception.openai_vision.enabled = False
    config.perception.segmentation.enabled = False
    config.perception.stream_runtime.enabled = False
    config.perception.stream_runtime.auto_start = False
    config.scene_mapping.enabled = False
    config.manipulation = ManipulationConfig(enabled=True, base_url=args.device_url)
    robot = create_customrobot_assembly(config)
    robot.manifest = replace(robot.manifest, robot_name="g1_sim", robot_model="G1 · 三指手动作仿真",
        robot_profile=replace(robot.manifest.robot_profile, robot_name="g1_sim"),
        capability_matrix=robot.manifest.capability_matrix.model_copy(update={"robot_model": "G1 · 三指手动作仿真"}),
        description="由独立物理平台提供实际操作任务，未接入机体导航和雷达。")
    runtime = GatewayRuntime(config=config, robot=robot)
    uvicorn.run(create_gateway_app(runtime, config), host="127.0.0.1", port=args.port)


if __name__ == "__main__":
    main()
