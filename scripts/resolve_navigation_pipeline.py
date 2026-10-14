"""启动前解析固定导航配置，不创建运行时或连接机器人。"""

import os
import math
import importlib.util
from pathlib import Path
import sys

# 启动预检只加载纯契约文件，不导入契约包及其运行依赖。
_SPEC = importlib.util.spec_from_file_location(
    "nuwax_navigation_pipeline", Path(__file__).resolve().parents[1] / "contracts/navigation_pipeline.py"
)
if _SPEC is None or _SPEC.loader is None:
    raise RuntimeError("无法读取导航执行链契约")
_PIPELINE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_PIPELINE)


def actuator_configuration():
    # 与网关共用纯配置实现，避免导入core包时初始化全部契约和运行时。
    spec = importlib.util.spec_from_file_location(
        "nuwax_startup_config", Path(__file__).resolve().parents[1] / "core/config_values.py"
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("无法读取共同配置解析器")
    config = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(config)
    keys = ("NUWAX_NAV_ACTUATOR_MAX_LINEAR_MPS", "NUWAX_NAV_ACTUATOR_MAX_YAW_RPS",
            "NUWAX_NAV_ACTUATOR_MOTION_MODEL")
    explicit = tuple(config.runtime_value(key, "").strip() for key in keys)
    if any(explicit):
        if not all(explicit):
            raise ValueError("基本运动声明须同时填写平面速度、偏航速度和运动形式")
        linear, angular, model = explicit
        if model != "omnidirectional":
            raise ValueError("基本运动声明当前仅支持omnidirectional（前后左右移动和原地转向）")
        limits = (float(linear), float(angular))
        if (not all(math.isfinite(value) and value > 0 for value in limits)
                or limits[0] > 5.0 or limits[1] > 4.0 * math.pi):
            raise ValueError("基本平面速度须在(0,5]米/秒，偏航速度须在(0,4π]弧度/秒")
        return str(limits[0]), str(limits[1]), model
    if config.runtime_value("NUWAX_ACTIVE_ROBOT", "").strip().lower() != "go2":
        return "", "", ""
    values = []
    for key, default in [("GO2_OFFICIAL_MAX_LINEAR_VELOCITY_MPS", "1.0"),
                         ("GO2_OFFICIAL_MAX_YAW_RATE_RPS", "0.8")]:
        value = float(config.runtime_value(key, default))
        if not math.isfinite(value) or value <= 0:
            raise ValueError("Go2速度上限必须为有限正值")
        values.append(str(value))
    return values[0], values[1], "omnidirectional"


def actuator_limits():
    # 保留已有单项查询入口；显式配置仍须整体校验。
    return actuator_configuration()[:2]


def main() -> int:
    try:
        arguments = sys.argv[1:]
        if arguments in (["--max-linear-speed"], ["--max-yaw-rate"]):
            linear, angular = actuator_limits()
            print(angular if arguments == ["--max-yaw-rate"] else linear)
            return 0
        if arguments not in ([], ["--runtime-env"]):
            raise ValueError("不支持的启动配置参数")
        profile = _PIPELINE.resolve_navigation_profile(
            os.environ.get("NUWAX_GEOMETRY_PIPELINE"), os.environ.get("NUWAX_NAV_PROFILE")
        )
        if arguments == ["--runtime-env"]:
            linear, angular, model = actuator_configuration()
            print("NUWAX_NAV_PROFILE=" + profile)
            print("_NUWAX_NAV_ACTUATOR_MAX_LINEAR_MPS=" + linear)
            print("_NUWAX_NAV_ACTUATOR_MAX_YAW_RPS=" + angular)
            # 设备声明映射到既有MotionKinematics；不推测制动参数或跨二维链启用。
            print("_NUWAX_NAV_ACTUATOR_MOTION_MODEL=" + (model if profile == "pct-scan-3d" else ""))
        else:
            print(profile)
    except ValueError as error:
        print(f"导航执行链配置错误：{error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
