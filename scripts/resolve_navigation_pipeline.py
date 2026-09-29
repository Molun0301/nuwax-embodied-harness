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


def actuator_limits():
    # 与网关共用纯配置实现，避免导入core包时初始化全部契约和运行时。
    spec = importlib.util.spec_from_file_location(
        "nuwax_startup_config", Path(__file__).resolve().parents[1] / "core/config_values.py"
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("无法读取共同配置解析器")
    config = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(config)
    if config.runtime_value("NUWAX_ACTIVE_ROBOT", "").strip().lower() != "go2":
        return "", ""
    values = []
    for key, default in [("GO2_OFFICIAL_MAX_LINEAR_VELOCITY_MPS", "1.0"),
                         ("GO2_OFFICIAL_MAX_YAW_RATE_RPS", "0.8")]:
        value = float(config.runtime_value(key, default))
        if not math.isfinite(value) or value <= 0:
            raise ValueError("Go2速度上限必须为有限正值")
        values.append(str(value))
    return tuple(values)


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
            linear, angular = actuator_limits()
            print("NUWAX_NAV_PROFILE=" + profile)
            print("_NUWAX_NAV_ACTUATOR_MAX_LINEAR_MPS=" + linear)
            print("_NUWAX_NAV_ACTUATOR_MAX_YAW_RPS=" + angular)
            # Go2官方Move接口声明前后、左右及偏航通道；不推测制动参数。
            print("_NUWAX_NAV_ACTUATOR_MOTION_MODEL=" + ("omnidirectional" if linear and profile == "pct-scan-3d" else ""))
        else:
            print(profile)
    except ValueError as error:
        print(f"导航执行链配置错误：{error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
