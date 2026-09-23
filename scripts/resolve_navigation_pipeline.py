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


def main() -> int:
    try:
        if sys.argv[1:] == ["--max-linear-speed"]:
            # 设备装配只提供已配置的命令上限，不伪造加速度或制动标定。
            sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
            from core.config_values import runtime_value
            if runtime_value("NUWAX_ACTIVE_ROBOT", "").strip().lower() == "go2":
                value = float(runtime_value("GO2_OFFICIAL_MAX_LINEAR_VELOCITY_MPS", "0.35"))
                if not math.isfinite(value) or value <= 0:
                    raise ValueError("Go2最大线速度必须为有限正值")
                print(value)
            else:
                print("")
            return 0
        profile = _PIPELINE.resolve_navigation_profile(
            os.environ.get("NUWAX_GEOMETRY_PIPELINE"), os.environ.get("NUWAX_NAV_PROFILE")
        )
    except ValueError as error:
        print(f"导航执行链配置错误：{error}", file=sys.stderr)
        return 1
    print(profile)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
