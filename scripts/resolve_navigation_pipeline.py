"""启动前解析固定导航配置，不创建运行时或连接机器人。"""

import os
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
