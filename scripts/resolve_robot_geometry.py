"""为几何与导航启动器输出同一份机器人外形，不启动任何运行时。"""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from core.config_values import runtime_value
from core.robot_geometry import ROBOT_GEOMETRY_SETTING_KEYS, RobotGeometryConfig, resolve_robot_geometry


def main() -> int:
    values = {key: runtime_value(key, "") for key in ROBOT_GEOMETRY_SETTING_KEYS | {"NUWAX_NAV_ROBOT_WIDTH_M"}}
    result = resolve_robot_geometry(
        RobotGeometryConfig.from_values(values),
        robot_name=runtime_value("NUWAX_ACTIVE_ROBOT", ""),
        base_dir=ROOT,
    )
    print("机器人外形已解析：来源={}，长宽高={:.4g}×{:.4g}×{:.4g}米，中心={}。".format(
        result.source, result.length_m, result.width_m, result.height_m, result.center_xyz_m,
    ), file=sys.stderr)
    for warning in result.warnings:
        print("机器人外形：" + warning, file=sys.stderr)
    print(result.to_runtime_json())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
