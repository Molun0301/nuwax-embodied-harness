#!/usr/bin/env python3
"""用生产解析器生成跨语言资格夹具；仅包含模型数学坐标，不声明实机标定。"""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("fixed_self_fixture_geometry", ROOT / "core/robot_geometry.py")
geometry = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = geometry
spec.loader.exec_module(geometry)

GENERIC = '''<robot name="nonroot_fixed_test">
<link name="root"><collision><geometry><sphere radius="0.1"/></geometry></collision></link>
<link name="base"><collision><origin xyz="0 0 0.1" rpy="0 0 1.5707963267948966"/><geometry><box size="0.4 0.2 0.2"/></geometry></collision></link>
<joint name="root_base" type="fixed"><parent link="root"/><child link="base"/><origin xyz="1 0 0" rpy="0 0 1.5707963267948966"/></joint>
<link name="sensor"><collision><geometry><cylinder radius="0.05" length="0.2"/></geometry></collision></link>
<joint name="base_sensor" type="fixed"><parent link="base"/><child link="sensor"/><origin xyz="0.6 0 0.1" rpy="0 1.5707963267948966 0"/></joint>
<link name="leg"><collision><geometry><box size="0.1 0.1 0.4"/></geometry></collision></link>
<joint name="hip" type="revolute"><parent link="base"/><child link="leg"/><origin xyz="0 -0.3 0"/><axis xyz="0 1 0"/></joint>
<link name="foot"><collision><geometry><sphere radius="0.05"/></geometry></collision></link>
<joint name="fixed_foot" type="fixed"><parent link="leg"/><child link="foot"/><origin xyz="0 0 -0.4"/></joint>
</robot>'''


def case(name, model, points, fixed_links, excluded_moving, **dimensions):
    config = geometry.RobotGeometryConfig(model_path=str(model), model_base_link="base", **dimensions)
    result = geometry.resolve_robot_geometry(config, robot_name="generic", base_dir=ROOT)
    return {
        "name": name,
        "runtime_json": result.to_runtime_json(),
        "fixed_links": fixed_links,
        "excluded_moving": excluded_moving,
        "expected_source": "manual" if dimensions else "model",
        "points": [{"label": label, "xyz": xyz, "is_self": is_self} for label, xyz, is_self in points],
    }


def main():
    # 足端取显式URDF数学零位；这些位置不代表机器人的实际站立标定。
    go2 = ROOT / "drivers/go2/robot_geometry.urdf"
    cases = [case("bundled-go2", go2, [
        ("躯干内部", [0.0, 0.0, 0.0], True),
        ("固定头部圆柱内部", [0.285, 0.0, 0.01], True),
        ("固定头部球体内部", [0.293, 0.0, -0.06], True),
        ("躯干与头部间隙", [0.22, 0.0, 0.0], False),
        ("腿间空间", [0.0, 0.0, -0.2], False),
        ("模型左前足碰撞中心", [0.1914, 0.142, -0.426], False),
        ("模型右前足碰撞中心", [0.1914, -0.142, -0.426], False),
        ("模型左后足碰撞中心", [-0.1954, 0.142, -0.426], False),
        ("模型右后足碰撞中心", [-0.1954, -0.142, -0.426], False),
    ], ["base", "Head_upper", "Head_lower"], 24)]
    with tempfile.TemporaryDirectory(prefix="neh-cross-language-") as directory:
        model = Path(directory) / "nonroot.urdf"
        model.write_text(GENERIC, encoding="utf-8")
        cases.append(case("generic-nonroot", model, [
            ("旋转躯干内部", [0.0, 0.0, 0.1], True),
            ("旋转盒长轴内部", [0.0, 0.15, 0.1], True),
            ("旋转盒短轴外部", [0.15, 0.0, 0.1], False),
            ("固定祖先球体内部", [0.0, 1.0, 0.0], True),
            ("旋转圆柱轴向内部", [0.65, 0.0, 0.1], True),
            ("旋转圆柱径向外部", [0.6, 0.0, 0.18], False),
            ("躯干与传感器间隙", [0.3, 0.0, 0.1], False),
            ("活动子树足端中心", [0.0, -0.3, -0.4], False),
        ], ["root", "base", "sensor"], 2))
    cases.append(case("manual-overrides-model", go2, [], [], 0,
                      length_m=1.0, width_m=1.0, height_m=1.0))
    json.dump(cases, sys.stdout, ensure_ascii=False, allow_nan=False, sort_keys=True, indent=2)
    sys.stdout.write("\n")


if __name__ == "__main__":
    main()
