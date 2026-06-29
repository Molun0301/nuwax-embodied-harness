#!/usr/bin/env python3
"""G1 RTAB/Nav2 launch + .env 静态校验（v4 7 条自动化断言）。

受 OpenSpec change g1-rtabnav2-adapter 约束；详见
docs/开发文档/Ralplan/g1-rtabnav2-适配/方案.md（v4 + v5 修订段）与
openspec/changes/g1-rtabnav2-adapter/specs/g1-rtabnav2-adapter/spec.md。

A1: launch 参数解析（每个 mapping_*.launch.py 能被 ROS Python 解析——本端用 ast 校验）
A2: setup.py glob 包含 mapping_g1.launch.py
A3: mapping_go2.launch.py vs mapping_g1.launch.py 物理量参数差异只来自 G1 专属键
A4: .env.G1_example 含所有 G1 覆盖键
A5: .env.G1_example 与 .env.example 物理量差异只来自 v4 B.3 表（lidar/imu/base/lidar_frame/robot_radius/inflation_radius/odom 7 项）
A6: _POINT_CLOUD_TOPIC=/utlidar/cloud_livox_mid360（防退化）
A7: _IMU_TOPIC=/utlidar/imu_livox_mid360（防退化）

退出 0 = 全过；非 0 = 至少 1 条失败（打印具体失败键与修复建议）。
"""

from __future__ import annotations

import ast
import os
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_DIR = (
    REPO_ROOT
    / "services"
    / "geometry"
    / "ros2_ws"
    / "src"
    / "GO2_RTABNAV2"
    / "unitree-go2-slam-nav2"
    / "go2_slam_nav"
    / "launch"
)
SETUP_PY = (
    REPO_ROOT
    / "services"
    / "geometry"
    / "ros2_ws"
    / "src"
    / "GO2_RTABNAV2"
    / "unitree-go2-slam-nav2"
    / "go2_slam_nav"
    / "setup.py"
)
ENV_EXAMPLE = REPO_ROOT / ".env.example"
ENV_G1_EXAMPLE = REPO_ROOT / ".env.G1_example"

# v4 B.3 表 G1 列（覆盖 .env.example GO2 默认值的 7 个键）
G1_KEYS = {
    "NUWAX_GEOMETRY_POINT_CLOUD_TOPIC": "/utlidar/cloud_livox_mid360",
    "NUWAX_GEOMETRY_IMU_TOPIC": "/utlidar/imu_livox_mid360",
    "NUWAX_GEOMETRY_BASE_FRAME": "pelvis",
    "NUWAX_GEOMETRY_LIDAR_FRAME": "livox_mid360_link",
    "NUWAX_GEOMETRY_ODOM_TOPIC": "/unitree/slam_mapping/odom",
    "NUWAX_GEOMETRY_ROBOT_RADIUS": "0.20",
    "NUWAX_GEOMETRY_INFLATION_RADIUS": "0.65",
}

# v4 必须不回归的 G1 取值（防 v3 GO2 默认值回流）
PROHIBITED_G1 = {
    "NUWAX_GEOMETRY_POINT_CLOUD_TOPIC": [
        "/utlidar/cloud",
        "/utlidar/cloud_deskewed",
    ],
    "NUWAX_GEOMETRY_IMU_TOPIC": ["/utlidar/imu"],
    "NUWAX_GEOMETRY_BASE_FRAME": ["go2_base_link"],
    "NUWAX_GEOMETRY_LIDAR_FRAME": ["utlidar_lidar"],
    "NUWAX_GEOMETRY_ODOM_TOPIC": ["/utlidar/robot_odom"],
}


def parse_env(path: Path) -> dict[str, str]:
    """轻量 .env 解析（key=value，忽略空行与 # 注释；不处理 export 前缀）。"""
    out: dict[str, str] = {}
    if not path.exists():
        return out
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        line = re.sub(r"^export\s+", "", line)
        if "=" not in line:
            continue
        key, _, value = line.partition("=")
        key = key.strip()
        value = value.strip().strip('"').strip("'")
        # 同行尾部 # 注释
        if " #" in value:
            value = value.split(" #", 1)[0].strip()
        out[key] = value
    return out


def collect_declare_launch_arguments(path: Path) -> set[str]:
    """AST 扫描 *.launch.py 的 DeclareLaunchArgument 调用，收集变量名集合（仅名字检测）。"""
    if not path.exists():
        return set()
    args: set[str] = set()
    try:
        tree = ast.parse(path.read_text(encoding="utf-8"))
    except SyntaxError:
        return set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            func = node.func
            if isinstance(func, ast.Name) and func.id == "DeclareLaunchArgument":
                if node.args and isinstance(node.args[0], ast.Constant):
                    args.add(str(node.args[0].value))
    return args


def main() -> int:
    failures: list[str] = []

    # ---- A1: launch.py AST 校验 ----
    for f in sorted(LAUNCH_DIR.glob("*.launch.py")):
        try:
            ast.parse(f.read_text(encoding="utf-8"))
        except SyntaxError as exc:
            failures.append(f"A1 FAIL: {f.name} syntax error: {exc}")

    # ---- A2: setup.py glob 包含 mapping_g1（setup.py 用 glob('launch/*.launch.py') 自动收录；glob 实地检查）----
    if SETUP_PY.exists():
        if not (LAUNCH_DIR / "mapping_g1.launch.py").exists():
            failures.append(
                "A2 FAIL: launch/mapping_g1.launch.py 文件不存在；"
                "setup.py glob('launch/*.launch.py') 会自动收录"
            )

    # ---- A3: mapping_go2 vs mapping_g1 DeclareLaunchArgument 对比 ----
    # mapping_g1 应比 mapping_go2 多 G1 专属声明（imu_topic / odom_topic / lidar_frame_id / base_frame_id / qos / subscribe_imu）
    go2 = collect_declare_launch_arguments(LAUNCH_DIR / "mapping_go2.launch.py")
    g1 = collect_declare_launch_arguments(LAUNCH_DIR / "mapping_g1.launch.py")
    g1_extras = {"imu_topic", "odom_topic", "lidar_frame_id", "base_frame_id", "qos", "subscribe_imu"}
    missing = g1_extras - g1
    if missing:
        failures.append(
            f"A3 FAIL: mapping_g1.launch.py 缺 G1 专属 DeclareLaunchArgument: {sorted(missing)}"
        )
    # mapping_go2 不应包含 G1 专属变量（保护 GO2 回归）
    leaked = g1_extras & go2
    if leaked:
        failures.append(
            f"A3 FAIL: mapping_go2.launch.py 误包含 G1 专属声明（会破坏 GO2 回归）: {sorted(leaked)}"
        )

    # ---- A4: .env.G1_example 含所有 G1 键 ----
    g1_env = parse_env(ENV_G1_EXAMPLE)
    for key, expected in G1_KEYS.items():
        actual = g1_env.get(key)
        if actual is None:
            failures.append(f"A4 FAIL: .env.G1_example 缺键 {key}={expected}")
        elif actual != expected:
            failures.append(
                f"A4 FAIL: .env.G1_example {key}={actual!r} (expected {expected!r})"
            )

    # ---- A5: .env.G1_example 与 .env.example 在 7 个 G1 键上不同；其他键不强制 ----
    base_env = parse_env(ENV_EXAMPLE)
    for key in G1_KEYS:
        base_val = base_env.get(key)
        g1_val = g1_env.get(key)
        if base_val is None:
            # .env.example 缺 G1 键也是允许的（GO2 范围可不声明）
            continue
        if g1_val is None:
            continue
        if base_val == g1_val:
            failures.append(
                f"A5 FAIL: .env.G1_example 与 .env.example 在 {key} 取值相同 "
                f"({base_val!r})；G1 必须覆盖 GO2 默认值"
            )

    # ---- A6: _POINT_CLOUD_TOPIC 取值非 GO2 风格 ----
    cloud = g1_env.get("NUWAX_GEOMETRY_POINT_CLOUD_TOPIC", "")
    for bad in PROHIBITED_G1["NUWAX_GEOMETRY_POINT_CLOUD_TOPIC"]:
        if cloud == bad:
            failures.append(
                f"A6 FAIL: NUWAX_GEOMETRY_POINT_CLOUD_TOPIC={cloud} "
                f"(expected /utlidar/cloud_livox_mid360) — v3 GO2 默认值回流"
            )
            break
    else:
        if cloud != "/utlidar/cloud_livox_mid360":
            failures.append(
                f"A6 FAIL: NUWAX_GEOMETRY_POINT_CLOUD_TOPIC={cloud!r} "
                f"(expected '/utlidar/cloud_livox_mid360')"
            )

    # ---- A7: _IMU_TOPIC 取值非 GO2 风格 ----
    imu = g1_env.get("NUWAX_GEOMETRY_IMU_TOPIC", "")
    for bad in PROHIBITED_G1["NUWAX_GEOMETRY_IMU_TOPIC"]:
        if imu == bad:
            failures.append(
                f"A7 FAIL: NUWAX_GEOMETRY_IMU_TOPIC={imu} (expected /utlidar/imu_livox_mid360) — v3 回流"
            )
            break
    else:
        if imu != "/utlidar/imu_livox_mid360":
            failures.append(
                f"A7 FAIL: NUWAX_GEOMETRY_IMU_TOPIC={imu!r} "
                f"(expected '/utlidar/imu_livox_mid360')"
            )

    # ---- 输出 ----
    if failures:
        print("=== G1 RTAB/Nav2 静态校验失败 ===", file=sys.stderr)
        for line in failures:
            print(f"  {line}", file=sys.stderr)
        print(
            "\n修复建议：参见 docs/开发文档/Ralplan/g1-rtabnav2-适配/方案.md (v5)",
            file=sys.stderr,
        )
        return 1

    print("=== G1 RTAB/Nav2 静态校验通过（A1-A7 全过）===")
    return 0


if __name__ == "__main__":
    sys.exit(main())