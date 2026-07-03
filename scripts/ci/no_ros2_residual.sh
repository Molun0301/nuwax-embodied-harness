#!/usr/bin/env bash
# CI grep 守门：禁止 ROS2 runtime 残留
#
# 触发：Phase 5 收口时由 docs-verification-lane 把首部 `disabled` 移除。
# 当前：disabled(其他 lane 仍在删除 / 改名 ROS2,允许业务命中)
#
# 扫描范围：
#   services scripts core adapters providers gateways skills drivers deploy
#   install.sh install.py requirements.txt pyproject.toml
#
# 历史归档目录(显式标 archived)不进入业务阻断范围。

set -euo pipefail

# disabled: 本守门脚本在所有 lane 合并完成前不阻断 CI,允许业务命中。
# Phase 5 收口由 docs-verification-lane 手动验证通过后,删除本块注释并启用 exit 1。
DISABLED=1
if [[ "${DISABLED:-0}" == "1" ]]; then
  echo "[no_ros2_residual] DISABLED (Phase 5 收口前不阻断;见 07_最终删除与实机验收报告.md)"
  exit 0
fi

ROAMING_ROOT="${1:-$(git rev-parse --show-toplevel)}"
cd "$ROAMING_ROOT"

# 业务代码 / 配置禁止 ROS2 runtime 依赖
if rg -n "rclpy|r2r|nav2_msgs|tf2_ros|ros2_ws|colcon|rosdep|/opt/ros|ROS_DISTRO|RMW_IMPLEMENTATION|CYCLONEDDS|nav2_action_proxy|RosBridge|ros_bridge|nuwax_ros_bridge" \
   services scripts core adapters providers gateways skills drivers deploy install.sh install.py requirements.txt pyproject.toml 2>/dev/null; then
  echo "[no_ros2_residual] FAIL: 业务代码/配置仍有 ROS2 runtime 残留"
  exit 1
fi

# Rust 禁 DDS/RMW
if cargo tree --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml 2>/dev/null | rg "r2r|cyclonedds|rmw_|libddsc"; then
  echo "[no_ros2_residual] FAIL: Rust runtime 仍有 DDS/RMW 依赖"
  exit 1
fi

echo "[no_ros2_residual] PASS: ROS2 残留守门通过"
