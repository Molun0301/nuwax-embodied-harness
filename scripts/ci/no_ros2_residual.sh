#!/usr/bin/env bash
# CI grep 守门：禁止 ROS2 runtime 残留
#
# 默认启用。任意 lane 引入新的 ROS2 引用都必须先在 docs/开发文档 留下根因备忘。
#
# 扫描范围（git grep 仅扫已跟踪文件，本守门假设代码与配置都已入库）：
#   services scripts core adapters providers gateways skills drivers deploy
#   install.sh install.py requirements.txt
#   contracts/ .env.example .github/workflows/
#
# 排除项：
#   - 守门脚本自身（scripts/ci/no_ros2_residual.sh）
#   - 历史归档目录（*/archived/*、*/ros2_ws/* 由其它 lane 自行处理）
#   - 测试与本地缓存（tests/ 已 git ignored；__pycache__/ 同）
#
# 选择 git grep 的原因：
#   1) 系统并不总保证 rg 二进制可用；本仓库曾因 shell 函数劫持 rg 导致命中漏判。
#   2) git grep 自动按 .gitignore 过滤、对缺失路径不会返回非零，与 rg 行为差异更可控。
#   3) 已跟踪文件才是 CI 关注对象；Cargo.lock 等构建产物由 cargo tree 单独守门。

set -uo pipefail

ROAMING_ROOT="${1:-$(git rev-parse --show-toplevel)}"
cd "$ROAMING_ROOT"

# 禁用词表：所有关键字集中此处审阅，修改需要同步 docs-verification-lane。
PATTERNS='rclpy|r2r|nav2_msgs|tf2_ros|ros2_ws|colcon|rosdep|/opt/ros|ROS_DISTRO|RMW_IMPLEMENTATION|CYCLONEDDS|nav2_action_proxy|RosBridge|ros_bridge|nuwax_ros_bridge'

# 业务代码 / 配置禁止 ROS2 runtime 依赖。
# 路径用 git grep 的 pathspec 形式精确指定；缺哪个就自动跳过，不会报路径错误。
#
# 注：因 git grep 未命中时 exit=1，这里必须配合 || true，并把判定交给输出非空检查。
HITS_BUSINESS=$(git grep -nE -e "$PATTERNS" \
  -- ':!scripts/ci/no_ros2_residual.sh' \
  services scripts core adapters providers gateways skills drivers deploy \
  install.sh install.py requirements.txt \
  contracts \
  .env.example .github/workflows 2>/dev/null || true)

if [ -n "$HITS_BUSINESS" ]; then
  echo "[no_ros2_residual] FAIL: 业务代码/配置仍有 ROS2 runtime 残留" >&2
  echo "$HITS_BUSINESS" >&2
  exit 1
fi

# pyproject.toml 顶层检查（可能不存在；存在才纳入扫描）。
if [[ -e pyproject.toml ]]; then
  HITS_PYPROJ=$(git grep -nE -e "$PATTERNS" -- pyproject.toml 2>/dev/null || true)
  if [ -n "$HITS_PYPROJ" ]; then
    echo "[no_ros2_residual] FAIL: pyproject.toml 仍有 ROS2 runtime 残留" >&2
    echo "$HITS_PYPROJ" >&2
    exit 1
  fi
fi

# Rust 禁 DDS/RMW。仅当 Cargo.toml 存在时才执行 cargo tree。
CARGO_TOML=services/geometry/nuwax_geometry_runtime/Cargo.toml
if [[ -e "$CARGO_TOML" ]]; then
  HITS_CARGO=$(cargo tree --manifest-path "$CARGO_TOML" 2>/dev/null \
    | grep -E "r2r|cyclonedds|rmw_|libddsc" || true)
  if [ -n "$HITS_CARGO" ]; then
    echo "[no_ros2_residual] FAIL: Rust runtime 仍有 DDS/RMW 依赖" >&2
    echo "$HITS_CARGO" >&2
    exit 1
  fi
fi

echo "[no_ros2_residual] PASS: ROS2 残留守门通过"
exit 0
