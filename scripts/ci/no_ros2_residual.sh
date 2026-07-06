#!/usr/bin/env bash
# CI grep 守门：禁止 ROS2 runtime 残留
#
# 默认启用。任意 lane 引入新的 ROS2 引用都必须先在 docs/开发文档 留下根因备忘。
#
# 双层结构（L1 硬阻断 / L2 软告警）：
#   L1 = 真业务 import 层。扫描"明确的 Python / bash 业务 import 模式"，命中即阻断（exit 1）。
#        模式见下方 L1_PATTERNS。
#   L2 = 全部命中层（含注释 / 字符串 / 迁移说明），allowlist 命中跳过；未命中阻断。
#
# allowlist 机制：
#   - 文件：scripts/ci/no_ros2_residual.allowlist
#   - 格式：每条 4 字段 (file, line, category, reason)
#   - 用途：L2 阶段精确放行迁移说明 / 历史注释 / legacy 隔离代码 / 强约束反向锚点
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
#
# 用法：
#   bash scripts/ci/no_ros2_residual.sh            # 默认模式（L1 阻断 + L2 allowlist）
#   bash scripts/ci/no_ros2_residual.sh --dry-run  # 打印全部命中（含 allowlist）但不 exit 1

set -uo pipefail

DRY_RUN=0
for arg in "$@"; do
  if [[ "$arg" == "--dry-run" ]]; then
    DRY_RUN=1
  fi
done

REPO_ROOT="$(git rev-parse --show-toplevel)"
cd "$REPO_ROOT"

# L1 真业务 import 模式：仅"明确的 ROS2 库 import"算硬残留。
# 字符级精确匹配，避免误伤路径、字符串、注释、文档。
#  - 'import rclpy' / 'from rclpy'
#  - 'import r2r' / 'from r2r'
#  - 'import nav2_msgs' / 'from nav2_msgs' / 'import tf2_ros' / 'from tf2_ros'
#  - 'import sensor_msgs' / 'from sensor_msgs' / 'import geometry_msgs' / 'from geometry_msgs'
#  - 'import rosidl' / 'from rosidl' / 'import rclcpp' / 'from rclcpp'
# 注意：本模式匹配的是 Python `import` 语句 + 紧跟空格 + 库名；不会误伤"rclpy 字段"、
# "ros2_ws 路径字符串"等迁移注释/字符串字面量。
L1_PATTERNS='(^|[[:space:]])import[[:space:]]+rclpy($|[[:space:]]|\.)|(^|[[:space:]])from[[:space:]]+rclpy($|[[:space:]]|\.)|(^|[[:space:]])import[[:space:]]+r2r($|[[:space:]]|\.)|(^|[[:space:]])from[[:space:]]+r2r($|[[:space:]]|\.)|(^|[[:space:]])import[[:space:]]+nav2_msgs($|[[:space:]]|\.)|(^|[[:space:]])from[[:space:]]+nav2_msgs($|[[:space:]]|\.)|(^|[[:space:]])import[[:space:]]+tf2_ros($|[[:space:]]|\.)|(^|[[:space:]])from[[:space:]]+tf2_ros($|[[:space:]]|\.)|(^|[[:space:]])import[[:space:]]+sensor_msgs($|[[:space:]]|\.)|(^|[[:space:]])from[[:space:]]+sensor_msgs($|[[:space:]]|\.)|(^|[[:space:]])import[[:space:]]+geometry_msgs($|[[:space:]]|\.)|(^|[[:space:]])from[[:space:]]+geometry_msgs($|[[:space:]]|\.)|(^|[[:space:]])import[[:space:]]+rosidl($|[[:space:]]|\.)|(^|[[:space:]])from[[:space:]]+rosidl($|[[:space:]]|\.)|(^|[[:space:]])import[[:space:]]+rclcpp($|[[:space:]]|\.)|(^|[[:space:]])from[[:space:]]+rclcpp($|[[:space:]]|\.)'

# L2 全部命中模式：放宽到所有 ROS2 关键字
PATTERNS='rclpy|r2r|nav2_msgs|tf2_ros|ros2_ws|colcon|rosdep|/opt/ros|ROS_DISTRO|RMW_IMPLEMENTATION|CYCLONEDDS|nav2_action_proxy|RosBridge|ros_bridge|nuwax_ros_bridge'

# allowlist 文件路径
ALLOWLIST_FILE="${REPO_ROOT}/scripts/ci/no_ros2_residual.allowlist"

# 加载 allowlist 到关联数组 key="file:line"
declare -A ALLOWLISTED=()
if [[ -f "$ALLOWLIST_FILE" ]]; then
  while IFS= read -r line; do
    # 跳过空行与注释
    [[ -z "$line" || "$line" =~ ^[[:space:]]*# ]] && continue
    # 解析 - file: ... 行
    if [[ "$line" =~ ^[[:space:]]*-[[:space:]]*file:[[:space:]]*([^[:space:]]+) ]]; then
      cur_file="${BASH_REMATCH[1]}"
      continue
    fi
    # 解析 line: ... 行
    if [[ "$line" =~ ^[[:space:]]*line:[[:space:]]*([0-9]+) ]]; then
      cur_line="${BASH_REMATCH[1]}"
      key="${cur_file}:${cur_line}"
      ALLOWLISTED["$key"]=1
    fi
  done < "$ALLOWLIST_FILE"
fi

# 把 git grep 输出按 file:line 过滤 allowlist。
# stdin: git grep 输出（file:line:content）
# stdout: 过滤后剩余行（未命中 allowlist）
filter_allowlist() {
  local input="$1"
  while IFS= read -r hit; do
    [[ -z "$hit" ]] && continue
    local file line
    file=$(echo "$hit" | cut -d: -f1)
    line=$(echo "$hit" | cut -d: -f2)
    key="${file}:${line}"
    if [[ -n "${ALLOWLISTED[$key]:-}" ]]; then
      continue
    fi
    echo "$hit"
  done <<< "$input"
}

# filter_allowlist 的互补函数：返回**被** allowlist 命中的行（dry-run 信息输出用）
filter_allowlist_skip() {
  local input="$1"
  while IFS= read -r hit; do
    [[ -z "$hit" ]] && continue
    local file line
    file=$(echo "$hit" | cut -d: -f1)
    line=$(echo "$hit" | cut -d: -f2)
    key="${file}:${line}"
    if [[ -n "${ALLOWLISTED[$key]:-}" ]]; then
      echo "$hit"
    fi
  done <<< "$input"
}

# === L1 扫描：真业务 import ===
# 路径用 git grep 的 pathspec 形式精确指定；缺哪个就自动跳过，不会报路径错误。
# 注：因 git grep 未命中时 exit=1，这里必须配合 || true，并把判定交给输出非空检查。
HITS_L1=$(git grep -nE -e "$L1_PATTERNS" \
  -- ':!scripts/ci/no_ros2_residual.sh' \
  -- ':!scripts/ci/no_ros2_residual.allowlist' \
  services scripts core adapters providers gateways skills drivers deploy \
  install.sh install.py requirements.txt \
  contracts \
  .env.example .github/workflows 2>/dev/null || true)

# pyproject.toml 顶层检查（可能不存在）
if [[ -e pyproject.toml ]]; then
  HITS_L1_PYPROJ=$(git grep -nE -e "$L1_PATTERNS" -- pyproject.toml 2>/dev/null || true)
  if [ -n "$HITS_L1_PYPROJ" ]; then
    HITS_L1="${HITS_L1}${HITS_L1:+$'\n'}${HITS_L1_PYPROJ}"
  fi
fi

# === L2 扫描：全部 ROS2 关键字 ===
HITS_L2=$(git grep -nE -e "$PATTERNS" \
  -- ':!scripts/ci/no_ros2_residual.sh' \
  -- ':!scripts/ci/no_ros2_residual.allowlist' \
  services scripts core adapters providers gateways skills drivers deploy \
  install.sh install.py requirements.txt \
  contracts \
  .env.example .github/workflows 2>/dev/null || true)

if [[ -e pyproject.toml ]]; then
  HITS_L2_PYPROJ=$(git grep -nE -e "$PATTERNS" -- pyproject.toml 2>/dev/null || true)
  if [ -n "$HITS_L2_PYPROJ" ]; then
    HITS_L2="${HITS_L2}${HITS_L2:+$'\n'}${HITS_L2_PYPROJ}"
  fi
fi

# --dry-run 模式：打印所有命中但不 exit 1
if [[ "$DRY_RUN" == "1" ]]; then
  echo "[no_ros2_residual] DRY-RUN 模式：打印全部命中"
  echo ""
  echo "=== L1 真业务 import（命中即阻断，不受 allowlist 影响） ==="
  if [ -n "$HITS_L1" ]; then
    echo "$HITS_L1"
  else
    echo "(空)"
  fi
  echo ""
  echo "=== L2 全部命中（未命中 allowlist 会阻断） ==="
  if [ -n "$HITS_L2" ]; then
    L2_BLOCKED=$(filter_allowlist "$HITS_L2" | grep -v '^$' || true)
    L2_ALLOWED=$(filter_allowlist_skip "$HITS_L2" | grep -v '^$' || true)
    echo "[L2 阻断（未在 allowlist）]"
    if [ -n "$L2_BLOCKED" ]; then
      echo "$L2_BLOCKED"
    else
      echo "(空)"
    fi
    echo ""
    echo "[L2 已放行（命中 allowlist）]"
    if [ -n "$L2_ALLOWED" ]; then
      echo "$L2_ALLOWED"
    else
      echo "(空)"
    fi
  else
    echo "(空)"
  fi
  echo ""
  echo "[no_ros2_residual] DRY-RUN 完成，不修改 exit code"
  exit 0
fi

# === 默认模式：L1 任意命中 → exit 1；L2 未命中 allowlist → exit 1 ===
if [ -n "$HITS_L1" ]; then
  echo "[no_ros2_residual] FAIL: L1 真业务 import 残留（硬阻断）" >&2
  echo "$HITS_L1" >&2
  exit 1
fi

if [ -n "$HITS_L2" ]; then
  L2_BLOCKED=$(filter_allowlist "$HITS_L2" | grep -v '^$' || true)
  if [ -n "$L2_BLOCKED" ]; then
    echo "[no_ros2_residual] FAIL: L2 命中未在 allowlist 中登记" >&2
    echo "$L2_BLOCKED" >&2
    echo "" >&2
    echo "如需放行，请追加 file:line 到 ${ALLOWLIST_FILE} 并在 docs/ 留根因备忘。" >&2
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