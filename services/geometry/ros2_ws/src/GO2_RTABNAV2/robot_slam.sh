#!/usr/bin/env bash
# 通用机器人 RTAB-Map+Nav2 启动器（替代 go2.sh）。
#
# 范围（v4 收敛 + OpenSpec g1-rtabnav2-adapter）：
#   - mapping (SLAM + 实时 2D 障碍)
#   - localization (重定位 + Nav2)
#   - 未来可加 explore / camera_fusion（follow-up）
#
# 替代 go2.sh 的逻辑（保留 GO2 历史行为），但去 GO2 专属命名：
#   - 模式分发：--mapping / --localization（v4 范围）
#   - 数据库路径：NUWAX_RTABMAP_DATABASE_ROOT/live_rtabmap.db（沿用 v3 范式）
#   - 物理量从 ROS env 注入（脚本不写死任何机器人物理量）
#   - 选 mapping_go2 / mapping_g1 launch 按 ROBOT 环境变量（默认 go2）
#
# 用法：
#   ./robot_slam.sh --mapping [map.db]
#   ./robot_slam.sh --localization <map.db>
#   ./robot_slam.sh --fresh --mapping   # 强制 restart_map=true
#   ROBOT=g1 ./robot_slam.sh --mapping  # G1 端（需 NUWAX_ACTIVE_ROBOT=g1 + .env.G1_example）
#
# 本期不在范围：
#   - --mapping-only / --explore / --boundary（v4 范围外；未来 follow-up 加）
#   - go2.sh → 已删除（本脚本唯一入口）

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT="${ROBOT:-${NUWAX_ACTIVE_ROBOT:-go2}}"
ROBOT="$(printf '%s' "${ROBOT}" | tr '[:upper:]' '[:lower:]')"
MODE=""
DB_ARG=""
FRESH_MAP="false"

if [[ -z "${GO2_ROS_DISTRO:-}" ]]; then
  if [[ -f /opt/ros/foxy/setup.bash ]]; then
    GO2_ROS_DISTRO=foxy
  elif [[ -f /opt/ros/jazzy/setup.bash ]]; then
    GO2_ROS_DISTRO=jazzy
  else
    GO2_ROS_DISTRO="${ROS_DISTRO:-foxy}"
  fi
fi
export GO2_ROS_DISTRO

usage() {
  sed -n '2,18p' "$0" | sed 's/^# \{0,1\}//'
}

die() {
  echo "robot_slam.sh: $*" >&2
  exit 1
}

warn() {
  echo "robot_slam.sh: warning: $*" >&2
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --fresh)
      FRESH_MAP="true"
      shift
      ;;
    --mapping|--localization)
      [[ -z "$MODE" ]] || die "only one mode flag allowed"
      MODE="${1#--}"
      shift
      if [[ $# -gt 0 && "$1" != --* ]]; then
        DB_ARG="$1"
        shift
      fi
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "unknown argument: $1 (try --help)"
      ;;
  esac
done

[[ -n "$MODE" ]] || die "missing mode: --mapping | --localization"

if [[ "$MODE" == "localization" ]]; then
  [[ -n "$DB_ARG" ]] || die "--localization requires a database path"
fi

# 物理量注入：GO2 端走既有 scripts/load_platform_geometry_env.sh 注入 16 个 NUWAX_GEOMETRY_* 键；
# G1 端走 start_gateway.sh 的 .env.G1_example 覆盖路径（不调用本脚本）。
if [[ -f "${SCRIPT_DIR}/scripts/load_platform_geometry_env.sh" ]]; then
  set +u
  # shellcheck disable=SC1091
  source "${SCRIPT_DIR}/scripts/load_platform_geometry_env.sh"
  set -u
fi

# 数据库路径解析（沿用 v3 范式）
resolve_default_db() {
  local root="${NUWAX_RTABMAP_DATABASE_ROOT:-${NUWAX_BACKEND_DATABASE_ROOT:-}}"
  if [[ -n "$root" ]]; then
    mkdir -p "$root"
    echo "${root%/}/live_rtabmap.db"
  else
    echo "${HOME}/.ros/rtabmap.db"
  fi
}

DEFAULT_DB="$(resolve_default_db)"
TARGET_DB="${DB_ARG:-$DEFAULT_DB}"

if [[ "$TARGET_DB" != /* ]]; then
  TARGET_DB="$(pwd)/${TARGET_DB}"
fi
mkdir -p "$(dirname "$TARGET_DB")"

# RTAB-Map 数据库符号链接（沿用 v3 范式）
mkdir -p "${HOME}/.ros"
local_default_db="${HOME}/.ros/rtabmap.db"
active_real="$(realpath -m "$TARGET_DB")"
default_real="$(realpath -m "$local_default_db")"
if [[ "$active_real" != "$default_real" ]]; then
  ln -sfn "$active_real" "$local_default_db"
  echo "==> RTAB-Map DB link: ${local_default_db} -> ${active_real}"
else
  echo "==> RTAB-Map DB: ${default_real}"
fi

# 模式分发：mapping / localization
launch_file=""
case "$MODE" in
  mapping)
    launch_file="mapping_${ROBOT}.launch.py"
    ;;
  localization)
    launch_file="localization_${ROBOT}.launch.py"
    # v4 范围：本期内 localization 模式复用 navigation_go2.launch.py（GO2 端）
    # G1 端 localization 模式 F1 跟入（nav2 闭环）；本期不实现
    if [[ "${ROBOT}" == "g1" ]]; then
      die "G1 localization 模式 nav2 闭环不在本期范围（follow-up F1）；请用 mapping 模式或切换到 GO2"
    fi
    ;;
esac

[[ -n "${launch_file}" ]] || die "ROBOT=${ROBOT} 不支持 MODE=${MODE}"

# 启动参数
args=(
  "localize_only:=$([[ "$MODE" == "localization" ]] && echo true || echo false)"
  "restart_map:=${FRESH_MAP}"
)
case "$MODE" in
  localization) args+=("slam_mode:=localization") ;;
  *) args+=("slam_mode:=mapping") ;;
esac

echo "==> Robot: ${ROBOT}"
echo "==> Mode: ${MODE}"
echo "==> Launch: go2_slam_nav/${launch_file}"
echo "    localize_only=${args[0]#localize_only:=}  restart_map=${args[1]#restart_map:=}"
echo "    active db: ${TARGET_DB}"

exec ros2 launch go2_slam_nav "${launch_file}" "${args[@]}"
