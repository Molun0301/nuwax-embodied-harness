#!/usr/bin/env bash
# 通用机器人 RTAB-Map+Nav2 启动器（替代 go2.sh）。
#
# 范围（v4 收敛 + OpenSpec g1-rtabnav2-adapter）：
#   - mapping (SLAM + 实时 2D 障碍)
#   - localization (重定位 + Nav2)
#   - explore (GO2 自主探索；G1 暂未开放)
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
#   ./robot_slam.sh --explore [map.db]
#   ./robot_slam.sh --fresh --mapping   # 强制 restart_map=true
#   ROBOT=g1 ./robot_slam.sh --mapping  # G1 端（需 NUWAX_ACTIVE_ROBOT=g1 + .env.G1_example）
#
# 本期不在范围：
#   - --mapping-only / --boundary（v4 范围外；未来 follow-up 加）
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

source_without_nounset() {
  local target="$1"
  local had_nounset=0
  if [[ $- == *u* ]]; then
    had_nounset=1
    set +u
  fi
  # shellcheck disable=SC1090
  source "${target}"
  if [[ "${had_nounset}" -eq 1 ]]; then
    set -u
  fi
}

prepend_path_once() {
  local target="$1"
  [[ -n "${target}" && -d "${target}" ]] || return 0
  case ":${PATH:-}:" in
    *":${target}:"*) return 0 ;;
  esac
  export PATH="${target}:${PATH:-}"
}

source_ros_runtime_env() {
  local shared_runtime_env="${SCRIPT_DIR}/../../../source_runtime_env.sh"
  if [[ -f "${shared_runtime_env}" ]]; then
    export NUWAX_GEOMETRY_RUNTIME_SOURCE_ROS_ENV=true
    export NUWAX_GEOMETRY_RUNTIME_FORCE_CYCLONEDDS="${NUWAX_GEOMETRY_RUNTIME_FORCE_CYCLONEDDS:-true}"
    source_without_nounset "${shared_runtime_env}"
    echo "==> ROS env: sourced ${shared_runtime_env}"
  else
    local ros_setup=""
    if [[ -n "${NUWAX_ROS_SETUP_SCRIPT:-}" && -f "${NUWAX_ROS_SETUP_SCRIPT}" ]]; then
      ros_setup="${NUWAX_ROS_SETUP_SCRIPT}"
    elif [[ -f "/opt/ros/${GO2_ROS_DISTRO}/setup.bash" ]]; then
      ros_setup="/opt/ros/${GO2_ROS_DISTRO}/setup.bash"
    fi
    if [[ -n "${ros_setup}" ]]; then
      source_without_nounset "${ros_setup}"
      echo "==> ROS env: sourced ${ros_setup}"
    fi
    if [[ -n "${NUWAX_GEOMETRY_SETUP_SCRIPT:-}" && -f "${NUWAX_GEOMETRY_SETUP_SCRIPT}" ]]; then
      source_without_nounset "${NUWAX_GEOMETRY_SETUP_SCRIPT}"
      echo "==> ROS env: sourced ${NUWAX_GEOMETRY_SETUP_SCRIPT}"
    fi
    if [[ -n "${NUWAX_GEOMETRY_GO2SLAM_SETUP_SCRIPT:-}" && -f "${NUWAX_GEOMETRY_GO2SLAM_SETUP_SCRIPT}" ]]; then
      source_without_nounset "${NUWAX_GEOMETRY_GO2SLAM_SETUP_SCRIPT}"
      echo "==> ROS env: sourced ${NUWAX_GEOMETRY_GO2SLAM_SETUP_SCRIPT}"
    elif [[ -f "${SCRIPT_DIR}/ws/install/setup.bash" ]]; then
      source_without_nounset "${SCRIPT_DIR}/ws/install/setup.bash"
      echo "==> ROS env: sourced ${SCRIPT_DIR}/ws/install/setup.bash"
    fi
  fi
  prepend_path_once "/opt/ros/${GO2_ROS_DISTRO}/bin"
  if ! command -v ros2 >/dev/null 2>&1; then
    die "ros2 not found after sourcing ROS env. Set NUWAX_ROS_SETUP_SCRIPT=/opt/ros/${GO2_ROS_DISTRO}/setup.bash or install ROS2 ${GO2_ROS_DISTRO}."
  fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --fresh)
      FRESH_MAP="true"
      shift
      ;;
    -m|--new|--mapping|-l|--loc|--localization|-e|--explore|--exploration)
      [[ -z "$MODE" ]] || die "only one mode flag allowed"
      case "$1" in
        -m|--new|--mapping) MODE="mapping" ;;
        -l|--loc|--localization) MODE="localization" ;;
        -e|--explore|--exploration) MODE="explore" ;;
      esac
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

[[ -n "$MODE" ]] || die "missing mode: --mapping | --localization | --explore"

if [[ "$MODE" == "localization" ]]; then
  [[ -n "$DB_ARG" ]] || die "--localization requires a database path"
fi

# 物理量注入：GO2 端走既有 scripts/load_platform_geometry_env.sh 注入 16 个 NUWAX_GEOMETRY_* 键；
# G1 端走 start_gateway.sh 的 .env.G1_example 覆盖路径（不调用本脚本）。
if [[ -f "${SCRIPT_DIR}/scripts/load_platform_geometry_env.sh" ]]; then
  source_without_nounset "${SCRIPT_DIR}/scripts/load_platform_geometry_env.sh"
fi

source_ros_runtime_env

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
  explore)
    launch_file="explore_${ROBOT}.launch.py"
    if [[ "${ROBOT}" == "g1" ]]; then
      die "G1 explore 模式暂未开放（follow-up F2）；请用 mapping 模式或切换到 GO2"
    fi
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
  # explore_go2.launch.py 内部已向 nav_go2.launch.py 透传 slam_mode:=explore；
  # 顶层未声明 slam_mode，避免额外传参触发 ROS2 launch unknown argument。
  explore) ;;
  *) args+=("slam_mode:=mapping") ;;
esac

echo "==> Robot: ${ROBOT}"
echo "==> Mode: ${MODE}"
echo "==> Launch: go2_slam_nav/${launch_file}"
echo "    localize_only=${args[0]#localize_only:=}  restart_map=${args[1]#restart_map:=}"
echo "    active db: ${TARGET_DB}"

exec ros2 launch go2_slam_nav "${launch_file}" "${args[@]}"
