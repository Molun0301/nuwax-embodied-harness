#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="${NUWAX_REPO_ROOT:-$(cd "${SCRIPT_DIR}/../../../../.." && pwd)}"

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

detect_ros_setup() {
  local candidates=()
  if [[ -n "${NUWAX_ROS_SETUP_SCRIPT:-}" ]]; then
    candidates+=("${NUWAX_ROS_SETUP_SCRIPT}")
  fi
  candidates+=(
    "/opt/ros/humble/setup.bash"
    "/opt/ros/jazzy/setup.bash"
    "/opt/ros/iron/setup.bash"
    "/opt/ros/foxy/setup.bash"
  )

  local candidate
  for candidate in "${candidates[@]}"; do
    if [[ -f "${candidate}" ]]; then
      printf '%s' "${candidate}"
      return 0
    fi
  done
  return 1
}

load_env_subset() {
  local env_file="${REPO_ROOT}/.env"
  [[ -f "${env_file}" ]] || return 0

  local raw_line line key value
  while IFS= read -r raw_line || [[ -n "${raw_line}" ]]; do
    line="${raw_line#"${raw_line%%[![:space:]]*}"}"
    line="${line%"${line##*[![:space:]]}"}"
    if [[ -z "${line}" || "${line}" == \#* || "${line}" != *=* ]]; then
      continue
    fi
    key="${line%%=*}"
    value="${line#*=}"
    key="${key#"${key%%[![:space:]]*}"}"
    key="${key%"${key##*[![:space:]]}"}"
    value="${value#"${value%%[![:space:]]*}"}"
    value="${value%"${value##*[![:space:]]}"}"
    if [[ ${#value} -ge 2 ]]; then
      if [[ "${value:0:1}" == '"' && "${value: -1}" == '"' ]]; then
        value="${value:1:${#value}-2}"
      elif [[ "${value:0:1}" == "'" && "${value: -1}" == "'" ]]; then
        value="${value:1:${#value}-2}"
      fi
    fi
    case "${key}" in
      NUWAX_GEOMETRY_*|NUWAX_RTABMAP_*|NUWAX_NAV2_*|NUWAX_GO2_SLAM_*|NUWAX_ROS_BRIDGE_*|CYCLONEDDS_URI|RMW_IMPLEMENTATION|FASTDDS_BUILTIN_TRANSPORTS)
        export "${key}=${value}"
        ;;
    esac
  done < "${env_file}"
}

is_truthy() {
  local value
  value="$(printf '%s' "${1:-}" | tr '[:upper:]' '[:lower:]')"
  [[ "${value}" == "1" || "${value}" == "true" || "${value}" == "yes" || "${value}" == "on" ]]
}

normalize_initial_pose() {
  local raw="$1"
  [[ -n "${raw}" ]] || return 0

  local parts=()
  # shellcheck disable=SC2206
  parts=(${raw})
  case "${#parts[@]}" in
    3)
      printf '%s %s 0 0 0 %s' "${parts[0]}" "${parts[1]}" "${parts[2]}"
      ;;
    6|7)
      printf '%s' "${raw}"
      ;;
    *)
      echo "initial pose 格式错误: ${raw}" >&2
      echo "可用格式: \"x y yaw\"、\"x y z roll pitch yaw\" 或 \"x y z qx qy qz qw\"。" >&2
      return 1
      ;;
  esac
}

cleanup_stale_slam_processes() {
  local pid args
  if ! is_truthy "${NUWAX_GO2_SLAM_CLEAN_STALE_PROCESSES:-true}"; then
    return 0
  fi

  while read -r pid args; do
    if [[ -z "${pid}" || "${pid}" == "$$" || "${pid}" == "${BASHPID}" ]]; then
      continue
    fi
    case "${args}" in
      *"ros2 launch go2_slam go2_lidar_slam.launch.py"*|*"rtabmap_slam/rtabmap"*|*"go2_slam/launch/odom_to_tf.py"*)
        echo "停止残留 Go2 SLAM 进程 pid=${pid}: ${args}" >&2
        kill -TERM "${pid}" >/dev/null 2>&1 || true
        ;;
    esac
  done < <(ps -eo pid=,args=)

  sleep 1
  while read -r pid args; do
    if [[ -z "${pid}" || "${pid}" == "$$" || "${pid}" == "${BASHPID}" ]]; then
      continue
    fi
    case "${args}" in
      *"ros2 launch go2_slam go2_lidar_slam.launch.py"*|*"rtabmap_slam/rtabmap"*|*"go2_slam/launch/odom_to_tf.py"*)
        kill -KILL "${pid}" >/dev/null 2>&1 || true
        ;;
    esac
  done < <(ps -eo pid=,args=)
}

wait_for_topic_message() {
  local topic="$1"
  local label="$2"
  local timeout_sec="$3"
  local message_type="$4"

  echo "等待 ${label} topic 有消息: ${topic} (timeout=${timeout_sec}s)" >&2
  if timeout "$((timeout_sec + 2))" python3 - "${topic}" "${message_type}" "${timeout_sec}" >/tmp/nuwax_go2_slam_topic_check.out 2>/tmp/nuwax_go2_slam_topic_check.err <<'PY'
import importlib
import sys
import time

import rclpy

topic = sys.argv[1]
message_type = sys.argv[2]
timeout_sec = float(sys.argv[3])

module_name, class_name = message_type.rsplit(".", 1)
message_cls = getattr(importlib.import_module(module_name), class_name)

rclpy.init()
node = rclpy.create_node("nuwax_go2_slam_input_check")
received = False

def callback(_msg):
    global received
    received = True

node.create_subscription(message_cls, topic, callback, 10)
deadline = time.monotonic() + timeout_sec
while rclpy.ok() and not received and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)

node.destroy_node()
rclpy.shutdown()
sys.exit(0 if received else 1)
PY
  then
    echo "${label} topic 已有消息: ${topic}" >&2
    return 0
  fi

  echo "${label} topic 在 ${timeout_sec}s 内没有收到消息: ${topic}" >&2
  echo "最近一次检查输出:" >&2
  sed -n '1,20p' /tmp/nuwax_go2_slam_topic_check.err >&2 || true
  return 1
}

configure_cyclonedds_interface() {
  local iface="${NUWAX_GEOMETRY_DDS_IFACE:-${GO2_DDS_IFACE:-}}"
  if [[ -z "${iface}" ]]; then
    return 0
  fi
  if [[ "${CYCLONEDDS_URI:-}" == *"NetworkInterfaceAddress"* || "${CYCLONEDDS_URI:-}" == *"NetworkInterface "* ]]; then
    return 0
  fi
  if [[ -z "${CYCLONEDDS_URI:-}" ]]; then
    export CYCLONEDDS_URI="<CycloneDDS><Domain><General><NetworkInterfaceAddress>${iface}</NetworkInterfaceAddress></General><Compatibility><StandardsConformance>lax</StandardsConformance></Compatibility></Domain></CycloneDDS>"
    return 0
  fi
  if [[ "${CYCLONEDDS_URI}" == "<CycloneDDS"* && "${CYCLONEDDS_URI}" == *"<Domain>"* ]]; then
    export CYCLONEDDS_URI="${CYCLONEDDS_URI/<Domain>/<Domain><General><NetworkInterfaceAddress>${iface}</NetworkInterfaceAddress></General>}"
  fi
}

load_env_subset

ROS_SETUP="$(detect_ros_setup || true)"
if [[ -z "${ROS_SETUP}" ]]; then
  echo "未找到 ROS2 setup.bash，请设置 NUWAX_ROS_SETUP_SCRIPT 或安装 ROS2。" >&2
  exit 1
fi

source_without_nounset "${ROS_SETUP}"

CYCLONEDDS_WS_SETUP="${NUWAX_GO2_SLAM_CYCLONEDDS_SETUP:-${HOME:-/home/unitree}/cyclonedds_ws/install/setup.bash}"
if [[ -f "${CYCLONEDDS_WS_SETUP}" ]]; then
  source_without_nounset "${CYCLONEDDS_WS_SETUP}"
fi

if [[ ! -f "${SCRIPT_DIR}/install/setup.bash" ]]; then
  echo "未找到 Go2 SLAM workspace install/setup.bash: ${SCRIPT_DIR}/install/setup.bash" >&2
  echo "请先执行: cd ${SCRIPT_DIR} && source ${ROS_SETUP} && colcon build --symlink-install" >&2
  exit 1
fi
source_without_nounset "${SCRIPT_DIR}/install/setup.bash"

SLAM_RMW="${NUWAX_GEOMETRY_EXTERNAL_RMW_IMPLEMENTATION:-${NUWAX_GO2_SLAM_RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}}"
export RMW_IMPLEMENTATION="${SLAM_RMW}"
case "${SLAM_RMW}" in
  rmw_cyclonedds_cpp)
    # 与 ~/.bashrc 中 choose=1 的 Foxy 环境保持一致。这里故意不复用
    # 根 .env 的 CYCLONEDDS_URI，因为它可能是通用 XML 片段，不是 Go2 上
    # 已验证可收到 /utlidar/cloud_base 的 cyclonedds.xml 文件。
    export CYCLONEDDS_URI="${NUWAX_GO2_SLAM_CYCLONEDDS_URI:-${HOME:-/home/unitree}/cyclonedds_ws/cyclonedds.xml}"
    configure_cyclonedds_interface
    export NUWAX_GEOMETRY_RUNTIME_SOURCE_ROS_ENV=false
    export NUWAX_GEOMETRY_RUNTIME_FORCE_CYCLONEDDS=true
    source_without_nounset "${REPO_ROOT}/services/geometry/source_runtime_env.sh"
    ;;
  *)
    # Go2/Jetson 上 FastDDS 共享内存传输偶发会让多个 ROS2 进程同时报
    # std::bad_alloc；默认走 UDPv4 更稳，必要时可在 .env 覆盖。
    export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
    unset CYCLONEDDS_URI
    ;;
esac

DB_ROOT="${NUWAX_RTABMAP_DATABASE_ROOT:-${REPO_ROOT}/runtime_data/geometry/rtabmap}"
mkdir -p "${DB_ROOT}"

DATABASE_PATH="${NUWAX_RTABMAP_DATABASE_PATH:-${DB_ROOT}/go2_lidar_slam.db}"
USE_SIM_TIME="${NUWAX_NAV2_USE_SIM_TIME:-false}"
POINT_CLOUD_TOPIC="${NUWAX_GEOMETRY_POINT_CLOUD_TOPIC:-/utlidar/cloud_base}"
ODOM_TOPIC="${NUWAX_GEOMETRY_ODOM_TOPIC:-/utlidar/robot_odom}"
MAP_FRAME="${NUWAX_GEOMETRY_MAP_FRAME:-map}"
ODOM_FRAME="${NUWAX_GEOMETRY_ODOM_FRAME:-odom}"
BASE_FRAME="${NUWAX_GEOMETRY_BASE_FRAME:-base_link}"
LIDAR_FRAME="${NUWAX_GEOMETRY_LIDAR_FRAME:-utlidar_lidar}"
LOCALIZATION_ODOM_TOPIC="${NUWAX_GO2_SLAM_LOCALIZATION_ODOM_TOPIC:-/rtabmap/localization_odom}"
WAIT_FOR_INPUTS="false"
INPUT_TIMEOUT_SEC="${NUWAX_GO2_SLAM_INPUT_TIMEOUT_SEC:-30}"
SLAM_MODE="${NUWAX_GO2_SLAM_MODE:-auto}"
RTABMAP_ARGS="${NUWAX_GO2_SLAM_RTABMAP_ARGS:-}"
PLANAR_ODOM="${NUWAX_GO2_SLAM_PLANAR_ODOM:-true}"
INITIAL_POSE="${NUWAX_GO2_SLAM_INITIAL_POSE:-}"
AUTO_SWITCH_TO_MAPPING="false"
START_AT_ORIGIN="${NUWAX_GO2_SLAM_START_AT_ORIGIN:-false}"
OPTIMIZE_MAX_ERROR="${NUWAX_GO2_SLAM_OPTIMIZE_MAX_ERROR:-20.0}"
MAX_ODOM_CACHE_SIZE="${NUWAX_GO2_SLAM_MAX_ODOM_CACHE_SIZE:-0}"

LAUNCH_ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-rviz|--no-rviz=true|--no-rviz=false|--rviz|--rviz=true|--rviz=false)
      shift
      ;;
    --database-path|--dataset-path)
      if [[ $# -lt 2 ]]; then
        echo "$1 需要一个路径参数。" >&2
        exit 2
      fi
      DATABASE_PATH="$2"
      shift 2
      ;;
    --use-sim-time)
      USE_SIM_TIME="true"
      shift
      ;;
    --planar-odom)
      PLANAR_ODOM="true"
      shift
      ;;
    --full-6dof-odom)
      PLANAR_ODOM="false"
      shift
      ;;
    --initial-pose)
      if [[ $# -lt 2 ]]; then
        echo "--initial-pose 需要一个位姿字符串，例如 \"1.2 0.5 1.57\"。" >&2
        exit 2
      fi
      INITIAL_POSE="$2"
      shift 2
      ;;
    --start-at-origin)
      START_AT_ORIGIN="true"
      shift
      ;;
    --resume-last-localization)
      START_AT_ORIGIN="false"
      shift
      ;;
    --optimize-max-error)
      if [[ $# -lt 2 ]]; then
        echo "--optimize-max-error 需要一个数值参数。" >&2
        exit 2
      fi
      OPTIMIZE_MAX_ERROR="$2"
      shift 2
      ;;
    --max-odom-cache-size)
      if [[ $# -lt 2 ]]; then
        echo "--max-odom-cache-size 需要一个整数参数。" >&2
        exit 2
      fi
      MAX_ODOM_CACHE_SIZE="$2"
      shift 2
      ;;
    --localization|--localization-mode)
      SLAM_MODE="localization"
      shift
      ;;
    --mapping|--mapping-mode)
      SLAM_MODE="mapping"
      shift
      ;;
    --direct-mapping)
      SLAM_MODE="direct-mapping"
      shift
      ;;
    --new-map)
      SLAM_MODE="new-map"
      shift
      ;;
    --no-wait-inputs)
      WAIT_FOR_INPUTS="false"
      shift
      ;;
    --wait-inputs)
      WAIT_FOR_INPUTS="true"
      shift
      ;;
    --input-timeout)
      if [[ $# -lt 2 ]]; then
        echo "--input-timeout 需要秒数参数。" >&2
        exit 2
      fi
      INPUT_TIMEOUT_SEC="$2"
      shift 2
      ;;
    *)
      LAUNCH_ARGS+=("$1")
      shift
      ;;
  esac
done

cd "${REPO_ROOT}"

mkdir -p "$(dirname "${DATABASE_PATH}")"

INITIAL_POSE="$(normalize_initial_pose "${INITIAL_POSE}")"

DATABASE_EXISTS="false"
if [[ -s "${DATABASE_PATH}" ]]; then
  DATABASE_EXISTS="true"
fi

case "${SLAM_MODE}" in
  auto)
    if is_truthy "${DATABASE_EXISTS}"; then
      SLAM_MODE="localization"
    else
      SLAM_MODE="mapping"
    fi
    ;;
  localization|mapping|direct-mapping|new-map)
    ;;
  *)
    echo "未知 NUWAX_GO2_SLAM_MODE/启动模式: ${SLAM_MODE}，可选 auto/localization/mapping/direct-mapping/new-map。" >&2
    exit 2
    ;;
esac

case "${SLAM_MODE}" in
  localization)
    if ! is_truthy "${DATABASE_EXISTS}"; then
      echo "请求定位复用旧图，但数据库不存在或为空: ${DATABASE_PATH}" >&2
      echo "请先建图，或使用 --mapping / --new-map。" >&2
      exit 2
    fi
    INCREMENTAL_MEMORY="false"
    INIT_WM_WITH_ALL_NODES="true"
    ;;
  mapping)
    if is_truthy "${DATABASE_EXISTS}"; then
      # First relocalize against the existing map, then switch to mapping
      # through /rtabmap/set_mode_mapping. In mapping mode alone, RTAB-Map's
      # global scan map relocalization is not active.
      INCREMENTAL_MEMORY="false"
      INIT_WM_WITH_ALL_NODES="true"
      AUTO_SWITCH_TO_MAPPING="true"
    else
      INCREMENTAL_MEMORY="true"
      INIT_WM_WITH_ALL_NODES="false"
    fi
    ;;
  direct-mapping)
    INCREMENTAL_MEMORY="true"
    if is_truthy "${DATABASE_EXISTS}"; then
      INIT_WM_WITH_ALL_NODES="true"
    else
      INIT_WM_WITH_ALL_NODES="false"
    fi
    ;;
  new-map)
    INCREMENTAL_MEMORY="true"
    INIT_WM_WITH_ALL_NODES="false"
    RTABMAP_ARGS="${RTABMAP_ARGS:+${RTABMAP_ARGS} }--delete_db_on_start"
    ;;
esac

cleanup_stale_slam_processes

if is_truthy "${WAIT_FOR_INPUTS}"; then
  wait_for_topic_message "${POINT_CLOUD_TOPIC}" "点云" "${INPUT_TIMEOUT_SEC}" "sensor_msgs.msg.PointCloud2" || {
    echo "无法启动 RTAB-Map：${POINT_CLOUD_TOPIC} 没有实际 PointCloud2 消息。" >&2
    echo "请先确认平台点云桥已发布数据，或临时使用 --no-wait-inputs 跳过预检。" >&2
    exit 3
  }
  wait_for_topic_message "${ODOM_TOPIC}" "里程计" "${INPUT_TIMEOUT_SEC}" "nav_msgs.msg.Odometry" || {
    echo "无法启动 RTAB-Map：${ODOM_TOPIC} 没有实际 Odometry 消息。" >&2
    echo "请先确认 Go2 odom 数据源正常，或临时使用 --no-wait-inputs 跳过预检。" >&2
    exit 3
  }
fi

echo "启动 Go2 LiDAR SLAM: mode=${SLAM_MODE} database_path=${DATABASE_PATH} database_exists=${DATABASE_EXISTS} incremental_memory=${INCREMENTAL_MEMORY} init_wm_with_all_nodes=${INIT_WM_WITH_ALL_NODES}" >&2
echo "ROS: RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION} CYCLONEDDS_URI=${CYCLONEDDS_URI:-} point_cloud_topic=${POINT_CLOUD_TOPIC} odom_topic=${ODOM_TOPIC} planar_odom=${PLANAR_ODOM} auto_switch_to_mapping=${AUTO_SWITCH_TO_MAPPING}" >&2
echo "RTAB-Map localization params: start_at_origin=${START_AT_ORIGIN} optimize_max_error=${OPTIMIZE_MAX_ERROR} max_odom_cache_size=${MAX_ODOM_CACHE_SIZE}" >&2
if [[ -n "${INITIAL_POSE}" ]]; then
  echo "RTAB-Map initial_pose=${INITIAL_POSE}" >&2
fi

ROS_LAUNCH_ARGS=(
  "database_path:=${DATABASE_PATH}" \
  "use_sim_time:=${USE_SIM_TIME}" \
  "point_cloud_topic:=${POINT_CLOUD_TOPIC}" \
  "odom_topic:=${ODOM_TOPIC}" \
  "map_frame:=${MAP_FRAME}" \
  "odom_frame:=${ODOM_FRAME}" \
  "base_frame:=${BASE_FRAME}" \
  "lidar_frame:=${LIDAR_FRAME}" \
  "localization_odom_topic:=${LOCALIZATION_ODOM_TOPIC}" \
  "incremental_memory:=${INCREMENTAL_MEMORY}" \
  "init_wm_with_all_nodes:=${INIT_WM_WITH_ALL_NODES}" \
  "planar_odom:=${PLANAR_ODOM}" \
  "auto_switch_to_mapping:=${AUTO_SWITCH_TO_MAPPING}" \
  "start_at_origin:=${START_AT_ORIGIN}" \
  "optimize_max_error:=${OPTIMIZE_MAX_ERROR}" \
  "max_odom_cache_size:=${MAX_ODOM_CACHE_SIZE}"
)

if [[ -n "${RTABMAP_ARGS}" ]]; then
  ROS_LAUNCH_ARGS+=("rtabmap_args:=${RTABMAP_ARGS}")
fi

if [[ -n "${INITIAL_POSE}" ]]; then
  ROS_LAUNCH_ARGS+=("initial_pose:=${INITIAL_POSE}")
fi

exec ros2 launch go2_slam go2_lidar_slam.launch.py \
  "${ROS_LAUNCH_ARGS[@]}" \
  "${LAUNCH_ARGS[@]}"
