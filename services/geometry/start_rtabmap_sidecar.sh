#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="${NUWAX_REPO_ROOT:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
LOG_DIR="${NUWAX_GEOMETRY_LOG_DIR:-${REPO_ROOT}/runtime_data/geometry/logs}"
DB_ROOT="${NUWAX_RTABMAP_DATABASE_ROOT:-${REPO_ROOT}/runtime_data/geometry/rtabmap}"
mkdir -p "${LOG_DIR}" "${DB_ROOT}"
cd "${REPO_ROOT}"

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-<CycloneDDS><Domain><Compatibility><StandardsConformance>lax</StandardsConformance></Compatibility></Domain></CycloneDDS>}"

ODOM_TOPIC="${NUWAX_GEOMETRY_ODOM_TOPIC:-/utlidar/robot_odom}"
IMU_TOPIC="${NUWAX_GEOMETRY_IMU_TOPIC:-/utlidar/imu}"
POINT_CLOUD_TOPIC="${NUWAX_GEOMETRY_POINT_CLOUD_TOPIC:-/utlidar/cloud}"
MAP_FRAME="${NUWAX_GEOMETRY_MAP_FRAME:-odom}"
ODOM_FRAME="${NUWAX_GEOMETRY_ODOM_FRAME:-odom}"
BASE_FRAME="${NUWAX_GEOMETRY_BASE_FRAME:-body}"
LIDAR_FRAME="${NUWAX_GEOMETRY_LIDAR_FRAME:-utlidar_lidar}"
IMU_FRAME="${NUWAX_GEOMETRY_IMU_FRAME:-utlidar_imu}"
RTABMAP_LAUNCH_PACKAGE="${NUWAX_RTABMAP_LAUNCH_PACKAGE:-rtabmap_ros}"
RTABMAP_LAUNCH_FILE="${NUWAX_RTABMAP_LAUNCH_FILE:-rtabmap.launch.py}"
RTABMAP_DB_PATH="${NUWAX_RTABMAP_DATABASE_PATH:-${DB_ROOT}/live_rtabmap.db}"
NAV2_MODE="${NUWAX_NAV2_MODE:-external}"
NAV2_LAUNCH_PACKAGE="${NUWAX_NAV2_LAUNCH_PACKAGE:-nav2_bringup}"
NAV2_LAUNCH_FILE="${NUWAX_NAV2_LAUNCH_FILE:-navigation_launch.py}"
NAV2_MAP_YAML="${NUWAX_NAV2_MAP_YAML:-}"
NAV2_PARAMS_FILE="${NUWAX_NAV2_PARAMS_FILE:-}"
NAV2_USE_SIM_TIME="${NUWAX_NAV2_USE_SIM_TIME:-false}"
NAV2_AUTOSTART="${NUWAX_NAV2_AUTOSTART:-true}"

PIDS=()
cleanup() {
  local pid=""
  for pid in "${PIDS[@]}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" >/dev/null 2>&1; then
      kill "${pid}" >/dev/null 2>&1 || true
    fi
  done
  wait >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

start_background() {
  local log_file="$1"
  shift
  "$@" > "${LOG_DIR}/${log_file}" 2>&1 &
  PIDS+=("$!")
}

start_static_tf() {
  local log_name="$1"
  local parent_frame="$2"
  local child_frame="$3"
  if [[ -z "${parent_frame}" || -z "${child_frame}" || "${parent_frame}" == "${child_frame}" ]]; then
    return 0
  fi
  start_background "${log_name}.log" \
    bash "${REPO_ROOT}/services/geometry/launch_ros2_stack.sh" \
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 "${parent_frame}" "${child_frame}"
}

start_background "odom_tf_bridge.log" \
  bash "${REPO_ROOT}/services/geometry/launch_ros2_stack.sh" \
  python3 "${REPO_ROOT}/services/geometry/odom_tf_bridge.py" \
  --odom-topic "${ODOM_TOPIC}" \
  --default-child-frame "${BASE_FRAME}" \
  --child-frame "${BASE_FRAME}"

start_static_tf "static_tf_base_to_body" "${BASE_FRAME}" "body"
start_static_tf "static_tf_base_to_lidar" "${BASE_FRAME}" "${LIDAR_FRAME}"
start_static_tf "static_tf_base_to_imu" "${BASE_FRAME}" "${IMU_FRAME}"

start_background "rtabmap_live.log" \
  bash "${REPO_ROOT}/services/geometry/launch_ros2_stack.sh" \
  ros2 launch "${RTABMAP_LAUNCH_PACKAGE}" "${RTABMAP_LAUNCH_FILE}" \
  namespace:='/' rtabmapviz:=false rviz:=false \
  depth:=false subscribe_rgb:=false subscribe_rgbd:=false \
  frame_id:="${BASE_FRAME}" map_frame_id:="${MAP_FRAME}" odom_frame_id:="${ODOM_FRAME}" publish_tf_map:=true \
  visual_odometry:=false icp_odometry:=false \
  subscribe_scan_cloud:=true scan_cloud_topic:="${POINT_CLOUD_TOPIC}" \
  odom_topic:="${ODOM_TOPIC}" imu_topic:="${IMU_TOPIC}" \
  qos:=2 qos_scan:=2 qos_odom:=1 qos_imu:=2 \
  approx_sync:=false queue_size:=30 wait_for_transform:=0.5 \
  database_path:="${RTABMAP_DB_PATH}" \
  args:='--delete_db_on_start --RGBD/CreateOccupancyGrid true --Grid/FromDepth false --Grid/RangeMax 8.0 --Grid/CellSize 0.05 --Grid/3D true --Grid/RayTracing true --Grid/FlatObstacleDetected true --Grid/MaxGroundHeight 0.35 --Grid/MaxObstacleHeight 1.8 --Grid/NormalsSegmentation false --Grid/MinClusterSize 5'

case "${NAV2_MODE,,}" in
  sidecar|managed|launch|internal)
    NAV2_ARGS=(bash "${REPO_ROOT}/services/geometry/launch_ros2_stack.sh" ros2 launch "${NAV2_LAUNCH_PACKAGE}" "${NAV2_LAUNCH_FILE}" "use_sim_time:=${NAV2_USE_SIM_TIME}" "autostart:=${NAV2_AUTOSTART}")
    if [[ -n "${NAV2_MAP_YAML}" ]]; then
      NAV2_ARGS+=("map:=${NAV2_MAP_YAML}")
    fi
    if [[ -n "${NAV2_PARAMS_FILE}" ]]; then
      NAV2_ARGS+=("params_file:=${NAV2_PARAMS_FILE}")
    fi
    start_background "nav2.log" "${NAV2_ARGS[@]}"
    ;;
  external|external_action|action|client|pending|disabled|off|none|"")
    ;;
  *)
    printf '未知 NUWAX_NAV2_MODE=%s，Nav2 未启动。\n' "${NAV2_MODE}" >&2
    ;;
esac

wait "${PIDS[@]}"
