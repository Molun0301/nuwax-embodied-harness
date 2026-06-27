#!/usr/bin/env bash
# Load NUWAX_GEOMETRY_* / database root keys from the platform .env for go2.sh / ROS launch.
# Sourced by scripts/env.sh.

_go2_load_env_value() {
  local key="$1"
  local default_value="$2"
  local line value

  if [[ -n "${!key:-}" ]]; then
    return 0
  fi

  if [[ -f "${GO2_PLATFORM_ENV_FILE}" ]]; then
    line="$(grep -E "^[[:space:]]*${key}=" "${GO2_PLATFORM_ENV_FILE}" | tail -n 1 || true)"
    if [[ -n "${line}" ]]; then
      value="${line#*=}"
      value="${value%%#*}"
      value="$(echo "${value}" | sed -e 's/^[[:space:]]*//' -e 's/[[:space:]]*$//' -e 's/^"//' -e 's/"$//' -e "s/^'//" -e "s/'$//")"
      if [[ -n "${value}" ]]; then
        export "${key}=${value}"
        return 0
      fi
    fi
  fi

  export "${key}=${default_value}"
}

_go2_script_dir="$(cd "$(dirname "${BASH_SOURCE[1]:-${BASH_SOURCE[0]}}")" && pwd)"
_go2_workspace_dir="$(cd "${_go2_script_dir}/.." && pwd)"
GO2_PLATFORM_REPO_ROOT="${NUWAX_REPO_ROOT:-$(cd "${_go2_workspace_dir}/../../../../.." && pwd)}"
GO2_PLATFORM_ENV_FILE="${GO2_PLATFORM_ENV_FILE:-${GO2_PLATFORM_REPO_ROOT}/.env}"

_go2_load_env_value NUWAX_GEOMETRY_ODOM_TOPIC "/utlidar/robot_odom"
_go2_load_env_value NUWAX_GEOMETRY_POINT_CLOUD_TOPIC "/utlidar/cloud_deskewed"
_go2_load_env_value NUWAX_GEOMETRY_SCAN_TOPIC "/scan"
_go2_load_env_value NUWAX_GEOMETRY_PLAN_TOPIC "/plan"
_go2_load_env_value NUWAX_GEOMETRY_OCCUPANCY_TOPIC "/map_obstacles"
_go2_load_env_value NUWAX_GEOMETRY_MAP_FRAME "map"
_go2_load_env_value NUWAX_GEOMETRY_ODOM_FRAME "odom"
_go2_load_env_value NUWAX_GEOMETRY_BASE_FRAME "go2_base_link"
_go2_load_env_value NUWAX_GEOMETRY_POSE_TOPIC "/go2/pose_map"
_go2_load_env_value NUWAX_GEOMETRY_LIDAR_FRAME "utlidar_lidar"
_go2_load_env_value NUWAX_GEOMETRY_IMU_TOPIC "/utlidar/imu"
_go2_load_env_value NUWAX_GEOMETRY_ROBOT_RADIUS "0.30"
_go2_load_env_value NUWAX_GEOMETRY_INFLATION_RADIUS "0.85"
_go2_load_env_value NUWAX_GEOMETRY_INFLATION_COST_SCALING "2.5"
_go2_load_env_value NUWAX_GEOMETRY_EXTERNAL_FRONTIER_TOPIC "explore/frontiers"
_go2_load_env_value NUWAX_NAV2_ACTION_NAME "/navigate_to_pose"
# Route B：/map_obstacles 仅融合 RTAB /map + /cloud_obstacles；实时扫描走 Nav2 voxel_layer
_go2_load_env_value GO2_OBSTACLE_GRID_LIVE_CLOUD_ENABLED "false"
_go2_load_env_value GO2_OBSTACLE_GRID_PUBLISH_ON_CLOUD "false"

_default_db_root="${GO2_PLATFORM_REPO_ROOT}/runtime_data/geometry/rtabmap"
_go2_load_env_value NUWAX_BACKEND_DATABASE_ROOT "${_default_db_root}"
if [[ -z "${NUWAX_RTABMAP_DATABASE_ROOT:-}" ]]; then
  export NUWAX_RTABMAP_DATABASE_ROOT="${NUWAX_BACKEND_DATABASE_ROOT}"
fi

unset -f _go2_load_env_value
unset _go2_script_dir _go2_workspace_dir _default_db_root