#!/usr/bin/env bash
# Load NUWAX_GEOMETRY_* keys from the platform .env for go2.sh / ROS launch files.
# Sourced by go2.sh after scripts/env.sh.

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
_go2_load_env_value NUWAX_GEOMETRY_OCCUPANCY_TOPIC "/map"
_go2_load_env_value NUWAX_GEOMETRY_MAP_FRAME "map"
_go2_load_env_value NUWAX_GEOMETRY_ODOM_FRAME "odom"
_go2_load_env_value NUWAX_GEOMETRY_BASE_FRAME "base_link"
_go2_load_env_value NUWAX_GEOMETRY_LIDAR_FRAME "utlidar_lidar"
_go2_load_env_value NUWAX_GEOMETRY_IMU_TOPIC "/utlidar/imu"
_go2_load_env_value NUWAX_GEOMETRY_ROBOT_RADIUS "0.30"
_go2_load_env_value NUWAX_GEOMETRY_INFLATION_RADIUS "0.60"
_go2_load_env_value NUWAX_GEOMETRY_INFLATION_COST_SCALING "2.5"
_go2_load_env_value NUWAX_NAV2_ACTION_NAME "/navigate_to_pose"

unset -f _go2_load_env_value
unset _go2_script_dir _go2_workspace_dir
