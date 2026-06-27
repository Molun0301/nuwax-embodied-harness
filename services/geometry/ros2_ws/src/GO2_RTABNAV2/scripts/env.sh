#!/usr/bin/env bash
# Thin wrapper: GO2_RTABNAV2 ROS overlay + platform .env injection (NUWAX_GEOMETRY_*).
# Sourced by go2.sh and start_gateway.sh (Rust ros-bridge).

_THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_GO2_ROOT="$(cd "${_THIS_DIR}/.." && pwd)"
export GO2_WS="${GO2_WS:-${_GO2_ROOT}/ws}"

_had_nounset=0
case $- in *u*) _had_nounset=1; set +u ;; esac

if [[ -f "${GO2_WS}/source_go2.bash" ]]; then
  # shellcheck disable=SC1091
  source "${GO2_WS}/source_go2.bash"
else
  echo "[env.sh] WARNING: ${GO2_WS}/source_go2.bash not found" >&2
  if [[ -f "/opt/ros/${GO2_ROS_DISTRO:-foxy}/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "/opt/ros/${GO2_ROS_DISTRO:-foxy}/setup.bash"
  fi
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
fi

# shellcheck disable=SC1091
source "${_THIS_DIR}/load_platform_geometry_env.sh"

if [[ $_had_nounset -eq 1 ]]; then set -u; fi
unset _had_nounset _THIS_DIR _GO2_ROOT

echo "[env.sh] GO2_RTABNAV2 ready  RMW=${RMW_IMPLEMENTATION:-unknown}  occupancy=${NUWAX_GEOMETRY_OCCUPANCY_TOPIC:-/map_obstacles}"