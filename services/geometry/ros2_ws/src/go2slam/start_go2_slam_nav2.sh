#!/usr/bin/env bash

set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="$ROOT_DIR/install/setup.bash"
RVIZ_CONFIG="/opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz"
EXPLORE_PARAMS_FILE="$ROOT_DIR/src/go2_navigation2/config/explore_params.yaml"
NAV2_PID=""
MAPPING_PID=""

# --- 新增参数处理 ---
START_MAPPING=false  # 默认不启动点云建图
START_EXPLORE=false  # 默认不启动 frontier 探索
START_RVIZ=true      # 默认启动 RViz

usage() {
  local status="${1:-1}"
  echo "Usage: $0 [--mapping] [--explore] [--no-rviz]"
  echo "  --mapping    启动 Go2 点云建图 (mapping_only.launch.py)"
  echo "  --explore    启动 m-explore-ros2 frontier 自动探索"
  echo "  --no-rviz    不启动 RViz，适用于无显示环境或远程终端"
  exit "${status}"
}

# 简单的参数解析
while [[ "$#" -gt 0 ]]; do
  case $1 in
    --mapping) START_MAPPING=true; shift ;;
    --explore) START_EXPLORE=true; shift ;;
    --no-rviz) START_RVIZ=false; shift ;;
    -h|--help) usage 0 ;;
    *) echo "Unknown parameter: $1"; usage ;;
  esac
done
# ------------------

cleanup() {
  # 只有在 PID 不为空时才尝试清理
  for pid in "${MAPPING_PID}" "${NAV2_PID}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -INT "${pid}" 2>/dev/null || true
      for _ in {1..30}; do
        kill -0 "${pid}" 2>/dev/null || break
        sleep 0.1
      done
      kill -0 "${pid}" 2>/dev/null && kill "${pid}" 2>/dev/null || true
      wait "${pid}" 2>/dev/null || true
    fi
  done
}

trap cleanup EXIT INT TERM

configure_cyclonedds_participant_limit() {
  local max_auto_participant_index=100

  if [[ "${RMW_IMPLEMENTATION:-}" != "rmw_cyclonedds_cpp" && -z "${CYCLONEDDS_URI:-}" ]]; then
    return
  fi

  if [[ -z "${CYCLONEDDS_URI:-}" ]]; then
    export CYCLONEDDS_URI="<CycloneDDS><Domain><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>${max_auto_participant_index}</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>"
    return
  fi

  if [[ "${CYCLONEDDS_URI}" == *"MaxAutoParticipantIndex"* ]]; then
    return
  fi

  if [[ "${CYCLONEDDS_URI}" == "<CycloneDDS"* && "${CYCLONEDDS_URI}" == *"</Discovery>"* ]]; then
    local discovery_close="</Discovery>"
    local discovery_with_limit="<ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>${max_auto_participant_index}</MaxAutoParticipantIndex></Discovery>"
    export CYCLONEDDS_URI="${CYCLONEDDS_URI/$discovery_close/$discovery_with_limit}"
    echo "Configured CycloneDDS MaxAutoParticipantIndex=${max_auto_participant_index}"
    return
  fi

  echo "Warning: CYCLONEDDS_URI is set but could not be patched automatically. If nodes fail with participant index errors, add MaxAutoParticipantIndex=${max_auto_participant_index} to the CycloneDDS config." >&2
}

# 环境检查
if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "Missing ROS setup: ${ROS_SETUP}" >&2
  exit 1
fi

if [[ ! -f "${WS_SETUP}" ]]; then
  echo "Missing workspace setup: ${WS_SETUP}" >&2
  echo "Build the workspace first: colcon build" >&2
  exit 1
fi

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

if [[ "${START_RVIZ}" = true && -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo "No GUI session detected. RViz needs DISPLAY or WAYLAND_DISPLAY." >&2
  exit 1
fi

source "${ROS_SETUP}"
source "${WS_SETUP}"
configure_cyclonedds_participant_limit

set -u

# RViz 配置选择
if ros2 pkg prefix go2_navigation2 >/dev/null 2>&1; then
  RVIZ_CONFIG="$(ros2 pkg prefix go2_navigation2)/share/go2_navigation2/rviz/nav2_map_view.rviz"
fi

# 启动 Nav2
echo "Starting Go2 Nav2 in SLAM mode..."
NAV2_ARGS=(localization:=slam use_rviz:=false)
if [[ "${START_EXPLORE}" = true ]]; then
  if ! ros2 pkg prefix explore_lite >/dev/null 2>&1; then
    echo "Error: Missing ROS package 'explore_lite' but --explore was requested." >&2
    echo "Build the workspace first, including m-explore-ros2: colcon build --symlink-install" >&2
    exit 1
  fi
  if [[ ! -f "${EXPLORE_PARAMS_FILE}" ]]; then
    echo "Error: Missing explore params file: ${EXPLORE_PARAMS_FILE}" >&2
    exit 1
  fi
  NAV2_ARGS+=(use_explore:=true "explore_params_file:=${EXPLORE_PARAMS_FILE}")
fi
ros2 launch go2_navigation2 go2_nav2.launch.py "${NAV2_ARGS[@]}" &
NAV2_PID=$!

# --- 条件启动点云建图 ---
if [[ "${START_MAPPING}" = true ]]; then
  if ! ros2 pkg prefix go2_mapping_only >/dev/null 2>&1; then
    echo "Error: Missing ROS package 'go2_mapping_only' but --mapping was requested." >&2
    exit 1
  fi
  echo "Starting Go2 point-cloud map builder..."
  ros2 launch go2_mapping_only mapping_only.launch.py rviz:=false &
  MAPPING_PID=$!
fi
# -----------------------

sleep 4

# 运行状态检查
if ! kill -0 "${NAV2_PID}" 2>/dev/null; then
  echo "Navigation launch exited before startup completed." >&2
  exit 1
fi

if [[ "${START_MAPPING}" = true ]] && ! kill -0 "${MAPPING_PID}" 2>/dev/null; then
  echo "Point-cloud mapping launch exited before startup completed." >&2
  exit 1
fi

if [[ "${START_RVIZ}" != true ]]; then
  echo "RViz disabled. Press Ctrl-C to stop Go2 Nav2."
  if [[ "${START_MAPPING}" = true ]]; then
    wait -n "${NAV2_PID}" "${MAPPING_PID}"
  else
    wait "${NAV2_PID}"
  fi
  exit $?
fi

echo "Starting RViz..."
rviz2 -d "${RVIZ_CONFIG}"
