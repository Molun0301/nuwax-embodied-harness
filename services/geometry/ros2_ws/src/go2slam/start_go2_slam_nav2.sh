#!/usr/bin/env bash

set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_SETUP="$ROOT_DIR/install/setup.bash"
GEOMETRY_RUNTIME_ENV_SCRIPT="$ROOT_DIR/../../../source_runtime_env.sh"
NAV2_PID=""
MAPPING_PID=""
BRIDGE_PID=""
ODOM_BRIDGE_PID=""
STOP_REQUESTED=false

detect_ros_setup() {
  local candidates=()
  if [[ -n "${NUWAX_ROS_SETUP_SCRIPT:-}" ]]; then
    candidates+=("${NUWAX_ROS_SETUP_SCRIPT}")
  fi
  if [[ -n "${ROS_DISTRO:-}" ]]; then
    candidates+=("/opt/ros/${ROS_DISTRO}/setup.bash")
  fi
  candidates+=(
    "/opt/ros/jazzy/setup.bash"
    "/opt/ros/humble/setup.bash"
    "/opt/ros/iron/setup.bash"
    "/opt/ros/foxy/setup.bash"
  )
  local candidate
  for candidate in "${candidates[@]}"; do
    if [[ -f "${candidate}" ]]; then
      echo "${candidate}"
      return 0
    fi
  done
  return 1
}

resolve_nav2_rviz_config() {
  local ros_setup="$1"
  local ros_prefix="${ros_setup%/setup.bash}"
  echo "${ros_prefix}/share/nav2_bringup/rviz/nav2_default_view.rviz"
}

ROS_SETUP="$(detect_ros_setup || true)"
RVIZ_CONFIG=""

# --- 新增参数处理 ---
START_MAPPING=false  # 默认不启动点云建图
START_RVIZ=true      # 默认启动 RViz

usage() {
  local status="${1:-1}"
  echo "Usage: $0 [--mapping] [--no-rviz]"
  echo "  --mapping    启动 Go2 点云建图 (mapping_only.launch.py)"
  echo "  --no-rviz    不启动 RViz，适用于无显示环境或远程终端"
  exit "${status}"
}

# 简单的参数解析
while [[ "$#" -gt 0 ]]; do
  case $1 in
    --mapping) START_MAPPING=true; shift ;;
    --no-rviz) START_RVIZ=false; shift ;;
    -h|--help) usage 0 ;;
    *) echo "Unknown parameter: $1"; usage ;;
  esac
done
# ------------------

cleanup() {
  STOP_REQUESTED=true
  # 只有在 PID 不为空时才尝试清理
  for pid in "${ODOM_BRIDGE_PID}" "${BRIDGE_PID}" "${MAPPING_PID}" "${NAV2_PID}"; do
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

workspace_stale_source() {
  local setup_file="$1"
  local source_file=""
  local search_roots=()
  [[ -d "${ROOT_DIR}/src" ]] && search_roots+=("${ROOT_DIR}/src")
  if [[ "${#search_roots[@]}" -eq 0 ]]; then
    return 1
  fi
  while IFS= read -r -d '' source_file; do
    if [[ "${source_file}" -nt "${setup_file}" ]]; then
      echo "${source_file}"
      return 0
    fi
  done < <(find "${search_roots[@]}" -type f \
    \( -name 'package.xml' -o -name 'CMakeLists.txt' -o -name '*.launch.py' -o -name '*.py' -o -name '*.cpp' -o -name '*.hpp' -o -name '*.h' -o -name '*.yaml' -o -name '*.xml' \) \
    -print0 2>/dev/null)
  return 1
}

assert_workspace_current() {
  local setup_file="$1"
  local stale_source=""
  stale_source="$(workspace_stale_source "${setup_file}" || true)"
  if [[ -n "${stale_source}" ]]; then
    echo "Workspace install is older than source: ${stale_source}" >&2
    echo "Rebuild first: cd ${ROOT_DIR} && source ${ROS_SETUP} && colcon build --symlink-install" >&2
    exit 1
  fi
}

# 环境检查
if [[ -z "${ROS_SETUP}" || ! -f "${ROS_SETUP}" ]]; then
  echo "Missing ROS setup. Set NUWAX_ROS_SETUP_SCRIPT or install ROS2 Jazzy/Humble/Iron/Foxy." >&2
  exit 1
fi
RVIZ_CONFIG="$(resolve_nav2_rviz_config "${ROS_SETUP}")"

if [[ ! -f "${WS_SETUP}" ]]; then
  echo "Missing workspace setup: ${WS_SETUP}" >&2
  echo "Build the workspace first: colcon build" >&2
  exit 1
fi

assert_workspace_current "${WS_SETUP}"

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

if [[ "${START_RVIZ}" = true && -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo "No GUI session detected. RViz needs DISPLAY or WAYLAND_DISPLAY." >&2
  exit 1
fi

source "${ROS_SETUP}"
source "${WS_SETUP}"
export NUWAX_GEOMETRY_RUNTIME_SOURCE_ROS_ENV=false
export NUWAX_GEOMETRY_RUNTIME_FORCE_CYCLONEDDS=true
source "${GEOMETRY_RUNTIME_ENV_SCRIPT}"

set -u

# RViz 配置选择
if ros2 pkg prefix go2_navigation2 >/dev/null 2>&1; then
  RVIZ_CONFIG="$(ros2 pkg prefix go2_navigation2)/share/go2_navigation2/rviz/nav2_map_view.rviz"
fi

# 启动 Nav2
echo "Starting Go2 Nav2 in SLAM mode..."
NAV2_ARGS=(localization:=slam use_rviz:=false)
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

# 启动 odom_tf_bridge（go2_driver 的 pose_callback 依赖 utlidar/robot_pose，
# 如果该 topic 无数据则 driver 不发布 TF，需要 odom_tf_bridge 补充）
REPO_ROOT="${ROOT_DIR}/../../../../.."
ODOM_BRIDGE_SCRIPT="${REPO_ROOT}/services/geometry/odom_tf_bridge.py"
if [[ -f "${ODOM_BRIDGE_SCRIPT}" ]]; then
  echo "Starting odom_tf_bridge..."
  python3 "${ODOM_BRIDGE_SCRIPT}" --odom-topic /odom --default-odom-frame odom --odom-frame odom --default-child-frame base_footprint --child-frame base_footprint &
  ODOM_BRIDGE_PID=$!
fi

# 启动 Nav2 Goal Bridge（topic-to-action 桥接，解决跨 RMW action 通信 bug）
BRIDGE_SCRIPT="${REPO_ROOT}/services/geometry/nav2_goal_bridge.py"
if [[ -f "${BRIDGE_SCRIPT}" ]]; then
  echo "Starting Nav2 Goal Bridge..."
  python3 "${BRIDGE_SCRIPT}" &
  BRIDGE_PID=$!
fi

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
  set +e
  if [[ "${START_MAPPING}" = true ]]; then
    wait -n "${NAV2_PID}" "${MAPPING_PID}"
  else
    wait "${NAV2_PID}"
  fi
  exit_code="$?"
  set -e
  if [[ "${STOP_REQUESTED}" == true ]]; then
    exit 0
  fi
  echo "Go2 SLAM/Nav2 launch exited unexpectedly with code ${exit_code}. Check runtime_data/geometry/logs/external_slam_nav2.log." >&2
  exit 1
fi

echo "Starting RViz..."
rviz2 -d "${RVIZ_CONFIG}"
