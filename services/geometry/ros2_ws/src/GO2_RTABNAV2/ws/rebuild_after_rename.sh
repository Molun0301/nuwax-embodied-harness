#!/usr/bin/env bash
# Rebuild GO2_RTABNAV2 workspace (repo-relative paths).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GO2_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS="${SCRIPT_DIR}"
SRC="${WS}/src"
LOG="${WS}/rebuild_after_rename.log"

exec > >(tee -a "${LOG}") 2>&1

echo "========== $(date -Is) rebuild_after_rename.sh =========="
echo "GO2_ROOT=${GO2_ROOT}"

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
echo "ROS distro: ${GO2_ROS_DISTRO}"

echo ""
echo "=== STEP 1: Check ws/src symlinks ==="
ls -la "${SRC}" || true

echo ""
echo "=== STEP 2: Create/fix symlinks ==="
mkdir -p "${SRC}"
cd "${SRC}"
ln -sfn "${GO2_ROOT}/unitree-go2-slam-nav2/go2_slam_nav" .
ln -sfn "${GO2_ROOT}/unitree-go2-slam-nav2/go2_cmd_processor" .
ln -sfn "${GO2_ROOT}/unitree-go2-slam-nav2/frontier" .
ln -sfn "${GO2_ROOT}/m-explore-ros2/explore" explore_lite
ln -sfn "${GO2_ROOT}/m-explore-ros2/explore_lite_msgs" .
echo "Symlinks after fix:"
ls -la "${SRC}"

echo ""
echo "=== STEP 3: Clean and rebuild ==="
cd "${WS}"
rm -rf build install log
export PATH="/usr/bin:/bin:/opt/ros/${GO2_ROS_DISTRO}/bin:${PATH}"
set +u
source "/opt/ros/${GO2_ROS_DISTRO}/setup.bash"
for _u in \
  "${HOME}/unitree_ros2/install/setup.bash" \
  "${HOME}/unitree_ros2/cyclonedds_ws/install/setup.bash" \
  "${HOME}/cyclonedds_ws/install/setup.bash"; do
  if [[ -f "${_u}" ]]; then
    source "${_u}"
    break
  fi
done
set -u
colcon build --symlink-install
BUILD_RC=$?
echo "colcon build exit code: ${BUILD_RC}"
if [[ ${BUILD_RC} -ne 0 ]]; then
  echo "BUILD FAILED"
  exit ${BUILD_RC}
fi
echo "BUILD SUCCEEDED"

echo ""
echo "=== STEP 4: Verify package prefixes ==="
set +u
source "${WS}/source_go2.bash"
set -u
echo "go2_slam_nav:      $(ros2 pkg prefix go2_slam_nav)"
echo "go2_cmd_processor: $(ros2 pkg prefix go2_cmd_processor)"
echo "explore_lite:      $(ros2 pkg prefix explore_lite)"

echo ""
echo "=== DONE ==="