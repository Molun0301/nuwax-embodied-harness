#!/usr/bin/env bash
# ============================================================================
# Environment setup for GO2 + RTAB-Map workspace.
#
# Mirrors the 'choose 1' branch of ~/.bashrc so the run scripts work whether
# the user has interactively selected option 1 or not. Critical because the
# Unitree GO2 publisher uses CycloneDDS with a specific config XML; without
# RMW_IMPLEMENTATION + CYCLONEDDS_URI set correctly we cannot subscribe to
# /utlidar/* or send unitree_api commands.
#
# Sourced by run_mapping.sh / run_localization.sh / run_exploration.sh.
# ============================================================================

# Locate workspace root (parent of this scripts/ dir).
_THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$( cd "$_THIS_DIR/.." && pwd )"

# Be careful: in nounset mode unset vars will crash sourced setup.bash files.
# Save and disable transiently.
_had_nounset=0
case $- in *u*) _had_nounset=1; set +u ;; esac

# ---- ROS 2 Foxy
source /opt/ros/foxy/setup.bash

# ---- CycloneDDS RMW (matches Unitree GO2 driver expectations)
source "$HOME/cyclonedds_ws/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="$HOME/cyclonedds_ws/cyclonedds.xml"
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"

# ---- CUDA (kept in sync with .bashrc for any CUDA-linked rtabmap libs)
export PATH="/usr/local/cuda-11.4/bin:${PATH:-}"
export LD_LIBRARY_PATH="/usr/local/cuda-11.4/lib64:${LD_LIBRARY_PATH:-}"

# ---- CycloneDDS native install (used by cyclonedds_ws build)
if [[ -d "$HOME/cyclonedds/install" ]]; then
  export CYCLONEDDS_HOME="$HOME/cyclonedds/install"
  export CMAKE_PREFIX_PATH="$CYCLONEDDS_HOME:${CMAKE_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="$CYCLONEDDS_HOME/lib:${LD_LIBRARY_PATH:-}"
fi

# ---- Unitree messages workspace (provides unitree_api, unitree_go, etc.)
if [[ -f "$HOME/unitree_msgs_ws/install/setup.bash" ]]; then
  source "$HOME/unitree_msgs_ws/install/setup.bash"
fi

# ---- GO2_RTABMAP workspace overlay
if [[ -f "$WORKSPACE_DIR/install/setup.bash" ]]; then
  source "$WORKSPACE_DIR/install/setup.bash"
else
  echo "WARNING: $WORKSPACE_DIR/install/setup.bash not found." >&2
  echo "         Did you forget to run 'colcon build' inside $WORKSPACE_DIR ?" >&2
fi

# Restore nounset if caller had it on.
if [[ $_had_nounset -eq 1 ]]; then set -u; fi
unset _had_nounset _THIS_DIR

echo "[env.sh] ROS_DISTRO=foxy  RMW=$RMW_IMPLEMENTATION  CYCLONEDDS_URI=$CYCLONEDDS_URI"
