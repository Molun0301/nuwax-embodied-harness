#!/usr/bin/env bash

set -euo pipefail

# Kill ROS 2 related processes gracefully first, then force kill survivors.
# Usage:
#   ./kill_all_ros2.sh
#   ./kill_all_ros2.sh --force

GRACE_SECONDS=3
FORCE_ONLY=0

if [[ "${1:-}" == "--force" ]]; then
  FORCE_ONLY=1
fi

SELF_PID=$$

patterns=(
  "ros2"
  "_ros2_daemon"
  "launch_ros"
  "rviz2"
  "rqt"
  "rosbag"
  "component_container"
  "robot_state_publisher"
  "static_transform_publisher"
  "/opt/ros/"
  "/home/huang/baobao/ros2_ws/install/"
  "/home/huang/baobao/ros2_ws/build/"
)

collect_pids() {
  local pattern
  for pattern in "${patterns[@]}"; do
    pgrep -f "$pattern" || true
  done | sort -nu | awk -v self="$SELF_PID" '$1 != self'
}

print_matches() {
  local pid
  while read -r pid; do
    [[ -z "$pid" ]] && continue
    ps -p "$pid" -o pid=,ppid=,stat=,cmd= || true
  done
}

filter_alive_pids() {
  local pid
  while read -r pid; do
    [[ -z "$pid" ]] && continue
    if kill -0 "$pid" 2>/dev/null; then
      echo "$pid"
    fi
  done
}

send_signal_to_pids() {
  local signal="$1"
  local pid
  while read -r pid; do
    [[ -z "$pid" ]] && continue
    if kill -0 "$pid" 2>/dev/null; then
      if [[ -n "$signal" ]]; then
        kill "$signal" "$pid" 2>/dev/null || true
      else
        kill "$pid" 2>/dev/null || true
      fi
    fi
  done
}

PIDS="$(collect_pids)"
PIDS="$(printf '%s\n' "$PIDS" | filter_alive_pids)"

if [[ -z "$PIDS" ]]; then
  echo "No ROS 2 related processes found."
  exit 0
fi

echo "Matched ROS 2 related processes:"
printf '%s\n' "$PIDS" | print_matches

if [[ "$FORCE_ONLY" -eq 1 ]]; then
  echo
  echo "Force killing..."
  printf '%s\n' "$PIDS" | send_signal_to_pids -9
else
  echo
  echo "Sending SIGTERM..."
  printf '%s\n' "$PIDS" | send_signal_to_pids ""

  sleep "$GRACE_SECONDS"

  REMAINING="$(printf '%s\n' "$PIDS" | while read -r pid; do kill -0 "$pid" 2>/dev/null && echo "$pid"; done)"

  if [[ -n "$REMAINING" ]]; then
    echo "Still running after ${GRACE_SECONDS}s, sending SIGKILL:"
    printf '%s\n' "$REMAINING" | print_matches
    printf '%s\n' "$REMAINING" | send_signal_to_pids -9
  fi
fi

echo
echo "Done."
