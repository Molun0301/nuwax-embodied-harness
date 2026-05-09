#!/usr/bin/env bash
set -euo pipefail

cat >&2 <<'MSG'
已废弃旧 sidecar：旧 RTAB-Map/Nav2 入口已删除。
请使用 services/geometry/ros2_ws/src/go2slam/start_go2_slam_nav2.sh 启动 external_slam_nav2 外部 SLAM/Nav2 栈。
MSG
exit 64
