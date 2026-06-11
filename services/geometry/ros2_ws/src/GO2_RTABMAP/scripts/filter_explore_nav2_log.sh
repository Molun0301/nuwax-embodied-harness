#!/usr/bin/env bash
# Filter ros2 launch stdout for Nav2 + frontier exploration analysis.
# Used by: go2.sh --explore --explore-log [FILE]

set -euo pipefail

# ROS2 console: [node_name] [LEVEL] [time] [logger]: message
# go2.sh banners: [go2.sh] ...
FILTER_RE='\[go2\.sh\].*(explore|nav2|MODE = exploration)|frontier_explorer|bt_navigator|controller_server|planner_server|recoveries_server|recovery_server|lifecycle_manager|navigate_to_pose|nav2_|amcl|map_server|costmap|cmd_vel_bridge|Planner|Controller|recovery|Goal |goal |blacklist|Blacklisted|Selected frontier|Reached frontier|abandoned|STRATEGY:|DISPATCHING|NAVIGATING|PAUSE|HOLDING|WARMING|bootstrap|escape|WAITING|TIMEOUT|RETRY|failures=|SUCCEEDED|ABORTED|CANCELED|Rejected|send_goal|FollowPath|ComputePath'

while IFS= read -r line || [[ -n "$line" ]]; do
  if [[ "$line" =~ $FILTER_RE ]]; then
    printf '%s\n' "$line"
  fi
done
