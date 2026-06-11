#!/usr/bin/env bash
# Export Nav2 + frontier_explorer lines from the latest ~/.ros/log session.
#
# Usage:
#   ./scripts/export_explore_nav2_ros_log.sh [OUTPUT_FILE]
#
# Default output: ~/logs/go2_explore_ros_export_YYYYMMDD_HHMMSS.log

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FILTER="$SCRIPT_DIR/filter_explore_nav2_log.sh"

LATEST_LOG_DIR="$(ls -td "${HOME}/.ros/log/"* 2>/dev/null | head -1 || true)"
if [[ -z "$LATEST_LOG_DIR" ]]; then
  echo "ERROR: no ~/.ros/log sessions found." >&2
  exit 1
fi

OUT="${1:-${HOME}/logs/go2_explore_ros_export_$(date +%Y%m%d_%H%M%S).log}"
mkdir -p "$(dirname "$OUT")"

{
  echo "===== exported from ${LATEST_LOG_DIR} at $(date -Iseconds) ====="
  find "$LATEST_LOG_DIR" -type f -name '*.log' -print0 \
    | sort -z \
    | xargs -0 cat 2>/dev/null || true
} | bash "$FILTER" | tee "$OUT"

echo "[export_explore_nav2_ros_log] wrote $(wc -l < "$OUT") lines -> $OUT"
