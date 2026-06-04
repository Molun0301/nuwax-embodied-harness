#!/usr/bin/env bash
# ============================================================================
# GO2 + RTAB-Map + Nav2 launcher
# ----------------------------------------------------------------------------
# One script, three modes (pick exactly one):
#
#   ./go2.sh --new          [DB_PATH]   Build a NEW map (manual teleop / Nav2)
#   ./go2.sh --localization  DB_PATH    Load existing map and navigate (no
#                                       map updates). DB_PATH is REQUIRED.
#   ./go2.sh --explore      [DB_PATH]   Build a new map AUTONOMOUSLY using
#                                       frontier exploration.
#
# Aliases:
#   --new   == --mapping == -m
#   --localization == --loc == -l
#   --explore == --exploration == -e
#
# Default DB_PATH is ~/rtabmap.db. For --new / --explore the database is
# overwritten; for --localization the database is read-only and must exist.
#
# Common environment (CycloneDDS / Unitree DDS) is loaded from scripts/env.sh
# so this works even when the calling shell hasn't picked the bashrc
# 'choose 1' branch yet.
# ============================================================================
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# ---------------------------------------------------------------------------
# Help
# ---------------------------------------------------------------------------
usage() {
  cat <<EOF
GO2 + RTAB-Map + Nav2 launcher

Usage:
  $(basename "$0") <mode> [DB_PATH]

Modes:
  -m, --new, --mapping             Build a fresh map (manual / RViz goals)
  -l, --loc, --localization        Load existing map, navigate only
                                   (DB_PATH is REQUIRED)
  -e, --explore, --exploration     Build a fresh map autonomously
                                   via frontier exploration

Arguments:
  DB_PATH   Path to the RTAB-Map database.
              new / explore : default \$HOME/rtabmap.db, will be OVERWRITTEN
              localization  : required, must exist

Options:
  -h, --help                       Show this help

Examples:
  $(basename "$0") --new
  $(basename "$0") --new ~/maps/lab.db
  $(basename "$0") --localization ~/rtabmap.db
  $(basename "$0") --explore ~/maps/floor3.db
EOF
}

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------
if [[ $# -lt 1 ]]; then
  usage
  exit 2
fi

MODE=""
DB_PATH=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    -m|--new|--mapping)
      [[ -n "$MODE" ]] && { echo "ERROR: pick exactly one mode." >&2; exit 2; }
      MODE="mapping"
      shift
      ;;
    -l|--loc|--localization)
      [[ -n "$MODE" ]] && { echo "ERROR: pick exactly one mode." >&2; exit 2; }
      MODE="localization"
      shift
      ;;
    -e|--explore|--exploration)
      [[ -n "$MODE" ]] && { echo "ERROR: pick exactly one mode." >&2; exit 2; }
      MODE="exploration"
      shift
      ;;
    -*)
      echo "ERROR: unknown option '$1'" >&2
      usage
      exit 2
      ;;
    *)
      if [[ -n "$DB_PATH" ]]; then
        echo "ERROR: unexpected extra argument '$1' (DB_PATH already set to '$DB_PATH')" >&2
        exit 2
      fi
      DB_PATH="$1"
      shift
      ;;
  esac
done

if [[ -z "$MODE" ]]; then
  echo "ERROR: missing mode flag (--new / --localization / --explore)" >&2
  usage
  exit 2
fi

# ---------------------------------------------------------------------------
# DB path resolution (handle ~, relative, defaults)
# ---------------------------------------------------------------------------
DB_PATH="${DB_PATH:-$HOME/rtabmap.db}"
DB_PATH="${DB_PATH/#\~/$HOME}"
if [[ ! "$DB_PATH" = /* ]]; then
  DB_PATH="$(pwd)/$DB_PATH"
fi

# Localization requires an existing file
if [[ "$MODE" == "localization" && ! -f "$DB_PATH" ]]; then
  echo "ERROR: localization mode requires an existing database." >&2
  echo "       Not found: $DB_PATH" >&2
  echo "       Build one first:  ./$(basename "$0") --new" >&2
  exit 1
fi

# For mapping / exploration we WILL create the file; make sure the parent
# directory exists (RTAB-Map crashes hard with SIGABRT if it cannot open
# the sqlite database, and that takes the whole Nav2 lifecycle with it).
if [[ "$MODE" == "mapping" || "$MODE" == "exploration" ]]; then
  DB_DIR="$(dirname "$DB_PATH")"
  if [[ ! -d "$DB_DIR" ]]; then
    echo "[go2.sh] Creating database directory: $DB_DIR"
    mkdir -p "$DB_DIR"
  fi
fi

# ---------------------------------------------------------------------------
# Environment
# ---------------------------------------------------------------------------
source "$SCRIPT_DIR/scripts/env.sh"

# ---------------------------------------------------------------------------
# Kill any stale processes from a previous run so launch can rebind cleanly.
# ---------------------------------------------------------------------------
echo "[go2.sh] Cleaning up any leftover Nav2/RTAB-Map/explorer processes..."
KILL_PATTERNS=(
  "ros2 launch go2_nav"
  "go2_mapping.launch.py|go2_navigation.launch.py|go2_exploration.launch.py"
  "bt_navigator|planner_server|controller_server|recoveries_server|behavior_server|smoother_server|waypoint_follower|velocity_smoother"
  "map_server|amcl|lifecycle_manager"
  "rtabmap|rtabmap_ros|rtabmap_viz|rtabmap_sync"
  "cloud_to_scan|odom_tf_broadcaster|cmd_vel_bridge|frontier_explorer"
)

kill_matching_processes() {
  local signal_name="$1"
  local pattern="$2"
  local pid
  local cmd

  while IFS= read -r pid; do
    [[ -z "$pid" ]] && continue
    # The DB path passed to this script contains "/rtabmap/..."; a broad
    # "rtabmap" cleanup pattern must not kill this launcher itself.
    [[ "$pid" == "$$" ]] && continue
    cmd="$(ps -p "$pid" -o command= 2>/dev/null || true)"
    [[ -z "$cmd" ]] && continue
    if [[ "$cmd" == *"$SCRIPT_DIR/go2.sh"* ]] || [[ "$cmd" == *"/go2.sh "* ]]; then
      continue
    fi
    kill "-$signal_name" "$pid" 2>/dev/null || true
  done < <(pgrep -f "$pattern" 2>/dev/null || true)
}

has_matching_processes() {
  local pattern="$1"
  local pid
  local cmd

  while IFS= read -r pid; do
    [[ -z "$pid" ]] && continue
    [[ "$pid" == "$$" ]] && continue
    cmd="$(ps -p "$pid" -o command= 2>/dev/null || true)"
    [[ -z "$cmd" ]] && continue
    if [[ "$cmd" == *"$SCRIPT_DIR/go2.sh"* ]] || [[ "$cmd" == *"/go2.sh "* ]]; then
      continue
    fi
    return 0
  done < <(pgrep -f "$pattern" 2>/dev/null || true)
  return 1
}

for pattern in "${KILL_PATTERNS[@]}"; do
  kill_matching_processes TERM "$pattern"
done
sleep 1

# Escalate to SIGKILL if anything still survives.
for pattern in "${KILL_PATTERNS[@]}"; do
  if has_matching_processes "$pattern"; then
    echo "[go2.sh] Force killing leftover: $pattern"
    kill_matching_processes KILL "$pattern"
  fi
done

# Verify no leftovers remain; retry a few rounds for stubborn children.
for attempt in 1 2 3; do
  leftovers=0
  for pattern in "${KILL_PATTERNS[@]}"; do
    if has_matching_processes "$pattern"; then
      leftovers=1
      echo "[go2.sh] Cleanup retry $attempt for: $pattern"
      kill_matching_processes KILL "$pattern"
    fi
  done
  [[ "$leftovers" -eq 0 ]] && break
  sleep 1
done

# ---------------------------------------------------------------------------
# Dispatch to the matching ROS 2 launch file.
# ---------------------------------------------------------------------------
case "$MODE" in
  mapping)
    echo "[go2.sh] MODE = mapping (new map)"
    echo "[go2.sh]   database_path = $DB_PATH  (will be overwritten)"
    LAUNCH=go2_mapping.launch.py
    ;;
  localization)
    echo "[go2.sh] MODE = localization (load existing map)"
    echo "[go2.sh]   database_path = $DB_PATH  (read-only)"
    LAUNCH=go2_navigation.launch.py
    ;;
  exploration)
    echo "[go2.sh] MODE = exploration (autonomous mapping)"
    echo "[go2.sh]   database_path = $DB_PATH  (will be overwritten)"
    LAUNCH=go2_exploration.launch.py
    ;;
esac
echo

exec ros2 launch go2_nav "$LAUNCH" "database_path:=$DB_PATH"
