#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SDK_PATH="${NUWAX_GO2_SDK_PATH:-${GO2_SDK_PATH:-}}"

[[ -x /usr/bin/python3 ]] || {
  echo "motion guard requires executable /usr/bin/python3" >&2
  exit 1
}
[[ -x "${REPO_ROOT}" ]] || {
  echo "motion guard service UID cannot traverse repository: ${REPO_ROOT}" >&2
  exit 1
}
[[ -r "${REPO_ROOT}/services/navigation/motion_safety_guard/__main__.py" ]] || {
  echo "motion guard service UID cannot read production entrypoint" >&2
  exit 1
}
[[ -n "${SDK_PATH}" && -d "${SDK_PATH}" && -x "${SDK_PATH}" ]] || {
  echo "GO2_SDK_PATH/NUWAX_GO2_SDK_PATH is missing or inaccessible: ${SDK_PATH:-<unset>}" >&2
  exit 1
}

export PYTHONPATH="${REPO_ROOT}:${SDK_PATH}${PYTHONPATH:+:${PYTHONPATH}}"
export PYTHONNOUSERSITE=1
exec /usr/bin/python3 -c '
import google.protobuf
import unitree_sdk2py.core.channel
import unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client
import unitree_sdk2py.go2.sport.sport_client
import drivers.go2.control.low_level_controller
import services.navigation.motion_safety_guard
'
