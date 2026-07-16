#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
REPO_ROOT="${NUWAX_REPO_ROOT_OVERRIDE:-${DEFAULT_REPO_ROOT}}"
SDK_PATH="${NUWAX_GO2_SDK_PATH:-${GO2_SDK_PATH:-}}"
PYTHON_BIN="${NUWAX_PYTHON_BIN:-${REPO_ROOT}/.venv/bin/python}"

if [[ -z "${NUWAX_PYTHON_BIN:-}" && ! -x "${PYTHON_BIN}" ]]; then
  PYTHON_BIN=/usr/bin/python3
fi

[[ -x "${PYTHON_BIN}" ]] || {
  echo "motion guard requires executable Python interpreter: ${PYTHON_BIN}" >&2
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
exec "${PYTHON_BIN}" -c '
import google.protobuf
import unitree_sdk2py.core.channel
import unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client
import unitree_sdk2py.go2.sport.sport_client
import drivers.go2.control.low_level_controller
import services.navigation.motion_safety_guard
'
