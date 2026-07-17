#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="${NUWAX_REPO_ROOT:?NUWAX_REPO_ROOT is required}"
REQUEST_FILE="${NUWAX_RUNTIME_ACTIVATION_REQUEST:-/run/nuwax/gateway/geometry.request}"
RESULT_FILE="${NUWAX_RUNTIME_ACTIVATION_RESULT:-/run/nuwax/gateway/geometry.result.json}"
CANCEL_FILE="${NUWAX_RUNTIME_ACTIVATION_CANCEL:-${REQUEST_FILE}.cancel.json}"
ACK_FILE="${NUWAX_RUNTIME_ACTIVATION_ACK:-${REQUEST_FILE}.ack.json}"
MOTION_SOCKET="${NUWAX_MOTION_GUARD_SOCKET:-/run/nuwax/motion-guard.sock}"
NAV_READY_FILE="${NUWAX_NAV_STACK_READY_FILE:-/run/nuwax/navigation/nuwax_nav_stack.ready}"
GEOMETRY_READY_FILE="${NUWAX_GEOMETRY_RUNTIME_READY_FILE:-/run/nuwax/geometry/nuwax_geometry_runtime.ready}"
PERSON_FOLLOW_READY_FILE="${NUWAX_PERSON_FOLLOW_READY_FILE:-/run/nuwax/person-follow/nuwax_person_follow.ready}"
GEOMETRY_PORT="${NUWAX_GEOMETRY_RUNTIME_PORT:-50051}"
LAST_STAGE="starting"
LAST_UNIT=""
ACTIVATION_LEVEL="motion"
CAPABILITY_NAME="unknown"
REQUEST_ID=""
SAFETY_EPOCH="0"
STARTED_UNITS=()
FAILURE_UNIT=""
FAILURE_UNIT_RESULT=""
FAILURE_UNIT_EXIT_STATUS=""
FAILURE_DIAGNOSTIC=""

write_result() {
  local state="$1"
  local message="$2"
  local error_code="${3:-}"
  local temporary="${RESULT_FILE}.$$"
  STATE="${state}" MESSAGE="${message}" STAGE="${LAST_STAGE}" \
    ERROR_CODE="${error_code}" REQUEST_ID="${REQUEST_ID}" \
    SAFETY_EPOCH="${SAFETY_EPOCH}" ACTIVATION_LEVEL="${ACTIVATION_LEVEL}" \
    CAPABILITY_NAME="${CAPABILITY_NAME}" FAILURE_UNIT="${FAILURE_UNIT}" \
    FAILURE_UNIT_RESULT="${FAILURE_UNIT_RESULT}" \
    FAILURE_UNIT_EXIT_STATUS="${FAILURE_UNIT_EXIT_STATUS}" \
    FAILURE_DIAGNOSTIC="${FAILURE_DIAGNOSTIC}" \
    /usr/bin/python3 -c 'import json, os; payload = {"state": os.environ["STATE"], "stage": os.environ["STAGE"], "message": os.environ["MESSAGE"], "request_id": os.environ["REQUEST_ID"], "safety_epoch": int(os.environ["SAFETY_EPOCH"]), "activation_level": os.environ["ACTIVATION_LEVEL"], "capability_name": os.environ["CAPABILITY_NAME"]}; optional = {"error_code": os.environ.get("ERROR_CODE", ""), "unit": os.environ.get("FAILURE_UNIT", ""), "unit_result": os.environ.get("FAILURE_UNIT_RESULT", ""), "unit_exit_status": os.environ.get("FAILURE_UNIT_EXIT_STATUS", ""), "diagnostic": os.environ.get("FAILURE_DIAGNOSTIC", "")}; payload.update({key: value for key, value in optional.items() if value}); print(json.dumps(payload, ensure_ascii=True, sort_keys=True))' \
    > "${temporary}"
  chmod 0644 "${temporary}"
  mv -f "${temporary}" "${RESULT_FILE}"
}

rollback_started_units() {
  local index=0
  local unit=""
  for ((index = ${#STARTED_UNITS[@]} - 1; index >= 0; index--)); do
    unit="${STARTED_UNITS[index]}"
    systemctl stop "${unit}" >/dev/null 2>&1 || true
  done
  STARTED_UNITS=()
}

cleanup_protocol_files() {
  rm -f "${REQUEST_FILE}" "${ACK_FILE}" "${CANCEL_FILE}"
}

capture_unit_failure() {
  local status_detail=""
  local journal_detail=""
  [[ -n "${LAST_UNIT}" ]] || return 0
  FAILURE_UNIT="${LAST_UNIT}"
  FAILURE_UNIT_RESULT="$(systemctl show --property=Result --value "${LAST_UNIT}" 2>/dev/null || true)"
  FAILURE_UNIT_EXIT_STATUS="$(systemctl show --property=ExecMainStatus --value "${LAST_UNIT}" 2>/dev/null || true)"
  status_detail="$(systemctl status --no-pager --full --lines=20 "${LAST_UNIT}" 2>&1 || true)"
  journal_detail="$(journalctl --no-pager --quiet --lines=20 --unit "${LAST_UNIT}" 2>/dev/null || true)"
  FAILURE_DIAGNOSTIC="${status_detail}"
  if [[ -n "${journal_detail}" ]]; then
    FAILURE_DIAGNOSTIC+=$'\n'"${journal_detail}"
  fi
  FAILURE_DIAGNOSTIC="${FAILURE_DIAGNOSTIC//$'\n'/ | }"
  FAILURE_DIAGNOSTIC="${FAILURE_DIAGNOSTIC:0:4000}"
}

fail_activation() {
  local exit_code=$?
  trap - ERR INT TERM HUP
  local message="activation failed at ${LAST_STAGE} (exit=${exit_code})"
  capture_unit_failure
  if [[ -n "${FAILURE_DIAGNOSTIC}" ]]; then
    message+=": ${FAILURE_DIAGNOSTIC}"
  fi
  rollback_started_units
  write_result failed "${message}" runtime_activation_failed || true
  cleanup_protocol_files
  exit "${exit_code}"
}

fail_activation_with_message() {
  local error_code="$1"
  local message="$2"
  trap - ERR INT TERM HUP
  rollback_started_units
  write_result failed "${message}" "${error_code}" || true
  cleanup_protocol_files
  exit 1
}

cancel_activation() {
  local message="$1"
  trap - ERR INT TERM HUP
  rollback_started_units
  write_result cancelled "${message}" safe_stop_active || true
  cleanup_protocol_files
  exit 0
}

read_request() {
  IFS=$'\t' read -r ACTIVATION_LEVEL CAPABILITY_NAME REQUEST_ID SAFETY_EPOCH < <(
    /usr/bin/python3 - "${REQUEST_FILE}" <<'PY'
import json
import pathlib
import sys

payload = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
level = str(payload.get("activation_level") or "motion").strip().lower()
if level not in {"guard", "geometry", "motion", "person_follow"}:
    raise SystemExit("invalid activation_level: %s" % level)
capability = str(payload.get("capability_name") or "unknown").strip() or "unknown"
request_id = str(payload.get("request_id") or "").strip()
if not request_id:
    raise SystemExit("missing request_id")
try:
    safety_epoch = int(payload.get("safety_epoch", 0))
except (TypeError, ValueError):
    raise SystemExit("invalid safety_epoch")
if safety_epoch < 0:
    raise SystemExit("invalid safety_epoch")
print("%s\t%s\t%s\t%s" % (level, capability, request_id, safety_epoch))
PY
  )
}

marker_matches_request() {
  local marker_file="$1"
  local marker_kind="$2"
  [[ -f "${marker_file}" ]] || return 1
  /usr/bin/python3 - "${marker_file}" "${REQUEST_ID}" "${SAFETY_EPOCH}" "${marker_kind}" <<'PY'
import json
import pathlib
import sys

payload = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
if str(payload.get("request_id") or "").strip() != sys.argv[2]:
    raise SystemExit(1)
try:
    marker_epoch = int(payload.get("safety_epoch", -1))
    request_epoch = int(sys.argv[3])
except (TypeError, ValueError):
    raise SystemExit(1)
kind = sys.argv[4]
if kind == "cancel":
    raise SystemExit(0 if marker_epoch >= request_epoch else 1)
if str(payload.get("state") or "").strip().lower() != "acknowledged":
    raise SystemExit(1)
raise SystemExit(0 if marker_epoch == request_epoch else 1)
PY
}

activation_cancelled() {
  marker_matches_request "${CANCEL_FILE}" cancel
}

activation_acknowledged() {
  marker_matches_request "${ACK_FILE}" ack
}

check_cancel() {
  if activation_cancelled; then
    cancel_activation "safe stop cancelled runtime activation at ${LAST_STAGE}"
  fi
}

wait_for_ack() {
  local timeout_sec="${NUWAX_RUNTIME_ACTIVATION_ACK_TIMEOUT_SEC:-10}"
  local attempts=0
  attempts="$(/usr/bin/python3 - "${timeout_sec}" <<'PY'
import math
import sys

try:
    value = max(0.1, float(sys.argv[1]))
except (TypeError, ValueError):
    value = 10.0
print(max(1, int(math.ceil(value * 10))))
PY
)"
  for ((attempt = 0; attempt < attempts; attempt++)); do
    check_cancel
    if activation_acknowledged; then
      return 0
    fi
    sleep 0.1
  done
  return 1
}

wait_for_gateway_commit() {
  local timeout_sec="${NUWAX_RUNTIME_ACTIVATION_COMMIT_TIMEOUT_SEC:-10}"
  local attempts=0
  attempts="$(/usr/bin/python3 - "${timeout_sec}" <<'PY'
import math
import sys

try:
    value = max(0.1, float(sys.argv[1]))
except (TypeError, ValueError):
    value = 10.0
print(max(1, int(math.ceil(value * 10))))
PY
)"
  for ((attempt = 0; attempt < attempts; attempt++)); do
    check_cancel
    if [[ ! -e "${REQUEST_FILE}" ]]; then
      return 0
    fi
    sleep 0.1
  done
  return 1
}

start_unit_if_needed() {
  local unit="$1"
  check_cancel
  if systemctl is-active --quiet "${unit}"; then
    return 0
  fi
  STARTED_UNITS+=("${unit}")
  systemctl start "${unit}"
  check_cancel
}

trap fail_activation ERR
trap 'cancel_activation "runtime activation interrupted"' INT TERM HUP
mkdir -p "$(dirname "${RESULT_FILE}")"
LAST_STAGE="request"
read_request
check_cancel

if [[ "${ACTIVATION_LEVEL}" == "guard" \
  || "${ACTIVATION_LEVEL}" == "motion" \
  || "${ACTIVATION_LEVEL}" == "person_follow" ]]; then
  write_result activating "starting motion guard component"
  LAST_STAGE="motion_guard"
  LAST_UNIT="nuwax-motion-safety-guard.service"
  start_unit_if_needed nuwax-motion-safety-guard.service
  NUWAX_MOTION_GUARD_SOCKET="${MOTION_SOCKET}" \
    NUWAX_MOTION_GUARD_START_TIMEOUT_SEC=15 \
    bash "${REPO_ROOT}/scripts/systemd/wait_for_motion_guard_socket.sh" wait
  check_cancel
fi

if [[ "${ACTIVATION_LEVEL}" == "motion" \
  || "${ACTIVATION_LEVEL}" == "person_follow" ]]; then
  write_result activating "starting navigation component"
  LAST_STAGE="navigation"
  LAST_UNIT="nuwax-nav-stack.service"
  start_unit_if_needed nuwax-nav-stack.service
  NUWAX_NAV_STACK_READY_FILE="${NAV_READY_FILE}" \
    NUWAX_NAV_STACK_START_TIMEOUT_SEC=15 \
    bash "${REPO_ROOT}/scripts/systemd/wait_for_nav_stack_ready.sh" wait
  check_cancel
fi

if [[ "${ACTIVATION_LEVEL}" == "geometry" \
  || "${ACTIVATION_LEVEL}" == "motion" \
  || "${ACTIVATION_LEVEL}" == "person_follow" ]]; then
  write_result activating "starting geometry component"
  LAST_STAGE="geometry"
  LAST_UNIT="nuwax-geometry-runtime.service"
  start_unit_if_needed nuwax-geometry-runtime.service
  NUWAX_GEOMETRY_RUNTIME_READY_FILE="${GEOMETRY_READY_FILE}" \
    NUWAX_GEOMETRY_RUNTIME_PORT="${GEOMETRY_PORT}" \
    NUWAX_GEOMETRY_RUNTIME_HEALTH_TIMEOUT=30 \
    bash "${REPO_ROOT}/scripts/systemd/wait_for_geometry_runtime_ready.sh" wait
  check_cancel
fi

if [[ "${ACTIVATION_LEVEL}" == "person_follow" ]]; then
  write_result activating "starting person-follow component"
  LAST_STAGE="person_follow"
  LAST_UNIT="nuwax-person-follow.service"
  start_unit_if_needed nuwax-person-follow.service
  NUWAX_PERSON_FOLLOW_READY_FILE="${PERSON_FOLLOW_READY_FILE}" \
    NUWAX_PERSON_FOLLOW_START_TIMEOUT_SEC=15 \
    bash "${REPO_ROOT}/scripts/systemd/wait_for_person_follow_ready.sh" wait
  check_cancel
fi

LAST_STAGE="prepared"
write_result prepared "requested runtime components are prepared; waiting for Gateway ACK"
if ! wait_for_ack; then
  fail_activation_with_message \
    runtime_activation_ack_timeout \
    "Gateway ACK timeout; rolling back requested runtime components"
fi
check_cancel

LAST_STAGE="ready"
write_result ready "requested runtime components are ready; waiting for Gateway commit"
if ! wait_for_gateway_commit; then
  fail_activation_with_message \
    runtime_activation_commit_timeout \
    "Gateway commit timeout; rolling back requested runtime components"
fi

trap - ERR INT TERM HUP
rm -f "${ACK_FILE}" "${CANCEL_FILE}"
