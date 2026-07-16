#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="${NUWAX_REPO_ROOT:?NUWAX_REPO_ROOT is required}"
REQUEST_FILE="${NUWAX_RUNTIME_ACTIVATION_REQUEST:-/run/nuwax/gateway/geometry.request}"
RESULT_FILE="${NUWAX_RUNTIME_ACTIVATION_RESULT:-/run/nuwax/gateway/geometry.result.json}"
MOTION_SOCKET="${NUWAX_MOTION_GUARD_SOCKET:-/run/nuwax/motion-guard.sock}"
NAV_READY_FILE="${NUWAX_NAV_STACK_READY_FILE:-/run/nuwax/navigation/nuwax_nav_stack.ready}"
GEOMETRY_READY_FILE="${NUWAX_GEOMETRY_RUNTIME_READY_FILE:-/run/nuwax/geometry/nuwax_geometry_runtime.ready}"
PERSON_FOLLOW_READY_FILE="${NUWAX_PERSON_FOLLOW_READY_FILE:-/run/nuwax/person-follow/nuwax_person_follow.ready}"
GEOMETRY_PORT="${NUWAX_GEOMETRY_RUNTIME_PORT:-50051}"
LAST_STAGE="starting"
ACTIVATION_LEVEL="motion"
CAPABILITY_NAME="unknown"

write_result() {
  local state="$1"
  local message="$2"
  local temporary="${RESULT_FILE}.$$"
  STATE="${state}" MESSAGE="${message}" STAGE="${LAST_STAGE}" \
    ACTIVATION_LEVEL="${ACTIVATION_LEVEL}" CAPABILITY_NAME="${CAPABILITY_NAME}" \
    /usr/bin/python3 -c 'import json, os; print(json.dumps({"state": os.environ["STATE"], "stage": os.environ["STAGE"], "message": os.environ["MESSAGE"], "activation_level": os.environ["ACTIVATION_LEVEL"], "capability_name": os.environ["CAPABILITY_NAME"]}, ensure_ascii=True, sort_keys=True))' \
    > "${temporary}"
  chmod 0644 "${temporary}"
  mv -f "${temporary}" "${RESULT_FILE}"
}

fail_activation() {
  local exit_code=$?
  local message="activation failed at ${LAST_STAGE} (exit=${exit_code})"
  write_result failed "${message}" || true
  rm -f "${REQUEST_FILE}"
  exit "${exit_code}"
}

trap fail_activation ERR
mkdir -p "$(dirname "${RESULT_FILE}")"
LAST_STAGE="request"
IFS=$'\t' read -r ACTIVATION_LEVEL CAPABILITY_NAME < <(
  /usr/bin/python3 - "${REQUEST_FILE}" <<'PY'
import json
import pathlib
import sys

payload = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
level = str(payload.get("activation_level") or "motion").strip().lower()
if level not in {"guard", "geometry", "motion", "person_follow"}:
    raise SystemExit("invalid activation_level: %s" % level)
capability = str(payload.get("capability_name") or "unknown").strip() or "unknown"
print("%s\t%s" % (level, capability))
PY
)

if [[ "${ACTIVATION_LEVEL}" == "guard" \
  || "${ACTIVATION_LEVEL}" == "motion" \
  || "${ACTIVATION_LEVEL}" == "person_follow" ]]; then
  write_result activating "starting motion guard component"
  LAST_STAGE="motion_guard"
  systemctl start nuwax-motion-safety-guard.service
  NUWAX_MOTION_GUARD_SOCKET="${MOTION_SOCKET}" \
    NUWAX_MOTION_GUARD_START_TIMEOUT_SEC=15 \
    bash "${REPO_ROOT}/scripts/systemd/wait_for_motion_guard_socket.sh" wait

fi

if [[ "${ACTIVATION_LEVEL}" == "motion" \
  || "${ACTIVATION_LEVEL}" == "person_follow" ]]; then
  write_result activating "starting navigation component"
  LAST_STAGE="navigation"
  systemctl start nuwax-nav-stack.service
  NUWAX_NAV_STACK_READY_FILE="${NAV_READY_FILE}" \
    NUWAX_NAV_STACK_START_TIMEOUT_SEC=15 \
    bash "${REPO_ROOT}/scripts/systemd/wait_for_nav_stack_ready.sh" wait
fi

if [[ "${ACTIVATION_LEVEL}" == "geometry" \
  || "${ACTIVATION_LEVEL}" == "motion" \
  || "${ACTIVATION_LEVEL}" == "person_follow" ]]; then
  write_result activating "starting geometry component"
  LAST_STAGE="geometry"
  systemctl start nuwax-geometry-runtime.service
  NUWAX_GEOMETRY_RUNTIME_READY_FILE="${GEOMETRY_READY_FILE}" \
    NUWAX_GEOMETRY_RUNTIME_PORT="${GEOMETRY_PORT}" \
    NUWAX_GEOMETRY_RUNTIME_HEALTH_TIMEOUT=30 \
    bash "${REPO_ROOT}/scripts/systemd/wait_for_geometry_runtime_ready.sh" wait
fi

if [[ "${ACTIVATION_LEVEL}" == "person_follow" ]]; then
  write_result activating "starting person-follow component"
  LAST_STAGE="person_follow"
  systemctl start nuwax-person-follow.service
  NUWAX_PERSON_FOLLOW_READY_FILE="${PERSON_FOLLOW_READY_FILE}" \
    NUWAX_PERSON_FOLLOW_START_TIMEOUT_SEC=15 \
    bash "${REPO_ROOT}/scripts/systemd/wait_for_person_follow_ready.sh" wait
fi

LAST_STAGE="ready"
write_result ready "requested runtime components are ready"
rm -f "${REQUEST_FILE}"
