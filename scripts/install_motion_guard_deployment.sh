#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PRODUCTION_REPO_ROOT="/home/unitree/nuwax-robot-bridge"
ACTION="${1:-check-sources}"
ACTUATOR_SYMBOLS_FILE="${2:-${NUWAX_ACTUATOR_SYMBOLS_FILE:-}}"

SYSUSERS_SOURCE="${REPO_ROOT}/deploy/sysusers.d/nuwax-motion.conf"
TMPFILES_SOURCE="${REPO_ROOT}/deploy/tmpfiles.d/nuwax-motion.conf"
GUARD_UNIT_SOURCE="${REPO_ROOT}/deploy/systemd/nuwax-motion-safety-guard.service"
NAV_UNIT_SOURCE="${REPO_ROOT}/deploy/systemd/nuwax-nav-stack.service"
GEOMETRY_UNIT_SOURCE="${REPO_ROOT}/deploy/systemd/nuwax-geometry-runtime.service"
PERSON_FOLLOW_UNIT_SOURCE="${REPO_ROOT}/deploy/systemd/nuwax-person-follow.service"
GATEWAY_UNIT_SOURCE="${REPO_ROOT}/deploy/systemd/nuwax-host-gateway.service"
GUARD_MODULE="${REPO_ROOT}/services/navigation/motion_safety_guard/__main__.py"
PROVISIONER="${REPO_ROOT}/scripts/provision_motion_guard.py"
NAV_BINARY="${REPO_ROOT}/services/nuwax-workspace/target/release/nuwax-nav-stack"
GEOMETRY_BINARY="${REPO_ROOT}/services/geometry/nuwax_geometry_runtime/target/release/nuwax-geometry-runtime"
PERSON_FOLLOW_BINARY="${REPO_ROOT}/services/nuwax-workspace/target/release/nuwax-person-follow"
GUARD_SOCKET="${NUWAX_MOTION_GUARD_SOCKET:-/run/nuwax/motion-guard.sock}"
GUARD_LOCK="${NUWAX_MOTION_GUARD_LOCK:-/run/nuwax/motion-guard.lock}"
GUARD_STATE_DIR="${NUWAX_MOTION_GUARD_STATE_DIR:-/var/lib/nuwax/motion-guard}"
SDK_PATH="${NUWAX_GO2_SDK_PATH:-${GO2_SDK_PATH:-/home/unitree/unitree_sdk2_python}}"
INSTALLED_ACTUATOR_SYMBOLS_FILE="/usr/share/nuwax/actuator-symbols.json"

fail() {
  echo "ERROR: $*" >&2
  exit 1
}

require_root() {
  [[ "${EUID}" -eq 0 ]] || fail "${ACTION} requires root"
}

check_file() {
  [[ -f "$1" ]] || fail "missing deployment artifact: $1"
}

check_sources() {
  check_file "${SYSUSERS_SOURCE}"
  check_file "${TMPFILES_SOURCE}"
  check_file "${GUARD_UNIT_SOURCE}"
  check_file "${NAV_UNIT_SOURCE}"
  check_file "${GEOMETRY_UNIT_SOURCE}"
  check_file "${PERSON_FOLLOW_UNIT_SOURCE}"
  check_file "${GATEWAY_UNIT_SOURCE}"
  check_file "${GUARD_MODULE}"
  check_file "${PROVISIONER}"
  [[ -x "${NAV_BINARY}" ]] || fail "navigation stack is not executable: ${NAV_BINARY}; run scripts/build_nav_stack.sh"
  [[ -x "${GEOMETRY_BINARY}" ]] || fail "geometry runtime is not executable: ${GEOMETRY_BINARY}"
  [[ -x "${PERSON_FOLLOW_BINARY}" ]] || fail "person-follow runtime is not executable: ${PERSON_FOLLOW_BINARY}; build the nuwax-person-follow release binary"
  systemd-sysusers --dry-run "${SYSUSERS_SOURCE}" >/dev/null
  bash -n \
    "${REPO_ROOT}/scripts/systemd/check_motion_guard_python.sh" \
    "${REPO_ROOT}/scripts/systemd/start_nav_stack.sh" \
    "${REPO_ROOT}/scripts/systemd/wait_for_nav_stack_ready.sh" \
    "${REPO_ROOT}/scripts/systemd/wait_for_motion_guard_socket.sh" \
    "${REPO_ROOT}/scripts/build_nav_stack.sh" \
    "${REPO_ROOT}/scripts/start_gateway.sh"
  echo "motion guard deployment sources: PASS"
}

require_actuator_manifest() {
  [[ -n "${ACTUATOR_SYMBOLS_FILE}" ]] \
    || fail "set NUWAX_ACTUATOR_SYMBOLS_FILE or pass an actuator manifest path"
  [[ -s "${ACTUATOR_SYMBOLS_FILE}" ]] \
    || fail "actuator symbols manifest is missing or empty: ${ACTUATOR_SYMBOLS_FILE}"
}

check_installed_actuator_manifest() {
  [[ -f "${INSTALLED_ACTUATOR_SYMBOLS_FILE}" \
    && ! -L "${INSTALLED_ACTUATOR_SYMBOLS_FILE}" \
    && -s "${INSTALLED_ACTUATOR_SYMBOLS_FILE}" ]] \
    || fail "installed actuator symbols manifest is missing, empty, non-regular, or a symlink"
  [[ "$(stat -c '%U:%G %a' "${INSTALLED_ACTUATOR_SYMBOLS_FILE}")" == "root:root 444" ]] \
    || fail "installed actuator symbols manifest must be root:root 0444"
}

install_actuator_manifest() {
  require_actuator_manifest
  [[ ! -L "${INSTALLED_ACTUATOR_SYMBOLS_FILE}" ]] \
    || fail "refusing actuator symbols manifest symlink: ${INSTALLED_ACTUATOR_SYMBOLS_FILE}"
  if [[ -e "${INSTALLED_ACTUATOR_SYMBOLS_FILE}" ]]; then
    cmp -s "${ACTUATOR_SYMBOLS_FILE}" "${INSTALLED_ACTUATOR_SYMBOLS_FILE}" \
      || fail "refusing to overwrite installed actuator symbols manifest with different content"
  else
    install -D -o root -g root -m 0444 \
      "${ACTUATOR_SYMBOLS_FILE}" "${INSTALLED_ACTUATOR_SYMBOLS_FILE}"
  fi
  check_installed_actuator_manifest
}

check_service_access() {
  runuser -u nuwax-motion-guard -- test -x "${REPO_ROOT}" \
    || fail "nuwax-motion-guard cannot traverse ${REPO_ROOT}"
  runuser -u nuwax-motion-guard -- test -r "${GUARD_MODULE}" \
    || fail "nuwax-motion-guard cannot read ${GUARD_MODULE}"
  runuser -u nuwax-motion-guard -- env \
    GO2_SDK_PATH="${SDK_PATH}" \
    PYTHONPATH="${REPO_ROOT}:${SDK_PATH}" \
    bash "${REPO_ROOT}/scripts/systemd/check_motion_guard_python.sh" \
    || fail "nuwax-motion-guard cannot import protobuf/SDK/production module"
  runuser -u nuwax-navigation -- test -x "${NAV_BINARY}" \
    || fail "nuwax-navigation cannot traverse/read/execute ${NAV_BINARY}"
  runuser -u nuwax-navigation -- test -r "${REPO_ROOT}/scripts/systemd/start_nav_stack.sh" \
    || fail "nuwax-navigation cannot read start_nav_stack.sh"
  runuser -u nuwax-navigation -- test -x "${PERSON_FOLLOW_BINARY}" \
    || fail "nuwax-navigation cannot traverse/read/execute ${PERSON_FOLLOW_BINARY}"
  runuser -u nuwax-geometry -- test -x "${GEOMETRY_BINARY}" \
    || fail "nuwax-geometry cannot traverse/read/execute ${GEOMETRY_BINARY}"
  runuser -u nuwax-gateway -- test -x "${REPO_ROOT}" \
    || fail "nuwax-gateway cannot traverse ${REPO_ROOT}"
  runuser -u nuwax-gateway -- test -r "${REPO_ROOT}/scripts/start_gateway.sh" \
    || fail "nuwax-gateway cannot read start_gateway.sh"
}

check_record() {
  local path="$1"
  local expected_mode="$2"
  local actual_mode=""
  [[ -s "${path}" ]] || fail "missing or empty durable record: ${path}"
  actual_mode="$(stat -c '%a' "${path}")"
  [[ "${actual_mode}" == "${expected_mode}" ]] \
    || fail "invalid mode for ${path}: ${actual_mode}, expected ${expected_mode}"
  [[ "$(stat -c '%U:%G' "${path}")" == "nuwax-motion-guard:nuwax-motion-guard" ]] \
    || fail "invalid owner for ${path}"
}

verify_installed() {
  getent passwd nuwax-motion-guard >/dev/null || fail "missing user nuwax-motion-guard"
  getent passwd nuwax-motion-admin >/dev/null || fail "missing user nuwax-motion-admin"
  getent passwd nuwax-navigation >/dev/null || fail "missing user nuwax-navigation"
  getent passwd nuwax-geometry >/dev/null || fail "missing user nuwax-geometry"
  getent passwd nuwax-gateway >/dev/null || fail "missing user nuwax-gateway"
  getent group nuwax-motion-clients >/dev/null || fail "missing group nuwax-motion-clients"
  check_installed_actuator_manifest
  "${PROVISIONER}" verify \
    --state-dir "${GUARD_STATE_DIR}" \
    --actuator-symbols-file "${INSTALLED_ACTUATOR_SYMBOLS_FILE}"
  check_record "${GUARD_STATE_DIR}/provisioned.json" 400
  check_record "${GUARD_STATE_DIR}/epoch.json" 600
  [[ -S "${GUARD_SOCKET}" ]] || fail "motion guard socket is absent: ${GUARD_SOCKET}"
  [[ -f "${GUARD_LOCK}" && ! -L "${GUARD_LOCK}" ]] \
    || fail "motion guard lock is absent, non-regular, or a symlink: ${GUARD_LOCK}"
  [[ "$(stat -c '%U:%G %a' "${GUARD_LOCK}")" == "nuwax-motion-guard:nuwax-motion-guard 600" ]] \
    || fail "motion guard lock owner/mode drift: ${GUARD_LOCK}"
  NUWAX_MOTION_GUARD_SOCKET="${GUARD_SOCKET}" \
    bash "${REPO_ROOT}/scripts/systemd/wait_for_motion_guard_socket.sh" check
  systemctl is-active --quiet nuwax-motion-safety-guard.service \
    || fail "nuwax-motion-safety-guard.service is not active"
  systemctl is-active --quiet nuwax-nav-stack.service \
    || fail "nuwax-nav-stack.service is not active"
  bash "${REPO_ROOT}/scripts/systemd/wait_for_nav_stack_ready.sh" check \
    || fail "nuwax-nav-stack ready state is absent or stale"
  systemctl is-active --quiet nuwax-geometry-runtime.service \
    || fail "nuwax-geometry-runtime.service is not active"
  systemctl is-active --quiet nuwax-person-follow.service \
    || fail "nuwax-person-follow.service is not active"
  systemctl is-active --quiet nuwax-host-gateway.service \
    || fail "nuwax-host-gateway.service is not active"
  echo "motion guard deployment runtime: PASS"
}

install_sources() {
  require_root
  [[ "${REPO_ROOT}" == "${PRODUCTION_REPO_ROOT}" ]] \
    || fail "production units require repository at ${PRODUCTION_REPO_ROOT}, got ${REPO_ROOT}"
  check_sources
  install -D -m 0644 "${SYSUSERS_SOURCE}" /usr/lib/sysusers.d/nuwax-motion.conf
  install -D -m 0644 "${TMPFILES_SOURCE}" /usr/lib/tmpfiles.d/nuwax-motion.conf
  systemd-sysusers /usr/lib/sysusers.d/nuwax-motion.conf
  systemd-tmpfiles --create /usr/lib/tmpfiles.d/nuwax-motion.conf
  check_service_access
  install -D -m 0644 "${GUARD_UNIT_SOURCE}" /etc/systemd/system/nuwax-motion-safety-guard.service
  install -D -m 0644 "${NAV_UNIT_SOURCE}" /etc/systemd/system/nuwax-nav-stack.service
  install -D -m 0644 "${GEOMETRY_UNIT_SOURCE}" /etc/systemd/system/nuwax-geometry-runtime.service
  install -D -m 0644 "${PERSON_FOLLOW_UNIT_SOURCE}" /etc/systemd/system/nuwax-person-follow.service
  install -D -m 0644 "${GATEWAY_UNIT_SOURCE}" /etc/systemd/system/nuwax-host-gateway.service
  systemctl daemon-reload
  echo "deployment files installed; provision durable records before enabling the stack"
}

enable_stack() {
  require_root
  check_installed_actuator_manifest
  "${PROVISIONER}" verify \
    --state-dir "${GUARD_STATE_DIR}" \
    --actuator-symbols-file "${INSTALLED_ACTUATOR_SYMBOLS_FILE}"
  systemctl enable nuwax-motion-safety-guard.service
  systemctl enable nuwax-nav-stack.service
  systemctl enable nuwax-geometry-runtime.service
  systemctl enable nuwax-person-follow.service
  systemctl enable nuwax-host-gateway.service
  echo "enabled startup order: guard -> nav-stack -> geometry runtime -> person-follow -> Gateway"
}

provision_records() {
  require_root
  install_actuator_manifest
  "${PROVISIONER}" provision \
    --state-dir "${GUARD_STATE_DIR}" \
    --actuator-symbols-file "${INSTALLED_ACTUATOR_SYMBOLS_FILE}"
}

case "${ACTION}" in
  check-sources)
    check_sources
    ;;
  install)
    install_sources
    ;;
  provision)
    provision_records
    ;;
  enable)
    enable_stack
    ;;
  verify-runtime)
    verify_installed
    ;;
  *)
    echo "usage: $0 [check-sources|install|provision|enable|verify-runtime] [actuator-symbols.json]" >&2
    exit 2
    ;;
esac
