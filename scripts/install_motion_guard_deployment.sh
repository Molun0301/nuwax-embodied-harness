#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ACTION="${1:-check-sources}"
ACTUATOR_SYMBOLS_FILE="${2:-${NUWAX_ACTUATOR_SYMBOLS_FILE:-}}"

SYSUSERS_SOURCE="${NUWAX_MOTION_SYSUSERS_SOURCE:-${REPO_ROOT}/deploy/sysusers.d/nuwax-motion.conf}"
TMPFILES_SOURCE="${REPO_ROOT}/deploy/tmpfiles.d/nuwax-motion.conf"
GUARD_UNIT_SOURCE="${REPO_ROOT}/deploy/systemd/nuwax-motion-safety-guard.service"
NAV_UNIT_SOURCE="${REPO_ROOT}/deploy/systemd/nuwax-nav-stack.service"
GEOMETRY_UNIT_SOURCE="${REPO_ROOT}/deploy/systemd/nuwax-geometry-runtime.service"
PERSON_FOLLOW_UNIT_SOURCE="${REPO_ROOT}/deploy/systemd/nuwax-person-follow.service"
GATEWAY_UNIT_SOURCE="${REPO_ROOT}/deploy/systemd/nuwax-host-gateway.service"
GUARD_MODULE="${REPO_ROOT}/services/navigation/motion_safety_guard/__main__.py"
PROVISIONER="${REPO_ROOT}/scripts/provision_motion_guard.py"
LCM_PREFLIGHT="${REPO_ROOT}/scripts/systemd/check_lcm_multicast.sh"
PROCESS_IDENTITY_HELPER="${REPO_ROOT}/scripts/systemd/process_identity.sh"
GEOMETRY_LAUNCHER="${REPO_ROOT}/scripts/systemd/start_geometry_runtime.sh"
GEOMETRY_READY_HELPER="${REPO_ROOT}/scripts/systemd/wait_for_geometry_runtime_ready.sh"
PERSON_FOLLOW_LAUNCHER="${REPO_ROOT}/scripts/systemd/start_person_follow.sh"
PERSON_FOLLOW_READY_HELPER="${REPO_ROOT}/scripts/systemd/wait_for_person_follow_ready.sh"
NAV_BINARY="${REPO_ROOT}/services/nuwax-workspace/target/release/nuwax-nav-stack"
GEOMETRY_BINARY="${REPO_ROOT}/services/geometry/nuwax_geometry_runtime/target/release/nuwax-geometry-runtime"
PERSON_FOLLOW_BINARY="${REPO_ROOT}/services/nuwax-workspace/target/release/nuwax-person-follow"
GUARD_SOCKET="${NUWAX_MOTION_GUARD_SOCKET:-/run/nuwax/motion-guard.sock}"
GUARD_LOCK="${NUWAX_MOTION_GUARD_LOCK:-/run/nuwax/motion-guard.lock}"
GUARD_STATE_DIR="${NUWAX_MOTION_GUARD_STATE_DIR:-/var/lib/nuwax/motion-guard}"
SDK_PATH="${NUWAX_GO2_SDK_PATH:-${GO2_SDK_PATH:-/home/unitree/unitree_sdk2_python}}"
INSTALLED_ACTUATOR_SYMBOLS_FILE="/usr/share/nuwax/actuator-symbols.json"
PERSON_FOLLOW_AUTOSTART_MODE=""
SERVICE_IDENTITY_USERS=(
  nuwax-motion-guard
  nuwax-motion-admin
  nuwax-navigation
  nuwax-geometry
  nuwax-gateway
)

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

check_service_uid_collisions() {
  local source="${1:-system}"
  local passwd_file=""
  local rendered_root=""
  local user=""
  local uid=""
  local prior_user=""
  local passwd_entry=""
  declare -A user_by_uid=()
  declare -A requested_user_by_uid=()
  declare -A actual_user_by_uid=()

  if [[ "${source}" == "system" ]]; then
    passwd_file=""
  else
    while read -r record_type user uid _rest; do
      [[ "${record_type}" == "u" && "${uid}" =~ ^[0-9]+$ ]] || continue
      case " ${SERVICE_IDENTITY_USERS[*]} " in
        *" ${user} "*) ;;
        *) continue ;;
      esac
      prior_user="${requested_user_by_uid[${uid}]:-}"
      if [[ -n "${prior_user}" ]]; then
        fail "service identity UID collision in sysusers source: ${prior_user}, ${user} request UID ${uid}"
      fi
      requested_user_by_uid["${uid}"]="${user}"
    done < "${source}"
    rendered_root="$(mktemp -d)"
    mkdir -p "${rendered_root}/etc"
    if ! systemd-sysusers --root="${rendered_root}" "${source}" >/dev/null; then
      rm -rf "${rendered_root}"
      fail "cannot render sysusers source for UID collision validation: ${source}"
    fi
    passwd_file="${rendered_root}/etc/passwd"
  fi

  for user in "${SERVICE_IDENTITY_USERS[@]}"; do
    if [[ -n "${passwd_file}" ]]; then
      uid="$(awk -F: -v expected="${user}" '$1 == expected { print $3; found=1; exit } END { if (!found) exit 1 }' "${passwd_file}")" \
        || { rm -rf "${rendered_root}"; fail "sysusers source did not render ${user}"; }
    else
      uid="$(getent passwd "${user}" | awk -F: 'NR == 1 { print $3 }')"
      [[ -n "${uid}" ]] \
        || { rm -rf "${rendered_root}"; fail "missing service identity ${user}"; }
    fi
    [[ "${uid}" =~ ^[0-9]+$ ]] \
      || { rm -rf "${rendered_root}"; fail "invalid numeric UID for ${user}: ${uid}"; }
    prior_user="${user_by_uid[${uid}]:-}"
    if [[ -n "${prior_user}" ]]; then
      rm -rf "${rendered_root}"
      fail "service identity UID collision: ${prior_user}, ${user} share UID ${uid}"
    fi
    user_by_uid["${uid}"]="${user}"
  done
  rm -rf "${rendered_root}"

  if [[ "${source}" != "system" ]]; then
    for user in "${SERVICE_IDENTITY_USERS[@]}"; do
      passwd_entry="$(getent passwd "${user}" || true)"
      [[ -n "${passwd_entry}" ]] || continue
      uid="$(printf '%s\n' "${passwd_entry}" | awk -F: 'NR == 1 { print $3 }')"
      [[ "${uid}" =~ ^[0-9]+$ ]] || fail "invalid installed numeric UID for ${user}: ${uid}"
      prior_user="${requested_user_by_uid[${uid}]:-}"
      if [[ -n "${prior_user}" && "${prior_user}" != "${user}" ]]; then
        fail "service identity UID collision across sysusers/NSS: ${prior_user}, ${user} share UID ${uid}"
      fi
      prior_user="${actual_user_by_uid[${uid}]:-}"
      [[ -z "${prior_user}" ]] \
        || fail "installed service identity UID collision: ${prior_user}, ${user} share UID ${uid}"
      actual_user_by_uid["${uid}"]="${user}"
    done
  fi
}

resolve_person_follow_autostart() {
  local value=""
  local raw_line=""
  local key=""
  local candidate=""
  [[ -z "${PERSON_FOLLOW_AUTOSTART_MODE}" ]] || return 0

  if [[ -n "${NUWAX_PERSON_FOLLOW_AUTOSTART+x}" ]]; then
    value="${NUWAX_PERSON_FOLLOW_AUTOSTART}"
  elif [[ -f "${REPO_ROOT}/.env" ]]; then
    while IFS= read -r raw_line || [[ -n "${raw_line}" ]]; do
      candidate="${raw_line#"${raw_line%%[![:space:]]*}"}"
      [[ -n "${candidate}" && "${candidate}" != \#* && "${candidate}" == *=* ]] \
        || continue
      key="${candidate%%=*}"
      key="${key%"${key##*[![:space:]]}"}"
      if [[ "${key}" == "NUWAX_PERSON_FOLLOW_AUTOSTART" ]]; then
        value="${candidate#*=}"
      fi
    done < "${REPO_ROOT}/.env"
  fi
  value="${value:-true}"
  value="${value%%#*}"
  value="${value#"${value%%[![:space:]]*}"}"
  value="${value%"${value##*[![:space:]]}"}"
  if [[ ${#value} -ge 2 ]]; then
    if [[ "${value:0:1}" == '"' && "${value: -1}" == '"' ]] \
      || [[ "${value:0:1}" == "'" && "${value: -1}" == "'" ]]; then
      value="${value:1:${#value}-2}"
    fi
  fi
  value="$(printf '%s' "${value}" | tr '[:upper:]' '[:lower:]')"
  case "${value}" in
    1|true|yes|on)
      PERSON_FOLLOW_AUTOSTART_MODE="true"
      ;;
    0|false|no|off)
      PERSON_FOLLOW_AUTOSTART_MODE="false"
      ;;
    *)
      fail "invalid NUWAX_PERSON_FOLLOW_AUTOSTART=${value}"
      ;;
  esac
}

person_follow_autostart_enabled() {
  resolve_person_follow_autostart
  [[ "${PERSON_FOLLOW_AUTOSTART_MODE}" == "true" ]]
}

render_unit_to() {
  local source="$1"
  local destination="$2"
  local escaped_root=""
  escaped_root="$(printf '%s' "${REPO_ROOT}" | sed 's/[&|\\]/\\&/g')"
  sed "s|@NUWAX_REPO_ROOT@|${escaped_root}|g" "${source}" > "${destination}"
  ! grep -qF '@NUWAX_REPO_ROOT@' "${destination}" \
    || fail "unresolved repository placeholder in ${source}"
}

render_unit() {
  local source="$1"
  local destination="$2"
  local rendered=""
  rendered="$(mktemp)"
  render_unit_to "${source}" "${rendered}"
  install -D -m 0644 "${rendered}" "${destination}"
  rm -f "${rendered}"
}

check_unit_templates() {
  local rendered_dir=""
  local source=""
  local unit_path=""
  rendered_dir="$(mktemp -d)"
  for source in "${GUARD_UNIT_SOURCE}" "${NAV_UNIT_SOURCE}" "${GEOMETRY_UNIT_SOURCE}" \
                "${PERSON_FOLLOW_UNIT_SOURCE}" "${GATEWAY_UNIT_SOURCE}"; do
    render_unit_to "${source}" "${rendered_dir}/$(basename "${source}")"
  done
  # Unit syntax validation is independent from release binary availability;
  # install-time binary gates are conditional on person-follow autostart.
  unit_path="${rendered_dir}:/etc/systemd/system:/run/systemd/system:/usr/local/lib/systemd/system:/usr/lib/systemd/system:/lib/systemd/system"
  if ! SYSTEMD_UNIT_PATH="${unit_path}" systemd-analyze verify \
      "${rendered_dir}"/*.service; then
    rm -rf "${rendered_dir}"
    fail "rendered systemd unit verification failed"
  fi
  rm -rf "${rendered_dir}"
}

check_systemd_owned_invariants() {
  local unit=""
  local command=""
  for unit in "${GUARD_UNIT_SOURCE}" "${NAV_UNIT_SOURCE}" "${GEOMETRY_UNIT_SOURCE}" \
              "${PERSON_FOLLOW_UNIT_SOURCE}" "${GATEWAY_UNIT_SOURCE}"; do
    grep -q '^ExecStart=/usr/bin/env ' "${unit}" \
      || fail "ExecStart does not force systemd-owned invariants: ${unit}"
    while IFS= read -r command; do
      [[ "${command}" == ExecStartPre=/usr/bin/env\ * \
        || "${command}" == ExecStartPost=/usr/bin/env\ * ]] \
        || fail "systemd shell helper lacks invariant env prefix: ${unit}: ${command}"
    done < <(grep -E '^ExecStart(Pre|Post)=.*\.sh([[:space:]]|$)' "${unit}" || true)
  done

  grep -q 'ExecStart=/usr/bin/env .*NUWAX_NAV_STACK_READY_FILE=/run/nuwax/navigation/nuwax_nav_stack.ready' \
    "${NAV_UNIT_SOURCE}" \
    || fail "nav ExecStart does not own the production ready path"
  grep -qF 'NUWAX_NAV_STACK_BINARY=@NUWAX_REPO_ROOT@/services/nuwax-workspace/target/release/nuwax-nav-stack' \
    "${NAV_UNIT_SOURCE}" \
    || fail "nav ExecStart does not pin the repository release binary"
  for unit in "${NAV_UNIT_SOURCE}" "${GEOMETRY_UNIT_SOURCE}" \
              "${PERSON_FOLLOW_UNIT_SOURCE}" "${GATEWAY_UNIT_SOURCE}"; do
    grep -q 'ExecStart=/usr/bin/env .*NUWAX_LCM_BACKEND=real-lcm .*NUWAX_REAL_LCM_REQUIRED=true' \
      "${unit}" \
      || fail "production ExecStart does not force real-lcm: ${unit}"
  done
  ! grep -q 'kill -0' "${REPO_ROOT}/scripts/systemd/wait_for_nav_stack_ready.sh" \
    || fail "nav ready consumer must not signal a cross-UID process"
  ! grep -q 'kill -0' "${GEOMETRY_READY_HELPER}" \
    || fail "geometry ready consumer must not signal a cross-UID process"
  ! grep -q 'kill -0' "${PERSON_FOLLOW_READY_HELPER}" \
    || fail "person-follow ready consumer must not signal a cross-UID process"
  ! grep -q '/usr/bin/test -f /run/nuwax/geometry' "${PERSON_FOLLOW_UNIT_SOURCE}" \
    || fail "person-follow must use bounded geometry readiness, not a one-shot file test"
  ! grep -Eq '^(Requires|Wants)=.*nuwax-person-follow\.service' "${GATEWAY_UNIT_SOURCE}" \
    || fail "Gateway must not pull the optional person-follow unit"
  grep -qx 'BindsTo=nuwax-motion-safety-guard.service' "${NAV_UNIT_SOURCE}" \
    || fail "nav unit does not fail-stop with motion guard"
  grep -qx 'PartOf=nuwax-motion-safety-guard.service' "${NAV_UNIT_SOURCE}" \
    || fail "nav restart lifecycle is not part of motion guard"
  grep -qx 'BindsTo=nuwax-motion-safety-guard.service nuwax-nav-stack.service' \
    "${GEOMETRY_UNIT_SOURCE}" \
    || fail "geometry unit does not fail-stop with guard/nav"
  grep -qx 'PartOf=nuwax-motion-safety-guard.service nuwax-nav-stack.service' \
    "${GEOMETRY_UNIT_SOURCE}" \
    || fail "geometry restart lifecycle is not part of guard/nav"
  grep -qx 'BindsTo=nuwax-motion-safety-guard.service nuwax-nav-stack.service nuwax-geometry-runtime.service' \
    "${PERSON_FOLLOW_UNIT_SOURCE}" \
    || fail "person-follow unit does not fail-stop with mandatory dependencies"
  grep -qx 'PartOf=nuwax-motion-safety-guard.service nuwax-nav-stack.service nuwax-geometry-runtime.service' \
    "${PERSON_FOLLOW_UNIT_SOURCE}" \
    || fail "person-follow restart lifecycle is not part of mandatory dependencies"
  grep -qx 'BindsTo=nuwax-motion-safety-guard.service nuwax-nav-stack.service nuwax-geometry-runtime.service' \
    "${GATEWAY_UNIT_SOURCE}" \
    || fail "Gateway unit does not fail-stop with mandatory dependencies"
  grep -qx 'PartOf=nuwax-motion-safety-guard.service nuwax-nav-stack.service nuwax-geometry-runtime.service' \
    "${GATEWAY_UNIT_SOURCE}" \
    || fail "Gateway restart lifecycle is not part of mandatory dependencies"
  for assignment in \
      'NUWAX_GEOMETRY_RUNTIME_NAV_GOAL_CHANNEL=/nav/goal#nuwax_msgs.GoalEnvelope' \
      'NUWAX_LCM_CHANNEL_GOAL_POSE=/nav/goal#nuwax_msgs.GoalEnvelope' \
      'NUWAX_GEOMETRY_RUNTIME_NAV_PATH_CHANNEL=/nav/path#nuwax_msgs.PathEnvelope' \
      'NUWAX_LCM_CHANNEL_NAV_PATH=/nav/path#nuwax_msgs.PathEnvelope'; do
    for unit in "${NAV_UNIT_SOURCE}" "${GEOMETRY_UNIT_SOURCE}" "${GATEWAY_UNIT_SOURCE}"; do
      grep -qF "${assignment}" "${unit}" \
        || fail "missing production envelope channel ${assignment}: ${unit}"
    done
  done
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
  check_file "${LCM_PREFLIGHT}"
  check_file "${PROCESS_IDENTITY_HELPER}"
  check_file "${GEOMETRY_LAUNCHER}"
  check_file "${GEOMETRY_READY_HELPER}"
  check_file "${PERSON_FOLLOW_LAUNCHER}"
  check_file "${PERSON_FOLLOW_READY_HELPER}"
  systemd-sysusers --dry-run "${SYSUSERS_SOURCE}" >/dev/null
  check_service_uid_collisions "${SYSUSERS_SOURCE}"
  for unit in "${GUARD_UNIT_SOURCE}" "${NAV_UNIT_SOURCE}" "${GEOMETRY_UNIT_SOURCE}" \
              "${PERSON_FOLLOW_UNIT_SOURCE}" "${GATEWAY_UNIT_SOURCE}"; do
    grep -qF '@NUWAX_REPO_ROOT@' "${unit}" \
      || fail "systemd source is not a repository-root template: ${unit}"
  done
  bash -n \
    "${REPO_ROOT}/scripts/systemd/check_motion_guard_python.sh" \
    "${REPO_ROOT}/scripts/systemd/start_nav_stack.sh" \
    "${GEOMETRY_LAUNCHER}" \
    "${GEOMETRY_READY_HELPER}" \
    "${PERSON_FOLLOW_LAUNCHER}" \
    "${PERSON_FOLLOW_READY_HELPER}" \
    "${PROCESS_IDENTITY_HELPER}" \
    "${REPO_ROOT}/scripts/systemd/wait_for_nav_stack_ready.sh" \
    "${REPO_ROOT}/scripts/systemd/wait_for_motion_guard_socket.sh" \
    "${LCM_PREFLIGHT}" \
    "${REPO_ROOT}/scripts/build_nav_stack.sh" \
    "${REPO_ROOT}/scripts/start_gateway.sh"
  check_systemd_owned_invariants
  check_unit_templates
  echo "motion guard deployment sources: PASS"
}

check_runtime_binaries() {
  [[ -x "${NAV_BINARY}" ]] \
    || fail "navigation stack is not executable: ${NAV_BINARY}; run scripts/build_nav_stack.sh"
  [[ -x "${GEOMETRY_BINARY}" ]] \
    || fail "geometry runtime is not executable: ${GEOMETRY_BINARY}"
  if person_follow_autostart_enabled; then
    [[ -x "${PERSON_FOLLOW_BINARY}" ]] \
      || fail "person-follow runtime is not executable: ${PERSON_FOLLOW_BINARY}; build the release binary"
  fi
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
  runuser -u nuwax-geometry -- test -r "${GEOMETRY_LAUNCHER}" \
    || fail "nuwax-geometry cannot read start_geometry_runtime.sh"
  for user in nuwax-navigation nuwax-geometry nuwax-gateway; do
    runuser -u "${user}" -- test -r "${REPO_ROOT}/scripts/systemd/wait_for_nav_stack_ready.sh" \
      || fail "${user} cannot read wait_for_nav_stack_ready.sh"
    runuser -u "${user}" -- test -r "${PROCESS_IDENTITY_HELPER}" \
      || fail "${user} cannot read process_identity.sh"
    runuser -u "${user}" -- test -r "${GEOMETRY_READY_HELPER}" \
      || fail "${user} cannot read wait_for_geometry_runtime_ready.sh"
  done
  if person_follow_autostart_enabled; then
    runuser -u nuwax-navigation -- test -x "${PERSON_FOLLOW_BINARY}" \
      || fail "nuwax-navigation cannot traverse/read/execute ${PERSON_FOLLOW_BINARY}"
    runuser -u nuwax-navigation -- test -r "${PERSON_FOLLOW_LAUNCHER}" \
      || fail "nuwax-navigation cannot read start_person_follow.sh"
    runuser -u nuwax-navigation -- test -r "${PERSON_FOLLOW_READY_HELPER}" \
      || fail "nuwax-navigation cannot read wait_for_person_follow_ready.sh"
  fi
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
  check_service_uid_collisions system
  getent group nuwax-motion-clients >/dev/null || fail "missing group nuwax-motion-clients"
  for user in nuwax-motion-guard nuwax-motion-admin nuwax-navigation nuwax-geometry nuwax-gateway; do
    id -nG "${user}" | tr ' ' '\n' | grep -qx nuwax-motion-clients \
      || fail "${user} is not a member of nuwax-motion-clients"
  done
  for unit in nuwax-motion-safety-guard nuwax-nav-stack nuwax-geometry-runtime \
              nuwax-person-follow nuwax-host-gateway; do
    installed_unit="/etc/systemd/system/${unit}.service"
    [[ -f "${installed_unit}" ]] || fail "missing installed unit: ${installed_unit}"
    ! grep -qF '@NUWAX_REPO_ROOT@' "${installed_unit}" \
      || fail "unrendered repository placeholder: ${installed_unit}"
    grep -qF "WorkingDirectory=${REPO_ROOT}" "${installed_unit}" \
      || fail "installed unit does not target ${REPO_ROOT}: ${installed_unit}"
  done
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
  NUWAX_NAV_STACK_READY_FILE=/run/nuwax/navigation/nuwax_nav_stack.ready \
    bash "${REPO_ROOT}/scripts/systemd/wait_for_nav_stack_ready.sh" check \
    || fail "nuwax-nav-stack ready state is absent or stale"
  systemctl is-active --quiet nuwax-geometry-runtime.service \
    || fail "nuwax-geometry-runtime.service is not active"
  NUWAX_GEOMETRY_RUNTIME_READY_FILE=/run/nuwax/geometry/nuwax_geometry_runtime.ready \
    NUWAX_GEOMETRY_RUNTIME_PORT=50051 \
    bash "${GEOMETRY_READY_HELPER}" check \
    || fail "nuwax-geometry-runtime ready state is absent or stale"
  if person_follow_autostart_enabled; then
    systemctl is-active --quiet nuwax-person-follow.service \
      || fail "nuwax-person-follow.service is not active"
    NUWAX_PERSON_FOLLOW_READY_FILE=/run/nuwax/person-follow/nuwax_person_follow.ready \
      bash "${PERSON_FOLLOW_READY_HELPER}" check \
      || fail "nuwax-person-follow ready state is absent or stale"
  else
    ! systemctl is-enabled --quiet nuwax-person-follow.service \
      || fail "nuwax-person-follow.service is enabled while AUTOSTART=false"
  fi
  systemctl is-active --quiet nuwax-host-gateway.service \
    || fail "nuwax-host-gateway.service is not active"
  echo "motion guard deployment runtime: PASS"
}

install_sources() {
  require_root
  check_sources
  check_runtime_binaries
  install -D -m 0644 "${SYSUSERS_SOURCE}" /usr/lib/sysusers.d/nuwax-motion.conf
  install -D -m 0644 "${TMPFILES_SOURCE}" /usr/lib/tmpfiles.d/nuwax-motion.conf
  systemd-sysusers /usr/lib/sysusers.d/nuwax-motion.conf
  check_service_uid_collisions system
  systemd-tmpfiles --create /usr/lib/tmpfiles.d/nuwax-motion.conf
  bash "${LCM_PREFLIGHT}"
  check_service_access
  render_unit "${GUARD_UNIT_SOURCE}" /etc/systemd/system/nuwax-motion-safety-guard.service
  render_unit "${NAV_UNIT_SOURCE}" /etc/systemd/system/nuwax-nav-stack.service
  render_unit "${GEOMETRY_UNIT_SOURCE}" /etc/systemd/system/nuwax-geometry-runtime.service
  render_unit "${PERSON_FOLLOW_UNIT_SOURCE}" /etc/systemd/system/nuwax-person-follow.service
  render_unit "${GATEWAY_UNIT_SOURCE}" /etc/systemd/system/nuwax-host-gateway.service
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
  if person_follow_autostart_enabled; then
    systemctl enable nuwax-person-follow.service
  else
    systemctl disable nuwax-person-follow.service
  fi
  systemctl enable nuwax-host-gateway.service
  echo "enabled mandatory startup order: guard -> nav-stack -> geometry runtime -> Gateway; person-follow autostart=${PERSON_FOLLOW_AUTOSTART_MODE}"
}

provision_records() {
  require_root
  install_actuator_manifest
  "${PROVISIONER}" provision \
    --state-dir "${GUARD_STATE_DIR}" \
    --actuator-symbols-file "${INSTALLED_ACTUATOR_SYMBOLS_FILE}"
}

migrate_peer_allowlist_records() {
  require_root
  check_installed_actuator_manifest
  "${PROVISIONER}" migrate-peer-allowlist \
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
  migrate-peer-allowlist)
    migrate_peer_allowlist_records
    ;;
  enable)
    enable_stack
    ;;
  verify-runtime)
    verify_installed
    ;;
  *)
    echo "usage: $0 [check-sources|install|provision|migrate-peer-allowlist|enable|verify-runtime] [actuator-symbols.json]" >&2
    exit 2
    ;;
esac
