#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="${NUWAX_REPO_ROOT_OVERRIDE:-${DEFAULT_REPO_ROOT}}"
[[ "${REPO_ROOT}" == "/" ]] || REPO_ROOT="${REPO_ROOT%/}"
RENDER_ROOT="${NUWAX_RENDER_ROOT:-${REPO_ROOT}}"
[[ "${RENDER_ROOT}" == "/" ]] || RENDER_ROOT="${RENDER_ROOT%/}"
ACTION="${1:-check-sources}"
ACTUATOR_SYMBOLS_FILE="${2:-${NUWAX_ACTUATOR_SYMBOLS_FILE:-}}"

BIN_DIR="${NUWAX_BIN_DIR:-${REPO_ROOT}/bin}"
RENDER_BIN_DIR="${NUWAX_RENDER_BIN_DIR:-${BIN_DIR}}"
CONFIG_DIR="${NUWAX_CONFIG_DIR:-/etc/nuwax}"
RENDER_CONFIG_DIR="${NUWAX_RENDER_CONFIG_DIR:-${CONFIG_DIR}}"
NUWAX_ENV_FILE="${CONFIG_DIR}/nuwax.env"
GO2_DATA_PLANE_ENV_FILE="${CONFIG_DIR}/go2-data-plane.env"
PYTHON_BIN="${NUWAX_PYTHON_BIN:-${REPO_ROOT}/.venv/bin/python}"
if [[ -z "${NUWAX_PYTHON_BIN:-}" && ! -x "${PYTHON_BIN}" ]]; then
  PYTHON_BIN=/usr/bin/python3
fi
RENDER_PYTHON_BIN="${NUWAX_RENDER_PYTHON_BIN:-${PYTHON_BIN}}"

SYSUSERS_SOURCE="${NUWAX_MOTION_SYSUSERS_SOURCE:-${REPO_ROOT}/deploy/sysusers.d/nuwax-motion.conf}"
TMPFILES_SOURCE="${REPO_ROOT}/deploy/tmpfiles.d/nuwax-motion.conf"
GUARD_UNIT_SOURCE="${REPO_ROOT}/deploy/systemd/nuwax-motion-safety-guard.service"
NAV_UNIT_SOURCE="${REPO_ROOT}/deploy/systemd/nuwax-nav-stack.service"
GEOMETRY_UNIT_SOURCE="${REPO_ROOT}/deploy/systemd/nuwax-geometry-runtime.service"
PERSON_FOLLOW_UNIT_SOURCE="${REPO_ROOT}/deploy/systemd/nuwax-person-follow.service"
RUNTIME_ACTIVATION_UNIT_SOURCE="${REPO_ROOT}/deploy/systemd/nuwax-runtime-activation.service"
RUNTIME_ACTIVATION_PATH_SOURCE="${REPO_ROOT}/deploy/systemd/nuwax-runtime-activation.path"
GATEWAY_UNIT_SOURCE="${REPO_ROOT}/deploy/systemd/nuwax-host-gateway.service"
VIDEO_STREAMER_UNIT_SOURCE="${REPO_ROOT}/drivers/go2/systemd/nuwax-go2-video-streamer.service"
GUARD_MODULE="${REPO_ROOT}/services/navigation/motion_safety_guard/__main__.py"
PROVISIONER="${REPO_ROOT}/scripts/provision_motion_guard.py"
LCM_PREFLIGHT="${REPO_ROOT}/scripts/systemd/check_lcm_multicast.sh"
PROCESS_IDENTITY_HELPER="${REPO_ROOT}/scripts/systemd/process_identity.sh"
GEOMETRY_LAUNCHER="${REPO_ROOT}/scripts/systemd/start_geometry_runtime.sh"
GEOMETRY_READY_HELPER="${REPO_ROOT}/scripts/systemd/wait_for_geometry_runtime_ready.sh"
PERSON_FOLLOW_LAUNCHER="${REPO_ROOT}/scripts/systemd/start_person_follow.sh"
PERSON_FOLLOW_READY_HELPER="${REPO_ROOT}/scripts/systemd/wait_for_person_follow_ready.sh"
NAV_BINARY="${BIN_DIR}/nuwax-nav-stack"
NAV_LAUNCHER_CONTRACT="nuwax-nav-stack-internal-ready-v1"
GEOMETRY_BINARY="${BIN_DIR}/nuwax-geometry-runtime"
PERSON_FOLLOW_BINARY="${BIN_DIR}/nuwax-person-follow"
GUARD_SOCKET="${NUWAX_MOTION_GUARD_SOCKET:-/run/nuwax/motion-guard.sock}"
GUARD_LOCK="${NUWAX_MOTION_GUARD_LOCK:-/run/nuwax/motion-guard.lock}"
GUARD_STATE_DIR="${NUWAX_MOTION_GUARD_STATE_DIR:-/var/lib/nuwax/motion-guard}"
SDK_PATH="${NUWAX_GO2_SDK_PATH:-${GO2_SDK_PATH:-/home/unitree/unitree_sdk2_python}}"
RENDER_SDK_PATH="${NUWAX_RENDER_GO2_SDK_PATH:-${SDK_PATH}}"
INSTALLED_ACTUATOR_SYMBOLS_FILE="${CONFIG_DIR}/actuator/actuator-symbols.json"
SYSTEMD_BACKUP_DIR="${NUWAX_SYSTEMD_BACKUP_DIR:-}"
PERSON_FOLLOW_AUTOSTART_MODE=""
SERVICE_IDENTITY_USERS=(
  nuwax-motion-guard
  nuwax-navigation
  nuwax-geometry
  nuwax-gateway
)
GATEWAY_WRITE_DIRS=(
  /var/lib/nuwax/gateway/artifacts
  /var/lib/nuwax/gateway/maps
  /var/lib/nuwax/gateway/memory
  /var/lib/nuwax/gateway/geometry/mapping
  /var/lib/nuwax/gateway/geometry/pcd
  /var/lib/nuwax/gateway/ultralytics
  /var/lib/nuwax/gateway/config
  /var/lib/nuwax/gateway/cache
  /var/log/nuwax/gateway
)
DEPLOYMENT_UNITS=(
  nuwax-motion-safety-guard.service
  nuwax-nav-stack.service
  nuwax-geometry-runtime.service
  nuwax-person-follow.service
  nuwax-runtime-activation.service
  nuwax-runtime-activation.path
  nuwax-host-gateway.service
  nuwax-go2-video-streamer.service
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
  elif [[ -f "${NUWAX_ENV_FILE}" ]]; then
    while IFS= read -r raw_line || [[ -n "${raw_line}" ]]; do
      candidate="${raw_line#"${raw_line%%[![:space:]]*}"}"
      [[ -n "${candidate}" && "${candidate}" != \#* && "${candidate}" == *=* ]] \
        || continue
      key="${candidate%%=*}"
      key="${key%"${key##*[![:space:]]}"}"
      if [[ "${key}" == "NUWAX_PERSON_FOLLOW_AUTOSTART" ]]; then
        value="${candidate#*=}"
      fi
    done < "${NUWAX_ENV_FILE}"
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

validate_template_path() {
  local name="$1"
  local value="$2"
  [[ -n "${value}" && "${value}" == /* ]] \
    || fail "${name} must be an absolute path: ${value:-<unset>}"
  [[ "${value}" != *[[:space:]]* ]] \
    || fail "${name} cannot contain whitespace in systemd templates: ${value}"
}

render_unit_to() {
  local source="$1"
  local destination="$2"
  local escaped_root=""
  local escaped_sdk=""
  local escaped_python=""
  local escaped_bin=""
  local escaped_config=""
  validate_template_path NUWAX_RENDER_ROOT "${RENDER_ROOT}"
  validate_template_path NUWAX_RENDER_GO2_SDK_PATH "${RENDER_SDK_PATH}"
  validate_template_path NUWAX_RENDER_PYTHON_BIN "${RENDER_PYTHON_BIN}"
  validate_template_path NUWAX_RENDER_BIN_DIR "${RENDER_BIN_DIR}"
  validate_template_path NUWAX_RENDER_CONFIG_DIR "${RENDER_CONFIG_DIR}"
  escaped_root="$(printf '%s' "${RENDER_ROOT}" | sed 's/[&|\\]/\\&/g')"
  escaped_sdk="$(printf '%s' "${RENDER_SDK_PATH}" | sed 's/[&|\\]/\\&/g')"
  escaped_python="$(printf '%s' "${RENDER_PYTHON_BIN}" | sed 's/[&|\\]/\\&/g')"
  escaped_bin="$(printf '%s' "${RENDER_BIN_DIR}" | sed 's/[&|\\]/\\&/g')"
  escaped_config="$(printf '%s' "${RENDER_CONFIG_DIR}" | sed 's/[&|\\]/\\&/g')"
  sed \
    -e "s|@NUWAX_REPO_ROOT@|${escaped_root}|g" \
    -e "s|@NUWAX_GO2_SDK_PATH@|${escaped_sdk}|g" \
    -e "s|@NUWAX_PYTHON_BIN@|${escaped_python}|g" \
    -e "s|@NUWAX_BIN_DIR@|${escaped_bin}|g" \
    -e "s|@NUWAX_CONFIG_DIR@|${escaped_config}|g" \
    "${source}" > "${destination}"
  ! grep -Eq '@NUWAX_[A-Z0-9_]+@' "${destination}" \
    || fail "unresolved deployment placeholder in ${source}"
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
                "${PERSON_FOLLOW_UNIT_SOURCE}" "${RUNTIME_ACTIVATION_UNIT_SOURCE}" \
                "${RUNTIME_ACTIVATION_PATH_SOURCE}" "${GATEWAY_UNIT_SOURCE}" \
                "${VIDEO_STREAMER_UNIT_SOURCE}"; do
    render_unit_to "${source}" "${rendered_dir}/$(basename "${source}")"
  done
  # Unit syntax validation is independent from release binary availability;
  # install-time binary gates are conditional on person-follow autostart.
  unit_path="${rendered_dir}:/etc/systemd/system:/run/systemd/system:/usr/local/lib/systemd/system:/usr/lib/systemd/system:/lib/systemd/system"
  if ! SYSTEMD_UNIT_PATH="${unit_path}" systemd-analyze verify \
      "${rendered_dir}"/*.service "${rendered_dir}"/*.path; then
    rm -rf "${rendered_dir}"
    fail "rendered systemd unit verification failed"
  fi
  rm -rf "${rendered_dir}"
}

check_systemd_owned_invariants() {
  local unit=""
  local command=""
  for unit in "${GUARD_UNIT_SOURCE}" "${NAV_UNIT_SOURCE}" "${GEOMETRY_UNIT_SOURCE}" \
              "${PERSON_FOLLOW_UNIT_SOURCE}" "${RUNTIME_ACTIVATION_UNIT_SOURCE}" \
              "${GATEWAY_UNIT_SOURCE}" "${VIDEO_STREAMER_UNIT_SOURCE}"; do
    grep -qx 'WorkingDirectory=@NUWAX_REPO_ROOT@' "${unit}" \
      || fail "production service must start from the rendered repository root: ${unit}"
    grep -qx 'ProtectHome=read-only' "${unit}" \
      || fail "repository-backed production service must keep /home readable: ${unit}"
    ! grep -qx 'ProtectHome=yes' "${unit}" \
      || fail "repository-backed production service cannot hide its /home working directory: ${unit}"
  done
  grep -Eq '^ReadWritePaths=([^[:space:]]+[[:space:]])*/tmp([[:space:]]|$)' \
    "${GATEWAY_UNIT_SOURCE}" \
    || fail "Gateway unit must expose shared /tmp for CycloneDDS and video IPC"

  for unit in "${GUARD_UNIT_SOURCE}" "${NAV_UNIT_SOURCE}" "${GEOMETRY_UNIT_SOURCE}" \
              "${PERSON_FOLLOW_UNIT_SOURCE}" "${RUNTIME_ACTIVATION_UNIT_SOURCE}" \
              "${GATEWAY_UNIT_SOURCE}" "${VIDEO_STREAMER_UNIT_SOURCE}"; do
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
  grep -qF 'NUWAX_NAV_STACK_BINARY=@NUWAX_BIN_DIR@/nuwax-nav-stack' \
    "${NAV_UNIT_SOURCE}" \
    || fail "nav ExecStart does not pin the installed release binary"
  grep -qF 'NUWAX_GEOMETRY_RUNTIME_BINARY=@NUWAX_BIN_DIR@/nuwax-geometry-runtime' \
    "${GEOMETRY_UNIT_SOURCE}" \
    || fail "geometry ExecStart does not pin the installed release binary"
  grep -qF 'NUWAX_PERSON_FOLLOW_BINARY=@NUWAX_BIN_DIR@/nuwax-person-follow' \
    "${PERSON_FOLLOW_UNIT_SOURCE}" \
    || fail "person-follow ExecStart does not pin the installed release binary"
  for unit in "${NAV_UNIT_SOURCE}" "${GEOMETRY_UNIT_SOURCE}" \
              "${PERSON_FOLLOW_UNIT_SOURCE}" "${GATEWAY_UNIT_SOURCE}"; do
    grep -q 'ExecStart=/usr/bin/env .*NUWAX_LCM_BACKEND=real-lcm .*NUWAX_REAL_LCM_REQUIRED=true' \
      "${unit}" \
      || fail "production ExecStart does not force real-lcm: ${unit}"
  done
  [[ "$(grep -cF 'PYTHONPATH=@NUWAX_REPO_ROOT@:@NUWAX_GO2_SDK_PATH@' "${GUARD_UNIT_SOURCE}")" -eq 3 ]] \
    || fail "motion guard must pin repository and SDK paths in every Python execution context"
  grep -qF 'NUWAX_PYTHON_BIN=@NUWAX_PYTHON_BIN@' "${GUARD_UNIT_SOURCE}" \
    || fail "motion guard preflight does not pin the rendered Python interpreter"
  grep -qF '@NUWAX_PYTHON_BIN@ -m services.navigation.motion_safety_guard' \
    "${GUARD_UNIT_SOURCE}" \
    || fail "motion guard ExecStart does not pin the rendered Python interpreter"
  for unit in "${GUARD_UNIT_SOURCE}" "${NAV_UNIT_SOURCE}" "${GEOMETRY_UNIT_SOURCE}" \
              "${PERSON_FOLLOW_UNIT_SOURCE}" "${GATEWAY_UNIT_SOURCE}" \
              "${VIDEO_STREAMER_UNIT_SOURCE}"; do
    grep -qF 'EnvironmentFile=-@NUWAX_CONFIG_DIR@/nuwax.env' "${unit}" \
      || fail "production unit does not use the managed config directory: ${unit}"
  done
  for unit in "${GUARD_UNIT_SOURCE}" "${GATEWAY_UNIT_SOURCE}" \
              "${VIDEO_STREAMER_UNIT_SOURCE}"; do
    grep -qF 'EnvironmentFile=-@NUWAX_CONFIG_DIR@/go2-data-plane.env' "${unit}" \
      || fail "Go2-facing service does not use the managed data-plane config: ${unit}"
  done
  grep -qF 'SupplementaryGroups=nuwax-motion-clients video render' "${GATEWAY_UNIT_SOURCE}" \
    || fail "Gateway unit must include Jetson video/render device groups"
  grep -qF 'SupplementaryGroups=nuwax-motion-clients video render' "${VIDEO_STREAMER_UNIT_SOURCE}" \
    || fail "Go2 video streamer unit must include Jetson video/render device groups"
  grep -qx 'Wants=network-online.target nuwax-geometry-runtime.service' \
    "${VIDEO_STREAMER_UNIT_SOURCE}" \
    || fail "Go2 video streamer must start the geometry-owned video relay"
  grep -qx 'After=network-online.target nuwax-geometry-runtime.service' \
    "${VIDEO_STREAMER_UNIT_SOURCE}" \
    || fail "Go2 video streamer must wait for the geometry-owned video relay"
  grep -qF 'NUWAX_VIDEO_UDS_PATH=/tmp/nuwax_video.sock' \
    "${VIDEO_STREAMER_UNIT_SOURCE}" \
    || fail "Go2 video streamer does not pin the production video UDS"
  grep -qF '@NUWAX_PYTHON_BIN@ -m drivers.go2.video_streamer' \
    "${VIDEO_STREAMER_UNIT_SOURCE}" \
    || fail "Go2 video streamer does not pin its production Python entrypoint"
  grep -qF 'Environment=NUWAX_BOOTSTRAP_ENV_FILE=@NUWAX_CONFIG_DIR@/nuwax.env' \
    "${GATEWAY_UNIT_SOURCE}" \
    || fail "Gateway unit does not pin the managed bootstrap environment"
  ! grep -q '^RuntimeDirectory=' "${GUARD_UNIT_SOURCE}" \
    || fail "motion guard must not own shared /run/nuwax as RuntimeDirectory"
  for assignment in \
      'NUWAX_CLOUDMAP_SHM_PATH=/var/lib/nuwax_robot_bridge/nuwax_cloudmap.mmap' \
      'NUWAX_OCCUPANCY_SHM_PATH=/var/lib/nuwax_robot_bridge/nuwax_occupancy.mmap'; do
    grep -qF "${assignment}" "${GEOMETRY_UNIT_SOURCE}" \
      || fail "geometry unit does not pin ${assignment}"
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
  ! grep -Eq '^(Requires|BindsTo|PartOf)=.*nuwax-(motion-safety-guard|nav-stack|geometry-runtime|person-follow)\.service' \
    "${GATEWAY_UNIT_SOURCE}" \
    || fail "Gateway must remain independent from optional runtime components"
  grep -qx 'After=network-online.target' "${GATEWAY_UNIT_SOURCE}" \
    || fail "Gateway must only order itself after the network"
  grep -qF 'PathExists=/run/nuwax/gateway/geometry.request' \
    "${RUNTIME_ACTIVATION_PATH_SOURCE}" \
    || fail "runtime activation path does not watch the Gateway request file"
  grep -qF 'activate_geometry_runtime.sh' "${RUNTIME_ACTIVATION_UNIT_SOURCE}" \
    || fail "runtime activation service does not call the canonical activator"
  grep -qx 'BindsTo=nuwax-motion-safety-guard.service' "${NAV_UNIT_SOURCE}" \
    || fail "nav unit does not fail-stop with motion guard"
  grep -qx 'PartOf=nuwax-motion-safety-guard.service' "${NAV_UNIT_SOURCE}" \
    || fail "nav restart lifecycle is not part of motion guard"
  ! grep -Eq '^(Requires|BindsTo|PartOf)=.*nuwax-(motion-safety-guard|nav-stack)\.service' \
    "${GEOMETRY_UNIT_SOURCE}" \
    || fail "geometry unit must remain independently startable for mapping"
  grep -qx 'After=network-online.target' "${GEOMETRY_UNIT_SOURCE}" \
    || fail "geometry unit must only order itself after the network"
  ! grep -Eq 'wait_for_(motion_guard_socket|nav_stack_ready)\.sh' \
    "${GEOMETRY_UNIT_SOURCE}" \
    || fail "geometry startup must not wait for motion-only components"
  grep -qx 'BindsTo=nuwax-motion-safety-guard.service nuwax-nav-stack.service nuwax-geometry-runtime.service' \
    "${PERSON_FOLLOW_UNIT_SOURCE}" \
    || fail "person-follow unit does not fail-stop with mandatory dependencies"
  grep -qx 'PartOf=nuwax-motion-safety-guard.service nuwax-nav-stack.service nuwax-geometry-runtime.service' \
    "${PERSON_FOLLOW_UNIT_SOURCE}" \
    || fail "person-follow restart lifecycle is not part of mandatory dependencies"
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

validate_sysusers_source() {
  local rendered_root=""

  if systemd-sysusers --help 2>&1 | grep -q -- '--dry-run'; then
    systemd-sysusers --dry-run "${SYSUSERS_SOURCE}" >/dev/null
    return
  fi

  # systemd 245 (used by the Go2 image) has --root but no --dry-run.
  # Render into an isolated root so validation never mutates host accounts.
  rendered_root="$(mktemp -d)"
  mkdir -p "${rendered_root}/etc"
  if ! systemd-sysusers --root="${rendered_root}" "${SYSUSERS_SOURCE}" >/dev/null; then
    rm -rf "${rendered_root}"
    fail "cannot validate sysusers source: ${SYSUSERS_SOURCE}"
  fi
  rm -rf "${rendered_root}"
}

check_sources() {
  check_file "${SYSUSERS_SOURCE}"
  check_file "${TMPFILES_SOURCE}"
  check_file "${GUARD_UNIT_SOURCE}"
  check_file "${NAV_UNIT_SOURCE}"
  check_file "${GEOMETRY_UNIT_SOURCE}"
  check_file "${PERSON_FOLLOW_UNIT_SOURCE}"
  check_file "${RUNTIME_ACTIVATION_UNIT_SOURCE}"
  check_file "${RUNTIME_ACTIVATION_PATH_SOURCE}"
  check_file "${GATEWAY_UNIT_SOURCE}"
  check_file "${VIDEO_STREAMER_UNIT_SOURCE}"
  check_file "${GUARD_MODULE}"
  check_file "${PROVISIONER}"
  check_file "${LCM_PREFLIGHT}"
  check_file "${PROCESS_IDENTITY_HELPER}"
  check_file "${GEOMETRY_LAUNCHER}"
  check_file "${GEOMETRY_READY_HELPER}"
  check_file "${PERSON_FOLLOW_LAUNCHER}"
  check_file "${PERSON_FOLLOW_READY_HELPER}"
  check_file "${REPO_ROOT}/scripts/systemd/activate_geometry_runtime.sh"
  grep -qF 'API_VERSION = 2' "${REPO_ROOT}/services/navigation/motion_safety_guard/constants.py" \
    || fail "motion guard Python API version is not v2"
  grep -qF 'package nuwax.motion_guard.v2;' "${REPO_ROOT}/services/navigation/proto/motion_guard.proto" \
    || fail "motion guard proto package is not v2"
  grep -qF 'startup_zero_confirmed' "${REPO_ROOT}/services/navigation/proto/motion_guard.proto" \
    || fail "motion guard v2 proto does not expose startup zero confirmation"
  ! grep -Eq '^(enum SafetyGate|message (LatchSafetyStop|ResetSafeStop))' \
    "${REPO_ROOT}/services/navigation/proto/motion_guard.proto" \
    || fail "retired motion gate/reset messages remain in the v2 proto"
  validate_sysusers_source
  check_service_uid_collisions "${SYSUSERS_SOURCE}"
  for unit in "${GUARD_UNIT_SOURCE}" "${NAV_UNIT_SOURCE}" "${GEOMETRY_UNIT_SOURCE}" \
              "${PERSON_FOLLOW_UNIT_SOURCE}" "${GATEWAY_UNIT_SOURCE}" \
              "${VIDEO_STREAMER_UNIT_SOURCE}"; do
    grep -qF '@NUWAX_REPO_ROOT@' "${unit}" \
      || fail "systemd source is not a repository-root template: ${unit}"
    grep -qF '@NUWAX_CONFIG_DIR@' "${unit}" \
      || fail "systemd source is not a config-directory template: ${unit}"
  done
  grep -qF '@NUWAX_REPO_ROOT@' "${RUNTIME_ACTIVATION_UNIT_SOURCE}" \
    || fail "runtime activation service is not a repository-root template"
  grep -qF '@NUWAX_PYTHON_BIN@' "${GUARD_UNIT_SOURCE}" \
    || fail "motion guard source is not a Python interpreter template"
  grep -qF '@NUWAX_PYTHON_BIN@' "${VIDEO_STREAMER_UNIT_SOURCE}" \
    || fail "Go2 video streamer source is not a Python interpreter template"
  for unit in "${NAV_UNIT_SOURCE}" "${GEOMETRY_UNIT_SOURCE}" "${PERSON_FOLLOW_UNIT_SOURCE}"; do
    grep -qF '@NUWAX_BIN_DIR@' "${unit}" \
      || fail "Rust service source is not a release bin template: ${unit}"
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
    "${REPO_ROOT}/scripts/systemd/activate_geometry_runtime.sh" \
    "${LCM_PREFLIGHT}" \
    "${REPO_ROOT}/scripts/build_nav_stack.sh" \
    "${REPO_ROOT}/scripts/start_gateway.sh"
  check_systemd_owned_invariants
  check_unit_templates
  echo "motion guard deployment sources: PASS"
}

check_runtime_config() {
  [[ -d "${CONFIG_DIR}" && ! -L "${CONFIG_DIR}" && -x "${CONFIG_DIR}" ]] \
    || fail "runtime config directory is missing, inaccessible, or a symlink: ${CONFIG_DIR}"
  [[ -f "${NUWAX_ENV_FILE}" && ! -L "${NUWAX_ENV_FILE}" && -r "${NUWAX_ENV_FILE}" ]] \
    || fail "runtime config is missing, unreadable, non-regular, or a symlink: ${NUWAX_ENV_FILE}"
  [[ -f "${GO2_DATA_PLANE_ENV_FILE}" && ! -L "${GO2_DATA_PLANE_ENV_FILE}" \
    && -r "${GO2_DATA_PLANE_ENV_FILE}" ]] \
    || fail "Go2 data-plane config is missing, unreadable, non-regular, or a symlink: ${GO2_DATA_PLANE_ENV_FILE}"
}

check_runtime_config_permissions() {
  local path=""
  for path in "${NUWAX_ENV_FILE}" "${GO2_DATA_PLANE_ENV_FILE}"; do
    [[ "$(stat -c '%U:%G %a' "${path}")" == "root:nuwax-motion-clients 640" ]] \
      || fail "runtime config must be root:nuwax-motion-clients 0640: ${path}"
  done
}

secure_runtime_config_permissions() {
  local path=""
  check_runtime_config
  for path in "${NUWAX_ENV_FILE}" "${GO2_DATA_PLANE_ENV_FILE}"; do
    chown root:nuwax-motion-clients "${path}"
    chmod 0640 "${path}"
  done
  check_runtime_config_permissions
}

check_runtime_binaries() {
  [[ -x "${PYTHON_BIN}" ]] \
    || fail "motion guard Python interpreter is not executable: ${PYTHON_BIN}"
  [[ -x "${NAV_BINARY}" ]] \
    || fail "navigation stack is not executable: ${NAV_BINARY}"
  local nav_contract=""
  nav_contract="$(timeout 2s "${NAV_BINARY}" --launcher-contract 2>/dev/null || true)"
  [[ "${nav_contract}" == "${NAV_LAUNCHER_CONTRACT}" ]] \
    || fail "navigation stack does not support the internal readiness protocol: ${NAV_BINARY}; rebuild it via scripts/build_nav_stack.sh"
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
  [[ "$(stat -c '%U:%G %a' "${INSTALLED_ACTUATOR_SYMBOLS_FILE}")" == "root:nuwax-motion-clients 640" ]] \
    || fail "installed actuator symbols manifest must be root:nuwax-motion-clients 0640"
}

check_identity_write_access() {
  local user="$1"
  local directory="$2"
  runuser -u "${user}" -- test -w "${directory}" \
    || fail "${user} cannot write ${directory}"
  runuser -u "${user}" -- sh -c '
    set -eu
    directory="$1"
    probe="${directory}/.nuwax-access-$$"
    trap '\''rm -f "${probe}"'\'' EXIT HUP INT TERM
    umask 077
    : > "${probe}"
    rm -f "${probe}"
    trap - EXIT HUP INT TERM
  ' sh "${directory}" || fail "${user} failed the real write probe for ${directory}"
}

ensure_gateway_device_groups() {
  local group=""
  for group in video render; do
    getent group "${group}" >/dev/null \
      || fail "Jetson device group is missing: ${group}"
    usermod -a -G "${group}" nuwax-gateway
  done
}

secure_runtime_data_permissions() {
  local directory=""
  local unsafe=""
  for directory in "${GATEWAY_WRITE_DIRS[@]}"; do
    unsafe="$(find -P "${directory}" -xdev ! -type d ! -type f -print -quit)"
    [[ -z "${unsafe}" ]] \
      || fail "gateway writable tree contains a link or special file: ${unsafe}"
    chown -hR nuwax-gateway:nuwax-gateway "${directory}"
    find "${directory}" -xdev -type d -exec chmod 0750 {} +
    find "${directory}" -xdev -type f -exec chmod 0640 {} +
  done
  chown nuwax-geometry:nuwax-motion-clients /var/lib/nuwax_robot_bridge
  chmod 2770 /var/lib/nuwax_robot_bridge
  [[ -f /tmp/cdds.LOG && ! -L /tmp/cdds.LOG ]] \
    || fail "/tmp/cdds.LOG must be a regular non-symlink created by systemd-tmpfiles"
  [[ "$(stat -c '%U:%G:%a' /tmp/cdds.LOG)" == "root:nuwax-motion-clients:660" ]] \
    || fail "/tmp/cdds.LOG must be root:nuwax-motion-clients 0660"
}

snapshot_systemd_state() {
  local unit=""
  local base=""
  local dropins=""
  local enabled="disabled"
  local active="inactive"
  [[ -n "${SYSTEMD_BACKUP_DIR}" ]] \
    || fail "NUWAX_SYSTEMD_BACKUP_DIR is required for transactional systemd changes"
  if [[ -f "${SYSTEMD_BACKUP_DIR}/snapshot.complete" ]]; then
    return 0
  fi
  mkdir -p \
    "${SYSTEMD_BACKUP_DIR}/units" \
    "${SYSTEMD_BACKUP_DIR}/dropins" \
    "${SYSTEMD_BACKUP_DIR}/sysusers" \
    "${SYSTEMD_BACKUP_DIR}/tmpfiles"
  : > "${SYSTEMD_BACKUP_DIR}/service-state.tsv"
  for unit in "${DEPLOYMENT_UNITS[@]}"; do
    base="/etc/systemd/system/${unit}"
    dropins="${base}.d"
    enabled="$(systemctl is-enabled "${unit}" 2>/dev/null || true)"
    active="$(systemctl is-active "${unit}" 2>/dev/null || true)"
    printf '%s\t%s\t%s\n' "${unit}" "${enabled:-disabled}" "${active:-inactive}" \
      >> "${SYSTEMD_BACKUP_DIR}/service-state.tsv"
    [[ ! -L "${base}" ]] || fail "refusing systemd unit symlink: ${base}"
    if [[ -f "${base}" && ! -L "${base}" ]]; then
      cp -a "${base}" "${SYSTEMD_BACKUP_DIR}/units/${unit}"
    fi
    if [[ -d "${dropins}" && ! -L "${dropins}" ]]; then
      cp -a "${dropins}" "${SYSTEMD_BACKUP_DIR}/dropins/${unit}.d"
    elif [[ -L "${dropins}" ]]; then
      fail "refusing systemd drop-in symlink: ${dropins}"
    fi
  done
  for path in \
      /usr/lib/sysusers.d/nuwax-motion.conf \
      /usr/lib/tmpfiles.d/nuwax-motion.conf; do
    [[ ! -L "${path}" ]] || fail "refusing systemd support-file symlink: ${path}"
    if [[ -f "${path}" && ! -L "${path}" ]]; then
      case "${path}" in
        */sysusers.d/*) cp -a "${path}" "${SYSTEMD_BACKUP_DIR}/sysusers/" ;;
        */tmpfiles.d/*) cp -a "${path}" "${SYSTEMD_BACKUP_DIR}/tmpfiles/" ;;
      esac
    fi
  done
  chmod -R go-rwx "${SYSTEMD_BACKUP_DIR}"
  : > "${SYSTEMD_BACKUP_DIR}/snapshot.complete"
}

remove_legacy_dropins() {
  local unit=""
  local dropins=""
  snapshot_systemd_state
  for unit in "${DEPLOYMENT_UNITS[@]}"; do
    dropins="/etc/systemd/system/${unit}.d"
    if [[ -d "${dropins}" && ! -L "${dropins}" ]]; then
      rm -rf "${dropins}"
    elif [[ -L "${dropins}" ]]; then
      fail "refusing systemd drop-in symlink: ${dropins}"
    fi
  done
}

systemd_sources_changed() {
  local rendered_dir=""
  local source=""
  local installed=""
  local unit=""
  rendered_dir="$(mktemp -d)"
  for source in "${GUARD_UNIT_SOURCE}" "${NAV_UNIT_SOURCE}" "${GEOMETRY_UNIT_SOURCE}" \
                "${PERSON_FOLLOW_UNIT_SOURCE}" "${RUNTIME_ACTIVATION_UNIT_SOURCE}" \
                "${RUNTIME_ACTIVATION_PATH_SOURCE}" "${GATEWAY_UNIT_SOURCE}" \
                "${VIDEO_STREAMER_UNIT_SOURCE}"; do
    unit="$(basename "${source}")"
    render_unit_to "${source}" "${rendered_dir}/${unit}"
    installed="/etc/systemd/system/${unit}"
    if [[ ! -f "${installed}" || -L "${installed}" ]] \
        || ! cmp -s "${rendered_dir}/${unit}" "${installed}" \
        || [[ -e "${installed}.d" ]]; then
      rm -rf "${rendered_dir}"
      return 0
    fi
  done
  if [[ ! -f /usr/lib/sysusers.d/nuwax-motion.conf ]] \
      || ! cmp -s "${SYSUSERS_SOURCE}" /usr/lib/sysusers.d/nuwax-motion.conf \
      || [[ ! -f /usr/lib/tmpfiles.d/nuwax-motion.conf ]] \
      || ! cmp -s "${TMPFILES_SOURCE}" /usr/lib/tmpfiles.d/nuwax-motion.conf; then
    rm -rf "${rendered_dir}"
    return 0
  fi
  rm -rf "${rendered_dir}"
  return 1
}

restore_systemd_state() {
  local unit=""
  local enabled=""
  local active=""
  local base=""
  local saved=""
  [[ -f "${SYSTEMD_BACKUP_DIR}/snapshot.complete" ]] \
    || fail "systemd rollback snapshot is missing: ${SYSTEMD_BACKUP_DIR}"
  for unit in "${DEPLOYMENT_UNITS[@]}"; do
    base="/etc/systemd/system/${unit}"
    saved="${SYSTEMD_BACKUP_DIR}/units/${unit}"
    if [[ -f "${saved}" ]]; then
      install -D -m 0644 "${saved}" "${base}"
    else
      rm -f "${base}"
    fi
    rm -rf "${base}.d"
    if [[ -d "${SYSTEMD_BACKUP_DIR}/dropins/${unit}.d" ]]; then
      cp -a "${SYSTEMD_BACKUP_DIR}/dropins/${unit}.d" "${base}.d"
    fi
  done
  for kind in sysusers tmpfiles; do
    saved="${SYSTEMD_BACKUP_DIR}/${kind}/nuwax-motion.conf"
    if [[ -f "${saved}" ]]; then
      install -D -m 0644 "${saved}" "/usr/lib/${kind}.d/nuwax-motion.conf"
    else
      rm -f "/usr/lib/${kind}.d/nuwax-motion.conf"
    fi
  done
  systemctl daemon-reload
  while IFS=$'\t' read -r unit enabled active; do
    case "${enabled}" in
      enabled|enabled-runtime|linked|linked-runtime|alias) systemctl enable "${unit}" ;;
      *) systemctl disable "${unit}" || true ;;
    esac
  done < "${SYSTEMD_BACKUP_DIR}/service-state.tsv"
  while IFS=$'\t' read -r unit enabled active; do
    if [[ "${active}" == "active" ]]; then
      systemctl restart "${unit}"
    else
      systemctl stop "${unit}" || true
    fi
  done < "${SYSTEMD_BACKUP_DIR}/service-state.tsv"
}

install_actuator_manifest() {
  require_actuator_manifest
  [[ ! -L "${INSTALLED_ACTUATOR_SYMBOLS_FILE}" ]] \
    || fail "refusing actuator symbols manifest symlink: ${INSTALLED_ACTUATOR_SYMBOLS_FILE}"
  if [[ -e "${INSTALLED_ACTUATOR_SYMBOLS_FILE}" ]]; then
    cmp -s "${ACTUATOR_SYMBOLS_FILE}" "${INSTALLED_ACTUATOR_SYMBOLS_FILE}" \
      || fail "refusing to overwrite installed actuator symbols manifest with different content"
  else
    install -D -o root -g nuwax-motion-clients -m 0640 \
      "${ACTUATOR_SYMBOLS_FILE}" "${INSTALLED_ACTUATOR_SYMBOLS_FILE}"
  fi
  chown root:nuwax-motion-clients "${INSTALLED_ACTUATOR_SYMBOLS_FILE}"
  chmod 0640 "${INSTALLED_ACTUATOR_SYMBOLS_FILE}"
  check_installed_actuator_manifest
}

check_service_access() {
  local user=""
  for user in nuwax-motion-guard nuwax-navigation nuwax-geometry nuwax-gateway; do
    runuser -u "${user}" -- test -x "${CONFIG_DIR}" \
      || fail "${user} cannot traverse runtime config directory ${CONFIG_DIR}"
    runuser -u "${user}" -- test -r "${NUWAX_ENV_FILE}" \
      || fail "${user} cannot read runtime config ${NUWAX_ENV_FILE}"
  done
  runuser -u nuwax-motion-guard -- test -r "${GO2_DATA_PLANE_ENV_FILE}" \
    || fail "nuwax-motion-guard cannot read ${GO2_DATA_PLANE_ENV_FILE}"
  runuser -u nuwax-gateway -- test -r "${GO2_DATA_PLANE_ENV_FILE}" \
    || fail "nuwax-gateway cannot read ${GO2_DATA_PLANE_ENV_FILE}"
  runuser -u nuwax-motion-guard -- test -x "${REPO_ROOT}" \
    || fail "nuwax-motion-guard cannot traverse ${REPO_ROOT}"
  runuser -u nuwax-motion-guard -- test -r "${GUARD_MODULE}" \
    || fail "nuwax-motion-guard cannot read ${GUARD_MODULE}"
  runuser -u nuwax-motion-guard -- test -x "${PYTHON_BIN}" \
    || fail "nuwax-motion-guard cannot execute ${PYTHON_BIN}"
  runuser -u nuwax-motion-guard -- test -x "${SDK_PATH}" \
    || fail "nuwax-motion-guard cannot traverse ${SDK_PATH}"
  runuser -u nuwax-motion-guard -- env \
    NUWAX_REPO_ROOT_OVERRIDE="${REPO_ROOT}" \
    NUWAX_GO2_SDK_PATH="${SDK_PATH}" \
    NUWAX_PYTHON_BIN="${PYTHON_BIN}" \
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
  runuser -u nuwax-gateway -- test -x "${PYTHON_BIN}" \
    || fail "nuwax-gateway cannot execute ${PYTHON_BIN}"
  runuser -u nuwax-gateway -- test -r "${REPO_ROOT}/scripts/start_gateway.sh" \
    || fail "nuwax-gateway cannot read start_gateway.sh"
  runuser -u nuwax-gateway -- test -r "${REPO_ROOT}/drivers/go2/video_streamer.py" \
    || fail "nuwax-gateway cannot read the Go2 video streamer"
  check_identity_write_access nuwax-geometry /var/lib/nuwax_robot_bridge
  for directory in "${GATEWAY_WRITE_DIRS[@]}"; do
    check_identity_write_access nuwax-gateway "${directory}"
  done
  runuser -u nuwax-gateway -- test -w /tmp/cdds.LOG \
    || fail "nuwax-gateway cannot write /tmp/cdds.LOG"
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
  local effective_unit=""
  local installed_unit=""
  local unit_file=""
  local unit_name=""
  getent passwd nuwax-motion-guard >/dev/null || fail "missing user nuwax-motion-guard"
  getent passwd nuwax-navigation >/dev/null || fail "missing user nuwax-navigation"
  getent passwd nuwax-geometry >/dev/null || fail "missing user nuwax-geometry"
  getent passwd nuwax-gateway >/dev/null || fail "missing user nuwax-gateway"
  check_service_uid_collisions system
  getent group nuwax-motion-clients >/dev/null || fail "missing group nuwax-motion-clients"
  for user in nuwax-motion-guard nuwax-navigation nuwax-geometry nuwax-gateway; do
    id -nG "${user}" | tr ' ' '\n' | grep -qx nuwax-motion-clients \
      || fail "${user} is not a member of nuwax-motion-clients"
  done
  for group in video render; do
    id -nG nuwax-gateway | tr ' ' '\n' | grep -qx "${group}" \
      || fail "nuwax-gateway is not a member of ${group}"
  done
  check_runtime_config
  check_runtime_config_permissions
  check_runtime_binaries
  check_service_access
  for unit_file in "${DEPLOYMENT_UNITS[@]}"; do
    installed_unit="/etc/systemd/system/${unit_file}"
    [[ -f "${installed_unit}" ]] || fail "missing installed unit: ${installed_unit}"
    ! grep -Eq '@NUWAX_[A-Z0-9_]+@' "${installed_unit}" \
      || fail "unrendered deployment placeholder: ${installed_unit}"
    effective_unit="$(systemctl cat --no-pager "${unit_file}")"
    if [[ "${unit_file}" == *.service ]]; then
      grep -qF "WorkingDirectory=${RENDER_ROOT}" "${installed_unit}" \
        || fail "installed unit does not target ${RENDER_ROOT}: ${installed_unit}"
      grep -qF "WorkingDirectory=${RENDER_ROOT}" <<< "${effective_unit}" \
        || fail "effective systemd unit does not target ${RENDER_ROOT}: ${unit_file}"
    fi
    ! grep -qF "${unit_file}.d/" <<< "${effective_unit}" \
      || fail "unexpected systemd drop-in still overrides ${unit_file}"
  done
  check_installed_actuator_manifest
  "${PROVISIONER}" verify \
    --state-dir "${GUARD_STATE_DIR}" \
    --actuator-symbols-file "${INSTALLED_ACTUATOR_SYMBOLS_FILE}"
  check_record "${GUARD_STATE_DIR}/provisioned.json" 400
  check_record "${GUARD_STATE_DIR}/epoch.json" 600
  systemctl is-enabled --quiet nuwax-host-gateway.service \
    || fail "nuwax-host-gateway.service is not enabled"
  systemctl is-active --quiet nuwax-host-gateway.service \
    || fail "nuwax-host-gateway.service is not active"
  systemctl is-enabled --quiet nuwax-runtime-activation.path \
    || fail "nuwax-runtime-activation.path is not enabled"
  systemctl is-active --quiet nuwax-runtime-activation.path \
    || fail "nuwax-runtime-activation.path is not active"
  systemctl is-enabled --quiet nuwax-go2-video-streamer.service \
    || fail "nuwax-go2-video-streamer.service is not enabled"
  systemctl is-active --quiet nuwax-go2-video-streamer.service \
    || fail "nuwax-go2-video-streamer.service is not active"
  for unit_name in nuwax-motion-safety-guard.service nuwax-nav-stack.service \
                   nuwax-geometry-runtime.service nuwax-person-follow.service \
                   nuwax-runtime-activation.service; do
    ! systemctl is-enabled --quiet "${unit_name}" \
      || fail "${unit_name} must not be enabled for eager boot startup"
  done

  if systemctl is-active --quiet nuwax-motion-safety-guard.service; then
    [[ -S "${GUARD_SOCKET}" ]] || fail "motion guard socket is absent: ${GUARD_SOCKET}"
    [[ -f "${GUARD_LOCK}" && ! -L "${GUARD_LOCK}" ]] \
      || fail "motion guard lock is absent, non-regular, or a symlink: ${GUARD_LOCK}"
    [[ "$(stat -c '%U:%G %a' "${GUARD_LOCK}")" == "nuwax-motion-guard:nuwax-motion-guard 600" ]] \
      || fail "motion guard lock owner/mode drift: ${GUARD_LOCK}"
    NUWAX_MOTION_GUARD_SOCKET="${GUARD_SOCKET}" \
      bash "${REPO_ROOT}/scripts/systemd/wait_for_motion_guard_socket.sh" check
  fi
  if systemctl is-active --quiet nuwax-nav-stack.service; then
    systemctl is-active --quiet nuwax-motion-safety-guard.service \
      || fail "active nav stack requires an active motion guard"
    NUWAX_NAV_STACK_READY_FILE=/run/nuwax/navigation/nuwax_nav_stack.ready \
      bash "${REPO_ROOT}/scripts/systemd/wait_for_nav_stack_ready.sh" check \
      || fail "nuwax-nav-stack ready state is absent or stale"
  fi
  if systemctl is-active --quiet nuwax-geometry-runtime.service; then
    NUWAX_GEOMETRY_RUNTIME_READY_FILE=/run/nuwax/geometry/nuwax_geometry_runtime.ready \
      NUWAX_GEOMETRY_RUNTIME_PORT=50051 \
      bash "${GEOMETRY_READY_HELPER}" check \
      || fail "nuwax-geometry-runtime ready state is absent or stale"
    for path in \
        /var/lib/nuwax_robot_bridge/nuwax_cloudmap.mmap \
        /var/lib/nuwax_robot_bridge/nuwax_occupancy.mmap; do
      [[ -f "${path}" && ! -L "${path}" ]] \
        || fail "geometry shared-memory file is missing or unsafe: ${path}"
      [[ "$(stat -c '%U:%G %a' "${path}")" == "nuwax-geometry:nuwax-motion-clients 660" ]] \
        || fail "geometry shared-memory owner/mode drift: ${path}"
    done
  fi
  if systemctl is-active --quiet nuwax-person-follow.service; then
    systemctl is-active --quiet nuwax-motion-safety-guard.service \
      || fail "active person-follow requires an active motion guard"
    systemctl is-active --quiet nuwax-nav-stack.service \
      || fail "active person-follow requires an active nav stack"
    systemctl is-active --quiet nuwax-geometry-runtime.service \
      || fail "active person-follow requires an active geometry runtime"
    NUWAX_PERSON_FOLLOW_READY_FILE=/run/nuwax/person-follow/nuwax_person_follow.ready \
      bash "${PERSON_FOLLOW_READY_HELPER}" check \
      || fail "nuwax-person-follow ready state is absent or stale"
  fi
  for unit_name in nuwax-host-gateway.service nuwax-motion-safety-guard.service \
                   nuwax-nav-stack.service nuwax-geometry-runtime.service \
                   nuwax-person-follow.service nuwax-go2-video-streamer.service; do
    if systemctl is-active --quiet "${unit_name}"; then
      [[ "$(systemctl show -p NRestarts --value "${unit_name}")" == "0" ]] \
        || fail "${unit_name} restarted during deployment readiness observation"
    fi
  done
  echo "lazy runtime deployment: PASS"
}

install_sources() {
  local sources_changed="false"
  require_root
  check_sources
  check_runtime_config
  check_runtime_binaries
  snapshot_systemd_state
  if systemd_sources_changed; then
    sources_changed="true"
    install -D -m 0644 "${SYSUSERS_SOURCE}" /usr/lib/sysusers.d/nuwax-motion.conf
    install -D -m 0644 "${TMPFILES_SOURCE}" /usr/lib/tmpfiles.d/nuwax-motion.conf
  fi
  systemd-sysusers /usr/lib/sysusers.d/nuwax-motion.conf
  check_service_uid_collisions system
  ensure_gateway_device_groups
  secure_runtime_config_permissions
  systemd-tmpfiles --create /usr/lib/tmpfiles.d/nuwax-motion.conf
  secure_runtime_data_permissions
  bash "${LCM_PREFLIGHT}"
  check_service_access
  if [[ "${sources_changed}" == "true" ]]; then
    remove_legacy_dropins
    render_unit "${GUARD_UNIT_SOURCE}" /etc/systemd/system/nuwax-motion-safety-guard.service
    render_unit "${NAV_UNIT_SOURCE}" /etc/systemd/system/nuwax-nav-stack.service
    render_unit "${GEOMETRY_UNIT_SOURCE}" /etc/systemd/system/nuwax-geometry-runtime.service
    render_unit "${PERSON_FOLLOW_UNIT_SOURCE}" /etc/systemd/system/nuwax-person-follow.service
    render_unit "${RUNTIME_ACTIVATION_UNIT_SOURCE}" /etc/systemd/system/nuwax-runtime-activation.service
    render_unit "${RUNTIME_ACTIVATION_PATH_SOURCE}" /etc/systemd/system/nuwax-runtime-activation.path
    render_unit "${GATEWAY_UNIT_SOURCE}" /etc/systemd/system/nuwax-host-gateway.service
    render_unit "${VIDEO_STREAMER_UNIT_SOURCE}" /etc/systemd/system/nuwax-go2-video-streamer.service
    systemctl daemon-reload
    : > "${SYSTEMD_BACKUP_DIR}/sources.changed"
  else
    rm -f "${SYSTEMD_BACKUP_DIR}/sources.changed"
  fi
  echo "deployment files installed; provision durable records before enabling the stack"
}

enable_stack() {
  require_root
  check_installed_actuator_manifest
  "${PROVISIONER}" verify \
    --state-dir "${GUARD_STATE_DIR}" \
    --actuator-symbols-file "${INSTALLED_ACTUATOR_SYMBOLS_FILE}"
  systemctl disable nuwax-motion-safety-guard.service
  systemctl disable nuwax-nav-stack.service
  systemctl disable nuwax-geometry-runtime.service
  systemctl disable nuwax-person-follow.service
  systemctl disable nuwax-runtime-activation.service
  systemctl enable nuwax-runtime-activation.path
  systemctl enable nuwax-host-gateway.service
  systemctl enable nuwax-go2-video-streamer.service
  echo "enabled base services: Gateway + Go2 video + runtime activation watcher; motion components start on demand"
}

start_stack() {
  require_root
  check_runtime_config
  check_runtime_config_permissions
  check_runtime_binaries
  check_installed_actuator_manifest
  "${PROVISIONER}" verify \
    --state-dir "${GUARD_STATE_DIR}" \
    --actuator-symbols-file "${INSTALLED_ACTUATOR_SYMBOLS_FILE}"
  check_service_uid_collisions system
  check_service_access

  systemctl stop nuwax-go2-video-streamer.service
  systemctl stop nuwax-person-follow.service
  systemctl stop nuwax-geometry-runtime.service
  systemctl stop nuwax-nav-stack.service
  systemctl stop nuwax-motion-safety-guard.service
  rm -f /run/nuwax/gateway/geometry.request /run/nuwax/gateway/geometry.result.json
  systemctl restart nuwax-runtime-activation.path
  systemctl restart nuwax-host-gateway.service
  systemctl restart nuwax-go2-video-streamer.service
  echo "started base services: Gateway + Go2 video + runtime activation watcher; motion components remain idle"
}

stop_stack() {
  require_root
  systemctl stop nuwax-go2-video-streamer.service
  systemctl stop nuwax-host-gateway.service
  systemctl stop nuwax-runtime-activation.path
  systemctl stop nuwax-runtime-activation.service
  systemctl stop nuwax-person-follow.service
  systemctl stop nuwax-geometry-runtime.service
  systemctl stop nuwax-nav-stack.service
  systemctl stop nuwax-motion-safety-guard.service
  echo "stopped stack in reverse dependency order"
}

provision_records() {
  require_root
  install_actuator_manifest
  "${PROVISIONER}" provision \
    --state-dir "${GUARD_STATE_DIR}" \
    --actuator-symbols-file "${INSTALLED_ACTUATOR_SYMBOLS_FILE}"
}

main() {
  case "${ACTION}" in
    check-sources)
      check_sources
      ;;
    snapshot-systemd)
      require_root
      snapshot_systemd_state
      ;;
    restore-systemd)
      require_root
      restore_systemd_state
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
    start)
      start_stack
      ;;
    stop)
      stop_stack
      ;;
    verify-runtime)
      verify_installed
      ;;
    *)
      echo "usage: $0 [check-sources|snapshot-systemd|restore-systemd|install|provision|enable|start|stop|verify-runtime] [actuator-symbols.json]" >&2
      exit 2
      ;;
  esac
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main
fi
