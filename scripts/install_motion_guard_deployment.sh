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
SETTINGS_APPLY_UNIT_SOURCE="${REPO_ROOT}/deploy/systemd/nuwax-settings-apply.service"
SETTINGS_APPLY_PATH_SOURCE="${REPO_ROOT}/deploy/systemd/nuwax-settings-apply.path"
GATEWAY_UNIT_SOURCE="${REPO_ROOT}/deploy/systemd/nuwax-host-gateway.service"
VIDEO_STREAMER_UNIT_SOURCE="${REPO_ROOT}/drivers/go2/systemd/nuwax-go2-video-streamer.service"
GUARD_MODULE="${REPO_ROOT}/services/navigation/motion_safety_guard/__main__.py"
PROVISIONER="${REPO_ROOT}/scripts/provision_motion_guard.py"
LCM_NETWORK_UNIT_SOURCE="${REPO_ROOT}/deploy/systemd/nuwax-lcm-network.service"
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
PROJECT_USER=nuwax
PROJECT_GROUP=nuwax
LEGACY_USERS=(
  nuwax-motion-guard
  nuwax-navigation
  nuwax-geometry
  nuwax-gateway
)
LEGACY_GROUP=nuwax-motion-clients
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
  nuwax-lcm-network.service
  nuwax-motion-safety-guard.service
  nuwax-nav-stack.service
  nuwax-geometry-runtime.service
  nuwax-person-follow.service
  nuwax-runtime-activation.service
  nuwax-runtime-activation.path
  nuwax-settings-apply.service
  nuwax-settings-apply.path
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

systemd_unit_file_state() {
  local unit_name="$1"
  local state=""
  local rc=0
  if state="$(systemctl show -p UnitFileState --value "${unit_name}")"; then
    :
  else
    rc=$?
    fail "cannot read UnitFileState for ${unit_name}: state=${state:-<empty>} rc=${rc}"
  fi
  [[ -n "${state}" ]] \
    || fail "cannot read UnitFileState for ${unit_name}: state=<empty> rc=0"
  printf '%s\n' "${state}"
}

require_always_on_unit_file_state() {
  local unit_name="$1"
  local state=""
  state="$(systemd_unit_file_state "${unit_name}")"
  case "${state}" in
    enabled) ;;
    *) fail "${unit_name} has UnitFileState=${state}; expected enabled" ;;
  esac
}

require_on_demand_unit_file_state() {
  local unit_name="$1"
  local state=""
  state="$(systemd_unit_file_state "${unit_name}")"
  case "${state}" in
    disabled|static|indirect) ;;
    *) fail "${unit_name} has UnitFileState=${state}; expected one of disabled/static/indirect" ;;
  esac
}

apply_systemd_tmpfiles() {
  local config_path="$1"
  local systemd_version="unknown"
  local version_output=""
  local rc=0
  if version_output="$(systemd-tmpfiles --version 2>&1)"; then
    systemd_version="${version_output%%$'\n'*}"
  fi
  printf 'applying systemd-tmpfiles: config=%s systemd_version=%s\n' \
    "${config_path}" "${systemd_version}" >&2
  if systemd-tmpfiles --create "${config_path}"; then
    return 0
  else
    rc=$?
    printf 'ERROR: systemd-tmpfiles failed: config=%s systemd_version=%s rc=%s\n' \
      "${config_path}" "${systemd_version}" "${rc}" >&2
    return "${rc}"
  fi
}

check_project_identity() {
  local source="${1:-system}"
  local rendered_root=""
  if [[ "${source}" == "system" ]]; then
    getent passwd "${PROJECT_USER}" >/dev/null \
      || fail "missing project user ${PROJECT_USER}"
    getent group "${PROJECT_GROUP}" >/dev/null \
      || fail "missing project group ${PROJECT_GROUP}"
    return 0
  fi
  ! grep -Eq 'nuwax-(motion-guard|navigation|geometry|gateway|motion-clients)' "${source}" \
    || fail "legacy multi-user identity remains in sysusers source"
  rendered_root="$(mktemp -d)"
  mkdir -p "${rendered_root}/etc"
  if ! systemd-sysusers --root="${rendered_root}" "${source}" >/dev/null; then
    rm -rf "${rendered_root}"
    fail "cannot render sysusers source: ${source}"
  fi
  awk -F: -v expected="${PROJECT_USER}" '$1 == expected { found=1 } END { exit(found ? 0 : 1) }' \
    "${rendered_root}/etc/passwd" \
    || { rm -rf "${rendered_root}"; fail "sysusers source did not render ${PROJECT_USER}"; }
  awk -F: -v expected="${PROJECT_GROUP}" '$1 == expected { found=1 } END { exit(found ? 0 : 1) }' \
    "${rendered_root}/etc/group" \
    || { rm -rf "${rendered_root}"; fail "sysusers source did not render ${PROJECT_GROUP}"; }
  rm -rf "${rendered_root}"
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
                "${PERSON_FOLLOW_UNIT_SOURCE}" "${LCM_NETWORK_UNIT_SOURCE}" "${RUNTIME_ACTIVATION_UNIT_SOURCE}" \
                "${RUNTIME_ACTIVATION_PATH_SOURCE}" "${SETTINGS_APPLY_UNIT_SOURCE}" "${SETTINGS_APPLY_PATH_SOURCE}" "${GATEWAY_UNIT_SOURCE}" \
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
              "${PERSON_FOLLOW_UNIT_SOURCE}" "${LCM_NETWORK_UNIT_SOURCE}" "${RUNTIME_ACTIVATION_UNIT_SOURCE}" \
              "${GATEWAY_UNIT_SOURCE}" "${VIDEO_STREAMER_UNIT_SOURCE}"; do
    if [[ "${unit}" == "${RUNTIME_ACTIVATION_UNIT_SOURCE}" || "${unit}" == "${LCM_NETWORK_UNIT_SOURCE}" ]]; then
      # 系统服务编排及宿主机网络准备需要特权，不接管普通组件的共享日志目录。
      grep -qx 'User=root' "${unit}" \
        || fail "runtime activator must run as root to manage system services: ${unit}"
      ! grep -q '^LogsDirectory=' "${unit}" \
        || fail "runtime activator must not change shared log directory ownership: ${unit}"
    else
      grep -qx 'User=nuwax' "${unit}" \
        || fail "production service must run as nuwax: ${unit}"
    fi
    grep -qx 'Group=nuwax' "${unit}" \
      || fail "production service must run with group nuwax: ${unit}"
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
              "${PERSON_FOLLOW_UNIT_SOURCE}" "${LCM_NETWORK_UNIT_SOURCE}" "${RUNTIME_ACTIVATION_UNIT_SOURCE}" \
              "${GATEWAY_UNIT_SOURCE}" "${VIDEO_STREAMER_UNIT_SOURCE}"; do
    grep -q '^ExecStart=/usr/bin/env ' "${unit}" \
      || fail "ExecStart does not force systemd-owned invariants: ${unit}"
    while IFS= read -r command; do
      [[ "${command}" == ExecStartPre=/usr/bin/env\ * \
        || "${command}" == ExecStartPost=/usr/bin/env\ * ]] \
        || fail "systemd shell helper lacks invariant env prefix: ${unit}: ${command}"
    done < <(grep -E '^ExecStart(Pre|Post)=.*\.sh([[:space:]]|$)' "${unit}" || true)
  done

  grep -q '^Environment=NUWAX_NAV_STACK_READY_FILE=/run/nuwax/navigation/nuwax_nav_stack.ready' \
    "${NAV_UNIT_SOURCE}" \
    || fail "nav ExecStart does not own the production ready path"
  grep -qF 'NUWAX_NAV_STACK_BINARY=@NUWAX_BIN_DIR@/nuwax-nav-stack' \
    "${NAV_UNIT_SOURCE}" \
    || fail "nav ExecStart does not pin the installed release binary"
  grep -qF 'NUWAX_GEOMETRY_RUNTIME_BINARY=@NUWAX_BIN_DIR@/nuwax-geometry-runtime' \
    "${GEOMETRY_UNIT_SOURCE}" \
    || fail "geometry ExecStart does not pin the canonical runtime binary"
  grep -qF 'NUWAX_PERSON_FOLLOW_BINARY=@NUWAX_BIN_DIR@/nuwax-person-follow' \
    "${PERSON_FOLLOW_UNIT_SOURCE}" \
    || fail "person-follow ExecStart does not pin the installed release binary"
  for unit in "${NAV_UNIT_SOURCE}" "${GEOMETRY_UNIT_SOURCE}" \
              "${PERSON_FOLLOW_UNIT_SOURCE}" "${GATEWAY_UNIT_SOURCE}"; do
    grep -q '^Environment=NUWAX_LCM_BACKEND=real-lcm' \
      "${unit}" \
      || fail "production service lacks the real-lcm default: ${unit}"
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
  grep -qF 'SupplementaryGroups=video render' "${GATEWAY_UNIT_SOURCE}" \
    || fail "Gateway unit must include Jetson video/render device groups"
  grep -qF 'SupplementaryGroups=video render' "${VIDEO_STREAMER_UNIT_SOURCE}" \
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
      'NUWAX_LCM_CHANNEL_GOAL_POSE=/nav/goal#nuwax_msgs.GoalEnvelope' \
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
  check_file "${LCM_NETWORK_UNIT_SOURCE}"
  check_file "${REPO_ROOT}/scripts/systemd/prepare_lcm_network.py"
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
  check_project_identity "${SYSUSERS_SOURCE}"
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
  for unit in "${NAV_UNIT_SOURCE}" "${PERSON_FOLLOW_UNIT_SOURCE}"; do
    grep -qF '@NUWAX_BIN_DIR@' "${unit}" \
      || fail "Rust service source is not a release bin template: ${unit}"
  done
  grep -qF '@NUWAX_BIN_DIR@/nuwax-geometry-runtime' \
    "${GEOMETRY_UNIT_SOURCE}" \
    || fail "Geometry service source does not pin the canonical runtime binary"
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
    [[ "$(stat -c '%U:%G %a' "${path}")" == "root:nuwax 640" ]] \
      || fail "runtime config must be root:nuwax 0640: ${path}"
  done
}

secure_runtime_config_permissions() {
  local path=""
  check_runtime_config
  for path in "${NUWAX_ENV_FILE}" "${GO2_DATA_PLANE_ENV_FILE}"; do
    chown root:nuwax "${path}"
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
  [[ "$(stat -c '%U:%G %a' "${INSTALLED_ACTUATOR_SYMBOLS_FILE}")" == "root:nuwax 640" ]] \
    || fail "installed actuator symbols manifest must be root:nuwax 0640"
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
    usermod -a -G "${group}" "${PROJECT_USER}"
  done
}

legacy_identity_present() {
  local user=""
  for user in "${LEGACY_USERS[@]}"; do
    getent passwd "${user}" >/dev/null && return 0
  done
  getent group "${LEGACY_GROUP}" >/dev/null && return 0
  return 1
}

stop_legacy_services() {
  local unit=""
  local owner=""
  local user=""
  for unit in "${DEPLOYMENT_UNITS[@]}"; do
    owner="$(systemctl show -p User --value "${unit}" 2>/dev/null || true)"
    for user in "${LEGACY_USERS[@]}"; do
      if [[ "${owner}" == "${user}" ]] && systemctl is-active --quiet "${unit}"; then
        systemctl stop "${unit}"
        break
      fi
    done
  done
}

migrate_legacy_ownership() {
  local path=""
  for path in /run/nuwax /var/lib/nuwax /var/lib/nuwax_robot_bridge /var/log/nuwax; do
    [[ -e "${path}" && ! -L "${path}" ]] || continue
    find -P "${path}" -xdev -exec chown -h "${PROJECT_USER}:${PROJECT_GROUP}" {} +
  done
}

remove_legacy_identities() {
  local user=""
  local pids=""
  local members=""
  for user in "${LEGACY_USERS[@]}"; do
    if getent passwd "${user}" >/dev/null; then
      pids="$(pgrep -u "${user}" || true)"
      [[ -z "${pids}" ]] || fail "legacy identity ${user} still owns running processes: ${pids}"
      userdel "${user}" || fail "cannot remove legacy identity ${user}"
    fi
  done
  if getent group "${LEGACY_GROUP}" >/dev/null; then
    members="$(getent group "${LEGACY_GROUP}" | awk -F: '{print $4}')"
    [[ -z "${members}" ]] || fail "legacy group ${LEGACY_GROUP} still has members: ${members}"
    groupdel "${LEGACY_GROUP}" || fail "cannot remove legacy group ${LEGACY_GROUP}"
  fi
  for user in "${LEGACY_USERS[@]}"; do
    ! getent passwd "${user}" >/dev/null || fail "legacy identity remains in NSS: ${user}"
  done
  ! getent group "${LEGACY_GROUP}" >/dev/null \
    || fail "legacy group remains in NSS: ${LEGACY_GROUP}"
}

assert_legacy_identities_absent() {
  local user=""
  for user in "${LEGACY_USERS[@]}"; do
    ! getent passwd "${user}" >/dev/null \
      || fail "legacy identity remains in NSS: ${user}"
  done
  ! getent group "${LEGACY_GROUP}" >/dev/null \
    || fail "legacy group remains in NSS: ${LEGACY_GROUP}"
}

secure_runtime_data_permissions() {
  local directory=""
  local unsafe=""
  for directory in "${GATEWAY_WRITE_DIRS[@]}"; do
    unsafe="$(find -P "${directory}" -xdev ! -type d ! -type f -print -quit)"
    [[ -z "${unsafe}" ]] \
      || fail "gateway writable tree contains a link or special file: ${unsafe}"
    chown -hR "${PROJECT_USER}:${PROJECT_GROUP}" "${directory}"
    find "${directory}" -xdev -type d -exec chmod 0750 {} +
    find "${directory}" -xdev -type f -exec chmod 0640 {} +
  done
  chown "${PROJECT_USER}:${PROJECT_GROUP}" /var/lib/nuwax_robot_bridge
  chmod 2770 /var/lib/nuwax_robot_bridge
  [[ -f /tmp/cdds.LOG && ! -L /tmp/cdds.LOG ]] \
    || fail "/tmp/cdds.LOG must be a regular non-symlink created by systemd-tmpfiles"
  [[ "$(stat -c '%U:%G:%a' /tmp/cdds.LOG)" == "nuwax:nuwax:660" ]] \
    || fail "/tmp/cdds.LOG must be nuwax:nuwax 0660"
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
                "${PERSON_FOLLOW_UNIT_SOURCE}" "${LCM_NETWORK_UNIT_SOURCE}" "${RUNTIME_ACTIVATION_UNIT_SOURCE}" \
                "${RUNTIME_ACTIVATION_PATH_SOURCE}" "${SETTINGS_APPLY_UNIT_SOURCE}" "${SETTINGS_APPLY_PATH_SOURCE}" "${GATEWAY_UNIT_SOURCE}" \
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
  local index state restore_lcm=false
  # 回滚可能跨越尚无网络准备单元的旧版本，先收回当前活动或启动中的消费者。
  for (( index=${#DEPLOYMENT_UNITS[@]}-1; index>=0; index-- )); do
    unit="${DEPLOYMENT_UNITS[index]}"
    state="$(systemctl show -p ActiveState --value "${unit}" 2>/dev/null || true)"
    case "${state}" in active|activating|reloading|deactivating) systemctl stop "${unit}" ;; esac
  done
  while IFS=$'\t' read -r unit enabled active; do
    if [[ "${active}" == active ]]; then
      case "${unit}" in
        nuwax-lcm-network.service|nuwax-host-gateway.service|nuwax-geometry-runtime.service|nuwax-nav-stack.service|nuwax-person-follow.service) restore_lcm=true ;;
      esac
    fi
  done < "${SYSTEMD_BACKUP_DIR}/service-state.tsv"
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
  if [[ "${restore_lcm}" == true ]]; then
    # 使用当前安装器的解析器读取已恢复配置；旧发布可以没有这项新工具。
    /usr/bin/python3 -I -S "${DEFAULT_REPO_ROOT}/scripts/systemd/prepare_lcm_network.py" --config-dir "${CONFIG_DIR}"
  fi
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
    install -D -o root -g nuwax -m 0640 \
      "${ACTUATOR_SYMBOLS_FILE}" "${INSTALLED_ACTUATOR_SYMBOLS_FILE}"
  fi
  chown root:nuwax "${INSTALLED_ACTUATOR_SYMBOLS_FILE}"
  chmod 0640 "${INSTALLED_ACTUATOR_SYMBOLS_FILE}"
  check_installed_actuator_manifest
}

check_service_access() {
  local user="${PROJECT_USER}"
  runuser -u "${user}" -- test -x "${CONFIG_DIR}" \
    || fail "${user} cannot traverse runtime config directory ${CONFIG_DIR}"
  runuser -u "${user}" -- test -r "${NUWAX_ENV_FILE}" \
    || fail "${user} cannot read runtime config ${NUWAX_ENV_FILE}"
  runuser -u "${user}" -- test -r "${GO2_DATA_PLANE_ENV_FILE}" \
    || fail "${user} cannot read ${GO2_DATA_PLANE_ENV_FILE}"
  runuser -u "${user}" -- test -x "${REPO_ROOT}" \
    || fail "${user} cannot traverse ${REPO_ROOT}"
  runuser -u "${user}" -- test -r "${GUARD_MODULE}" \
    || fail "${user} cannot read ${GUARD_MODULE}"
  runuser -u "${user}" -- test -x "${PYTHON_BIN}" \
    || fail "${user} cannot execute ${PYTHON_BIN}"
  runuser -u "${user}" -- test -x "${SDK_PATH}" \
    || fail "${user} cannot traverse ${SDK_PATH}"
  runuser -u "${user}" -- env \
    NUWAX_REPO_ROOT_OVERRIDE="${REPO_ROOT}" \
    NUWAX_GO2_SDK_PATH="${SDK_PATH}" \
    NUWAX_PYTHON_BIN="${PYTHON_BIN}" \
    PYTHONPATH="${REPO_ROOT}:${SDK_PATH}" \
    bash "${REPO_ROOT}/scripts/systemd/check_motion_guard_python.sh" \
    || fail "${user} cannot import protobuf/SDK/production module"
  runuser -u "${user}" -- test -x "${NAV_BINARY}" \
    || fail "${user} cannot traverse/read/execute ${NAV_BINARY}"
  runuser -u "${user}" -- test -r "${REPO_ROOT}/scripts/systemd/start_nav_stack.sh" \
    || fail "${user} cannot read start_nav_stack.sh"
  runuser -u "${user}" -- test -r "${GEOMETRY_LAUNCHER}" \
    || fail "${user} cannot read start_geometry_runtime.sh"
  for path in "${REPO_ROOT}/scripts/systemd/wait_for_nav_stack_ready.sh" "${PROCESS_IDENTITY_HELPER}" "${GEOMETRY_READY_HELPER}"; do
    runuser -u "${user}" -- test -r "${path}" \
      || fail "${user} cannot read ${path}"
  done
  if person_follow_autostart_enabled; then
    runuser -u "${user}" -- test -x "${PERSON_FOLLOW_BINARY}" \
      || fail "${user} cannot traverse/read/execute ${PERSON_FOLLOW_BINARY}"
    runuser -u "${user}" -- test -r "${PERSON_FOLLOW_LAUNCHER}" \
      || fail "${user} cannot read start_person_follow.sh"
    runuser -u "${user}" -- test -r "${PERSON_FOLLOW_READY_HELPER}" \
      || fail "${user} cannot read wait_for_person_follow_ready.sh"
  fi
  runuser -u "${user}" -- test -x "${GEOMETRY_BINARY}" \
    || fail "${user} cannot traverse/read/execute ${GEOMETRY_BINARY}"
  runuser -u "${user}" -- test -r "${REPO_ROOT}/scripts/start_gateway.sh" \
    || fail "${user} cannot read start_gateway.sh"
  runuser -u "${user}" -- test -r "${REPO_ROOT}/drivers/go2/video_streamer.py" \
    || fail "${user} cannot read the Go2 video streamer"
  check_identity_write_access "${user}" /var/lib/nuwax_robot_bridge
  for directory in "${GATEWAY_WRITE_DIRS[@]}"; do
    check_identity_write_access "${user}" "${directory}"
  done
  runuser -u "${user}" -- test -w /tmp/cdds.LOG \
    || fail "${user} cannot write /tmp/cdds.LOG"
}

check_record() {
  local path="$1"
  local expected_mode="$2"
  local actual_mode=""
  [[ -s "${path}" ]] || fail "missing or empty durable record: ${path}"
  actual_mode="$(stat -c '%a' "${path}")"
  [[ "${actual_mode}" == "${expected_mode}" ]] \
    || fail "invalid mode for ${path}: ${actual_mode}, expected ${expected_mode}"
  [[ "$(stat -c '%U:%G' "${path}")" == "${PROJECT_USER}:${PROJECT_GROUP}" ]] \
    || fail "invalid owner for ${path}"
}

verify_installed() {
  local effective_unit=""
  local installed_unit=""
  local unit_file=""
  local unit_name=""
  check_project_identity system
  for group in video render; do
    id -nG "${PROJECT_USER}" | tr ' ' '\n' | grep -qx "${group}" \
      || fail "${PROJECT_USER} is not a member of ${group}"
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
  require_always_on_unit_file_state nuwax-lcm-network.service
  systemctl is-active --quiet nuwax-lcm-network.service || fail "本机通信网络准备未完成"
  require_always_on_unit_file_state nuwax-host-gateway.service
  systemctl is-active --quiet nuwax-host-gateway.service \
    || fail "nuwax-host-gateway.service is not active"
  require_always_on_unit_file_state nuwax-runtime-activation.path
  systemctl is-active --quiet nuwax-runtime-activation.path \
    || fail "nuwax-runtime-activation.path is not active"
  require_always_on_unit_file_state nuwax-go2-video-streamer.service
  systemctl is-active --quiet nuwax-go2-video-streamer.service \
    || fail "nuwax-go2-video-streamer.service is not active"
  for unit_name in nuwax-motion-safety-guard.service nuwax-nav-stack.service \
                   nuwax-geometry-runtime.service nuwax-person-follow.service \
                   nuwax-runtime-activation.service; do
    require_on_demand_unit_file_state "${unit_name}"
  done

  if systemctl is-active --quiet nuwax-motion-safety-guard.service; then
    [[ -S "${GUARD_SOCKET}" ]] || fail "motion guard socket is absent: ${GUARD_SOCKET}"
    [[ -f "${GUARD_LOCK}" && ! -L "${GUARD_LOCK}" ]] \
      || fail "motion guard lock is absent, non-regular, or a symlink: ${GUARD_LOCK}"
    [[ "$(stat -c '%U:%G %a' "${GUARD_LOCK}")" == "${PROJECT_USER}:${PROJECT_GROUP} 600" ]] \
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
      [[ "$(stat -c '%U:%G %a' "${path}")" == "${PROJECT_USER}:${PROJECT_GROUP} 660" ]] \
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
  check_project_identity system
  stop_legacy_services
  migrate_legacy_ownership
  remove_legacy_identities
  assert_legacy_identities_absent
  ensure_gateway_device_groups
  secure_runtime_config_permissions
  apply_systemd_tmpfiles /usr/lib/tmpfiles.d/nuwax-motion.conf
  bash "${REPO_ROOT}/scripts/configure_realtime_network.sh"
  secure_runtime_data_permissions
  /usr/bin/python3 -I -S "${DEFAULT_REPO_ROOT}/scripts/systemd/prepare_lcm_network.py" --config-dir "${CONFIG_DIR}" --print-url
  check_service_access
  if [[ "${sources_changed}" == "true" ]]; then
    remove_legacy_dropins
    render_unit "${LCM_NETWORK_UNIT_SOURCE}" /etc/systemd/system/nuwax-lcm-network.service
    render_unit "${GUARD_UNIT_SOURCE}" /etc/systemd/system/nuwax-motion-safety-guard.service
    render_unit "${NAV_UNIT_SOURCE}" /etc/systemd/system/nuwax-nav-stack.service
    render_unit "${GEOMETRY_UNIT_SOURCE}" /etc/systemd/system/nuwax-geometry-runtime.service
    render_unit "${PERSON_FOLLOW_UNIT_SOURCE}" /etc/systemd/system/nuwax-person-follow.service
    render_unit "${RUNTIME_ACTIVATION_UNIT_SOURCE}" /etc/systemd/system/nuwax-runtime-activation.service
    render_unit "${RUNTIME_ACTIVATION_PATH_SOURCE}" /etc/systemd/system/nuwax-runtime-activation.path
    render_unit "${SETTINGS_APPLY_UNIT_SOURCE}" /etc/systemd/system/nuwax-settings-apply.service
    render_unit "${SETTINGS_APPLY_PATH_SOURCE}" /etc/systemd/system/nuwax-settings-apply.path
    render_unit "${GATEWAY_UNIT_SOURCE}" /etc/systemd/system/nuwax-host-gateway.service
    render_unit "${VIDEO_STREAMER_UNIT_SOURCE}" /etc/systemd/system/nuwax-go2-video-streamer.service
    systemctl daemon-reload
    : > "${SYSTEMD_BACKUP_DIR}/sources.changed"
  else
    rm -f "${SYSTEMD_BACKUP_DIR}/sources.changed"
  fi
  install -d -o "${PROJECT_USER}" -g "${PROJECT_GROUP}" -m 2770 "${REPO_ROOT}/runtime_data/settings"
  local robot_directory=""
  for robot_directory in go2 g1 customrobot raspi_lidar_robot; do
    install -d -o "${PROJECT_USER}" -g "${PROJECT_GROUP}" -m 2770 "${REPO_ROOT}/drivers/${robot_directory}/configs"
  done
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
  systemctl enable nuwax-lcm-network.service
  systemctl enable nuwax-runtime-activation.path
  systemctl enable nuwax-settings-apply.path
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
  check_project_identity system
  check_service_access

  # 网络选路改变前先沿正常反向顺序收回全部旧通信会话。
  stop_stack
  systemctl restart nuwax-lcm-network.service
  rm -f /run/nuwax/gateway/geometry.request /run/nuwax/gateway/geometry.result.json
  systemctl restart nuwax-runtime-activation.path
  systemctl start nuwax-settings-apply.path
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
  systemctl stop nuwax-lcm-network.service 2>/dev/null \
    || [[ "$(systemctl show -p LoadState --value nuwax-lcm-network.service 2>/dev/null || true)" == not-found ]]
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
