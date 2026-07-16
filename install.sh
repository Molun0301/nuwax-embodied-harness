#!/usr/bin/env bash
set -euo pipefail
umask 022

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMMAND="${1:-}"

if [[ "${EUID}" -eq 0 ]]; then
  PYTHON_BIN="/usr/bin/python3"
else
  PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3}"
fi

if [[ "${EUID}" -eq 0 && "${NUWAX_INSTALLER_CLEAN_ENV:-}" != "1" ]]; then
  INVOKING_UID="${SUDO_UID:-0}"
  INVOKING_GID="${SUDO_GID:-0}"
  INVOKING_USER="${SUDO_USER:-root}"
  [[ "${INVOKING_UID}" =~ ^[0-9]+$ && "${INVOKING_GID}" =~ ^[0-9]+$ ]] || {
    echo "[install.sh] invalid invoking uid/gid environment" >&2
    exit 77
  }
  exec /usr/bin/env -i \
    HOME=/root \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    NUWAX_INSTALLER_CLEAN_ENV=1 \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    SUDO_GID="${INVOKING_GID}" \
    SUDO_UID="${INVOKING_UID}" \
    SUDO_USER="${INVOKING_USER}" \
    "${REPO_ROOT}/install.sh" "$@"
fi

command -v "${PYTHON_BIN}" >/dev/null 2>&1 || {
  echo "[install.sh] python interpreter not found: ${PYTHON_BIN}" >&2
  exit 127
}

"${PYTHON_BIN}" -c '
import sys
if sys.version_info < (3, 8):
    raise SystemExit("nuwax installer requires Python >= 3.8")
'

case "${COMMAND}" in
  check|deploy|resume|verify|status|rollback)
    if [[ "${EUID}" -ne 0 ]]; then
      command -v sudo >/dev/null 2>&1 || {
        echo "[install.sh] ${COMMAND} requires root and sudo is unavailable" >&2
        exit 77
      }
      INVOKING_UID="$(id -u)"
      INVOKING_GID="$(id -g)"
      INVOKING_USER="$(id -un)"
      exec sudo -- /usr/bin/env -i \
        HOME=/root \
        LANG=C.UTF-8 \
        LC_ALL=C.UTF-8 \
        NUWAX_INSTALLER_CLEAN_ENV=1 \
        PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
        SUDO_GID="${INVOKING_GID}" \
        SUDO_UID="${INVOKING_UID}" \
        SUDO_USER="${INVOKING_USER}" \
        "${REPO_ROOT}/install.sh" "$@"
    fi
    ;;
esac

cd "${REPO_ROOT}"
exec "${PYTHON_BIN}" "${REPO_ROOT}/install.py" "$@"
