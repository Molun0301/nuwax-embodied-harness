#!/usr/bin/env bash
# 按需管理本机模块，也可进入设备管理与现行软件更新。
set -euo pipefail
umask 022

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3}"
command -v "${PYTHON_BIN}" >/dev/null 2>&1 || {
  echo "未找到 Python（编程语言）解释器；请先安装 python3。" >&2
  exit 127
}
"${PYTHON_BIN}" -c 'import sys; sys.exit("安装器需要 Python 3.8 或更新版本" if sys.version_info < (3, 8) else 0)'
cd "${REPO_ROOT}"
exec "${PYTHON_BIN}" "${REPO_ROOT}/install.py" "$@"
