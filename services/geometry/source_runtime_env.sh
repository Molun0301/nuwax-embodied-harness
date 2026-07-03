#!/usr/bin/env bash
# services/geometry/source_runtime_env.sh — 通用 env 文件加载器（去 ROS2 化）
#
# 2026-07-03 drop-ros2：原脚本负责 source ROS2 setup / CycloneDDS lib / Go2SLAM 工作区
# setup，整体 ROS2 接触面已下线；本文件仅保留通用 env 文件加载能力，不再做任何 ROS2
# 专用检查。
#
# 用法：
#   在启动 native geometry runtime 之前 source 本文件：
#     source services/geometry/source_runtime_env.sh
#
#   可选 env：
#   - NUWAX_GEOMETRY_EXTRA_ENV_FILES：逗号分隔的额外 env 文件路径，加载顺序追加。

set -u

GEOMETRY_RUNTIME_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GEOMETRY_RUNTIME_REPO_ROOT="$(cd "${GEOMETRY_RUNTIME_SCRIPT_DIR}/../.." && pwd)"

# 按顺序加载根 .env / 运行时 / robot 专属 env（缺一不致命）
_geometry_load_env_file() {
  local env_file="$1"
  if [[ -f "${env_file}" ]]; then
    # shellcheck disable=SC1090
    set +u
    # shellcheck disable=SC1090
    source "${env_file}"
    set -u
    echo "==> 已加载 env: ${env_file}"
  fi
}

# 1. 根 .env（生成自 installer 的 NUWAX_GEOMETRY_RUNTIME_* / NUWAX_LCM_* 键）
_geometry_load_env_file "${GEOMETRY_RUNTIME_REPO_ROOT}/.env"

# 2. 运行时子模块 env（drivers/go2/data_plane.env 等；可选）
_geometry_load_env_file "${GEOMETRY_RUNTIME_REPO_ROOT}/runtime_data/geometry.env"

# 3. NUWAX_GEOMETRY_EXTRA_ENV_FILES（逗号分隔）
if [[ -n "${NUWAX_GEOMETRY_EXTRA_ENV_FILES:-}" ]]; then
  IFS=',' read -ra _extra_env_files <<< "${NUWAX_GEOMETRY_EXTRA_ENV_FILES}"
  for env_file in "${_extra_env_files[@]}"; do
    env_file="$(echo "${env_file}" | xargs)"
    _geometry_load_env_file "${env_file}"
  done
fi

unset -f _geometry_load_env_file
