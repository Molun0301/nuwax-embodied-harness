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
#
# dotenv 解析（2026-07-06 codex 收口 P3）：
#   原版用 `source ${env_file}` 只能处理 shell-style env 文件（不支持 `KEY=val # comment`
#   行内注释、缺失 quote 处理、不识别 export 前缀）。生成自 installer 的 .env /
#   web/.env / runtime_data/*.env 均为 dotenv 格式（含 `# 注释` 与可选双引号包裹），
#   必须用专用解析器而非 source。本文件采用与 `scripts/start_gateway.sh::load_env_file`
#   完全一致的解析语义（trim / 跳注释 / 跳空行 / 跳无 `=` 行 / 剥成对引号 / 仅在
#   env 未设置时 export），保证两条路径加载同一 env 文件结果一致。
#   若用户显式 export 了同名 env，dotenv 中的值不会覆盖（避免误覆盖 CI 注入）。
set -u

GEOMETRY_RUNTIME_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GEOMETRY_RUNTIME_REPO_ROOT="$(cd "${GEOMETRY_RUNTIME_SCRIPT_DIR}/../.." && pwd)"

trim_whitespace() {
  local value="$1"
  value="${value#"${value%%[![:space:]]*}"}"
  value="${value%"${value##*[![:space:]]}"}"
  printf '%s' "${value}"
}

# dotenv 解析：与 scripts/start_gateway.sh::load_env_file 语义一致。
# 行为：
#   - 跳空行 / 跳 `#` 起头的纯注释行
#   - 必须包含 `=`，否则跳过
#   - key/value 两侧 trim
#   - 若 value 含 " #" 形式的行内注释，先剥掉
#   - 成对单/双引号包裹时去引号
#   - 仅当 env 未设置（`${!key+x}` 为空）时 export，避免覆盖用户显式 export
_geometry_load_env_file() {
  local env_file="$1"
  if [[ ! -f "${env_file}" ]]; then
    return 0
  fi
  local raw_line="" line="" key="" value=""
  while IFS= read -r raw_line || [[ -n "${raw_line}" ]]; do
    line="$(trim_whitespace "${raw_line}")"
    if [[ -z "${line}" || "${line}" == \#* || "${line}" != *=* ]]; then
      continue
    fi
    key="$(trim_whitespace "${line%%=*}")"
    value="$(trim_whitespace "${line#*=}")"
    if [[ "${value}" == *[[:space:]]\#* ]]; then
      value="$(trim_whitespace "${value%%#*}")"
    fi
    if [[ -z "${key}" ]]; then
      continue
    fi
    if [[ ${#value} -ge 2 ]]; then
      if [[ "${value:0:1}" == '"' && "${value: -1}" == '"' ]]; then
        value="${value:1:${#value}-2}"
      elif [[ "${value:0:1}" == "'" && "${value: -1}" == "'" ]]; then
        value="${value:1:${#value}-2}"
      fi
    fi
    if [[ -z "${!key+x}" ]]; then
      export "${key}=${value}"
    fi
  done < "${env_file}"
  echo "==> 已加载 env: ${env_file}"
}

# 1. 根 .env（生成自 installer 的 NUWAX_GEOMETRY_RUNTIME_* / NUWAX_LCM_* 键）
_geometry_load_env_file "${GEOMETRY_RUNTIME_REPO_ROOT}/.env"

# 2. 运行时子模块 env（drivers/go2/data_plane.env 等；可选）
_geometry_load_env_file "${GEOMETRY_RUNTIME_REPO_ROOT}/runtime_data/geometry.env"

# 3. 兼容老路径：runtime_data/.env / web/.env（installer 可写其中之一）
_geometry_load_env_file "${GEOMETRY_RUNTIME_REPO_ROOT}/runtime_data/.env"
_geometry_load_env_file "${GEOMETRY_RUNTIME_REPO_ROOT}/web/.env"

# 4. NUWAX_GEOMETRY_EXTRA_ENV_FILES（逗号分隔）
if [[ -n "${NUWAX_GEOMETRY_EXTRA_ENV_FILES:-}" ]]; then
  IFS=',' read -ra _extra_env_files <<< "${NUWAX_GEOMETRY_EXTRA_ENV_FILES}"
  for env_file in "${_extra_env_files[@]}"; do
    env_file="$(echo "${env_file}" | xargs)"
    _geometry_load_env_file "${env_file}"
  done
fi

unset -f trim_whitespace _geometry_load_env_file