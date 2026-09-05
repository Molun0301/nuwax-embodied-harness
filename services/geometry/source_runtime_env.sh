#!/usr/bin/env bash
# 原生进程和网关共用设置文件及解析器；显式进程环境优先。
GEOMETRY_RUNTIME_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GEOMETRY_RUNTIME_REPO_ROOT="$(cd "${GEOMETRY_RUNTIME_SCRIPT_DIR}/../.." && pwd)"
source "${GEOMETRY_RUNTIME_REPO_ROOT}/scripts/lib/config_env.sh"

load_env_file "${GEOMETRY_RUNTIME_REPO_ROOT}/.config"
load_env_file "${NUWAX_BOOTSTRAP_ENV_FILE:-${GEOMETRY_RUNTIME_REPO_ROOT}/.env}"

# 额外文件必须显式配置，避免历史 web/.env、runtime_data/.env 悄悄改变行为。
if [[ -n "${NUWAX_GEOMETRY_EXTRA_ENV_FILES:-}" ]]; then
  IFS=',' read -ra _geometry_extra_files <<< "${NUWAX_GEOMETRY_EXTRA_ENV_FILES}"
  for _geometry_extra_file in "${_geometry_extra_files[@]}"; do
    _geometry_extra_file="${_geometry_extra_file#"${_geometry_extra_file%%[![:space:]]*}"}"
    _geometry_extra_file="${_geometry_extra_file%"${_geometry_extra_file##*[![:space:]]}"}"
    load_env_file "${_geometry_extra_file}"
  done
  unset _geometry_extra_files _geometry_extra_file
fi

for _geometry_override in "${NUWAX_GEOMETRY_WEB_CONFIG_ENV_FILE:-}" "${NUWAX_GATEWAY_WEB_CONFIG_ENV_FILE:-}" "${NUWAX_USER_CONFIG_ENV_FILE:-}"; do
  [[ -n "${_geometry_override}" ]] && load_env_file "${_geometry_override}"
done
unset _geometry_override
normalize_config_aliases
