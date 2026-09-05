#!/usr/bin/env bash
# 统一加载单行配置，使用纯标准库解析器；值不会作为 shell 代码执行。
_NUWAX_CONFIG_VALUES_SCRIPT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)/core/config_values.py"

# 首次进入时记录显式进程环境，后续文件可覆盖较早文件但不能覆盖显式环境。
if ! declare -p _NUWAX_CONFIG_INITIAL_KEYS >/dev/null 2>&1; then
  declare -Ag _NUWAX_CONFIG_INITIAL_KEYS=()
  while IFS= read -r _nuwax_env_key; do
    _NUWAX_CONFIG_INITIAL_KEYS["${_nuwax_env_key}"]=1
  done < <(compgen -e)
  unset _nuwax_env_key
fi

load_env_file() {
  local target="$1"
  local allowed_prefix="${2:-}"
  local override_existing="${3:-false}"
  local key="" value=""
  [[ -f "${target}" ]] || return 0
  while IFS= read -r -d '' key && IFS= read -r -d '' value; do
    if [[ -n "${allowed_prefix}" && "${key}" != "${allowed_prefix}"* ]]; then
      continue
    fi
    if [[ "${override_existing}" == "true" || -z "${_NUWAX_CONFIG_INITIAL_KEYS[${key}]+x}" ]]; then
      export "${key}=${value}"
    fi
  done < <(python3 "${_NUWAX_CONFIG_VALUES_SCRIPT}" "${target}")
}

normalize_config_aliases() {
  local key="" value=""
  while IFS= read -r -d '' key && IFS= read -r -d '' value; do
    export "${key}=${value}"
  done < <(python3 "${_NUWAX_CONFIG_VALUES_SCRIPT}" --aliases)
}
