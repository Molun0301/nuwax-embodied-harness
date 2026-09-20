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
  # 子进程继承的文件值不属于用户显式覆盖，重新启动后允许读取新方案。
  IFS=':' read -ra _nuwax_file_keys <<< "${_NUWAX_CONFIG_FILE_KEYS:-}"
  for _nuwax_env_key in "${_nuwax_file_keys[@]}"; do
    [[ -n "${_nuwax_env_key}" ]] && unset '_NUWAX_CONFIG_INITIAL_KEYS['"${_nuwax_env_key}"']'
  done
  unset _nuwax_file_keys _nuwax_env_key
fi

load_platform_config() {
  local key="" value="" loaded="" file_keys=""
  local -a old_keys=()
  if [[ "${NUWAX_CONFIG_FROM_SYSTEMD_FILES:-0}" == "1" ]]; then
    while IFS= read -r -d '' key && IFS= read -r -d '' value; do
      unset '_NUWAX_CONFIG_INITIAL_KEYS['"${key}"']'
      unset "${key}"
    done < <(python3 "${_NUWAX_CONFIG_VALUES_SCRIPT}" --systemd-file-keys)
  fi
  IFS=':' read -ra old_keys <<< "${_NUWAX_CONFIG_FILE_KEYS:-}"
  for key in "${old_keys[@]}"; do
    if [[ -n "${key}" && -z "${_NUWAX_CONFIG_INITIAL_KEYS[${key}]+x}" ]]; then
      unset "${key}"
    fi
  done
  while IFS= read -r -d '' key && IFS= read -r -d '' value; do
    if [[ "${key}" == "_NUWAX_CONFIG_LOAD_OK" ]]; then
      loaded=1
    elif [[ -z "${_NUWAX_CONFIG_INITIAL_KEYS[${key}]+x}" ]]; then
      export "${key}=${value}"
      file_keys+="${key}:"
    fi
  done < <(python3 "${_NUWAX_CONFIG_VALUES_SCRIPT}" --merged)
  [[ "${loaded}" == "1" ]] || return 1
  export _NUWAX_CONFIG_FILE_KEYS="${file_keys}"
  normalize_config_aliases
}

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
    export _NUWAX_CONFIG_FILE_KEYS="${_NUWAX_CONFIG_FILE_KEYS:-}${key}:"
  done < <(python3 "${_NUWAX_CONFIG_VALUES_SCRIPT}" --aliases)
}
