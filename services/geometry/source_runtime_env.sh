#!/usr/bin/env bash
# 原生进程和网关共用设置文件及解析器；显式进程环境优先。
GEOMETRY_RUNTIME_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GEOMETRY_RUNTIME_REPO_ROOT="$(cd "${GEOMETRY_RUNTIME_SCRIPT_DIR}/../.." && pwd)"
source "${GEOMETRY_RUNTIME_REPO_ROOT}/scripts/lib/config_env.sh"

load_platform_config || return 1

# 空白与未设置采用约定默认；非空值原样交给 Rust 启动边界校验。
if [[ "${NUWAX_GEOMETRY_FINE_CLOUD_CACHE_MIB:-}" != *[![:space:]]* ]]; then
  NUWAX_GEOMETRY_FINE_CLOUD_CACHE_MIB=auto
  # 物化的默认也随子进程重读配置；调用者显式空值仍保留环境优先级。
  if [[ -z "${_NUWAX_CONFIG_INITIAL_KEYS[NUWAX_GEOMETRY_FINE_CLOUD_CACHE_MIB]+x}" ]]; then
    export _NUWAX_CONFIG_FILE_KEYS="${_NUWAX_CONFIG_FILE_KEYS:-}NUWAX_GEOMETRY_FINE_CLOUD_CACHE_MIB:"
  fi
fi
if [[ "${NUWAX_GEOMETRY_FINE_CLOUD_CACHE_AUTO_MAX_MIB:-}" != *[![:space:]]* ]]; then
  NUWAX_GEOMETRY_FINE_CLOUD_CACHE_AUTO_MAX_MIB=256
  if [[ -z "${_NUWAX_CONFIG_INITIAL_KEYS[NUWAX_GEOMETRY_FINE_CLOUD_CACHE_AUTO_MAX_MIB]+x}" ]]; then
    export _NUWAX_CONFIG_FILE_KEYS="${_NUWAX_CONFIG_FILE_KEYS:-}NUWAX_GEOMETRY_FINE_CLOUD_CACHE_AUTO_MAX_MIB:"
  fi
fi
export NUWAX_GEOMETRY_FINE_CLOUD_CACHE_MIB NUWAX_GEOMETRY_FINE_CLOUD_CACHE_AUTO_MAX_MIB

# 导航链、执行器通道和速度上限一次解析；按固定键导出，不执行输出文本。
_NUWAX_NAV_STARTUP_VALUES="$("${NUWAX_PYTHON_BIN:-python3}" "${GEOMETRY_RUNTIME_REPO_ROOT}/scripts/resolve_navigation_pipeline.py" --runtime-env)" || return 1
while IFS='=' read -r _nuwax_nav_key _nuwax_nav_value; do
  case "${_nuwax_nav_key}" in
    NUWAX_NAV_PROFILE|_NUWAX_NAV_ACTUATOR_MAX_LINEAR_MPS|_NUWAX_NAV_ACTUATOR_MAX_YAW_RPS|_NUWAX_NAV_ACTUATOR_MOTION_MODEL)
      export "${_nuwax_nav_key}=${_nuwax_nav_value}" ;;
    *) return 1 ;;
  esac
done <<< "${_NUWAX_NAV_STARTUP_VALUES}"
unset _NUWAX_NAV_STARTUP_VALUES _nuwax_nav_key _nuwax_nav_value
