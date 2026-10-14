#!/usr/bin/env bash
# 原生进程和网关共用设置文件及解析器；显式进程环境优先。
GEOMETRY_RUNTIME_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GEOMETRY_RUNTIME_REPO_ROOT="$(cd "${GEOMETRY_RUNTIME_SCRIPT_DIR}/../.." && pwd)"
source "${GEOMETRY_RUNTIME_REPO_ROOT}/scripts/lib/config_env.sh"

load_platform_config || return 1

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
