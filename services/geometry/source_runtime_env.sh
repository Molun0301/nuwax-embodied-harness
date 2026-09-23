#!/usr/bin/env bash
# 原生进程和网关共用设置文件及解析器；显式进程环境优先。
GEOMETRY_RUNTIME_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GEOMETRY_RUNTIME_REPO_ROOT="$(cd "${GEOMETRY_RUNTIME_SCRIPT_DIR}/../.." && pwd)"
source "${GEOMETRY_RUNTIME_REPO_ROOT}/scripts/lib/config_env.sh"

load_platform_config || return 1

# 导航由建图链唯一决定；显式旧覆盖错配时在创建任何算法资源前失败。
NUWAX_NAV_PROFILE="$("${NUWAX_PYTHON_BIN:-python3}" "${GEOMETRY_RUNTIME_REPO_ROOT}/scripts/resolve_navigation_pipeline.py")" || return 1
export NUWAX_NAV_PROFILE

# 规划器与执行器使用同一设备速度上限；此值为启动装配结果，不是另一套标定。
_NUWAX_NAV_ACTUATOR_MAX_LINEAR_MPS="$("${NUWAX_PYTHON_BIN:-python3}" "${GEOMETRY_RUNTIME_REPO_ROOT}/scripts/resolve_navigation_pipeline.py" --max-linear-speed)" || return 1
export _NUWAX_NAV_ACTUATOR_MAX_LINEAR_MPS
