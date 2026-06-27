#!/usr/bin/env bash
# 顶层 bash dispatcher：探测环境后调用 python3 install.py。
#
# 设计要点（详见 openspec/changes/installer-redesign 与
# .omc/plans/installer-redesign.md plan 第 4.1 / RK3 / RK16）：
#
# 1. 不实现任何业务逻辑，只负责：
#    - 探测 OS / 架构 / Python / apt 可用性 / 磁盘空间
#    - 实机 python<3.10 时自动 pip install --user questionary rich tomli
#      （一次性 60s 成本，可接受；超时 10s 仍失败则保留仅 apply 能力）
#    - 透传所有 CLI 参数给 python3 install.py "$@"
# 2. 必须避免在脚本中改实机 .venv（installer 自身走 runtime_data/.venv-installer，
#    但首装 wizard 仍需 questionary，install.sh 兜底实机 --user 安装）
# 3. 与 scripts/start_gateway.sh / scripts/bootstrap.py 不冲突，本脚本是调度入口。

set -euo pipefail

# 工作区根：install.sh 必须位于工作区根目录
INSTALL_SH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${INSTALL_SH_DIR}"

PYTHON_BIN="${PYTHON_BIN:-python3}"
LOG_PREFIX="[install.sh]"

# 探测 Python 版本；目标 ≥ 3.10 才走 questionary/rich 完整 UI
PYTHON_MAJOR_MINOR="$(${PYTHON_BIN} -c 'import sys; print("%d.%d" % sys.version_info[:2])' 2>/dev/null || echo "0.0")"
PYTHON_OK=0
case "${PYTHON_MAJOR_MINOR}" in
  3.1[0-9]|3.[2-9][0-9]) PYTHON_OK=1 ;;
esac

if [ "${PYTHON_OK}" -eq 0 ]; then
  echo "${LOG_PREFIX} 探测到 Python ${PYTHON_MAJOR_MINOR}，目标 ≥ 3.10。" >&2
  echo "${LOG_PREFIX} 尝试 pip install --user questionary rich tomli（限时 60s）..." >&2
  if timeout 60s ${PYTHON_BIN} -m pip install --user --quiet questionary rich tomli 2>/dev/null; then
    POST_PYTHON_MAJOR_MINOR="$(${PYTHON_BIN} -c 'import sys; print("%d.%d" % sys.version_info[:2])' 2>/dev/null || echo "0.0")"
    case "${POST_PYTHON_MAJOR_MINOR}" in
      3.1[0-9]|3.[2-9][0-9]) PYTHON_OK=1 ;;
    esac
  fi
  if [ "${PYTHON_OK}" -eq 0 ]; then
    echo "${LOG_PREFIX} 警告: 仍无法获取 ≥3.10 Python；wizard / 交互 UI 不可用，仅保留 apply。" >&2
    echo "${LOG_PREFIX} 提示: 手动安装: ${PYTHON_BIN} -m pip install --user 'questionary>=2.0' 'rich>=13.0' 'tomli>=2.0'" >&2
  fi
fi

# 探测架构与 OS（仅记录日志，不做阻断）
ARCH="$(uname -m 2>/dev/null || echo unknown)"
OS_ID="$(. /etc/os-release 2>/dev/null && echo "${ID:-unknown}" || echo unknown)"
echo "${LOG_PREFIX} 环境: arch=${ARCH} os=${OS_ID} python=${PYTHON_MAJOR_MINOR}"

# 探测磁盘空间（≥ 5GB 才允许继续）
AVAIL_KB="$(df -Pk "${INSTALL_SH_DIR}" 2>/dev/null | awk 'NR==2 {print $4}' || echo 0)"
AVAIL_GB=$((AVAIL_KB / 1024 / 1024))
if [ "${AVAIL_GB}" -lt 5 ]; then
  echo "${LOG_PREFIX} 警告: 剩余磁盘空间 ${AVAIL_GB}GB < 5GB，YOLO-World 模型下载可能失败。" >&2
fi

# 探测 apt 是否可用（仅信息）
if command -v apt-get >/dev/null 2>&1; then
  APT_STATE="ok"
else
  APT_STATE="missing"
  echo "${LOG_PREFIX} 提示: 未找到 apt-get，system / geometry_deps 等阶段可能需要手工安装系统依赖。" >&2
fi

# 创建 runtime_data 目录（state / venv 落点）
mkdir -p "${INSTALL_SH_DIR}/runtime_data"

# 透传所有参数
exec "${PYTHON_BIN}" "${INSTALL_SH_DIR}/install.py" "$@"