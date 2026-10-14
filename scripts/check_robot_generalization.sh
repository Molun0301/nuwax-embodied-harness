#!/usr/bin/env bash
# Robot 通用化 CI 守门（cross-robot-generalization-foundations G12）。
#
# 三条硬约束：
#  R8  services/ 下禁止出现 `from drivers.X import`（除 drivers.common、drivers.registry、drivers.entry）。
#  R10 services/ 下禁止出现 `drivers.go2` / `drivers.g1` / `drivers.customrobot` / `drivers.raspi_lidar_robot`
#      路径字面量；标定 SSOT 应通过 manifest.sensor_topology_ref 解析。
#  R11 drivers/<robot>/manifest.py 中构造 RobotManifest 时必须含 `robot_profile=...` 字段；
#      缺则 CI 失败并打印 file_path:line_number。
#
# 触发方式：bash scripts/check_robot_generalization.sh；失败退出码 2。

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

EXIT_CODE=0
report_violation() {
    local rule="$1"; shift
    local file="$1"; shift
    local line="$1"; shift
    local match="$1"; shift
    echo "[${rule}] ${file}:${line}: ${match}" >&2
    EXIT_CODE=2
}

# --- R8 -------------------------------------------------------------------
# 扫描 services/ 下 Python 文件，匹配 `from drivers.<something> import`，
# 但允许 drivers.common / drivers.registry / drivers.entry（基础抽象）。
while IFS=: read -r file line match; do
    [ -z "${file}" ] && continue
    case "${match}" in
        drivers\.common*|drivers\.registry*|drivers\.entry*)
            continue
            ;;
    esac
    report_violation "R8" "${file}" "${line}" "${match}"
done < <(grep -RnE "from drivers\.[a-z_]+ import" services/ 2>/dev/null || true)

# --- R10 ------------------------------------------------------------------
# 扫描 services/ 下文件，禁止出现 drivers.go2 / drivers.g1 / drivers.customrobot / drivers.raspi_lidar_robot 字面量。
# 例外：docstring 与 # 注释（grep 排除）。
while IFS=: read -r file line match; do
    [ -z "${file}" ] && continue
    line_text="$(sed -n "${line}p" "${file}" 2>/dev/null || true)"
    trimmed="${line_text#"${line_text%%[![:space:]]*}"}"
    # 排除 docstring 反引号片段 + 注释行 + 显式 noqa 标注
    if [[ "${line_text}" == \`*\` || "${line_text}" =~ \`drivers\. || "${trimmed}" == \#* || "${line_text}" =~ \#[[:space:]]*noqa:[[:space:]]*R10 ]]; then
        continue
    fi
    report_violation "R10" "${file}" "${line}" "${match}"
done < <(grep -RnE "drivers\.(go2|g1|customrobot|raspi_lidar_robot)" services/ 2>/dev/null || true)

# --- R11 ------------------------------------------------------------------
# drivers/<robot>/manifest.py 构造 RobotManifest 时必须传 robot_profile=。
for manifest_file in drivers/*/manifest.py; do
    [ -f "${manifest_file}" ] || continue
    # 跳过 drivers/common/manifest.py（其定义 RobotManifest，不构造实例）。
    if [[ "${manifest_file}" == "drivers/common/manifest.py" ]]; then
        continue
    fi
    if ! grep -qE "robot_profile\s*=\s*RobotProfile\(" "${manifest_file}"; then
        report_violation "R11" "${manifest_file}" "0" "RobotManifest 构造缺 robot_profile 字段"
    fi
done

if [ "${EXIT_CODE}" -eq 0 ]; then
    echo "[check_robot_generalization] OK"
fi

exit "${EXIT_CODE}"