#!/usr/bin/env bash
# 分层依赖治理静态检查（01 §3.2 规则 R1-R9 的可机器检查 grep 版）。
#
# 用途：锁住 01-P0-1/P0-2/P0-3 解环成果，防止反向/越层依赖回退。
# 范围排除：__pycache__、*/ros2_ws/*、vendor、.claude/worktrees。
# 退出码：0=全部通过；1=存在违规（逐条打印命中行）。
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

VIOLATIONS=0

# 统一排除噪声路径。
EXCLUDES='--include=*.py'
PRUNE='ros2_ws|/vendor/|__pycache__|\.claude/worktrees'

# scan <规则号> <说明> <目录> <ERE 模式>
# 命中即违规：打印命中行并计数。
scan() {
    local rule="$1" desc="$2" dir="$3" pattern="$4"
    if [ ! -d "${dir}" ]; then
        return
    fi
    local hits
    hits="$(grep -rnE "${pattern}" "${dir}" ${EXCLUDES} 2>/dev/null | grep -vE "${PRUNE}")"
    if [ -n "${hits}" ]; then
        echo "✗ [${rule}] ${desc}"
        echo "${hits}" | sed 's/^/    /'
        VIOLATIONS=$((VIOLATIONS + 1))
    else
        echo "✓ [${rule}] ${desc}"
    fi
}

echo "== 分层依赖治理检查（R1-R9）=="

# R1: contracts 不得 import 任何顶层包（contracts 为最底层，仅允许 stdlib/第三方）。
scan "R1" "contracts 不得 import 上层包" \
    "contracts" \
    "^\s*(from|import) (gateways|services|adapters|providers|drivers|core|skills)(\.|\s|$)"

# R2: providers/adapters/core 只准 import contracts（不得 import 彼此之上或同级业务层）。
scan "R2a" "providers 只准依赖 contracts" \
    "providers" \
    "^\s*(from|import) (gateways|services|adapters|drivers|skills)(\.|\s|$)"
scan "R2b" "adapters 只准依赖 contracts" \
    "adapters" \
    "^\s*(from|import) (gateways|services|providers|drivers|core|skills)(\.|\s|$)"
scan "R2c" "core 只准依赖 contracts" \
    "core" \
    "^\s*(from|import) (gateways|services|adapters|providers|drivers|skills)(\.|\s|$)"

# R3: services 禁止 import gateways、drivers（01-P0-1/P0-2/P0-3 核心解环成果）。
scan "R3" "services 禁止 import gateways/drivers" \
    "services" \
    "^\s*(from|import) (gateways|drivers)(\.|\s|$)"

# R4: services 内部禁止 import skills；skills 禁止 import gateways。
scan "R4a" "services 禁止 import skills" \
    "services" \
    "^\s*(from|import) skills(\.|\s|$)"
scan "R4b" "skills 禁止 import gateways" \
    "skills" \
    "^\s*(from|import) gateways(\.|\s|$)"

# R5: drivers 禁止 import services、gateways、skills（01-P0-3 解环成果）。
scan "R5" "drivers 禁止 import services/gateways/skills" \
    "drivers" \
    "^\s*(from|import) (services|gateways|skills)(\.|\s|$)"

# R9: gateways 中 import drivers 仅允许 drivers.entry / drivers.common，
#     禁止直接 import drivers.go2 / drivers.g1 / drivers.customrobot（网关去 Go2 化成果）。
#     正则精确匹配 drivers.(go2|g1|customrobot)；drivers.entry / drivers.common 不被命中，
#     天然放行。01-P1-1 网关去 Go2 直连已完成，代码侧零违规，故由 advisory 提升为硬规则。
scan "R9" "gateways 禁直连具体机器人驱动 drivers.go2/g1/customrobot" \
    "gateways" \
    "^\s*(from|import) drivers\.(go2|g1|customrobot)(\.|\s|$)"

echo
echo "-- 以下为人工复核类规则（不计入硬失败，仅提示）--"

# R6: from settings import 仅允许出现在 gateways/、drivers/、scripts/ 与 TYPE_CHECKING 块。
#     此处仅列出 services 内的 settings 模块级 import，供人工复核（TYPE_CHECKING 内合规）。
echo "[R6] services 内 settings import（应仅在 TYPE_CHECKING 块）："
grep -rnE "^\s*(from settings import|import settings)(\s|$)" services ${EXCLUDES} 2>/dev/null \
    | grep -vE "${PRUNE}" | sed 's/^/    /' || true

# R7: 禁止跨模块导入下划线私有符号。
echo "[R7] 跨模块导入下划线私有符号（人工复核）："
grep -rnE "^\s*from [a-z_.]+ import .*(\s|,)_[a-z]" services gateways ${EXCLUDES} 2>/dev/null \
    | grep -vE "${PRUNE}" | grep -vE "from \.\s" | sed 's/^/    /' || true

# R8: services/gateways 禁止裸 frame 字符串字面量作为 frame_id 赋值/构造参数（治理规范 §4.1）。
#     单一出口：动态链 frame 读 GeometryFrameConfig（contracts/geometry_runtime.py 的
#     DEFAULT_MAP_FRAME / DEFAULT_ODOM_FRAME / DEFAULT_BASE_FRAME 常量与 frames 默认值），
#     机器人侧传感器 frame 读 assembly.defaults.frame_ids。
#     pattern 仅匹配 `frame_id = "..."` / `frame_id="..."`（含 `*.header.frame_id = "..."`）；
#     `frame_id=str(... or "map")` 这类 config 读取+兜底或经 frame_semantics 判等的读取比较处不匹配。
#     白名单（允许定义 frame 字面量的单一出口，均位于 contracts/，本扫描范围 services/gateways 之外）：
#       - contracts/frame_semantics.py 角色别名表
#       - contracts/geometry_runtime.py GeometryFrameConfig 默认值与 DEFAULT_*_FRAME 常量
R8_WHITELIST='contracts/frame_semantics\.py|contracts/geometry_runtime\.py'
R8_HITS="$(grep -rnE 'frame_id\s*=\s*"(map|odom|base_link)"' services gateways ${EXCLUDES} 2>/dev/null \
    | grep -vE "${PRUNE}" | grep -vE "${R8_WHITELIST}")"
if [ -n "${R8_HITS}" ]; then
    echo "✗ [R8] services/gateways 裸 frame_id 字面量赋值/构造参数"
    echo "${R8_HITS}" | sed 's/^/    /'
    VIOLATIONS=$((VIOLATIONS + 1))
else
    echo "✓ [R8] services/gateways 裸 frame_id 字面量赋值/构造参数"
fi

echo
if [ "${VIOLATIONS}" -eq 0 ]; then
    echo "== 硬规则全部通过（R1-R5、R8、R9 共 0 违规；R6/R7 为人工复核）=="
    exit 0
fi
echo "== 存在 ${VIOLATIONS} 类硬规则违规，见上 =="
exit 1
