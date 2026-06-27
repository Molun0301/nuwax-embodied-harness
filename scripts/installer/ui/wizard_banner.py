"""installer wizard 末尾 banner 输出（从原 planner.py 迁出）。

任务范围（task 4 / installer-shared-lock）：

本模块**仅**承担 wizard 完成后的"下一步提示 banner"输出契约（plan 4.5 +
spec.md "health phase 区分 gateway 来源" Scenario + R14）：

    echo "下一步: bash scripts/start_gateway.sh start 启动 gateway"

历史上 task 4 把 banner 写在 ``scripts/installer/planner.py``，但 task 7
（installer-yolo-engine-default）必须在 ``planner.py`` 落地 ``InstallPlanner``
dataclass 以承担 tier→export_yolo_engine 默认值映射。本文件作为拆出后的落点，
保持 banner 契约一字不变（``WIZARD_NEXT_STEP_BANNER`` 字符串相等），便于
task 4 / task 7 各自的回滚开关独立开关。
"""

from __future__ import annotations

# wizard 完成后向用户输出的下一步提示（plan 4.5 / R14 + spec Scenario "wizard 末尾 banner"）。
# 强制写死字符串内容，不允许从配置/i18n 注入（installer 边界内，避免 banner 漂移）。
WIZARD_NEXT_STEP_BANNER: str = (
    "下一步: bash scripts/start_gateway.sh start 启动 gateway"
)


def wizard_next_step_banner() -> str:
    """返回 wizard 完成后的下一步提示字符串。"""
    return WIZARD_NEXT_STEP_BANNER


def print_wizard_next_step_banner() -> None:
    """向 stdout 输出 wizard 下一步提示 banner（行尾换行）。"""
    print(WIZARD_NEXT_STEP_BANNER, flush=True)


__all__ = [
    "WIZARD_NEXT_STEP_BANNER",
    "wizard_next_step_banner",
    "print_wizard_next_step_banner",
]
