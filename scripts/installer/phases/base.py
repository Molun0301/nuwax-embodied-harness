"""installer phase 抽象基类。

每个 phase 实现：
- `name`：阶段名（小写 + 下划线，与 bootstrap.py 子阶段对齐）；
- `order`：阶段序号（决定执行顺序与 state.json 顺序）；
- `requires`：前置阶段名列表（用于断点续跑时跳过未完成的依赖）；
- `retryable`：失败后是否允许 `--resume` 重试；
- `run(planner, ctx)`：执行入口，副作用通过 `_legacy_bridge.py` 走 bootstrap 纯函数。

【cross-robot-generalization-foundations G7】每个 phase 在 `run()` 入口必须先调
`_is_forbidden_by_robot_profile(planner)`；命中 forbidden_phases 时直接 return 0
并打印 `[SKIP] robot_profile.forbidden_phases`。
"""

from __future__ import annotations

import abc
from typing import List, Optional, Tuple


class Phase(abc.ABC):
    """installer 单阶段抽象类。"""

    name: str = ""
    order: int = 0
    requires: Tuple[str, ...] = ()
    retryable: bool = True

    @abc.abstractmethod
    def run(self, planner, ctx) -> int:
        """执行阶段；返回 0=成功 / 非 0=失败。"""

    def dry_run(self, planner, ctx) -> List[str]:
        """返回 dry-run 模式下的提示行列表（默认空，由 phase 自实现）。"""
        return []

    # ------------------------------------------------------------------
    # 跨机器人通用化：按 robot_profile.forbidden_phases 跳过
    # ------------------------------------------------------------------

    def _is_forbidden_by_robot_profile(self, planner) -> bool:
        """检查当前 phase 是否在 `planner.robot_profile.forbidden_phases` 内。

        返回 True 时调用方应直接 `ctx.info(f\"[SKIP] robot_profile.forbidden_phases\")` 并
        return 0，不执行任何副作用。
        """

        profile = getattr(planner, "robot_profile", None)
        if profile is None:
            return False
        forbidden = tuple(getattr(profile, "forbidden_phases", ()) or ())
        return self.name in forbidden

    def _robot_profile_skip_message(self) -> str:
        return f"[SKIP] {self.name}: robot_profile.forbidden_phases 命中"
