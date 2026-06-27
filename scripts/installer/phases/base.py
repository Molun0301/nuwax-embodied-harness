"""installer phase 抽象基类。

每个 phase 实现：
- `name`：阶段名（小写 + 下划线，与 bootstrap.py 子阶段对齐）；
- `order`：阶段序号（决定执行顺序与 state.json 顺序）；
- `requires`：前置阶段名列表（用于断点续跑时跳过未完成的依赖）；
- `retryable`：失败后是否允许 `--resume` 重试；
- `run(planner, ctx)`：执行入口，副作用通过 `_legacy_bridge.py` 走 bootstrap 纯函数。

详见 plan 4.4 L150-159 草案。
"""

from __future__ import annotations

import abc
from typing import List, Tuple


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
