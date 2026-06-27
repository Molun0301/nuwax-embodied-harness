"""installer.ui.progress：多阶段进度条封装（rich.Progress 优先，降级 print）。

设计目标：
- 提供多阶段进度条（preflight / system / python_env / build_ws 等 10+ 阶段）。
- 阶段标签 + 当前步骤描述，遵循 plan 4.1 L123 "多阶段进度条 + 阶段标签"约定。
- rich 不可用时降级为顺序打印 "[1/N] <phase> ... ok"。

用法（rich 模式）：
    with MultiPhaseProgress(total_phases=11) as mp:
        mp.start_phase("preflight", "探测 OS/架构/Python")
        mp.advance_step("检查 apt 源")
        mp.complete_phase()
        mp.start_phase("system", "apt update + 安装基础工具")
        ...

用法（降级模式）：
    内部检测 _HAS_RICH 失败，自动走 print 分支；语义等价。
"""

from __future__ import annotations

import sys
from contextlib import contextmanager
from typing import Iterator, Optional

# 延迟导入 rich；导入失败不抛
try:
    from rich.progress import (
        Progress as _RichProgress,
        BarColumn as _RichBarColumn,
        TextColumn as _RichTextColumn,
        TimeElapsedColumn as _RichTimeElapsedColumn,
        SpinnerColumn as _RichSpinnerColumn,
    )
    _HAS_RICH = True
except ImportError:
    _HAS_RICH = False
    _RichProgress = None  # type: ignore[assignment]
    _RichBarColumn = None  # type: ignore[assignment]
    _RichTextColumn = None  # type: ignore[assignment]
    _RichTimeElapsedColumn = None  # type: ignore[assignment]
    _RichSpinnerColumn = None  # type: ignore[assignment]


def has_rich() -> bool:
    """诊断用：返回 rich 是否成功导入。"""
    return _HAS_RICH


# ---------- 多阶段进度条 ----------

class MultiPhaseProgress:
    """多阶段进度条封装。

    参数：
        total_phases: 总阶段数（plan 4.1 L110-120 共 11 个 phase：preflight 到 health）。

    行为：
        - 维护一个进度条实例，整体完成度 = (已完成阶段数 / total_phases)。
        - 每个阶段内部可以多次 advance_step() 更新当前步骤描述。
    """

    def __init__(self, total_phases: int = 11) -> None:
        self.total_phases = max(1, int(total_phases))
        self._completed = 0
        self._current_phase: Optional[str] = None
        self._rich_progress: Optional[object] = None
        self._rich_task_id: Optional[int] = None
        self._started = False

    def __enter__(self) -> "MultiPhaseProgress":
        if _HAS_RICH:
            # rich.Progress 自定义列：Spinner / [阶段名] / Bar / 步骤描述 / 已耗时
            self._rich_progress = _RichProgress(
                _RichSpinnerColumn(),
                _RichTextColumn("[bold cyan]{task.description}[/bold cyan]"),
                _RichBarColumn(bar_width=30),
                _RichTextColumn("{task.fields[step]}"),
                _RichTimeElapsedColumn(),
                transient=False,
            )
            self._rich_progress.__enter__()
            self._rich_task_id = self._rich_progress.add_task(
                "初始化",
                total=self.total_phases,
                step="",
            )
        else:
            # 降级模式：打印起始横幅
            print(f"[installer] 开始 {self.total_phases} 个阶段...")
        self._started = True
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        if self._rich_progress is not None:
            self._rich_progress.__exit__(exc_type, exc_val, exc_tb)
        else:
            # 降级模式：打印完成总结
            if exc_type is None:
                print(f"[installer] 全部 {self.total_phases} 个阶段完成")

    def start_phase(self, name: str, description: str = "") -> None:
        """标记进入新阶段。

        参数：
            name: 阶段名（如 "preflight" / "build_ws"）
            description: 阶段描述（如 "探测 OS/架构/Python"）
        """
        if not self._started:
            raise RuntimeError("MultiPhaseProgress 必须用 with 上下文使用")
        self._current_phase = name
        if self._rich_progress is not None and self._rich_task_id is not None:
            label = f"[{name}] {description}".strip()
            self._rich_progress.update(
                self._rich_task_id,
                description=label,
                step="",
                completed=self._completed,
            )
        else:
            # 降级模式：序号 + 阶段名 + 描述
            print(f"[{self._completed + 1}/{self.total_phases}] {name}: {description}")

    def advance_step(self, step_desc: str) -> None:
        """更新当前阶段内的子步骤描述（不影响整体进度）。"""
        if self._rich_progress is not None and self._rich_task_id is not None:
            self._rich_progress.update(
                self._rich_task_id,
                step=f"  -> {step_desc}",
            )
        else:
            # 降级模式：缩进打印子步骤
            print(f"    -> {step_desc}")

    def complete_phase(self) -> None:
        """标记当前阶段完成，整体进度 +1。"""
        self._completed += 1
        if self._rich_progress is not None and self._rich_task_id is not None:
            self._rich_progress.update(
                self._rich_task_id,
                completed=self._completed,
                step="[green]ok[/green]",
            )
        else:
            print(f"    [ok] {self._current_phase}")
        self._current_phase = None

    def fail_phase(self, error_msg: str) -> None:
        """标记当前阶段失败（不增进度）。

        参数：
            error_msg: 失败原因简述（详细错误由调用方另行打印）
        """
        if self._rich_progress is not None and self._rich_task_id is not None:
            self._rich_progress.update(
                self._rich_task_id,
                step=f"[red]{error_msg}[/red]",
            )
        else:
            print(f"    [FAIL] {self._current_phase}: {error_msg}", file=sys.stderr)


# ---------- 便捷上下文管理器 ----------

@contextmanager
def phase(name: str, description: str = "", total_phases: int = 11) -> Iterator[MultiPhaseProgress]:
    """单阶段便捷上下文：

        with phase("preflight", "探测环境", total_phases=11) as p:
            p.advance_step("检查 OS")
            ...
    """
    mp = MultiPhaseProgress(total_phases=total_phases)
    with mp:
        mp.start_phase(name, description)
        yield mp
        mp.complete_phase()