#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""system phase：阶段 1 — apt 源修复 + 基础工具 + ROS2 apt。

设计要点：
- 任务范围是 task 1 骨架，run/dry_run 占位。
- 真实实现由 task 5 (installer-ac-coverage) 替换：调 ``scripts/installer/_legacy_bridge.py``
  间接调用 bootstrap 内部函数（apt 源修复、ROS2 apt 探测）。
- 错误注入（plan 6.11）：``--phase-override system=exit-100`` 触发 AC7 错误输出。
"""

from __future__ import annotations

from typing import List

from scripts.installer.phases.base import Phase


class SystemPhase(Phase):
    """阶段 1 — apt 源 + 基础工具（plan 4.1 L111）。"""

    name = "system"
    order = 1
    requires = ("preflight",)
    retryable = True
    dry_run_capable = True

    def run(self, planner, ctx) -> int:
        print(f"[phase: {self.name}] 系统级 apt 准备（task 1 骨架）")
        return 0

    def dry_run(self, planner, ctx) -> List[str]:
        return [
            "+ sudo apt-get update",
            "+ sudo apt-get install -y build-essential cmake git python3-colcon-common-extensions",
        ]


from scripts.installer.phases import default_registry  # noqa: E402

default_registry.register(name="system", cls=SystemPhase)