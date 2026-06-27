#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""preflight phase：阶段 0 — OS / 架构 / Python / 磁盘 / 网络探测。

设计要点：
- 仅探测 + 打印，**不**改系统（plan "check" 语义）。
- 失败检测点（plan RK8）：ROS 发行版探测；不匹配时硬退出。
- UI 交互：纯日志输出（无 questionary / rich 依赖），降级路径同样可用。
- 任务范围：本文件是 task 1 骨架，run/dry_run 占位；task 5（installer-ac-coverage）
  才会把探测逻辑替换为完整实现。
"""

from __future__ import annotations

from typing import List

from scripts.installer.phases.base import Phase


class PreflightPhase(Phase):
    """阶段 0 — 环境探测（plan 4.1 L110）。"""

    name = "preflight"
    order = 0
    requires: tuple = ()
    retryable = False
    dry_run_capable = True

    def run(self, planner, ctx) -> int:
        """task 1 占位：仅打印探测动作；后续 task 5 替换为真实探测。"""
        print(f"[phase: {self.name}] 环境探测（task 1 骨架）")
        return 0

    def dry_run(self, planner, ctx) -> List[str]:
        return [
            "+ uname -m",
            "+ python3 --version",
            "+ . /etc/os-release && echo $ID",
            "+ df -Pk .",
        ]


# 模块导入时注册到 default_registry
from scripts.installer.phases import default_registry  # noqa: E402

default_registry.register(name="preflight", cls=PreflightPhase)