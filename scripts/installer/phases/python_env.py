#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""python_env phase：阶段 2 — 创建 .venv + 装 Python 依赖。

设计要点：
- 调 ``scripts/installer/_legacy_bridge.call_install_python_dependencies`` 触发
  ``bootstrap.install_python_dependencies``（bootstrap.py:1000）。
- 当 ``planner.install_python=False`` 时跳过（cli 的 skip flag 已先判，本函数再次防御）。
- 副作用本身走 ``runtime_data/.venv-installer``（RK11 隔离），不污染项目 .venv。
"""
from __future__ import annotations

from typing import List

from scripts.installer._legacy_bridge import call_install_python_dependencies
from scripts.installer.phases.base import Phase


class PythonEnvPhase(Phase):
    """阶段 2 — Python 依赖安装（plan 4.1 L112）。"""

    name = "python_env"
    order = 2
    requires = ("system",)
    retryable = True
    dry_run_capable = True

    def run(self, planner, ctx) -> int:
        if not bool(getattr(planner, "install_python", True)):
            ctx.info(f"[phase: {self.name}] planner.install_python=False，跳过")
            return 0
        ctx.info(f"[phase: {self.name}] 创建 .venv 并安装 Python 依赖")
        call_install_python_dependencies(planner)
        ctx.success("[phase: {self.name}] Python 依赖安装完成")
        return 0

    def dry_run(self, planner, ctx) -> List[str]:
        if not bool(getattr(planner, "install_python", True)):
            return ["[SKIP] planner.install_python=False"]
        return [
            "+ python3 -m venv --system-site-packages .venv-installer",
            "+ .venv-installer/bin/python -m pip install -r requirements.txt",
            "+ if memory_mode=local: pip install -r requirements-memory-models.txt",
        ]


from scripts.installer.phases import default_registry  # noqa: E402

default_registry.register(name="python_env", cls=PythonEnvPhase)