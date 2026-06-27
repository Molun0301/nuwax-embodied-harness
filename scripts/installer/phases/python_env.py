#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""python_env phase：阶段 2 — 创建 .venv + 装 Python 依赖。

设计要点：
- task 1 骨架：run/dry_run 占位；后续 task 5 替换为
  ``_legacy_bridge.call_install_python_dependencies`` 调用。
- 副作用本身走 ``runtime_data/.venv-installer``（RK11 隔离），不污染项目 .venv。
"""

from __future__ import annotations

from typing import List

from scripts.installer.phases.base import Phase


class PythonEnvPhase(Phase):
    """阶段 2 — Python 依赖安装（plan 4.1 L112）。"""

    name = "python_env"
    order = 2
    requires = ("system",)
    retryable = True
    dry_run_capable = True

    def run(self, planner, ctx) -> int:
        print(f"[phase: {self.name}] 创建 .venv 并安装 Python 依赖（task 1 骨架）")
        return 0

    def dry_run(self, planner, ctx) -> List[str]:
        return [
            "+ python3 -m venv --system-site-packages .venv-installer",
            "+ .venv-installer/bin/python -m pip install -r requirements.txt",
        ]


from scripts.installer.phases import default_registry  # noqa: E402

default_registry.register(name="python_env", cls=PythonEnvPhase)