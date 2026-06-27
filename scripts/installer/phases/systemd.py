#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""systemd phase：阶段 7 — systemd 模板渲染。

设计要点（plan 4.1 L117）：
- 模板来源：``deploy/systemd/nuwax-host-gateway.service.example``。
- task 1 骨架：run/dry_run 占位；后续 task 5 替换为
  ``_legacy_bridge.call_write_systemd_example`` 调用（bootstrap 内部 L1336）。
- 默认 disabled（``planner.systemd = False``）；wizard 可显式启用。
"""

from __future__ import annotations

from typing import List

from scripts.installer.phases.base import Phase


class SystemdPhase(Phase):
    """阶段 7 — systemd 示例渲染（plan 4.1 L117）。"""

    name = "systemd"
    order = 7
    requires = ("config_env",)
    retryable = True
    dry_run_capable = True

    def run(self, planner, ctx) -> int:
        print(f"[phase: {self.name}] 渲染 systemd 模板（task 1 骨架）")
        return 0

    def dry_run(self, planner, ctx) -> List[str]:
        return [
            "+ render deploy/systemd/nuwax-host-gateway.service from .example",
        ]


from scripts.installer.phases import default_registry  # noqa: E402

default_registry.register(name="systemd", cls=SystemdPhase)