#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""systemd phase：阶段 7 — systemd 模板渲染。

设计要点（plan 4.1 L117）：
- 模板来源：``deploy/systemd/nuwax-host-gateway.service.example``。
- 调 ``_legacy_bridge.call_write_systemd_example``（bootstrap.write_systemd_example L1336）。
- 默认 disabled（``planner.systemd = False``）；wizard 可显式启用。
- 当 ``planner.systemd=False`` 时 cli 已先 skip；本函数防御性 return。
"""
from __future__ import annotations

from typing import List

from scripts.installer._legacy_bridge import call_write_systemd_example
from scripts.installer.phases.base import Phase


class SystemdPhase(Phase):
    """阶段 7 — systemd 示例渲染（plan 4.1 L117）。"""

    name = "systemd"
    order = 7
    requires = ("config_env",)
    retryable = True
    dry_run_capable = True

    def run(self, planner, ctx) -> int:
        if not bool(getattr(planner, "systemd", False)):
            ctx.info(f"[phase: {self.name}] planner.systemd=False，跳过")
            return 0
        ctx.info(f"[phase: {self.name}] 渲染 systemd 模板")
        call_write_systemd_example(planner)
        ctx.success(f"[phase: {self.name}] systemd 示例已渲染")
        return 0

    def dry_run(self, planner, ctx) -> List[str]:
        if not bool(getattr(planner, "systemd", False)):
            return ["[SKIP] planner.systemd=False"]
        return [
            "+ render deploy/systemd/nuwax-host-gateway.service from .example",
        ]


from scripts.installer.phases import default_registry  # noqa: E402

default_registry.register(name="systemd", cls=SystemdPhase)