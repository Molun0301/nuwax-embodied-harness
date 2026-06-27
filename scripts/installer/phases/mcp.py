#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""mcp phase：阶段 8 — MCP 注入配置生成。

设计要点（plan 4.1 L118）：
- 调 ``bootstrap.configure_mcp``（L1302）生成 external_mcp_servers.host_gateway.json +
  mcp_proxy_config.host_gateway.json。
- ``--phase-override mcp=skip`` 时整个 phase 跳过（对应 plan 4.4 ``=skip`` 语义）。
- task 1 骨架：run/dry_run 占位；后续 task 5 替换为
  ``_legacy_bridge.call_configure_mcp`` 调用。
- docs_guard 集成（plan 6.12 + spec "文档 token 引用清理与守门"）：task 6 实现。
"""

from __future__ import annotations

from typing import List

from scripts.installer.phases.base import Phase


class McpPhase(Phase):
    """阶段 8 — MCP 配置注入（plan 4.1 L118）。"""

    name = "mcp"
    order = 8
    requires = ("systemd",)
    retryable = True
    dry_run_capable = True

    def run(self, planner, ctx) -> int:
        print(f"[phase: {self.name}] 注入 MCP 配置（task 1 骨架）")
        return 0

    def dry_run(self, planner, ctx) -> List[str]:
        return [
            "+ write deploy/nuwaclaw/external_mcp_servers.host_gateway.json",
            "+ write deploy/nuwaclaw/mcp_proxy_config.host_gateway.json",
        ]


from scripts.installer.phases import default_registry  # noqa: E402

default_registry.register(name="mcp", cls=McpPhase)