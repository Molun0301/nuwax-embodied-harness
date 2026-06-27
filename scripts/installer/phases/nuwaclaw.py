#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""nuwaclaw phase：阶段 9 — NuwaxClaw Docker（可选，默认关闭）。

设计要点（plan RK6 + spec "错误定位输出契约" / RK15）：
- 默认 disabled（``planner.install_nuwaclaw_docker = False``）；cli 先 skip。
- 启用时 ``nuwaclaw_host_mount == "rw"`` 必须双重确认（plan RK6）；由 wizard 阶段
  完成确认，installer 阶段不重复。
- 镜像源 ``nuwax-docker-images-registry.cn-hangzhou.cr.aliyuncs.com`` 在 ``--offline``
  模式下视为网络依赖；本 phase 受 ``ctx.guard_network`` 拦截。
- ``dry_run_capable = False``（spec "dry-run 渲染契约"）。
- 调 ``_legacy_bridge.call_install_nuwaclaw_client``（bootstrap.install_nuwaclaw_client L1255）。
"""
from __future__ import annotations

from typing import List

from scripts.installer._legacy_bridge import call_install_nuwaclaw_client
from scripts.installer.phases.base import Phase


class NuwaclawPhase(Phase):
    """阶段 9 — NuwaxClaw Docker 客户端（plan 4.1 L119）。"""

    name = "nuwaclaw"
    order = 9
    requires = ("mcp",)
    retryable = False
    dry_run_capable = False  # spec "dry-run 渲染契约"

    def run(self, planner, ctx) -> int:
        if self._is_forbidden_by_robot_profile(planner):
            ctx.info(self._robot_profile_skip_message())
            return 0
        if not bool(getattr(planner, "install_nuwaclaw_docker", False)):
            ctx.info(f"[phase: {self.name}] planner.install_nuwaclaw_docker=False，跳过")
            return 0
        ctx.guard_network(self.name)
        ctx.info(f"[phase: {self.name}] 启动 NuwaxClaw Docker 客户端（host_mount={getattr(planner, 'nuwaclaw_host_mount', 'none')}）")
        call_install_nuwaclaw_client(planner)
        ctx.success(f"[phase: {self.name}] NuwaxClaw Docker 客户端已启动")
        return 0

    def dry_run(self, planner, ctx) -> List[str]:
        if not bool(getattr(planner, "install_nuwaclaw_docker", False)):
            return ["[SKIP] planner.install_nuwaclaw_docker=False"]
        return [
            "[WARN] nuwaclaw 阶段 dry-run 不模拟真实副作用（Docker pull/run 无法精确 dry-run）",
            "+ docker run -d --name nuwax-client ...",
        ]


from scripts.installer.phases import default_registry  # noqa: E402

default_registry.register(name="nuwaclaw", cls=NuwaclawPhase)