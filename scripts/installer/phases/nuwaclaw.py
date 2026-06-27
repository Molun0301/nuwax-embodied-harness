#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""nuwaclaw phase：阶段 9 — NuwaxClaw Docker（可选，默认关闭）。

设计要点（plan RK6 + spec "错误定位输出契约" / RK15）：
- 默认 disabled（``planner.install_nuwaclaw_docker = False``）；wizard 中显式确认才打开。
- 启用时 ``nuwaclaw_host_mount == "rw"`` 必须双重确认（plan RK6）；installer 阶段抛
  ``InstallerError(code=3, NUWACLAW_RISK_REFUSED)``。
- 镜像源 ``nuwax-docker-images-registry.cn-hangzhou.cr.aliyuncs.com`` 在 ``--offline``
  模式下视为网络依赖；spec "offline 模式网络拦截" 要求 ``exit 8`` 提示。
- ``dry_run_capable = False``（spec "dry-run 渲染契约"）：Docker 副作用无法精确模拟。
- task 1 骨架：run/dry_run 占位；后续 task 5 替换为
  ``_legacy_bridge.call_install_nuwaclaw_client`` 调用（bootstrap 内部 L1255）。
"""

from __future__ import annotations

from typing import List

from scripts.installer.phases.base import Phase


class NuwaclawPhase(Phase):
    """阶段 9 — NuwaxClaw Docker 客户端（plan 4.1 L119）。"""

    name = "nuwaclaw"
    order = 9
    requires = ("mcp",)
    retryable = False
    dry_run_capable = False  # spec "dry-run 渲染契约"

    def run(self, planner, ctx) -> int:
        print(f"[phase: {self.name}] 启动 NuwaxClaw Docker 客户端（task 1 骨架）")
        return 0

    def dry_run(self, planner, ctx) -> List[str]:
        return [
            "[WARN] nuwaclaw 阶段 dry-run 不模拟真实副作用（Docker pull/run 无法精确 dry-run）",
            "+ docker run -d --name nuwax-client ...",
        ]


from scripts.installer.phases import default_registry  # noqa: E402

default_registry.register(name="nuwaclaw", cls=NuwaclawPhase)