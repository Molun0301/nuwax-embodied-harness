#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""systemd phase：阶段 7 — systemd 模板渲染。

2026-07-07 Phase 8 升级：
- ``deploy/systemd/nuwax-geometry-runtime.service``（非 .example）已显式设置
  ``NUWAX_LCM_BACKEND=real-lcm`` + ``NUWAX_REAL_LCM_REQUIRED=true`` 并移除
  ``SuccessExitStatus=0 4``，让 systemd 把缺 liblcm / 缺 udpm multicast
  视为真实失败。``nuwax-host-gateway.service.example`` 走 .env 默认值，
  不在本 phase 强制覆盖。
- 渲染顺序保持原 plan：``deploy/systemd/nuwax-host-gateway.service.example``
  → ``nuwax-host-gateway.service``。
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
        if self._is_forbidden_by_robot_profile(planner):
            ctx.info(self._robot_profile_skip_message())
            return 0
        if not bool(getattr(planner, "systemd", False)):
            ctx.info(f"[phase: {self.name}] planner.systemd=False，跳过")
            return 0
        ctx.info(f"[phase: {self.name}] 渲染 systemd 模板")
        call_write_systemd_example(planner)
        # Phase 8:为 production profile 明确提示 LCM env 已经随 systemd unit 注入,
        # 操作员不需要在 .env 再覆盖。``nuwax-geometry-runtime.service``（非 example）
        # 已含 NUWAX_LCM_BACKEND=real-lcm + NUWAX_REAL_LCM_REQUIRED=true + udpm URL。
        ctx.info(
            f"  [systemd] 提醒:nuwax-geometry-runtime.service 已 hardcode "
            f"NUWAX_LCM_BACKEND=real-lcm + NUWAX_REAL_LCM_REQUIRED=true,"
            f"SuccessExitStatus 仅 0。缺 liblcm/缺 udpm 视为真实失败并 Restart=on-failure。"
        )
        ctx.success(f"[phase: {self.name}] systemd 示例已渲染")
        return 0

    def dry_run(self, planner, ctx) -> List[str]:
        if not bool(getattr(planner, "systemd", False)):
            return ["[SKIP] planner.systemd=False"]
        return [
            "+ render deploy/systemd/nuwax-host-gateway.service from .example",
            "+ assert deploy/systemd/nuwax-geometry-runtime.service Environment=NUWAX_LCM_BACKEND=real-lcm",
            "+ assert deploy/systemd/nuwax-geometry-runtime.service Environment=NUWAX_REAL_LCM_REQUIRED=true",
            "+ assert deploy/systemd/nuwax-geometry-runtime.service SuccessExitStatus=0（不再 0 4）",
        ]


from scripts.installer.phases import default_registry  # noqa: E402

default_registry.register(name="systemd", cls=SystemdPhase)