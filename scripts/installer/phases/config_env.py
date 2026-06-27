#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""config_env phase：阶段 5 — 生成 .env + data_plane.env。

设计要点（plan RK5）：
- 调 ``_legacy_bridge.call_write_root_env`` + ``call_write_robot_env``，
  分别触发 bootstrap.write_root_env（L983）与 write_robot_env（L990）。
- 检测到 ``.env`` 已存在时由 bootstrap.write_root_env 的 _read_existing_env
  路径处理；强制覆盖需 ``--force-overwrite-env``（RK5，本期未实装强制覆盖开关）。
"""
from __future__ import annotations

from typing import List

from scripts.installer._legacy_bridge import (
    call_write_root_env,
    call_write_robot_env,
)
from scripts.installer.phases.base import Phase


class ConfigEnvPhase(Phase):
    """阶段 5 — .env / data_plane.env 渲染（plan 4.1 L115）。"""

    name = "config_env"
    order = 5
    requires = ("python_env",)
    retryable = True
    dry_run_capable = True

    def run(self, planner, ctx) -> int:
        if self._is_forbidden_by_robot_profile(planner):
            ctx.info(self._robot_profile_skip_message())
            return 0
        ctx.info(f"[phase: {self.name}] 生成根 .env + 机器人专属 env")
        call_write_root_env(planner)
        call_write_robot_env(planner)
        ctx.success(f"[phase: {self.name}] env 渲染完成")
        return 0

    def dry_run(self, planner, ctx) -> List[str]:
        return [
            "+ render .env from .env.example",
            "+ render drivers/go2/data_plane.env from data_plane.env.example",
        ]


from scripts.installer.phases import default_registry  # noqa: E402

default_registry.register(name="config_env", cls=ConfigEnvPhase)