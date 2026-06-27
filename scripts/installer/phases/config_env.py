#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""config_env phase：阶段 5 — 生成 .env + data_plane.env。

设计要点（plan RK5）：
- 检测到 ``.env`` 已存在时走 ``--migrate`` 路径：用 dotenv 解析旧值，写回新模板空白处，
  原值保留。强制覆盖需 ``--force-overwrite-env``。
- task 1 骨架：run/dry_run 占位；后续 task 5 替换为
  ``_legacy_bridge.call_write_root_env`` + ``call_write_robot_env`` 调用。
"""

from __future__ import annotations

from typing import List

from scripts.installer.phases.base import Phase


class ConfigEnvPhase(Phase):
    """阶段 5 — .env / data_plane.env 渲染（plan 4.1 L115）。"""

    name = "config_env"
    order = 5
    requires = ("python_env",)
    retryable = True
    dry_run_capable = True

    def run(self, planner, ctx) -> int:
        print(f"[phase: {self.name}] 生成根 .env + 机器人专属 env（task 1 骨架）")
        return 0

    def dry_run(self, planner, ctx) -> List[str]:
        return [
            "+ render .env from .env.example",
            "+ render drivers/go2/data_plane.env from data_plane.env.example",
        ]


from scripts.installer.phases import default_registry  # noqa: E402

default_registry.register(name="config_env", cls=ConfigEnvPhase)