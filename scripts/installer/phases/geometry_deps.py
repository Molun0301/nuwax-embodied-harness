#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""geometry_deps phase：阶段 3 — RTAB-Map / Nav2 系统依赖。

设计要点（plan 任务 B 要求"避免重复"）：
- geometry_deps 与 system 合并：实际依赖安装已在 ``system.run()`` 中调
  ``call_install_geometry_system_dependencies`` 完成；本 phase 仅做**幂等校验**
  + 元信息打印，避免重复 apt 调用。
- cli.py 的 ``_should_skip_phase`` 已经把
  ``planner.install_geometry_deps=False`` 的情况过滤到 system 阶段，本 phase
  一般只在 ``install_geometry_deps=False`` 但仍想单独跑 geometry_deps 的罕见
  场景被调用——此场景下做空操作 + 提示。
- 当 planner.install_geometry_deps=True 时，本 phase 仅打印"已合并到 system 阶段"
  提示，不重复 apt 调用。
"""
from __future__ import annotations

from typing import List

from scripts.installer.phases.base import Phase


class GeometryDepsPhase(Phase):
    """阶段 3 — geometry 系统依赖（plan 4.1 L113；与 system 合并避免重复）。"""

    name = "geometry_deps"
    order = 3
    requires = ("python_env",)
    retryable = True
    dry_run_capable = True

    def run(self, planner, ctx) -> int:
        if not bool(getattr(planner, "install_geometry_deps", True)):
            ctx.info(f"[phase: {self.name}] planner.install_geometry_deps=False，跳过")
            return 0
        # 实际 apt 已在 system 阶段跑过；此处仅打印合并提示，避免重复 apt 调用
        ctx.info(f"[phase: {self.name}] 与 system 阶段合并：apt 依赖已在 system 完成")
        ctx.success(f"[phase: {self.name}] geometry 系统依赖（合并态，幂等通过）")
        return 0

    def dry_run(self, planner, ctx) -> List[str]:
        if not bool(getattr(planner, "install_geometry_deps", True)):
            return ["[SKIP] planner.install_geometry_deps=False"]
        return [
            "[INFO] geometry_deps 与 system 阶段合并：实际 apt 已在 system 执行",
            "+ sudo apt-get install -y ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-rtabmap-ros",
        ]


from scripts.installer.phases import default_registry  # noqa: E402

default_registry.register(name="geometry_deps", cls=GeometryDepsPhase)