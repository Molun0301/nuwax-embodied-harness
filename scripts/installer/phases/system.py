#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""system phase：阶段 1 — apt 源修复 + 基础工具 + ROS2 apt。

设计要点：
- 任务范围：task 5（installer-ac-coverage）实装，调 ``scripts/installer/_legacy_bridge``
  间接调用 bootstrap 内部函数（apt 源修复、ROS2 apt 探测）。
- 错误注入（plan 6.11）：``--phase-override system=exit-100`` 触发 AC7 错误输出。
- **整合**：geometry_deps 与 system 合并（plan 任务 B 要求"避免重复"）。当
  ``planner.install_geometry_deps == True`` 时，system 阶段额外调
  ``call_install_geometry_system_dependencies``。
"""
from __future__ import annotations

from typing import List

from scripts.installer._legacy_bridge import (
    call_install_geometry_system_dependencies,
)
from scripts.installer.phases.base import Phase


class SystemPhase(Phase):
    """阶段 1 — apt 源 + 基础工具（plan 4.1 L111）。"""

    name = "system"
    order = 1
    requires = ("preflight",)
    retryable = True
    dry_run_capable = True

    def run(self, planner, ctx) -> int:
        # 离线守卫：system 阶段是网络相关
        ctx.guard_network(self.name)
        ctx.info(f"[phase: {self.name}] 系统级 apt 准备（apt 源修复 + 基础工具）")
        # 1. 基础工具（bootstrap.install_geometry_system_dependencies 已包含
        #    apt-get update + install_geometry/apt 依赖；与 system 合并避免重复）
        # 当 planner.install_geometry_deps=True 时一并跑；否则只走几何系统依赖检查
        if bool(getattr(planner, "install_geometry_deps", True)):
            call_install_geometry_system_dependencies(planner)
            ctx.success(f"[phase: {self.name}] 系统级 + geometry 系统依赖已完成")
        else:
            ctx.info(f"[phase: {self.name}] planner.install_geometry_deps=False，跳过 apt 步骤")
            ctx.success(f"[phase: {self.name}] 系统级 phase 跳过（geometry_deps 关闭）")
        return 0

    def dry_run(self, planner, ctx) -> List[str]:
        if bool(getattr(planner, "install_geometry_deps", True)):
            return [
                "+ sudo apt-get update",
                "+ sudo apt-get install -y build-essential cmake git python3-colcon-common-extensions",
                "+ rosdep init / update",
                "+ sudo apt-get install -y ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-rtabmap-ros",
            ]
        return [
            "+ sudo apt-get update（仅 info，不安装系统依赖）",
            "[SKIP] geometry_deps 关闭，跳过 RTAB-Map/Nav2 apt 安装",
        ]


from scripts.installer.phases import default_registry  # noqa: E402

default_registry.register(name="system", cls=SystemPhase)