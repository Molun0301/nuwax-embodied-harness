#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""build_ws phase：阶段 4 — colcon 构建 geometry ROS2 工作区。

设计要点（plan RK4 + spec "dry-run 渲染契约"）：
- ``retryable = False``：colcon 构建失败率高（GO2SLAM 依赖多），失败立即停。
- 调 ``scripts/installer/_legacy_bridge.call_build_workspaces`` 触发
  ``bootstrap.build_workspaces``（bootstrap.py:1129）。
- ``dry_run_capable = False``：colcon 构建无法精确 dry-run 模拟。
"""
from __future__ import annotations

from typing import List

from scripts.installer._legacy_bridge import call_build_workspaces
from scripts.installer.phases.base import Phase


class BuildWsPhase(Phase):
    """阶段 4 — colcon 构建（plan 4.1 L114）。"""

    name = "build_ws"
    order = 4
    requires = ("geometry_deps",)
    retryable = False  # plan RK4
    dry_run_capable = False  # spec "dry-run 渲染契约"

    def run(self, planner, ctx) -> int:
        if self._is_forbidden_by_robot_profile(planner):
            ctx.info(self._robot_profile_skip_message())
            return 0
        if not bool(getattr(planner, "build_geometry", True)):
            ctx.info(f"[phase: {self.name}] planner.build_geometry=False，跳过")
            return 0
        ctx.guard_network(self.name)
        ctx.info(f"[phase: {self.name}] colcon 构建 geometry ROS2 工作区")
        call_build_workspaces(planner)
        ctx.success(f"[phase: {self.name}] colcon 构建完成")
        return 0

    def dry_run(self, planner, ctx) -> List[str]:
        if not bool(getattr(planner, "build_geometry", True)):
            return ["[SKIP] planner.build_geometry=False"]
        return [
            "[WARN] build_ws 阶段 dry-run 不模拟真实副作用（colcon 构建无法精确 dry-run）",
            "+ bash services/geometry/build_ros2_workspace.sh",
            "+ source install/setup.bash",
        ]


from scripts.installer.phases import default_registry  # noqa: E402

default_registry.register(name="build_ws", cls=BuildWsPhase)