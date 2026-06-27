#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""build_ws phase：阶段 4 — colcon 构建 geometry ROS2 工作区。

设计要点（plan RK4 + spec "dry-run 渲染契约"）：
- ``retryable = False``：colcon 构建失败率高（GO2SLAM 依赖多），失败立即停。
  提供 ``--force-retry-phase build_ws`` 覆盖（task 5 实装）。
- ``dry_run_capable = False``：colcon 构建无法精确 dry-run 模拟；spec 要求
  醒目颜色标注"该阶段 dry-run 不模拟真实副作用"。
- task 1 骨架：run/dry_run 占位；后续 task 5 替换为
  ``_legacy_bridge.call_build_workspaces`` 调用。
"""

from __future__ import annotations

from typing import List

from scripts.installer.phases.base import Phase


class BuildWsPhase(Phase):
    """阶段 4 — colcon 构建（plan 4.1 L114）。"""

    name = "build_ws"
    order = 4
    requires = ("geometry_deps",)
    retryable = False  # plan RK4
    dry_run_capable = False  # spec "dry-run 渲染契约"

    def run(self, planner, ctx) -> int:
        print(f"[phase: {self.name}] colcon 构建 geometry ROS2 工作区（task 1 骨架）")
        return 0

    def dry_run(self, planner, ctx) -> List[str]:
        return [
            "[WARN] build_ws 阶段 dry-run 不模拟真实副作用（colcon 构建无法精确 dry-run）",
            "+ bash services/geometry/build_ros2_workspace.sh",
        ]


from scripts.installer.phases import default_registry  # noqa: E402

default_registry.register(name="build_ws", cls=BuildWsPhase)