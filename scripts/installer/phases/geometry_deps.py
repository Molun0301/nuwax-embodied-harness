#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""geometry_deps phase：阶段 3 — RTAB-Map / Nav2 系统依赖。

设计要点：
- task 1 骨架：run/dry_run 占位；后续 task 5 替换为
  ``_legacy_bridge.call_install_geometry_system_dependencies`` 调用。
- 调 bootstrap 内部 ``install_geometry_system_dependencies``（L1020）。
"""

from __future__ import annotations

from typing import List

from scripts.installer.phases.base import Phase


class GeometryDepsPhase(Phase):
    """阶段 3 — geometry 系统依赖（plan 4.1 L113）。"""

    name = "geometry_deps"
    order = 3
    requires = ("python_env",)
    retryable = True
    dry_run_capable = True

    def run(self, planner, ctx) -> int:
        print(f"[phase: {self.name}] 安装 RTAB-Map / Nav2 系统依赖（task 1 骨架）")
        return 0

    def dry_run(self, planner, ctx) -> List[str]:
        return [
            "+ sudo apt-get update",
            "+ sudo apt-get install -y ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-rtabmap-ros",
        ]


from scripts.installer.phases import default_registry  # noqa: E402

default_registry.register(name="geometry_deps", cls=GeometryDepsPhase)