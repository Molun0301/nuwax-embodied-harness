#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""system phase：阶段 1 — apt 源修复 + 基础工具 + Rust/LCM native runtime 依赖。

设计要点：
- 任务范围：task 5（installer-ac-coverage）实装，调 ``scripts/installer/_legacy_bridge``
  间接调用 bootstrap 内部函数（apt 源修复、Rust/LCM native runtime 系统依赖）。
- 去 ROS2 化（2026-07-03 drop-ros2 / 2026-07-04 B3）：原 ROS2 apt 安装
  （python3-colcon-common-extensions / rosdep / ros-${ROS_DISTRO}-navigation2 /
  ros-${ROS_DISTRO}-rtabmap-ros）已物理删除，由 ``nuwax-geometry-runtime`` Rust
  二进制 + LCM 系统库替代。本 phase 现在只负责基础工具与 native runtime 必需
  系统包（rustup / liblcm-dev / build-essential）。
- 错误注入（plan 6.11）：``--phase-override system=exit-100`` 触发 AC7 错误输出。
- ``call_install_geometry_system_dependencies`` 在 ``scripts/installer/_legacy_bridge.py``
  已 no-op 化（去 ROS2 后无任何 apt ROS 包要装），但保留调用以稳定 cli 调用链。
"""
from __future__ import annotations

from typing import List

from scripts.installer._legacy_bridge import (
    call_install_geometry_system_dependencies,
)
from scripts.installer.phases.base import Phase


class SystemPhase(Phase):
    """阶段 1 — apt 源 + 基础工具 + native runtime 系统依赖（plan 4.1 L111）。"""

    name = "system"
    order = 1
    requires = ("preflight",)
    retryable = True
    dry_run_capable = True

    def run(self, planner, ctx) -> int:
        if self._is_forbidden_by_robot_profile(planner):
            ctx.info(self._robot_profile_skip_message())
            return 0
        # 离线守卫：system 阶段是网络相关
        ctx.guard_network(self.name)
        ctx.info(f"[phase: {self.name}] 系统级 apt 准备（apt 源修复 + 基础工具 + Rust/LCM 依赖）")
        # 1. 基础工具 + native runtime 系统依赖（rustup / liblcm-dev / build-essential /
        #    pkg-config）。原 ROS2 apt 已由 bootstrap.no-op 桥接替代，此处仅打印完成消息。
        call_install_geometry_system_dependencies(planner)
        ctx.success(f"[phase: {self.name}] 系统级 + native runtime 系统依赖已完成（no-op 桥接）")
        return 0

    def dry_run(self, planner, ctx) -> List[str]:
        return [
            "+ sudo apt-get update",
            "+ sudo apt-get install -y build-essential pkg-config curl git",
            "+ sudo apt-get install -y liblcm-dev",
            "+ rustup target add aarch64-unknown-linux-gnu（仅当 NUWAX_TARGET_ARCH=aarch64）",
        ]


from scripts.installer.phases import default_registry  # noqa: E402

default_registry.register(name="system", cls=SystemPhase)
