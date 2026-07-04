#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""geometry_runtime_deps phase：阶段 3 — native runtime（nuwax-geometry-runtime）系统依赖。

去 ROS2 化（2026-07-03 drop-ros2 / 2026-07-04 B3）：

- 原 ``geometry_deps`` 阶段（RTAB-Map / Nav2 / colcon / rosdep）已物理删除；
- ``nuwax-geometry-runtime`` Rust 二进制承担原 ROS2 运行时职责，平台侧依赖
  收敛为 LCM 系统库 + Rust 工具链（preflight 已探测）。
- 本 phase 现在负责**校验** ``nuwax-geometry-runtime`` 编译产物 + 提示用户
  重建命令，不直接执行 apt/cargo（避免重复 system 阶段动作）。

设计要点：

- ``planner.install_geometry_deps`` 字段保留语义（= "是否期望安装
  nuwax-geometry-runtime OS 依赖"），用于 wizard / cli 跳过判断；本 phase 只
  在 planner.install_geometry_deps=True 时校验产物，否则提示跳过。
- 不重复 apt：实际 liblcm-dev / rustup 已由 system 阶段处理（no-op 桥接）。
"""
from __future__ import annotations

import os
import shutil
from pathlib import Path
from typing import List

from scripts.installer.phases.base import Phase


# nuwax-geometry-runtime 编译产物默认路径（与 ``services/geometry/nuwax_geometry_runtime/Cargo.toml``
# 一致）。plan 锚点：installer 仅做提示，不主动 build（build 由独立 lane 接管）。
DEFAULT_GEOMETRY_RUNTIME_TARGET: str = (
    "services/geometry/nuwax_geometry_runtime/target/release/nuwax-geometry-runtime"
)


def _resolve_workspace_root() -> Path:
    """从当前 cwd / scripts/installer 推断仓库根。

    兼容两种调用形态：``python3 scripts/installer/cli.py``（cwd=repo 根）和
    ``python3 -m scripts.installer``（cwd=repo 根）。
    """
    cwd = Path.cwd().resolve()
    if (cwd / "scripts" / "installer").is_dir():
        return cwd
    # 兜底：往上找包含 ``scripts/installer`` 的最近祖先
    for parent in cwd.parents:
        if (parent / "scripts" / "installer").is_dir():
            return parent
    return cwd


class GeometryRuntimeDepsPhase(Phase):
    """阶段 3 — nuwax-geometry-runtime native runtime 产物校验（plan 4.1 L113）。"""

    name = "geometry_runtime_deps"
    order = 3
    requires = ("python_env",)
    retryable = True
    dry_run_capable = True

    def run(self, planner, ctx) -> int:
        if self._is_forbidden_by_robot_profile(planner):
            ctx.info(self._robot_profile_skip_message())
            return 0
        if not bool(getattr(planner, "install_geometry_deps", True)):
            ctx.info(
                f"[phase: {self.name}] planner.install_geometry_deps=False，跳过 nuwax-geometry-runtime 产物校验"
            )
            return 0
        # 探测 nuwax-geometry-runtime 产物（仅探测 + 提示，不主动 build）
        repo_root = _resolve_workspace_root()
        binary = repo_root / DEFAULT_GEOMETRY_RUNTIME_TARGET
        if binary.is_file() and os.access(binary, os.X_OK):
            ctx.info(f"  nuwax_geometry_runtime_binary={binary}（executable）")
            ctx.success(
                f"[phase: {self.name}] native runtime 产物已就位（plan 2026-07-03 drop-ros2 目标态）"
            )
            return 0
        ctx.warn(
            f"  nuwax_geometry_runtime_binary=missing（{binary}）；"
            "将依赖实机 build_geometry 阶段或手动 cargo build"
        )
        # 失败仅警告，不抛错（目标态允许 build 与 installer 解耦）
        ctx.success(
            f"[phase: {self.name}] native runtime 产物缺失已提示（installer 不强制 build）"
        )
        return 0

    def dry_run(self, planner, ctx) -> List[str]:
        if not bool(getattr(planner, "install_geometry_deps", True)):
            return ["[SKIP] planner.install_geometry_deps=False"]
        return [
            "+ test -x services/geometry/nuwax_geometry_runtime/target/release/nuwax-geometry-runtime",
            "[INFO] native runtime 产物由 build_geometry / 实机 cargo build 兜底；installer 不重复 apt/cargo",
        ]


from scripts.installer.phases import default_registry  # noqa: E402

default_registry.register(name="geometry_runtime_deps", cls=GeometryRuntimeDepsPhase)