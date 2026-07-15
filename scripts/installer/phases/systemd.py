#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""systemd phase：安装 service identities、目录和已渲染 production units。"""
from __future__ import annotations

import os
from pathlib import Path
import subprocess
from typing import List

from scripts.installer.phases.base import Phase


class SystemdPhase(Phase):
    """阶段 7 — 调用唯一部署安装器，安装但不越过 provision/enable 门。"""

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
        repo_root = Path(__file__).resolve().parents[3]
        installer = repo_root / "scripts" / "install_motion_guard_deployment.sh"
        command = ["bash", str(installer), "install"]
        if os.geteuid() != 0:
            command = ["sudo", "-n", *command]
        ctx.info(f"[phase: {self.name}] 安装 systemd identities、目录与 production units")
        completed = subprocess.run(command, cwd=repo_root, check=False)
        if completed.returncode != 0:
            raise RuntimeError(
                "systemd deployment install failed; ensure passwordless sudo, "
                "built runtime binaries, and a valid multicast route"
            )
        ctx.info(
            "  [systemd] units 已安装并 daemon-reload；motion guard durable records "
            "仍须通过 provision 动作后才能显式 enable"
        )
        ctx.success(f"[phase: {self.name}] systemd 部署文件已安装")
        return 0

    def dry_run(self, planner, ctx) -> List[str]:
        if not bool(getattr(planner, "systemd", False)):
            return ["[SKIP] planner.systemd=False"]
        return [
            "+ sudo -n bash scripts/install_motion_guard_deployment.sh install",
            "+ render @NUWAX_REPO_ROOT@ in all production units",
            "+ systemd-sysusers + systemd-tmpfiles + multicast preflight + daemon-reload",
            "+ keep provision/enable explicit until actuator records are verified",
        ]


from scripts.installer.phases import default_registry  # noqa: E402

default_registry.register(name="systemd", cls=SystemdPhase)
