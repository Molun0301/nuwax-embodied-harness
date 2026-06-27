#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""preflight phase：阶段 0 — OS / 架构 / Python / 磁盘 / 网络 + rosdistro 探测。

设计要点（plan 4.1 L110 + RK8）：
- 仅探测 + 打印，**不**改系统（plan "check" 语义）。
- 失败检测点（plan RK8）：ROS 发行版探测；不匹配时硬退出。
- UI 交互：纯日志输出（无 questionary / rich 依赖），降级路径同样可用。
- 任务范围：task 5（installer-ac-coverage）实装。
"""
from __future__ import annotations

import os
import platform
import shutil
import subprocess
from pathlib import Path
from typing import List

from scripts.installer.phases.base import Phase


def _detect_rosdistro() -> str:
    """探测当前主机的 ROS2 发行版（humble/iron/jazzy/...）。

    复用 ``scripts/bootstrap.py:_detect_ros_setup`` 的逻辑：扫描 /opt/ros/
    目录，返回首个安装的发行版。未找到时返回 "none"。
    """
    opt = Path("/opt/ros")
    if not opt.is_dir():
        return "none"
    for entry in sorted(opt.iterdir(), reverse=True):
        if entry.is_dir() and (entry / "setup.bash").exists():
            return entry.name
    return "none"


class PreflightPhase(Phase):
    """阶段 0 — 环境探测（plan 4.1 L110）。"""

    name = "preflight"
    order = 0
    requires: tuple = ()
    retryable = False
    dry_run_capable = True

    def run(self, planner, ctx) -> int:
        ctx.info(f"[phase: {self.name}] 环境探测开始")
        # OS / 架构 / Python
        os_name = platform.system()
        os_release = platform.release()
        arch = platform.machine()
        py_ver = platform.python_version()
        ctx.info(f"  OS={os_name} {os_release}  arch={arch}  python={py_ver}")

        # /etc/os-release 详细
        try:
            with open("/etc/os-release", encoding="utf-8") as fp:
                lines = [l.strip() for l in fp if "=" in l and not l.startswith("#")]
            os_id = next((l.split("=", 1)[1].strip().strip('"') for l in lines if l.startswith("ID=")), "unknown")
            ctx.info(f"  /etc/os-release ID={os_id}")
        except (FileNotFoundError, PermissionError):
            os_id = "unknown"

        # 磁盘
        try:
            stat = os.statvfs(Path.cwd())
            free_gb = (stat.f_bavail * stat.f_frsize) / (1024 ** 3)
            ctx.info(f"  disk_free={free_gb:.1f} GB（at cwd）")
        except (OSError, AttributeError):
            ctx.info("  disk_free=无法探测（statvfs 失败）")

        # 网络（探测 DNS + curl 头）
        if shutil.which("curl") is not None:
            ctx.info("  curl=available")
        else:
            ctx.warn("  curl=missing（health 阶段将失败）")

        # ROS 发行版（plan RK8）
        ros_distro = _detect_rosdistro()
        if ros_distro == "none":
            ctx.warn("  ros_distro=none（/opt/ros/ 未发现有效安装；geometry_deps 阶段将拒绝执行）")
        else:
            ctx.info(f"  ros_distro={ros_distro}")

        # 网络可达性（不真正发大流量请求，只做 TCP 探测）
        try:
            import socket
            socket.create_connection(("pypi.org", 443), timeout=3).close()
            ctx.info("  network=reachable (pypi.org:443)")
        except OSError:
            ctx.warn("  network=unreachable (pypi.org:443 不可达；离线模式需 --offline)")

        ctx.success(f"[phase: {self.name}] 环境探测完成")
        return 0

    def dry_run(self, planner, ctx) -> List[str]:
        return [
            "+ uname -m",
            "+ python3 --version",
            "+ . /etc/os-release && echo $ID",
            "+ df -Pk .",
            "+ /opt/ros/*/setup.bash（rosdistro 探测）",
            "+ python3 -c 'import socket; socket.create_connection((\"pypi.org\", 443), timeout=3)'",
        ]


from scripts.installer.phases import default_registry  # noqa: E402

default_registry.register(name="preflight", cls=PreflightPhase)