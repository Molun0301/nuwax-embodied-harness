#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""preflight phase：阶段 0 — OS / 架构 / Python / 磁盘 / 网络 + native runtime 探测。

设计要点（plan 4.1 L110 + 2026-07-03 drop-ros2）：
- 仅探测 + 打印，**不**改系统（plan "check" 语义）。
- 探测目标态 runtime 必需：rustc / cargo + ``nuwax-workspace`` 路径存在 + LCM
  系统库可用 + ``nuwax-geometry-runtime`` 二进制（非强制，仅 print）。
- 探测失败**仅警告**，不阻断；目标态运行门槛由实机启动 + health 阶段把关。
- UI 交互：纯日志输出（无 questionary / rich 依赖），降级路径同样可用。
- 任务范围：task 5（installer-ac-coverage）实装 + 2026-07-04 B3 目标态改造。
"""
from __future__ import annotations

import os
import platform
import shutil
import subprocess
from pathlib import Path
from typing import List, Tuple

from scripts.installer.phases.base import Phase


def _run_safe(args: tuple, timeout: int = 3) -> Tuple[bool, str]:
    """运行外部命令并返回 ``(ok, output)``。失败或超时返回 ``(False, "")``。

    用于探测 rustc / cargo 版本；不抛异常，不污染日志。
    """
    try:
        proc = subprocess.run(
            args,
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False, ""
    if proc.returncode != 0:
        return False, ""
    return True, proc.stdout.strip()


def _check_rustc_version() -> str:
    """探测 ``rustc --version``；失败返回 "missing"。"""
    if shutil.which("rustc") is None:
        return "missing"
    ok, out = _run_safe(("rustc", "--version"))
    if not ok:
        return "missing"
    # 输出形如 "rustc 1.88.0 (...)"
    return out.split(maxsplit=1)[1] if len(out.split(maxsplit=1)) > 1 else out


def _check_cargo_version() -> str:
    """探测 ``cargo --version``；失败返回 "missing"。"""
    if shutil.which("cargo") is None:
        return "missing"
    ok, out = _run_safe(("cargo", "--version"))
    if not ok:
        return "missing"
    return out.split(maxsplit=1)[1] if len(out.split(maxsplit=1)) > 1 else out


def _check_rustup_toolchains() -> str:
    """探测 ``rustup toolchain list`` 已安装工具链；失败返回 "n/a"。"""
    if shutil.which("rustup") is None:
        return "n/a"
    ok, out = _run_safe(("rustup", "toolchain", "list"), timeout=4)
    if not ok:
        return "n/a"
    return out.replace("\n", " | ") or "empty"


def _check_nuwax_workspace() -> str:
    """探测 ``nuwax-workspace`` 目录是否存在（路径硬编码见 plan 锚点）。

    Returns:
        "found=<path>" / "missing"
    """
    candidates = (
        Path("/home/ml/Deeprob/dimos/rust重写与移植/nuwax-workspace"),
        Path(os.path.expanduser("~")) / "dimos" / "rust重写与移植" / "nuwax-workspace",
    )
    for cand in candidates:
        if cand.is_dir():
            return f"found={cand}"
    return "missing"


def _check_lcm_library() -> str:
    """探测 ``liblcm-dev`` / ``liblcm.so`` 是否可用。

    Returns:
        "available" / "missing"
    """
    # pkg-config 是最权威的；失败再用 find 兜底
    if shutil.which("pkg-config") is not None:
        ok, out = _run_safe(("pkg-config", "--modversion", "lcm"), timeout=2)
        if ok and out:
            return f"available (pkg-config: {out})"
    # 兜底：常见 liblcm 路径
    for cand in ("/usr/lib/aarch64-linux-gnu/liblcm.so", "/usr/lib/x86_64-linux-gnu/liblcm.so",
                 "/usr/lib/liblcm.so", "/usr/local/lib/liblcm.so"):
        if Path(cand).is_file():
            return f"available ({cand})"
    return "missing"


class PreflightPhase(Phase):
    """阶段 0 — 环境探测（plan 4.1 L110）。"""

    name = "preflight"
    order = 0
    requires: tuple = ()
    retryable = False
    dry_run_capable = True

    def run(self, planner, ctx) -> int:
        if self._is_forbidden_by_robot_profile(planner):
            ctx.info(self._robot_profile_skip_message())
            return 0
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

        # Rust 工具链（plan 2026-07-03 drop-ros2 目标态 runtime 必须）
        rustc_v = _check_rustc_version()
        cargo_v = _check_cargo_version()
        toolchains = _check_rustup_toolchains()
        if rustc_v != "missing":
            ctx.info(f"  rustc_version={rustc_v}")
        else:
            ctx.warn("  rustc_version=missing（system 阶段将尝试 rustup 安装）")
        if cargo_v != "missing":
            ctx.info(f"  cargo_version={cargo_v}")
        else:
            ctx.warn("  cargo_version=missing（system 阶段将尝试 cargo 安装）")
        if toolchains != "n/a":
            ctx.info(f"  rustup_toolchains={toolchains}")

        # nuwax-workspace 路径
        ws = _check_nuwax_workspace()
        if ws.startswith("found="):
            ctx.info(f"  nuwax_workspace_path={ws.split('=', 1)[1]}")
        else:
            ctx.warn("  nuwax_workspace_path=missing（services/geometry/nuwax_geometry_runtime build_workspace 阶段会失败）")

        # LCM 系统库
        lcm = _check_lcm_library()
        if lcm.startswith("available"):
            ctx.info(f"  lcm_library={lcm}")
        else:
            ctx.warn("  lcm_library=missing（system 阶段将尝试 apt-get install liblcm-dev）")

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
            "+ rustc --version",
            "+ cargo --version",
            "+ rustup toolchain list",
            "+ test -d /home/ml/Deeprob/dimos/rust重写与移植/nuwax-workspace",
            "+ pkg-config --modversion lcm",
            "+ python3 -c 'import socket; socket.create_connection((\"pypi.org\", 443), timeout=3)'",
        ]


from scripts.installer.phases import default_registry  # noqa: E402

default_registry.register(name="preflight", cls=PreflightPhase)
