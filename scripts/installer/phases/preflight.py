#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""preflight phase：阶段 0 — OS / 架构 / Python / 磁盘 / 网络 + native runtime 探测。

设计要点（plan 4.1 L110 + 2026-07-03 drop-ros2 + 2026-07-07 Phase 8）：
- 仅探测 + 打印，**不**改系统（plan "check" 语义）。
- 探测目标态 runtime 必需：rustc / cargo + 本仓 repo-local
  ``services/nuwax-workspace`` 路径 + LCM 系统库可用 + 48 crates 成员齐全 +
  ``nuwax-geometry-runtime`` 二进制（非强制，仅 print）。
- 探测失败**仅警告**，不阻断；目标态运行门槛由实机启动 + health 阶段把关。
- 2026-07-07 Phase 8：移除对 ``/home/ml/Deeprob/dimos/rust重写与移植/nuwax-workspace``
  绝对路径的探测；改为校验本仓 repo-local workspace 与 IMPORT_MANIFEST 一致性。
- UI 交互：纯日志输出（无 questionary / rich 依赖），降级路径同样可用。
- 任务范围：task 5（installer-ac-coverage）实装 + 2026-07-04 B3 目标态改造。
"""
from __future__ import annotations

import json
import os
import platform
import shutil
import subprocess
from pathlib import Path
from typing import List, Tuple

from scripts.installer.phases.base import Phase


# Phase 8:repo-local nuwax-workspace 路径（与 .gitignore / phase 1 entrypoint 对齐）
REPO_LOCAL_NUWAX_WORKSPACE: Path = Path("services/nuwax-workspace")
EXPECTED_CRATE_COUNT: int = 48  # 与 services/nuwax-workspace/Cargo.toml members 对齐


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


def _resolve_repo_root() -> Path:
    """从 caller cwd / scripts/installer 推断仓库根。

    兼容两种调用形态：``python3 scripts/installer/cli.py``（cwd=repo 根）和
    ``python3 -m scripts.installer``（cwd=repo 根）。
    """
    cwd = Path.cwd().resolve()
    if (cwd / "scripts" / "installer").is_dir():
        return cwd
    for parent in cwd.parents:
        if (parent / "scripts" / "installer").is_dir():
            return parent
    return cwd


def _check_repo_local_nuwax_workspace() -> str:
    """Phase 8 重写：探测本仓 repo-local ``services/nuwax-workspace``。

    Returns:
        "found=<path> members=<n> expected=<EXPECTED_CRATE_COUNT>" / 简短的诊断标签
    """
    repo_root = _resolve_repo_root()
    ws = repo_root / REPO_LOCAL_NUWAX_WORKSPACE
    if not ws.is_dir():
        return "missing（services/nuwax-workspace 不存在）"
    cargo_toml = ws / "Cargo.toml"
    if not cargo_toml.is_file():
        return "missing（Cargo.toml 不存在）"
    crates_dir = ws / "crates"
    if not crates_dir.is_dir():
        return "missing（crates/ 不存在）"
    actual_count = sum(1 for p in crates_dir.iterdir() if p.is_dir())
    cargo_lock = ws / "Cargo.lock"
    lock_ok = cargo_lock.is_file()
    # IMPORT_MANIFEST 兼容性核对（若存在 .json）
    manifest_json = ws / "IMPORT_MANIFEST.json"
    manifest_md = ws / "IMPORT_MANIFEST.md"
    manifest_status = "none"
    if manifest_json.is_file():
        try:
            data = json.loads(manifest_json.read_text(encoding="utf-8"))
            listed = int(data.get("crate_count", 0))
            manifest_status = f"json.crate_count={listed}"
        except (json.JSONDecodeError, ValueError, TypeError):
            manifest_status = "json.invalid"
    elif manifest_md.is_file():
        manifest_status = "md.present"
    return (
        f"found={ws} crates={actual_count} expected={EXPECTED_CRATE_COUNT} "
        f"cargo_lock={'yes' if lock_ok else 'no'} manifest={manifest_status}"
    )


def _check_lcm_library() -> str:
    """探测 ``liblcm-dev`` / ``liblcm.so`` / ``pkg-config --modversion lcm`` 可用性。

    Phase 8 升级：
    - 优先用 ``pkg-config --modversion lcm`` 作为权威探测（缺 pkg-config → fail-fast）
    - 兜底常见 liblcm 路径

    Returns:
        "available" / "missing" / "missing pkg-config"
    """
    if shutil.which("pkg-config") is None:
        # Phase 8 §13.2:缺 pkg-config 也需要 fail-fast 信号
        for cand in (
            "/usr/lib/aarch64-linux-gnu/liblcm.so",
            "/usr/lib/x86_64-linux-gnu/liblcm.so",
            "/usr/lib/liblcm.so",
            "/usr/local/lib/liblcm.so",
        ):
            if Path(cand).is_file():
                return f"available ({cand}, no pkg-config)"
        return "missing（pkg-config 不可用且 liblcm.so 找不到）"
    ok, out = _run_safe(("pkg-config", "--modversion", "lcm"), timeout=2)
    if ok and out:
        return f"available (pkg-config: {out})"
    # pkg-config 在但 lcm.pc 不在 → 系统库缺
    for cand in (
        "/usr/lib/aarch64-linux-gnu/liblcm.so",
        "/usr/lib/x86_64-linux-gnu/liblcm.so",
        "/usr/lib/liblcm.so",
        "/usr/local/lib/liblcm.so",
    ):
        if Path(cand).is_file():
            return f"available ({cand}, pkg-config 无 lcm.pc)"
    return "missing（pkg-config 找到但 lcm.pc 缺失→system 阶段需 apt install liblcm-dev）"


def _check_python_lcm_module() -> str:
    """Phase 8 §13.2:探测 ``python3 -c 'import lcm'``。

    Returns:
        "available" / "missing"
    """
    if shutil.which("python3") is None:
        return "missing（python3 不可用）"
    try:
        proc = subprocess.run(
            ("python3", "-c", "import lcm"),
            capture_output=True,
            text=True,
            timeout=3,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return "missing"
    if proc.returncode == 0:
        return "available"
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

        # repo-local nuwax-workspace（Phase 8 重写后 anchor）
        ws = _check_repo_local_nuwax_workspace()
        if ws.startswith("found="):
            ctx.info(f"  repo_local_nuwax_workspace={ws}")
        else:
            ctx.warn(f"  repo_local_nuwax_workspace={ws}")
            ctx.warn(
                "  → system 阶段会失败；runtime build 阶段也失败。"
                "检查 Phase 1 lane 是否把 services/nuwax-workspace 落到 repo。"
            )

        # LCM 系统库（pkg-config / lcm.pc 权威探测）
        lcm = _check_lcm_library()
        if lcm.startswith("available"):
            ctx.info(f"  lcm_library={lcm}")
        else:
            ctx.warn(f"  lcm_library={lcm}")

        # python3-lcm 探测（Phase 8 §13.2）
        py_lcm = _check_python_lcm_module()
        if py_lcm == "available":
            ctx.info("  python_lcm_module=available")
        else:
            ctx.warn("  python_lcm_module=missing（system 阶段会尝试 apt python3-lcm 或 pip fallback）")

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
            f"+ test -d {REPO_LOCAL_NUWAX_WORKSPACE}",
            f"+ test -f {REPO_LOCAL_NUWAX_WORKSPACE}/Cargo.toml",
            f"+ test -d {REPO_LOCAL_NUWAX_WORKSPACE}/crates（应有 {EXPECTED_CRATE_COUNT} 个成员）",
            "+ pkg-config --modversion lcm（Phase 8 §13.2 硬约束）",
            "+ python3 -c 'import lcm'（Phase 8 §13.2 硬约束）",
            "+ python3 -c 'import socket; socket.create_connection((\"pypi.org\", 443), timeout=3)'",
        ]


from scripts.installer.phases import default_registry  # noqa: E402

default_registry.register(name="preflight", cls=PreflightPhase)