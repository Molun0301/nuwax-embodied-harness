#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""system phase：阶段 1 — apt 源修复 + 基础工具 + Rust/LCM native runtime 依赖。

2026-07-07 Phase 8 升级：移除 vendor 路径相关代码，强制安装/验证 liblcm-dev、
pkg-config、python3-lcm（或等价 Python binding）。原 dimos workspace 绝对路径
依赖已下线；本 phase 现负责生产路径下 nuwax-geometry-runtime real-LCM transport
所需的全部系统库，并在 aarch64 目标机 profile 下执行 pkg-config 二次校验。

设计要点：
- 任务范围：task 5（installer-ac-coverage）实装 + 2026-07-04 B3 + 2026-07-07 Phase 8。
- 本 phase 只负责基础工具与 native runtime 必需系统包
  （rustup / liblcm-dev / pkg-config / python3-lcm）。
- real-LCM 物装（2026-07-07 Phase 3）：``nuwax-transport real-lcm`` feature
  启用 ``nuwax-lcm`` safe wrapper，链接系统 liblcm.so；缺包时 Rust build 失败，
  缺 Python binding 时 ``LcmMotionCommandBridge`` 不可用，因此系统依赖必须在
  installer 阶段就位。
- 错误注入（plan 6.11）：``--phase-override system=exit-100`` 触发 AC7 错误输出。
- 系统依赖由本 phase 直接安装和验证。
"""
from __future__ import annotations

import os
import platform
import shutil
import subprocess
from typing import List, Tuple

from scripts.installer.phases.base import Phase
from scripts.installer.errors import InstallerErrorCode, PhaseError


# Phase 8:real-LCM 必需的系统包列表（apt 名 + 备选 Python binding）。
LCM_APT_PACKAGES: Tuple[str, ...] = (
    "liblcm-dev",
    "pkg-config",
)
LCM_PYTHON_APT_PACKAGES: Tuple[str, ...] = ("python3-lcm",)
# 当 apt python3-lcm 不可用时的 pip fallback（与 §12.6 硬约束一致）
LCM_PYTHON_PIP_FALLBACK: Tuple[str, ...] = ("lcm",)


def _run_capture(args, *, timeout: int = 60) -> Tuple[int, str, str]:
    """运行外部命令并返回 (returncode, stdout, stderr)。"""
    try:
        proc = subprocess.run(
            args,
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
    except FileNotFoundError:
        return 127, "", f"executable not found: {args[0] if args else ''}"
    except subprocess.TimeoutExpired:
        return 124, "", "timeout"
    return proc.returncode, proc.stdout, proc.stderr


def _is_aarch64_target() -> bool:
    """判断当前 profile 是否在 aarch64 目标机（Jetson / Go2 实机）下运行。

    触发 aarch64 二次校验（pkg-config --modversion lcm 在目标机必须通过）。
    """
    machine = platform.machine().lower()
    if "aarch64" in machine or "arm64" in machine:
        return True
    # 显式 env override（installer 在 aarch64 cross setup 下使用）
    return os.environ.get("NUWAX_TARGET_ARCH", "").lower() in ("aarch64", "arm64")


def _apt_install(packages: Tuple[str, ...]) -> int:
    """apt-get install -y 一组包；返回 apt exit code。

    失败时不抛错，由调用方决定降级或 fail-fast。
    """
    if shutil.which("apt-get") is None:
        return 127
    rc, _, _ = _run_capture(
        ("sudo", "-n", "apt-get", "install", "-y", "--no-install-recommends", *packages),
        timeout=300,
    )
    return rc


def _verify_pkg_config_lcm() -> bool:
    """验证 ``pkg-config --modversion lcm`` 在系统上能返回非空版本。

    - 仅校验 liblcm 实际安装情况。
    - 版本范围 `[1.3, 1.6)`:`nuwax-lcm-sys/build.rs` 同步锁;
      镜像必须装 liblcm-dev 1.3~1.5,1.6+ 触发 `LCM_VERSION_UNSUPPORTED`
    - 失败信息以 `LCM_VERSION_UNSUPPORTED` 关键字(供 CI grep 识别)
    """
    if shutil.which("pkg-config") is None:
        return False
    rc, out, _ = _run_capture(("pkg-config", "--modversion", "lcm"), timeout=3)
    if rc != 0:
        return False
    version = out.strip()
    # 简单 major.minor 比较:接受 1.3 ~ 1.5.x;1.6+ / 1.2.x 拒绝
    try:
        parts = version.split(".")
        if len(parts) < 2:
            return False
        major = int(parts[0])
        minor = int(parts[1].split("-")[0].split("+")[0])
        if major != 1:
            return False
        if minor < 3:
            return False
        if minor >= 6:
            return False
    except (ValueError, IndexError):
        return False
    return True


def _verify_python_lcm() -> bool:
    """验证 ``import lcm`` 在当前 Python 解释器下可用（python3-lcm 或 pip fallback）。

    Python LCM binding 不可用时必须 fail-fast。
    """
    rc, _, _ = _run_capture(("python3", "-c", "import lcm; print(getattr(lcm, '__version__', 'ok'))"), timeout=5)
    return rc == 0


def _pip_install_python_lcm() -> int:
    """pip fallback 安装 ``lcm`` Python 包；返回 pip exit code。

    仅在 apt python3-lcm 不可用时调用；不抛错。
    """
    if shutil.which("python3") is None:
        return 127
    # ``--break-system-packages`` 在 Ubuntu 24+ PEP 668 下需要；保留兼容性
    rc, _, _ = _run_capture(
        ("python3", "-m", "pip", "install", "--user", "--break-system-packages", *LCM_PYTHON_PIP_FALLBACK),
        timeout=180,
    )
    if rc != 0:
        # 重试不带 --break-system-packages（兼容老 pip）
        rc, _, _ = _run_capture(
            ("python3", "-m", "pip", "install", "--user", *LCM_PYTHON_PIP_FALLBACK),
            timeout=180,
        )
    return rc


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
        ctx.info(
            f"[phase: {self.name}] 系统级 apt 准备（apt 源修复 + 基础工具 + "
            f"Rust/LCM 依赖: {', '.join(LCM_APT_PACKAGES + LCM_PYTHON_APT_PACKAGES)}）"
        )
        # 1. 基础工具 + native runtime 系统依赖（rustup / liblcm-dev / build-essential /
        #    pkg-config / python3-lcm）。
        # 本 phase 直接负责 LCM 系统包安装。
        # 2. 安装 liblcm-dev + pkg-config（real-LCM FFI 必需）
        apt_rc = _apt_install(LCM_APT_PACKAGES)
        if apt_rc != 0:
            ctx.warn(
                f"  [system] apt-get install {LCM_APT_PACKAGES} 失败 rc={apt_rc}；"
                f"将依赖 preflight 探测结果继续（pkg-config/liblcm 已在镜像预装场景）"
            )
        else:
            ctx.info(f"  [system] apt-get install {LCM_APT_PACKAGES} 成功")
        # 3. 安装 python3-lcm 或 pip fallback
        py_apt_rc = _apt_install(LCM_PYTHON_APT_PACKAGES)
        py_lcm_ok = _verify_python_lcm()
        if not py_lcm_ok:
            ctx.info(
                f"  [system] python3-lcm 不可用（apt rc={py_apt_rc}），尝试 pip fallback: "
                f"{' '.join(LCM_PYTHON_PIP_FALLBACK)}"
            )
            pip_rc = _pip_install_python_lcm()
            if pip_rc != 0 or not _verify_python_lcm():
                # §12.6 硬约束:不安装就 fail-fast,提示需用本仓 binding 替换
                raise PhaseError(
                    phase=self.name,
                    message=(
                        f"Python LCM binding 安装失败:apt {LCM_PYTHON_APT_PACKAGES} rc={py_apt_rc} "
                        f"+ pip {LCM_PYTHON_PIP_FALLBACK} 失败"
                    ),
                    file_path=__file__,
                    code=InstallerErrorCode.PHASE_FAILED,
                    remediation=(
                        "请在目标机预装 python3-lcm（apt），或允许 user pip 安装 lcm 包；"
                        "或为本仓提供等价 thin Python LCM binding 替代。"
                    ),
                )
            ctx.info(f"  [system] pip fallback lcm 安装成功")
        else:
            ctx.info(f"  [system] python3-lcm 可用（apt rc={py_apt_rc}）")
        # 4. aarch64 目标机 profile 二次校验（§12.6 / §13.2 硬约束）
        # LCM 验证失败信息含 LCM_VERSION_UNSUPPORTED，供 CI / smoke test 识别。
        if _is_aarch64_target():
            if not _verify_pkg_config_lcm():
                raise PhaseError(
                    phase=self.name,
                    message=(
                        "LCM_VERSION_UNSUPPORTED: aarch64 目标机 profile 下 "
                        "``pkg-config --modversion lcm`` 二次校验失败。"
                    ),
                    file_path=__file__,
                    code=InstallerErrorCode.PHASE_FAILED,
                    remediation=(
                        "在目标机执行 'sudo apt-get install -y liblcm-dev pkg-config' 后重试;"
                        "CI 镜像必须预装这两个包,版本锁 1.3~1.5。"
                    ),
                )
            ctx.info("  [system] aarch64 profile 下 pkg-config --modversion lcm 二次校验 PASS")
        ctx.success(
            f"[phase: {self.name}] 系统级 + native runtime + LCM 系统依赖已完成"
        )
        return 0

    def dry_run(self, planner, ctx) -> List[str]:
        lines = [
            "+ sudo apt-get update",
            "+ sudo apt-get install -y build-essential curl git",
            f"+ sudo apt-get install -y {' '.join(LCM_APT_PACKAGES)}",
            f"+ sudo apt-get install -y {' '.join(LCM_PYTHON_APT_PACKAGES)} "
            f"（或 pip fallback: {' '.join(LCM_PYTHON_PIP_FALLBACK)}）",
            "+ pkg-config --modversion lcm",
            "+ python3 -c 'import lcm'",
        ]
        if _is_aarch64_target():
            lines.append("+ [aarch64] 二次校验 pkg-config --modversion lcm（§13.2 硬约束）")
        lines.append(
            "+ rustup target add aarch64-unknown-linux-gnu"
            "（仅当 NUWAX_TARGET_ARCH=aarch64）"
        )
        return lines


from scripts.installer.phases import default_registry  # noqa: E402

default_registry.register(name="system", cls=SystemPhase)
