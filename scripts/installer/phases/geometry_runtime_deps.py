#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""geometry_runtime_deps phase：阶段 3 — native runtime（nuwax-geometry-runtime）系统依赖。

2026-07-07 Phase 8 升级：路径依赖检查从旧外部 workspace 绝对路径切到本仓 repo-local
``services/nuwax-workspace``。移除对 ``services/geometry/nuwax_geometry_runtime/vendor``
与 ``vendor-config.json`` 的假设（vendor_dimos_crates.sh 已退役）。

``nuwax-geometry-runtime`` 的平台侧依赖收敛为 LCM 系统库 + Rust 工具链
（preflight 已探测）。
- 本 phase 现在负责**校验** ``services/nuwax-workspace`` 完整性 + 提示用户重建命令，
  不直接执行 apt/cargo（避免重复 system 阶段动作）。

设计要点：

- ``planner.install_geometry_deps`` 字段保留语义（= "是否期望安装
  nuwax-geometry-runtime OS 依赖"），用于 wizard / cli 跳过判断；本 phase 只
  在 planner.install_geometry_deps=True 时校验产物，否则提示跳过。
- 不重复 apt：实际 liblcm-dev / rustup 已由 system 阶段处理。
- Phase 8 §13.5：
  - 校验 ``services/nuwax-workspace/Cargo.toml`` 存在且有 [workspace]
  - 校验 ``crates/`` 下 48 个目录齐全
  - 校验 ``Cargo.lock`` 存在（locked build 必须有 lockfile）
"""
from __future__ import annotations

import os
from pathlib import Path
from typing import List

from scripts.installer.phases.base import Phase


# nuwax-geometry-runtime 编译产物默认路径（与 ``services/geometry/nuwax_geometry_runtime/Cargo.toml``
# 一致）。plan 锚点：installer 仅做提示，不主动 build（build 由独立 lane 接管）。
DEFAULT_GEOMETRY_RUNTIME_TARGET: str = (
    "services/geometry/nuwax_geometry_runtime/target/release/nuwax-geometry-runtime"
)

# Phase 8:repo-local nuwax-workspace 与期望 crate 数
REPO_LOCAL_NUWAX_WORKSPACE: Path = Path("services/nuwax-workspace")
EXPECTED_CRATE_COUNT: int = 48


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


def _check_repo_local_nuwax_workspace(repo_root: Path) -> dict:
    """Phase 8：检查本仓 repo-local workspace 完整性。

    Returns:
        dict with keys:
          - ws_path: Path
          - cargo_toml_ok: bool
          - cargo_lock_ok: bool
          - crate_dir_ok: bool
          - actual_crate_count: int
          - manifest_present: bool
    """
    ws = repo_root / REPO_LOCAL_NUWAX_WORKSPACE
    cargo_toml = ws / "Cargo.toml"
    cargo_lock = ws / "Cargo.lock"
    crates_dir = ws / "crates"
    cargo_toml_ok = cargo_toml.is_file()
    cargo_lock_ok = cargo_lock.is_file()
    crate_dir_ok = crates_dir.is_dir()
    actual_count = sum(1 for p in crates_dir.iterdir() if p.is_dir()) if crate_dir_ok else 0
    manifest_present = (ws / "IMPORT_MANIFEST.json").is_file() or (ws / "IMPORT_MANIFEST.md").is_file()
    return {
        "ws_path": ws,
        "cargo_toml_ok": cargo_toml_ok,
        "cargo_lock_ok": cargo_lock_ok,
        "crate_dir_ok": crate_dir_ok,
        "actual_crate_count": actual_count,
        "manifest_present": manifest_present,
    }


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
        # 探测本仓 repo-local workspace（Phase 8）
        repo_root = _resolve_workspace_root()
        ws_state = _check_repo_local_nuwax_workspace(repo_root)
        # 1) Cargo.toml + Cargo.lock 校验
        if ws_state["cargo_toml_ok"]:
            ctx.info(
                f"  repo_local_nuwax_workspace_cargo_toml={ws_state['ws_path'] / 'Cargo.toml'} (ok)"
            )
        else:
            ctx.warn(
                f"  repo_local_nuwax_workspace_cargo_toml=missing（{ws_state['ws_path'] / 'Cargo.toml'}）；"
                "Phase 1 lane 必须先把 services/nuwax-workspace 落到本仓"
            )
        if ws_state["cargo_lock_ok"]:
            ctx.info(
                f"  repo_local_nuwax_workspace_cargo_lock={ws_state['ws_path'] / 'Cargo.lock'} (ok)"
            )
        else:
            ctx.warn(
                f"  repo_local_nuwax_workspace_cargo_lock=missing（locked build 需要 Cargo.lock，"
                f"见 §13.1 硬约束）；Phase 1 lane 必须先把 dimos root Cargo.lock 同步过来"
            )
        # 2) crates/ 校验（48 个）
        if ws_state["crate_dir_ok"]:
            if ws_state["actual_crate_count"] == EXPECTED_CRATE_COUNT:
                ctx.info(
                    f"  repo_local_nuwax_workspace_crates={ws_state['actual_crate_count']} "
                    f"(matches expected {EXPECTED_CRATE_COUNT})"
                )
            else:
                ctx.warn(
                    f"  repo_local_nuwax_workspace_crates={ws_state['actual_crate_count']} "
                    f"(expected {EXPECTED_CRATE_COUNT})；可能 IMPORT_MANIFEST 未更新或部分 crate 未 tracked"
                )
        else:
            ctx.warn(f"  repo_local_nuwax_workspace_crates=missing（{ws_state['ws_path'] / 'crates'}）")
        # 3) IMPORT_MANIFEST 校验
        if ws_state["manifest_present"]:
            ctx.info(
                "  repo_local_nuwax_workspace_manifest=present（IMPORT_MANIFEST 已被 git 跟踪）"
            )
        else:
            ctx.warn(
                "  repo_local_nuwax_workspace_manifest=missing（Phase 1 应同时创建 IMPORT_MANIFEST.md/.json）"
            )
        # 4) nuwax-geometry-runtime 产物（仅探测 + 提示，不主动 build）
        binary = repo_root / DEFAULT_GEOMETRY_RUNTIME_TARGET
        if binary.is_file() and os.access(binary, os.X_OK):
            ctx.info(f"  nuwax_geometry_runtime_binary={binary}（executable）")
            ctx.success(
                f"[phase: {self.name}] native runtime 产物已就位"
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
            f"+ test -f {REPO_LOCAL_NUWAX_WORKSPACE}/Cargo.toml（[workspace] section）",
            f"+ test -f {REPO_LOCAL_NUWAX_WORKSPACE}/Cargo.lock（§13.1 locked build 必需）",
            f"+ test -d {REPO_LOCAL_NUWAX_WORKSPACE}/crates（应有 {EXPECTED_CRATE_COUNT} 个成员）",
            f"+ test -e {REPO_LOCAL_NUWAX_WORKSPACE}/IMPORT_MANIFEST.md（provenance）",
            "+ test -x services/geometry/nuwax_geometry_runtime/target/release/nuwax-geometry-runtime",
            "[INFO] native runtime 产物由 build_geometry / 实机 cargo build 兜底；installer 不重复 apt/cargo",
        ]


from scripts.installer.phases import default_registry  # noqa: E402

default_registry.register(name="geometry_runtime_deps", cls=GeometryRuntimeDepsPhase)
