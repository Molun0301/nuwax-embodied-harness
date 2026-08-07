#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""geometry_runtime_deps phase：校验完整 Geometry immutable release。"""
from __future__ import annotations

from pathlib import Path
from typing import List

from scripts.installer.phases.base import Phase
from services.geometry.release_manifest import (
    ReleaseManifestError,
    default_release_manifest_path,
    verify_release_manifest,
)


class GeometryRuntimeDepsPhase(Phase):
    """阶段 3 — fail-closed 校验完整 Geometry Release manifest。"""

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
                f"[phase: {self.name}] planner.install_geometry_deps=False，跳过 Geometry Release 校验"
            )
            return 0

        manifest = default_release_manifest_path()
        try:
            proof = verify_release_manifest(manifest)
        except ReleaseManifestError as exc:
            ctx.error(
                f"[phase: {self.name}] Geometry Release manifest 校验失败（fail-closed）：{exc}"
            )
            return 1
        ctx.info(f"  geometry_release_manifest={proof.manifest_path}")
        ctx.info(f"  geometry_release_manifest_sha256={proof.manifest_sha256}")
        ctx.info(f"  geometry_release_generation={proof.generation}")
        ctx.success(
            f"[phase: {self.name}] Geometry 完整不可变制品集合已验证（{len(proof.artifacts)} artifacts）"
        )
        return 0

    def dry_run(self, planner, ctx) -> List[str]:
        if not bool(getattr(planner, "install_geometry_deps", True)):
            return ["[SKIP] planner.install_geometry_deps=False"]
        manifest = Path(default_release_manifest_path())
        return [
            f"+ python3 services/geometry/release_manifest.py check --manifest {manifest}",
            "[REQUIRE] runtime、native PGO、guardian、profile native LIO、mapping CLI、build-input closure 与 canonical manifest 必须完整且摘要一致",
            "[FAIL-CLOSED] manifest 缺失、非 canonical、制品缺失/多余或摘要漂移均终止安装",
        ]


from scripts.installer.phases import default_registry  # noqa: E402

default_registry.register(name="geometry_runtime_deps", cls=GeometryRuntimeDepsPhase)
