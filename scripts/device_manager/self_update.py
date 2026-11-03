"""管理程序独立更新，复用引导器的封闭归档校验，不触碰业务拥有方。"""
from __future__ import annotations

import shutil
import sys

from scripts.neh_bootstrap import (BootstrapError, remove_incomplete_manager, require_install_layout,
                                   unpack_manager, switch_link)

from . import MANAGER_REVISION
from .common import ManagerError, atomic_json, digest_file, identifier, read_json
from .runtime import clean_environment, run_command


def apply_manager(engine, job):
    plan = job["_plan"]
    source = plan["_source"]
    if source["revision"] == MANAGER_REVISION:
        engine._journal(job, committed=True)
        return {"manager_version": source["version"], "revision": source["revision"], "unchanged": True, "restart_required": False}
    from scripts.neh_source import source_checkout
    engine._progress(job, "manager_source", 5, "获取固定源码并在设备本地组装管理程序")
    archive = engine.home / "cache" / ("source-manager-" + job["id"] + ".tar.gz")
    with source_checkout(engine.home, source["repository"], source["branch"], source["revision"],
                         run=run_command, allow_local=source.get("allow_local", False),
                         cancelled=lambda: engine._cancelled(job["id"]),
                         progress=lambda message: engine._progress(job, "manager_source", 25, message)) as checkout:
        # 使用目标提交自己的打包规则，新增管理依赖不会被旧版本白名单遗漏。
        run_command([sys.executable, "-B", "-m", "scripts.device_manager.source_package", "--root", str(checkout["root"]),
                     "--output", str(archive), "--version", source["version"]], cwd=checkout["root"],
                    environment=clean_environment(), timeout=180, cancelled=lambda: engine._cancelled(job["id"]))
    metadata = {"version": source["version"], "revision": source["revision"], "sha256": digest_file(archive), "size": archive.stat().st_size}
    root = engine.manager_home / "manager"
    versions = root / "versions"
    versions.mkdir(parents=True, exist_ok=True)
    if (root / "current").exists() and not (root / "current").is_symlink():
        raise ManagerError("manager_state", "管理入口不是受管符号链接，拒绝覆盖。", 409)
    previous = (root / "current").resolve() if (root / "current").is_symlink() else None
    if previous and previous.parent != versions:
        raise ManagerError("manager_state", "管理入口不属于当前安装根。", 409)
    target = versions / (identifier(metadata["version"]) + "-" + metadata["sha256"][:16])
    remove_incomplete_manager(target)
    if previous == target and target.exists():
        raise ManagerError("already_current", "当前管理程序已是此完整版本。", 409)
    engine._cancelled(job["id"])
    engine._progress(job, "manager_verify", 55, "验证管理归档和解释器入口")
    if not target.exists():
        target.mkdir()
        try:
            manifest = unpack_manager(archive, target)
            if manifest["version"] != metadata["version"]:
                raise ManagerError("manager_mismatch", "管理包版本与已确认计划不一致。")
            run_command([sys.executable, "-m", "scripts.device_manager", "--help"], cwd=target, environment=clean_environment(), timeout=30)
        except BaseException:
            shutil.rmtree(target)
            raise
    else:
        manifest = read_json(target / "manager-manifest.json", {})
        if manifest.get("version") != metadata["version"]:
            raise ManagerError("manager_state", "已有候选管理版本记录不符。", 409)
        for entry in manifest.get("files", []):
            path = target / entry["path"]
            if not path.is_file() or digest_file(path) != entry["sha256"]:
                raise ManagerError("manager_state", "已有候选管理版本文件已改变。", 409)
    try:
        require_install_layout(engine.manager_home, manifest)
    except BootstrapError as exc:
        raise ManagerError("manager_layout_unsupported", str(exc), 409) from exc
    atomic_json(target / "installed.json", {"complete": True, "product": "neh-manager", "version": manifest["version"]})
    engine._cancelled(job["id"])
    engine._journal(job, manager_previous=str(previous) if previous else None, manager_candidate=str(target), manager_switch=True)
    engine._progress(job, "manager_switch", 90, "切换管理程序，业务拥有方保持运行")
    switch_link(root / "current", target)
    if previous:
        switch_link(root / "previous", previous)
    engine._journal(job, committed=True)
    archive.unlink(missing_ok=True)
    archive.with_name(archive.name + ".manifest.json").unlink(missing_ok=True)
    return {"manager_version": metadata["version"], "revision": metadata.get("revision", ""), "restart_required": True}
