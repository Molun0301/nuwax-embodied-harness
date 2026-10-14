"""管理程序独立更新，复用引导器的封闭归档校验，不触碰业务拥有方。"""
from __future__ import annotations

import re
import shutil
import sys
import urllib.parse
from pathlib import Path

from scripts.neh_bootstrap import MAX_MANAGER_BYTES, unpack_manager, switch_link, version_tuple

from .common import ManagerError, digest_file, identifier, read_json
from .releases import download, validate_url
from .runtime import clean_environment, run_command


def manager_metadata(engine, catalog, local=None):
    metadata = catalog.get("manager")
    if not isinstance(metadata, dict):
        raise ManagerError("manager_unavailable", "当前版本目录没有管理程序发布包。", 409)
    metadata = dict(metadata)
    identifier(metadata.get("version", ""))
    if not re.fullmatch(r"[0-9a-f]{64}", str(metadata.get("sha256", ""))) or type(metadata.get("size")) is not int or not 0 < metadata["size"] <= MAX_MANAGER_BYTES:
        raise ManagerError("manager_invalid", "管理发布包的大小或摘要声明无效。")
    python = metadata.get("python", {})
    try:
        if not version_tuple(python.get("minimum")) <= sys.version_info[:2] < version_tuple(python.get("maximum_exclusive")):
            raise ValueError()
    except Exception as exc:
        raise ManagerError("manager_incompatible", "管理发布包不支持当前Python解释器。") from exc
    source = Path(local).resolve().as_uri() if local else engine.settings()["feed_url"]
    metadata["url"] = urllib.parse.urljoin(source, metadata.get("url", ""))
    validate_url(metadata["url"], allow_file=bool(local))
    return metadata


def apply_manager(engine, job):
    plan = job["_plan"]
    metadata = plan["_manager"]
    root = engine.home / "manager"
    versions = root / "versions"
    versions.mkdir(parents=True, exist_ok=True)
    if (root / "current").exists() and not (root / "current").is_symlink():
        raise ManagerError("manager_state", "管理入口不是受管符号链接，拒绝覆盖。", 409)
    previous = (root / "current").resolve() if (root / "current").is_symlink() else None
    if previous and previous.parent != versions:
        raise ManagerError("manager_state", "管理入口不属于当前安装根。", 409)
    target = versions / (identifier(metadata["version"]) + "-" + metadata["sha256"][:16])
    if previous == target:
        raise ManagerError("already_current", "当前管理程序已是此完整版本。", 409)
    archive = engine.home / "cache" / (metadata["sha256"] + ".manager.tar.gz")
    engine._progress(job, "manager_download", 5, "下载管理程序发布包")
    if not archive.is_file() or archive.stat().st_size != metadata["size"] or digest_file(archive) != metadata["sha256"]:
        download(metadata, archive, progress=lambda n, total: engine._progress(job, "manager_download", 5 + int(45 * n / total), "正在下载管理程序"),
                 cancelled=lambda: engine._cancelled(job["id"]), **engine._source_options(plan.get("_local_catalog")))
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
    engine._cancelled(job["id"])
    engine._journal(job, manager_previous=str(previous) if previous else None, manager_candidate=str(target), manager_switch=True)
    engine._progress(job, "manager_switch", 90, "切换管理程序，业务拥有方保持运行")
    switch_link(root / "current", target)
    if previous:
        switch_link(root / "previous", previous)
    engine._journal(job, committed=True)
    return {"manager_version": metadata["version"], "restart_required": True}
