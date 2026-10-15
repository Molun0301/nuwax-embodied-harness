"""源码来源设置、固定提交发现与受管构建子进程。"""
from __future__ import annotations

import json
import re
import sys

from scripts.neh_source import DEFAULT_BRANCH, DEFAULT_REPOSITORY, SourceError, remote_revision, source_version

from . import PACKAGE_ROOT
from .common import ManagerError, atomic_json, read_json
from .runtime import clean_environment, run_command


def is_project_default(value):
    repository = DEFAULT_REPOSITORY.rstrip("/")
    if repository.endswith(".git"):
        repository = repository[:-4]
    return not value or value in {DEFAULT_REPOSITORY, repository, repository + "/releases/latest/download/catalog.json"}


def normalized_settings(stored):
    result = dict(feed_url=DEFAULT_REPOSITORY, source_repository=DEFAULT_REPOSITORY,
                  source_branch=DEFAULT_BRANCH, source_mode="git", channel="stable", auto_start=True, gateway_port=8766)
    result.update(stored)
    if "source_mode" not in stored:
        result["source_mode"] = "git" if stored.get("source_repository") or is_project_default(stored.get("feed_url", "")) else "catalog"
    if result["source_mode"] == "git":
        result["source_repository"] = result.get("source_repository") or DEFAULT_REPOSITORY
        result["feed_url"] = result["source_repository"]
    return result


def credentials(engine):
    values = read_json(engine.home / "config/credentials.json", {})
    origin = values.get("origin", "")
    # 只迁移本项目旧默认目录的同仓库授权，不扩大自定义源的凭据范围。
    if origin and is_project_default(origin):
        origin = DEFAULT_REPOSITORY
    return {"token": values.get("token", ""), "token_origin": origin}


def discover(engine, *, refresh=False, request=None, local_request=False):
    settings = engine.settings()
    request = request or {}
    repository = request.get("source_repository", settings["source_repository"])
    branch = request.get("source_branch", settings["source_branch"])
    allow_local = bool(local_request or read_json(engine.home / "config/settings.json", {}).get("_allow_local_source"))
    if request.get("source_repository") and repository != settings["source_repository"] and not local_request:
        raise ManagerError("local_only", "临时源码仓库覆盖只能从设备命令行指定；网页请先保存源码设置。", 403)
    cache_path = engine.home / "cache/source-catalog.json"
    cache = read_json(cache_path, {})
    identity = {"repository": repository, "branch": branch}
    if not refresh and not request and cache.get("identity") == identity:
        return cache["source"]
    try:
        info = remote_revision(repository, branch, run=run_command, allow_local=allow_local, **credentials(engine))
    except (SourceError, OSError) as exc:
        raise ManagerError("source_unavailable", "无法读取源码分支，请检查仓库、分支与私有访问授权。", 502) from exc
    revision = request.get("source_revision")
    if revision:
        if not isinstance(revision, str) or not re.fullmatch(r"[0-9a-f]{40}", revision):
            raise ManagerError("source_revision", "源码修订必须是完整40位提交标识。")
        info = dict(info, revision=revision, version=source_version(revision))
    info = dict(info, allow_local=allow_local)
    if not request:
        atomic_json(cache_path, {"identity": identity, "source": info})
    return info


def bridge(engine, source, candidate, operation, job, **values):
    payload = dict(values, operation=operation, root=str(source["root"]), candidate=str(candidate), home=str(engine.home),
                   build_prefix=str(source["build_prefix"]), build_cache=str(source["build_cache"]),
                   modules=job["_plan"]["modules"], robot=job["_plan"]["robot"], revision=source["revision"])
    environment = clean_environment(candidate)
    if operation == "build":
        # 原生检查解析readelf/ldd的字段，固定工具语言而不修改设备或业务语言。
        environment["LC_ALL"] = "C"
    output = run_command([sys.executable, str(PACKAGE_ROOT / "scripts/neh_source_build.py")],
                         cwd=source["root"], environment=environment, payload=json.dumps(payload),
                         timeout=21600 if operation in ("dependencies", "build", "models") else 180,
                         cancelled=lambda: engine._cancelled(job["id"]))
    for line in reversed(output.splitlines()):
        if line.startswith("NEH_SOURCE_RESULT="):
            return json.loads(line[len("NEH_SOURCE_RESULT="):])
    raise ManagerError("source_build_result", "源码构建步骤没有返回完整结果。", 500)
