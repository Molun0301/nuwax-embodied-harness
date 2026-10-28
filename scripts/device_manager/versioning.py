"""按完整提交历史判定更新方向，浅克隆和检查失败均不能当作新版本。"""
from __future__ import annotations

import hashlib

from scripts.neh_source import SourceError, _environment, _git, source_version, validate_repository

from .common import file_lock, read_json, atomic_json


def revision_relation(engine, source, installed):
    target = source["revision"]
    if not installed:
        return "unknown"
    if target == installed:
        return "same"
    source_version(installed)
    repository = validate_repository(source["repository"], allow_local=source.get("allow_local", False))
    identity = {"repository": repository, "current": installed, "target": target}
    area = engine.home / "cache/version-graph"
    area.mkdir(parents=True, exist_ok=True)
    key = hashlib.sha256(repository.encode()).hexdigest()[:24]
    record = area / (key + ".json")
    cached = read_json(record, {})
    if cached.get("identity") == identity:
        return cached["relation"]
    environment = _environment(repository)
    graph = area / (key + ".git")
    with file_lock(area / (key + ".lock"), blocking=True):
        if not graph.exists():
            _git(["init", "--bare", "--quiet", str(graph)], environment=environment)
        def git(*arguments):
            return _git(arguments, root=graph, environment=environment)
        git("config", "remote.origin.url", repository)
        # 单独保存完整提交图；不用业务的浅克隆，也不下载源码内容。
        git("fetch", "--quiet", "--no-tags", "--filter=tree:0", "origin", installed, target)
        for revision in (installed, target):
            git("cat-file", "-e", revision + "^{commit}")
        try:
            git("merge-base", "--is-ancestor", installed, target)
            relation = "newer"
        except SourceError:
            try:
                git("merge-base", "--is-ancestor", target, installed)
                relation = "older"
            except SourceError:
                relation = "diverged"
        atomic_json(record, {"identity": identity, "relation": relation})
    return relation
