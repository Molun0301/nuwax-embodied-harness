"""按完整提交历史判定更新方向，浅克隆和检查失败均不能当作新版本。"""
from __future__ import annotations

import hashlib
import json
import subprocess
from urllib.parse import unquote, urlsplit
from pathlib import Path

from scripts.neh_source import SourceError, _environment, _git, source_version, validate_repository

from .common import file_lock, read_json, atomic_json


def _import_installed_history(engine, graph):
    """只读导入已安装本地来源的提交对象，不要求未公开提交存在于云端。"""
    current = engine.current()
    repository = current.get("source_repository", "")
    if not repository.startswith("file:"):
        return
    parsed = urlsplit(repository)
    root = Path(unquote(parsed.path))
    if not root.is_dir():
        return
    repository = validate_repository(repository, allow_local=True)
    environment = _environment(repository)
    command = ["git", "-c", "core.hooksPath=/dev/null", "-c", "core.fsmonitor=false", "-C", str(root)]
    revision = current.get("revision")
    source_version(revision)
    # pack-objects无--revs，列表只含提交，不读取/复制源码树或大文件；源仓库只读。
    try:
        commits = subprocess.run(command + ["rev-list", revision], env=environment, check=True, capture_output=True).stdout
        pack = subprocess.run(command + ["pack-objects", "--stdout"], env=environment, input=commits, check=True, capture_output=True).stdout
        subprocess.run(["git", "-C", str(graph), "index-pack", "--stdin", "--fix-thin"], env=environment,
                       input=pack, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    except subprocess.SubprocessError as exc:
        raise SourceError("无法读取安装时本地来源的提交历史") from exc


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
    record = area / (hashlib.sha256(json.dumps(identity, sort_keys=True).encode()).hexdigest() + ".json")
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
        _import_installed_history(engine, graph)
        # 单独保存完整提交图；不用业务的浅克隆，也不下载源码内容。
        try:
            git("cat-file", "-e", installed + "^{commit}")
            requested = [target]
        except SourceError:
            requested = [installed, target]
        git("fetch", "--quiet", "--no-tags", "--filter=tree:0", "origin", *requested)
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
        atomic_json(record, {"identity": identity, "relation": relation,
                             "same_tree": git("cat-file", "-p", installed).splitlines()[0] == git("cat-file", "-p", target).splitlines()[0]})
    return relation


def source_tree_unchanged(engine, source, installed):
    if not installed:
        return False
    if source["revision"] == installed:
        return True
    revision_relation(engine, source, installed)
    repository = validate_repository(source["repository"], allow_local=source.get("allow_local", False))
    identity = {"repository": repository, "current": installed, "target": source["revision"]}
    key = hashlib.sha256(json.dumps(identity, sort_keys=True).encode()).hexdigest()
    return read_json(engine.home / "cache/version-graph" / (key + ".json"), {}).get("same_tree", False)
