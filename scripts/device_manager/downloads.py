"""安装时下载的第三方文件缓存；固定来源、大小和摘要才能进入候选环境。"""
from __future__ import annotations

import hashlib
import json
import re
import shutil
import urllib.parse
from pathlib import Path, PurePosixPath

from .common import ManagerError, atomic_json, digest_file
from .releases import MAX_DOWNLOAD, download, validate_url

MODEL_SETTINGS = {"NUWAX_PERCEPTION_YOLO_WORLD_MODEL", "NUWAX_MEMORY_TEXT_EMBEDDING_MODEL",
                  "NUWAX_MEMORY_IMAGE_EMBEDDING_MODEL", "NUWAX_MEMORY_EMBEDDING_MODEL"}


def uses_default_model(value):
    """与共同安装器保持一致：词法模式及旧占位符都不是用户模型路径。"""
    value = str(value).strip().lower()
    return value in {"", "lexical-only", "lexical_only", "disabled"} or value.startswith("hashing")


def safe_relative(value, allow_dot=False):
    if value == "." and allow_dot:
        return PurePosixPath(".")
    if not isinstance(value, str) or not value or "\\" in value or "\x00" in value:
        raise ManagerError("download_recipe_invalid", "下载配方包含非法文件路径。")
    path = PurePosixPath(value)
    if not path.parts or path.is_absolute() or path.as_posix() != value or any(part in (".", "..") for part in path.parts):
        raise ManagerError("download_recipe_invalid", "下载配方路径不能越出固定目录。")
    return path


def validate_download(record):
    if not isinstance(record, dict) or type(record.get("size")) is not int or not 0 < record["size"] <= MAX_DOWNLOAD:
        raise ManagerError("download_recipe_invalid", "下载配方缺少有效文件大小。")
    if not re.fullmatch(r"[0-9a-fA-F]{64}", str(record.get("sha256", ""))):
        raise ManagerError("download_recipe_invalid", "下载配方缺少固定SHA256摘要。")
    validate_url(record.get("url", ""))


def cached_download(engine, record, *, progress, cancelled, filename=None):
    """摘要相同且内容完整时复用；保留轮子原名，命中仍核对真实内容。"""
    validate_download(record)
    filename = filename or "download.bin"
    if len(safe_relative(filename).parts) != 1:
        raise ManagerError("download_recipe_invalid", "缓存文件名不能包含目录。")
    directory = engine.home / "cache/downloads" / record["sha256"].lower()
    directory.mkdir(parents=True, exist_ok=True)
    target = directory / filename
    cancelled()
    if target.is_file() and target.stat().st_size == record["size"] and digest_file(target) == record["sha256"].lower():
        progress(record["size"], record["size"])
        return target, True
    download(record, target, progress=progress, cancelled=cancelled, **engine._source_options())
    target.chmod(0o600)
    return target, False


def materialize_cached(source, destination, mode=0o600):
    """复制已校验缓存，避免业务修改缓存文件。"""
    destination = Path(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with open(source, "rb") as reader, open(destination, "xb") as writer:
        shutil.copyfileobj(reader, writer, length=1024 * 1024)
    destination.chmod(mode)


def normalize_recipe(manifest, artifact_url):
    recipe = manifest.get("install")
    if not isinstance(recipe, dict) or set(recipe) - {"python", "native", "models", "go2"}:
        raise ManagerError("download_recipe_invalid", "业务包缺少受支持的在线安装配方。")
    recipe = json.loads(json.dumps(recipe))
    for key in ("python", "native", "models"):
        if not isinstance(recipe.get(key), list) or len(recipe[key]) > 10000:
            raise ManagerError("download_recipe_invalid", "在线安装配方结构或数量无效。")
    seen = set()
    for record in recipe["python"]:
        if not isinstance(record, dict):
            raise ManagerError("download_recipe_invalid", "Python下载记录必须是对象。")
        name, version, filename = record.get("name", ""), record.get("version", ""), record.get("filename", "")
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]*", name) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9.!+_-]*", version):
            raise ManagerError("download_recipe_invalid", "Python依赖缺少固定名称或版本。")
        if len(safe_relative(filename).parts) != 1 or not filename.endswith(".whl") or len(filename.split("-")) < 5:
            raise ManagerError("download_recipe_invalid", "Python依赖必须是目标平台的预编译轮子。")
        canonical = re.sub(r"[-_.]+", "-", name).lower()
        if canonical in seen or re.sub(r"[-_.]+", "-", filename.split("-")[0]).lower() != canonical or filename.split("-")[1] != version:
            raise ManagerError("download_recipe_invalid", "轮子名称、版本或依赖唯一性不符。")
        seen.add(canonical)
    targets = set()
    for record in recipe["native"]:
        if not isinstance(record, dict):
            raise ManagerError("download_recipe_invalid", "原生下载记录必须是对象。")
        target = safe_relative(record.get("target"))
        if target.parts[0] not in ("lib", "lib64") or len(target.parts) < 2 or str(target) in targets or not record.get("name") or not record.get("version"):
            raise ManagerError("download_recipe_invalid", "原生依赖必须有固定名称、版本和lib/lib64目标。")
        targets.add(str(target))
    directories, settings = set(), set()
    for model in recipe["models"]:
        if not isinstance(model, dict):
            raise ManagerError("download_recipe_invalid", "模型配方必须是对象。")
        directory = safe_relative(model.get("directory"))
        if directory.parts[0] != "models" or len(directory.parts) < 2 or str(directory) in directories or not model.get("id"):
            raise ManagerError("download_recipe_invalid", "模型必须声明唯一稳定目录与固定身份。")
        directories.add(str(directory))
        if not isinstance(model.get("settings"), dict) or not model["settings"] or set(model["settings"]) - MODEL_SETTINGS or set(model["settings"]) & settings:
            raise ManagerError("download_recipe_invalid", "模型配方不允许覆盖任意设置或重复声明。")
        settings.update(model["settings"])
        for entry in model["settings"].values():
            safe_relative(entry, allow_dot=True)
        if not isinstance(model.get("files"), list) or not 0 < len(model["files"]) <= 100000:
            raise ManagerError("download_recipe_invalid", "模型必须提供逐文件下载清单。")
        files = set()
        for record in model["files"]:
            if not isinstance(record, dict):
                raise ManagerError("download_recipe_invalid", "模型文件记录必须是对象。")
            path = str(safe_relative(record.get("path")))
            if path in files:
                raise ManagerError("download_recipe_invalid", "模型文件重复。")
            files.add(path)
        if any(entry != "." and entry not in files for entry in model["settings"].values()):
            raise ManagerError("download_recipe_invalid", "模型设置入口没有逐文件下载声明。")
    for record in recipe["python"] + recipe["native"] + [item for model in recipe["models"] for item in model["files"]]:
        record["url"] = urllib.parse.urljoin(artifact_url, record.get("url", ""))
        validate_download(record)
    if "go2" in recipe:
        from scripts.go2_dependencies import validate_recipe
        try:
            validate_recipe(recipe["go2"])
        except ValueError as exc:
            raise ManagerError("download_recipe_invalid", str(exc)) from exc
    return recipe


def check_requirements(text, recipe):
    declared = {}
    for line in text.splitlines():
        text = line.strip()
        if not text or text.startswith("#"):
            continue
        match = re.fullmatch(r"([A-Za-z0-9][A-Za-z0-9._-]*)==([A-Za-z0-9][A-Za-z0-9.!+_-]*)", text)
        if not match:
            raise ManagerError("requirements_unsafe", "Python依赖必须使用清单中的固定名称和精确版本。")
        name = re.sub(r"[-_.]+", "-", match.group(1)).lower()
        if name in declared:
            raise ManagerError("requirements_unsafe", "Python依赖声明重复。")
        declared[name] = match.group(2)
    listed = {re.sub(r"[-_.]+", "-", record["name"]).lower(): record["version"] for record in recipe["python"]}
    if declared != listed:
        raise ManagerError("requirements_mismatch", "依赖声明与逐文件下载配方不一致。")


def _managed_model(path, home):
    for directory in (path, *path.parents):
        if directory == home / "data" or home not in directory.parents:
            break
        if (directory / ".neh-model.json").is_file():
            return True
    return False


def validate_model_entry(key, path):
    """只核对现有消费者的基本文件格式，不加载模型或宣称推理兼容。"""
    path = Path(path)
    if key == "NUWAX_PERCEPTION_YOLO_WORLD_MODEL":
        if not path.is_file() or path.stat().st_size == 0 or path.suffix.lower() not in (".pt", ".engine"):
            raise ManagerError("model_format_invalid", "当前视觉消费者需要非空的.pt或.engine模型文件。")
        if path.suffix.lower() == ".pt":
            import zipfile
            with path.open("rb") as handle:
                prefix = handle.read(2)
            if not zipfile.is_zipfile(path) and prefix[:1] != b"\x80":
                raise ManagerError("model_format_invalid", "视觉.pt文件不具备PyTorch归档或序列化格式。")
    else:
        if not path.is_dir():
            raise ManagerError("model_format_invalid", "本地记忆模型需要包含配置和权重的模型目录。")
        configurations = [path / "config.json", path / "modules.json"]
        readable = False
        for configuration in configurations:
            if configuration.is_file():
                try:
                    if configuration.stat().st_size > 8 * 1024 * 1024:
                        raise ValueError()
                    readable = isinstance(json.loads(configuration.read_text()), (dict, list)) or readable
                except (OSError, ValueError):
                    raise ManagerError("model_format_invalid", "自定义模型配置不是有效JSON。")
        if not readable or not any(item.is_file() and item.suffix.lower() in (".bin", ".pt", ".safetensors") for item in path.rglob("*")):
            raise ManagerError("model_format_invalid", "模型目录缺少现有记忆消费者所需的配置或权重文件。")


def prepare_dependencies(engine, candidate, manifest, artifact, existing, job):
    recipe = normalize_recipe(manifest, artifact["url"])
    check_requirements((candidate / "requirements.txt").read_text(), recipe)
    go2_sources = []
    if "go2" in recipe:
        from scripts.go2_dependencies import source_records
        go2_sources = source_records(recipe["go2"])
    dependency_bytes = sum(item["size"] for item in recipe["python"] + recipe["native"] + go2_sources)
    model_bytes = sum(item["size"] for model in recipe["models"] for item in model["files"])
    if ("dependency_bytes" in artifact and artifact["dependency_bytes"] != dependency_bytes) or ("model_bytes" in artifact and artifact["model_bytes"] != model_bytes):
        raise ManagerError("recipe_size_mismatch", "下载配方总量与已确认的版本目录不一致。")
    job["download_breakdown"] = {"application": artifact["size"], "dependencies": dependency_bytes, "models": model_bytes}
    if shutil.disk_usage(engine.home).free < 2 * dependency_bytes + 256 * 1024 ** 2:
        raise ManagerError("disk_space", "磁盘空间不足以下载并准备依赖和模型。")
    job["downloads"] = {"files": 0, "reused": 0, "downloaded_bytes": 0, "verified_bytes": 0}
    total = dependency_bytes + model_bytes
    completed = 0
    def fetch(record, label, filename=None):
        nonlocal completed
        engine._progress(job, "dependencies", 38 + int(24 * completed / max(total, 1)), "正在准备" + label)
        path, cached = cached_download(engine, record, filename=filename, cancelled=lambda: engine._cancelled(job["id"]),
                                       progress=lambda count, _: engine._progress(job, "dependencies", 38 + int(24 * (completed + count) / max(total, 1)), "正在准备" + label))
        completed += record["size"]
        job["downloads"]["files"] += 1
        job["downloads"]["reused"] += int(cached)
        job["downloads"]["verified_bytes"] += record["size"]
        job["downloads"]["downloaded_bytes"] += 0 if cached else record["size"]
        return path
    wheelhouse = candidate / "downloads/wheels"
    wheelhouse.mkdir(parents=True)
    for record in recipe["python"]:
        cached = fetch(record, "Python依赖 " + record["name"] + " " + record["version"], record["filename"])
        materialize_cached(cached, wheelhouse / record["filename"])
    for record in recipe["native"]:
        target = candidate / record["target"]
        if target.exists():
            raise ManagerError("native_conflict", "第三方依赖与自有发布文件冲突。")
        materialize_cached(fetch(record, "运行库 " + record["name"]), target, 0o644)
    for record in go2_sources:
        fetch(record, "Go2官方依赖 " + record["id"] + " " + record["version"], record["filename"])
    overrides = {}
    for model in recipe["models"]:
        preserved = {}
        for key, entry in model["settings"].items():
            value = existing.get(key, "").strip()
            if uses_default_model(value):
                continue
            path = Path(value)
            if not path.is_absolute() and engine.current():
                path = Path(engine.current()["prefix"]) / "source" / path
            if _managed_model(path, engine.home):
                continue
            if not path.exists() or (entry == "." and not path.is_dir()) or (entry != "." and not path.is_file()):
                raise ManagerError("custom_model_missing", "用户自定义模型路径无效，请在设置中修正：" + key)
            if path.is_dir() and not any(child.is_file() for child in path.rglob("*")):
                raise ManagerError("custom_model_missing", "用户自定义模型目录没有文件：" + key)
            validate_model_entry(key, path)
            preserved[key] = str(path)
        if preserved:
            if set(preserved) != set(model["settings"]):
                raise ManagerError("custom_model_incomplete", "同一模型配方的自定义入口不完整，请补齐设置。")
            overrides.update(preserved)
            skipped = sum(item["size"] for item in model["files"])
            job["download_breakdown"]["models"] -= skipped
            total -= skipped
            engine._progress(job, "models", 62, "保留自定义模型，实际功能在启动时验证：" + model["id"])
            continue
        identity = hashlib.sha256(json.dumps(model, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
        directory = engine.home / "data" / model["directory"] / identity[:20]
        verified = directory.is_dir() and all((directory / item["path"]).is_file() and (directory / item["path"]).stat().st_size == item["size"] and digest_file(directory / item["path"]) == item["sha256"].lower() for item in model["files"])
        if not verified:
            required = sum(item["size"] for item in model["files"])
            if shutil.disk_usage(engine.home).free < 2 * required + 256 * 1024 ** 2:
                raise ManagerError("disk_space", "磁盘空间不足以准备所选模型。")
            if directory.exists():
                directory = directory.with_name(directory.name + "-" + job["id"][:8])
            temporary = engine.home / "cache" / ("model-" + identity[:20] + "-" + job["id"][:8])
            temporary.mkdir(parents=True)
            try:
                for record in model["files"]:
                    materialize_cached(fetch(record, "模型 " + model["id"]), temporary / record["path"])
                atomic_json(temporary / ".neh-model.json", {"id": model["id"], "recipe_sha256": identity})
                directory.parent.mkdir(parents=True, exist_ok=True)
                temporary.rename(directory)
            except BaseException:
                shutil.rmtree(temporary)
                raise
        else:
            verified_size = sum(item["size"] for item in model["files"])
            completed += verified_size
            job["downloads"]["files"] += len(model["files"])
            job["downloads"]["reused"] += len(model["files"])
            job["downloads"]["verified_bytes"] += verified_size
            engine._progress(job, "models", 62, "复用逐文件校验通过的模型：" + model["id"])
        for key, entry in model["settings"].items():
            path = directory / entry
            if not path.exists():
                raise ManagerError("model_entry_missing", "模型配方声明的入口文件不存在。")
            validate_model_entry(key, path)
            overrides[key] = str(path)
    if "perception" in manifest["modules"] and "NUWAX_PERCEPTION_YOLO_WORLD_MODEL" not in overrides:
        raise ManagerError("model_missing", "视觉模块缺少模型下载或明确自定义入口。")
    if "memory" in manifest["modules"] and not {"NUWAX_MEMORY_TEXT_EMBEDDING_MODEL", "NUWAX_MEMORY_IMAGE_EMBEDDING_MODEL"}.issubset(overrides):
        raise ManagerError("model_missing", "记忆模块缺少文本或图像模型配方。")
    return wheelhouse, overrides


def apply_model_overrides(existing, overrides):
    """重新启用记忆模型时让原安装器恢复模型维度，不沿用停用时的占位维度。"""
    existing = dict(existing)
    for kind in ("TEXT", "IMAGE"):
        key = "NUWAX_MEMORY_" + kind + "_EMBEDDING_MODEL"
        dimension = "NUWAX_MEMORY_" + kind + "_EMBEDDING_DIM"
        if key in overrides and uses_default_model(existing.get(key, "")):
            remembered = existing.get("_NUWAX_INSTALLER_SAVED_" + dimension, "")
            if remembered:
                existing[dimension] = remembered
            else:
                existing.pop(dimension, None)
    existing.update(overrides)
    return existing
