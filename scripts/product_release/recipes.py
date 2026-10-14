"""在线安装的固定下载配方；依赖和模型永不进入业务归档。"""
from __future__ import annotations

import json
import re
import shutil
import subprocess
import sys
import urllib.parse
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from .validation import ReleaseError, digest, safe_path

MODEL_SETTINGS = {
    "NUWAX_PERCEPTION_YOLO_WORLD_MODEL", "NUWAX_MEMORY_TEXT_EMBEDDING_MODEL",
    "NUWAX_MEMORY_IMAGE_EMBEDDING_MODEL", "NUWAX_MEMORY_EMBEDDING_MODEL",
}


def canonical_name(name: str) -> str:
    return re.sub(r"[-_.]+", "-", name).lower()


def validate_url(value: str) -> None:
    if not isinstance(value, str) or not value or any(ord(character) < 32 or ord(character) == 127 for character in value):
        raise ReleaseError("依赖下载地址不能为空")
    parsed = urllib.parse.urlsplit(value)
    if parsed.username or parsed.password or parsed.fragment:
        raise ReleaseError("依赖地址不能包含凭据或片段")
    if not parsed.scheme:
        if parsed.netloc or parsed.query:
            raise ReleaseError("相对依赖地址必须是安全文件路径")
        safe_path(urllib.parse.unquote(value))
    elif not parsed.hostname or (parsed.scheme != "https" and not (parsed.scheme == "http" and parsed.hostname in {"127.0.0.1", "localhost", "::1"})):
        raise ReleaseError("依赖下载地址必须为HTTPS或本机验收地址")


def _download_entry(item: dict) -> None:
    if not isinstance(item, dict):
        raise ReleaseError("下载配方条目必须是对象")
    validate_url(item.get("url"))
    if not re.fullmatch(r"[0-9a-f]{64}", str(item.get("sha256", ""))):
        raise ReleaseError("下载配方缺少SHA256摘要")
    size = item.get("size")
    if not isinstance(size, int) or isinstance(size, bool) or size < 0:
        raise ReleaseError("下载配方文件大小无效")


def validate_install(install: dict, modules=()) -> dict:
    if not isinstance(install, dict) or not {"python", "native", "models"}.issubset(install) or set(install) - {"python", "native", "models", "go2"} or any(not isinstance(install[key], list) for key in ("python", "native", "models")):
        raise ReleaseError("安装配方必须包含python/native/models三个列表")
    if "go2" in install:
        from scripts.go2_dependencies import validate_recipe
        try:
            validate_recipe(install["go2"])
        except ValueError as exc:
            raise ReleaseError(str(exc)) from exc
    names, filenames, targets = set(), set(), set()
    for item in install["python"]:
        _download_entry(item)
        name, version, filename = item.get("name"), item.get("version"), item.get("filename")
        if not isinstance(name, str) or not re.fullmatch(r"[A-Za-z0-9_.-]+", name) or not isinstance(version, str) or not re.fullmatch(r"[0-9][A-Za-z0-9_.+!-]*", version):
            raise ReleaseError("Python依赖名称或精确版本无效")
        if not isinstance(filename, str) or safe_path(filename).name != filename or not filename.endswith(".whl"):
            raise ReleaseError("Python依赖必须提供原始轮子文件名")
        key = canonical_name(name)
        if key in names or filename in filenames:
            raise ReleaseError("Python下载配方包含重复依赖")
        names.add(key)
        filenames.add(filename)
    if not names:
        raise ReleaseError("业务包不能缺少Python下载配方")
    for item in install["native"]:
        _download_entry(item)
        target = safe_path(item.get("target"))
        if len(target.parts) != 2 or target.parts[0] not in {"lib", "lib64"} or target.name.startswith("libnuwax"):
            raise ReleaseError("第三方原生依赖只能写入lib/lib64中的独立文件")
        if not isinstance(item.get("name"), str) or not item["name"] or not isinstance(item.get("version"), str) or not item["version"] or target.as_posix() in targets:
            raise ReleaseError("原生依赖身份、固定版本或目标无效")
        targets.add(target.as_posix())
    model_ids, directories, setting_keys = set(), set(), set()
    for model in install["models"]:
        if not isinstance(model, dict) or not isinstance(model.get("id"), str) or not model["id"] or model["id"] in model_ids:
            raise ReleaseError("模型配方必须提供唯一固定修订身份")
        directory = safe_path(model.get("directory"))
        if len(directory.parts) < 2 or directory.parts[0] != "models" or directory.as_posix() in directories:
            raise ReleaseError("模型目录必须是models下唯一安全路径")
        settings = model.get("settings")
        if not isinstance(settings, dict) or not settings or set(settings) - MODEL_SETTINGS or set(settings) & setting_keys:
            raise ReleaseError("模型设置只能引用明确允许且不重复的模型配置键")
        for value in settings.values():
            if value != ".":
                safe_path(value)
        files = model.get("files")
        if not isinstance(files, list) or not files:
            raise ReleaseError("模型配方缺少固定文件列表")
        paths = set()
        for item in files:
            _download_entry(item)
            path = safe_path(item.get("path")).as_posix()
            if path in paths:
                raise ReleaseError("模型清单含重复文件")
            paths.add(path)
            parsed = urllib.parse.urlsplit(item["url"])
            if parsed.hostname == "huggingface.co" and "/resolve/" in parsed.path and not re.search(r"/resolve/[0-9a-f]{40}/", parsed.path):
                raise ReleaseError("官方模型下载地址必须固定完整修订，不能使用main/latest")
        for key, value in settings.items():
            if value != "." and value not in paths and not any(path.startswith(value + "/") for path in paths):
                raise ReleaseError("模型设置入口没有对应文件：" + key)
        model_ids.add(model["id"])
        directories.add(directory.as_posix())
        setting_keys.update(settings)
    required = set()
    if "perception" in modules:
        required.add("NUWAX_PERCEPTION_YOLO_WORLD_MODEL")
    if "memory" in modules:
        required.update({"NUWAX_MEMORY_TEXT_EMBEDDING_MODEL", "NUWAX_MEMORY_IMAGE_EMBEDDING_MODEL"})
    if not required.issubset(setting_keys):
        raise ReleaseError("所选模块缺少模型下载配方：" + ",".join(sorted(required - setting_keys)))
    return install


def dependency_bytes(install: dict) -> int:
    total = sum(item["size"] for item in install["python"] + install["native"])
    if "go2" in install:
        from scripts.go2_dependencies import source_records
        total += sum(item["size"] for item in source_records(install["go2"]))
    return total


def copy_asset(source: Path, destination: Path) -> None:
    if source.is_symlink() or not source.is_file():
        raise ReleaseError("独立依赖资产必须是普通文件：" + str(source))
    if source.resolve() == destination.resolve():
        return
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        if destination.is_symlink() or digest(source) != digest(destination):
            raise ReleaseError("同名独立依赖资产内容不一致：" + destination.name)
        return
    shutil.copyfile(source, destination)
    destination.chmod(0o644)


def _official_wheel(record: dict) -> dict:
    url = "https://pypi.org/pypi/%s/%s/json" % (urllib.parse.quote(record["name"], safe=""), urllib.parse.quote(record["version"], safe=""))
    request = urllib.request.Request(url, headers={"Accept": "application/json", "User-Agent": "NEH-release/2"})
    try:
        with urllib.request.urlopen(request, timeout=30) as response:
            data = json.load(response)
    except Exception as exc:
        raise ReleaseError("无法核对官方依赖元数据：%s；%s" % (record["name"], exc)) from exc
    same_name = [item for item in data.get("urls", []) if item.get("filename") == record["filename"]]
    if same_name:
        item = same_name[0]
        if item.get("digests", {}).get("sha256") != record["sha256"]:
            raise ReleaseError("本地轮子与官方同名文件摘要不同：" + record["filename"])
        return {"url": item["url"], "size": item["size"], "origin": {"provider": "pypi", "metadata_url": url}}
    source = next((item for item in data.get("urls", []) if item.get("packagetype") == "sdist"), None)
    if source is None:
        raise ReleaseError("上游没有目标轮子或固定源码，须提供已核准的独立下载配方：" + record["filename"])
    source_info = {"url": source["url"], "sha256": source["digests"]["sha256"]}
    # pip按文件名区分大小写匹配；PyAudio等发行名与实际轮子名大小写不同。
    cached = subprocess.run([sys.executable, "-m", "pip", "cache", "list", record["filename"][:-4], "--format=abspath"], capture_output=True, text=True, timeout=30)
    for raw in cached.stdout.splitlines():
        wheel = Path(raw.strip())
        origin_path = wheel.parent / "origin.json"
        if wheel.name != record["filename"] or not origin_path.is_file() or digest(wheel) != record["sha256"]:
            continue
        origin = json.loads(origin_path.read_text(encoding="utf-8"))
        if origin.get("archive_info", {}).get("hashes", {}).get("sha256") == source_info["sha256"]:
            return {"origin": {"provider": "publisher-built", "metadata_url": url, "source": source_info, "build_origin": origin}}
    raise ReleaseError("本地自建轮子缺少与官方源码摘要一致的构建来源记录：" + record["filename"])


def python_recipe(records: list, wheelhouse: Path, assets: Path, *, local_assets: bool = False, approved_sources: dict = None) -> tuple:
    """优先锁定官方文件；缺少目标轮子时仅生成包外独立下载资产。"""
    def metadata_for(record):
        approved = (approved_sources or {}).get(canonical_name(record["name"]))
        if approved:
            if approved.get("status") != "ready" or approved.get("integrity", {}).get("wheel_sha256") != record["sha256"]:
                raise ReleaseError("厂商轮子尚未批准或摘要不符：" + record["name"])
            return {"origin": {"provider": "approved-source", "source": approved["source"], "integrity": approved["integrity"]}}
        return _official_wheel(record)
    with ThreadPoolExecutor(max_workers=4) as pool:
        metadata = list(pool.map(metadata_for, records))
    output, origins = [], []
    for record, official in zip(records, metadata):
        path = wheelhouse / record["filename"]
        size = path.stat().st_size
        if "size" in official and official["size"] != size:
            raise ReleaseError("本地轮子与官方大小不符：" + record["filename"])
        if local_assets or "url" not in official:
            copy_asset(path, assets / path.name)
            url = path.name
        else:
            url = official["url"]
        output.append({"name": record["name"], "version": record["version"], "filename": path.name, "url": url, "sha256": record["sha256"], "size": size})
        origins.append({"filename": path.name, "sha256": record["sha256"], **official["origin"]})
    return output, origins


def relative_assets(install: dict) -> list:
    values = list(install["python"]) + list(install["native"])
    # 模型来自固定上游修订，不能作为NEH发布附件携带。
    if any(not urllib.parse.urlsplit(item["url"]).scheme for model in install["models"] for item in model["files"]):
        raise ReleaseError("模型文件必须来自独立下载源，不能放入NEH发布附件")
    result = {}
    for item in values:
        parsed = urllib.parse.urlsplit(item["url"])
        if not parsed.scheme:
            name = safe_path(urllib.parse.unquote(item["url"])).as_posix()
            if "/" in name:
                raise ReleaseError("发布附件相对地址必须为同级文件名")
            if name in {"catalog.json", "manager.tar.gz", "release-plan.json", "release-index.json", "receipt.json", "get-neh.sh", "neh_bootstrap.py"} or name.endswith((".tar.gz", ".manifest.json", ".dependency-origins.json")):
                raise ReleaseError("依赖附件不能覆盖NEH业务包或发布元数据：" + name)
            if name in result and (result[name]["sha256"], result[name]["size"]) != (item["sha256"], item["size"]):
                raise ReleaseError("相同依赖附件地址声明了不同内容")
            result[name] = {"path": name, "sha256": item["sha256"], "size": item["size"]}
    return list(result.values())
