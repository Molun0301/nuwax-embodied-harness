"""从官方归档安装Go2依赖；直接复用宇树原生库，仅在设备上生成Python适配缓存。"""
from __future__ import annotations

import email.parser
import fcntl
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import struct
import tarfile
import time
import urllib.parse
import uuid
import zipfile


SOURCES = {
    "unitree-sdk2-python": ("1.0.1", "unitreerobotics/unitree_sdk2_python"),
    "unitree-dds-native": ("0.10.2", "unitreerobotics/unitree_sdk2"),
    "cyclonedds-python": ("0.10.2", None),
}
WHEEL_VERSIONS = {"unitree-sdk2py": "1.0.1", "cyclonedds": "0.10.2"}


def digest(path):
    value = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def validate_recipe(recipe):
    if (not isinstance(recipe, dict) or set(recipe) != {"schema_version", "sources"}
            or type(recipe.get("schema_version")) is not int or recipe["schema_version"] != 1):
        raise ValueError("Go2官方依赖配方结构无效")
    records = recipe.get("sources")
    if not isinstance(records, list) or len(records) != len(SOURCES):
        raise ValueError("Go2官方依赖必须包含SDK、DDS原生库和Python接口")
    seen = set()
    for record in records:
        if not isinstance(record, dict) or set(record) - {"id", "version", "filename", "url", "sha256", "size", "commit"}:
            raise ValueError("Go2官方下载记录包含未知字段")
        identity = record.get("id")
        if not isinstance(identity, str) or identity not in SOURCES or identity in seen:
            raise ValueError("Go2官方依赖身份重复或未知")
        seen.add(identity)
        version, repository = SOURCES[identity]
        if record.get("version") != version:
            raise ValueError("Go2官方依赖版本与适配接口不一致")
        if not isinstance(record.get("size"), int) or isinstance(record["size"], bool) or not 0 < record["size"] <= 128 * 1024 * 1024:
            raise ValueError("Go2官方归档大小无效")
        if not re.fullmatch(r"[0-9a-f]{64}", str(record.get("sha256", ""))):
            raise ValueError("Go2官方归档缺少固定摘要")
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*\.tar\.gz", str(record.get("filename", ""))):
            raise ValueError("Go2官方归档文件名无效")
        if not isinstance(record.get("url"), str):
            raise ValueError("Go2官方依赖缺少下载地址")
        url = urllib.parse.urlsplit(record["url"])
        if url.scheme != "https" or url.username or url.password or url.query or url.fragment or url.port:
            raise ValueError("Go2官方依赖必须使用固定HTTPS地址")
        if repository:
            commit = record.get("commit", "")
            if not re.fullmatch(r"[0-9a-f]{40}", str(commit)) or url.netloc != "codeload.github.com" or url.path != "/" + repository + "/tar.gz/" + commit:
                raise ValueError("Go2官方仓库与下载修订不匹配")
        elif (url.netloc != "files.pythonhosted.org" or not url.path.startswith("/packages/")
              or not url.path.endswith("/cyclonedds-0.10.2.tar.gz") or "commit" in record):
            raise ValueError("CycloneDDS Python必须来自官方固定PyPI归档")
    return recipe


def load_recipe(root):
    return validate_recipe(json.loads((Path(root) / "third_party/go2-online-sources.json").read_text(encoding="utf-8")))


def source_records(recipe):
    return list(validate_recipe(recipe)["sources"])


def _extract(archive, destination, include=lambda path: True):
    """逐文件解包，只写普通文件；官方库的固定别名由安装器自行创建。"""
    total, count, seen = 0, 0, set()
    with tarfile.open(archive, "r:gz") as stream:
        for member in stream:
            parts = PurePosixPath(member.name).parts
            if member.name.startswith("/") or "\\" in member.name or ".." in parts:
                raise ValueError("官方依赖归档包含越界路径")
            if len(parts) < 2:
                continue
            relative = PurePosixPath(*parts[1:])
            if not include(relative):
                continue
            if member.isdir():
                continue
            if not member.isfile() or relative in seen:
                raise ValueError("官方依赖归档包含重复或非普通文件")
            count += 1
            total += member.size
            if count > 50000 or total > 512 * 1024 * 1024 or member.size > 128 * 1024 * 1024:
                raise ValueError("官方依赖解包超出大小预算")
            seen.add(relative)
            target = destination.joinpath(*relative.parts)
            target.parent.mkdir(parents=True, exist_ok=True)
            with stream.extractfile(member) as source, target.open("xb") as output:
                shutil.copyfileobj(source, output)
            target.chmod(0o755 if member.mode & 0o111 else 0o644)


def _elf_machine(data, architecture):
    expected = {"aarch64": 183, "x86_64": 62}.get(architecture)
    if expected is None or len(data) < 20 or data[:4] != b"\x7fELF" or data[4:6] != b"\x02\x01" or struct.unpack("<H", data[18:20])[0] != expected:
        raise ValueError("官方原生库或Python适配文件架构不匹配")


def _check_wheel(path, architecture):
    with zipfile.ZipFile(path) as archive:
        metadata = [name for name in archive.namelist() if name.endswith(".dist-info/METADATA")]
        if len(metadata) != 1:
            raise ValueError("Go2本地适配轮子缺少唯一发行元数据")
        package = email.parser.Parser().parsestr(archive.read(metadata[0]).decode())
        name = re.sub(r"[-_.]+", "-", package["Name"] or "").lower()
        if WHEEL_VERSIONS.get(name) != package["Version"]:
            raise ValueError("Go2本地适配轮子版本不符")
        native = []
        for member in archive.infolist():
            if member.filename.endswith(".so") or ".so." in member.filename:
                with archive.open(member) as handle:
                    _elf_machine(handle.read(20), architecture)
                native.append(member.filename)
        if name == "cyclonedds" and not any(member.startswith("cyclonedds.libs/libddsc") for member in native):
            raise ValueError("Python适配缓存没有保留下载的官方DDS库")
        return name


def _cached_wheels(directory, architecture):
    try:
        record = json.loads((directory / "cache.json").read_text())
        wheels = []
        seen = set()
        if len(record["wheels"]) != 2:
            return None
        for item in record["wheels"]:
            filename = item["filename"]
            if Path(filename).name != filename or not filename.endswith(".whl"):
                return None
            path = directory / "wheels" / filename
            if path.is_symlink() or not path.is_file() or path.stat().st_size != item["size"] or digest(path) != item["sha256"]:
                return None
            seen.add(_check_wheel(path, architecture))
            wheels.append(path)
        return wheels if seen == set(WHEEL_VERSIONS) else None
    except (OSError, ValueError, KeyError, TypeError, zipfile.BadZipFile):
        return None


def install(prefix, cache, recipe, *, fetch, run, environment, cancelled=lambda: None, progress=lambda message: None):
    """下载官方依赖，在当前设备首次生成Python适配，后续安装复用完整性校验缓存。"""
    validate_recipe(recipe)
    prefix, cache = Path(prefix), Path(cache)
    interpreter = str(prefix / "venv/bin/python")
    target = json.loads(run([interpreter, "-c", "import sys,platform,json; print(json.dumps({'python':list(sys.version_info[:2]),'architecture':platform.machine(),'system':platform.system(),'libc':list(platform.libc_ver())}))"],
                            environment=environment, cancelled=cancelled))
    architecture = target["architecture"]
    if target["system"] != "Linux" or architecture not in ("aarch64", "x86_64") or target["python"] != [3, 8]:
        raise ValueError("此Go2官方依赖组合仅验证Python3.8和Linux的ARM64/x86_64安装")
    cache.mkdir(parents=True, exist_ok=True)
    identity = hashlib.sha256(json.dumps({"recipe": recipe, "target": target}, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
    final = cache / identity
    if final.is_symlink():
        raise ValueError("Go2依赖缓存不能是符号链接")
    with (cache / ".lock").open("a+b") as lock:
        deadline = time.monotonic() + 1800
        while True:
            cancelled()
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                break
            except BlockingIOError:
                if time.monotonic() >= deadline:
                    raise RuntimeError("等待Go2官方依赖安装锁超时")
                time.sleep(0.2)
        wheels = _cached_wheels(final, architecture)
        reused = wheels is not None
        if wheels is None:
            staging = cache / (identity + ".prepare-" + uuid.uuid4().hex)
            staging.mkdir()
            try:
                downloads = {}
                for record in source_records(recipe):
                    cancelled()
                    progress("下载并校验官方依赖：" + record["id"])
                    path = Path(fetch(record))
                    if path.stat().st_size != record["size"] or digest(path) != record["sha256"]:
                        raise ValueError("官方Go2依赖下载内容不匹配")
                    downloads[record["id"]] = path
                native_source = staging / "official-native"
                library = PurePosixPath("thirdparty/lib") / architecture / "libddsc.so"
                _extract(downloads["unitree-dds-native"], native_source,
                         lambda path: path == library or path.parts[:3] == ("thirdparty", "include", "dds"))
                header = native_source / "thirdparty/include/dds/version.h"
                if not header.is_file() or '#define DDS_VERSION "0.10.2"' not in header.read_text():
                    raise ValueError("下载的官方DDS头文件版本不匹配")
                native = staging / "native"
                (native / "lib").mkdir(parents=True)
                # 官方Python构建探测同时要求bin目录，运行接口不需要额外idlc程序。
                (native / "bin").mkdir()
                shutil.copytree(native_source / "thirdparty/include", native / "include")
                binary = native_source.joinpath(*library.parts)
                with binary.open("rb") as handle:
                    _elf_machine(handle.read(20), architecture)
                shutil.copyfile(binary, native / "lib/libddsc.so")
                (native / "lib/libddsc.so.0").symlink_to("libddsc.so")
                output = staging / "wheels"
                output.mkdir()
                build_environment = dict(environment, CYCLONEDDS_HOME=str(native), STANDALONE_WHEELS="1", SOURCE_DATE_EPOCH="315532800")
                build_environment["LD_LIBRARY_PATH"] = str(native / "lib") + ":" + environment.get("LD_LIBRARY_PATH", "")
                build_environment["PATH"] = str(prefix / "venv/bin") + os.pathsep + environment.get("PATH", os.defpath)
                for name in ("cyclonedds-python", "unitree-sdk2-python"):
                    source = staging / name
                    source.mkdir()
                    _extract(downloads[name], source)
                    progress("按官方流程安装Python适配并缓存：" + name)
                    run([interpreter, "-m", "pip", "--isolated", "wheel", "--no-cache-dir", "--no-index", "--no-deps", "--no-build-isolation", "--wheel-dir", str(output), str(source)],
                        cwd=source, environment=build_environment, timeout=1800, cancelled=cancelled)
                # 官方独立轮子加载器使用auditwheel的库目录；按其Linux发布步骤修复链接。
                original = next(output.glob("cyclonedds-*.whl"))
                repaired = staging / "repaired"
                run([interpreter, "-m", "auditwheel", "repair", "--plat", "manylinux_2_31_" + architecture,
                     "--wheel-dir", str(repaired), str(original)],
                    environment=build_environment, timeout=600, cancelled=cancelled)
                replacements = list(repaired.glob("cyclonedds-*.whl"))
                if len(replacements) != 1:
                    raise ValueError("官方Python适配未生成唯一可搬迁轮子")
                original.unlink()
                shutil.move(str(replacements[0]), str(output / replacements[0].name))
                artifacts = []
                for wheel in sorted(output.glob("*.whl")):
                    _check_wheel(wheel, architecture)
                    artifacts.append({"filename": wheel.name, "sha256": digest(wheel), "size": wheel.stat().st_size})
                (staging / "cache.json").write_text(json.dumps({"sources": recipe, "target": target, "wheels": artifacts}, sort_keys=True))
                if _cached_wheels(staging, architecture) is None:
                    raise ValueError("Go2官方依赖未生成完整缓存")
                # 缓存只保留可重用的完整轮子；构建目录及原生临时目录均可回收。
                for child in staging.iterdir():
                    if child.name not in ("wheels", "cache.json"):
                        shutil.rmtree(child) if child.is_dir() else child.unlink()
                if final.exists():
                    shutil.rmtree(final)
                staging.rename(final)
                wheels = _cached_wheels(final, architecture)
            finally:
                if staging.exists():
                    shutil.rmtree(staging)
        else:
            progress("复用已校验的Go2官方依赖安装缓存")
        cancelled()
        run([interpreter, "-m", "pip", "--isolated", "install", "--no-index", "--no-deps", "--force-reinstall", *[str(wheel) for wheel in wheels]],
            environment=environment, timeout=600, cancelled=cancelled)
        return {"reused": reused, "python": target["python"], "architecture": architecture, "sources": [item["id"] for item in recipe["sources"]]}
