"""发布包的封闭文件清单、归档边界和实际目标平台校验。"""
from __future__ import annotations

import hashlib
import json
import platform
import re
import sys
import tarfile
from pathlib import Path, PurePosixPath

MAX_FILES = 100000
MAX_UNPACKED = 40 * 1024 ** 3
MAX_MANIFEST = 32 * 1024 ** 2


class ReleaseError(RuntimeError):
    """发布产物不能满足安装契约。"""


def digest(path: Path) -> str:
    value = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def safe_path(value: str) -> PurePosixPath:
    if not isinstance(value, str) or not value or "\\" in value or any(ord(character) < 32 or ord(character) == 127 for character in value):
        raise ReleaseError("非法归档路径")
    path = PurePosixPath(value)
    if path.is_absolute() or ".." in path.parts or path.as_posix() != value or value == ".":
        raise ReleaseError("归档路径逃逸或非规范路径：" + value)
    return path


def read_json(data: bytes) -> dict:
    def pairs(items):
        output = {}
        for key, value in items:
            if key in output:
                raise ReleaseError("JSON包含重复字段：" + key)
            output[key] = value
        return output
    try:
        value = json.loads(data, object_pairs_hook=pairs)
    except (ValueError, UnicodeError) as exc:
        raise ReleaseError("清单不是有效JSON：" + str(exc)) from exc
    if not isinstance(value, dict):
        raise ReleaseError("清单必须是对象")
    return value


def host_platform() -> dict:
    os_info = {}
    for line in Path("/etc/os-release").read_text(encoding="utf-8").splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            os_info[key] = value.strip('"')
    architecture = {"amd64": "x86_64", "arm64": "aarch64"}.get(platform.machine().lower(), platform.machine().lower())
    libc_name, libc_version = platform.libc_ver()
    return {
        "system": platform.system().lower(), "architecture": architecture,
        "os_ids": [os_info.get("ID", "")], "os_versions": [os_info.get("VERSION_ID", "")],
        "python": "%s.%s" % sys.version_info[:2],
        "libc": {"name": libc_name, "minimum": libc_version},
    }


def validate_platform(value: dict, *, match_host: bool = True) -> None:
    if not isinstance(value, dict) or value.get("system") != "linux":
        raise ReleaseError("发布目标目前只支持Linux")
    if value.get("architecture") not in ("x86_64", "aarch64"):
        raise ReleaseError("发布目标架构无效")
    if not re.fullmatch(r"3\.(8|9|10|11|12)", str(value.get("python", ""))):
        raise ReleaseError("业务解释器须为Python3.8—3.12")
    for key in ("os_ids", "os_versions"):
        if not isinstance(value.get(key), list) or not value[key] or any(not isinstance(item, str) or not re.fullmatch(r"[A-Za-z0-9._-]+", item) for item in value[key]):
            raise ReleaseError("发布目标必须声明有效发行版及版本")
    if set(value["os_ids"]) - {"ubuntu", "debian"}:
        raise ReleaseError("当前系统依赖适配器仅支持Ubuntu和Debian")
    if not isinstance(value.get("libc"), dict) or value["libc"].get("name") != "glibc" or not value["libc"].get("minimum"):
        raise ReleaseError("发布目标必须声明glibc最低版本")
    if match_host and value != host_platform():
        raise ReleaseError("声明目标与实际构建环境不符；请在对应架构、发行版和解释器的构建机上执行")


def elf_architecture(path: Path) -> str:
    with path.open("rb") as handle:
        header = handle.read(20)
    if len(header) < 20 or header[:4] != b"\x7fELF" or header[4] != 2 or header[5] not in (1, 2):
        raise ReleaseError("缺少有效64位ELF原生程序：" + str(path))
    machine = int.from_bytes(header[18:20], "little" if header[5] == 1 else "big")
    try:
        return {62: "x86_64", 183: "aarch64"}[machine]
    except KeyError as exc:
        raise ReleaseError("不支持的ELF目标架构：" + str(machine)) from exc


def elf_interpreter(path: Path) -> str:
    """检查真实ELF装载器，避免将构建机私有工具链路径带到设备。"""
    with path.open("rb") as handle:
        header = handle.read(64)
        if len(header) != 64 or header[:4] != b"\x7fELF" or header[4] != 2 or header[5] not in (1, 2):
            raise ReleaseError("原生文件的ELF头不完整：" + str(path))
        order = "little" if header[5] == 1 else "big"
        offset = int.from_bytes(header[32:40], order)
        entry_size = int.from_bytes(header[54:56], order)
        count = int.from_bytes(header[56:58], order)
        if count > 128 or (count and entry_size < 56) or offset + entry_size * count > path.stat().st_size:
            raise ReleaseError("原生文件的ELF程序头无效：" + str(path))
        for index in range(count):
            handle.seek(offset + index * entry_size)
            entry = handle.read(entry_size)
            if int.from_bytes(entry[:4], order) != 3:
                continue
            start, size = int.from_bytes(entry[8:16], order), int.from_bytes(entry[32:40], order)
            if not 1 < size < 4096 or start + size > path.stat().st_size:
                raise ReleaseError("原生文件的装载器路径无效：" + str(path))
            handle.seek(start)
            return handle.read(size).rstrip(b"\0").decode("utf-8")
    return ""


def verify_archive(path: Path, *, manifest_name: str = "manifest.json", max_bytes: int = MAX_UNPACKED) -> dict:
    """不解压先逐文件验证，拒绝遗漏、额外文件和任何链接。"""
    safe_path(manifest_name)
    try:
        with tarfile.open(path, "r:gz") as archive:
            members = {}
            total = 0
            for member in archive:
                name = member.name.rstrip("/") if member.isdir() else member.name
                safe_path(name)
                if name in members:
                    raise ReleaseError("归档包含重复路径：" + name)
                if not (member.isfile() or member.isdir()) or member.issparse():
                    raise ReleaseError("归档包含链接或特殊文件：" + name)
                total += member.size
                if total > max_bytes or len(members) >= MAX_FILES:
                    raise ReleaseError("归档超过允许的文件数量或解压体积")
                members[name] = member
            selected = members.get(manifest_name)
            if selected is None or not selected.isfile() or selected.size > MAX_MANIFEST:
                raise ReleaseError("归档缺少有效清单：" + manifest_name)
            manifest = read_json(archive.extractfile(selected).read())
            if manifest.get("schema_version") != (1 if manifest_name == "manager-manifest.json" else 2) or manifest.get("product") != ("neh-manager" if manifest_name == "manager-manifest.json" else "neh"):
                raise ReleaseError("不支持的产品或清单版本")
            if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._+-]{0,95}", str(manifest.get("version", ""))):
                raise ReleaseError("发布版本无效")
            if not re.fullmatch(r"[0-9a-f]{40,64}", str(manifest.get("revision", ""))):
                raise ReleaseError("发布源码修订无效")
            declared = {}
            if not isinstance(manifest.get("files"), list):
                raise ReleaseError("清单缺少文件列表")
            for item in manifest["files"]:
                if not isinstance(item, dict):
                    raise ReleaseError("文件声明无效")
                name = item.get("path")
                safe_path(name)
                if name == manifest_name or name in declared:
                    raise ReleaseError("清单重复声明文件：" + name)
                size = item.get("size")
                if isinstance(size, bool) or not isinstance(size, int) or size < 0 or not re.fullmatch(r"[0-9a-f]{64}", str(item.get("sha256", ""))):
                    raise ReleaseError("文件摘要或大小无效：" + name)
                declared[name] = item
            actual = {name for name, member in members.items() if member.isfile() and name != manifest_name}
            if actual != set(declared):
                raise ReleaseError("归档文件与清单不一致：未声明=%s，缺失=%s" % (sorted(actual - set(declared)), sorted(set(declared) - actual)))
            for name, item in declared.items():
                member = members[name]
                if member.size != item["size"]:
                    raise ReleaseError("文件大小不符：" + name)
                checksum = hashlib.sha256()
                handle = archive.extractfile(member)
                for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                    checksum.update(chunk)
                if checksum.hexdigest() != item["sha256"]:
                    raise ReleaseError("文件摘要不符：" + name)
            for name, member in members.items():
                if member.isdir() and not any(p.startswith(name + "/") for p in declared):
                    raise ReleaseError("归档包含未声明的空目录：" + name)
                if any(parent.as_posix() in actual for parent in PurePosixPath(name).parents if parent.as_posix() != "."):
                    raise ReleaseError("归档文件与目录路径冲突：" + name)
            if manifest_name == "manifest.json":
                from scripts.module_installer.catalog import resolve_modules
                from .recipes import validate_install
                validate_platform(manifest.get("platform"), match_host=False)
                modules = manifest.get("modules")
                if not isinstance(modules, list) or modules != list(resolve_modules(modules)):
                    raise ReleaseError("发布模块缺少依赖或存在重复项")
                if manifest.get("data_schema") != 1 or manifest.get("robot") not in ("customrobot", "go2"):
                    raise ReleaseError("发布数据格式或机器人接入无效")
                if not isinstance(manifest.get("system_packages"), list) or any(not isinstance(item, str) or not re.fullmatch(r"[a-z0-9][a-z0-9+.-]+", item) for item in manifest["system_packages"]):
                    raise ReleaseError("系统依赖清单无效")
                if any(PurePosixPath(name).parts[0] not in ("source", "payload", "requirements.txt") for name in actual):
                    raise ReleaseError("业务归档包含契约之外的文件")
                validate_install(manifest.get("install"), modules)
                if (manifest["robot"] == "go2") != ("go2" in manifest["install"]):
                    raise ReleaseError("Go2机器人身份与官方安装配方不一致")
                for name in actual:
                    path_name = PurePosixPath(name)
                    if name.startswith("source/") and (path_name.name.lower().endswith((".whl", ".download", ".deb", ".rpm", ".pt", ".pth", ".onnx", ".safetensors", ".gguf", ".ckpt", ".h5", ".hdf5", ".engine", ".trt", ".bin", ".dll", ".dylib", ".a", ".o")) or re.search(r"\.so(?:\.\d+)*$", path_name.name.lower())):
                        raise ReleaseError("业务源码夹带第三方二进制或模型：" + name)
                    if name.startswith("payload/"):
                        if len(path_name.parts) != 3 or path_name.parts[1] not in ("bin", "lib", "lib64"):
                            raise ReleaseError("自有产物路径无效：" + name)
                        if path_name.parts[1] in ("lib", "lib64") and not path_name.name.startswith("libnuwax"):
                            raise ReleaseError("业务包不能携带第三方原生库：" + name)
                        if path_name.parts[1] == "bin" and path_name.name not in {"nuwax-geometry-runtime", "nuwax-mapping-cli", "nuwax-nav-stack", "nuwax-person-follow", "nuwax-slam-2d-worker", "fastlio2_native", "pointlio_native"}:
                            raise ReleaseError("业务包包含未声明为自有的程序：" + name)
                required = {"source/scripts/start_gateway.sh", "source/scripts/module_installer/runtime.py", "source/core/config_values.py", "requirements.txt"}
                if not required.issubset(actual):
                    raise ReleaseError("业务包缺少源码入口或精确依赖声明")
                if members["requirements.txt"].size > 1024 ** 2:
                    raise ReleaseError("精确依赖声明体积异常")
                requirements = archive.extractfile(members["requirements.txt"]).read().decode("utf-8")
                expected = {item["name"] + "==" + item["version"] for item in manifest["install"]["python"]}
                lines = [line.strip() for line in requirements.splitlines() if line.strip() and not line.lstrip().startswith("#")]
                if set(lines) != expected or len(lines) != len(expected):
                    raise ReleaseError("精确依赖声明与下载配方不一致")
            elif manifest.get("python") != {"minimum": "3.8", "maximum_exclusive": "3.13"}:
                raise ReleaseError("管理包解释器范围无效")
            return manifest
    except (tarfile.TarError, EOFError, OSError) as exc:
        raise ReleaseError("归档无法读取：" + str(exc)) from exc
