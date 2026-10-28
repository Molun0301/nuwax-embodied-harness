"""云端与设备共用的程序指纹、匿名制品索引及逐程序复用。"""
from __future__ import annotations

import ctypes
import ctypes.util
import hashlib
import json
import os
import platform
import re
import shlex
import shutil
import stat
import subprocess
import tarfile
import tempfile
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path, PurePosixPath
from typing import Iterable, Mapping, Optional, Sequence

RELEASE = "neh-prebuilt-v1"
SCHEMA = 2
NATIVE_ROOT = "services/nuwax-workspace/crates/nuwax-lio-backends/native"
WORKSPACE = "services/nuwax-workspace"
# 每个条目与实际 build_modules 配方相同；映射工具属于几何模块。
LOCAL_PROGRAMS = {
    "fastlio2_native": {"module": "fastlio2", "native": "fastlio2"},
    "pointlio_native": {"module": "pointlio", "native": "pointlio"},
}
PROGRAMS = {
    "nuwax-slam-2d-worker": {"module": "slam2d", "manifest": WORKSPACE + "/Cargo.toml", "package": "nuwax-slam-2d-runtime", "features": ["worker"]},
    "nuwax-geometry-runtime": {"module": "geometry", "manifest": "services/geometry/nuwax_geometry_runtime/Cargo.toml", "package": "nuwax-geometry-runtime", "features": []},
    "nuwax-nav-stack": {"module": "navigation", "manifest": WORKSPACE + "/Cargo.toml", "package": "nuwax-nav-stack", "features": ["simple-2d", "replanning-astar", "mls"]},
    "nuwax-person-follow": {"module": "follow", "manifest": WORKSPACE + "/Cargo.toml", "package": "nuwax-nav-visual-servoing", "features": ["link-liblcm"]},
    "nuwax-mapping-cli": {"module": "geometry", "manifest": WORKSPACE + "/Cargo.toml", "package": "nuwax-mapping-cli", "features": []},
}
ALL_PROGRAMS = {**LOCAL_PROGRAMS, **PROGRAMS}
PLATFORMS = {"focal-arm64", "noble-x86_64"}
_HEX = re.compile(r"[0-9a-f]{64}")
_REVISION = re.compile(r"[0-9a-f]{40}")
_MAX_ARCHIVE = 2 * 1024 ** 3
_MAX_EXPANDED = 8 * 1024 ** 3


def program_share(program: str) -> str:
    """程序专属说明材料只落在本程序的share命名空间。"""
    if program not in PROGRAMS:
        raise ValueError("未知可预编译程序的说明材料目录")
    return "share/neh-prebuilt/" + program


class PrebuiltPending(RuntimeError):
    """完整提交尚未由云端发布；管理程序可展示状态并延后重试。"""
    status = "prebuilt_pending"

    def __init__(self, revision: str):
        self.revision = revision
        self.message = "云端编译中，请稍后重试；也可选择立即本机编译。"
        super().__init__(self.message)

    def as_dict(self) -> dict:
        return {"status": self.status, "revision": self.revision, "message": self.message}


class PrebuiltUnavailable(RuntimeError):
    """网络或索引无效，允许既有真实本机构建继续。"""


def platform_key() -> Optional[str]:
    if platform.system() != "Linux":
        return None
    try:
        values = {}
        for line in Path("/etc/os-release").read_text(encoding="utf-8").splitlines():
            if "=" in line:
                key, value = line.split("=", 1)
                values[key] = value.strip('"')
    except OSError:
        return None
    machine = {"aarch64": "arm64", "amd64": "x86_64"}.get(platform.machine(), platform.machine())
    return {("ubuntu", "20.04", "arm64"): "focal-arm64", ("ubuntu", "24.04", "x86_64"): "noble-x86_64"}.get((values.get("ID"), values.get("VERSION_ID"), machine))


def program_names(modules: Iterable[str]) -> tuple:
    selected = set(modules)
    return tuple(name for name, recipe in ALL_PROGRAMS.items() if recipe["module"] in selected)


def eligible_program_names(modules: Iterable[str]) -> tuple:
    """仅返回允许云端分发的五程序；设备等待索引必须使用此入口。"""
    selected = set(modules)
    return tuple(name for name, recipe in PROGRAMS.items() if recipe["module"] in selected)


def _repository(value: str) -> str:
    if not isinstance(value, str):
        raise ValueError("预编译仓库名称无效")
    if value.startswith("https://github.com/"):
        value = value[len("https://github.com/"):]
        value = value.rstrip("/")
        if value.endswith(".git"):
            value = value[:-4]
    if not re.fullmatch(r"[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+", value):
        raise ValueError("预编译来源必须是公开GitHub仓库")
    return value


def release_url(repository: str, asset: str) -> str:
    if not isinstance(asset, str) or not re.fullmatch(r"[A-Za-z0-9_.-]+", asset):
        raise ValueError("预编译附件名称非法")
    return "https://github.com/" + _repository(repository) + "/releases/download/" + RELEASE + "/" + asset


def _urlopen(url: str, *, timeout: int):
    response = urllib.request.urlopen(urllib.request.Request(url, headers={"User-Agent": "NEH-prebuilt/1"}), timeout=timeout)
    parsed = urllib.parse.urlsplit(response.geturl())
    if parsed.scheme != "https" or parsed.hostname not in {"github.com", "release-assets.githubusercontent.com", "objects.githubusercontent.com"}:
        response.close()
        raise PrebuiltUnavailable("预编译下载跳转到未经允许的地址")
    return response


def validate_index(payload: Mapping, revision: str, programs: Iterable[str] = (), target_platform: Optional[str] = None) -> dict:
    """索引旧内容或选中项暂缺只等待；其它无效内容继续严格拒绝。"""
    if not isinstance(payload, dict) or payload.get("schema") != SCHEMA or not isinstance(payload.get("platforms"), dict):
        raise ValueError("预编译索引身份不符")
    if payload.get("revision") != revision:
        if isinstance(payload.get("revision"), str) and _REVISION.fullmatch(payload["revision"]):
            raise PrebuiltPending(revision)
        raise ValueError("预编译索引提交无效")
    for key, entry in payload["platforms"].items():
        if key not in PLATFORMS or not isinstance(entry, dict) or not isinstance(entry.get("programs"), dict):
            raise ValueError("预编译索引平台无效")
        if not set(entry["programs"]).issubset(PROGRAMS) or payload.get("source_archives"):
            raise ValueError("预编译索引包含禁止分发的LIO程序或源码包")
    if target_platform is not None:
        target = payload["platforms"].get(target_platform, {})
        if not set(programs).issubset(target.get("programs", {})):
            raise PrebuiltPending(revision)
    return payload


def fetch_index(repository: str, revision: str, *, timeout: int = 20) -> dict:
    """只用匿名Release直链；404或旧提交等待，其它无效内容单独拒绝。"""
    if not isinstance(revision, str) or not _REVISION.fullmatch(revision):
        raise ValueError("预编译索引需要完整提交")
    try:
        with _urlopen(release_url(repository, "index-" + revision + ".json"), timeout=timeout) as response:
            raw = response.read(4 * 1024 * 1024 + 1)
        if len(raw) > 4 * 1024 * 1024:
            raise ValueError("预编译索引过大")
        return validate_index(json.loads(raw), revision)
    except urllib.error.HTTPError as exc:
        if exc.code == 404:
            raise PrebuiltPending(revision) from exc
        raise PrebuiltUnavailable("预编译索引下载失败：HTTP " + str(exc.code)) from exc
    except (OSError, ValueError, KeyError, TypeError) as exc:
        raise PrebuiltUnavailable("预编译索引不可用：" + str(exc)) from exc


def _capture(argv: Sequence[str], *, cwd: Optional[Path] = None, env: Optional[Mapping] = None) -> str:
    return subprocess.check_output(list(argv), cwd=cwd, env={**os.environ, **(env or {})}, text=True).strip()


def sha256(path: Path) -> str:
    result = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def _relative(value: str) -> str:
    if not isinstance(value, str):
        raise ValueError("指纹路径不是字符串")
    path = PurePosixPath(value)
    if not path.parts or path.is_absolute() or ".." in path.parts or "\\" in value or path.as_posix() != value:
        raise ValueError("指纹或归档路径越界：" + value)
    return value


def discover_inputs(root: Path, program: str, cargo: Path, environment: Mapping, target: str) -> dict:
    """Cargo解析真实特性依赖闭包；CMake外部源码固定在仓库锁文件中。"""
    root = Path(root).resolve()
    recipe = PROGRAMS[program]
    paths = {"scripts/module_installer/builds.py", "scripts/neh_prebuilt.py", "scripts/ci/prebuilt.py", "LICENSE", "docs/公开文档/第三方许可证说明.md"}
    excluded = []
    manifest = recipe["manifest"]
    command = [str(cargo), "metadata", "--locked", "--format-version", "1", "--filter-platform", target, "--manifest-path", str(root / manifest)]
    if recipe["features"]:
        command.extend(("--features", ",".join(recipe["package"] + "/" + feature for feature in recipe["features"])))
    metadata = json.loads(_capture(command, cwd=root, env=environment))
    packages = {item["id"]: item for item in metadata["packages"]}
    nodes = {item["id"]: item for item in metadata["resolve"]["nodes"]}
    pending = [key for key, item in packages.items() if item["name"] == recipe["package"] and item.get("source") is None]
    if len(pending) != 1:
        raise RuntimeError("Cargo无法唯一确定程序包：" + program)
    pending = [(pending[0], "runtime")]
    seen, roles, cargo_packages = set(), {}, {}
    while pending:
        key, role = pending.pop()
        if (key, role) in seen:
            continue
        seen.add((key, role))
        roles.setdefault(key, set()).add(role)
        item = packages[key]
        cargo_packages[key] = {field: item.get(field) for field in ("id", "name", "version", "source", "license", "license_file", "manifest_path")}
        if item.get("source") is None:
            directory = Path(item["manifest_path"]).parent.relative_to(root).as_posix()
            paths.add(directory)
            external_inputs = {
                "nuwax-mapping": "contracts/pgo_options.json",
                "nuwax-lio-backends": "contracts/pointlio_options.json",
                "nuwax-nav-pgo-bridge": WORKSPACE + "/crates/nuwax-gtsam/native",
            }
            if item["name"] in external_inputs:
                paths.add(external_inputs[item["name"]])
            # Rust适配层不会编译两个独立LIO进程；依赖记录审计会拒绝新增的跨界读取。
            if directory == WORKSPACE + "/crates/nuwax-lio-backends":
                excluded.append(directory + "/native")
        for dependency in nodes[key].get("deps", []):
            destination = dependency["pkg"]
            proc_macro = any("proc-macro" in target["kind"] for target in packages[destination]["targets"])
            for kind in dependency["dep_kinds"]:
                if kind["kind"] != "dev":
                    pending.append((destination, "build" if role == "build" or kind["kind"] == "build" or proc_macro else "runtime"))
    for key, item in cargo_packages.items():
        item["dependency_roles"] = sorted(roles[key])
    paths.update((manifest, str(PurePosixPath(manifest).with_name("Cargo.lock")), WORKSPACE + "/Cargo.toml", WORKSPACE + "/build_support", WORKSPACE + "/lcmtypes", "services/navigation/proto", "services/geometry/nuwax_geometry_runtime/proto"))
    if program == "nuwax-slam-2d-worker":
        paths.update(("scripts/prepare_slam2d_dependencies.py", "drivers/sensors/slamtec_native"))
    # 编译器读到的Cargo配置也是构建输入；外部配置由CI隔离的CARGO_HOME控制。
    for config in (".cargo", "rust-toolchain", "rust-toolchain.toml"):
        if (root / config).exists():
            paths.add(config)
    return {"paths": sorted(paths), "exclude": sorted(set(excluded)), "cargo_packages": sorted(cargo_packages.values(), key=lambda item: item["id"])}


def fingerprint(root: Path, program: str, target_platform: str, input_spec: Mapping, build_identity: Mapping) -> str:
    """设备和云端唯一的指纹实现：选定Git树的blob/模式、锁文件、配方与环境。"""
    if program not in PROGRAMS or target_platform not in PLATFORMS:
        raise ValueError("未知预编译程序或平台")
    root = Path(root).resolve()
    paths = [_relative(item) for item in input_spec["paths"]]
    excluded = [_relative(item) for item in input_spec.get("exclude", [])]
    if not paths:
        raise ValueError("指纹缺少源码范围")
    # ls-tree直接读取Git对象；未提交文件不允许借用已有源码指纹。
    raw = subprocess.check_output(["git", "-C", str(root), "ls-tree", "-r", "-z", "HEAD", "--", *paths])
    entries = []
    for row in raw.split(b"\0"):
        if not row:
            continue
        metadata, name = row.split(b"\t", 1)
        relative = name.decode("utf-8")
        if any(relative == item or relative.startswith(item + "/") for item in excluded):
            continue
        entries.append([relative, metadata.decode("ascii")])
    if not entries:
        raise ValueError("指纹范围未包含任何受版本控制的源码")
    payload = {"schema": SCHEMA, "program": program, "platform": target_platform, "recipe": PROGRAMS[program], "input_spec": {"paths": sorted(paths), "exclude": sorted(set(excluded))}, "git_tree": sorted(entries), "build_identity": build_identity}
    return hashlib.sha256(json.dumps(payload, sort_keys=True, ensure_ascii=False, separators=(",", ":")).encode("utf-8")).hexdigest()


def covered(relative: str, input_spec: Mapping) -> bool:
    return (any(relative == item or relative.startswith(item + "/") for item in input_spec["paths"]) and not any(relative == item or relative.startswith(item + "/") for item in input_spec.get("exclude", [])))


def dependency_paths(record: Path) -> list:
    """解析rustc/C/C++的Make依赖记录以及CMake配置输入。"""
    content = record.read_text(encoding="utf-8", errors="strict")
    if record.name == "Makefile.cmake" or record.suffix == ".json":
        return re.findall(r'"(/[^"\n]+)"', content)
    result = []
    for line in content.replace("\\\n", " ").splitlines():
        if ": " in line:
            result.extend(shlex.split(line.split(": ", 1)[1], posix=True))
    return result


def audit_dependencies(root: Path, input_spec: Mapping, records: Iterable[Path], *, environment_roots: Iterable[Path] = (), record_cwds: Optional[Mapping] = None) -> dict:
    """不自动把漏算输入加入范围；失败必须由维护者修正声明并重新编译。"""
    root = Path(root).resolve()
    registered = [Path(item["path"] if isinstance(item, Mapping) else item).resolve() for item in environment_roots]
    seen, external, missing, record_count = set(), set(), set(), 0
    for record in records:
        record_count += 1
        for value in dependency_paths(record):
            path = Path(value)
            if not path.is_absolute():
                base = Path((record_cwds or {}).get(str(record.resolve()), root))
                path = base / path
            lexical = Path(os.path.abspath(path))
            if root == lexical or root in lexical.parents:
                relative = lexical.relative_to(root).as_posix()
                if covered(relative, input_spec):
                    seen.add(relative)
                else:
                    missing.add(relative)
            absolute = path.resolve()
            try:
                relative = absolute.relative_to(root).as_posix()
            except ValueError:
                external.add(str(absolute))
                if not any(absolute == allowed or allowed in absolute.parents for allowed in registered):
                    missing.add(str(absolute))
                continue
            if not covered(relative, input_spec):
                # 仓内路径始终属于程序指纹核查，登记外部根不能放宽仓内范围。
                missing.add(relative)
            else:
                seen.add(relative)
    if not record_count or not seen:
        raise RuntimeError("缺少编译器依赖记录，拒绝发布未审计程序")
    if missing:
        raise RuntimeError("程序指纹漏掉实际编译输入：\n" + "\n".join(sorted(missing)))
    return {"records": record_count, "repository_inputs": sorted(seen), "external_inputs": sorted(external), "coverage": "complete"}


def _write_json(path: Path, payload: Mapping) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent, prefix=".receipt-", delete=False) as handle:
        temporary = Path(handle.name)
        json.dump(payload, handle, ensure_ascii=False, sort_keys=True)
        handle.write("\n")
    try:
        temporary.chmod(0o600)
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def _ordinary_file(base: Path, relative: str) -> Path:
    relative = _relative(relative)
    current = base
    for part in PurePosixPath(relative).parts:
        current = current / part
        if current.is_symlink():
            raise ValueError("制品路径包含符号链接")
    if not current.is_file():
        raise ValueError("制品文件缺失")
    return current


def _receipt_files(prefix: Path, files: Iterable[str]) -> dict:
    return {name: {"sha256": sha256(_ordinary_file(prefix, name)), "mode": stat.S_IMODE((prefix / name).stat().st_mode)} for name in sorted(files)}


def reusable(prefix: Path, receipt: Path, fingerprint_value: str) -> bool:
    try:
        if receipt.is_symlink():
            return False
        payload = json.loads(receipt.read_text(encoding="utf-8"))
        files = payload["files"]
        if payload["fingerprint"] != fingerprint_value or not isinstance(files, dict) or not files:
            return False
        if payload.get("local_runtime") is True and set(files) != {"bin/" + payload["program"], *_local_runtime_files(prefix, payload["program"])}:
            return False
        return _receipt_files(prefix, files) == files
    except (OSError, ValueError, KeyError, TypeError, RuntimeError, subprocess.SubprocessError):
        return False


def _local_runtime_files(prefix: Path, program: str) -> set:
    """只核对实际动态闭包；显式dlopen按本程序固定归属加入。"""
    if program not in PROGRAMS:
        raise ValueError("本机回退回执只接受可预编译程序")
    prefix = prefix.resolve(strict=True)
    pending = [_ordinary_file(prefix, "bin/" + program)]
    if program == "nuwax-slam-2d-worker":
        pending.append(prefix / "lib/libnuwax_slamtec.so.1")
    files, seen = set(), set()
    while pending:
        binary = pending.pop().resolve(strict=True)
        if not binary.is_file():
            raise ValueError("实际动态依赖不是普通文件")
        if binary in seen:
            continue
        seen.add(binary)
        if binary != prefix / "bin" / program:
            files.add(binary.relative_to(prefix).as_posix())
        listing = subprocess.check_output(["ldd", str(binary)], text=True, env={**os.environ, "LC_ALL": "C"})
        if "not found" in listing:
            raise ValueError("实际动态依赖缺失：" + listing)
        for line in listing.splitlines():
            line = line.strip()
            if not line or line == "statically linked" or line.startswith("linux-vdso.so.1 "):
                continue
            match = re.fullmatch(r"(?:\S+\s+=>\s+)?(/.+?)\s+\(0x[0-9a-fA-F]+\)", line)
            if match is None:
                raise ValueError("无法解析实际动态依赖来源：" + line)
            selected = Path(match.group(1)).resolve(strict=True)
            if selected == prefix or prefix in selected.parents:
                pending.append(selected)
            elif not any(selected == base or base in selected.parents for base in map(Path, ("/usr", "/lib", "/lib64"))):
                raise ValueError("实际动态依赖未安装到受管前缀或系统目录：" + str(selected))
    return files


def record_local(prefix: Path, cache: Path, program: str, fingerprint_value: str) -> None:
    """本机回退成功后也记录文件摘要，使下一版本能按同一指纹复用。"""
    files = {"bin/" + program, *_local_runtime_files(prefix, program)}
    checked = _receipt_files(prefix, files)
    current = prefix
    if current.is_symlink():
        raise ValueError("本机回退前缀不能为符号链接")
    for part in PurePosixPath(program_share(program)).parts:
        current = current / part
        if current.is_symlink() or current.exists() and not current.is_dir():
            raise ValueError("本机回退材料目录包含符号链接或非目录")
    # 真实本机输出通过核对后，清除本程序旧云端证明，不能冒充新输出的来源。
    if current.exists():
        shutil.rmtree(current)
    _write_json(cache / "prebuilt/receipts" / (program + ".json"), {"schema": SCHEMA, "program": program, "fingerprint": fingerprint_value, "local_runtime": True, "files": checked})


def _decode_zstd_library(archive: Path, stream) -> None:
    """设备可直接调用基础系统的libzstd，不依赖新增命令或提权助手。"""
    try:
        library = ctypes.CDLL(ctypes.util.find_library("zstd") or "libzstd.so.1")
        class InputBuffer(ctypes.Structure):
            _fields_ = [("src", ctypes.c_void_p), ("size", ctypes.c_size_t), ("pos", ctypes.c_size_t)]
        class OutputBuffer(ctypes.Structure):
            _fields_ = [("dst", ctypes.c_void_p), ("size", ctypes.c_size_t), ("pos", ctypes.c_size_t)]
        library.ZSTD_createDStream.argtypes = []
        library.ZSTD_createDStream.restype = ctypes.c_void_p
        library.ZSTD_freeDStream.argtypes = [ctypes.c_void_p]
        library.ZSTD_freeDStream.restype = ctypes.c_size_t
        library.ZSTD_initDStream.argtypes = [ctypes.c_void_p]
        library.ZSTD_initDStream.restype = ctypes.c_size_t
        library.ZSTD_decompressStream.argtypes = [ctypes.c_void_p, ctypes.POINTER(OutputBuffer), ctypes.POINTER(InputBuffer)]
        library.ZSTD_decompressStream.restype = ctypes.c_size_t
        library.ZSTD_isError.argtypes = [ctypes.c_size_t]
        library.ZSTD_isError.restype = ctypes.c_uint
        library.ZSTD_getErrorName.argtypes = [ctypes.c_size_t]
        library.ZSTD_getErrorName.restype = ctypes.c_char_p
    except (OSError, AttributeError) as exc:
        raise PrebuiltUnavailable("基础系统缺少可用的libzstd，退回本机编译") from exc
    decoder = library.ZSTD_createDStream()
    if not decoder:
        raise PrebuiltUnavailable("libzstd无法创建解码器，退回本机编译")
    try:
        status = library.ZSTD_initDStream(decoder)
        if library.ZSTD_isError(status):
            raise PrebuiltUnavailable("libzstd解码器初始化失败")
        output_memory = ctypes.create_string_buffer(1024 * 1024)
        expanded, received = 0, False
        with archive.open("rb") as source:
            while True:
                block = source.read(1024 * 1024)
                # 文件结束后允许解码器排出尚未写出的输出；没有进度表示截断。
                if not block and received and status == 0:
                    break
                input_memory = ctypes.create_string_buffer(block)
                input_buffer = InputBuffer(ctypes.cast(input_memory, ctypes.c_void_p), len(block), 0)
                while True:
                    output_buffer = OutputBuffer(ctypes.cast(output_memory, ctypes.c_void_p), len(output_memory), 0)
                    previous = input_buffer.pos
                    status = library.ZSTD_decompressStream(decoder, ctypes.byref(output_buffer), ctypes.byref(input_buffer))
                    if library.ZSTD_isError(status):
                        message = library.ZSTD_getErrorName(status).decode("utf-8", errors="replace")
                        raise ValueError("预编译归档zstd解码失败：" + message)
                    expanded += output_buffer.pos
                    if expanded > _MAX_EXPANDED:
                        raise ValueError("预编译归档解码大小超限")
                    if output_buffer.pos:
                        stream.write(ctypes.string_at(output_memory, output_buffer.pos))
                    if not block:
                        if not received or (status != 0 and output_buffer.pos == 0):
                            raise ValueError("预编译归档zstd数据为空或被截断")
                        if status == 0:
                            return
                    if input_buffer.pos == input_buffer.size:
                        break
                    if input_buffer.pos == previous and not output_buffer.pos:
                        raise ValueError("预编译归档zstd解码没有进展")
                received = received or bool(block)
    finally:
        library.ZSTD_freeDStream(decoder)


def _decode_zstd(archive: Path, stream) -> None:
    program = shutil.which("zstd")
    if program is None:
        _decode_zstd_library(archive, stream)
    else:
        process = subprocess.Popen([program, "-q", "-d", "-c", str(archive)], stdout=subprocess.PIPE)
        expanded = 0
        try:
            while True:
                block = process.stdout.read(1024 * 1024)
                if not block:
                    break
                expanded += len(block)
                if expanded > _MAX_EXPANDED:
                    raise ValueError("预编译归档解码大小超限")
                stream.write(block)
            if process.wait() != 0:
                raise ValueError("预编译归档zstd解码失败")
        finally:
            process.stdout.close()
            if process.poll() is None:
                process.kill()
                process.wait()


def _extract(archive: Path, destination: Path, program: str, fingerprint_value: str) -> dict:
    """有界解码后tar只接受普通文件和目录，不接受链接或设备。"""
    with tempfile.TemporaryFile() as stream:
        _decode_zstd(archive, stream)
        stream.seek(0)
        total, names = 0, set()
        with tarfile.open(fileobj=stream, mode="r:") as bundle:
            for member in bundle:
                name = _relative(member.name.rstrip("/"))
                if name in names or not (member.isdir() or member.isfile()) or member.mode & 0o7000:
                    raise ValueError("预编译归档包含重复、链接或特殊文件")
                names.add(name)
                if member.size < 0 or member.size > _MAX_ARCHIVE:
                    raise ValueError("预编译文件大小无效")
                total += member.size
                if total > _MAX_EXPANDED:
                    raise ValueError("预编译归档解压大小超限")
                target = destination / name
                if member.isdir():
                    target.mkdir(parents=True, exist_ok=True)
                else:
                    target.parent.mkdir(parents=True, exist_ok=True)
                    with bundle.extractfile(member) as source, target.open("xb") as output:
                        shutil.copyfileobj(source, output)
                    target.chmod(member.mode & 0o777)
    manifest = json.loads((destination / "prebuilt-manifest.json").read_text(encoding="utf-8"))
    if manifest.get("schema") != SCHEMA or manifest.get("program") != program or manifest.get("fingerprint") != fingerprint_value:
        raise ValueError("预编译归档身份不符")
    files = manifest["files"]
    if not isinstance(files, dict) or "bin/" + program not in files or set(files) != {name for name in names if (destination / name).is_file()} - {"prebuilt-manifest.json"}:
        raise ValueError("预编译归档文件清单不完整")
    if _receipt_files(destination, files) != files:
        raise ValueError("预编译归档内部文件摘要不符")
    for relative in files:
        if not (relative == "bin/" + program or relative.startswith(("lib/", "lib64/", program_share(program) + "/"))):
            raise ValueError("预编译归档写入非程序目录")
    binary = destination / "bin" / program
    with binary.open("rb") as handle:
        if handle.read(4) != b"\x7fELF" or not os.access(binary, os.X_OK):
            raise ValueError("预编译程序不是可执行ELF文件")
    return manifest


def install_archive(root: Path, prefix: Path, cache: Path, program: str, entry: Mapping, repository: str) -> None:
    if program not in PROGRAMS:
        raise ValueError("本机源码构建专用程序禁止使用预编译附件")
    fingerprint_value = entry["fingerprint"]
    asset, expected = entry["asset"], entry["sha256"]
    if not _HEX.fullmatch(fingerprint_value) or not _HEX.fullmatch(expected) or asset != program + "-" + fingerprint_value + ".tar.zst":
        raise ValueError("预编译条目摘要或名称不符")
    size = entry["size"]
    if type(size) is not int or not 0 < size <= _MAX_ARCHIVE:
        raise ValueError("预编译条目大小无效")
    parent = cache / "prebuilt/archives"
    parent.mkdir(parents=True, exist_ok=True)
    archive = parent / asset
    if archive.is_symlink():
        raise ValueError("预编译下载缓存不接受符号链接")
    if not archive.is_file() or archive.stat().st_size != size or sha256(archive) != expected:
        archive.unlink(missing_ok=True)
        fd, raw = tempfile.mkstemp(prefix=".download-", dir=parent)
        temporary = Path(raw)
        try:
            with os.fdopen(fd, "wb") as output, _urlopen(release_url(repository, asset), timeout=60) as response:
                remaining = size + 1
                while remaining:
                    block = response.read(min(1024 * 1024, remaining))
                    if not block:
                        break
                    output.write(block)
                    remaining -= len(block)
            if temporary.stat().st_size != size or sha256(temporary) != expected:
                raise ValueError("预编译下载SHA-256（文件校验摘要）不符")
            temporary.replace(archive)
        finally:
            temporary.unlink(missing_ok=True)
    with tempfile.TemporaryDirectory(prefix=".extract-", dir=parent) as temporary:
        directory = Path(temporary)
        manifest = _extract(archive, directory, program, fingerprint_value)
        # 全部核对通过后才复制到候选构建前缀，受管运行版本由上层独立导出。
        for relative in manifest["files"]:
            source = directory / relative
            destination = prefix / relative
            current = prefix
            for part in PurePosixPath(relative).parts[:-1]:
                current = current / part
                if current.is_symlink():
                    raise ValueError("目标制品目录包含符号链接")
                current.mkdir(exist_ok=True)
            if destination.is_symlink():
                raise ValueError("目标制品文件是符号链接")
            shutil.copyfile(source, destination)
            destination.chmod(source.stat().st_mode & 0o777)
        _write_json(cache / "prebuilt/receipts" / (program + ".json"), manifest)


def prepare_prebuilt(root: Path, prefix: Path, cache: Path, programs: Iterable[str], config: Mapping) -> tuple:
    """返回已复用路径和指纹；索引暂缺/旧内容等待，附件故障逐程序回退。"""
    programs = tuple(name for name in programs if name in PROGRAMS)
    if not programs:
        return {}, {}
    key = platform_key()
    if key is None:
        return {}, {}
    revision, repository = config["revision"], config["repository"]
    # 未提交开发快照不能用Git对象指纹匹配云端产物。
    if _capture(["git", "-C", str(root), "status", "--porcelain", "--untracked-files=no"]):
        print("源码含未提交修改，使用本机增量编译。", flush=True)
        return {}, {}
    if _capture(["git", "-C", str(root), "rev-parse", "HEAD"]) != revision:
        raise ValueError("预编译请求与实际源码提交不符")
    try:
        # 管理进程已读到完整索引时交接同一快照，避免构建子进程再次命中旧404。
        index = config["index"] if "index" in config else fetch_index(repository, revision)
        validate_index(index, revision, programs, key)
    except PrebuiltUnavailable as exc:
        print(str(exc) + "；退回本机编译。", flush=True)
        return {}, {}
    target = index["platforms"].get(key)
    binaries, fingerprints = {}, {}
    for program in programs:
        entry = target["programs"].get(program)
        if not isinstance(entry, dict):
            continue
        try:
            value = fingerprint(root, program, key, entry["input_spec"], target["build_identity"])
            if value != entry["fingerprint"]:
                raise ValueError("源码输入指纹与云端不符")
            fingerprints[program] = value
            receipt = cache / "prebuilt/receipts" / (program + ".json")
            if reusable(prefix, receipt, value):
                print("复用同指纹程序：" + program, flush=True)
            else:
                print("下载预编译程序：" + program, flush=True)
                install_archive(root, prefix, cache, program, entry, repository)
            binaries[program] = str(prefix / "bin" / program)
        except (OSError, ValueError, KeyError, TypeError, RuntimeError, tarfile.TarError, subprocess.SubprocessError) as exc:
            print("预编译不可用，退回本机编译 " + program + "：" + str(exc), flush=True)
    return binaries, fingerprints
