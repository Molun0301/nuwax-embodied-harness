"""按所选大模块构建真实程序，所有可变产物归专属缓存和安装目录。"""

from __future__ import annotations

import ctypes
import errno
import hashlib
import json
import os
import platform
import re
import shutil
import stat
import subprocess
import tempfile
import time
import urllib.parse
import urllib.request
import uuid
import zipfile
from pathlib import Path, PurePosixPath
from typing import Callable, Iterable, Mapping, Optional, Sequence

from scripts.device_manager.common import atomic_write

RUST_VERSION = "1.88.0"
_NATIVE_ROOT = Path("services/nuwax-workspace/crates/nuwax-lio-backends/native")
_WORKSPACE = "services/nuwax-workspace/Cargo.toml"
_RUST_BUILDS = {
    "slam2d": (_WORKSPACE, "nuwax-slam-2d-runtime", "nuwax-slam-2d-worker", ("worker",)),
    "geometry": (
        "services/geometry/nuwax_geometry_runtime/Cargo.toml",
        "nuwax-geometry-runtime", "nuwax-geometry-runtime", (),
    ),
    "navigation": (
        _WORKSPACE, "nuwax-nav-stack", "nuwax-nav-stack",
        ("simple-2d", "replanning-astar", "mls"),
    ),
    "follow": (
        _WORKSPACE, "nuwax-nav-visual-servoing", "nuwax-person-follow", ("link-liblcm",),
    ),
}
_MAPPING_BUILD = (_WORKSPACE, "nuwax-mapping-cli", "nuwax-mapping-cli", ())
# 与 native/livox/cpp/flake.nix 的 v1.2.5 一致；固定其官方提交，避免标签漂移。
_LIVOX_SOURCE = (
    "https://github.com/Livox-SDK/Livox-SDK2.git",
    "6a940156dd7151c3ab6a52442d86bc83613bd11b",
)
_SOURCE_ROLES = frozenset((
    "fastlio2-algorithm", "pointlio-algorithm", "dimos-lcm", "livox-sdk2", "slam2d-onetbb", "slam2d-ceres",
))
_TARGET_ROLES = {_WORKSPACE: "nuwax-workspace", _RUST_BUILDS["geometry"][0]: "geometry-runtime"}
_LEGACY_SOURCE = re.compile(r"[0-9a-f]{12}-([0-9a-f]{40})")
_ASSIGNED_SOURCE = re.compile(r"(?:" + "|".join(sorted(_SOURCE_ROLES)) + r")-[0-9a-f]{32}")


def _cache_parent(cache: Path, relative: str) -> Path:
    # build_modules resolves the configured base once. Linked cache placement
    # and linked intermediate directories remain ordinary supported paths.
    parent = cache / relative
    parent.mkdir(parents=True, exist_ok=True)
    return parent


def _read_slots(path: Path, roles: Iterable[str], fields: tuple) -> dict:
    if path.is_symlink():
        raise RuntimeError(f"缓存路径记录不能是符号链接：{path}")
    if not path.exists():
        return {}
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
        slots = payload["slots"]
        if type(payload.get("version")) is not int or payload["version"] != 1 or not isinstance(slots, dict) or not set(slots).issubset(roles):
            raise ValueError("不支持的缓存路径记录")
        for entry in slots.values():
            if not isinstance(entry, dict) or set(entry) != set(fields):
                raise ValueError("缓存路径记录字段错误")
            if any(not isinstance(value, str) or not value or "\x00" in value for value in entry.values()):
                raise ValueError("缓存路径记录值错误")
            name = entry["path"]
            if not re.fullmatch(r"[A-Za-z0-9_-]+", name):
                raise ValueError("缓存路径必须是单一受管目录名")
        return slots
    except (OSError, ValueError, KeyError, TypeError) as exc:
        raise RuntimeError(f"缓存路径记录损坏，拒绝重新分配：{path}") from exc


def _save_slots(path: Path, slots: Mapping) -> None:
    # build_modules is serialized by manager.installation_lock. Persist the name
    # before invoking a builder so a failed build retries the same assigned path.
    atomic_write(path, (json.dumps({"version": 1, "slots": slots}, ensure_ascii=False, indent=2) + "\n").encode("utf-8"), mode=0o600)


def _cache_destination(parent: Path, name: str) -> Path:
    destination = parent / name
    if destination.is_symlink() or (destination.exists() and not destination.is_dir()):
        raise RuntimeError(f"缓存目标不是受管普通目录：{destination}")
    return destination


def _publish_source_directory(temporary: Path, destination: Path) -> None:
    # Same Linux no-clobber publication used by the release/asset publishers.
    # A destination arriving during a clone must remain untouched, even empty.
    libc = ctypes.CDLL(None, use_errno=True)
    renameat2 = getattr(libc, "renameat2", None)
    if renameat2 is None:
        raise OSError(errno.ENOTSUP, "源码缓存原子发布需要 Linux renameat2")
    renameat2.argtypes = (ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint)
    renameat2.restype = ctypes.c_int
    if renameat2(-100, os.fsencode(temporary), -100, os.fsencode(destination), 1) != 0:
        number = ctypes.get_errno()
        raise OSError(number, os.strerror(number), str(destination))


def _clean_source(destination: Path, revision: str) -> bool:
    actual = _capture(("git", "-C", str(destination), "rev-parse", "HEAD"))
    if actual != revision:
        return False
    dirty = _capture(("git", "-C", str(destination), "status", "--porcelain", "--untracked-files=all"))
    return not dirty


def _legacy_source(parent: Path, revision: str) -> Optional[Path]:
    selected = None
    # This is source admission, only on a missing/changed recipe binding. Keep
    # memory bounded and never interpret an old URL digest as an owner proof.
    with os.scandir(parent) as entries:
        for entry in entries:
            old = _LEGACY_SOURCE.fullmatch(entry.name)
            assigned = _ASSIGNED_SOURCE.fullmatch(entry.name)
            if not ((old and old.group(1) == revision) or assigned):
                continue
            candidate = parent / entry.name
            if entry.is_symlink() or not entry.is_dir(follow_symlinks=False):
                continue
            try:
                actual = _capture(("git", "-C", str(candidate), "rev-parse", "HEAD"))
                if actual != revision:
                    continue
                dirty = _capture(("git", "-C", str(candidate), "status", "--porcelain", "--untracked-files=all"))
            except (OSError, subprocess.SubprocessError):
                continue
            if dirty:
                continue
            if selected is None or candidate.name < selected.name:
                selected = candidate
    return selected


def _source_destination(cache: Path, role: str, source: tuple) -> tuple:
    if role not in _SOURCE_ROLES:
        raise ValueError("未知源码缓存配方")
    url, revision = source
    if not isinstance(url, str) or not url or not re.fullmatch(r"[0-9a-f]{40}", revision):
        raise ValueError("源码缓存需要明确地址和完整固定版本")
    parent = _cache_parent(cache, "sources")
    record = parent / "source-paths-v1.json"
    slots = _read_slots(record, _SOURCE_ROLES, ("url", "revision", "path"))
    saved = slots.get(role)
    if saved and (saved["url"], saved["revision"]) == source:
        destination = _cache_destination(parent, saved["path"])
        if destination.exists() and not _clean_source(destination, revision):
            raise RuntimeError(f"源码缓存被修改，请检查该专属缓存：{destination}")
        return destination, destination.exists()
    destination = _legacy_source(parent, revision)
    admitted = destination is not None
    if destination is None:
        destination = parent / (role + "-" + uuid.uuid4().hex)
        if destination.exists() or destination.is_symlink():
            raise RuntimeError(f"新分配的源码缓存名称已存在：{destination}")
    # url is requested provenance, never a claim about a legacy fetch origin.
    slots[role] = {"url": url, "revision": revision, "path": destination.name}
    _save_slots(record, slots)
    return destination, admitted


def _rust_target(cache: Path, root: Path, manifest: str) -> Path:
    role = _TARGET_ROLES[manifest]
    parent = _cache_parent(cache, "build/rust")
    record = parent / "target-paths-v1.json"
    slots = _read_slots(record, _TARGET_ROLES.values(), ("manifest", "path"))
    identity = str((root / manifest).resolve())
    expected = {"version": 1, "manifest": identity}
    saved = slots.get(role)
    if saved and saved["manifest"] == identity:
        target = _cache_destination(parent, saved["path"])
        if target.exists():
            owner = target / "neh-target-owner.json"
            try:
                if owner.is_symlink() or json.loads(owner.read_text(encoding="utf-8")) != expected:
                    raise ValueError("构建缓存所有者不符")
            except (OSError, ValueError, TypeError) as exc:
                raise RuntimeError(f"构建缓存缺少匹配的所有者记录：{target}") from exc
            return target
    # The slot is the visibility boundary. Prepare the exclusively created
    # directory and owner first; only then expose it to a builder. A crash
    # before slot publication leaves an unbound directory, never a broken slot.
    target = parent / (role + "-" + uuid.uuid4().hex)
    target.mkdir()
    atomic_write(target / "neh-target-owner.json", (json.dumps(expected, ensure_ascii=False) + "\n").encode("utf-8"), mode=0o600)
    slots[role] = {"manifest": identity, "path": target.name}
    _save_slots(record, slots)
    return target


def describe_builds(modules: Iterable[str]) -> tuple[str, ...]:
    selected = set(modules)
    descriptions = []
    for name in ("fastlio2", "pointlio"):
        if name in selected:
            descriptions.append(f"构建锁定源码的 {name} 原生程序及专属雷达开发库")
    for name, (_, _, binary, _) in _RUST_BUILDS.items():
        if name in selected:
            descriptions.append(f"使用 Rust（系统编程语言）{RUST_VERSION} 构建 {binary}")
    if "geometry" in selected and "slam2d" not in selected:
        descriptions.append("构建地图导出与工作区操作所需的 nuwax-mapping-cli")
    return tuple(descriptions)


def _capture(argv: Sequence[str], *, env: Optional[Mapping[str, str]] = None) -> str:
    completed = subprocess.run(
        list(argv), check=True, capture_output=True, text=True, timeout=30, env={**os.environ, **(env or {})},
    )
    return completed.stdout.strip()


def _locked_source(path: Path, name: str) -> tuple[str, str]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    nodes = payload["nodes"]
    node = nodes[payload["root"]]["inputs"][name]
    locked = nodes[node]["locked"]
    owner, repository, revision = locked["owner"], locked["repo"], locked["rev"]
    if (
        locked.get("type") != "github"
        or not re.fullmatch(r"[A-Za-z0-9_.-]+", owner)
        or not re.fullmatch(r"[A-Za-z0-9_.-]+", repository)
        or not re.fullmatch(r"[0-9a-f]{40}", revision)
    ):
        raise RuntimeError(f"原生依赖锁文件不包含有效的固定提交：{path}/{name}")
    return f"https://github.com/{owner}/{repository}.git", revision


def _checkout(cache: Path, source: tuple[str, str], run: Callable, *, role: str) -> Path:
    url, revision = source
    destination, admitted = _source_destination(cache, role, source)
    parent = destination.parent
    if admitted:
        return destination
    # 只有提交完整检出并核对后才发布缓存；下载中断不会留下可复用的半成品。
    temporary = Path(tempfile.mkdtemp(prefix=".download-", dir=str(parent)))
    try:
        run(("git", "init", str(temporary)))
        for attempt in range(3):
            try:
                run((
                    "git", "-C", str(temporary), "-c", "core.hooksPath=/dev/null",
                    "fetch", "--depth", "1", "--no-recurse-submodules", url, revision,
                ), env={"GIT_TERMINAL_PROMPT": "0"})
                break
            except Exception:
                if attempt == 2:
                    raise
                time.sleep(attempt + 1)
        run((
            "git", "-C", str(temporary), "-c", "core.hooksPath=/dev/null",
            "checkout", "--detach", revision,
        ))
        if _capture(("git", "-C", str(temporary), "rev-parse", "HEAD")) != revision:
            raise RuntimeError(f"原生依赖检出提交与锁文件不符：{url}")
        _publish_source_directory(temporary, destination)
    finally:
        if temporary.exists():
            shutil.rmtree(temporary)
    return destination


def _download(url: str, destination: Path, *, trusted_hosts: Sequence[str] = ()) -> None:
    def trusted(address: str) -> bool:
        parsed = urllib.parse.urlsplit(address)
        return (
            parsed.scheme == "https" and parsed.hostname in trusted_hosts and parsed.port in (None, 443)
            and not parsed.username and not parsed.password
        )

    if trusted_hosts and not trusted(url):
        raise RuntimeError("工具链下载地址不是受信任的官方地址")
    destination.parent.mkdir(parents=True, exist_ok=True)
    for attempt in range(3):
        fd, raw_path = tempfile.mkstemp(prefix=".download-", dir=str(destination.parent))
        temporary = Path(raw_path)
        try:
            with os.fdopen(fd, "wb") as output, urllib.request.urlopen(url, timeout=60) as response:
                if not response.geturl().startswith("https://"):
                    raise RuntimeError("工具链下载不能降级到未加密地址")
                if trusted_hosts and not trusted(response.geturl()):
                    raise RuntimeError("工具链下载跳转不是受信任的官方地址")
                shutil.copyfileobj(response, output)
            temporary.replace(destination)
            return
        except Exception:
            if attempt == 2:
                raise
            time.sleep(attempt + 1)
        finally:
            temporary.unlink(missing_ok=True)


def _protocol_compiler(cache: Path) -> dict[str, str]:
    """复用支持optional的协议编译器；旧系统直接下载官方静态程序并缓存。"""
    configured = os.environ.get("PROTOC", "").strip()
    program = configured or shutil.which("protoc")
    if program:
        try:
            version = subprocess.check_output([program, "--version"], text=True, timeout=30).strip()
            match = re.fullmatch(r"libprotoc (\d+)\.(\d+)(?:\.\d+)?", version)
            if match and tuple(map(int, match.groups())) >= (3, 15):
                return {"PROTOC": str(Path(shutil.which(program) or program).resolve())}
        except (OSError, subprocess.SubprocessError):
            pass
        if configured:
            raise RuntimeError("指定的PROTOC不可用或不支持当前协议的optional字段")
    architecture = {"arm64": "aarch64", "amd64": "x86_64"}.get(platform.machine().lower(), platform.machine().lower())
    archives = {"x86_64": "x86_64", "aarch64": "aarch_64"}
    if platform.system() != "Linux" or architecture not in archives:
        raise RuntimeError("当前架构没有已验证的协议编译器")
    archive = cache / "downloads" / ("protoc-25.3-linux-" + archives[architecture] + ".zip")
    url = "https://github.com/protocolbuffers/protobuf/releases/download/v25.3/" + archive.name
    parent = _cache_parent(cache, "tools")
    destination = _cache_destination(parent, "protoc-25.3-" + architecture)
    marker_name = "neh-protoc-install.json"

    def member_path(name: str) -> PurePosixPath:
        path = PurePosixPath(name)
        if not path.parts or path.is_absolute() or ".." in path.parts or "\\" in name or path.as_posix() != name.rstrip("/"):
            raise RuntimeError("官方协议编译器包含越界路径")
        return path

    def file_state(path: Path, base: Path) -> list[int]:
        # 用轻量文件元数据确认本机已完成的工作；缓存复用不读取内容或计算摘要。
        relative = path.relative_to(base)
        current = base
        for part in relative.parts:
            current = current / part
            if current.is_symlink():
                raise RuntimeError("协议编译器缓存不接受链接文件或目录")
        info = path.stat()
        if not stat.S_ISREG(info.st_mode):
            raise RuntimeError("协议编译器缓存需要普通文件")
        return [info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns, info.st_ctime_ns, info.st_mode]

    def read_record(path: Path) -> dict:
        if path.is_symlink():
            raise RuntimeError("协议编译器缓存记录不接受链接文件")
        try:
            value = json.loads(path.read_text(encoding="utf-8"))
            return value if isinstance(value, dict) else {}
        except (OSError, ValueError):
            return {}

    def result(directory: Path) -> dict[str, str]:
        program = directory / "bin/protoc"
        _executable(program)
        if subprocess.check_output([str(program), "--version"], text=True, timeout=30).strip() != "libprotoc 25.3":
            raise RuntimeError("官方协议编译器版本不符")
        return {"PROTOC": str(program), "PROTOC_INCLUDE": str(directory / "include")}

    receipt = read_record(destination / marker_name)
    files = receipt.get("files")
    required = {"bin/protoc", "include/google/protobuf/descriptor.proto"}
    if (
        receipt.get("source") == url and type(receipt.get("recipe")) is int and receipt["recipe"] == 1
        and isinstance(files, dict) and required.issubset(files)
    ):
        complete = True
        for name, expected in files.items():
            path = member_path(name)
            try:
                if file_state(destination.joinpath(*path.parts), destination) != expected:
                    complete = False
            except OSError:
                complete = False
        if complete:
            return result(destination)

    # 仅接受固定官方版本的 HTTPS 下载；记录保留已下载文件，供解压中断后离线重试。
    # 未知或已改变的 ZIP 不能仅凭文件名相符就获得执行许可。
    download_record = archive.with_suffix(".download.json")
    saved = read_record(download_record)
    try:
        admitted = saved == {"source": url, "file": file_state(archive, archive.parent)}
    except OSError:
        admitted = False
    if not admitted:
        _download(url, archive, trusted_hosts=("github.com", "release-assets.githubusercontent.com", "objects.githubusercontent.com"))
        atomic_write(download_record, (json.dumps({"source": url, "file": file_state(archive, archive.parent)}) + "\n").encode("utf-8"), mode=0o600)

    temporary = Path(tempfile.mkdtemp(prefix=".protoc-", dir=str(parent)))
    previous = None
    try:
        files = {}
        with zipfile.ZipFile(archive) as source:
            for item in source.infolist():
                path = member_path(item.filename)
                mode = stat.S_IFMT(item.external_attr >> 16)
                if mode not in (0, stat.S_IFREG, stat.S_IFDIR):
                    raise RuntimeError("官方协议编译器包含链接或特殊文件")
                if item.is_dir() or not (item.filename == "bin/protoc" or item.filename.startswith("include/")):
                    continue
                if item.filename in files:
                    raise RuntimeError("官方协议编译器包含重复文件")
                target = temporary.joinpath(*path.parts)
                target.parent.mkdir(parents=True, exist_ok=True)
                with source.open(item) as content, target.open("xb") as output:
                    shutil.copyfileobj(content, output)
                target.chmod(0o755 if item.filename == "bin/protoc" else 0o644)
                files[item.filename] = file_state(target, temporary)
        if not required.issubset(files):
            raise RuntimeError("官方协议编译器缺少程序或协议头文件")
        result(temporary)
        atomic_write(temporary / marker_name, (json.dumps({"source": url, "recipe": 1, "files": files}) + "\n").encode("utf-8"), mode=0o600)
        # 完整解压并核对版本后才发布；替换准备完成之前保留已有安装。
        _cache_destination(parent, destination.name)
        if destination.exists():
            previous = parent / (".protoc-previous-" + uuid.uuid4().hex)
            _publish_source_directory(destination, previous)
        try:
            _publish_source_directory(temporary, destination)
        except BaseException:
            if previous is not None and not destination.exists() and not destination.is_symlink():
                _publish_source_directory(previous, destination)
            raise
        if previous is not None:
            shutil.rmtree(previous)
        return {"PROTOC": str(destination / "bin/protoc"), "PROTOC_INCLUDE": str(destination / "include")}
    except (zipfile.BadZipFile, RuntimeError):
        download_record.unlink(missing_ok=True)
        raise
    finally:
        if temporary.exists():
            shutil.rmtree(temporary)


def _rust_toolchain(cache: Path, run: Callable) -> tuple[Path, dict[str, str]]:
    if os.geteuid() == 0:
        raise RuntimeError("请用普通用户运行安装器；构建工具链不应以管理员身份安装。")
    cargo_home = cache / "cargo"
    rustup_home = cache / "rustup"
    own_env = {"CARGO_HOME": str(cargo_home), "RUSTUP_HOME": str(rustup_home)}
    candidates = [
        (cargo_home / "bin" / "rustup", own_env),
        (Path(shutil.which("rustup") or "/nonexistent-rustup"), {}),
        (Path.home() / ".cargo" / "bin" / "rustup", {}),
    ]
    cargo = None
    selected_env = own_env
    for rustup, probe_env in candidates:
        if not rustup.is_file():
            continue
        try:
            # rustup which 只查询已安装版本，不会为了检查触发下载。
            found = Path(_capture((str(rustup), "which", "--toolchain", RUST_VERSION, "cargo"), env=probe_env))
            rustc = found.with_name("rustc")
            if found.is_file() and _capture((str(rustc), "--version")).startswith("rustc " + RUST_VERSION + " "):
                cargo = found
                selected_env = {
                    "CARGO_HOME": str(cargo_home),
                    "RUSTUP_HOME": str(found.parents[3]),
                }
                break
        except (OSError, subprocess.SubprocessError, IndexError):
            continue
    if cargo is None:
        architecture = platform.machine().lower()
        architecture = {"amd64": "x86_64", "arm64": "aarch64"}.get(architecture, architecture)
        if platform.system() != "Linux" or architecture not in {"x86_64", "aarch64"}:
            raise RuntimeError("自动构建工具链暂支持 Linux（操作系统）的 x86_64/aarch64 架构。")
        url = f"https://static.rust-lang.org/rustup/dist/{architecture}-unknown-linux-gnu/rustup-init"
        installer = cache / "downloads" / "rustup-init"
        checksum = installer.with_suffix(".sha256")
        _download(url + ".sha256", checksum)
        _download(url, installer)
        expected = checksum.read_text(encoding="utf-8").split()[0]
        actual = hashlib.sha256(installer.read_bytes()).hexdigest()
        if not re.fullmatch(r"[0-9a-f]{64}", expected) or actual != expected:
            installer.unlink(missing_ok=True)
            raise RuntimeError("官方工具链安装程序的 SHA-256（文件校验摘要）校验失败。")
        installer.chmod(0o755)
        run((
            str(installer), "-y", "--no-modify-path", "--profile", "minimal",
            "--default-toolchain", RUST_VERSION,
        ), env=own_env)
        cargo = Path(_capture((
            str(cargo_home / "bin" / "rustup"), "which", "--toolchain", RUST_VERSION, "cargo",
        ), env=own_env))
    rustc = cargo.with_name("rustc")
    if not cargo.is_file() or not _capture((str(rustc), "--version")).startswith("rustc " + RUST_VERSION + " "):
        raise RuntimeError(f"构建工具链未提供所需的 Rust {RUST_VERSION}。")
    return cargo, {
        **selected_env,
        "RUSTC": str(rustc), "RUSTDOC": str(cargo.with_name("rustdoc")),
        "PATH": str(cargo.parent) + os.pathsep + os.environ.get("PATH", os.defpath),
        "RUSTC_WRAPPER": "", "RUSTC_WORKSPACE_WRAPPER": "",
        "CARGO_INCREMENTAL": "0",
    }


def _cmake(source: Path, build: Path, prefix: Path, jobs: int, run: Callable,
           options: Sequence[str] = ()) -> None:
    build.mkdir(parents=True, exist_ok=True)
    run((
        "cmake", "-S", str(source), "-B", str(build),
        "-DCMAKE_BUILD_TYPE=Release", "-DCMAKE_INSTALL_PREFIX=" + str(prefix),
        "-DCMAKE_POLICY_VERSION_MINIMUM=3.5", "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON", *options,
    ))
    run(("cmake", "--build", str(build), "--parallel", str(jobs)))
    run(("cmake", "--install", str(build)))


def _prepare_livox(source: Path, destination: Path) -> None:
    # 与仓内 Nix（可复现构建工具）配方保持一致，仅修改专属构建副本。
    # Bump recipe when the patch operations below change. The checkout already
    # admits the pinned source; a completed preparation needs no second scan.
    prepared = {"source": str(source.resolve()), "revision": _LIVOX_SOURCE[1], "recipe": 1}
    marker = destination / ".neh-livox-prepared.json"
    if not destination.is_symlink() and not marker.is_symlink():
        try:
            if json.loads(marker.read_text(encoding="utf-8")) == prepared:
                return
        except (OSError, ValueError):
            pass
    if destination.exists():
        shutil.rmtree(destination)
    shutil.copytree(source, destination, ignore=shutil.ignore_patterns(".git", marker.name))
    cmake = destination / "CMakeLists.txt"
    content = cmake.read_text(encoding="utf-8")
    if "add_subdirectory(samples)" not in content:
        raise RuntimeError("锁定的 Livox 开发库源码布局发生变化。")
    cmake.write_text(content.replace("add_subdirectory(samples)", ""), encoding="utf-8")
    for relative in ("sdk_core/comm/define.h", "sdk_core/logger_handler/file_manager.h"):
        header = destination / relative
        header.write_text("#include <cstdint>\n" + header.read_text(encoding="utf-8"), encoding="utf-8")
    for cmake in destination.rglob("CMakeLists.txt"):
        cmake.write_text(cmake.read_text(encoding="utf-8").replace("-Werror", ""), encoding="utf-8")
    atomic_write(marker, (json.dumps(prepared) + "\n").encode("utf-8"), mode=0o600)


def _build_native(root: Path, prefix: Path, cache: Path, name: str, jobs: int,
                  run: Callable) -> Path:
    source = root / _NATIVE_ROOT / name / "cpp"
    lock = source / "flake.lock"
    livox_recipe = (root / _NATIVE_ROOT / "livox/cpp/flake.nix").read_text(encoding="utf-8")
    if not re.search(r'version\s*=\s*"1\.2\.5"\s*;', livox_recipe):
        raise RuntimeError("Livox 开发库版本已变化，须同步固定提交的构建配方。")
    algorithm = _checkout(cache, _locked_source(lock, "fast-lio"), run, role=name + "-algorithm")
    messages = _checkout(cache, _locked_source(lock, "dimos-lcm"), run, role="dimos-lcm")
    livox = _checkout(cache, _LIVOX_SOURCE, run, role="livox-sdk2")
    native_cache = cache / "build" / "native"
    livox_copy = native_cache / "livox-source"
    _prepare_livox(livox, livox_copy)
    # Keep consumer include/link paths stable across installation generations.
    # This small SDK install shares the existing native cache lifetime.
    livox_prefix = native_cache / "livox-install"
    _cmake(livox_copy, native_cache / "livox", livox_prefix, jobs, run, (
        "-DBUILD_SHARED_LIBS=ON", "-DCMAKE_INSTALL_RPATH=$ORIGIN",
    ))
    run(("cmake", "--install", str(native_cache / "livox"), "--prefix", str(prefix)))
    library = prefix / "lib" / "liblivox_lidar_sdk_shared.so"
    if not library.is_file():
        raise RuntimeError(f"雷达开发库构建未产生预期共享库：{library}")
    _cmake(source, native_cache / name, prefix, jobs, run, (
        "-DFASTLIO_DIR=" + str(algorithm),
        "-DFETCHCONTENT_SOURCE_DIR_DIMOS_LCM=" + str(messages),
        "-DFETCHCONTENT_FULLY_DISCONNECTED=ON",
        "-DLIVOX_COMMON_DIR=" + str(root / _NATIVE_ROOT / "common"),
        "-DLIVOX_SDK=" + str(livox_prefix / "lib/liblivox_lidar_sdk_shared.so"),
        "-DCMAKE_PREFIX_PATH=" + str(prefix),
        "-DCMAKE_INSTALL_RPATH=$ORIGIN/../lib",
    ))
    return prefix / "bin" / (name + "_native")


def _executable(path: Path) -> None:
    if path.is_symlink() or not path.is_file() or not os.access(str(path), os.X_OK):
        raise RuntimeError(f"构建缺少可执行程序：{path}")
    with path.open("rb") as handle:
        if handle.read(4) != b"\x7fELF":
            raise RuntimeError(f"构建输出不是 Linux 本机二进制程序：{path}")


def build_modules(root: Path, prefix: Path, cache: Path, modules: Iterable[str],
                  jobs: int, run: Callable, *, prebuilt: Optional[Mapping] = None,
                  local_build: bool = False, programs: Optional[Iterable[str]] = None) -> dict[str, str]:
    """构建所选模块；run 接收参数数组及 cwd/env，命令失败必须抛出异常。"""
    root, prefix, cache = Path(root).resolve(), Path(prefix).resolve(), Path(cache).resolve()
    selected = set(modules)
    from scripts.neh_prebuilt import PROGRAMS, prepare_prebuilt, program_names, record_local
    requested = set(program_names(selected))
    if programs is not None:
        narrowed = set(programs)
        if not narrowed.issubset(requested) or not narrowed.issubset(PROGRAMS):
            raise ValueError("逐程序构建范围不属于所选模块。")
        requested = narrowed
    if not isinstance(jobs, int) or isinstance(jobs, bool) or jobs < 1:
        raise ValueError("编译并行数必须是正整数。")
    print(f"构建并行预算：{jobs}；独立 CMake/Boost 使用此配额，Cargo 原生构建共享任务池。", flush=True)
    binaries = {}  # type: Dict[str, str]
    if not selected.intersection(set(_RUST_BUILDS) | {"fastlio2", "pointlio"}):
        return binaries
    if os.geteuid() == 0:
        raise RuntimeError("请用普通用户运行安装器；系统依赖由安装器单独提权处理。")
    (prefix / "bin").mkdir(parents=True, exist_ok=True)
    cache.mkdir(parents=True, exist_ok=True)
    fingerprints = {}
    if prebuilt and not local_build and requested:
        binaries, fingerprints = prepare_prebuilt(root, prefix, cache, requested, prebuilt)
    remaining = requested - set(binaries)
    if not remaining:
        return binaries
    for name in ("fastlio2", "pointlio"):
        if name + "_native" in remaining:
            binary = _build_native(root, prefix, cache, name, jobs, run)
            _executable(binary)
            binaries[binary.name] = str(binary)
    rust_modules = [name for name in _RUST_BUILDS if _RUST_BUILDS[name][2] in remaining]
    if rust_modules or _MAPPING_BUILD[2] in remaining:
        cargo, environment = _rust_toolchain(cache, run)
        environment = {**environment, **_protocol_compiler(cache), "CARGO_BUILD_JOBS": str(jobs)}
        if "nuwax-slam-2d-worker" in remaining:
            from scripts.prepare_slam2d_dependencies import build_dependencies
            build_dependencies(prefix, cache / "slam2d-dependencies", jobs=jobs, run=run)
            _cmake(root / "drivers/sensors/slamtec_native", cache / "build/slamtec", prefix, jobs, run)
        recipes = [_RUST_BUILDS[name] for name in rust_modules]
        if _MAPPING_BUILD[2] in remaining:
            recipes.append(_MAPPING_BUILD)
        for manifest, package, binary_name, features in recipes:
            if not (root / manifest).with_name("Cargo.lock").is_file():
                raise RuntimeError(f"构建清单缺少依赖锁文件：{manifest}")
            target = _rust_target(cache, root, manifest)
            command = [
                str(cargo), "build", "--locked", "--release", "--jobs", str(jobs),
                "--manifest-path", str(root / manifest), "--target-dir", str(target),
                "--package", package, "--bin", binary_name,
            ]
            if features:
                command.extend(("--features", ",".join(features)))
            build_environment = environment
            if binary_name == "nuwax-slam-2d-worker":
                build_environment = {**environment, "NUWAX_SLAM2D_DEPENDENCY_PREFIX": str(prefix),
                                     "CMAKE_PREFIX_PATH": str(prefix)}
            run(tuple(command), cwd=root, env=build_environment)
            built = target / "release" / binary_name
            _executable(built)
            installed = prefix / "bin" / binary_name
            shutil.copyfile(built, installed)
            installed.chmod(0o755)
            binaries[binary_name] = str(installed)
            if binary_name == "nuwax-slam-2d-worker":
                from scripts.package_slam2d import elf_info
                _machine, _needed, search = elf_info(built)
                native = next((p for p in search if (p / "libnuwax_slam_toolbox.so.1").is_file()), None)
                if native is None:
                    raise RuntimeError("二维工作程序未绑定本次原生核心")
                run(("cmake", "--install", str(native), "--prefix", str(prefix)))
                sources = prefix / "share/nuwax-slam2d/native"
                if sources.exists():
                    shutil.rmtree(sources)
                shutil.copytree(root / "services/nuwax-workspace/crates/nuwax-slam-toolbox-sys/native", sources)
                driver_sources = prefix / "share/nuwax-slam2d/slamtec"
                if driver_sources.exists():
                    shutil.rmtree(driver_sources)
                shutil.copytree(root / "drivers/sensors/slamtec_native", driver_sources)
                # 运行版本不能回到可变编译缓存寻找二维动态库。
                run(("patchelf", "--set-rpath", "$ORIGIN/../lib:$ORIGIN/../lib64", str(installed)))
    for program in remaining:
        if program in fingerprints and program in binaries:
            record_local(prefix, cache, program, fingerprints[program])
    return binaries


def prune_build_cache(cache: Path, removed: Iterable[str], selected: Iterable[str]) -> None:
    """删除已退出模块的专属编译缓存；保留工具链及仍使用的公共缓存。"""
    cache = Path(cache).absolute()
    if cache.is_symlink():
        raise RuntimeError(f"构建缓存根目录不能是符号链接：{cache}")
    removed, selected = set(removed), set(selected)
    targets = [cache / "build/native" / name for name in ("fastlio2", "pointlio")
               if name in removed and name not in selected]
    if "slam2d" in removed and "slam2d" not in selected:
        targets.extend((cache / "slam2d-dependencies", cache / "build/slamtec"))
    if not selected.intersection({"fastlio2", "pointlio", "slam2d"}):
        targets.extend((cache / "build/native", cache / "sources"))
    if not selected.intersection(_RUST_BUILDS):
        targets.append(cache / "build/rust")
    for path in targets:
        # 不跟随受管路径内部的链接，避免缓存清理触及其它目录。
        for ancestor in path.parents:
            if ancestor == cache:
                break
            if ancestor.is_symlink():
                raise RuntimeError(f"构建缓存路径包含符号链接：{ancestor}")
        if path.is_symlink():
            path.unlink()
        elif path.is_dir():
            shutil.rmtree(path)
