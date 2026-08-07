#!/usr/bin/env python3
"""构建、暂存并原子发布 Geometry 完整 Release generation。"""
from __future__ import annotations

import argparse
import ctypes
import errno
import fcntl
import hashlib
import importlib.util
import json
import os
import re
import secrets
import shutil
import stat
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Set, Tuple

REPO_ROOT = Path(__file__).resolve().parents[1]


def _load_release_manifest_module():
    # 生产构建器只允许从仓库内逐级打开并执行固定的发布清单文件。
    manifest_source = REPO_ROOT / "services" / "geometry" / "release_manifest.py"
    directory_flags = (
        os.O_RDONLY
        | getattr(os, "O_DIRECTORY", 0)
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    file_flags = (
        os.O_RDONLY
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    opened_fds = []
    try:
        repo_fd = os.open(str(REPO_ROOT), directory_flags)
        opened_fds.append(repo_fd)
        services_fd = os.open("services", directory_flags, dir_fd=repo_fd)
        opened_fds.append(services_fd)
        geometry_fd = os.open("geometry", directory_flags, dir_fd=services_fd)
        opened_fds.append(geometry_fd)
        manifest_fd = os.open(
            "release_manifest.py", file_flags, dir_fd=geometry_fd
        )
        opened_fds.append(manifest_fd)

        initial_stat = os.fstat(manifest_fd)
        if not stat.S_ISREG(initial_stat.st_mode):
            raise RuntimeError(
                "release_manifest.py 必须是 regular file: %s" % manifest_source
            )
        initial_identity = (
            initial_stat.st_dev,
            initial_stat.st_ino,
            initial_stat.st_size,
            initial_stat.st_mtime_ns,
        )
        source_parts = []
        while True:
            chunk = os.read(manifest_fd, 1024 * 1024)
            if not chunk:
                break
            source_parts.append(chunk)
        source = b"".join(source_parts)
        before_exec = os.fstat(manifest_fd)
        before_exec_identity = (
            before_exec.st_dev,
            before_exec.st_ino,
            before_exec.st_size,
            before_exec.st_mtime_ns,
        )
        if before_exec_identity != initial_identity or len(source) != initial_stat.st_size:
            raise RuntimeError("release_manifest.py 在读取期间发生漂移")

        module_name = "nuwax_geometry_release_manifest"
        spec = importlib.util.spec_from_loader(
            module_name, loader=None, origin=str(manifest_source)
        )
        if spec is None:
            raise RuntimeError("无法为 release_manifest.py 创建模块 spec")
        module = importlib.util.module_from_spec(spec)
        module.__file__ = str(manifest_source)
        sys.modules[module_name] = module
        try:
            code = compile(source, str(manifest_source), "exec")
            exec(code, module.__dict__)
            after_exec = os.fstat(manifest_fd)
            after_exec_identity = (
                after_exec.st_dev,
                after_exec.st_ino,
                after_exec.st_size,
                after_exec.st_mtime_ns,
            )
            if after_exec_identity != initial_identity:
                raise RuntimeError("release_manifest.py 在执行期间发生漂移")
            required_symbols = (
                "_artifact_set_generation",
                "_canonical_json_bytes",
                "verify_release_manifest",
            )
            missing = [name for name in required_symbols if not hasattr(module, name)]
            if missing:
                raise RuntimeError(
                    "release_manifest.py 缺少必要符号: %s" % ", ".join(missing)
                )
        except BaseException:
            sys.modules.pop(module_name, None)
            raise
        return module
    finally:
        for fd in reversed(opened_fds):
            os.close(fd)


_release_manifest = _load_release_manifest_module()
_artifact_set_generation = _release_manifest._artifact_set_generation
_canonical_json_bytes = _release_manifest._canonical_json_bytes
verify_release_manifest = _release_manifest.verify_release_manifest

_SCHEMA_VERSION = "nuwax.geometry.release.v1"
_RECEIPT_SCHEMA_VERSION = "nuwax.geometry.release-stage.v1"
_CLOSURE_SCHEMA_VERSION = "nuwax.geometry.build-inputs.v3"
_ISOLATED_TARGET_SENTINEL = "<NUWAX_ISOLATED_CARGO_TARGET_DIR>"
_PROFILE_PATH = REPO_ROOT / "deploy" / "profiles" / "go2-jetson.json"
_SOURCE_SUFFIXES = {
    ".c", ".cc", ".cmake", ".cpp", ".h", ".hpp", ".lock", ".lcm",
    ".proto", ".rs", ".toml",
}
_SOURCE_NAMES = {"CMakeLists.txt"}
_BUILD_ENV_KEYS = (
    "AR", "CC", "CFLAGS", "CPPFLAGS", "CXX", "CXXFLAGS", "HOST", "LDFLAGS",
    "LIBLCM_PKG_CONFIG_PATH", "NUWAX_FORCE_LIBLCM_LINK", "PKG_CONFIG",
    "PKG_CONFIG_ALLOW_CROSS", "PKG_CONFIG_PATH", "PKG_CONFIG_SYSROOT_DIR",
    "RUSTC", "RUSTDOC", "RUSTFLAGS", "RUSTUP_TOOLCHAIN",
    "CARGO_ENCODED_RUSTFLAGS", "CARGO_HOME", "CARGO_TARGET_DIR",
    "CARGO_BUILD_RUSTC", "CARGO_BUILD_RUSTFLAGS", "CARGO_BUILD_TARGET", "TARGET",
)
_RERUN_CHANGED = re.compile(r"cargo:rerun-if-changed=([^\"{}]+)")
_RERUN_FORMATTED = re.compile(r'println!\(\s*"cargo:rerun-if-changed=\{([A-Za-z_][A-Za-z0-9_]*)\}"')
_RUST_STRING = re.compile(r'let\s+([A-Za-z_][A-Za-z0-9_]*)\s*=\s*"([^"{}]+)"\s*;')
_RERUN_ENV = re.compile(r"cargo:rerun-if-env-changed=([A-Za-z_][A-Za-z0-9_]*)")
_O_DIRECTORY = getattr(os, "O_DIRECTORY", 0)
_O_NOFOLLOW = getattr(os, "O_NOFOLLOW", 0)
_O_CLOEXEC = getattr(os, "O_CLOEXEC", 0)
_RENAME_NOREPLACE = 1


def _run(command, *, capture=False, env=None):
    # type: (Sequence[str], bool, Optional[Mapping[str, str]]) -> str
    completed = subprocess.run(
        list(command), cwd=str(REPO_ROOT), env=dict(env) if env is not None else None,
        check=True, stdout=subprocess.PIPE if capture else None,
        stderr=subprocess.PIPE if capture else None, text=True,
    )
    return completed.stdout if capture else ""


def _cargo_target_directory(cargo, manifest, *, env=None):
    # type: (str, Path, Optional[Mapping[str, str]]) -> Path
    payload = json.loads(_run((cargo, "metadata", "--format-version", "1", "--no-deps",
                               "--manifest-path", str(manifest)), capture=True, env=env))
    return Path(payload["target_directory"]).resolve()


def _sha256_fd(fd):
    # type: (int) -> str
    digest = hashlib.sha256()
    os.lseek(fd, 0, os.SEEK_SET)
    while True:
        block = os.read(fd, 1024 * 1024)
        if not block:
            break
        digest.update(block)
    os.lseek(fd, 0, os.SEEK_SET)
    return digest.hexdigest()


def _read_all_fd(fd):
    # type: (int) -> bytes
    chunks = []
    os.lseek(fd, 0, os.SEEK_SET)
    while True:
        block = os.read(fd, 1024 * 1024)
        if not block:
            break
        chunks.append(block)
    os.lseek(fd, 0, os.SEEK_SET)
    return b"".join(chunks)


def _sha256_file(path):
    # type: (Path) -> str
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            block = stream.read(1024 * 1024)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def _iter_build_inputs(package_roots):
    # type: (Sequence[Path]) -> Iterable[Path]
    roots = set(package_roots)
    roots.update({
        REPO_ROOT / "services" / "navigation" / "proto",
        REPO_ROOT / "services" / "nuwax-workspace" / "lcmtypes",
        REPO_ROOT / "services" / "nuwax-workspace" / "Cargo.toml",
        REPO_ROOT / "services" / "nuwax-workspace" / "Cargo.lock",
    })
    for root in sorted(roots, key=lambda item: item.as_posix()):
        candidates = root.rglob("*") if root.is_dir() else (root,)
        for path in candidates:
            if any(part in {".git", "target"} for part in path.parts):
                continue
            if not path.is_file() or path.is_symlink():
                continue
            if path.name in _SOURCE_NAMES or path.suffix.lower() in _SOURCE_SUFFIXES:
                yield path


def _command_evidence(command, *, env=None):
    # type: (Sequence[str], Optional[Mapping[str, str]]) -> str
    try:
        return _run(command, capture=True, env=env).strip()
    except (OSError, subprocess.CalledProcessError) as exc:
        raise RuntimeError("无法取得 build-input toolchain 证据: %s" % " ".join(command)) from exc


def _rust_host(rustc_evidence):
    # type: (str) -> str
    for line in rustc_evidence.splitlines():
        if line.startswith("host: "):
            return line[6:].strip()
    raise RuntimeError("rustc --version --verbose 未提供 host triple")


def _resolve_rerun_inputs(package_roots):
    # type: (Sequence[Path]) -> Tuple[Set[Path], Set[str]]
    files = set()  # type: Set[Path]
    environment_keys = set(_BUILD_ENV_KEYS)
    for package_root in package_roots:
        build_script = package_root / "build.rs"
        if not build_script.is_file() or build_script.is_symlink():
            continue
        encoded = build_script.read_text(encoding="utf-8")
        environment_keys.update(_RERUN_ENV.findall(encoded))
        raw_rerun_paths = list(_RERUN_CHANGED.findall(encoded))
        bindings = dict(_RUST_STRING.findall(encoded))
        for name in _RERUN_FORMATTED.findall(encoded):
            value = bindings.get(name)
            if value is None:
                raise RuntimeError("build.rs 含无法解析的 rerun-if-changed 插值: %s" % build_script)
            raw_rerun_paths.append(value)
        if encoded.count("cargo:rerun-if-changed=") != len(raw_rerun_paths):
            raise RuntimeError("build.rs rerun-if-changed 未被完整解析: %s" % build_script)
        if "cargo:rerun-if-env-changed=" in encoded and not _RERUN_ENV.findall(encoded):
            raise RuntimeError("build.rs 含无法静态解析的 rerun-if-env-changed: %s" % build_script)
        for raw in raw_rerun_paths:
            candidate = (package_root / raw.strip()).resolve(strict=True)
            try:
                candidate.relative_to(REPO_ROOT)
            except ValueError as exc:
                raise RuntimeError("build.rs rerun-if-changed 逃逸仓库: %s" % candidate) from exc
            if candidate.is_dir():
                files.update(path for path in candidate.rglob("*") if path.is_file() and not path.is_symlink())
            elif candidate.is_file() and not candidate.is_symlink():
                files.add(candidate)
            else:
                raise RuntimeError("build.rs rerun-if-changed 不是 regular input: %s" % candidate)
    return files, environment_keys


def _capture_build_input_closure(cargo, manifests, *, env):
    # type: (str, Sequence[Path], Mapping[str, str]) -> Dict[str, object]
    resolved_packages = {}  # type: Dict[str, str]
    package_roots = set()  # type: Set[Path]
    for manifest in manifests:
        payload = json.loads(_run((cargo, "metadata", "--format-version", "1", "--locked",
                                   "--manifest-path", str(manifest)), capture=True, env=env))
        for package in payload.get("packages", []):
            manifest_path = Path(str(package.get("manifest_path", "")))
            if package.get("source") is None and manifest_path.is_absolute():
                try:
                    relative = manifest_path.relative_to(REPO_ROOT).as_posix()
                except ValueError as exc:
                    raise RuntimeError("Cargo 解析到仓库外本地 package") from exc
                resolved_packages[str(package.get("id", ""))] = relative
                package_roots.add(manifest_path.parent)
    rerun_files, environment_keys = _resolve_rerun_inputs(tuple(package_roots))
    inputs = set(_iter_build_inputs(tuple(package_roots)))
    inputs.update(rerun_files)
    cargo_config = REPO_ROOT / "services" / "nuwax-workspace" / ".cargo" / "config.toml"
    if cargo_config.is_file() and not cargo_config.is_symlink():
        inputs.add(cargo_config)
    files = []
    for path in sorted(inputs, key=lambda item: item.as_posix()):
        metadata = path.stat()
        files.append({"path": path.relative_to(REPO_ROOT).as_posix(),
                      "size": metadata.st_size, "sha256": _sha256_file(path)})
    rustc_evidence = _command_evidence((env.get("RUSTC") or "rustc", "--version", "--verbose"), env=env)
    host = _rust_host(rustc_evidence)
    target = env.get("CARGO_BUILD_TARGET") or env.get("TARGET") or host
    recorded_environment = []
    for key in sorted(environment_keys):
        if key == "CARGO_TARGET_DIR":
            recorded_environment.append({"name": key, "present": True,
                                         "value": _ISOLATED_TARGET_SENTINEL})
            continue
        value = env.get(key)
        recorded_environment.append({"name": key, "present": value is not None,
                                     "value": value if value is not None else ""})
    return {
        "schema_version": _CLOSURE_SCHEMA_VERSION,
        "cargo_target_directory": "isolated",
        "contract_digests": {
            "builder_sha256": _sha256_file(Path(__file__).resolve()),
            "profile_sha256": _sha256_file(_PROFILE_PATH),
        },
        "files": files,
        "resolved_local_packages": [
            {"id": package_id, "manifest_path": resolved_packages[package_id]}
            for package_id in sorted(resolved_packages)
        ],
        "build_environment": recorded_environment,
        "toolchain": {
            "cargo_version": _command_evidence((cargo, "--version"), env=env),
            "rustc_version": rustc_evidence,
            "lcm_modversion": _command_evidence((env.get("PKG_CONFIG") or "pkg-config", "--modversion", "lcm"), env=env),
            "lcm_cflags": _command_evidence((env.get("PKG_CONFIG") or "pkg-config", "--cflags", "lcm"), env=env),
            "lcm_libs": _command_evidence((env.get("PKG_CONFIG") or "pkg-config", "--libs", "lcm"), env=env),
            "host": host, "target": target,
        },
    }


# 保留旧私有名称，避免现有调用方在默认模式下失效。
def _build_input_closure(cargo, manifests, *, env):
    # type: (str, Sequence[Path], Mapping[str, str]) -> Dict[str, object]
    return _capture_build_input_closure(cargo, manifests, env=env)


def _verify_stage_build_inputs(expected, cargo, manifests, env):
    # type: (Mapping[str, object], str, Sequence[Path], Mapping[str, str]) -> None
    current = _capture_build_input_closure(cargo, manifests, env=env)
    if _canonical_json_bytes(current) != _canonical_json_bytes(expected):
        raise RuntimeError("build-input closure 与当前仓库、profile、锁文件或工具链不一致")


def _profile_native_lio_contract(profile_path):
    # type: (Path) -> List[Tuple[str, str]]
    payload = json.loads(profile_path.read_text(encoding="utf-8"))
    declared = payload.get("metadata", {}).get("geometry_release", {}).get("native_lio_payloads")
    if not isinstance(declared, list) or not declared:
        raise ValueError("发布 profile 必须声明非空 native_lio_payloads")
    result = []
    names = set()
    sources = set()
    for item in declared:
        if not isinstance(item, dict) or set(item) != {"role", "source"}:
            raise ValueError("发布 profile native LIO item 字段集合不精确")
        name = item.get("role")
        source = item.get("source")
        if (not isinstance(name, str) or not name
                or any(character not in "abcdefghijklmnopqrstuvwxyz0123456789_-" for character in name)
                or not isinstance(source, str) or not source):
            raise ValueError("发布 profile native LIO role/source 非法")
        if name in names or source in sources:
            raise ValueError("发布 profile native LIO role/source 重复")
        names.add(name)
        sources.add(source)
        result.append((name, source))
    return sorted(result)


def _parse_native_lio(values, profile_contract):
    # type: (Sequence[str], Sequence[Tuple[str, str]]) -> List[Tuple[str, Path]]
    by_source = {source: role for role, source in profile_contract}
    resolved = []
    seen_sources = set()
    for raw_path in values:
        path = Path(raw_path)
        if not path.is_absolute():
            raise ValueError("--native-lio 必须只提供绝对 artifact path: %s" % raw_path)
        source = path.name
        role = by_source.get(source)
        if role is None:
            raise ValueError("--native-lio path basename 不属于发布 profile source 集合: %s" % source)
        if source in seen_sources:
            raise ValueError("--native-lio source 重复: %s" % source)
        seen_sources.add(source)
        resolved.append((role, path))
    expected_sources = set(by_source)
    if seen_sources != expected_sources:
        raise ValueError("--native-lio path 集合必须覆盖发布 profile 全部 source: expected=%s actual=%s"
                         % (",".join(sorted(expected_sources)), ",".join(sorted(seen_sources))))
    return sorted(resolved, key=lambda item: item[0])


def _validate_source(path, role, *, executable):
    # type: (Path, str, bool) -> Path
    try:
        metadata = path.lstat()
    except OSError as exc:
        raise RuntimeError("release artifact 缺失 role=%s path=%s" % (role, path)) from exc
    if stat.S_ISLNK(metadata.st_mode) or not stat.S_ISREG(metadata.st_mode):
        raise RuntimeError("release artifact 必须是非 symlink regular file role=%s" % role)
    if executable and not metadata.st_mode & 0o111:
        raise RuntimeError("release artifact 不可执行 role=%s path=%s" % (role, path))
    return path.resolve(strict=True)


def _embedded_artifacts(runtime):
    # type: (Path) -> List[Tuple[str, Path]]
    try:
        payload = json.loads(subprocess.run(
            (str(runtime), "--print-release-artifacts"), check=True,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
            cwd=str(REPO_ROOT)).stdout)
    except (OSError, subprocess.CalledProcessError, json.JSONDecodeError) as exc:
        raise RuntimeError("Geometry embedded-artifact discovery 失败") from exc
    if not isinstance(payload, dict) or set(payload) != {"schema_version", "artifacts"}:
        raise RuntimeError("Geometry embedded-artifact discovery 顶层合同无效")
    if payload.get("schema_version") != "nuwax.geometry.embedded-artifacts.v1":
        raise RuntimeError("Geometry embedded-artifact discovery schema 不受支持")
    artifacts = payload.get("artifacts")
    if not isinstance(artifacts, list):
        raise RuntimeError("Geometry embedded-artifact discovery artifacts 无效")
    result = []
    for item in artifacts:
        if not isinstance(item, dict) or set(item) != {"role", "path"}:
            raise RuntimeError("Geometry embedded-artifact discovery item 无效")
        result.append((str(item["role"]), Path(str(item["path"]))))
    if {role for role, _ in result} != {"native_pgo_payload", "group_guardian"}:
        raise RuntimeError("Geometry embedded-artifact discovery 角色集合不精确")
    return result


def _copy_artifact(staging, role, source, *, executable):
    # type: (Path, str, Path, bool) -> Dict[str, object]
    source = _validate_source(source, role, executable=executable)
    relative = Path("artifacts") / role.replace(":", "-") / source.name
    destination = staging / relative
    destination.parent.mkdir(parents=True, exist_ok=False)
    shutil.copy2(str(source), str(destination), follow_symlinks=False)
    return {"role": role, "path": relative.as_posix(), "size": destination.stat().st_size,
            "sha256": _sha256_file(destination),
            "executable": bool(destination.stat().st_mode & 0o111)}


def _seal_stage(stage):
    # type: (Path) -> None
    for file_path in (candidate for candidate in stage.rglob("*") if candidate.is_file()):
        os.chmod(str(file_path), 0o555 if file_path.stat().st_mode & 0o111 else 0o444)
    for directory in sorted((candidate for candidate in stage.rglob("*") if candidate.is_dir()),
                            key=lambda candidate: len(candidate.parts), reverse=True):
        os.chmod(str(directory), 0o555)
    os.chmod(str(stage), 0o555)


def _make_tree_removable(path):
    # type: (Path) -> None
    directories = [candidate for candidate in path.rglob("*")
                   if candidate.is_dir() and not candidate.is_symlink()]
    for candidate in sorted(directories, key=lambda item: len(item.parts)):
        os.chmod(str(candidate), 0o700)
    os.chmod(str(path), 0o700)


def _validate_directory_fd(fd, label, expected_uid):
    # type: (int, str, int) -> os.stat_result
    metadata = os.fstat(fd)
    if (not stat.S_ISDIR(metadata.st_mode) or metadata.st_uid != expected_uid
            or stat.S_IMODE(metadata.st_mode) & 0o022):
        raise RuntimeError("%s owner/mode/type 不安全" % label)
    return metadata


def _write_all(fd, encoded, label):
    # type: (int, bytes, str) -> None
    view = memoryview(encoded)
    while view:
        written = os.write(fd, view)
        if written <= 0:
            raise RuntimeError("%s 写入零进展" % label)
        view = view[written:]


def _fsync_stage_payload(directory_fd):
    # type: (int) -> None
    for name in sorted(os.listdir(directory_fd)):
        fd = os.open(name, os.O_RDONLY | _O_NOFOLLOW | _O_CLOEXEC, dir_fd=directory_fd)
        try:
            metadata = os.fstat(fd)
            if stat.S_ISDIR(metadata.st_mode):
                _fsync_stage_payload(fd)
                os.fsync(fd)
            elif stat.S_ISREG(metadata.st_mode):
                os.fsync(fd)
            else:
                raise RuntimeError("stage payload 含非 regular/dir node: %s" % name)
        finally:
            os.close(fd)
    os.fsync(directory_fd)


def _create_stage(parent, native_lio, runtime, mapping_cli, build_inputs):
    # type: (Path, Sequence[Tuple[str, Path]], Path, Path, Mapping[str, object]) -> Path
    parent_fd = _open_absolute_directory(parent)
    stage_fd = None  # type: Optional[int]
    basename = ""
    try:
        parent_before = _validate_directory_fd(parent_fd, "--stage-only parent", os.geteuid())
        for _attempt in range(128):
            candidate = "nuwax-geometry-stage-%s" % secrets.token_hex(16)
            try:
                os.mkdir(candidate, 0o700, dir_fd=parent_fd)
                basename = candidate
                break
            except FileExistsError:
                continue
        if not basename:
            raise RuntimeError("无法创建随机 stage 目录")
        stage_fd = os.open(basename, os.O_RDONLY | _O_DIRECTORY | _O_NOFOLLOW | _O_CLOEXEC,
                           dir_fd=parent_fd)
        _validate_directory_fd(stage_fd, "新建 stage", os.geteuid())
        stable_stage = Path("/proc/self/fd/%d" % stage_fd)
        generation_root = stable_stage / "generation"
        generation_root.mkdir(mode=0o700)
        try:
            sources = [("geometry_runtime", runtime, True),
                       ("offline_mapping_cli", mapping_cli, True)]
            sources.extend((role, path, True) for role, path in _embedded_artifacts(runtime))
            sources.extend(("native_lio_payload:%s" % name, path, True) for name, path in native_lio)
            if len({role for role, _, _ in sources}) != len(sources):
                raise RuntimeError("release artifact role 重复")
            if len({path.resolve(strict=True) for _, path, _ in sources}) != len(sources):
                raise RuntimeError("release artifact source path 重复；同一 payload 应使用一个聚合 profile role")
            entries = [_copy_artifact(generation_root, role, path, executable=executable)
                       for role, path, executable in sorted(sources, key=lambda item: item[0])]
            closure_path = generation_root / "artifacts" / "build_input_closure" / "build-inputs.json"
            closure_path.parent.mkdir(parents=True, exist_ok=False)
            closure_bytes = _canonical_json_bytes(build_inputs)
            closure_path.write_bytes(closure_bytes)
            entries.append({"role": "build_input_closure",
                            "path": closure_path.relative_to(generation_root).as_posix(),
                            "size": closure_path.stat().st_size,
                            "sha256": _sha256_file(closure_path), "executable": False})
            entries.sort(key=lambda item: str(item["role"]))
            generation = _artifact_set_generation(entries)
            manifest = {"schema_version": _SCHEMA_VERSION, "generation": generation,
                        "native_lio_roles": sorted(name for name, _ in native_lio),
                        "artifacts": entries}
            manifest_bytes = _canonical_json_bytes(manifest)
            (generation_root / "manifest.json").write_bytes(manifest_bytes)
            receipt = {
                "schema_version": _RECEIPT_SCHEMA_VERSION,
                "stage_nonce": secrets.token_hex(32),
                "creator_uid": os.geteuid(),
                "builder_sha256": _sha256_file(Path(__file__).resolve()),
                "profile_sha256": _sha256_file(_PROFILE_PATH),
                "manifest_sha256": hashlib.sha256(manifest_bytes).hexdigest(),
                "generation": generation,
                "closure_sha256": hashlib.sha256(closure_bytes).hexdigest(),
            }
            # receipt 是 durable seal 标志；payload 与目录必须先全部落盘。
            _fsync_stage_payload(stage_fd)
            flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | _O_NOFOLLOW | _O_CLOEXEC
            receipt_fd = os.open("receipt.json", flags, 0o400, dir_fd=stage_fd)
            try:
                _write_all(receipt_fd, _canonical_json_bytes(receipt), "stage receipt")
                os.fsync(receipt_fd)
            finally:
                os.close(receipt_fd)
            os.fsync(stage_fd)
            _seal_stage(stable_stage)
            os.fsync(stage_fd)
            parent_after = os.fstat(parent_fd)
            if ((parent_after.st_dev, parent_after.st_ino)
                    != (parent_before.st_dev, parent_before.st_ino)):
                raise RuntimeError("stage parent identity 在创建期间发生变化")
            os.fsync(parent_fd)
            return parent / basename
        except Exception:
            _make_tree_removable(stable_stage)
            _remove_tree_at(parent_fd, basename)
            os.fsync(parent_fd)
            raise
    finally:
        if stage_fd is not None:
            os.close(stage_fd)
        os.close(parent_fd)


def _open_absolute_directory(path):
    # type: (Path) -> int
    if not path.is_absolute():
        raise RuntimeError("安全目录路径必须是绝对路径: %s" % path)
    current_fd = os.open("/", os.O_RDONLY | _O_DIRECTORY | _O_NOFOLLOW | _O_CLOEXEC)
    try:
        for part in path.parts[1:]:
            if part in {"", ".", ".."}:
                raise RuntimeError("安全目录路径组件非法: %s" % path)
            next_fd = os.open(part, os.O_RDONLY | _O_DIRECTORY | _O_NOFOLLOW | _O_CLOEXEC,
                              dir_fd=current_fd)
            os.close(current_fd)
            current_fd = next_fd
        return current_fd
    except Exception:
        os.close(current_fd)
        raise


def _open_or_create_absolute_directory(path, expected_uid):
    # type: (Path, int) -> int
    if not path.is_absolute():
        raise RuntimeError("安全目录路径必须是绝对路径: %s" % path)
    current_fd = os.open("/", os.O_RDONLY | _O_DIRECTORY | _O_NOFOLLOW | _O_CLOEXEC)
    try:
        for part in path.parts[1:]:
            if part in {"", ".", ".."}:
                raise RuntimeError("安全目录路径组件非法: %s" % path)
            try:
                os.mkdir(part, 0o700, dir_fd=current_fd)
                os.fsync(current_fd)
            except FileExistsError:
                pass
            next_fd = os.open(part, os.O_RDONLY | _O_DIRECTORY | _O_NOFOLLOW | _O_CLOEXEC,
                              dir_fd=current_fd)
            os.close(current_fd)
            current_fd = next_fd
        _validate_directory_fd(current_fd, "publish immediate parent", expected_uid)
        return current_fd
    except Exception:
        os.close(current_fd)
        raise


def _open_directory_at(directory_fd, name, stage_uid, label):
    # type: (int, str, int, str) -> int
    fd = os.open(name, os.O_RDONLY | _O_DIRECTORY | _O_NOFOLLOW | _O_CLOEXEC,
                 dir_fd=directory_fd)
    try:
        _validate_directory_fd(fd, label, stage_uid)
        return fd
    except Exception:
        os.close(fd)
        raise


def _open_regular_at(directory_fd, name, stage_uid, label):
    # type: (int, str, int, str) -> int
    fd = os.open(name, os.O_RDONLY | os.O_NONBLOCK | _O_NOFOLLOW | _O_CLOEXEC,
                 dir_fd=directory_fd)
    try:
        metadata = os.fstat(fd)
        if (not stat.S_ISREG(metadata.st_mode) or metadata.st_uid != stage_uid
                or stat.S_IMODE(metadata.st_mode) & 0o022 or metadata.st_nlink != 1):
            raise RuntimeError("stage 文件 owner/mode/type/link 不安全: %s" % label)
        return fd
    except Exception:
        os.close(fd)
        raise


def _load_canonical_fd(fd, label):
    # type: (int, str) -> Tuple[bytes, Dict[str, object]]
    encoded = _read_all_fd(fd)
    try:
        payload = json.loads(encoded.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise RuntimeError("%s 不是有效 UTF-8 JSON" % label) from exc
    if not isinstance(payload, dict) or encoded != _canonical_json_bytes(payload):
        raise RuntimeError("%s 必须是 canonical JSON object" % label)
    return encoded, payload


def _intake_stage_owned(stage_path, *, expected_uid=None):
    # type: (Path, Optional[int]) -> Tuple[int, Dict[str, int], Dict[str, object], Dict[str, object], Dict[str, object]]
    stage_fd = _open_absolute_directory(stage_path)
    opened = {}  # type: Dict[str, int]
    directories = set()  # type: Set[str]
    try:
        stage_metadata = os.fstat(stage_fd)
        stage_uid = stage_metadata.st_uid
        if expected_uid is None:
            if stage_uid == 0:
                raise RuntimeError("external stage 必须由普通用户拥有")
        elif stage_uid != expected_uid:
            raise RuntimeError("private stage owner 与预期 UID 不一致")
        _validate_directory_fd(stage_fd, "stage root", stage_uid)

        def walk(directory_fd, prefix):
            # type: (int, str) -> None
            for name in sorted(os.listdir(directory_fd)):
                if name in {"", ".", ".."} or "/" in name:
                    raise RuntimeError("stage 含非法目录项")
                relative = name if not prefix else "%s/%s" % (prefix, name)
                try:
                    child_fd = _open_directory_at(directory_fd, name, stage_uid, relative)
                except (NotADirectoryError, OSError) as exc:
                    if not isinstance(exc, NotADirectoryError) and getattr(exc, "errno", None) != errno.ENOTDIR:
                        raise RuntimeError("stage 目录无法安全 openat: %s" % relative) from exc
                    child_fd = None
                if child_fd is not None:
                    directories.add(relative)
                    try:
                        walk(child_fd, relative)
                    finally:
                        os.close(child_fd)
                    continue
                try:
                    opened[relative] = _open_regular_at(directory_fd, name, stage_uid, relative)
                except OSError as exc:
                    raise RuntimeError("stage 含 symlink 或非 regular/dir node: %s" % relative) from exc

        walk(stage_fd, "")
        if "receipt.json" not in opened or "generation/manifest.json" not in opened:
            raise RuntimeError("stage 缺少 receipt 或 generation manifest")
        _receipt_bytes, receipt = _load_canonical_fd(opened["receipt.json"], "stage receipt")
        expected_receipt_fields = {"schema_version", "stage_nonce", "creator_uid",
                                   "builder_sha256", "profile_sha256", "manifest_sha256",
                                   "generation", "closure_sha256"}
        if set(receipt) != expected_receipt_fields:
            raise RuntimeError("stage receipt 字段集合不精确")
        if receipt.get("schema_version") != _RECEIPT_SCHEMA_VERSION:
            raise RuntimeError("stage receipt schema 不受支持")
        nonce = receipt.get("stage_nonce")
        digests = [receipt.get("builder_sha256"), receipt.get("profile_sha256"),
                   receipt.get("manifest_sha256"), receipt.get("generation"),
                   receipt.get("closure_sha256")]
        if (not isinstance(nonce, str) or len(nonce) != 64
                or any(character not in "0123456789abcdef" for character in nonce)
                or any(not isinstance(value, str) or len(value) != 64
                       or any(character not in "0123456789abcdef" for character in value)
                       for value in digests)):
            raise RuntimeError("stage receipt nonce/digest 非法")
        if receipt.get("creator_uid") != stage_uid:
            raise RuntimeError("stage receipt creator UID 与固定目录 owner 不一致")
        if receipt.get("builder_sha256") != _sha256_file(Path(__file__).resolve()):
            raise RuntimeError("stage builder digest 与当前 finalizer 不一致")
        if receipt.get("profile_sha256") != _sha256_file(_PROFILE_PATH):
            raise RuntimeError("stage profile digest 与当前 profile 不一致")
        manifest_bytes, manifest = _load_canonical_fd(opened["generation/manifest.json"], "stage manifest")
        if hashlib.sha256(manifest_bytes).hexdigest() != receipt.get("manifest_sha256"):
            raise RuntimeError("stage manifest digest 与 receipt 不一致")
        if manifest.get("generation") != receipt.get("generation"):
            raise RuntimeError("stage generation 与 receipt 不一致")
        raw_artifacts = manifest.get("artifacts")
        if not isinstance(raw_artifacts, list):
            raise RuntimeError("stage manifest artifacts 无效")
        expected_files = {"receipt.json", "generation/manifest.json"}
        expected_directories = {"generation"}
        for entry in raw_artifacts:
            if not isinstance(entry, dict) or set(entry) != {"role", "path", "size", "sha256", "executable"}:
                raise RuntimeError("stage manifest artifact 字段集合不精确")
            relative = str(entry.get("path") or "")
            parsed = Path(relative)
            if (not relative or parsed.is_absolute() or relative != parsed.as_posix()
                    or any(part in {"", ".", ".."} for part in parsed.parts)):
                raise RuntimeError("stage manifest artifact path 非规范")
            expected_files.add("generation/%s" % relative)
            current = Path("generation")
            for part in parsed.parts[:-1]:
                current = current / part
                expected_directories.add(current.as_posix())
        if set(opened) != expected_files:
            raise RuntimeError("stage 精确文件集合与 manifest 不一致")
        if directories != expected_directories:
            raise RuntimeError("stage 精确目录集合与 manifest 不一致")
        closure_entries = [entry for entry in raw_artifacts if entry.get("role") == "build_input_closure"]
        if len(closure_entries) != 1:
            raise RuntimeError("stage build_input_closure role 不精确")
        closure_key = "generation/%s" % closure_entries[0]["path"]
        closure_bytes, closure = _load_canonical_fd(opened[closure_key], "build-input closure")
        if hashlib.sha256(closure_bytes).hexdigest() != receipt.get("closure_sha256"):
            raise RuntimeError("stage closure digest 与 receipt 不一致")
        if closure.get("schema_version") != _CLOSURE_SCHEMA_VERSION:
            raise RuntimeError("stage closure schema 不受支持")
        return stage_fd, opened, receipt, closure, manifest
    except Exception:
        for fd in opened.values():
            os.close(fd)
        os.close(stage_fd)
        raise


def _intake_external_stage(stage_path, expected_uid):
    # type: (Path, int) -> Tuple[int, Dict[str, int], Dict[str, object], Dict[str, object], Dict[str, object]]
    return _intake_stage_owned(stage_path, expected_uid=expected_uid)


def _intake_private_root_stage(stage_path):
    # type: (Path) -> Tuple[int, Dict[str, int], Dict[str, object], Dict[str, object], Dict[str, object]]
    return _intake_stage_owned(stage_path, expected_uid=0)


def _mkdirs_at(root_fd, relative_parent):
    # type: (int, Sequence[str]) -> None
    current_fd = os.dup(root_fd)
    try:
        for part in relative_parent:
            try:
                os.mkdir(part, 0o700, dir_fd=current_fd)
            except FileExistsError:
                pass
            next_fd = os.open(part, os.O_RDONLY | _O_DIRECTORY | _O_NOFOLLOW | _O_CLOEXEC,
                              dir_fd=current_fd)
            os.close(current_fd)
            current_fd = next_fd
    finally:
        os.close(current_fd)


def _copy_fd_to_tree(source_fd, root_fd, relative, executable):
    # type: (int, int, str, bool) -> None
    parts = relative.split("/")
    _mkdirs_at(root_fd, parts[:-1])
    directory_fd = os.dup(root_fd)
    try:
        for part in parts[:-1]:
            next_fd = os.open(part, os.O_RDONLY | _O_DIRECTORY | _O_NOFOLLOW | _O_CLOEXEC,
                              dir_fd=directory_fd)
            os.close(directory_fd)
            directory_fd = next_fd
        destination_fd = os.open(parts[-1], os.O_WRONLY | os.O_CREAT | os.O_EXCL
                                 | _O_NOFOLLOW | _O_CLOEXEC, 0o600, dir_fd=directory_fd)
        try:
            os.lseek(source_fd, 0, os.SEEK_SET)
            while True:
                block = os.read(source_fd, 1024 * 1024)
                if not block:
                    break
                view = memoryview(block)
                while view:
                    written = os.write(destination_fd, view)
                    view = view[written:]
            os.fchmod(destination_fd, 0o755 if executable else 0o644)
            os.fsync(destination_fd)
        finally:
            os.close(destination_fd)
            os.lseek(source_fd, 0, os.SEEK_SET)
    finally:
        os.close(directory_fd)


def _fsync_tree_directories(root_fd):
    # type: (int) -> None
    children = []
    for name in os.listdir(root_fd):
        metadata = os.stat(name, dir_fd=root_fd, follow_symlinks=False)
        if stat.S_ISDIR(metadata.st_mode):
            child_fd = os.open(name, os.O_RDONLY | _O_DIRECTORY | _O_NOFOLLOW | _O_CLOEXEC,
                               dir_fd=root_fd)
            try:
                _fsync_tree_directories(child_fd)
                os.fchmod(child_fd, 0o755)
                os.fsync(child_fd)
            finally:
                os.close(child_fd)
            children.append(name)
    os.fchmod(root_fd, 0o755)
    os.fsync(root_fd)


def _remove_tree_at(parent_fd, name):
    # type: (int, str) -> None
    try:
        metadata = os.stat(name, dir_fd=parent_fd, follow_symlinks=False)
    except FileNotFoundError:
        return
    if not stat.S_ISDIR(metadata.st_mode):
        os.unlink(name, dir_fd=parent_fd)
        return
    directory_fd = os.open(name, os.O_RDONLY | _O_DIRECTORY | _O_NOFOLLOW | _O_CLOEXEC,
                           dir_fd=parent_fd)
    try:
        for child in os.listdir(directory_fd):
            _remove_tree_at(directory_fd, child)
    finally:
        os.close(directory_fd)
    os.rmdir(name, dir_fd=parent_fd)


def _rename_noreplace(source_fd, source, destination_fd, destination):
    # type: (int, str, int, str) -> None
    libc = ctypes.CDLL(None, use_errno=True)
    renameat2 = getattr(libc, "renameat2", None)
    if renameat2 is None:
        raise RuntimeError("当前 libc 不提供 renameat2，拒绝降级 no-clobber 发布")
    result = renameat2(source_fd, source.encode("utf-8"), destination_fd,
                       destination.encode("utf-8"), _RENAME_NOREPLACE)
    if result != 0:
        error = ctypes.get_errno()
        raise OSError(error, os.strerror(error), destination)


def _identity(fd):
    # type: (int) -> Tuple[int, int]
    metadata = os.fstat(fd)
    return metadata.st_dev, metadata.st_ino


def _verify_manifest_with_stable_root(manifest_path, root_fd, expected_generation=None,
                                      expected_manifest_sha256=None, identity_root_path=None):
    # type: (Path, int, Optional[str], Optional[str], Optional[Path]) -> object
    before = _identity(root_fd)
    stable_path = identity_root_path or manifest_path.parent
    path_root_fd = _open_absolute_directory(stable_path)
    try:
        if _identity(path_root_fd) != before:
            raise RuntimeError("pathname checker 未指向固定 generation FD")
    finally:
        os.close(path_root_fd)
    proof = verify_release_manifest(manifest_path)
    after = _identity(root_fd)
    if before != after:
        raise RuntimeError("checker 前后固定 generation FD identity 变化")
    path_root_fd = _open_absolute_directory(stable_path)
    try:
        if _identity(path_root_fd) != before:
            raise RuntimeError("checker 后 pathname 不再指向固定 generation FD")
    finally:
        os.close(path_root_fd)
    if expected_generation is not None and proof.generation != expected_generation:
        raise RuntimeError("checker generation 与预期不一致")
    if (expected_manifest_sha256 is not None
            and proof.manifest_sha256 != expected_manifest_sha256):
        raise RuntimeError("checker manifest digest 与预期不一致")
    return proof


def _manifest_role_contract(manifest):
    # type: (Mapping[str, object]) -> Tuple[Tuple[str, ...], Tuple[str, ...]]
    raw_roles = manifest.get("native_lio_roles")
    raw_artifacts = manifest.get("artifacts")
    if not isinstance(raw_roles, list) or not isinstance(raw_artifacts, list):
        raise RuntimeError("manifest native LIO 合同无效")
    roles = tuple(sorted(str(role) for role in raw_roles))
    artifact_roles = tuple(sorted(str(entry.get("role")) for entry in raw_artifacts
                                  if isinstance(entry, dict)
                                  and str(entry.get("role", "")).startswith("native_lio_payload:")))
    return roles, artifact_roles


def _verify_profile_roles(manifest, profile_contract):
    # type: (Mapping[str, object], Sequence[Tuple[str, str]]) -> Tuple[str, ...]
    expected_by_role = {role: source for role, source in profile_contract}
    expected_roles = tuple(sorted(expected_by_role))
    roles, artifact_roles = _manifest_role_contract(manifest)
    expected_artifacts = tuple("native_lio_payload:%s" % role for role in expected_roles)
    if roles != expected_roles or artifact_roles != expected_artifacts:
        raise RuntimeError("manifest native LIO roles/artifacts 与当前 profile 精确合同不一致")
    native_entries = {str(entry["role"]): str(entry["path"])
                      for entry in manifest["artifacts"]
                      if str(entry.get("role", "")).startswith("native_lio_payload:")}
    for role in expected_roles:
        artifact_role = "native_lio_payload:%s" % role
        if Path(native_entries[artifact_role]).name != expected_by_role[role]:
            raise RuntimeError("manifest native LIO artifact source 与当前 profile 不一致: %s"
                               % role)
    return expected_roles


def _secure_existing_generation(output_root, output_fd, generation, manifest_sha256, expected_uid):
    # type: (Path, int, str, str, int) -> bool
    try:
        generation_fd = os.open(generation, os.O_RDONLY | _O_DIRECTORY | _O_NOFOLLOW | _O_CLOEXEC,
                                dir_fd=output_fd)
    except FileNotFoundError:
        return False
    try:
        _validate_directory_fd(generation_fd, "existing generation", expected_uid)
        _verify_manifest_with_stable_root(output_root / generation / "manifest.json",
                                          generation_fd, generation, manifest_sha256)
        return True
    finally:
        os.close(generation_fd)


def _read_active(parent_fd, active_name):
    # type: (int, str) -> Optional[str]
    try:
        fd = os.open(active_name, os.O_PATH | _O_NOFOLLOW | _O_CLOEXEC, dir_fd=parent_fd)
    except FileNotFoundError:
        return None
    try:
        if not stat.S_ISLNK(os.fstat(fd).st_mode):
            raise RuntimeError("active 路径已存在且不是 symlink")
    finally:
        os.close(fd)
    return os.readlink(active_name, dir_fd=parent_fd)


def _assert_active_state(parent_fd, active_name, expected_target):
    # type: (int, str, Optional[str]) -> None
    actual = _read_active(parent_fd, active_name)
    if actual != expected_target:
        raise RuntimeError("fatal partial-state: 旧 active 未恢复 expected=%r actual=%r"
                           % (expected_target, actual))


def _switch_active(output_root, output_fd, parent_fd, generation, manifest_sha256):
    # type: (Path, int, int, str, str) -> object
    active_name = "nuwax-geometry-release"
    token = "%d.%s" % (os.getpid(), secrets.token_hex(8))
    temporary_name = ".%s.tmp.%s" % (active_name, token)
    rollback_name = ".%s.rollback.%s" % (active_name, token)
    old_target = _read_active(parent_fd, active_name)
    target = Path(output_root.name, generation).as_posix()
    rollback_ready = False
    switched = False
    try:
        if old_target is not None:
            os.symlink(old_target, rollback_name, dir_fd=parent_fd)
            rollback_ready = True
        os.fsync(parent_fd)
        os.symlink(target, temporary_name, dir_fd=parent_fd)
        os.fsync(parent_fd)
        os.replace(temporary_name, active_name, src_dir_fd=parent_fd, dst_dir_fd=parent_fd)
        switched = True
        os.fsync(parent_fd)
        generation_fd = os.open(generation, os.O_RDONLY | _O_DIRECTORY | _O_NOFOLLOW | _O_CLOEXEC,
                                dir_fd=output_fd)
        try:
            proof = _verify_manifest_with_stable_root(
                output_root.parent / active_name / "manifest.json", generation_fd,
                generation, manifest_sha256, output_root / generation)
        finally:
            os.close(generation_fd)
        if rollback_ready:
            os.unlink(rollback_name, dir_fd=parent_fd)
            rollback_ready = False
            os.fsync(parent_fd)
        return proof
    except Exception as original:
        cleanup_errors = []
        try:
            os.unlink(temporary_name, dir_fd=parent_fd)
        except FileNotFoundError:
            pass
        except Exception as exc:
            cleanup_errors.append("temp cleanup=%s" % exc)
        if switched:
            try:
                if old_target is None:
                    os.unlink(active_name, dir_fd=parent_fd)
                elif rollback_ready:
                    os.replace(rollback_name, active_name,
                               src_dir_fd=parent_fd, dst_dir_fd=parent_fd)
                    rollback_ready = False
                else:
                    raise RuntimeError("rollback 恢复节点不存在")
                os.fsync(parent_fd)
            except Exception as exc:
                cleanup_errors.append("rollback=%s" % exc)
        try:
            _assert_active_state(parent_fd, active_name, old_target)
        except Exception as exc:
            cleanup_errors.append(str(exc))
        if rollback_ready:
            try:
                os.unlink(rollback_name, dir_fd=parent_fd)
                rollback_ready = False
                os.fsync(parent_fd)
            except Exception as exc:
                cleanup_errors.append("rollback cleanup=%s" % exc)
        if cleanup_errors:
            raise RuntimeError("fatal partial-state after %s; %s"
                               % (original, "; ".join(cleanup_errors))) from original
        raise
    finally:
        for name in (temporary_name, rollback_name):
            try:
                os.unlink(name, dir_fd=parent_fd)
            except FileNotFoundError:
                pass


def _finalize_existing_stage(args, manifests, *, private_stage=False):
    # type: (argparse.Namespace, Sequence[Path], bool) -> int
    expected_uid = os.geteuid()
    stage_path = args.finalize_existing_stage
    if not stage_path.is_absolute():
        raise RuntimeError("--finalize-existing-stage 必须是 exact absolute stage directory")
    profile_contract = _profile_native_lio_contract(_PROFILE_PATH)
    if private_stage:
        stage_fd, opened, receipt, closure, manifest = _intake_private_root_stage(stage_path)
    else:
        stage_fd, opened, receipt, closure, manifest = _intake_external_stage(
            stage_path, expected_uid)
    output_fd = None  # type: Optional[int]
    parent_fd = None  # type: Optional[int]
    temporary_name = None  # type: Optional[str]
    try:
        expected_roles = _verify_profile_roles(manifest, profile_contract)
        capture_env = dict(os.environ)
        _verify_stage_build_inputs(closure, args.cargo, manifests, capture_env)
        default_output = (REPO_ROOT / "services" / "geometry" / "nuwax_geometry_runtime"
                          / "target" / "release" / "nuwax-geometry-releases")
        output_root = (args.output_root or default_output).absolute()
        if output_root.name in {"", ".", ".."}:
            raise RuntimeError("output-root basename 非法")
        parent_fd = _open_or_create_absolute_directory(output_root.parent, expected_uid)
        try:
            os.mkdir(output_root.name, 0o700, dir_fd=parent_fd)
            os.fsync(parent_fd)
        except FileExistsError:
            pass
        output_fd = os.open(output_root.name,
                            os.O_RDONLY | _O_DIRECTORY | _O_NOFOLLOW | _O_CLOEXEC,
                            dir_fd=parent_fd)
        _validate_directory_fd(output_fd, "publish output-root", expected_uid)
        lock_fd = os.open(".nuwax-geometry-release.publish.lock",
                          os.O_RDWR | os.O_CREAT | _O_NOFOLLOW | _O_CLOEXEC,
                          0o600, dir_fd=parent_fd)
        try:
            lock_metadata = os.fstat(lock_fd)
            if (not stat.S_ISREG(lock_metadata.st_mode)
                    or lock_metadata.st_uid != expected_uid
                    or lock_metadata.st_nlink != 1 or stat.S_IMODE(lock_metadata.st_mode) & 0o077):
                raise RuntimeError("publish namespace lock 属性不安全")
            fcntl.flock(lock_fd, fcntl.LOCK_EX)
            temporary_name = ".verified.%d.%s" % (os.getpid(), secrets.token_hex(16))
            os.mkdir(temporary_name, 0o700, dir_fd=output_fd)
            temp_fd = os.open(temporary_name,
                              os.O_RDONLY | _O_DIRECTORY | _O_NOFOLLOW | _O_CLOEXEC,
                              dir_fd=output_fd)
            try:
                _validate_directory_fd(temp_fd, "runner verified copy", expected_uid)
                executable_by_path = {str(entry["path"]): bool(entry["executable"])
                                      for entry in manifest["artifacts"]}
                for stage_relative, source_fd in sorted(opened.items()):
                    if not stage_relative.startswith("generation/"):
                        continue
                    relative = stage_relative[len("generation/"):]
                    executable = executable_by_path.get(relative, False)
                    _copy_fd_to_tree(source_fd, temp_fd, relative, executable)
                _fsync_tree_directories(temp_fd)
                proof = _verify_manifest_with_stable_root(
                    output_root / temporary_name / "manifest.json", temp_fd,
                    str(receipt["generation"]), str(receipt["manifest_sha256"]))
            finally:
                os.close(temp_fd)
            if tuple(proof.native_lio_roles) != expected_roles:
                raise RuntimeError("runner copy checker native LIO proof 与当前 profile 不一致")
            if _secure_existing_generation(output_root, output_fd, proof.generation,
                                           proof.manifest_sha256, expected_uid):
                _remove_tree_at(output_fd, temporary_name)
                temporary_name = None
            else:
                try:
                    _rename_noreplace(output_fd, temporary_name, output_fd, proof.generation)
                    temporary_name = None
                    os.fsync(output_fd)
                except OSError as exc:
                    if exc.errno != errno.EEXIST:
                        raise
                    if not _secure_existing_generation(
                            output_root, output_fd, proof.generation,
                            proof.manifest_sha256, expected_uid):
                        raise RuntimeError("generation no-clobber conflict") from exc
                    _remove_tree_at(output_fd, temporary_name)
                    temporary_name = None
            active_proof = _switch_active(output_root, output_fd, parent_fd, proof.generation,
                                          proof.manifest_sha256)
            if tuple(active_proof.native_lio_roles) != expected_roles:
                raise RuntimeError("active checker native LIO proof 与当前 profile 不一致")
            print(str(active_proof.manifest_path))
            print(active_proof.manifest_sha256)
            print(active_proof.generation)
            return 0
        finally:
            if temporary_name is not None:
                _remove_tree_at(output_fd, temporary_name)
            os.close(lock_fd)
    finally:
        if output_fd is not None:
            os.close(output_fd)
        if parent_fd is not None:
            os.close(parent_fd)
        for fd in opened.values():
            os.close(fd)
        os.close(stage_fd)


def _build_parser():
    # type: () -> argparse.ArgumentParser
    parser = argparse.ArgumentParser(description="构建 Geometry immutable release artifact set")
    parser.add_argument("--cargo", default="cargo")
    parser.add_argument("--native-lio", action="append", default=[], metavar="/ABS/PATH")
    parser.add_argument("--output-root", type=Path, default=None)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--stage-only", type=Path, metavar="PARENT")
    modes.add_argument("--finalize-existing-stage", type=Path, metavar="EXACT_STAGE_DIR")
    return parser


def _validate_mode_arguments(parser, args):
    # type: (argparse.ArgumentParser, argparse.Namespace) -> None
    if args.stage_only is not None:
        if args.output_root is not None:
            parser.error("--stage-only 不得与 --output-root 组合")
        if not args.stage_only.is_absolute():
            parser.error("--stage-only 必须提供绝对 parent path")
    elif args.finalize_existing_stage is not None:
        if args.native_lio:
            parser.error("--finalize-existing-stage 不得提供 --native-lio 或重新 build 输入")
        if not args.finalize_existing_stage.is_absolute():
            parser.error("--finalize-existing-stage 必须提供 exact absolute stage directory")


def _build_and_stage(args, native_lio, manifests, stage_parent):
    # type: (argparse.Namespace, Sequence[Tuple[str, Path]], Sequence[Path], Path) -> Path
    with tempfile.TemporaryDirectory(prefix="nuwax-geometry-release-target-") as isolated_target:
        build_env = dict(os.environ)
        build_env["CARGO_TARGET_DIR"] = isolated_target
        before = _capture_build_input_closure(args.cargo, manifests, env=build_env)
        geometry_manifest, cli_manifest = manifests
        _run((args.cargo, "build", "--release", "--locked", "--manifest-path",
              str(geometry_manifest)), env=build_env)
        _run((args.cargo, "build", "--release", "--locked", "--manifest-path",
              str(cli_manifest), "--bin", "nuwax-mapping-cli"), env=build_env)
        geometry_target = _cargo_target_directory(args.cargo, geometry_manifest, env=build_env)
        cli_target = _cargo_target_directory(args.cargo, cli_manifest, env=build_env)
        runtime = _validate_source(geometry_target / "release" / "nuwax-geometry-runtime",
                                   "geometry_runtime", executable=True)
        mapping_cli = _validate_source(cli_target / "release" / "nuwax-mapping-cli",
                                       "offline_mapping_cli", executable=True)
        stage = _create_stage(stage_parent, native_lio, runtime, mapping_cli, before)
        try:
            after = _capture_build_input_closure(args.cargo, manifests, env=build_env)
            if _canonical_json_bytes(after) != _canonical_json_bytes(before):
                raise RuntimeError("build-input closure 在构建期间发生变化，拒绝 seal")
            return stage
        except Exception as original:
            cleanup_failure = None  # type: Optional[Exception]
            try:
                _make_tree_removable(stage)
                shutil.rmtree(str(stage))
            except Exception as exc:
                cleanup_failure = exc
            if cleanup_failure is not None:
                raise RuntimeError("%s; stage cleanup failure: %s"
                                   % (original, cleanup_failure)) from original
            raise


def main(argv=None):
    # type: (Optional[Sequence[str]]) -> int
    parser = _build_parser()
    args = parser.parse_args(argv)
    _validate_mode_arguments(parser, args)
    geometry_manifest = REPO_ROOT / "services" / "geometry" / "nuwax_geometry_runtime" / "Cargo.toml"
    cli_manifest = REPO_ROOT / "services" / "nuwax-workspace" / "crates" / "nuwax-mapping-cli" / "Cargo.toml"
    manifests = (geometry_manifest, cli_manifest)
    if args.finalize_existing_stage is not None:
        return _finalize_existing_stage(args, manifests)

    # legacy 是 root 私有一体化发布；必须在任何真实 build 前完成 gate。
    if args.stage_only is None and os.geteuid() != 0:
        raise RuntimeError("legacy 一体化发布必须由 root owner 执行")
    profile_contract = _profile_native_lio_contract(_PROFILE_PATH)
    native_lio = _parse_native_lio(args.native_lio, profile_contract)
    if args.stage_only is not None:
        stage = _build_and_stage(args, native_lio, manifests, args.stage_only)
        print(str(stage))
        return 0

    default_output = (REPO_ROOT / "services" / "geometry" / "nuwax_geometry_runtime"
                      / "target" / "release" / "nuwax-geometry-releases")
    output_root = (args.output_root or default_output).absolute()
    stage_parent = output_root.parent
    stage_parent.mkdir(parents=True, exist_ok=True)
    stage = _build_and_stage(args, native_lio, manifests, stage_parent)
    args.finalize_existing_stage = stage
    try:
        result = _finalize_existing_stage(args, manifests, private_stage=True)
        _make_tree_removable(stage)
        shutil.rmtree(str(stage))
        return result
    except Exception:
        # 已 SEALED 的普通 stage 在 finalize 失败后保留，便于审计和安全重试。
        raise


if __name__ == "__main__":
    raise SystemExit(main())
