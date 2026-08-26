"""Geometry Release 制品清单的唯一解析、校验与受控执行入口。"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import stat
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

_SCHEMA_VERSION = "nuwax.geometry.release.v1"
_REQUIRED_SINGLETON_ROLES = {
    "geometry_runtime",
    "build_input_closure",
}
_OPTIONAL_STANDALONE_ROLES = {"offline_mapping_cli"}
_NATIVE_LIO_ROLE_PREFIX = "native_lio_payload:"
_SUPPORTED_LIO_BACKENDS = {"device", "fastlio2", "pointlio"}
_SUPPORTED_MAPPING_BACKENDS = {"voxel", "raytracing"}
_SUPPORTED_CORRECTION_BACKENDS = {
    "none",
    "library_pgo",
    "native_pgo",
    "relocalization",
}
_HEX_DIGEST_LENGTH = 64
_VERIFIED_PGO_ENV = "NUWAX_GEOMETRY_RELEASE_NATIVE_PGO_EXECUTABLE"
_VERIFIED_GUARDIAN_ENV = "NUWAX_GEOMETRY_RELEASE_GROUP_GUARDIAN_EXECUTABLE"
_NATIVE_LIO_ENV = "NUWAX_GEOMETRY_NATIVE_LIO_EXECUTABLE"
_LIO_BACKEND_ENV = "NUWAX_GEOMETRY_LIO_BACKEND"
_MAPPING_BACKEND_ENV = "NUWAX_GEOMETRY_MAPPING_BACKEND"
_CORRECTION_BACKEND_ENV = "NUWAX_GEOMETRY_CORRECTION_BACKEND"


def _selected_backend(environment: Mapping[str, str], name: str, default: str) -> str:
    value = environment.get(name, default)
    if not isinstance(value, str):
        raise ReleaseManifestError("Geometry backend 配置必须是字符串: %s" % name)
    value = value.strip()
    supported = {
        _LIO_BACKEND_ENV: _SUPPORTED_LIO_BACKENDS,
        _MAPPING_BACKEND_ENV: _SUPPORTED_MAPPING_BACKENDS,
        _CORRECTION_BACKEND_ENV: _SUPPORTED_CORRECTION_BACKENDS,
    }[name]
    if value not in supported:
        raise ReleaseManifestError("Geometry backend 不受支持: %s=%s" % (name, value))
    return value


def _selected_backends(environment: Optional[Mapping[str, str]] = None) -> Tuple[str, str, str]:
    values = environment if environment is not None else os.environ
    return (
        _selected_backend(values, _LIO_BACKEND_ENV, "device"),
        _selected_backend(values, _MAPPING_BACKEND_ENV, "voxel"),
        _selected_backend(values, _CORRECTION_BACKEND_ENV, "none"),
    )


def _selected_artifact_roles(environment: Optional[Mapping[str, str]] = None) -> Tuple[str, ...]:
    lio, _mapping, correction = _selected_backends(environment)
    roles = []
    if lio != "device":
        roles.append(_NATIVE_LIO_ROLE_PREFIX + lio)
    if correction == "native_pgo":
        roles.extend(("native_pgo_payload", "group_guardian"))
    return tuple(sorted(roles))


class ReleaseManifestError(RuntimeError):
    """Release 制品集合无法证明时的失败。"""


@dataclass(frozen=True)
class ReleaseArtifact:
    """一项已验证的不可变 Release 制品。"""

    role: str
    path: Path
    size: int
    sha256: str
    executable: bool
    fd: Optional[int] = None


@dataclass(frozen=True)
class ReleaseProof:
    """完整 Release generation 的验证结果。"""

    manifest_path: Path
    manifest_sha256: str
    generation: str
    available_native_lio_roles: Tuple[str, ...]
    selected_native_lio_roles: Tuple[str, ...]
    artifacts: Tuple[ReleaseArtifact, ...]

    @property
    def native_lio_roles(self) -> Tuple[str, ...]:
        """兼容旧调用方读取全量 native LIO role。"""

        return self.available_native_lio_roles

    def artifact_for_role(self, role: str) -> ReleaseArtifact:
        for artifact in self.artifacts:
            if artifact.role == role:
                return artifact
        raise ReleaseManifestError("release manifest 缺少 role=%s" % role)

    def close(self) -> None:
        """关闭为受控执行保留的 artifact FD。"""

        for artifact in self.artifacts:
            if artifact.fd is not None:
                try:
                    os.close(artifact.fd)
                except OSError:
                    pass


def _canonical_json_bytes(payload: object) -> bytes:
    return (
        json.dumps(
            payload,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        )
        + "\n"
    ).encode("utf-8")


def _sha256_fd(fd: int) -> str:
    digest = hashlib.sha256()
    os.lseek(fd, 0, os.SEEK_SET)
    while True:
        block = os.read(fd, 1024 * 1024)
        if not block:
            break
        digest.update(block)
    os.lseek(fd, 0, os.SEEK_SET)
    return digest.hexdigest()


def _sha256_file(path: Path) -> str:
    fd = _open_regular_fd(path)
    try:
        return _sha256_fd(fd)
    finally:
        os.close(fd)


def _is_hex_digest(value: object) -> bool:
    text = str(value or "")
    return len(text) == _HEX_DIGEST_LENGTH and all(
        character in "0123456789abcdef" for character in text
    )


def _artifact_set_generation(entries: Sequence[Mapping[str, object]]) -> str:
    return hashlib.sha256(_canonical_json_bytes(list(entries))).hexdigest()


def default_release_manifest_path() -> Path:
    """按显式环境、runtime 同级路径、repo-local 路径依次解析清单。"""

    configured = os.environ.get("NUWAX_GEOMETRY_RELEASE_MANIFEST", "").strip()
    if configured:
        return Path(configured)
    binary = os.environ.get("NUWAX_GEOMETRY_RUNTIME_BINARY", "").strip()
    if binary:
        return Path(binary).parent / "nuwax-geometry-release" / "manifest.json"
    return (
        Path(__file__).resolve().parents[2]
        / "services"
        / "geometry"
        / "nuwax_geometry_runtime"
        / "target"
        / "release"
        / "nuwax-geometry-release"
        / "manifest.json"
    )


def _verify_immutable_generation_path(path: Path, expected_uid: int, *, directory: bool) -> None:
    try:
        metadata = path.lstat()
    except OSError as exc:
        raise ReleaseManifestError("release generation 路径组件不可读取: %s" % path) from exc
    if metadata.st_uid != expected_uid:
        raise ReleaseManifestError("release generation owner UID 不一致: %s" % path)
    if stat.S_IMODE(metadata.st_mode) & 0o022:
        raise ReleaseManifestError("release generation 路径不得允许 group/other 写入: %s" % path)
    if directory and not stat.S_ISDIR(metadata.st_mode):
        raise ReleaseManifestError("release generation 路径组件不是目录: %s" % path)
    if not directory and not stat.S_ISREG(metadata.st_mode):
        raise ReleaseManifestError("release generation 成员不是 regular file: %s" % path)


def _verify_generation_ownership(root: Path, manifest: Path) -> int:
    owner_uid = manifest.lstat().st_uid
    _verify_immutable_generation_path(root, owner_uid, directory=True)
    _verify_immutable_generation_path(manifest, owner_uid, directory=False)
    for candidate in root.rglob("*"):
        _verify_immutable_generation_path(candidate, owner_uid, directory=candidate.is_dir())
    return owner_uid


def _resolved_regular_path(path: Path) -> Path:
    try:
        resolved = path.resolve(strict=True)
    except OSError as exc:
        raise ReleaseManifestError("release 文件不可解析: %s" % path) from exc
    current = Path(resolved.anchor)
    for part in resolved.parts[1:]:
        current = current / part
        try:
            metadata = current.lstat()
        except OSError as exc:
            raise ReleaseManifestError("release 路径组件不可读取: %s" % current) from exc
        if stat.S_ISLNK(metadata.st_mode):
            raise ReleaseManifestError("release canonical 路径不得包含 symlink: %s" % current)
    return resolved


def _open_regular_fd(path: Path) -> int:
    resolved = _resolved_regular_path(path)
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    try:
        fd = os.open(str(resolved), flags)
        metadata = os.fstat(fd)
    except OSError as exc:
        raise ReleaseManifestError("release regular file 无法安全打开: %s" % resolved) from exc
    if not stat.S_ISREG(metadata.st_mode):
        os.close(fd)
        raise ReleaseManifestError("release 文件必须是 regular file: %s" % resolved)
    return fd


def _open_relative_regular_fd(root_fd: int, relative: Path) -> int:
    directory_fd = os.dup(root_fd)
    try:
        for part in relative.parts[:-1]:
            next_fd = os.open(
                part,
                os.O_RDONLY
                | getattr(os, "O_CLOEXEC", 0)
                | getattr(os, "O_NOFOLLOW", 0)
                | getattr(os, "O_DIRECTORY", 0),
                dir_fd=directory_fd,
            )
            os.close(directory_fd)
            directory_fd = next_fd
        fd = os.open(
            relative.parts[-1],
            os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0),
            dir_fd=directory_fd,
        )
    except OSError as exc:
        raise ReleaseManifestError("release artifact 无法通过固定目录 FD 打开: %s" % relative) from exc
    finally:
        os.close(directory_fd)
    if not stat.S_ISREG(os.fstat(fd).st_mode):
        os.close(fd)
        raise ReleaseManifestError("release artifact 必须是 regular file: %s" % relative)
    return fd


def _read_all_fd(fd: int) -> bytes:
    chunks = []
    os.lseek(fd, 0, os.SEEK_SET)
    while True:
        block = os.read(fd, 1024 * 1024)
        if not block:
            break
        chunks.append(block)
    os.lseek(fd, 0, os.SEEK_SET)
    return b"".join(chunks)


def _load_canonical_manifest(path: Path) -> Tuple[Path, bytes, Dict[str, object]]:
    resolved = _resolved_regular_path(path)
    fd = _open_regular_fd(resolved)
    try:
        encoded = _read_all_fd(fd)
        payload = json.loads(encoded.decode("utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ReleaseManifestError("release manifest 不是有效 UTF-8 JSON: %s" % path) from exc
    finally:
        os.close(fd)
    if not isinstance(payload, dict):
        raise ReleaseManifestError("release manifest 顶层必须是 object")
    if encoded != _canonical_json_bytes(payload):
        raise ReleaseManifestError("release manifest 不是 canonical JSON")
    return resolved, encoded, payload


def verify_release_manifest(
    manifest_path: Optional[Path] = None,
    *,
    expected_runtime: Optional[Path] = None,
    retain_artifact_fds: bool = False,
    require_selected_artifacts: bool = False,
) -> ReleaseProof:
    """验证 release artifact set；可选地执行当前 Geometry profile 准入。"""

    resolved_manifest, manifest_bytes, payload = _load_canonical_manifest(
        Path(manifest_path) if manifest_path is not None else default_release_manifest_path()
    )
    if set(payload) != {
        "schema_version",
        "generation",
        "available_native_lio_roles",
        "selected_native_lio_roles",
        "artifacts",
    }:
        raise ReleaseManifestError("release manifest 顶层字段集合不精确")
    if payload.get("schema_version") != _SCHEMA_VERSION:
        raise ReleaseManifestError("release manifest schema_version 不受支持")
    generation = str(payload.get("generation") or "")
    if not _is_hex_digest(generation):
        raise ReleaseManifestError("release generation 必须是 64 位小写十六进制摘要")
    configured_artifact_roles = set(_selected_artifact_roles())
    selected_artifact_roles = configured_artifact_roles if require_selected_artifacts else set()
    raw_available_lio_roles = payload.get("available_native_lio_roles")
    raw_selected_lio_roles = payload.get("selected_native_lio_roles")
    if (
        not isinstance(raw_available_lio_roles, list)
        or not raw_available_lio_roles
        or any(
            not isinstance(role, str)
            or role not in _SUPPORTED_LIO_BACKENDS - {"device"}
            for role in raw_available_lio_roles
        )
        or raw_available_lio_roles != sorted(set(raw_available_lio_roles))
        or not isinstance(raw_selected_lio_roles, list)
        or any(
            not isinstance(role, str)
            or role not in _SUPPORTED_LIO_BACKENDS - {"device"}
            for role in raw_selected_lio_roles
        )
        or raw_selected_lio_roles != sorted(set(raw_selected_lio_roles))
        or not set(raw_selected_lio_roles).issubset(set(raw_available_lio_roles))
    ):
        raise ReleaseManifestError(
            "release manifest native LIO available/selected roles 合同无效"
        )
    expected_lio_roles = tuple(
        _NATIVE_LIO_ROLE_PREFIX + role for role in raw_available_lio_roles
    )
    raw_artifacts = payload.get("artifacts")
    if not isinstance(raw_artifacts, list) or not raw_artifacts:
        raise ReleaseManifestError("release manifest artifacts 必须是非空数组")

    root = resolved_manifest.parent
    _verify_generation_ownership(root, resolved_manifest)
    root_fd = os.open(
        str(root),
        os.O_RDONLY
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
        | getattr(os, "O_DIRECTORY", 0),
    )
    seen_roles = set()
    seen_paths = set()
    normalized_entries: List[Dict[str, object]] = []
    verified: List[ReleaseArtifact] = []
    try:
        for raw in raw_artifacts:
            if not isinstance(raw, dict) or set(raw) != {
                "role", "path", "size", "sha256", "executable"
            }:
                raise ReleaseManifestError("release artifact 字段集合不精确")
            role = str(raw.get("role") or "").strip()
            relative_text = str(raw.get("path") or "")
            digest = str(raw.get("sha256") or "")
            size = raw.get("size")
            executable = raw.get("executable")
            if not role or role in seen_roles:
                raise ReleaseManifestError("release artifact role 为空或重复: %s" % role)
            relative = Path(relative_text)
            if (
                not relative_text
                or relative.is_absolute()
                or relative_text != relative.as_posix()
                or any(part in {"", ".", ".."} for part in relative.parts)
            ):
                raise ReleaseManifestError("release artifact path 非规范相对路径: %s" % relative_text)
            if relative_text in seen_paths:
                raise ReleaseManifestError("release artifact path 重复: %s" % relative_text)
            if isinstance(size, bool) or not isinstance(size, int) or size < 0:
                raise ReleaseManifestError("release artifact size 非法: %s" % role)
            if not _is_hex_digest(digest):
                raise ReleaseManifestError("release artifact sha256 非法: %s" % role)
            if not isinstance(executable, bool):
                raise ReleaseManifestError("release artifact executable 非法: %s" % role)

            fd = _open_relative_regular_fd(root_fd, relative)
            metadata = os.fstat(fd)
            actual_executable = bool(metadata.st_mode & 0o111)
            if metadata.st_size != size or actual_executable != executable:
                os.close(fd)
                raise ReleaseManifestError("release artifact 属性与 manifest 不一致: %s" % role)
            if _sha256_fd(fd) != digest:
                os.close(fd)
                raise ReleaseManifestError("release artifact SHA-256 不一致: %s" % role)
            kept_fd = fd if retain_artifact_fds else None
            if not retain_artifact_fds:
                os.close(fd)
            seen_roles.add(role)
            seen_paths.add(relative_text)
            normalized_entries.append(dict(raw))
            verified.append(
                ReleaseArtifact(
                    role=role,
                    path=root / relative,
                    size=size,
                    sha256=digest,
                    executable=executable,
                    fd=kept_fd,
                )
            )

        missing_roles = _REQUIRED_SINGLETON_ROLES - seen_roles
        if missing_roles:
            raise ReleaseManifestError(
                "release manifest 缺少固定角色: %s" % ",".join(sorted(missing_roles))
            )
        allowed_roles = (
            _REQUIRED_SINGLETON_ROLES
            | _OPTIONAL_STANDALONE_ROLES
            | {"native_pgo_payload", "group_guardian"}
            | {
                role
                for role in seen_roles
                if role.startswith(_NATIVE_LIO_ROLE_PREFIX)
            }
        )
        unexpected_roles = seen_roles - allowed_roles
        if unexpected_roles:
            raise ReleaseManifestError(
                "release manifest 含未声明 artifact role: %s"
                % ",".join(sorted(unexpected_roles))
            )
        actual_lio_roles = tuple(
            sorted(role for role in seen_roles if role.startswith(_NATIVE_LIO_ROLE_PREFIX))
        )
        if actual_lio_roles != tuple(sorted(expected_lio_roles)):
            raise ReleaseManifestError("release manifest native LIO available artifact 集合不精确")
        conditional_roles = seen_roles & {"native_pgo_payload", "group_guardian"}
        if conditional_roles not in (set(), {"native_pgo_payload", "group_guardian"}):
            raise ReleaseManifestError(
                "release manifest native PGO 与 group guardian 必须成对出现"
            )
        if require_selected_artifacts:
            missing_selected = set(selected_artifact_roles) - seen_roles
            if missing_selected:
                raise ReleaseManifestError(
                    "release manifest 缺少当前选中制品: %s"
                    % ",".join(sorted(missing_selected))
                )
            selected_conditional_roles = {
                role for role in seen_roles
                if role.startswith(_NATIVE_LIO_ROLE_PREFIX)
                or role in {"native_pgo_payload", "group_guardian"}
            }
            if not selected_artifact_roles.issubset(selected_conditional_roles):
                raise ReleaseManifestError(
                    "release manifest 缺少当前选中的条件制品: %s"
                    % ",".join(sorted(selected_artifact_roles - selected_conditional_roles))
                )
        if generation != _artifact_set_generation(normalized_entries):
            raise ReleaseManifestError("release generation 与 artifact set 不一致")

        expected_files = set(seen_paths)
        expected_files.add(resolved_manifest.name)
        actual_files = set()
        for candidate in root.rglob("*"):
            relative_text = candidate.relative_to(root).as_posix()
            metadata = candidate.lstat()
            if stat.S_ISLNK(metadata.st_mode):
                raise ReleaseManifestError("release generation 含额外 symlink: %s" % relative_text)
            if stat.S_ISREG(metadata.st_mode):
                actual_files.add(relative_text)
            elif not stat.S_ISDIR(metadata.st_mode):
                raise ReleaseManifestError("release generation 含非 regular artifact: %s" % relative_text)
        if actual_files != expected_files:
            raise ReleaseManifestError("release generation 文件集合与 manifest 不一致")

        proof = ReleaseProof(
            manifest_path=resolved_manifest,
            manifest_sha256=hashlib.sha256(manifest_bytes).hexdigest(),
            generation=generation,
            available_native_lio_roles=tuple(raw_available_lio_roles),
            selected_native_lio_roles=tuple(raw_selected_lio_roles),
            artifacts=tuple(verified),
        )
        if expected_runtime is not None:
            actual_runtime = proof.artifact_for_role("geometry_runtime").path
            required_runtime = _resolved_regular_path(Path(expected_runtime))
            if actual_runtime != required_runtime:
                raise ReleaseManifestError("启动 runtime 不属于已验证 release generation")
        return proof
    except Exception:
        for artifact in verified:
            if artifact.fd is not None:
                os.close(artifact.fd)
        raise
    finally:
        os.close(root_fd)


def _read_boot_id() -> str:
    try:
        return Path("/proc/sys/kernel/random/boot_id").read_text(encoding="ascii").strip().lower()
    except OSError as exc:
        raise ReleaseManifestError("无法读取当前 boot ID") from exc


def _read_proc_starttime(pid: int) -> str:
    try:
        fields = Path("/proc/%d/stat" % pid).read_text(encoding="ascii").split()
    except OSError as exc:
        raise ReleaseManifestError("ready PID 不存在") from exc
    if len(fields) < 22 or not fields[21].isdigit():
        raise ReleaseManifestError("ready PID starttime 无效")
    return fields[21]


def verify_ready_release(ready_path: Path, proof: ReleaseProof) -> None:
    """验证 ready lease 的进程实例、可执行文件和 Release 身份。"""

    values: Dict[str, str] = {}
    try:
        metadata = ready_path.lstat()
        if stat.S_ISLNK(metadata.st_mode) or not stat.S_ISREG(metadata.st_mode):
            raise ReleaseManifestError("Geometry ready 必须是非 symlink regular file")
        for line in ready_path.read_text(encoding="utf-8").splitlines():
            key, separator, value = line.partition("=")
            if not separator or not key or key in values:
                raise ReleaseManifestError("Geometry ready 字段无效或重复")
            values[key] = value
    except OSError as exc:
        raise ReleaseManifestError("Geometry ready 不可读取") from exc
    required = {
        "pid", "boot_id", "starttime", "port", "health_ready",
        "runtime_binary_sha256", "release_manifest_sha256", "release_generation",
    }
    if set(values) != required:
        raise ReleaseManifestError("Geometry ready 字段集合不精确")
    try:
        pid = int(values["pid"])
        port = int(values["port"])
    except ValueError as exc:
        raise ReleaseManifestError("Geometry ready PID/port 无效") from exc
    if pid <= 0 or not 1 <= port <= 65535 or values["health_ready"] != "1":
        raise ReleaseManifestError("Geometry ready 进程或健康字段无效")
    if values["boot_id"].lower() != _read_boot_id():
        raise ReleaseManifestError("Geometry ready boot ID 已陈旧")
    if values["starttime"] != _read_proc_starttime(pid):
        raise ReleaseManifestError("Geometry ready PID incarnation 已陈旧")
    runtime = proof.artifact_for_role("geometry_runtime")
    try:
        loaded_executable = Path("/proc/%d/exe" % pid).resolve(strict=True)
    except OSError as exc:
        raise ReleaseManifestError("无法解析 ready PID executable") from exc
    if loaded_executable != runtime.path:
        raise ReleaseManifestError("ready PID executable 不属于已验证 generation")
    if values["runtime_binary_sha256"] != runtime.sha256:
        raise ReleaseManifestError("Geometry ready runtime binary digest 不一致")
    if values["release_manifest_sha256"] != proof.manifest_sha256:
        raise ReleaseManifestError("Geometry ready manifest digest 与本地 release 不一致")
    if values["release_generation"] != proof.generation:
        raise ReleaseManifestError("Geometry ready generation 与本地 release 不一致")


def verify_exec_proof(proof_path: Path, pid: int, proof: ReleaseProof) -> None:
    """验证 launcher 收到的 exec 身份证明属于当前进程和 generation。"""

    values: Dict[str, str] = {}
    try:
        metadata = proof_path.lstat()
        if stat.S_ISLNK(metadata.st_mode) or not stat.S_ISREG(metadata.st_mode):
            raise ReleaseManifestError("exec proof 必须是非 symlink regular file")
        for line in proof_path.read_text(encoding="ascii").splitlines():
            key, separator, value = line.partition("=")
            if not separator or not key or key in values:
                raise ReleaseManifestError("exec proof 字段无效或重复")
            values[key] = value
    except OSError as exc:
        raise ReleaseManifestError("exec proof 不可读取") from exc
    required = {
        "RUNTIME_BINARY_SHA256",
        "RELEASE_MANIFEST_SHA256",
        "RELEASE_GENERATION",
    }
    if set(values) != required:
        raise ReleaseManifestError("exec proof 字段集合不精确")
    runtime = proof.artifact_for_role("geometry_runtime")
    if values["RUNTIME_BINARY_SHA256"] != runtime.sha256:
        raise ReleaseManifestError("exec proof runtime digest 不一致")
    if values["RELEASE_MANIFEST_SHA256"] != proof.manifest_sha256:
        raise ReleaseManifestError("exec proof manifest digest 不一致")
    if values["RELEASE_GENERATION"] != proof.generation:
        raise ReleaseManifestError("exec proof generation 不一致")
    if pid <= 0:
        raise ReleaseManifestError("exec proof PID 无效")
    try:
        loaded_executable = Path("/proc/%d/exe" % pid).resolve(strict=True)
    except OSError as exc:
        raise ReleaseManifestError("无法解析 exec proof PID executable") from exc
    if loaded_executable != runtime.path:
        raise ReleaseManifestError("exec proof PID executable 不属于已验证 generation")


def _write_exec_proof(path: Path, proof: ReleaseProof) -> None:
    runtime = proof.artifact_for_role("geometry_runtime")
    encoded = (
        "RUNTIME_BINARY_SHA256=%s\nRELEASE_MANIFEST_SHA256=%s\nRELEASE_GENERATION=%s\n"
        % (runtime.sha256, proof.manifest_sha256, proof.generation)
    ).encode("ascii")
    temporary = path.with_name(".%s.tmp.%d" % (path.name, os.getpid()))
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0)
    fd = os.open(str(temporary), flags, 0o600)
    try:
        os.write(fd, encoded)
        os.fsync(fd)
    finally:
        os.close(fd)
    os.replace(str(temporary), str(path))


def exec_verified_release(manifest_path: Optional[Path], proof_output: Path) -> None:
    """以同一组已验证 FD 执行 runtime，并注入 generation 内 native payload。"""

    proof = verify_release_manifest(
        manifest_path,
        retain_artifact_fds=True,
        require_selected_artifacts=True,
    )
    try:
        by_role = {artifact.role: artifact for artifact in proof.artifacts}
        runtime = by_role["geometry_runtime"]
        if runtime.fd is None:
            raise ReleaseManifestError("受控执行未保留 geometry runtime artifact FD")
        environment = dict(os.environ)
        selected_roles = set(_selected_artifact_roles(environment))
        inheritable_roles = {"geometry_runtime"} | selected_roles
        for artifact in proof.artifacts:
            if artifact.fd is not None:
                os.set_inheritable(artifact.fd, artifact.role in inheritable_roles)
        if "native_pgo_payload" in selected_roles:
            pgo = by_role.get("native_pgo_payload")
            guardian = by_role.get("group_guardian")
            if pgo is None or guardian is None or pgo.fd is None or guardian.fd is None:
                raise ReleaseManifestError("受控执行未保留选中的 native PGO artifact FD")
            environment[_VERIFIED_PGO_ENV] = "/proc/self/fd/%d" % pgo.fd
            environment[_VERIFIED_GUARDIAN_ENV] = "/proc/self/fd/%d" % guardian.fd
        else:
            environment.pop(_VERIFIED_PGO_ENV, None)
            environment.pop(_VERIFIED_GUARDIAN_ENV, None)
        backend = environment.get(_LIO_BACKEND_ENV, "device").strip()
        if backend in proof.native_lio_roles:
            lio = by_role[_NATIVE_LIO_ROLE_PREFIX + backend]
            if lio.fd is None:
                raise ReleaseManifestError("受控执行未保留 native LIO artifact FD")
            environment[_NATIVE_LIO_ENV] = "/proc/self/fd/%d" % lio.fd
        elif backend != "device":
            raise ReleaseManifestError("运行时 LIO backend 不属于已验证 profile 集合: %s" % backend)
        else:
            environment.pop(_NATIVE_LIO_ENV, None)
        _write_exec_proof(proof_output, proof)
        executable = "/proc/self/fd/%d" % runtime.fd
        os.execve(executable, [str(runtime.path)], environment)
    except Exception:
        proof.close()
        raise


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="校验 Geometry immutable release artifact set")
    parser.add_argument("action", choices=("check", "proof", "runtime", "exec", "exec-proof"))
    parser.add_argument("--manifest", type=Path, default=None)
    parser.add_argument("--runtime", type=Path, default=None)
    parser.add_argument("--ready", type=Path, default=None)
    parser.add_argument("--proof-output", type=Path, default=None)
    parser.add_argument("--pid", type=int, default=None)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _build_parser().parse_args(argv)
    try:
        if args.action == "exec":
            if args.proof_output is None:
                raise ReleaseManifestError("exec 必须提供 --proof-output")
            exec_verified_release(args.manifest, args.proof_output)
            raise ReleaseManifestError("受控 runtime exec 意外返回")
        proof = verify_release_manifest(
            args.manifest,
            expected_runtime=args.runtime,
            require_selected_artifacts=True,
        )
        if args.action == "exec-proof":
            if args.proof_output is None or args.pid is None:
                raise ReleaseManifestError("exec-proof 必须提供 --proof-output 和 --pid")
            verify_exec_proof(args.proof_output, args.pid, proof)
        if args.ready is not None:
            verify_ready_release(args.ready, proof)
    except ReleaseManifestError as exc:
        print("Geometry release proof failed: %s" % exc, file=sys.stderr)
        return 1
    if args.action == "proof":
        print("%s %s" % (proof.manifest_sha256, proof.generation))
    elif args.action == "runtime":
        print(str(proof.artifact_for_role("geometry_runtime").path))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
