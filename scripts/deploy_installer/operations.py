from __future__ import annotations

import grp
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
import urllib.error
import urllib.request
import zipfile
from pathlib import Path, PurePosixPath
from typing import Dict, FrozenSet, Mapping, Optional, Sequence, Tuple, TYPE_CHECKING

from .config import atomic_write, fsync_directory, render_env
from .profile import profile_sha256
from .runner import resolve_build_account, source_user_command, source_user_environment

if TYPE_CHECKING:
    from .engine import ExecutionContext


class DeploymentError(RuntimeError):
    pass


_REQUIRED_BUNDLE_DIRS = {
    "wheelhouse",
    "cargo-vendor",
    "apt-repo",
    "vendor",
    "rustup-home",
    "cargo-bin",
    "source",
    "licenses",
}


def _os_release() -> Tuple[str, str]:
    values: Dict[str, str] = {}
    try:
        for line in Path("/etc/os-release").read_text(encoding="utf-8").splitlines():
            if "=" not in line or line.startswith("#"):
                continue
            key, value = line.split("=", 1)
            values[key] = value.strip().strip('"')
    except OSError:
        return "unknown", "unknown"
    return values.get("ID", "unknown"), values.get("VERSION_ID", "unknown")


def _require_file(path: Optional[Path], label: str) -> Path:
    if path is None or path.is_symlink() or not path.is_file():
        raise DeploymentError(f"{label} is required and must be a non-symlink regular file: {path}")
    return path.resolve()


def _require_dir(path: Optional[Path], label: str) -> Path:
    if path is None or path.is_symlink() or not path.is_dir():
        raise DeploymentError(f"{label} is required and must be a non-symlink directory: {path}")
    return path.resolve()


def _first_wheel(bundle: Path, distribution: str) -> Optional[Path]:
    normalized = distribution.replace("-", "_")
    matches = sorted(
        {
            *bundle.joinpath("wheelhouse").glob(f"{normalized}-*.whl"),
            *bundle.joinpath("wheelhouse").glob(f"{distribution}-*.whl"),
        }
    )
    if len(matches) > 1:
        raise DeploymentError(f"offline bundle contains multiple candidate wheels for {distribution}")
    return matches[0].resolve() if matches else None


def _resolve_go2_artifacts(context: "ExecutionContext") -> None:
    options = context.options
    bundle = options.offline_bundle
    sdk = options.go2_sdk
    sdk_wheel = options.go2_sdk_wheel
    clip_source = options.clip_source
    clip_wheel = options.clip_wheel
    cyclone_source = options.cyclonedds_source
    cyclone_wheel = options.cyclonedds_wheel
    if bundle is not None:
        sdk_wheel = sdk_wheel or _first_wheel(bundle, "unitree_sdk2py")
        clip_wheel = clip_wheel or _first_wheel(bundle, "clip")
        cyclone_wheel = cyclone_wheel or _first_wheel(bundle, "cyclonedds")
    if sdk_wheel is not None:
        context.notes["go2_sdk_artifact"] = _require_file(sdk_wheel, "Unitree SDK wheel")
    elif sdk is not None and bundle is None and options.allow_unverified_third_party:
        context.notes["go2_sdk_artifact"] = _require_dir(sdk, "Go2 SDK checkout")
    else:
        raise DeploymentError(
            "Go2 deployment requires an approved Unitree SDK wheel; "
            "a source checkout is only allowed for explicit online controlled bring-up"
        )
    if clip_wheel is not None:
        context.notes["clip_artifact"] = _require_file(clip_wheel, "Ultralytics CLIP wheel")
    elif clip_source is not None and bundle is None and options.allow_unverified_third_party:
        context.notes["clip_artifact"] = _require_dir(clip_source, "Ultralytics CLIP source")
    else:
        raise DeploymentError(
            "Go2 deployment requires an approved CLIP wheel; source is only allowed "
            "for explicit online controlled bring-up"
        )
    if cyclone_wheel is not None:
        context.notes["cyclonedds_artifact"] = _require_file(cyclone_wheel, "CycloneDDS wheel")
    elif cyclone_source is not None and bundle is None and options.allow_unverified_third_party:
        context.notes["cyclonedds_artifact"] = _require_dir(cyclone_source, "CycloneDDS source")
    else:
        raise DeploymentError(
            "Go2 deployment requires an approved CycloneDDS wheel; source is only allowed "
            "for explicit online controlled bring-up"
        )


def _extract_unitree_wheel(wheel: Path, destination: Path) -> None:
    wheel = _require_file(wheel, "Unitree SDK wheel")
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.is_symlink() or (destination.exists() and not destination.is_dir()):
        raise DeploymentError(f"Unitree SDK vendor destination is unsafe: {destination}")
    staging = Path(tempfile.mkdtemp(prefix=f".{destination.name}.staging-", dir=destination.parent))
    extracted = set()
    total_size = 0
    try:
        with zipfile.ZipFile(wheel) as archive:
            members = archive.infolist()
            if len(members) > 100_000:
                raise DeploymentError("Unitree SDK wheel contains too many entries")
            for member in members:
                raw_name = member.filename
                if not raw_name or "\x00" in raw_name or "\\" in raw_name or raw_name.startswith("/"):
                    raise DeploymentError(f"Unitree SDK wheel path is unsafe: {raw_name!r}")
                relative = PurePosixPath(raw_name)
                if relative.is_absolute() or ".." in relative.parts or relative.as_posix() != raw_name.rstrip("/"):
                    raise DeploymentError(f"Unitree SDK wheel path is unsafe: {raw_name!r}")
                mode = (member.external_attr >> 16) & 0xFFFF
                file_type = stat.S_IFMT(mode)
                if file_type not in {0, stat.S_IFREG, stat.S_IFDIR}:
                    raise DeploymentError(f"Unitree SDK wheel contains a symlink or special file: {raw_name}")
                parts = relative.parts
                if parts and parts[0].endswith(".data"):
                    if len(parts) < 3 or parts[1] not in {"purelib", "platlib"}:
                        if member.is_dir():
                            continue
                        raise DeploymentError(f"Unitree SDK wheel uses an unsupported data scheme: {raw_name}")
                    parts = parts[2:]
                if not parts:
                    continue
                top_level = parts[0]
                if not (
                    top_level == "unitree_sdk2py"
                    or top_level == "unitree_sdk2py.libs"
                    or (top_level.startswith("unitree_sdk2py-") and top_level.endswith(".dist-info"))
                ):
                    raise DeploymentError(f"Unitree SDK wheel contains an unexpected top-level path: {raw_name}")
                normalized = PurePosixPath(*parts)
                normalized_name = normalized.as_posix()
                if normalized_name in extracted:
                    raise DeploymentError(f"Unitree SDK wheel contains a duplicate runtime path: {normalized_name}")
                extracted.add(normalized_name)
                target = staging.joinpath(*parts)
                if member.is_dir():
                    target.mkdir(parents=True, exist_ok=True)
                    os.chmod(target, 0o755)
                    continue
                if member.flag_bits & 0x1:
                    raise DeploymentError("encrypted Unitree SDK wheel members are unsupported")
                if member.file_size > 512 * 1024 * 1024:
                    raise DeploymentError(f"Unitree SDK wheel member exceeds 512 MiB: {raw_name}")
                if (
                    member.file_size > 1024 * 1024
                    and member.compress_size > 0
                    and member.file_size > member.compress_size * 1000
                ):
                    raise DeploymentError(f"Unitree SDK wheel member has an unsafe compression ratio: {raw_name}")
                total_size += member.file_size
                if total_size > 2 * 1024 * 1024 * 1024:
                    raise DeploymentError("Unitree SDK wheel expands beyond the 2 GiB limit")
                target.parent.mkdir(parents=True, exist_ok=True)
                with archive.open(member) as source, target.open("xb") as output:
                    shutil.copyfileobj(source, output)
                os.chmod(target, 0o755 if mode & 0o111 else 0o644)
        if not (staging / "unitree_sdk2py").is_dir():
            raise DeploymentError("Unitree SDK wheel has no unitree_sdk2py package")
        channel_module = staging / "unitree_sdk2py" / "core" / "channel.py"
        compiled_channels = tuple((staging / "unitree_sdk2py" / "core").glob("channel*.so"))
        if not channel_module.is_file() and not compiled_channels:
            raise DeploymentError("Unitree SDK wheel has no unitree_sdk2py.core.channel runtime module")
        if destination.is_dir():
            shutil.rmtree(destination)
        os.replace(staging, destination)
    except (OSError, RuntimeError, zipfile.BadZipFile) as exc:
        raise DeploymentError(f"cannot safely unpack Unitree SDK wheel: {exc}") from exc
    finally:
        if staging.exists():
            shutil.rmtree(staging, ignore_errors=True)


def _tree_digest(path: Path) -> str:
    digest = hashlib.sha256()
    for candidate in sorted(path.rglob("*")):
        relative = candidate.relative_to(path)
        if ".git" in relative.parts:
            continue
        if (
            "__pycache__" in relative.parts
            or candidate.suffix == ".pyc"
            or any(part.endswith(".egg-info") for part in relative.parts)
        ):
            raise DeploymentError(f"source artifact contains generated build/import state: {candidate}")
        if candidate.is_symlink():
            raise DeploymentError(f"source artifact must not contain symlinks: {candidate}")
        if not candidate.is_file():
            continue
        digest.update(relative.as_posix().encode("utf-8"))
        digest.update(b"\0")
        digest.update(_sha256(candidate).encode("ascii"))
        digest.update(b"\0")
    return digest.hexdigest()


def _git_source_identity(path: Path) -> str:
    git_env = source_user_environment()
    head = subprocess.run(
        source_user_command(("git", "rev-parse", "HEAD")),
        cwd=str(path),
        env=git_env,
        capture_output=True,
        check=False,
        timeout=30,
    )
    index = subprocess.run(
        source_user_command(("git", "ls-files", "--stage", "-z")),
        cwd=str(path),
        env=git_env,
        capture_output=True,
        check=False,
        timeout=60,
    )
    dirty = subprocess.run(
        source_user_command(("git", "status", "--porcelain", "--untracked-files=normal")),
        cwd=str(path),
        env=git_env,
        capture_output=True,
        check=False,
        timeout=30,
    )
    if head.returncode != 0 or index.returncode != 0 or dirty.returncode != 0 or dirty.stdout.strip():
        raise DeploymentError("production source identity is dirty or unreadable")
    return hashlib.sha256(head.stdout + b"\0" + index.stdout).hexdigest()


def _verify_source_artifacts(context: "ExecutionContext") -> None:
    relative = context.profile.python.source_artifact_manifest
    if not relative:
        return
    manifest_path = context.options.source_root / relative
    try:
        document = json.loads(
            manifest_path.read_text(encoding="utf-8"),
            object_pairs_hook=_reject_duplicate_keys,
        )
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        raise DeploymentError(f"invalid Python source artifact manifest: {exc}") from exc
    if not isinstance(document, dict):
        raise DeploymentError("Python source artifact manifest root must be an object")
    raw_artifacts = document.get("artifacts")
    if document.get("schema_version") != 1 or not isinstance(raw_artifacts, list):
        raise DeploymentError("Python source artifact manifest schema is invalid")
    artifacts = {}
    for entry in raw_artifacts:
        if not isinstance(entry, dict) or not isinstance(entry.get("distribution"), str):
            raise DeploymentError("Python source artifact entry is invalid")
        distribution = entry["distribution"].lower().replace("_", "-")
        if distribution in artifacts:
            raise DeploymentError(f"duplicate Python source artifact: {distribution}")
        artifacts[distribution] = entry

    resolved = {
        "unitree-sdk2py": context.notes.get("go2_sdk_artifact"),
        "clip": context.notes.get("clip_artifact"),
        "cyclonedds": context.notes.get("cyclonedds_artifact"),
    }
    strict = context.profile.production and not context.options.allow_unverified_third_party
    for distribution, raw_path in resolved.items():
        if raw_path is None:
            continue
        entry = artifacts.get(distribution)
        if entry is None:
            raise DeploymentError(f"approved artifact manifest has no entry for {distribution}")
        if isinstance(entry.get("version"), str):
            context.notes[f"artifact_version:{distribution}"] = entry["version"]
        if strict and entry.get("status") != "ready":
            raise DeploymentError(f"production artifact is not approved: {distribution}")
        path = Path(raw_path)
        integrity = entry.get("integrity")
        if not isinstance(integrity, dict):
            raise DeploymentError(f"artifact integrity is missing: {distribution}")
        if path.is_file():
            expected = integrity.get("wheel_sha256")
            if strict and not isinstance(expected, str):
                raise DeploymentError(f"approved wheel hash is missing: {distribution}")
            actual = _sha256(path)
            if isinstance(expected, str) and actual != expected:
                raise DeploymentError(f"approved wheel hash mismatch: {distribution}")
            context.notes[f"artifact_sha256:{distribution}"] = actual
            continue
        source = _require_dir(path, f"{distribution} source")
        source_config = entry.get("source")
        if not isinstance(source_config, dict):
            raise DeploymentError(f"artifact source metadata is missing: {distribution}")
        expected_commit = source_config.get("commit")
        git_dir = source / ".git"
        if strict and not git_dir.exists():
            raise DeploymentError(f"approved source is not a Git checkout: {distribution}")
        if git_dir.exists():
            head = subprocess.run(
                source_user_command(("git", "rev-parse", "HEAD")),
                cwd=str(source),
                env=source_user_environment(),
                capture_output=True,
                text=True,
                check=False,
                timeout=30,
            )
            dirty = subprocess.run(
                source_user_command(("git", "status", "--porcelain", "--untracked-files=normal")),
                cwd=str(source),
                env=source_user_environment(),
                capture_output=True,
                text=True,
                check=False,
                timeout=30,
            )
            if head.returncode != 0 or dirty.returncode != 0 or dirty.stdout.strip():
                raise DeploymentError(f"source artifact checkout is dirty or invalid: {distribution}")
            if isinstance(expected_commit, str) and head.stdout.strip() != expected_commit:
                raise DeploymentError(f"approved source commit mismatch: {distribution}")
            if strict and not isinstance(expected_commit, str):
                raise DeploymentError(f"approved source commit is missing: {distribution}")
        actual = _tree_digest(source)
        expected = integrity.get("source_sha256")
        if strict and not isinstance(expected, str):
            raise DeploymentError(f"approved source hash is missing: {distribution}")
        if isinstance(expected, str) and actual != expected:
            raise DeploymentError(f"approved source hash mismatch: {distribution}")
        context.notes[f"artifact_sha256:{distribution}"] = actual


def _verify_python_runtime(context: "ExecutionContext") -> None:
    expected = context.profile.python.version
    if not expected:
        return
    require_match = not context.options.allow_unsupported_host
    code = """
import sys

expected = sys.argv[1]
actual = "%d.%d" % sys.version_info[:2]
if sys.argv[2] == "1" and actual != expected:
    raise SystemExit("deployment Python version mismatch: expected=%s actual=%s" % (expected, actual))
"""
    try:
        context.runner.run(
            (context.options.python_bin, "-I", "-c", code, expected, "1" if require_match else "0"),
            cwd=Path("/"),
            env={"HOME": str(context.options.build_home), "PYTHONNOUSERSITE": "1"},
            as_build_user=True,
            timeout=10,
        )
    except Exception as exc:
        raise DeploymentError(f"cannot execute the approved deployment Python {context.options.python_bin}") from exc


def _installed_package_version(package: str) -> str:
    completed = subprocess.run(
        ("dpkg-query", "-W", "-f=${db:Status-Abbrev}\t${Version}", package),
        capture_output=True,
        text=True,
        check=False,
        timeout=30,
    )
    status, _, version = completed.stdout.partition("\t")
    return version.strip() if completed.returncode == 0 and status.startswith("ii ") else ""


def _validate_jetson_compatibility(context: "ExecutionContext") -> None:
    profile = context.profile
    if not profile.production or profile.robot != "go2":
        return
    config = profile.metadata.get("jetson_compatibility")
    required = (
        "l4t_package",
        "l4t_version_prefix",
        "jetpack_package",
        "jetpack_version_prefix",
        "cuda_package",
        "cuda_version_prefix",
        "tensorrt_package",
        "tensorrt_version_prefix",
    )
    if not isinstance(config, dict) or not all(isinstance(config.get(key), str) and config[key] for key in required):
        if context.options.allow_unverified_third_party:
            context.notes["jetson_compatibility_unverified"] = True
            return
        raise DeploymentError("production profile has no approved JetPack/L4T/CUDA/TensorRT compatibility matrix")
    facts = {}
    for component in ("l4t", "jetpack", "cuda", "tensorrt"):
        package = config[f"{component}_package"]
        version = _installed_package_version(package)
        expected = config[f"{component}_version_prefix"]
        if not version or not version.startswith(expected):
            raise DeploymentError(
                f"Jetson {component} compatibility mismatch: "
                f"package={package} expected_prefix={expected} actual={version or '<missing>'}"
            )
        facts[component] = {"package": package, "version": version}
    context.notes["jetson_compatibility"] = facts


def _verify_vendor_python_packages(context: "ExecutionContext") -> None:
    build_account = resolve_build_account()
    for package in context.profile.python.vendor_packages:
        if package.get("required") is not True:
            continue
        distribution = str(package.get("distribution") or "")
        modules = package.get("import_modules")
        if not distribution or not isinstance(modules, list) or not modules:
            raise DeploymentError("vendor package profile entry is incomplete")
        expected_version = package.get("version")
        if (
            context.profile.production
            and not context.options.allow_unverified_third_party
            and not isinstance(expected_version, str)
        ):
            raise DeploymentError(f"approved vendor package version is unresolved: {distribution}")
        expected = expected_version if isinstance(expected_version, str) else ""
        code = """
import importlib
import importlib.metadata
import json
import os
import sys

distribution, encoded_modules, expected, output = sys.argv[1:]
for module in json.loads(encoded_modules):
    importlib.import_module(module)
actual = importlib.metadata.version(distribution)
if expected and actual != expected:
    raise SystemExit("vendor package version mismatch")
with open(output, "w", encoding="utf-8") as handle:
    handle.write(actual + "\\n")
    handle.flush()
    os.fsync(handle.fileno())
"""
        provider = package.get("provider") or "approved vendor"
        with tempfile.TemporaryDirectory(prefix="nuwax-vendor-probe-") as temporary:
            probe_root = Path(temporary)
            os.chmod(probe_root, 0o711)
            result = probe_root / "version"
            descriptor = os.open(result, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC, 0o600)
            try:
                os.fchown(descriptor, build_account.pw_uid, build_account.pw_gid)
                if expected:
                    os.write(descriptor, (expected + "\n").encode("utf-8"))
                    os.fsync(descriptor)
            finally:
                os.close(descriptor)
            try:
                context.runner.run(
                    (
                        context.options.python_bin,
                        "-I",
                        "-c",
                        code,
                        distribution,
                        json.dumps(modules),
                        expected,
                        str(result),
                    ),
                    cwd=Path("/"),
                    env={"HOME": str(context.options.build_home), "PYTHONNOUSERSITE": "1"},
                    as_build_user=True,
                    network_isolated=bool(context.options.offline_bundle),
                    timeout=60,
                )
            except Exception as exc:
                raise DeploymentError(
                    f"required vendor package {distribution} is unavailable from {provider}; "
                    "public PyPI replacement is forbidden"
                ) from exc
            result_descriptor = os.open(result, os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW)
            try:
                metadata = os.fstat(result_descriptor)
                if (
                    not stat.S_ISREG(metadata.st_mode)
                    or metadata.st_uid != build_account.pw_uid
                    or metadata.st_nlink != 1
                    or metadata.st_size > 256
                ):
                    raise DeploymentError(f"vendor package probe result is unsafe: {distribution}")
                actual_version = os.read(result_descriptor, 257).decode("utf-8").strip()
            finally:
                os.close(result_descriptor)
        if re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._+!-]{0,127}", actual_version) is None:
            raise DeploymentError(f"vendor package returned an invalid version: {distribution}")
        if expected and actual_version != expected:
            raise DeploymentError(
                f"vendor package version mismatch: {distribution} expected={expected} actual={actual_version}"
            )
        context.notes[f"vendor:{distribution}"] = actual_version


def _reject_duplicate_keys(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def validate_actuator_manifest(path: Path) -> str:
    if path.is_symlink() or not path.is_file():
        raise DeploymentError(f"actuator manifest must be a non-symlink regular file: {path}")
    payload = path.read_bytes()
    if not payload or len(payload) > 1024 * 1024:
        raise DeploymentError("actuator manifest must be between 1 byte and 1 MiB")
    try:
        document = json.loads(payload.decode("utf-8"), object_pairs_hook=_reject_duplicate_keys)
    except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
        raise DeploymentError(f"invalid actuator manifest JSON: {exc}") from exc
    if not isinstance(document, dict) or not isinstance(document.get("symbols"), list):
        raise DeploymentError("actuator manifest requires a top-level symbols array")
    unknown = sorted(set(document) - {"schema_version", "source", "symbols"})
    if unknown:
        raise DeploymentError(f"actuator manifest contains unknown field: {unknown[0]}")
    if "schema_version" in document and document["schema_version"] != 1:
        raise DeploymentError("actuator manifest schema_version must be 1")
    if "source" in document and (not isinstance(document["source"], str) or not document["source"].strip()):
        raise DeploymentError("actuator manifest source must be a non-empty string")
    symbols = document["symbols"]
    if not symbols:
        raise DeploymentError("actuator manifest symbols must not be empty")
    if not all(
        isinstance(symbol, str) and re.fullmatch(r"[A-Za-z][A-Za-z0-9_.:-]{0,127}", symbol) for symbol in symbols
    ):
        raise DeploymentError("actuator manifest contains an invalid symbol")
    if len(set(symbols)) != len(symbols):
        raise DeploymentError("actuator manifest symbols must be unique")
    metadata = path.stat()
    if metadata.st_mode & (stat.S_IWGRP | stat.S_IWOTH):
        raise DeploymentError("actuator manifest must not be group/world writable")
    allowed_owners = {0}
    sudo_uid = os.environ.get("SUDO_UID", "")
    if sudo_uid.isdigit():
        allowed_owners.add(int(sudo_uid))
    if os.geteuid() == 0 and metadata.st_uid not in allowed_owners:
        raise DeploymentError("actuator manifest must be owned by root or the invoking sudo user")
    return hashlib.sha256(payload).hexdigest()


def validate_bundle_public_key(profile, options, public_key: Path) -> Path:
    public_key = _require_file(public_key, "offline bundle public key")
    expected_key_digest = profile.metadata.get("bundle_public_key_sha256")
    strict = profile.production and not options.allow_unverified_third_party
    if strict:
        if not isinstance(expected_key_digest, str):
            raise DeploymentError("production profile has no approved offline bundle public key")
        for component in (public_key, *public_key.parents):
            metadata = component.lstat()
            if component.is_symlink() or metadata.st_uid != 0 or metadata.st_mode & (stat.S_IWGRP | stat.S_IWOTH):
                raise DeploymentError(f"approved offline bundle public key path is not root-managed: {component}")
            if component == Path("/"):
                break
    if isinstance(expected_key_digest, str) and _sha256(public_key) != expected_key_digest:
        raise DeploymentError("offline bundle public key does not match the production profile")
    return public_key


def validate_offline_bundle_contract(profile, options, manifest: Mapping[str, object]) -> None:
    metadata = manifest.get("metadata")
    if not isinstance(metadata, dict):
        raise DeploymentError("offline bundle metadata is invalid")
    if metadata.get("profile") != profile.name:
        raise DeploymentError("offline bundle profile does not match deployment profile")
    if metadata.get("profile_sha256") != profile_sha256(profile):
        raise DeploymentError("offline bundle profile digest does not match trusted profile")
    if metadata.get("release_id") != options.release_id:
        raise DeploymentError("offline bundle release_id does not match requested release")
    if profile.rust.toolchain and metadata.get("rust_toolchain") != profile.rust.toolchain:
        raise DeploymentError("offline bundle Rust toolchain does not match profile")
    if profile.python.version and metadata.get("python_version") != profile.python.version:
        raise DeploymentError("offline bundle Python version does not match profile")
    if not isinstance(metadata.get("source_revision"), str) or not metadata["source_revision"]:
        raise DeploymentError("offline bundle source revision is missing")
    expected_os = {key: list(value) for key, value in profile.supported_os.items()}
    if metadata.get("supported_os") != expected_os:
        raise DeploymentError("offline bundle OS compatibility does not match profile")
    bundle_architectures = metadata.get("architectures")
    if bundle_architectures != list(profile.architectures):
        raise DeploymentError("offline bundle architecture metadata does not match profile")
    if platform.machine().lower() not in bundle_architectures and not options.allow_unsupported_host:
        raise DeploymentError("offline bundle does not support this architecture")


def verify_offline_bundle(path: Path, public_key: Path) -> Mapping[str, object]:
    if path.is_symlink() or not path.is_dir():
        raise DeploymentError(f"offline bundle must be a non-symlink directory: {path}")
    manifest_path = path / "bundle-manifest.json"
    signature_path = path / "bundle-manifest.sig"
    if manifest_path.is_symlink() or not manifest_path.is_file():
        raise DeploymentError(f"offline bundle manifest missing: {manifest_path}")
    if signature_path.is_symlink() or not signature_path.is_file():
        raise DeploymentError(f"offline bundle signature missing: {signature_path}")
    if public_key.is_symlink() or not public_key.is_file():
        raise DeploymentError(f"offline bundle public key missing or unsafe: {public_key}")
    if manifest_path.stat().st_size > 16 * 1024 * 1024:
        raise DeploymentError("offline bundle manifest exceeds 16 MiB")
    verified = subprocess.run(
        (
            "openssl",
            "dgst",
            "-sha256",
            "-verify",
            str(public_key),
            "-signature",
            str(signature_path),
            str(manifest_path),
        ),
        check=False,
        capture_output=True,
        timeout=60,
    )
    if verified.returncode != 0:
        raise DeploymentError("offline bundle manifest signature verification failed")
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise DeploymentError(f"invalid offline bundle manifest: {exc}") from exc
    if (
        not isinstance(manifest, dict)
        or manifest.get("schema_version") != 1
        or manifest.get("signature_algorithm") != "rsa-sha256"
        or not isinstance(manifest.get("metadata"), dict)
        or not isinstance(manifest.get("files"), dict)
    ):
        raise DeploymentError("offline bundle manifest schema is invalid")
    if len(manifest["files"]) > 500_000:
        raise DeploymentError("offline bundle contains too many manifest files")

    actual_files = {}
    directories = []
    node_count = 0
    for candidate in path.rglob("*"):
        node_count += 1
        if node_count > 750_000:
            raise DeploymentError("offline bundle contains too many filesystem nodes")
        if len(candidate.relative_to(path).parts) > 64:
            raise DeploymentError(f"offline bundle path is too deep: {candidate}")
        if candidate.is_symlink():
            raise DeploymentError(f"offline bundle must not contain symlinks: {candidate}")
        if candidate.is_dir():
            directories.append(candidate)
            continue
        if not candidate.is_file():
            raise DeploymentError(f"offline bundle contains a special filesystem node: {candidate}")
        relative = candidate.relative_to(path).as_posix()
        if relative in {"bundle-manifest.json", "bundle-manifest.sig"}:
            continue
        actual_files[relative] = candidate
    for directory in directories:
        if not any(directory.iterdir()) and not (directory.parent == path and directory.name in _REQUIRED_BUNDLE_DIRS):
            raise DeploymentError(f"offline bundle contains an unsigned empty directory: {directory}")

    expected_files = set(manifest["files"])
    actual_names = set(actual_files)
    unexpected = sorted(actual_names - expected_files)
    if unexpected:
        raise DeploymentError(f"offline bundle contains unexpected file: {unexpected[0]}")
    missing = sorted(expected_files - actual_names)
    if missing:
        raise DeploymentError(f"offline bundle file missing: {missing[0]}")

    root = path.resolve()
    for relative, expected in manifest["files"].items():
        normalized = Path(relative)
        if (
            not relative
            or normalized.is_absolute()
            or ".." in normalized.parts
            or normalized.as_posix() != relative
            or not isinstance(expected, dict)
        ):
            raise DeploymentError(f"offline bundle manifest path is invalid: {relative!r}")
        expected_digest = expected.get("sha256")
        expected_size = expected.get("size")
        if (
            not isinstance(expected_digest, str)
            or re.fullmatch(r"[0-9a-f]{64}", expected_digest) is None
            or not isinstance(expected_size, int)
            or expected_size < 0
        ):
            raise DeploymentError(f"offline bundle manifest entry is invalid: {relative}")
        candidate = (path / relative).resolve()
        if root not in candidate.parents or not candidate.is_file():
            raise DeploymentError(f"offline bundle file missing or escapes bundle: {relative}")
        if candidate.stat().st_size != expected_size:
            raise DeploymentError(f"offline bundle size mismatch: {relative}")
        actual = _sha256(candidate)
        if actual != expected_digest:
            raise DeploymentError(f"offline bundle checksum mismatch: {relative}")
    for required in sorted(_REQUIRED_BUNDLE_DIRS):
        if not (path / required).is_dir():
            raise DeploymentError(f"offline bundle is incomplete: missing {required}/")
    for proxy in ("cargo", "rustc", "rustup"):
        executable = path / "cargo-bin" / proxy
        if not executable.is_file() or not os.access(executable, os.X_OK):
            raise DeploymentError(f"offline bundle toolchain proxy is not executable: {proxy}")
    return manifest


def _prepare_root_managed_cache(cache_dir: Path, build_gid: int) -> Path:
    cache_dir = Path(os.path.abspath(cache_dir))
    ancestor = Path(cache_dir.anchor)
    for part in cache_dir.parts[1:]:
        ancestor /= part
        if ancestor.is_symlink():
            raise DeploymentError(f"deployment cache path is a symlink: {ancestor}")
        if not ancestor.exists():
            break
        metadata = ancestor.lstat()
        if (
            not stat.S_ISDIR(metadata.st_mode)
            or metadata.st_uid != 0
            or metadata.st_mode & (stat.S_IWGRP | stat.S_IWOTH)
        ):
            raise DeploymentError(f"deployment cache ancestor is not root-managed: {ancestor}")
    cache_dir.mkdir(parents=True, exist_ok=True)
    os.chown(cache_dir, 0, build_gid)
    os.chmod(cache_dir, 0o750)
    return cache_dir


def _freeze_root_tree(root: Path, build_gid: int) -> None:
    _seal_tree(
        root,
        owner_uid=0,
        group_gid=build_gid,
        public_read=False,
        allow_symlinks=False,
        label="immutable cache tree",
    )


def _freeze_toolchain_tree(root: Path, build_gid: int) -> None:
    symlinks = _seal_tree(
        root,
        owner_uid=0,
        group_gid=build_gid,
        public_read=False,
        allow_symlinks=True,
        label="Rust toolchain",
    )
    resolved_root = root.resolve()
    for candidate in symlinks:
        resolved = candidate.resolve()
        if resolved_root != resolved and resolved_root not in resolved.parents:
            raise DeploymentError(f"Rust toolchain symlink escapes its root: {candidate}")


def _prepare_mutable_build_tree(path: Path, build_uid: int, build_gid: int, label: str) -> Path:
    if path.is_symlink() or (path.exists() and not path.is_dir()):
        raise DeploymentError(f"{label} is unsafe: {path}")
    if path.is_dir() and path.lstat().st_uid != build_uid:
        shutil.rmtree(path)
    path.mkdir(parents=True, exist_ok=True)
    os.chown(path, build_uid, build_gid)
    os.chmod(path, 0o750)
    return path


def _prepare_managed_directory(path: Path, group_gid: int, mode: int, label: str) -> Path:
    if path.is_symlink():
        raise DeploymentError(f"{label} must not be a symlink: {path}")
    try:
        path.mkdir(mode=mode)
    except FileExistsError:
        pass
    except OSError as exc:
        raise DeploymentError(f"cannot create {label}: {path}") from exc
    metadata = path.lstat()
    if (
        not stat.S_ISDIR(metadata.st_mode)
        or metadata.st_uid != os.geteuid()
        or metadata.st_mode & (stat.S_IWGRP | stat.S_IWOTH)
    ):
        raise DeploymentError(f"{label} is not privileged-process managed: {path}")
    os.chown(path, os.geteuid(), group_gid)
    os.chmod(path, mode)
    return path


def _prepare_build_root(context: "ExecutionContext", build_gid: int) -> Path:
    options = context.options
    _prepare_managed_directory(options.cache_dir, build_gid, 0o750, "deployment cache root")
    _prepare_managed_directory(options.cache_dir / "build", build_gid, 0o750, "build cache root")
    return _prepare_managed_directory(options.build_dir, build_gid, 0o750, "release build directory")


def _prepare_release_root(context: "ExecutionContext") -> Path:
    options = context.options
    public_gid = os.getegid()
    _prepare_managed_directory(options.install_root, public_gid, 0o755, "installation root")
    _prepare_managed_directory(options.releases_dir, public_gid, 0o755, "release root")
    return _prepare_managed_directory(options.release_dir, public_gid, 0o755, "candidate release directory")


def _open_directory(path: Path, label: str) -> int:
    flags = os.O_RDONLY | os.O_CLOEXEC | os.O_DIRECTORY | os.O_NOFOLLOW
    try:
        descriptor = os.open(path, flags)
    except OSError as exc:
        raise DeploymentError(f"{label} is not a safe directory: {path}") from exc
    if not stat.S_ISDIR(os.fstat(descriptor).st_mode):
        os.close(descriptor)
        raise DeploymentError(f"{label} is not a directory: {path}")
    return descriptor


def _open_directory_at(parent: int, name: str, label: str) -> int:
    if not name or name in {".", ".."} or Path(name).name != name:
        raise DeploymentError(f"{label} has an invalid name: {name!r}")
    flags = os.O_RDONLY | os.O_CLOEXEC | os.O_DIRECTORY | os.O_NOFOLLOW
    try:
        descriptor = os.open(name, flags, dir_fd=parent)
    except OSError as exc:
        raise DeploymentError(f"{label} is not a safe directory: {name}") from exc
    if not stat.S_ISDIR(os.fstat(descriptor).st_mode):
        os.close(descriptor)
        raise DeploymentError(f"{label} is not a directory: {name}")
    return descriptor


def _seal_tree(
    root: Path,
    *,
    owner_uid: int,
    group_gid: int,
    public_read: bool,
    allow_symlinks: bool,
    label: str,
) -> Tuple[Path, ...]:
    root_descriptor = _open_directory(root, label)
    root_device = os.fstat(root_descriptor).st_dev
    symlinks = []

    def seal_directory(directory_descriptor: int, directory_path: Path) -> None:
        metadata = os.fstat(directory_descriptor)
        if metadata.st_dev != root_device:
            raise DeploymentError(f"{label} crosses a filesystem boundary: {directory_path}")
        os.fchown(directory_descriptor, owner_uid, group_gid)
        os.fchmod(directory_descriptor, 0o555 if public_read else 0o550)
        for name in sorted(os.listdir(directory_descriptor)):
            if not name or name in {".", ".."} or Path(name).name != name:
                raise DeploymentError(f"{label} contains an invalid entry name: {name!r}")
            candidate = directory_path / name
            entry = os.stat(name, dir_fd=directory_descriptor, follow_symlinks=False)
            if entry.st_dev != root_device:
                raise DeploymentError(f"{label} crosses a filesystem boundary: {candidate}")
            if stat.S_ISLNK(entry.st_mode):
                if not allow_symlinks:
                    raise DeploymentError(f"{label} contains a symlink: {candidate}")
                symlinks.append(candidate)
                continue
            if stat.S_ISDIR(entry.st_mode):
                child_descriptor = os.open(
                    name,
                    os.O_RDONLY | os.O_CLOEXEC | os.O_DIRECTORY | os.O_NOFOLLOW,
                    dir_fd=directory_descriptor,
                )
                try:
                    opened = os.fstat(child_descriptor)
                    if (opened.st_dev, opened.st_ino) != (entry.st_dev, entry.st_ino):
                        raise DeploymentError(f"{label} directory changed while being sealed: {candidate}")
                    seal_directory(child_descriptor, candidate)
                finally:
                    os.close(child_descriptor)
                continue
            if stat.S_ISREG(entry.st_mode):
                child_descriptor = os.open(
                    name,
                    os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW,
                    dir_fd=directory_descriptor,
                )
                try:
                    opened = os.fstat(child_descriptor)
                    if (opened.st_dev, opened.st_ino) != (entry.st_dev, entry.st_ino):
                        raise DeploymentError(f"{label} file changed while being sealed: {candidate}")
                    os.fchown(child_descriptor, owner_uid, group_gid)
                    if public_read:
                        os.fchmod(child_descriptor, 0o555 if entry.st_mode & 0o111 else 0o444)
                    else:
                        os.fchmod(child_descriptor, 0o550 if entry.st_mode & 0o111 else 0o440)
                finally:
                    os.close(child_descriptor)
                continue
            raise DeploymentError(f"{label} contains a special file: {candidate}")

    try:
        seal_directory(root_descriptor, root)
    finally:
        os.close(root_descriptor)
    return tuple(symlinks)


def _open_build_output(parent: int, name: str, build_uid: int, label: str) -> Tuple[int, os.stat_result]:
    if not name or name in {".", ".."} or Path(name).name != name:
        raise DeploymentError(f"{label} has an invalid name: {name!r}")
    flags = os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW
    try:
        descriptor = os.open(name, flags, dir_fd=parent)
    except OSError as exc:
        raise DeploymentError(f"{label} is missing or unsafe: {name}") from exc
    metadata = os.fstat(descriptor)
    if not stat.S_ISREG(metadata.st_mode) or metadata.st_uid != build_uid or not metadata.st_mode & 0o111:
        os.close(descriptor)
        raise DeploymentError(f"{label} is not a build-user-owned executable: {name}")
    return descriptor, metadata


def _file_identity(metadata: os.stat_result) -> Tuple[int, ...]:
    return (
        metadata.st_dev,
        metadata.st_ino,
        metadata.st_mode,
        metadata.st_uid,
        metadata.st_gid,
        metadata.st_size,
        metadata.st_mtime_ns,
        metadata.st_ctime_ns,
    )


def _copy_open_build_output(
    source_descriptor: int,
    source_metadata: os.stat_result,
    destination: Path,
    label: str,
) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    parent_descriptor = _open_directory(destination.parent, f"{label} destination")
    parent_metadata = os.fstat(parent_descriptor)
    if parent_metadata.st_uid != os.geteuid() or parent_metadata.st_mode & (stat.S_IWGRP | stat.S_IWOTH):
        os.close(parent_descriptor)
        raise DeploymentError(f"{label} destination is not privileged-process managed: {destination.parent}")
    temporary_name = f".{destination.name}.{os.getpid()}.{time.time_ns()}.tmp"
    destination_descriptor = -1
    try:
        destination_descriptor = os.open(
            temporary_name,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC | os.O_NOFOLLOW,
            0o600,
            dir_fd=parent_descriptor,
        )
        os.lseek(source_descriptor, 0, os.SEEK_SET)
        with os.fdopen(os.dup(source_descriptor), "rb") as source_handle, os.fdopen(
            destination_descriptor, "wb"
        ) as destination_handle:
            destination_descriptor = -1
            shutil.copyfileobj(source_handle, destination_handle, length=1024 * 1024)
            destination_handle.flush()
            os.fchmod(destination_handle.fileno(), 0o755)
            os.fsync(destination_handle.fileno())
        if _file_identity(os.fstat(source_descriptor)) != _file_identity(source_metadata):
            raise DeploymentError(f"{label} changed while it was copied")
        os.replace(
            temporary_name,
            destination.name,
            src_dir_fd=parent_descriptor,
            dst_dir_fd=parent_descriptor,
        )
        os.fsync(parent_descriptor)
    except Exception:
        try:
            os.unlink(temporary_name, dir_fd=parent_descriptor)
        except FileNotFoundError:
            pass
        raise
    finally:
        if destination_descriptor >= 0:
            os.close(destination_descriptor)
        os.close(parent_descriptor)


def _copy_rust_build_output(target: Path, binary: str, destination: Path, build_uid: int) -> None:
    target_descriptor = _open_directory(target, "Rust target directory")
    release_descriptor = -1
    source_descriptor = -1
    try:
        release_descriptor = _open_directory_at(target_descriptor, "release", "Rust release directory")
        source_descriptor, source_metadata = _open_build_output(
            release_descriptor,
            binary,
            build_uid,
            "Rust build output",
        )
        _copy_open_build_output(source_descriptor, source_metadata, destination, "Rust build output")
    finally:
        if source_descriptor >= 0:
            os.close(source_descriptor)
        if release_descriptor >= 0:
            os.close(release_descriptor)
        os.close(target_descriptor)


def _copy_rustup_proxies(cargo_home: Path, destination: Path, build_uid: int) -> None:
    cargo_descriptor = _open_directory(cargo_home, "Cargo home")
    bin_descriptor = -1
    source_descriptor = -1
    try:
        bin_descriptor = _open_directory_at(cargo_descriptor, "bin", "Cargo bin directory")
        source_descriptor, source_metadata = _open_build_output(
            bin_descriptor,
            "rustup",
            build_uid,
            "rustup proxy source",
        )
        for proxy in ("rustup", "rustc", "cargo"):
            _copy_open_build_output(
                source_descriptor,
                source_metadata,
                destination / proxy,
                f"Rust {proxy} proxy",
            )
    finally:
        if source_descriptor >= 0:
            os.close(source_descriptor)
        if bin_descriptor >= 0:
            os.close(bin_descriptor)
        os.close(cargo_descriptor)


def stage_verified_bundle(path: Path, public_key: Path, cache_dir: Path) -> Path:
    bundle_root = path.resolve()
    cache_root = Path(os.path.abspath(cache_dir)).resolve(strict=False)
    if bundle_root == cache_root or bundle_root in cache_root.parents or cache_root in bundle_root.parents:
        raise DeploymentError("verified cache and input bundle paths must not overlap")
    manifest = verify_offline_bundle(path, public_key)
    manifest_digest = _sha256(path / "bundle-manifest.json")
    cache_dir = cache_root
    target_parent = cache_dir / "verified-bundles"
    build_gid = resolve_build_account().pw_gid
    cache_dir = _prepare_root_managed_cache(cache_dir, build_gid)
    target = target_parent / manifest_digest
    if target.is_symlink():
        raise DeploymentError(f"verified bundle cache must not be a symlink: {target}")
    if target.exists() and not target.is_dir():
        raise DeploymentError(f"verified bundle cache target is not a directory: {target}")
    if target.is_dir():
        candidates = (target, *target.rglob("*"))
        for candidate in candidates:
            metadata = candidate.lstat()
            if candidate.is_symlink() or metadata.st_uid != 0 or metadata.st_mode & (stat.S_IWGRP | stat.S_IWOTH):
                raise DeploymentError(f"verified bundle cache ownership/mode is unsafe: {candidate}")
        for candidate in candidates:
            metadata = candidate.lstat()
            os.chown(candidate, 0, build_gid)
            if stat.S_ISDIR(metadata.st_mode):
                os.chmod(candidate, 0o550)
            elif stat.S_ISREG(metadata.st_mode):
                executable = bool(metadata.st_mode & stat.S_IXUSR)
                os.chmod(candidate, 0o550 if executable else 0o440)
            else:
                raise DeploymentError(f"verified bundle cache contains a special file: {candidate}")
        if _sha256(target / "bundle-manifest.json") != target.name:
            raise DeploymentError("verified bundle cache key does not match its manifest")
        verify_offline_bundle(target, public_key)
        return target
    files = manifest.get("files")
    if not isinstance(files, dict):
        raise DeploymentError("offline bundle manifest files are invalid")
    required_bytes = sum(int(record.get("size", 0)) for record in files.values() if isinstance(record, dict))
    probe = cache_dir
    while not probe.exists() and probe != probe.parent:
        probe = probe.parent
    if shutil.disk_usage(probe).free < required_bytes + 1024 * 1024 * 1024:
        raise DeploymentError("insufficient cache disk for a verified offline bundle snapshot")
    if target_parent.is_symlink():
        raise DeploymentError(f"verified bundle cache parent must not be a symlink: {target_parent}")
    if target_parent.exists():
        metadata = target_parent.stat()
        if metadata.st_uid != 0 or metadata.st_mode & (stat.S_IWGRP | stat.S_IWOTH):
            raise DeploymentError(f"verified bundle cache parent is not root-managed: {target_parent}")
    target_parent.mkdir(parents=True, exist_ok=True)
    os.chown(target_parent, 0, build_gid)
    os.chmod(target_parent, 0o750)
    staging = Path(tempfile.mkdtemp(prefix=f".{manifest_digest}.staging-", dir=target_parent))
    shutil.rmtree(staging)
    try:
        shutil.copytree(path, staging, symlinks=True)
        verify_offline_bundle(staging, public_key)
        if _sha256(staging / "bundle-manifest.json") != manifest_digest:
            raise DeploymentError("offline bundle changed while entering the verified cache")
        candidates = (staging, *staging.rglob("*"))
        for candidate in candidates:
            metadata = candidate.lstat()
            if candidate.is_symlink():
                raise DeploymentError(f"verified bundle staging contains a symlink: {candidate}")
            os.chown(candidate, 0, build_gid)
            if stat.S_ISDIR(metadata.st_mode):
                os.chmod(candidate, 0o550)
            elif stat.S_ISREG(metadata.st_mode):
                executable = bool(metadata.st_mode & stat.S_IXUSR)
                os.chmod(candidate, 0o550 if executable else 0o440)
            else:
                raise DeploymentError(f"verified bundle staging contains a special file: {candidate}")
        verify_offline_bundle(staging, public_key)
        os.replace(staging, target)
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    return target


def stage_source_snapshot(source_root: Path, cache_dir: Path) -> Tuple[Path, str, str]:
    from .bundle import BundleError, _copy_source_snapshot

    source_root = _require_dir(source_root, "production source checkout")
    build_gid = resolve_build_account().pw_gid
    cache_dir = _prepare_root_managed_cache(cache_dir, build_gid)
    if source_root == cache_dir or source_root in cache_dir.parents or cache_dir in source_root.parents:
        raise DeploymentError("production source and snapshot cache paths must not overlap")
    git_env = source_user_environment()
    head = subprocess.run(
        source_user_command(("git", "rev-parse", "HEAD")),
        cwd=str(source_root),
        env=git_env,
        capture_output=True,
        text=True,
        check=False,
        timeout=30,
    )
    tree = subprocess.run(
        source_user_command(("git", "rev-parse", "HEAD^{tree}")),
        cwd=str(source_root),
        env=git_env,
        capture_output=True,
        text=True,
        check=False,
        timeout=30,
    )
    revision = head.stdout.strip()
    tree_id = tree.stdout.strip()
    if head.returncode != 0 or tree.returncode != 0 or not revision or not tree_id:
        raise DeploymentError("cannot resolve production source Git identity")
    _git_source_identity(source_root)
    target_parent = cache_dir / "source-snapshots"
    if target_parent.is_symlink():
        raise DeploymentError(f"source snapshot cache is a symlink: {target_parent}")
    target_parent.mkdir(parents=True, exist_ok=True)
    os.chown(target_parent, 0, build_gid)
    os.chmod(target_parent, 0o750)
    target = target_parent / f"{revision}-{tree_id[:16]}"
    snapshot_manifest = target_parent / f"{target.name}.json"
    if target.is_symlink() or (target.exists() and not target.is_dir()):
        raise DeploymentError(f"source snapshot cache target is unsafe: {target}")
    if target.is_dir() and not snapshot_manifest.exists():
        shutil.rmtree(target)
    if target.is_dir():
        for candidate in (target, *target.rglob("*")):
            metadata = candidate.lstat()
            if candidate.is_symlink() or metadata.st_uid != 0 or metadata.st_mode & (stat.S_IWGRP | stat.S_IWOTH):
                raise DeploymentError(f"source snapshot cache was modified: {candidate}")
        snapshot_manifest = _require_file(
            snapshot_manifest,
            "source snapshot integrity manifest",
        )
        manifest_metadata = snapshot_manifest.stat()
        if manifest_metadata.st_uid != 0 or manifest_metadata.st_mode & (stat.S_IWGRP | stat.S_IWOTH):
            raise DeploymentError("source snapshot integrity manifest is unsafe")
        try:
            snapshot_record = json.loads(snapshot_manifest.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise DeploymentError("source snapshot integrity manifest is invalid") from exc
        if (
            snapshot_record.get("revision") != revision
            or snapshot_record.get("tree_id") != tree_id
            or snapshot_record.get("tree_sha256") != _tree_digest(target)
        ):
            raise DeploymentError("source snapshot cache failed integrity verification")
        _freeze_root_tree(target, build_gid)
        return target, revision, tree_id
    staging = Path(tempfile.mkdtemp(prefix=f".{target.name}.staging-", dir=target_parent))
    shutil.rmtree(staging)
    try:
        _copy_source_snapshot(source_root, staging, revision)
        tree_sha256 = _tree_digest(staging)
        _freeze_root_tree(staging, build_gid)
        os.replace(staging, target)
        atomic_write(
            snapshot_manifest,
            json.dumps(
                {
                    "schema_version": 1,
                    "revision": revision,
                    "tree_id": tree_id,
                    "tree_sha256": tree_sha256,
                },
                indent=2,
                sort_keys=True,
            )
            + "\n",
            mode=0o440,
        )
        fsync_directory(target_parent)
    except BundleError as exc:
        raise DeploymentError(str(exc)) from exc
    finally:
        if staging.exists():
            shutil.rmtree(staging, ignore_errors=True)
    return target, revision, tree_id


def verify_input_admission(context: "ExecutionContext") -> None:
    options = context.options
    if options.offline_bundle is not None:
        bundle = _require_dir(options.offline_bundle, "offline bundle")
        public_key = validate_bundle_public_key(
            context.profile,
            options,
            options.bundle_public_key,
        )
        manifest = verify_offline_bundle(bundle, public_key)
        validate_offline_bundle_contract(context.profile, options, manifest)
        context.notes["bundle_manifest"] = manifest
        return
    if context.profile.production and not options.allow_dirty_source:
        if options.source_is_snapshot:
            for candidate in (options.source_root, *options.source_root.rglob("*")):
                metadata = candidate.lstat()
                if candidate.is_symlink() or metadata.st_uid != 0 or metadata.st_mode & (stat.S_IWGRP | stat.S_IWOTH):
                    raise DeploymentError(f"production source snapshot is unsafe: {candidate}")
        else:
            live_source = options.fingerprint_source_root or options.source_root
            _git_source_identity(live_source)


def validate_protected_paths(options) -> None:
    for label, requested in (
        ("install root", options.install_root),
        ("configuration root", options.config_dir),
        ("state root", options.state_dir),
        ("cache root", options.cache_dir),
    ):
        path = Path(os.path.abspath(requested))
        current = Path(path.anchor)
        for part in path.parts[1:]:
            current /= part
            if current.is_symlink():
                raise DeploymentError(f"{label} path contains a symlink: {current}")
            if not current.exists():
                break
            metadata = current.lstat()
            if (
                not stat.S_ISDIR(metadata.st_mode)
                or metadata.st_uid != 0
                or metadata.st_mode & (stat.S_IWGRP | stat.S_IWOTH)
            ):
                raise DeploymentError(f"{label} path is not root-managed: {current}")


def preflight(context: "ExecutionContext") -> None:
    profile = context.profile
    options = context.options
    if os.geteuid() != 0:
        raise DeploymentError("deploy must run as root; use ./install.sh deploy ...")
    sudo_uid = os.environ.get("SUDO_UID", "0")
    if options.allow_unverified_third_party and (not sudo_uid.isdigit() or int(sudo_uid) != 0):
        raise DeploymentError("--allow-unverified-third-party may only run from a root-owned controlled session")
    if profile.production and options.allow_unsupported_host:
        raise DeploymentError("production profiles cannot bypass OS, architecture, or Python compatibility")
    if not options.source_root.is_dir():
        raise DeploymentError(f"source root does not exist: {options.source_root}")
    if options.release_dir.is_symlink():
        raise DeploymentError(f"release directory must not be a symlink: {options.release_dir}")
    if options.legacy_config is not None:
        if options.legacy_config.is_symlink():
            raise DeploymentError("legacy migration config must not be a symlink")
        _require_file(options.legacy_config, "legacy migration config")
    if options.offline_bundle is not None:
        bundle = _require_dir(options.offline_bundle, "offline bundle")
        public_key = validate_bundle_public_key(
            profile,
            options,
            options.bundle_public_key,
        )
        manifest = verify_offline_bundle(bundle, public_key)
        validate_offline_bundle_contract(profile, options, manifest)
        unshare = Path("/usr/bin/unshare")
        if not unshare.is_file():
            raise DeploymentError("offline deployment requires /usr/bin/unshare from util-linux")
        isolated = subprocess.run(
            (str(unshare), "--net", "--", "true"),
            capture_output=True,
            check=False,
            timeout=30,
        )
        if isolated.returncode != 0:
            raise DeploymentError("host cannot create the network namespace required for offline builds")
        if options.source_root != (bundle / "source").resolve():
            raise DeploymentError("offline deployment must build the signed bundle source snapshot")
        context.notes["bundle_manifest"] = manifest
    os_id, version_id = _os_release()
    architecture = platform.machine().lower()
    if not options.allow_unsupported_host and not profile.supports(os_id, version_id, architecture):
        raise DeploymentError(f"profile {profile.name} does not support {os_id} {version_id} {architecture}")
    _verify_python_runtime(context)
    if profile.production and profile.python.lock_status != "hashed":
        if not options.allow_unlocked_python:
            raise DeploymentError(
                f"production profile Python lock_status={profile.python.lock_status!r}; "
                "generate the approved lock or pass --allow-unlocked-python for controlled bring-up"
            )
    if profile.production and profile.metadata.get("deployable") is False:
        if not options.allow_unverified_third_party:
            blockers = profile.metadata.get("blockers") or []
            detail = "; ".join(str(item) for item in blockers)
            raise DeploymentError(
                "production profile is fail-closed because third-party artifacts are unresolved"
                + (f": {detail}" if detail else "")
            )
    validate_protected_paths(options)
    _validate_jetson_compatibility(context)
    protected_roots = {
        "install_root": options.install_root.resolve(),
        "config_dir": options.config_dir.resolve(),
        "state_dir": options.state_dir.resolve(),
        "cache_dir": options.cache_dir.resolve(),
    }
    immutable_roots = {
        "source_root": options.source_root.resolve(),
        "trust_root": (options.trust_root or options.source_root).resolve(),
    }
    if options.offline_bundle is not None:
        immutable_roots["offline_bundle"] = options.offline_bundle.resolve()

    def paths_overlap(left: Path, right: Path) -> bool:
        return left == right or left in right.parents or right in left.parents

    mutable_roots = {
        "install_root": protected_roots["install_root"],
        "config_dir": protected_roots["config_dir"],
        "state_dir": protected_roots["state_dir"],
        ("build_dir" if options.offline_bundle is not None or options.source_is_snapshot else "cache_dir"): (
            options.build_dir.resolve()
            if options.offline_bundle is not None or options.source_is_snapshot
            else protected_roots["cache_dir"]
        ),
    }
    for protected_name, protected_path in mutable_roots.items():
        for immutable_name, immutable_path in immutable_roots.items():
            if paths_overlap(protected_path, immutable_path):
                raise DeploymentError(f"deployment paths overlap: {protected_name} and {immutable_name}")
    protected_items = tuple(protected_roots.items())
    for index, (left_name, left_path) in enumerate(protected_items):
        for right_name, right_path in protected_items[index + 1 :]:
            if paths_overlap(left_path, right_path):
                raise DeploymentError(f"deployment paths overlap: {left_name} and {right_name}")
    if profile.production and not options.allow_dirty_source:
        if options.source_is_snapshot:
            for candidate in (options.source_root, *options.source_root.rglob("*")):
                metadata = candidate.lstat()
                if candidate.is_symlink() or metadata.st_uid != 0 or metadata.st_mode & (stat.S_IWGRP | stat.S_IWOTH):
                    raise DeploymentError(f"production source snapshot is unsafe: {candidate}")
        elif options.offline_bundle is None:
            clean_roots = [options.trust_root or options.source_root]
            if options.source_root not in clean_roots:
                clean_roots.append(options.source_root)
            for clean_root in clean_roots:
                dirty = subprocess.run(
                    source_user_command(("git", "status", "--porcelain", "--untracked-files=normal")),
                    cwd=str(clean_root),
                    env=source_user_environment(),
                    capture_output=True,
                    text=True,
                    check=False,
                    timeout=15,
                )
                if dirty.returncode != 0 or dirty.stdout.strip():
                    raise DeploymentError(
                        "production deployment requires a clean trusted installer/source revision; "
                        "--allow-dirty-source is only for controlled development"
                    )
            context.notes["source_identity_at_preflight"] = _git_source_identity(options.source_root)
    for relative in _python_requirement_paths(context):
        requirement = _require_file(options.source_root / relative, f"Python requirement {relative}")
        _validate_hashed_requirement_policy(context, requirement)
    if profile.python.constraints:
        _require_file(options.source_root / profile.python.constraints, "Python constraints")
    if profile.python.source_artifact_manifest:
        _require_file(
            options.source_root / profile.python.source_artifact_manifest,
            "Python source artifact manifest",
        )
    _verify_vendor_python_packages(context)
    for build in profile.rust.builds:
        _require_file(options.source_root / build.manifest, f"Rust manifest {build.name}")
    _require_file(options.source_root / "scripts" / "install_motion_guard_deployment.sh", "systemd installer")
    if profile.robot == "go2":
        _resolve_go2_artifacts(context)
        _verify_source_artifacts(context)
    if not options.no_systemd:
        manifest = _require_file(options.actuator_manifest, "actuator symbols manifest")
        context.notes["actuator_manifest_sha256"] = validate_actuator_manifest(manifest)
    minimum_free_gb = int(profile.metadata.get("minimum_free_gb", 30))
    for label, requested in (
        ("source", options.source_root),
        ("install", options.install_root),
        ("cache", options.cache_dir),
    ):
        probe = requested
        while not probe.exists() and probe != probe.parent:
            probe = probe.parent
        free_gb = shutil.disk_usage(probe).free // (1024**3)
        if free_gb < minimum_free_gb:
            raise DeploymentError(f"insufficient {label} disk: {free_gb} GiB available, {minimum_free_gb} GiB required")
    if options.dds_interface:
        iface = Path("/sys/class/net") / options.dds_interface
        if not iface.is_dir():
            raise DeploymentError(f"DDS interface does not exist: {options.dds_interface}")
    required_interface = profile.metadata.get("required_dds_interface")
    if profile.production and isinstance(required_interface, str):
        if options.dds_interface != required_interface:
            raise DeploymentError(f"profile {profile.name} requires --dds-interface {required_interface}")
    existing_manifest = options.release_dir / "release-manifest.json"
    if existing_manifest.is_file():
        try:
            existing = json.loads(existing_manifest.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise DeploymentError(f"existing release manifest is invalid: {existing_manifest}") from exc
        if existing.get("fingerprint") != context.plan.fingerprint:
            raise DeploymentError(f"release id {options.release_id} already belongs to a different deployment")
        _validate_completed_release(options.release_dir, existing)
        context.notes["reuse_release"] = True


def _missing_system_packages(packages: Sequence[str]) -> Tuple[str, ...]:
    missing = []
    for package_spec in packages:
        package, separator, expected_version = package_spec.partition("=")
        if not package or (separator and not expected_version):
            raise DeploymentError(f"invalid system package specification: {package_spec}")
        completed = subprocess.run(
            ("dpkg-query", "-W", "-f=${db:Status-Abbrev}\t${Version}", package),
            capture_output=True,
            text=True,
            check=False,
            timeout=30,
        )
        status, _, actual_version = completed.stdout.partition("\t")
        if (
            completed.returncode != 0
            or not status.startswith("ii ")
            or separator
            and actual_version.strip() != expected_version
        ):
            missing.append(package_spec)
    return tuple(missing)


def _record_system_package_versions(context: "ExecutionContext") -> None:
    versions = {}
    for package_spec in context.profile.system_packages:
        package = package_spec.partition("=")[0]
        version = _installed_package_version(package)
        if not version:
            raise DeploymentError(f"required system package is still missing: {package}")
        versions[package] = version
    context.notes["system_package_versions"] = versions


def install_online_system_packages(context: "ExecutionContext") -> None:
    missing = _missing_system_packages(context.profile.system_packages)
    if not missing:
        context.notes["system_packages_unchanged"] = True
        _record_system_package_versions(context)
        return
    context.runner.run(("apt-get", "update"), timeout=900)
    context.runner.run(
        ("apt-get", "install", "-y", "--no-install-recommends", *missing),
        timeout=1800,
    )
    _record_system_package_versions(context)


def install_offline_system_packages(context: "ExecutionContext") -> None:
    missing = _missing_system_packages(context.profile.system_packages)
    if not missing:
        context.notes["system_packages_unchanged"] = True
        _record_system_package_versions(context)
        return
    bundle = _require_dir(context.options.offline_bundle, "offline bundle")
    verified_apt_repo = _require_dir(bundle / "apt-repo", "offline apt repository")
    apt_parent = context.options.state_dir / "apt-repos"
    apt_repo = apt_parent / context.options.transaction_id
    for path in (context.options.state_dir, apt_parent, apt_repo):
        if path.is_symlink():
            raise DeploymentError(f"offline apt repository path is a symlink: {path}")
    apt_parent.mkdir(parents=True, exist_ok=True)
    os.chmod(context.options.state_dir, 0o755)
    os.chmod(apt_parent, 0o755)
    if apt_repo.is_dir():
        shutil.rmtree(apt_repo)
    elif apt_repo.exists():
        raise DeploymentError(f"offline apt repository target is unsafe: {apt_repo}")
    _copy_tree_normalized(verified_apt_repo, apt_repo)
    source_list = context.options.state_dir / "offline-apt.sources.list"
    atomic_write(
        source_list,
        f"deb [trusted=yes] {apt_repo.resolve().as_uri()} ./\n",
        mode=0o600,
    )
    apt_options = (
        "-o",
        f"Dir::Etc::sourcelist={source_list}",
        "-o",
        "Dir::Etc::sourceparts=-",
        "-o",
        "Acquire::Languages=none",
        "-o",
        "Acquire::Retries=0",
        "-o",
        "APT::Get::List-Cleanup=0",
    )
    context.runner.run(
        (
            "apt-get",
            *apt_options,
            "update",
        ),
        network_isolated=True,
        timeout=900,
    )
    context.runner.run(
        (
            "apt-get",
            *apt_options,
            "install",
            "-y",
            "--no-install-recommends",
            *missing,
        ),
        network_isolated=True,
        timeout=1800,
    )
    _record_system_package_versions(context)


def _prepare_offline_rustup_home(
    context: "ExecutionContext",
    build_uid: int,
    build_gid: int,
) -> Path:
    source = _require_dir(
        context.options.offline_bundle / "rustup-home",
        "offline rustup home",
    )
    destination = context.options.offline_rustup_home
    if destination.is_symlink() or (destination.exists() and not destination.is_dir()):
        raise DeploymentError(f"offline rustup build copy is unsafe: {destination}")
    source_resolved = source.resolve()
    destination_resolved = destination.resolve(strict=False)
    if (
        source_resolved == destination_resolved
        or source_resolved in destination_resolved.parents
        or destination_resolved in source_resolved.parents
    ):
        raise DeploymentError("offline rustup build copy overlaps the verified bundle")
    staging = Path(tempfile.mkdtemp(prefix=".offline-rustup-home.staging-", dir=destination.parent))
    shutil.rmtree(staging)
    try:
        shutil.copytree(source, staging, symlinks=False)
        for candidate in (staging, *staging.rglob("*")):
            metadata = candidate.lstat()
            if candidate.is_symlink():
                raise DeploymentError(f"offline rustup home contains a symlink: {candidate}")
            os.chown(candidate, build_uid, build_gid)
            if stat.S_ISDIR(metadata.st_mode):
                os.chmod(candidate, 0o750)
            elif stat.S_ISREG(metadata.st_mode):
                os.chmod(candidate, 0o750 if metadata.st_mode & 0o111 else 0o640)
            else:
                raise DeploymentError(f"offline rustup home contains a special file: {candidate}")
        if destination.is_symlink():
            raise DeploymentError(f"offline rustup build copy is unsafe: {destination}")
        if destination.is_dir():
            shutil.rmtree(destination)
        os.replace(staging, destination)
    finally:
        if staging.exists():
            shutil.rmtree(staging, ignore_errors=True)
    return destination


def _validate_rust_proxy(
    context: "ExecutionContext",
    kind: str,
    executable: Path,
    env: Mapping[str, str],
    *,
    network_isolated: bool,
) -> None:
    expected_host = {
        "aarch64": "aarch64-unknown-linux-gnu",
        "x86_64": "x86_64-unknown-linux-gnu",
    }.get(platform.machine().lower(), platform.machine().lower())
    code = r"""
import re
import subprocess
import sys

kind, executable, toolchain, expected_host = sys.argv[1:]
if kind == "rustup":
    command = [executable, "toolchain", "list"]
elif kind == "rustc":
    command = [executable, "+" + toolchain, "-vV"]
else:
    command = [executable, "+" + toolchain, "--version"]
completed = subprocess.run(command, check=False, capture_output=True, text=True)
if completed.returncode != 0:
    raise SystemExit(completed.returncode or 1)
output = completed.stdout.strip()
if kind == "rustup":
    if not any(line.split("-", 1)[0] == toolchain for line in output.splitlines()):
        raise SystemExit("approved rustup toolchain is missing")
elif kind == "rustc":
    release = re.search(r"^release: (\S+)$", output, re.MULTILINE)
    host = re.search(r"^host: (\S+)$", output, re.MULTILINE)
    if release is None or release.group(1) != toolchain:
        raise SystemExit("rustc release does not match profile")
    if host is None or host.group(1) != expected_host:
        raise SystemExit("rustc host does not match deployment architecture")
else:
    version = re.match(r"^cargo (\d+\.\d+\.\d+)(?:\s|$)", output)
    if version is None or version.group(1) != toolchain:
        raise SystemExit("cargo release does not match profile")
"""
    context.runner.run(
        (
            "/usr/bin/python3",
            "-c",
            code,
            kind,
            str(executable),
            context.profile.rust.toolchain,
            expected_host,
        ),
        env=env,
        as_build_user=True,
        network_isolated=network_isolated,
        timeout=60,
    )


def ensure_rust_toolchain(context: "ExecutionContext") -> None:
    toolchain = context.profile.rust.toolchain
    build_account = resolve_build_account()
    build_uid = build_account.pw_uid
    build_gid = build_account.pw_gid
    _prepare_build_root(context, build_gid)
    for build in context.profile.rust.builds:
        target = context.options.rust_target_dir(build.manifest)
        target.parent.mkdir(parents=True, exist_ok=True)
        if target.parent.is_symlink() or not target.parent.is_dir():
            raise DeploymentError(f"Rust target parent is unsafe: {target.parent}")
        os.chown(target.parent, 0, build_gid)
        os.chmod(target.parent, 0o750)
        _prepare_mutable_build_tree(target, build_uid, build_gid, "Rust target directory")
    cargo_home = _prepare_mutable_build_tree(
        context.options.cargo_home,
        build_uid,
        build_gid,
        "Cargo home",
    )
    build_home = _prepare_mutable_build_tree(
        context.options.build_home,
        build_uid,
        build_gid,
        "Build home",
    )
    if context.options.offline_bundle:
        bundle = context.options.offline_bundle
        rustup_home = _prepare_offline_rustup_home(context, build_uid, build_gid)
        env = {
            "CARGO_HOME": str(context.options.cargo_home),
            "CARGO_NET_OFFLINE": "true",
            "HOME": str(build_home),
            "RUSTUP_HOME": str(rustup_home),
            "RUSTUP_TOOLCHAIN": toolchain,
            "RUSTC_WRAPPER": "",
            "RUSTC_WORKSPACE_WRAPPER": "",
            "RUSTFLAGS": "",
        }
        for proxy in ("rustup", "rustc", "cargo"):
            executable = _require_file(bundle / "cargo-bin" / proxy, f"offline {proxy}")
            _validate_rust_proxy(
                context,
                proxy,
                executable,
                env,
                network_isolated=True,
            )
        _freeze_toolchain_tree(rustup_home, build_gid)
        for proxy in ("rustup", "rustc", "cargo"):
            _validate_rust_proxy(
                context,
                proxy,
                _require_file(bundle / "cargo-bin" / proxy, f"offline {proxy}"),
                env,
                network_isolated=True,
            )
        return
    rustup_home = context.options.rustup_home
    env = {
        "CARGO_HOME": str(cargo_home),
        "HOME": str(build_home),
        "RUSTUP_HOME": str(rustup_home),
        "RUSTUP_TOOLCHAIN": toolchain,
        "RUSTC_WRAPPER": "",
        "RUSTC_WORKSPACE_WRAPPER": "",
        "RUSTFLAGS": "",
    }
    toolchain_bin = context.options.toolchain_bin
    if toolchain_bin.is_symlink() or (toolchain_bin.exists() and not toolchain_bin.is_dir()):
        raise DeploymentError(f"immutable toolchain bin is unsafe: {toolchain_bin}")
    if toolchain_bin.is_dir():
        if rustup_home.is_symlink() or not rustup_home.is_dir():
            raise DeploymentError(f"immutable Rustup home is missing or unsafe: {rustup_home}")
        _freeze_toolchain_tree(rustup_home, build_gid)
        _freeze_toolchain_tree(toolchain_bin, build_gid)
        for proxy in ("rustup", "rustc", "cargo"):
            _validate_rust_proxy(
                context,
                proxy,
                _require_file(toolchain_bin / proxy, f"immutable {proxy}"),
                env,
                network_isolated=False,
            )
        return
    rustup_home = _prepare_mutable_build_tree(
        rustup_home,
        build_uid,
        build_gid,
        "Rustup home",
    )
    rustup = cargo_home / "bin" / "rustup"
    if not rustup.is_file():
        bootstrap = context.profile.rust.bootstrap
        url = bootstrap.get("url")
        expected = bootstrap.get("sha256")
        if not isinstance(url, str) or not url.startswith("https://"):
            raise DeploymentError("Rust bootstrap requires a profile-pinned HTTPS URL")
        if not isinstance(expected, str) and not context.options.allow_unverified_third_party:
            raise DeploymentError("Rust bootstrap SHA-256 is unresolved")
        destination = context.options.cache_dir / "downloads" / "rustup-init"
        destination.parent.mkdir(parents=True, exist_ok=True)
        if destination.is_symlink():
            raise DeploymentError(f"Rust bootstrap cache is a symlink: {destination}")
        if not destination.is_file():
            temporary = destination.with_name(f".{destination.name}.{os.getpid()}")
            try:
                with urllib.request.urlopen(url, timeout=120) as response, temporary.open("wb") as handle:
                    shutil.copyfileobj(response, handle)
                    handle.flush()
                    os.fsync(handle.fileno())
                os.replace(temporary, destination)
            except Exception:
                temporary.unlink(missing_ok=True)
                raise
        actual = _sha256(destination)
        if isinstance(expected, str) and actual != expected:
            raise DeploymentError("downloaded rustup-init SHA-256 does not match profile")
        os.chmod(destination, 0o755)
        context.runner.run(
            (str(destination), "-y", "--profile", "minimal", "--default-toolchain", "none"),
            env=env,
            as_build_user=True,
            timeout=900,
        )
    context.runner.run(
        (str(rustup), "toolchain", "install", toolchain, "--profile", "minimal"),
        env=env,
        as_build_user=True,
        timeout=1800,
    )
    for proxy in ("rustup", "rustc", "cargo"):
        _validate_rust_proxy(
            context,
            proxy,
            cargo_home / "bin" / proxy,
            env,
            network_isolated=False,
        )
    staging_bin = Path(tempfile.mkdtemp(prefix=".toolchain-bin.staging-", dir=context.options.build_dir))
    try:
        _copy_rustup_proxies(cargo_home, staging_bin, build_uid)
        _freeze_toolchain_tree(staging_bin, build_gid)
        if toolchain_bin.exists() or toolchain_bin.is_symlink():
            raise DeploymentError(f"immutable toolchain bin target is unsafe: {toolchain_bin}")
        os.replace(staging_bin, toolchain_bin)
    finally:
        if staging_bin.exists():
            shutil.rmtree(staging_bin, ignore_errors=True)
    _freeze_toolchain_tree(rustup_home, build_gid)
    frozen_env = dict(env)
    for proxy in ("rustup", "rustc", "cargo"):
        _validate_rust_proxy(
            context,
            proxy,
            toolchain_bin / proxy,
            frozen_env,
            network_isolated=False,
        )


def _venv_python(context: "ExecutionContext") -> Path:
    return context.options.release_dir / ".venv" / "bin" / "python"


def _build_user_env(context: "ExecutionContext") -> Mapping[str, str]:
    return {
        "HOME": str(context.options.build_home),
        "XDG_CACHE_HOME": str(context.options.build_home / ".cache"),
    }


def _freeze_build_tree(root: Path, build_gid: int, *, public_read: bool, allow_symlinks: bool) -> None:
    _seal_tree(
        root,
        owner_uid=os.geteuid(),
        group_gid=build_gid,
        public_read=public_read,
        allow_symlinks=allow_symlinks,
        label="build output tree",
    )


def _validate_root_managed_python(python_bin: Path) -> Path:
    if not python_bin.is_absolute():
        raise DeploymentError(f"deployment Python must be an absolute path: {python_bin}")

    def validate_chain(path: Path) -> None:
        current = Path(path.anchor)
        for part in path.parts[1:]:
            current /= part
            try:
                metadata = current.lstat()
            except OSError as exc:
                raise DeploymentError(f"deployment Python path is unavailable: {current}") from exc
            if metadata.st_uid != 0:
                raise DeploymentError(f"deployment Python path is not root-owned: {current}")
            if not stat.S_ISLNK(metadata.st_mode) and metadata.st_mode & (stat.S_IWGRP | stat.S_IWOTH):
                raise DeploymentError(f"deployment Python path is writable by an unprivileged account: {current}")

    validate_chain(python_bin)
    try:
        resolved = python_bin.resolve(strict=True)
    except (OSError, RuntimeError) as exc:
        raise DeploymentError(f"deployment Python path is unsafe: {python_bin}") from exc
    validate_chain(resolved)
    metadata = resolved.stat()
    if not stat.S_ISREG(metadata.st_mode) or not metadata.st_mode & 0o111:
        raise DeploymentError(f"deployment Python is not an executable regular file: {resolved}")
    return resolved


def _validate_venv_symlink(relative: Path, target: str, approved_python: Path) -> None:
    name = relative.as_posix()
    if name == "lib64":
        if target != "lib":
            raise DeploymentError(f"virtual environment symlink is not approved: {relative} -> {target}")
        return
    if name not in {"bin/python", "bin/python3"} and re.fullmatch(r"bin/python\d+\.\d+", name) is None:
        raise DeploymentError(f"virtual environment symlink is not approved: {relative} -> {target}")
    if not target or "\x00" in target:
        raise DeploymentError(f"virtual environment symlink has an invalid target: {relative}")
    target_path = Path(target)
    if target_path.is_absolute():
        resolved = _validate_root_managed_python(target_path)
        if resolved != approved_python:
            raise DeploymentError(
                f"virtual environment Python symlink does not use the approved interpreter: {relative} -> {target}"
            )
        return
    normalized = Path(os.path.normpath(str(relative.parent / target_path)))
    if normalized.is_absolute() or ".." in normalized.parts:
        raise DeploymentError(f"virtual environment symlink escapes its root: {relative} -> {target}")
    normalized_name = normalized.as_posix()
    if (
        normalized_name not in {"bin/python", "bin/python3"}
        and re.fullmatch(r"bin/python\d+\.\d+", normalized_name) is None
    ):
        raise DeploymentError(f"virtual environment Python symlink target is not approved: {relative} -> {target}")
    if normalized == relative:
        raise DeploymentError(f"virtual environment Python symlink is self-referential: {relative}")


def _copy_build_snapshot_directory(
    source_descriptor: int,
    destination_descriptor: int,
    relative: Path,
    *,
    build_uid: int,
    owner_uid: int,
    group_gid: int,
    public_read: bool,
    label: str,
    approved_python: Optional[Path],
) -> None:
    source_before = _file_identity(os.fstat(source_descriptor))
    for name in sorted(os.listdir(source_descriptor)):
        if not name or name in {".", ".."} or Path(name).name != name:
            raise DeploymentError(f"{label} contains an invalid entry name: {name!r}")
        entry_relative = relative / name
        source_entry = os.stat(name, dir_fd=source_descriptor, follow_symlinks=False)
        if source_entry.st_uid != build_uid:
            raise DeploymentError(f"{label} entry is not owned by the build account: {entry_relative}")
        if stat.S_ISLNK(source_entry.st_mode):
            if approved_python is None:
                raise DeploymentError(f"{label} contains a symlink: {entry_relative}")
            target = os.readlink(name, dir_fd=source_descriptor)
            _validate_venv_symlink(entry_relative, target, approved_python)
            if _file_identity(os.stat(name, dir_fd=source_descriptor, follow_symlinks=False)) != _file_identity(
                source_entry
            ):
                raise DeploymentError(f"{label} symlink changed while it was copied: {entry_relative}")
            os.symlink(target, name, dir_fd=destination_descriptor)
            os.chown(
                name,
                owner_uid,
                group_gid,
                dir_fd=destination_descriptor,
                follow_symlinks=False,
            )
            continue
        if stat.S_ISDIR(source_entry.st_mode):
            source_child = os.open(
                name,
                os.O_RDONLY | os.O_CLOEXEC | os.O_DIRECTORY | os.O_NOFOLLOW,
                dir_fd=source_descriptor,
            )
            destination_child = -1
            try:
                if _file_identity(os.fstat(source_child)) != _file_identity(source_entry):
                    raise DeploymentError(f"{label} directory changed while it was opened: {entry_relative}")
                os.mkdir(name, 0o700, dir_fd=destination_descriptor)
                destination_child = os.open(
                    name,
                    os.O_RDONLY | os.O_CLOEXEC | os.O_DIRECTORY | os.O_NOFOLLOW,
                    dir_fd=destination_descriptor,
                )
                _copy_build_snapshot_directory(
                    source_child,
                    destination_child,
                    entry_relative,
                    build_uid=build_uid,
                    owner_uid=owner_uid,
                    group_gid=group_gid,
                    public_read=public_read,
                    label=label,
                    approved_python=approved_python,
                )
            finally:
                if destination_child >= 0:
                    os.close(destination_child)
                os.close(source_child)
            continue
        if stat.S_ISREG(source_entry.st_mode):
            source_file = os.open(
                name,
                os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW,
                dir_fd=source_descriptor,
            )
            destination_file = -1
            try:
                if _file_identity(os.fstat(source_file)) != _file_identity(source_entry):
                    raise DeploymentError(f"{label} file changed while it was opened: {entry_relative}")
                destination_file = os.open(
                    name,
                    os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC | os.O_NOFOLLOW,
                    0o600,
                    dir_fd=destination_descriptor,
                )
                with os.fdopen(os.dup(source_file), "rb") as source_handle, os.fdopen(
                    destination_file, "wb"
                ) as destination_handle:
                    destination_file = -1
                    shutil.copyfileobj(source_handle, destination_handle, length=1024 * 1024)
                    destination_handle.flush()
                    os.fchown(destination_handle.fileno(), owner_uid, group_gid)
                    executable = bool(source_entry.st_mode & 0o111)
                    if public_read:
                        mode = 0o555 if executable else 0o444
                    else:
                        mode = 0o550 if executable else 0o440
                    os.fchmod(destination_handle.fileno(), mode)
                    os.fsync(destination_handle.fileno())
                if _file_identity(os.fstat(source_file)) != _file_identity(source_entry):
                    raise DeploymentError(f"{label} file changed while it was copied: {entry_relative}")
            finally:
                if destination_file >= 0:
                    os.close(destination_file)
                os.close(source_file)
            continue
        raise DeploymentError(f"{label} contains a special file: {entry_relative}")
    if _file_identity(os.fstat(source_descriptor)) != source_before:
        raise DeploymentError(f"{label} directory changed while it was copied: {relative or Path('.')}")
    os.fchown(destination_descriptor, owner_uid, group_gid)
    os.fchmod(destination_descriptor, 0o555 if public_read else 0o550)
    os.fsync(destination_descriptor)
    os.fchown(source_descriptor, owner_uid, group_gid)
    os.fchmod(source_descriptor, 0o700)


def _validate_venv_snapshot(root: Path, approved_python: Path) -> None:
    for link in (root / "bin").glob("python*"):
        if not link.is_symlink():
            continue
        try:
            resolved = link.resolve(strict=True)
        except (OSError, RuntimeError) as exc:
            raise DeploymentError(f"virtual environment Python symlink is dangling or cyclic: {link}") from exc
        if resolved != approved_python:
            raise DeploymentError(f"virtual environment Python symlink escaped the approved interpreter: {link}")
    lib64 = root / "lib64"
    if lib64.is_symlink():
        try:
            resolved_lib = lib64.resolve(strict=True)
            expected_lib = (root / "lib").resolve(strict=True)
        except (OSError, RuntimeError) as exc:
            raise DeploymentError("virtual environment lib64 symlink is dangling") from exc
        if resolved_lib != expected_lib:
            raise DeploymentError("virtual environment lib64 symlink escaped its root")


def _publish_build_snapshot(
    root: Path,
    build_uid: int,
    build_gid: int,
    *,
    public_read: bool,
    label: str,
    approved_python: Optional[Path] = None,
) -> None:
    parent_descriptor = _open_directory(root.parent, f"{label} parent")
    owner_uid = os.geteuid()
    parent_metadata = os.fstat(parent_descriptor)
    if parent_metadata.st_uid != owner_uid or parent_metadata.st_mode & (stat.S_IWGRP | stat.S_IWOTH):
        os.close(parent_descriptor)
        raise DeploymentError(f"{label} parent is not privileged-process managed: {root.parent}")
    source_metadata = os.stat(root.name, dir_fd=parent_descriptor, follow_symlinks=False)
    if not stat.S_ISDIR(source_metadata.st_mode) or source_metadata.st_uid != build_uid:
        os.close(parent_descriptor)
        raise DeploymentError(f"{label} is not a build-account-owned directory: {root}")

    approved_resolved = _validate_root_managed_python(approved_python) if approved_python is not None else None
    token = f"{os.getpid()}-{time.time_ns()}"
    stem = root.name.lstrip(".") or "tree"
    quarantine_name = f".{stem}.mutable-{token}"
    staging = Path(tempfile.mkdtemp(prefix=f".{stem}.staging-", dir=root.parent))
    source_descriptor = -1
    destination_descriptor = -1
    published = False
    quarantined = False
    try:
        os.rename(
            root.name,
            quarantine_name,
            src_dir_fd=parent_descriptor,
            dst_dir_fd=parent_descriptor,
        )
        quarantined = True
        os.fsync(parent_descriptor)
        source_descriptor = _open_directory_at(parent_descriptor, quarantine_name, label)
        destination_descriptor = _open_directory(staging, f"{label} staging")
        _copy_build_snapshot_directory(
            source_descriptor,
            destination_descriptor,
            Path(),
            build_uid=build_uid,
            owner_uid=owner_uid,
            group_gid=build_gid,
            public_read=public_read,
            label=label,
            approved_python=approved_resolved,
        )
        os.close(destination_descriptor)
        destination_descriptor = -1
        if approved_resolved is not None:
            _validate_venv_snapshot(staging, approved_resolved)
        os.rename(
            staging.name,
            root.name,
            src_dir_fd=parent_descriptor,
            dst_dir_fd=parent_descriptor,
        )
        published = True
        os.fsync(parent_descriptor)
    finally:
        if destination_descriptor >= 0:
            os.close(destination_descriptor)
        if source_descriptor >= 0:
            os.close(source_descriptor)
        if staging.exists():
            shutil.rmtree(staging, ignore_errors=True)
        quarantine = root.parent / quarantine_name
        if published:
            if quarantine.exists():
                shutil.rmtree(quarantine)
        elif quarantined and not root.exists() and quarantine.exists():
            os.rename(
                quarantine_name,
                root.name,
                src_dir_fd=parent_descriptor,
                dst_dir_fd=parent_descriptor,
            )
            os.fsync(parent_descriptor)
        os.close(parent_descriptor)


def _pip_install(
    context: "ExecutionContext",
    arguments: Sequence[str],
    *,
    timeout: int = 3600,
) -> None:
    context.runner.run(
        (str(_venv_python(context)), "-m", "pip", "install", *arguments),
        cwd=context.options.source_root,
        env=_build_user_env(context),
        as_build_user=True,
        network_isolated=bool(context.options.offline_bundle),
        timeout=timeout,
    )


def _python_install_artifact(
    context: "ExecutionContext",
    distribution: str,
    raw_artifact: object,
) -> Path:
    artifact = Path(raw_artifact)
    expected = context.notes.get(f"artifact_sha256:{distribution}")
    if re.fullmatch(r"[a-z0-9][a-z0-9._-]{0,63}", distribution) is None:
        raise DeploymentError(f"invalid Python source distribution name: {distribution}")
    build_root = context.options.build_dir
    if build_root.is_symlink():
        raise DeploymentError(f"build directory must not be a symlink: {build_root}")
    artifacts_root = build_root / "python-artifacts"
    if artifacts_root.is_symlink():
        raise DeploymentError(f"Python artifact directory must not be a symlink: {artifacts_root}")
    artifacts_root.mkdir(parents=True, exist_ok=True)
    build_gid = resolve_build_account().pw_gid
    os.chown(artifacts_root, os.geteuid(), build_gid)
    os.chmod(artifacts_root, 0o750)
    if artifact.is_file():
        verified = _require_file(artifact, f"{distribution} wheel")
        if verified.suffix != ".whl":
            raise DeploymentError(f"{distribution} artifact must be a wheel")
        wheel_parts = verified.name[:-4].split("-")
        if len(wheel_parts) not in {5, 6} or any(not part for part in wheel_parts):
            raise DeploymentError(f"{distribution} artifact has an invalid wheel filename: {verified.name}")
        actual_distribution = wheel_parts[0].lower().replace(".", "_")
        expected_distribution = distribution.lower().replace("-", "_").replace(".", "_")
        if actual_distribution != expected_distribution:
            raise DeploymentError(f"{distribution} artifact wheel name belongs to {wheel_parts[0]}")
        expected_version = context.notes.get(f"artifact_version:{distribution}")
        if isinstance(expected_version, str) and wheel_parts[1].replace("_", "-") != expected_version.replace("_", "-"):
            raise DeploymentError(f"{distribution} artifact wheel version does not match approval")
        actual = _sha256(verified)
        if isinstance(expected, str) and actual != expected:
            raise DeploymentError(f"{distribution} wheel changed after preflight")
        wheel_root = artifacts_root / "wheels" / distribution
        if wheel_root.is_symlink():
            raise DeploymentError(f"Python wheel snapshot directory is unsafe: {wheel_root}")
        wheel_root.mkdir(parents=True, exist_ok=True)
        os.chown(wheel_root, os.geteuid(), build_gid)
        os.chmod(wheel_root, 0o750)
        destination = wheel_root / verified.name
        if destination.is_symlink() or (destination.exists() and not destination.is_file()):
            raise DeploymentError(f"Python wheel snapshot is unsafe: {destination}")
        fd, temp_name = tempfile.mkstemp(
            prefix=f".{distribution}.",
            suffix=".whl",
            dir=wheel_root,
        )
        temporary = Path(temp_name)
        try:
            with verified.open("rb") as source, os.fdopen(fd, "wb") as output:
                shutil.copyfileobj(source, output)
                output.flush()
                os.fsync(output.fileno())
            if _sha256(temporary) != actual:
                raise DeploymentError(f"{distribution} wheel changed while being snapshotted")
            os.chown(temporary, os.geteuid(), build_gid)
            os.chmod(temporary, 0o440)
            os.replace(temporary, destination)
        finally:
            temporary.unlink(missing_ok=True)
        return destination
    source = _require_dir(artifact, f"{distribution} source")
    actual = _tree_digest(source)
    if isinstance(expected, str) and actual != expected:
        raise DeploymentError(f"{distribution} source changed after preflight")
    sources_root = artifacts_root / "sources"
    if sources_root.is_symlink():
        raise DeploymentError(f"Python source build directory must not be a symlink: {sources_root}")
    sources_root.mkdir(parents=True, exist_ok=True)
    destination = sources_root / distribution
    if destination.is_symlink() or (destination.exists() and not destination.is_dir()):
        raise DeploymentError(f"Python source build copy is unsafe: {destination}")
    staging = Path(tempfile.mkdtemp(prefix=f".{distribution}.staging-", dir=sources_root))
    shutil.rmtree(staging)
    try:
        shutil.copytree(source, staging, symlinks=False)
        if _tree_digest(staging) != actual:
            raise DeploymentError(f"{distribution} source copy failed integrity verification")
        _freeze_build_tree(staging, build_gid, public_read=False, allow_symlinks=False)
        if destination.is_dir():
            shutil.rmtree(destination)
        os.replace(staging, destination)
    finally:
        if staging.exists():
            shutil.rmtree(staging, ignore_errors=True)
    return destination


def _normalized_distribution(line: str) -> str:
    match = re.match(r"^([A-Za-z0-9_.-]+)", line.strip())
    if not match:
        return ""
    return match.group(1).lower().replace("_", "-")


def _filtered_requirement(
    context: "ExecutionContext",
    relative: str,
) -> Path:
    source = context.options.source_root / relative
    denied = {name.lower().replace("_", "-") for name in context.profile.python.deny_distributions}
    rendered = []
    for raw_line in source.read_text(encoding="utf-8").splitlines():
        stripped = raw_line.split("#", 1)[0].strip()
        if stripped.startswith("-") or "://" in stripped or " @ " in stripped:
            raise DeploymentError(f"requirement policy does not allow options, includes, or direct URLs: {source}")
        distribution = _normalized_distribution(stripped)
        if distribution and distribution in denied:
            rendered.append(f"# installer-filtered denied distribution: {raw_line}")
        else:
            rendered.append(raw_line)
    output = context.options.build_dir / "requirements" / Path(relative).name
    output.parent.mkdir(parents=True, exist_ok=True)
    atomic_write(output, "\n".join(rendered).rstrip("\n") + "\n", mode=0o644)
    return output


def _python_requirement_paths(context: "ExecutionContext") -> Tuple[str, ...]:
    if context.profile.python.lock_status == "hashed":
        target = context.profile.metadata.get("lock_target")
        if not isinstance(target, str) or not target:
            raise DeploymentError("hashed Python profile requires metadata.lock_target")
        return (target,)
    return context.profile.python.requirements


def _validate_hashed_requirement_policy(context: "ExecutionContext", path: Path) -> None:
    if context.profile.python.lock_status != "hashed":
        return
    denied = {name.lower().replace("_", "-") for name in context.profile.python.deny_distributions}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        stripped = raw_line.split("#", 1)[0].strip()
        if not stripped or stripped == "\\":
            continue
        if stripped.startswith(("-r", "--requirement", "-c", "--constraint")):
            raise DeploymentError(f"hashed Python lock contains a nested requirement include: {path}")
        if stripped.startswith("--hash="):
            hash_tokens = stripped.rstrip("\\").split()
            if not all(re.fullmatch(r"--hash=sha256:[0-9a-f]{64}", token) for token in hash_tokens):
                raise DeploymentError(f"hashed Python lock contains an invalid hash option: {path}")
            continue
        if stripped.startswith("-"):
            raise DeploymentError(f"hashed Python lock contains an unsupported pip option: {path}")
        if "://" in stripped or " @ " in stripped:
            raise DeploymentError(f"hashed Python lock contains a direct URL requirement: {path}")
        distribution = _normalized_distribution(stripped)
        if not distribution:
            raise DeploymentError(f"hashed Python lock contains an invalid requirement: {path}")
        if distribution in denied:
            raise DeploymentError(f"hashed Python lock contains index-forbidden distribution {distribution}: {path}")


def install_python_environment(context: "ExecutionContext") -> None:
    options = context.options
    existing_manifest = options.release_dir / "release-manifest.json"
    if existing_manifest.is_file():
        try:
            existing = json.loads(existing_manifest.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise DeploymentError(f"existing release manifest is invalid: {existing_manifest}") from exc
        if existing.get("fingerprint") != context.plan.fingerprint:
            raise DeploymentError(f"existing release has a different fingerprint: {options.release_dir}")
        context.notes["reuse_release"] = True
        return
    build_account = resolve_build_account()
    _prepare_build_root(context, build_account.pw_gid)
    _prepare_release_root(context)
    installing_marker = options.release_dir / ".installing"
    if installing_marker.is_symlink():
        raise DeploymentError(f"incomplete release marker is a symlink: {installing_marker}")
    if installing_marker.is_file():
        marker_fingerprint = installing_marker.read_text(encoding="utf-8").strip()
        if marker_fingerprint != context.plan.fingerprint:
            raise DeploymentError(f"incomplete release belongs to a different deployment: {options.release_dir}")
    elif installing_marker.exists():
        raise DeploymentError(f"incomplete release marker is unsafe: {installing_marker}")
    elif any(options.release_dir.iterdir()):
        raise DeploymentError(f"unmarked incomplete release directory is not reusable: {options.release_dir}")
    else:
        atomic_write(installing_marker, context.plan.fingerprint + "\n", mode=0o600)
    venv_root = _prepare_mutable_build_tree(
        options.release_dir / ".venv",
        build_account.pw_uid,
        build_account.pw_gid,
        "candidate Python virtual environment",
    )
    venv_python = _venv_python(context)
    if not venv_python.is_file():
        context.runner.run(
            (
                options.python_bin,
                "-m",
                "venv",
                "--system-site-packages",
                str(venv_root),
            ),
            env=_build_user_env(context),
            as_build_user=True,
            timeout=300,
        )
    pip_common = ["--disable-pip-version-check", "--isolated", "--no-input"]
    if context.profile.production or options.offline_bundle:
        pip_common.append("--only-binary=:all:")
    if options.offline_bundle:
        pip_common.extend(
            (
                "--no-index",
                "--find-links",
                str(options.offline_bundle / "wheelhouse"),
            )
        )
    constraints = []
    if context.profile.python.constraints and context.profile.python.lock_status != "hashed":
        constraints = ["--constraint", str(options.source_root / context.profile.python.constraints)]
    vendor_pins = []
    for package in context.profile.python.vendor_packages:
        distribution = str(package.get("distribution") or "")
        version = context.notes.get(f"vendor:{distribution}")
        if package.get("required") is True and distribution and isinstance(version, str):
            vendor_pins.append(f"{distribution}=={version.strip()}")
    if vendor_pins:
        vendor_constraints = options.build_dir / "requirements" / "vendor-system-constraints.txt"
        atomic_write(vendor_constraints, "\n".join(vendor_pins) + "\n", mode=0o644)
        constraints.extend(("--constraint", str(vendor_constraints)))
    for relative in _python_requirement_paths(context):
        requirement = (
            options.source_root / relative
            if context.profile.python.lock_status == "hashed"
            else _filtered_requirement(context, relative)
        )
        _validate_hashed_requirement_policy(context, requirement)
        requirement_flags = list(pip_common)
        if context.profile.python.lock_status == "hashed":
            requirement_flags.append("--require-hashes")
        _pip_install(
            context,
            (*requirement_flags, *constraints, "--requirement", str(requirement)),
        )
    if context.profile.robot == "go2":
        clip_artifact = context.notes.get("clip_artifact")
        if clip_artifact is not None:
            clip_install = _python_install_artifact(context, "clip", clip_artifact)
            _pip_install(context, (*pip_common, "--no-deps", str(clip_install)))
        cyclone_artifact = context.notes.get("cyclonedds_artifact")
        if cyclone_artifact is not None:
            cyclone_install = _python_install_artifact(
                context,
                "cyclonedds",
                cyclone_artifact,
            )
            _pip_install(context, (*pip_common, "--no-deps", str(cyclone_install)))
        sdk_install = _python_install_artifact(
            context,
            "unitree-sdk2py",
            context.notes["go2_sdk_artifact"],
        )
        context.notes["runtime_artifact:unitree-sdk2py"] = sdk_install
        _pip_install(
            context,
            (*pip_common, "--no-deps", str(sdk_install)),
        )
    context.runner.run(
        (str(venv_python), "-m", "pip", "check"),
        env=_build_user_env(context),
        as_build_user=True,
        network_isolated=bool(options.offline_bundle),
        timeout=120,
    )
    for package in context.profile.python.vendor_packages:
        distribution = str(package.get("distribution") or "")
        expected = context.notes.get(f"vendor:{distribution}")
        if package.get("required") is not True or not distribution or not isinstance(expected, str):
            continue
        code = (
            "import importlib.metadata; "
            f"actual=importlib.metadata.version({distribution!r}); "
            f"expected={expected.strip()!r}; "
            "assert actual == expected, '%s changed from %s to %s' % "
            f"({distribution!r}, expected, actual)"
        )
        context.runner.run(
            (str(venv_python), "-c", code),
            env=_build_user_env(context),
            as_build_user=True,
            network_isolated=bool(options.offline_bundle),
            timeout=60,
        )
    imports = list(context.profile.python.required_imports)
    if context.profile.robot == "go2":
        imports.extend(("clip", "cyclonedds", "unitree_sdk2py.core.channel"))
    for vendor in context.profile.python.vendor_packages:
        if vendor.get("required") is not True:
            continue
        modules = vendor.get("import_modules")
        if isinstance(modules, list):
            imports.extend(str(module) for module in modules if module)
    if not imports:
        imports.extend(("fastapi", "grpc", "google.protobuf", "cv2", "lcm"))
    code = "; ".join(f"import {name}" for name in dict.fromkeys(imports))
    context.runner.run(
        (str(venv_python), "-c", code),
        env=_build_user_env(context),
        as_build_user=True,
        network_isolated=bool(options.offline_bundle),
        timeout=120,
    )
    if context.profile.models.get("prepare_yolo") and not options.skip_models:
        context.runner.run(
            (
                str(venv_python),
                "-c",
                "import torch, torchvision; assert torch.cuda.is_available(), "
                "'Jetson CUDA torch is required; generic PyPI torch is forbidden'",
            ),
            env=_build_user_env(context),
            as_build_user=True,
            network_isolated=bool(options.offline_bundle),
            timeout=120,
        )
    _publish_build_snapshot(
        venv_root,
        build_account.pw_uid,
        build_account.pw_gid,
        public_read=True,
        label="candidate Python virtual environment",
        approved_python=Path(options.python_bin),
    )


def _materialize_verified_model(
    url: str,
    expected_sha256: str,
    destination: Path,
    build_gid: int,
) -> None:
    if not url.startswith("https://"):
        raise DeploymentError("model artifact URL must use HTTPS")
    if re.fullmatch(r"[0-9a-f]{64}", expected_sha256) is None:
        raise DeploymentError("model artifact SHA-256 is invalid")
    destination.parent.mkdir(parents=True, exist_ok=True)
    parent_descriptor = _open_directory(destination.parent, "verified model cache")
    parent_metadata = os.fstat(parent_descriptor)
    if parent_metadata.st_uid != os.geteuid() or parent_metadata.st_mode & (stat.S_IWGRP | stat.S_IWOTH):
        os.close(parent_descriptor)
        raise DeploymentError(f"verified model cache is not privileged-process managed: {destination.parent}")
    if destination.exists() or destination.is_symlink():
        existing_descriptor = -1
        reusable_model = False
        try:
            existing_descriptor = os.open(
                destination.name,
                os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW,
                dir_fd=parent_descriptor,
            )
            metadata = os.fstat(existing_descriptor)
            if not stat.S_ISREG(metadata.st_mode):
                raise DeploymentError(f"cached model artifact is unsafe: {destination}")
            reusable = (
                metadata.st_uid == os.geteuid()
                and metadata.st_gid == build_gid
                and metadata.st_nlink == 1
                and not metadata.st_mode & (stat.S_IWGRP | stat.S_IWOTH)
            )
            if reusable:
                digest = hashlib.sha256()
                while True:
                    chunk = os.read(existing_descriptor, 1024 * 1024)
                    if not chunk:
                        break
                    digest.update(chunk)
                if digest.hexdigest() == expected_sha256:
                    reusable_model = True
        except OSError as exc:
            os.close(parent_descriptor)
            raise DeploymentError(f"cached model artifact is unsafe: {destination}") from exc
        except DeploymentError:
            os.close(parent_descriptor)
            raise
        finally:
            if existing_descriptor >= 0:
                os.close(existing_descriptor)
        if reusable_model:
            os.close(parent_descriptor)
            return
        current = os.stat(destination.name, dir_fd=parent_descriptor, follow_symlinks=False)
        if (current.st_dev, current.st_ino) != (metadata.st_dev, metadata.st_ino):
            os.close(parent_descriptor)
            raise DeploymentError(f"cached model artifact changed during validation: {destination}")
        os.unlink(destination.name, dir_fd=parent_descriptor)
        os.fsync(parent_descriptor)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{destination.name}.", dir=destination.parent)
    temporary = Path(temporary_name)
    try:
        digest = hashlib.sha256()
        with urllib.request.urlopen(url, timeout=300) as response, os.fdopen(descriptor, "wb") as output:
            descriptor = -1
            while True:
                chunk = response.read(1024 * 1024)
                if not chunk:
                    break
                output.write(chunk)
                digest.update(chunk)
            output.flush()
            os.fchown(output.fileno(), os.geteuid(), build_gid)
            os.fchmod(output.fileno(), 0o440)
            os.fsync(output.fileno())
        if digest.hexdigest() != expected_sha256:
            raise DeploymentError("downloaded model SHA-256 does not match profile")
        os.replace(
            temporary.name,
            destination.name,
            src_dir_fd=parent_descriptor,
            dst_dir_fd=parent_descriptor,
        )
        os.fsync(parent_descriptor)
    finally:
        if descriptor >= 0:
            os.close(descriptor)
        temporary.unlink(missing_ok=True)
        os.close(parent_descriptor)


def prepare_assets(context: "ExecutionContext") -> None:
    options = context.options
    if options.skip_models:
        return
    models = context.profile.models
    build_account = resolve_build_account()
    _prepare_build_root(context, build_account.pw_gid)
    output = _prepare_mutable_build_tree(
        options.build_dir / "models",
        build_account.pw_uid,
        build_account.pw_gid,
        "model build output",
    )
    if options.offline_bundle:
        bundled_models = options.offline_bundle / "models"
        if bundled_models.is_dir():
            context.runner.run(
                (
                    "/bin/cp",
                    "-R",
                    "--no-preserve=mode,ownership,timestamps",
                    str(bundled_models) + "/.",
                    str(output),
                ),
                env=_build_user_env(context),
                as_build_user=True,
                timeout=1800,
            )
        elif models.get("prepare_yolo") or models.get("prepare_memory"):
            raise DeploymentError("offline bundle is missing required models/")
    if models.get("prepare_yolo"):
        weights = output / "yolov8s-worldv2.pt"
        engine = output / "yolov8s-worldv2.engine"
        weight_contract = models.get("yolo_weights")
        strict_model = context.profile.production
        if strict_model and (
            not isinstance(weight_contract, dict)
            or not isinstance(weight_contract.get("url"), str)
            or not isinstance(weight_contract.get("sha256"), str)
        ):
            raise DeploymentError("production YOLO weights URL/SHA-256 is unresolved")
        if isinstance(weight_contract, dict) and isinstance(weight_contract.get("sha256"), str):
            if options.offline_bundle:
                if not weights.is_file():
                    raise DeploymentError("offline YOLO weights do not match the production hash")
            else:
                cache_root = _prepare_root_managed_cache(options.cache_dir, build_account.pw_gid)
                verified_models = cache_root / "verified-models"
                if verified_models.is_symlink() or (verified_models.exists() and not verified_models.is_dir()):
                    raise DeploymentError(f"verified model cache is unsafe: {verified_models}")
                verified_models.mkdir(parents=True, exist_ok=True)
                os.chown(verified_models, os.geteuid(), build_account.pw_gid)
                os.chmod(verified_models, 0o750)
                cached_weights = verified_models / f"{weight_contract['sha256']}.pt"
                _materialize_verified_model(
                    str(weight_contract.get("url") or ""),
                    weight_contract["sha256"],
                    cached_weights,
                    build_account.pw_gid,
                )
                context.runner.run(
                    ("/usr/bin/install", "-m", "0640", str(cached_weights), str(weights)),
                    env=_build_user_env(context),
                    as_build_user=True,
                    timeout=300,
                )
        export_engine = options.export_yolo_engine or bool(models.get("export_yolo_engine_default"))
        needs_prepare = not weights.is_file() or (export_engine and not engine.is_file())
        if options.offline_bundle:
            required = engine if export_engine else weights
            if not required.is_file():
                raise DeploymentError(f"offline bundle is missing required model: {required.name}")
            needs_prepare = False
        if needs_prepare:
            command = [
                str(_venv_python(context)),
                "scripts/prepare_yolo_world_tensorrt.py",
                "--weights-output",
                str(weights),
                "--engine-output",
                str(engine),
            ]
            command.append("--no-int8" if export_engine else "--download-only")
            context.runner.run(
                command,
                cwd=options.source_root,
                env=_build_user_env(context),
                as_build_user=True,
                timeout=7200,
            )
    if models.get("prepare_memory") and not options.offline_bundle:
        context.runner.run(
            (str(_venv_python(context)), "scripts/prepare_memory_models.py"),
            cwd=options.source_root,
            env={**_build_user_env(context), "NUWAX_MODEL_ROOT": str(output)},
            as_build_user=True,
            timeout=7200,
        )
    _publish_build_snapshot(
        output,
        build_account.pw_uid,
        build_account.pw_gid,
        public_read=False,
        label="model build output",
    )
    if (
        models.get("prepare_yolo")
        and isinstance(models.get("yolo_weights"), dict)
        and isinstance(models["yolo_weights"].get("sha256"), str)
    ):
        weights = output / "yolov8s-worldv2.pt"
        if not weights.is_file() or _sha256(weights) != models["yolo_weights"]["sha256"]:
            raise DeploymentError("prepared YOLO weights do not match the production hash")


def _copy_ignore(_directory: str, names: Sequence[str]):
    ignored = {
        ".git",
        ".venv",
        ".omx",
        ".omc",
        ".codex",
        ".claude",
        ".pytest_cache",
        ".ruff_cache",
        "__pycache__",
        "runtime_data",
        "target",
        "tests",
        "docs",
        "openspec",
    }
    return [
        name
        for name in names
        if name in ignored
        or name == ".env"
        or name.startswith(".env.")
        or name == "data_plane.env"
        or name.startswith("data_plane.env.")
        or name.endswith((".pyc", ".log"))
    ]


def _copy_regular_file_normalized(source: Path, destination: Path) -> None:
    metadata = source.lstat()
    if source.is_symlink() or not stat.S_ISREG(metadata.st_mode):
        raise DeploymentError(f"release source contains a link or special file: {source}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    os.chmod(destination.parent, 0o755)
    shutil.copyfile(source, destination)
    os.chmod(destination, 0o755 if metadata.st_mode & stat.S_IXUSR else 0o644)


def _copy_tree_normalized(
    source_root: Path,
    destination_root: Path,
    *,
    ignore=None,
) -> None:
    if source_root.is_symlink() or not source_root.is_dir():
        raise DeploymentError(f"release source directory is unsafe: {source_root}")
    for root, directories, files in os.walk(source_root, followlinks=False):
        root_path = Path(root)
        names = tuple(directories) + tuple(files)
        ignored = set(ignore(str(root_path), names) if ignore is not None else ())
        kept_directories = []
        for name in directories:
            if name in ignored:
                continue
            directory = root_path / name
            metadata = directory.lstat()
            if directory.is_symlink() or not stat.S_ISDIR(metadata.st_mode):
                raise DeploymentError(f"release source contains a link or special directory: {directory}")
            kept_directories.append(name)
        directories[:] = kept_directories
        relative_root = root_path.relative_to(source_root)
        destination_directory = destination_root / relative_root
        destination_directory.mkdir(parents=True, exist_ok=True)
        os.chmod(destination_directory, 0o755)
        for name in files:
            if name in ignored:
                continue
            _copy_regular_file_normalized(
                root_path / name,
                destination_directory / name,
            )


def _copy_release_source(source_root: Path, release_dir: Path) -> None:
    tracked = subprocess.run(
        source_user_command(("git", "ls-files", "-z", "--cached")),
        cwd=str(source_root),
        env=source_user_environment(),
        capture_output=True,
        check=False,
        timeout=60,
    )
    if tracked.returncode == 0 and tracked.stdout:
        for raw_relative in tracked.stdout.split(b"\0"):
            if not raw_relative:
                continue
            relative = Path(os.fsdecode(raw_relative))
            if relative.is_absolute() or ".." in relative.parts:
                raise DeploymentError(f"tracked source path is unsafe: {relative}")
            source = source_root / relative
            if source.is_symlink():
                raise DeploymentError(f"release source must not contain symlinks: {relative}")
            if not source.is_file():
                continue
            destination = release_dir / relative
            _copy_regular_file_normalized(source, destination)
        return
    if not source_root.is_dir():
        raise DeploymentError(f"release source directory is missing: {source_root}")
    _copy_tree_normalized(
        source_root,
        release_dir,
        ignore=_copy_ignore,
    )


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _source_revision(context: "ExecutionContext") -> str:
    if context.options.source_revision:
        return context.options.source_revision
    bundle_manifest = context.notes.get("bundle_manifest")
    if isinstance(bundle_manifest, dict):
        metadata = bundle_manifest.get("metadata")
        if isinstance(metadata, dict) and isinstance(metadata.get("source_revision"), str):
            return metadata["source_revision"]
    completed = subprocess.run(
        source_user_command(("git", "rev-parse", "HEAD")),
        cwd=str(context.options.source_root),
        env=source_user_environment(),
        capture_output=True,
        text=True,
        check=False,
        timeout=30,
    )
    return completed.stdout.strip() if completed.returncode == 0 else "signed-source-snapshot"


def _hash_existing_files(source_root: Path, paths: Sequence[str]) -> Mapping[str, str]:
    hashes = {}
    for relative in dict.fromkeys(paths):
        path = source_root / relative
        if path.is_file():
            hashes[relative] = _sha256(path)
    return hashes


def _capture_as_build_user(argv: Sequence[str], *, timeout: int) -> subprocess.CompletedProcess:
    account = resolve_build_account()
    command = list(argv)
    if os.geteuid() == 0:
        command = ["runuser", "--preserve-environment", "-u", account.pw_name, "--", *command]
    environment = {
        "HOME": account.pw_dir,
        "USER": account.pw_name,
        "LOGNAME": account.pw_name,
        "LANG": "C.UTF-8",
        "LC_ALL": "C.UTF-8",
        "PATH": "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin",
    }
    return subprocess.run(
        command,
        env=environment,
        capture_output=True,
        text=True,
        check=False,
        timeout=timeout,
    )


def _validate_release_binary(path: Path) -> str:
    description = _capture_as_build_user(("/usr/bin/file", "-b", str(path)), timeout=30)
    if description.returncode != 0:
        raise DeploymentError(f"cannot inspect Rust release binary: {path}")
    actual_arch = platform.machine().lower()
    expected_tokens = {
        "aarch64": ("aarch64", "arm64"),
        "x86_64": ("x86-64", "x86_64"),
    }.get(actual_arch, (actual_arch,))
    normalized = description.stdout.lower()
    if not any(token.lower() in normalized for token in expected_tokens):
        raise DeploymentError(
            f"Rust release binary architecture mismatch for {path.name}: {description.stdout.strip()}"
        )
    dynamic = _capture_as_build_user(("/usr/bin/ldd", str(path)), timeout=30)
    linkage = dynamic.stdout + dynamic.stderr
    if "not found" in linkage:
        raise DeploymentError(f"Rust release binary has unresolved shared libraries: {path.name}")
    if path.name == "nuwax-nav-stack":
        contract = _capture_as_build_user((str(path), "--launcher-contract"), timeout=5)
        expected = "nuwax-nav-stack-internal-ready-v1"
        if contract.returncode != 0 or contract.stdout.strip() != expected:
            raise DeploymentError(
                "Rust release binary does not support the nav launcher readiness contract: "
                f"{path.name}"
            )
    return description.stdout.strip()


def _release_source_digest(release_dir: Path) -> str:
    digest = hashlib.sha256()
    excluded_dirs = {".venv", "bin", "models", "vendor", "__pycache__"}
    for root, directories, files in os.walk(release_dir):
        directories[:] = sorted(name for name in directories if name not in excluded_dirs)
        root_path = Path(root)
        for name in sorted(files):
            path = root_path / name
            relative = path.relative_to(release_dir)
            if relative.as_posix() in {"release-manifest.json", ".installing"}:
                continue
            if path.is_symlink():
                raise DeploymentError(f"release source contains a symlink: {relative}")
            digest.update(relative.as_posix().encode("utf-8"))
            digest.update(b"\0")
            digest.update(_sha256(path).encode("ascii"))
            digest.update(b"\0")
    return digest.hexdigest()


def _immutable_tree_digest(root: Path, label: str) -> str:
    if root.is_symlink() or not root.is_dir():
        raise DeploymentError(f"{label} root is unsafe: {root}")
    digest = hashlib.sha256()

    def record(path: Path, kind: bytes, payload: bytes = b"") -> None:
        relative = path.relative_to(root).as_posix().encode("utf-8")
        metadata = path.lstat()
        digest.update(kind)
        digest.update(b"\0")
        digest.update(relative)
        digest.update(b"\0")
        digest.update(f"{stat.S_IMODE(metadata.st_mode):04o}".encode("ascii"))
        digest.update(b"\0")
        digest.update(payload)
        digest.update(b"\0")

    for current, directories, files in os.walk(root, followlinks=False):
        current_path = Path(current)
        kept_directories = []
        for name in sorted(directories):
            candidate = current_path / name
            metadata = candidate.lstat()
            if candidate.is_symlink():
                record(candidate, b"L", os.readlink(candidate).encode("utf-8"))
            elif stat.S_ISDIR(metadata.st_mode):
                record(candidate, b"D")
                kept_directories.append(name)
            else:
                raise DeploymentError(f"{label} contains a special directory entry: {candidate}")
        directories[:] = kept_directories
        for name in sorted(files):
            candidate = current_path / name
            metadata = candidate.lstat()
            if candidate.is_symlink():
                record(candidate, b"L", os.readlink(candidate).encode("utf-8"))
            elif stat.S_ISREG(metadata.st_mode):
                record(candidate, b"F", _sha256(candidate).encode("ascii"))
            else:
                raise DeploymentError(f"{label} contains a special file: {candidate}")
    return digest.hexdigest()


def _venv_inventory_digest(release_dir: Path) -> str:
    venv = release_dir / ".venv"
    python = venv / "bin" / "python"
    if not python.is_file():
        raise DeploymentError(f"release virtual environment is missing: {python}")
    return _immutable_tree_digest(venv, "release virtual environment")


def _validate_completed_release(release_dir: Path, manifest: Mapping[str, object]) -> None:
    if manifest.get("schema_version") != 2:
        raise DeploymentError(f"unsupported completed release manifest: {release_dir}")
    binaries = manifest.get("binary_sha256")
    if not isinstance(binaries, dict) or not binaries:
        raise DeploymentError("completed release has no binary integrity records")
    for name, expected in binaries.items():
        if not isinstance(name, str) or not isinstance(expected, str):
            raise DeploymentError("invalid release binary integrity record")
        binary = (
            release_dir
            / "services"
            / "geometry"
            / "nuwax_geometry_runtime"
            / "target"
            / "release"
            / name
            if name == "nuwax-geometry-runtime"
            else release_dir / "bin" / name
        )
        _require_file(binary, f"release binary {name}")
        if _sha256(binary) != expected:
            raise DeploymentError(f"completed release binary checksum mismatch: {name}")
    if manifest.get("source_tree_sha256") != _release_source_digest(release_dir):
        raise DeploymentError("completed release source checksum mismatch")
    if manifest.get("venv_inventory_sha256") != _venv_inventory_digest(release_dir):
        raise DeploymentError("completed release Python environment drifted")
    vendor_hashes = manifest.get("vendor_tree_sha256")
    if not isinstance(vendor_hashes, dict):
        raise DeploymentError("completed release vendor integrity records are invalid")
    for relative, expected in vendor_hashes.items():
        normalized = Path(relative) if isinstance(relative, str) else Path("..")
        if not isinstance(expected, str) or normalized.is_absolute() or ".." in normalized.parts:
            raise DeploymentError("completed release vendor integrity record is invalid")
        if _tree_digest(_require_dir(release_dir / "vendor" / normalized, "release vendor")) != expected:
            raise DeploymentError(f"completed release vendor checksum mismatch: {relative}")
    model_hashes = manifest.get("model_sha256")
    if not isinstance(model_hashes, dict):
        raise DeploymentError("completed release model integrity records are invalid")
    actual_models = (
        {
            path.relative_to(release_dir / "models").as_posix()
            for path in (release_dir / "models").rglob("*")
            if path.is_file() and not path.is_symlink()
        }
        if (release_dir / "models").is_dir()
        else set()
    )
    if actual_models != set(model_hashes):
        raise DeploymentError("completed release model file set mismatch")
    for relative, expected in model_hashes.items():
        if not isinstance(relative, str) or not isinstance(expected, str):
            raise DeploymentError("completed release model integrity record is invalid")
        normalized = Path(relative)
        if normalized.is_absolute() or ".." in normalized.parts:
            raise DeploymentError("completed release model path is invalid")
        model = release_dir / "models" / normalized
        if _sha256(_require_file(model, f"release model {relative}")) != expected:
            raise DeploymentError(f"completed release model checksum mismatch: {relative}")


def _validate_managed_release(
    context: "ExecutionContext",
    release: Path,
    label: str,
) -> Path:
    release = release.resolve()
    if release.parent != context.options.releases_dir.resolve():
        raise DeploymentError(f"{label} escapes the managed releases directory")
    manifest_path = _require_file(
        release / "release-manifest.json",
        f"{label} release manifest",
    )
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise DeploymentError(f"{label} release manifest is invalid") from exc
    if manifest.get("profile") != context.profile.name or manifest.get("release_id") != release.name:
        raise DeploymentError(f"{label} release identity is invalid")
    _validate_completed_release(release, manifest)
    if not context.options.no_systemd:
        _require_file(
            release / "scripts" / "install_motion_guard_deployment.sh",
            f"{label} systemd installer",
        )
    return release


def _switch_current(context: "ExecutionContext", release_dir: Path) -> None:
    current = context.options.current_link
    current.parent.mkdir(parents=True, exist_ok=True)
    if current.is_symlink():
        previous_target = Path(os.readlink(current))
        if not previous_target.is_absolute():
            previous_target = (current.parent / previous_target).resolve()
        if current.resolve() == release_dir.resolve():
            _record_recovery(context, release_dir, activated=False)
            context.notes["current_unchanged"] = True
            return
        context.previous_release = str(_validate_managed_release(context, previous_target, "current rollback baseline"))
    elif current.exists():
        raise DeploymentError(f"current release path exists and is not a symlink: {current}")
    _record_recovery(context, release_dir, activated=False)
    temp_link = current.with_name(f".{current.name}.{os.getpid()}")
    try:
        temp_link.unlink()
    except FileNotFoundError:
        pass
    os.symlink(str(release_dir), temp_link)
    os.replace(temp_link, current)
    fsync_directory(current.parent)
    context.release_switched = True
    _record_recovery(context, release_dir, activated=True)


def _transaction_backup_dir(context: "ExecutionContext") -> Path:
    transaction_id = context.options.transaction_id
    if not transaction_id:
        raise DeploymentError("deployment transaction_id is required for rollback state")
    return context.options.state_dir / "backups" / transaction_id


def _systemd_backup_dir(context: "ExecutionContext") -> Path:
    return _transaction_backup_dir(context) / "systemd"


def _prepare_backup_root(context: "ExecutionContext") -> Path:
    root = _transaction_backup_dir(context)
    for path in (context.options.state_dir, context.options.state_dir / "backups", root):
        if path.is_symlink():
            raise DeploymentError(f"deployment backup path must not be a symlink: {path}")
    root.mkdir(parents=True, exist_ok=True)
    os.chmod(root, 0o700)
    return root


def _record_recovery(
    context: "ExecutionContext",
    release_dir: Path,
    *,
    activated: bool,
) -> None:
    current = context.options.current_link
    release_identity = str(release_dir.resolve())
    recovery_path = _transaction_backup_dir(context) / "recovery.json"
    existing = {}
    if recovery_path.is_file():
        try:
            existing = json.loads(recovery_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise DeploymentError(f"deployment recovery record is invalid: {recovery_path}") from exc
        if not isinstance(existing, dict):
            raise DeploymentError(f"deployment recovery record is invalid: {recovery_path}")
        existing_candidate = existing.get("candidate_release")
        if not isinstance(existing_candidate, str) or Path(existing_candidate).resolve() != release_dir.resolve():
            raise DeploymentError("deployment recovery candidate changed within transaction")

    current_target = None
    if current.is_symlink():
        previous_target = Path(os.readlink(current))
        if not previous_target.is_absolute():
            previous_target = (current.parent / previous_target).resolve()
        current_target = str(previous_target.resolve())
    elif current.exists():
        raise DeploymentError(f"current release path exists and is not a symlink: {current}")

    if existing.get("schema_version") == 2:
        current_at_start = existing.get("current_at_start")
        same_current = existing.get("same_current")
        previous = existing.get("previous_release")
    elif existing.get("schema_version") == 1:
        previous = existing.get("previous_release")
        current_at_start = previous
        if current_at_start is None and existing.get("activated") is False:
            current_at_start = current_target
        same_current = current_at_start == release_identity and previous is None
    else:
        current_at_start = current_target
        same_current = current_target == release_identity
        previous = context.previous_release
    if not isinstance(same_current, bool):
        raise DeploymentError("deployment recovery same-current state is invalid")
    context.notes["same_current_recovery"] = same_current
    if current_at_start is not None and not isinstance(current_at_start, str):
        raise DeploymentError("deployment recovery start target is invalid")
    if previous is None and current_at_start is not None and not same_current:
        previous = current_at_start
    if previous is not None and not isinstance(previous, str):
        raise DeploymentError("deployment recovery previous release is invalid")
    if previous is not None:
        context.previous_release = previous
    recovery = {
        "schema_version": 2,
        "current_link": str(current),
        "candidate_release": release_identity,
        "previous_release": previous,
        "current_at_start": current_at_start,
        "same_current": same_current,
        "activation_state": "activated" if activated else "prepared",
    }
    _prepare_backup_root(context)
    atomic_write(
        recovery_path,
        json.dumps(recovery, indent=2, sort_keys=True) + "\n",
        mode=0o600,
    )


def _mark_recovery_rolling_back(context: "ExecutionContext") -> None:
    recovery_path = _transaction_backup_dir(context) / "recovery.json"
    if not recovery_path.is_file():
        return
    try:
        recovery = json.loads(recovery_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise DeploymentError(f"deployment recovery record is invalid: {recovery_path}") from exc
    if recovery.get("schema_version") != 2:
        return
    if recovery.get("activation_state") not in {"prepared", "activated", "rolling_back"}:
        raise DeploymentError("deployment recovery activation state is invalid")
    recovery["activation_state"] = "rolling_back"
    atomic_write(
        recovery_path,
        json.dumps(recovery, indent=2, sort_keys=True) + "\n",
        mode=0o600,
    )


def _install_geometry_release(context: "ExecutionContext", release_dir: Path) -> None:
    contract = context.profile.metadata.get("geometry_release")
    if not isinstance(contract, dict):
        return
    installed_manifest = contract.get("installed_manifest")
    if not isinstance(installed_manifest, str):
        raise DeploymentError("geometry_release.installed_manifest is required")
    manifest_relative = PurePosixPath(installed_manifest)
    if (
        manifest_relative.is_absolute()
        or manifest_relative.name != "manifest.json"
        or any(part in {"", ".", ".."} for part in manifest_relative.parts)
    ):
        raise DeploymentError("geometry_release.installed_manifest must be a safe manifest path")

    build_parent = context.options.build_dir / "geometry-release"
    build_active = build_parent / "nuwax-geometry-release"
    try:
        active_metadata = build_active.lstat()
    except OSError as exc:
        raise DeploymentError("verified Geometry release build output is missing") from exc
    if not stat.S_ISLNK(active_metadata.st_mode):
        raise DeploymentError("verified Geometry release active path must be a symlink")
    active_target = PurePosixPath(os.readlink(build_active))
    if (
        active_target.is_absolute()
        or len(active_target.parts) != 2
        or active_target.parts[0] != "nuwax-geometry-releases"
        or not re.fullmatch(r"[0-9a-f]{64}", active_target.parts[1])
    ):
        raise DeploymentError("verified Geometry release active target is invalid")

    source_generation = build_parent.joinpath(*active_target.parts)
    if source_generation.is_symlink() or not source_generation.is_dir():
        raise DeploymentError("verified Geometry release generation is unsafe")
    destination_active = release_dir.joinpath(*manifest_relative.parent.parts)
    destination_parent = destination_active.parent
    destination_generations = destination_parent / "nuwax-geometry-releases"
    destination_generation = destination_generations / active_target.parts[1]
    if destination_active.exists() or destination_active.is_symlink():
        raise DeploymentError("candidate Geometry release active path already exists")
    if destination_generation.exists() or destination_generation.is_symlink():
        raise DeploymentError("candidate Geometry release generation already exists")
    _copy_tree_normalized(source_generation, destination_generation)
    os.symlink(
        PurePosixPath("nuwax-geometry-releases", active_target.parts[1]).as_posix(),
        destination_active,
    )

    checker = release_dir / "services" / "geometry" / "release_manifest.py"
    context.runner.run(
        (
            str(_venv_python(context)),
            str(checker),
            "check",
            "--manifest",
            str(destination_active / "manifest.json"),
        ),
        timeout=300,
    )


def install_release(context: "ExecutionContext") -> None:
    options = context.options
    release_dir = options.release_dir
    expected_source_identity = context.notes.get("source_identity_at_preflight")
    if isinstance(expected_source_identity, str):
        if _git_source_identity(options.source_root) != expected_source_identity:
            raise DeploymentError("production source changed after preflight")
    _prepare_backup_root(context)
    manifest_path = release_dir / "release-manifest.json"
    if manifest_path.is_file():
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise DeploymentError(f"existing release is incomplete: {release_dir}") from exc
        if manifest.get("fingerprint") != context.plan.fingerprint:
            raise DeploymentError(f"release id already exists with a different fingerprint: {release_dir}")
        if not options.no_systemd:
            snapshot_script = _require_file(
                options.source_root / "scripts" / "install_motion_guard_deployment.sh",
                "source systemd installer",
            )
            snapshot_env = dict(_deployment_env(context))
            snapshot_env["NUWAX_REPO_ROOT_OVERRIDE"] = str(options.source_root)
            snapshot_env["NUWAX_SYSTEMD_BACKUP_DIR"] = str(_systemd_backup_dir(context))
            context.runner.run(
                ("bash", str(snapshot_script), "snapshot-systemd"),
                env=snapshot_env,
                timeout=300,
            )
        _record_recovery(context, release_dir, activated=False)
        return

    options.releases_dir.mkdir(parents=True, exist_ok=True)
    release_dir.mkdir(parents=True, exist_ok=True)
    marker = release_dir / ".installing"
    if marker.is_file() and marker.read_text(encoding="utf-8").strip() != context.plan.fingerprint:
        raise DeploymentError(f"incomplete release has a different fingerprint: {release_dir}")
    if not _venv_python(context).is_file():
        raise DeploymentError(f"release virtual environment is missing: {_venv_python(context)}")
    _copy_release_source(options.source_root, release_dir)
    bin_dir = release_dir / "bin"
    bin_dir.mkdir(mode=0o755, exist_ok=True)
    binary_hashes = {}
    binary_metadata = {}
    build_uid = resolve_build_account().pw_uid
    for build in context.profile.rust.builds:
        destination = (
            release_dir / build.output
            if build.binary == "nuwax-geometry-runtime"
            else bin_dir / build.binary
        )
        _copy_rust_build_output(
            options.rust_target_dir(build.manifest),
            build.binary,
            destination,
            build_uid,
        )
        binary_hashes[build.binary] = _sha256(destination)
        binary_metadata[build.binary] = _validate_release_binary(destination)
    _install_geometry_release(context, release_dir)
    models = options.build_dir / "models"
    if models.is_dir():
        _copy_tree_normalized(models, release_dir / "models")
    if context.profile.robot == "go2":
        sdk_artifact = context.notes.get("runtime_artifact:unitree-sdk2py")
        if sdk_artifact is None:
            sdk_artifact = _python_install_artifact(
                context,
                "unitree-sdk2py",
                context.notes["go2_sdk_artifact"],
            )
        sdk_artifact = Path(sdk_artifact)
        sdk_destination = release_dir / "vendor" / "unitree_sdk2_python"
        if sdk_artifact.is_file():
            _extract_unitree_wheel(sdk_artifact, sdk_destination)
        else:
            sdk_source = _require_dir(sdk_artifact, "verified Go2 SDK source")
            if sdk_destination.is_symlink():
                raise DeploymentError(f"Go2 SDK vendor destination is unsafe: {sdk_destination}")
            if sdk_destination.is_dir():
                shutil.rmtree(sdk_destination)
            _copy_tree_normalized(sdk_source, sdk_destination)
        context.notes["runtime_go2_sdk"] = options.current_link / "vendor" / "unitree_sdk2_python"
    cargo_locks = []
    for build in context.profile.rust.builds:
        cargo_lock = str(Path(build.manifest).parent / "Cargo.lock")
        cargo_locks.append(cargo_lock)
    python_inputs = list(context.profile.python.requirements)
    if context.profile.python.constraints:
        python_inputs.append(context.profile.python.constraints)
    if context.profile.python.source_artifact_manifest:
        python_inputs.append(context.profile.python.source_artifact_manifest)
    model_hashes = {}
    model_root = release_dir / "models"
    if model_root.is_dir():
        for model in sorted(model_root.rglob("*")):
            if model.is_file() and not model.is_symlink():
                model_hashes[model.relative_to(model_root).as_posix()] = _sha256(model)
    release_manifest = {
        "schema_version": 2,
        "profile": context.profile.name,
        "fingerprint": context.plan.fingerprint,
        "release_id": options.release_id,
        "source_revision": _source_revision(context),
        "source_tree_id": options.source_tree_id,
        "rust_toolchain": context.profile.rust.toolchain,
        "rust_target": platform.machine().lower(),
        "system_package_versions": context.notes.get("system_package_versions", {}),
        "cargo_lock_sha256": _hash_existing_files(options.source_root, cargo_locks),
        "python_input_sha256": _hash_existing_files(options.source_root, python_inputs),
        "binary_sha256": binary_hashes,
        "binary_file": binary_metadata,
        "source_tree_sha256": _release_source_digest(release_dir),
        "venv_inventory_sha256": _venv_inventory_digest(release_dir),
        "vendor_tree_sha256": (
            {"unitree_sdk2_python": _tree_digest(release_dir / "vendor" / "unitree_sdk2_python")}
            if context.profile.robot == "go2"
            else {}
        ),
        "model_sha256": model_hashes,
        "third_party_sha256": {
            key.split(":", 1)[1]: value
            for key, value in context.notes.items()
            if key.startswith("artifact_sha256:") and isinstance(value, str)
        },
    }
    atomic_write(
        release_dir / "release-manifest.json",
        json.dumps(release_manifest, indent=2, sort_keys=True) + "\n",
        mode=0o644,
    )
    try:
        marker.unlink()
    except FileNotFoundError:
        pass
    if not options.no_systemd:
        snapshot_script = _require_file(
            options.source_root / "scripts" / "install_motion_guard_deployment.sh",
            "source systemd installer",
        )
        snapshot_env = dict(_deployment_env(context))
        snapshot_env["NUWAX_REPO_ROOT_OVERRIDE"] = str(options.source_root)
        snapshot_env["NUWAX_SYSTEMD_BACKUP_DIR"] = str(_systemd_backup_dir(context))
        context.runner.run(
            ("bash", str(snapshot_script), "snapshot-systemd"),
            env=snapshot_env,
            timeout=300,
        )
    _record_recovery(context, release_dir, activated=False)


def _backup_config(context: "ExecutionContext", path: Path) -> None:
    if path in context.config_backups:
        return
    if path.is_symlink():
        raise DeploymentError(f"refusing to read or replace config symlink: {path}")
    previous = path.read_bytes() if path.is_file() else None
    context.config_backups[path] = previous
    metadata = None
    if previous is not None:
        current = path.stat()
        metadata = {
            "uid": current.st_uid,
            "gid": current.st_gid,
            "mode": stat.S_IMODE(current.st_mode),
        }
    config_metadata = context.notes.setdefault("config_metadata", {})
    config_metadata[str(path)] = metadata

    backup_root = _prepare_backup_root(context) / "config"
    if backup_root.is_symlink():
        raise DeploymentError(f"config backup path must not be a symlink: {backup_root}")
    backup_root.mkdir(parents=True, exist_ok=True)
    backup_name = hashlib.sha256(str(path).encode("utf-8")).hexdigest() + ".bak"
    if previous is not None:
        backup_path = backup_root / backup_name
        if backup_path.is_symlink():
            raise DeploymentError(f"config backup file must not be a symlink: {backup_path}")
        backup_path.write_bytes(previous)
        os.chmod(backup_path, 0o600)
    manifest_path = backup_root / "manifest.json"
    if manifest_path.is_symlink():
        raise DeploymentError(f"config backup manifest must not be a symlink: {manifest_path}")
    manifest = {"schema_version": 1, "files": {}}
    if manifest_path.is_file():
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise DeploymentError(f"config backup manifest is invalid: {manifest_path}") from exc
    manifest["files"][str(path)] = {
        "existed": previous is not None,
        "backup": backup_name if previous is not None else None,
        "metadata": metadata,
    }
    atomic_write(
        manifest_path,
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        mode=0o600,
    )


def _secure_config_owner(path: Path) -> None:
    try:
        gid = grp.getgrnam("nuwax-motion-clients").gr_gid
    except KeyError:
        return
    if path.is_symlink():
        raise DeploymentError(f"refusing config symlink: {path}")
    if path.is_file():
        os.chown(path, 0, gid)
        os.chmod(path, 0o640)


def _parse_env_values(source: str) -> Mapping[str, str]:
    values = {}
    for raw_line in source.splitlines():
        stripped = raw_line.strip()
        if not stripped or stripped.startswith("#") or "=" not in raw_line:
            continue
        key, value = raw_line.split("=", 1)
        key = key.strip()
        value = value.strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in {"'", '"'}:
            value = value[1:-1]
        values[key] = value
    return values


def _copy_migration_source(source: Path, destination: Path) -> None:
    if source.is_symlink():
        raise DeploymentError(f"legacy migration source must not be a symlink: {source}")
    for component in (destination, *destination.parents):
        if component.is_symlink():
            raise DeploymentError(f"legacy migration target path contains a symlink: {component}")
        if component == Path("/"):
            break
    if source.is_file():
        destination.parent.mkdir(parents=True, exist_ok=True)
        if destination.exists():
            if not destination.is_file() or _sha256(destination) != _sha256(source):
                raise DeploymentError(f"legacy migration target already has different data: {destination}")
            return
        shutil.copy2(source, destination)
        return
    if not source.is_dir():
        return
    for candidate in sorted(source.rglob("*")):
        relative = candidate.relative_to(source)
        if candidate.is_symlink():
            raise DeploymentError(f"legacy migration tree contains a symlink: {candidate}")
        target = destination / relative
        if candidate.is_dir():
            target.mkdir(parents=True, exist_ok=True)
            continue
        if not candidate.is_file():
            continue
        target.parent.mkdir(parents=True, exist_ok=True)
        if target.exists():
            if not target.is_file() or _sha256(target) != _sha256(candidate):
                raise DeploymentError(f"legacy migration target already has different data: {target}")
            continue
        shutil.copy2(candidate, target)


def _migrate_legacy_runtime_data(context: "ExecutionContext", source_env: str) -> None:
    values = _parse_env_values(source_env)
    targets = {
        "NUWAX_GATEWAY_ARTIFACT_DIR": Path("/var/lib/nuwax/gateway/artifacts"),
        "NUWAX_MAP_CATALOG_ROOT": Path("/var/lib/nuwax/gateway/maps"),
        "NUWAX_MEMORY_DB_PATH": Path("/var/lib/nuwax/gateway/memory/vector_memory.db"),
        "NUWAX_MAPPING_DATABASE_ROOT": Path("/var/lib/nuwax/gateway/geometry/mapping"),
        "NUWAX_MAPPING_PCD_ROOT": Path("/var/lib/nuwax/gateway/geometry/pcd"),
    }
    migrations = []
    for key, destination in targets.items():
        raw_source = values.get(key, "").strip()
        if not raw_source:
            continue
        source = Path(raw_source).expanduser()
        if not source.is_absolute():
            raise DeploymentError(f"legacy migration path must be absolute: {key}={raw_source}")
        if source.resolve() == destination.resolve() or not source.exists():
            continue
        _copy_migration_source(source, destination)
        migrations.append({"key": key, "source": str(source), "destination": str(destination)})
    if migrations:
        atomic_write(
            _transaction_backup_dir(context) / "migration-manifest.json",
            json.dumps({"schema_version": 1, "copies": migrations}, indent=2, sort_keys=True) + "\n",
            mode=0o600,
        )


def write_configuration(context: "ExecutionContext") -> None:
    options = context.options
    config_dir = options.config_dir
    root_config = config_dir / "nuwax.env"
    go2_config = config_dir / "go2-data-plane.env"
    root_source = (
        root_config.read_text(encoding="utf-8")
        if root_config.is_file()
        else (
            options.legacy_config.read_text(encoding="utf-8")
            if options.legacy_config is not None
            else (options.source_root / ".env.example").read_text(encoding="utf-8")
        )
    )
    if root_config.is_file() or options.legacy_config is not None:
        _migrate_legacy_runtime_data(context, root_source)
    managed = {
        "NUWAX_ACTIVE_ROBOT": context.profile.robot,
        "NUWAX_PYTHON_BIN": str(options.current_link / ".venv" / "bin" / "python"),
        "NUWAX_GATEWAY_HOST": options.gateway_host,
        "NUWAX_NUWAX_WORKSPACE_DIR": str(options.current_link / "services" / "nuwax-workspace"),
        "NUWAX_GEOMETRY_RUNTIME_BINARY": str(
            options.current_link
            / "services"
            / "geometry"
            / "nuwax_geometry_runtime"
            / "target"
            / "release"
            / "nuwax-geometry-runtime"
        ),
        "NUWAX_GATEWAY_ARTIFACT_DIR": "/var/lib/nuwax/gateway/artifacts",
        "NUWAX_MAP_CATALOG_ROOT": "/var/lib/nuwax/gateway/maps",
        "NUWAX_MAPPING_DATABASE_ROOT": "/var/lib/nuwax/gateway/geometry/mapping",
        "NUWAX_MAPPING_PCD_ROOT": "/var/lib/nuwax/gateway/geometry/pcd",
        "NUWAX_MEMORY_DB_PATH": "/var/lib/nuwax/gateway/memory/vector_memory.db",
        "NUWAX_LOG_DIR": "/var/log/nuwax/gateway",
        "NUWAX_LOG_FILE": "/var/log/nuwax/gateway/nuwax_robot_bridge.log",
        "GO2_PROXY_LOG_DIR": "/var/log/nuwax/gateway",
        "GO2_PROXY_LOG_FILE": "/var/log/nuwax/gateway/nuwax_robot_bridge.log",
        "YOLO_CONFIG_DIR": "/var/lib/nuwax/gateway/ultralytics",
        "XDG_CONFIG_HOME": "/var/lib/nuwax/gateway/config",
        "XDG_CACHE_HOME": "/var/lib/nuwax/gateway/cache",
        "NUWAX_CLOUDMAP_SHM_PATH": "/var/lib/nuwax_robot_bridge/nuwax_cloudmap.mmap",
        "NUWAX_OCCUPANCY_SHM_PATH": "/var/lib/nuwax_robot_bridge/nuwax_occupancy.mmap",
        "NUWAX_FORBIDDEN_ZONES_PATH": "/var/lib/nuwax_robot_bridge/forbidden_zones.json",
        "GO2_SDK_PATH": str(
            context.notes.get("runtime_go2_sdk") or (options.current_link / "vendor" / "unitree_sdk2_python")
        ),
    }
    if context.profile.models.get("prepare_yolo") and not options.skip_models:
        export_engine = options.export_yolo_engine or bool(context.profile.models.get("export_yolo_engine_default"))
        model_name = "yolov8s-worldv2.engine" if export_engine else "yolov8s-worldv2.pt"
        managed["NUWAX_PERCEPTION_YOLO_WORLD_MODEL"] = str(options.current_link / "models" / model_name)
    root_rendered = render_env(root_source, managed)
    config_changed = not root_config.is_file() or root_config.read_text(encoding="utf-8") != root_rendered
    if config_changed:
        _backup_config(context, root_config)
        atomic_write(root_config, root_rendered, mode=0o640)
    if context.profile.robot == "go2":
        go2_source = (
            go2_config.read_text(encoding="utf-8")
            if go2_config.is_file()
            else (options.source_root / "drivers" / "go2" / "data_plane.env.example").read_text(encoding="utf-8")
        )
        go2_managed = {
            "GO2_DATA_PLANE_ENABLED": "true",
            "GO2_DIRECT_DDS_ENABLED": "true",
            "GO2_OFFICIAL_BACKEND_ENABLED": "true",
        }
        if options.dds_interface:
            go2_managed["GO2_DATA_PLANE_DDS_IFACE"] = options.dds_interface
        go2_rendered = render_env(go2_source, go2_managed)
        go2_changed = not go2_config.is_file() or go2_config.read_text(encoding="utf-8") != go2_rendered
        if go2_changed:
            _backup_config(context, go2_config)
            atomic_write(go2_config, go2_rendered, mode=0o640)
        config_changed = config_changed or go2_changed
    context.notes["config_unchanged"] = not config_changed


def _deployment_env(
    context: "ExecutionContext",
    *,
    active_release: bool = False,
) -> Mapping[str, str]:
    options = context.options
    sdk = context.notes.get("runtime_go2_sdk")
    if not sdk:
        config = options.config_dir / "nuwax.env"
        if config.is_file():
            for line in config.read_text(encoding="utf-8").splitlines():
                if line.startswith("GO2_SDK_PATH="):
                    sdk = line.split("=", 1)[1].strip()
                    break
    candidate_root = options.current_link if active_release else options.release_dir
    candidate_sdk = candidate_root / "vendor" / "unitree_sdk2_python"
    return {
        "NUWAX_REPO_ROOT_OVERRIDE": str(candidate_root),
        "NUWAX_BIN_DIR": str(candidate_root / "bin"),
        "NUWAX_PYTHON_BIN": str(candidate_root / ".venv" / "bin" / "python"),
        "NUWAX_GO2_SDK_PATH": str(candidate_sdk if candidate_sdk.is_dir() else (sdk or "")),
        "NUWAX_CONFIG_DIR": str(options.config_dir),
        "NUWAX_SYSTEMD_BACKUP_DIR": str(_systemd_backup_dir(context)),
        "NUWAX_RENDER_ROOT": str(options.current_link),
        "NUWAX_RENDER_BIN_DIR": str(options.current_link / "bin"),
        "NUWAX_RENDER_PYTHON_BIN": str(options.current_link / ".venv" / "bin" / "python"),
        "NUWAX_RENDER_GO2_SDK_PATH": str(options.current_link / "vendor" / "unitree_sdk2_python"),
        "NUWAX_RENDER_CONFIG_DIR": str(options.config_dir),
    }


def _deployment_script(context: "ExecutionContext") -> Path:
    return context.options.release_dir / "scripts" / "install_motion_guard_deployment.sh"


def install_systemd(context: "ExecutionContext") -> None:
    script = _require_file(_deployment_script(context), "release systemd installer")
    context.runner.run(("bash", str(script), "install"), env=_deployment_env(context), timeout=900)
    context.notes["systemd_unchanged"] = not (_systemd_backup_dir(context) / "sources.changed").is_file()
    try:
        gid = grp.getgrnam("nuwax-motion-clients").gr_gid
    except KeyError as exc:
        raise DeploymentError("systemd install did not create nuwax-motion-clients") from exc
    for path in (context.options.config_dir / "nuwax.env", context.options.config_dir / "go2-data-plane.env"):
        if path.exists():
            os.chown(path, 0, gid)
            os.chmod(path, 0o640)


def provision_safety(context: "ExecutionContext") -> None:
    manifest = _require_file(context.options.actuator_manifest, "actuator symbols manifest")
    expected = context.notes.get("actuator_manifest_sha256")
    snapshot_dir = _prepare_backup_root(context) / "inputs"
    if snapshot_dir.is_symlink():
        raise DeploymentError(f"deployment input snapshot path is a symlink: {snapshot_dir}")
    snapshot_dir.mkdir(parents=True, exist_ok=True)
    os.chmod(snapshot_dir, 0o700)
    snapshot = snapshot_dir / "actuator-symbols.json"
    fd, temp_name = tempfile.mkstemp(prefix=".actuator-symbols.", dir=snapshot_dir)
    temporary = Path(temp_name)
    try:
        with manifest.open("rb") as source, os.fdopen(fd, "wb") as output:
            shutil.copyfileobj(source, output)
            output.flush()
            os.fsync(output.fileno())
        os.chmod(temporary, 0o600)
        digest = validate_actuator_manifest(temporary)
        if expected and digest != expected:
            raise DeploymentError("actuator manifest changed after preflight")
        os.replace(temporary, snapshot)
        fsync_directory(snapshot_dir)
    finally:
        temporary.unlink(missing_ok=True)
    script = _require_file(_deployment_script(context), "release systemd installer")
    context.runner.run(
        ("bash", str(script), "provision", str(snapshot)),
        env=_deployment_env(context),
        timeout=300,
    )


def _service_activation_contract(profile) -> Tuple[FrozenSet[str], FrozenSet[str]]:
    services = frozenset(profile.services)
    metadata = profile.metadata if isinstance(profile.metadata, Mapping) else {}
    always_on_raw = metadata.get("always_on_services")
    on_demand_raw = metadata.get("on_demand_services")
    if always_on_raw is None and on_demand_raw is None:
        return services, frozenset()
    if not isinstance(always_on_raw, (list, tuple)) or not all(
        isinstance(item, str) and item for item in always_on_raw
    ):
        raise DeploymentError("profile metadata always_on_services must be a list of unit names")
    if not isinstance(on_demand_raw, (list, tuple)) or not all(
        isinstance(item, str) and item for item in on_demand_raw
    ):
        raise DeploymentError("profile metadata on_demand_services must be a list of unit names")
    always_on = frozenset(always_on_raw)
    on_demand = frozenset(on_demand_raw)
    if always_on & on_demand:
        raise DeploymentError("profile service activation sets overlap")
    if always_on | on_demand != services:
        raise DeploymentError("profile service activation sets must cover profile.services exactly")
    return always_on, on_demand


_ALWAYS_ON_UNIT_FILE_STATES = frozenset({"enabled"})
_ON_DEMAND_UNIT_FILE_STATES = frozenset({"disabled", "static", "indirect"})


def _unit_file_state(service: str) -> str:
    result = subprocess.run(
        ("systemctl", "show", "-p", "UnitFileState", "--value", service),
        check=False,
        capture_output=True,
        text=True,
        timeout=30,
    )
    if result.returncode != 0:
        detail = result.stderr.strip() or result.stdout.strip() or "no output"
        raise DeploymentError(
            f"cannot read UnitFileState for {service}: rc={result.returncode}: {detail}"
        )
    state = result.stdout.strip()
    if not state:
        raise DeploymentError(f"cannot read UnitFileState for {service}: state=<empty>")
    return state


def enable_and_start(context: "ExecutionContext") -> None:
    script = _require_file(_deployment_script(context), "release systemd installer")
    env = _deployment_env(context)
    always_on_services, on_demand_services = _service_activation_contract(
        context.profile
    )
    if (
        context.notes.get("current_unchanged")
        and context.notes.get("config_unchanged")
        and context.notes.get("systemd_unchanged")
    ):
        all_ready = True
        for service in context.profile.services:
            unit_file_state = _unit_file_state(service)
            active = (
                subprocess.run(
                    ("systemctl", "is-active", "--quiet", service),
                    check=False,
                    timeout=30,
                ).returncode
                == 0
            )
            if service in always_on_services:
                all_ready = (
                    all_ready
                    and unit_file_state in _ALWAYS_ON_UNIT_FILE_STATES
                    and active
                )
            elif service in on_demand_services:
                all_ready = (
                    all_ready and unit_file_state in _ON_DEMAND_UNIT_FILE_STATES
                )
            else:
                all_ready = False
        if all_ready:
            context.notes["services_unchanged"] = True
            return
    context.runner.run(("bash", str(script), "enable"), env=env, timeout=300)
    context.runner.run(("bash", str(script), "start"), env=env, timeout=600)


def activate_release(context: "ExecutionContext") -> None:
    release_dir = context.options.release_dir
    manifest = _require_file(release_dir / "release-manifest.json", "candidate release manifest")
    try:
        payload = json.loads(manifest.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise DeploymentError(f"candidate release manifest is invalid: {manifest}") from exc
    if payload.get("fingerprint") != context.plan.fingerprint:
        raise DeploymentError("candidate release fingerprint changed before activation")
    _switch_current(context, release_dir)


def _health_ready(timeout_seconds: int = 90) -> None:
    deadline = time.monotonic() + timeout_seconds
    last_error = "health endpoint not attempted"
    while time.monotonic() < deadline:
        try:
            with urllib.request.urlopen("http://127.0.0.1:8766/api/health/full", timeout=5) as response:
                if response.status != 200:
                    last_error = f"HTTP {response.status}"
                else:
                    payload = json.loads(response.read().decode("utf-8"))
                    if not isinstance(payload, dict):
                        last_error = "health payload is not a JSON object"
                        time.sleep(2)
                        continue
                    overall = payload.get("overall")
                    if payload.get("ready") is True and isinstance(overall, str) and overall.lower() != "unhealthy":
                        return
                    last_error = f"ready={payload.get('ready')!r} overall={overall!r}"
        except (OSError, urllib.error.URLError, json.JSONDecodeError) as exc:
            last_error = str(exc)
        time.sleep(2)
    raise DeploymentError(f"authoritative health did not become ready: {last_error}")


def verify_deployment(context: "ExecutionContext") -> None:
    current = context.options.current_link
    if not current.is_symlink():
        raise DeploymentError(f"active release link is missing or unsafe: {current}")
    active_release = current.resolve()
    if active_release.parent.resolve() != context.options.releases_dir.resolve():
        raise DeploymentError("active release escapes the managed releases directory")
    manifest_path = _require_file(
        active_release / "release-manifest.json",
        "active release manifest",
    )
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise DeploymentError(f"active release manifest is invalid: {manifest_path}") from exc
    if manifest.get("profile") != context.profile.name:
        raise DeploymentError("active release profile does not match requested verification profile")
    if manifest.get("release_id") != active_release.name:
        raise DeploymentError("active release manifest identity does not match its directory")
    _verify_python_runtime(context)
    _verify_vendor_python_packages(context)
    _validate_completed_release(active_release, manifest)
    recorded_packages = manifest.get("system_package_versions")
    if not isinstance(recorded_packages, dict):
        raise DeploymentError("active release has no system package inventory")
    for package, expected_version in recorded_packages.items():
        if (
            not isinstance(package, str)
            or not isinstance(expected_version, str)
            or _installed_package_version(package) != expected_version
        ):
            raise DeploymentError(f"active release system package drifted: {package}")
    required_package_names = {package_spec.partition("=")[0] for package_spec in context.profile.system_packages}
    if not required_package_names.issubset(recorded_packages):
        raise DeploymentError("active release system package inventory is incomplete")
    _validate_jetson_compatibility(context)
    if not context.options.no_systemd:
        script = _require_file(
            active_release / "scripts" / "install_motion_guard_deployment.sh",
            "active release systemd installer",
        )
        context.runner.run(
            ("bash", str(script), "verify-runtime"),
            env=_deployment_env(context, active_release=True),
            timeout=600,
        )
    context.runner.run(
        (
            str(context.options.current_link / ".venv" / "bin" / "python"),
            "-m",
            "scripts.deploy_installer.lcm_probe",
        ),
        cwd=context.options.current_link,
        env={
            **_build_user_env(context),
            "NUWAX_LCM_URL": "udpm://239.255.76.67:7667?ttl=1",
        },
        as_build_user=True,
        timeout=30,
    )
    _health_ready()


def rollback(context: "ExecutionContext") -> bool:
    options = context.options
    env = _deployment_env(context)
    candidate_script = options.release_dir / "scripts" / "install_motion_guard_deployment.sh"
    snapshot_exists = (_systemd_backup_dir(context) / "snapshot.complete").is_file()
    if context.release_switched and context.previous_release:
        _validate_managed_release(
            context,
            Path(context.previous_release),
            "rollback target",
        )
    rollback_performed = bool(
        context.release_switched
        or context.notes.get("same_current_recovery")
        or context.config_backups
        or snapshot_exists
    )
    if rollback_performed:
        _mark_recovery_rolling_back(context)
    stop_active = bool(context.release_switched or context.notes.get("same_current_recovery"))
    if stop_active and not options.no_systemd and (options.current_link.exists() or options.current_link.is_symlink()):
        script = _require_file(
            options.current_link / "scripts" / "install_motion_guard_deployment.sh",
            "active release rollback script",
        )
        context.runner.run(("bash", str(script), "stop"), env=env, timeout=300)
    for path, previous in context.config_backups.items():
        if previous is None:
            try:
                path.unlink()
            except FileNotFoundError:
                pass
        else:
            path.parent.mkdir(parents=True, exist_ok=True)
            atomic_write(path, previous.decode("utf-8"), mode=0o640)
            metadata = context.notes.get("config_metadata", {}).get(str(path))
            if isinstance(metadata, dict):
                os.chown(path, int(metadata["uid"]), int(metadata["gid"]))
                os.chmod(path, int(metadata["mode"]))
            else:
                _secure_config_owner(path)
    if context.release_switched:
        if context.previous_release:
            temp = options.current_link.with_name(f".{options.current_link.name}.rollback-{os.getpid()}")
            try:
                temp.unlink()
            except FileNotFoundError:
                pass
            os.symlink(context.previous_release, temp)
            os.replace(temp, options.current_link)
            fsync_directory(options.current_link.parent)
        else:
            if not options.no_systemd and context.profile.services:
                try:
                    context.runner.run(
                        ("systemctl", "disable", *context.profile.services),
                        timeout=300,
                    )
                except Exception:
                    pass
            try:
                options.current_link.unlink()
            except FileNotFoundError:
                pass
            else:
                fsync_directory(options.current_link.parent)
    if not options.no_systemd and candidate_script.is_file() and snapshot_exists:
        context.runner.run(
            ("bash", str(candidate_script), "restore-systemd"),
            env=env,
            timeout=600,
        )
    if (context.previous_release or context.notes.get("same_current_recovery")) and not options.no_verify:
        try:
            verify_deployment(context)
        except Exception:
            active_script = options.current_link / "scripts" / "install_motion_guard_deployment.sh"
            if active_script.is_file() and not options.no_systemd:
                try:
                    context.runner.run(
                        ("bash", str(active_script), "stop"),
                        env=_deployment_env(context, active_release=True),
                        timeout=300,
                    )
                except Exception:
                    pass
            raise
    return rollback_performed


def load_persisted_recovery(context: "ExecutionContext") -> bool:
    backup_root = _transaction_backup_dir(context)
    recovery_path = backup_root / "recovery.json"
    if not recovery_path.is_file():
        return False
    try:
        recovery = json.loads(recovery_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise DeploymentError(f"deployment recovery record is invalid: {recovery_path}") from exc
    schema_version = recovery.get("schema_version")
    if schema_version not in {1, 2}:
        raise DeploymentError("unsupported deployment recovery record")
    if recovery.get("current_link") != str(context.options.current_link):
        raise DeploymentError("deployment recovery current link does not match options")
    candidate_raw = recovery.get("candidate_release")
    if not isinstance(candidate_raw, str) or not Path(candidate_raw).is_absolute():
        raise DeploymentError("deployment recovery candidate path is invalid")
    candidate = Path(candidate_raw)
    if candidate.resolve() != context.options.release_dir.resolve():
        raise DeploymentError("deployment recovery candidate does not match release identity")
    previous = recovery.get("previous_release")
    if previous is not None:
        previous_path = Path(str(previous))
        if (
            not previous_path.is_absolute()
            or context.options.releases_dir.resolve() not in previous_path.resolve().parents
        ):
            raise DeploymentError("deployment recovery previous release path is invalid")
    current = context.options.current_link
    context.previous_release = str(previous) if previous else None
    if schema_version == 1:
        activated = recovery.get("activated")
        if not isinstance(activated, bool):
            raise DeploymentError("deployment recovery activation state is invalid")
        if activated:
            if not current.is_symlink() or current.resolve() != candidate.resolve():
                raise DeploymentError("refusing stale transaction rollback: current is not the recorded candidate")
            context.release_switched = True
        else:
            if context.previous_release:
                if not current.is_symlink() or current.resolve() != Path(context.previous_release).resolve():
                    raise DeploymentError("refusing stale pre-activation recovery: current no longer matches previous")
            elif current.exists() or current.is_symlink():
                raise DeploymentError(
                    "refusing stale pre-activation recovery: current was created by another transaction"
                )
            context.release_switched = False
    else:
        activation_state = recovery.get("activation_state")
        same_current = recovery.get("same_current")
        current_at_start = recovery.get("current_at_start")
        if activation_state not in {"prepared", "activated", "rolling_back"}:
            raise DeploymentError("deployment recovery activation state is invalid")
        if not isinstance(same_current, bool):
            raise DeploymentError("deployment recovery same-current state is invalid")
        start_target = None
        if current_at_start is not None:
            if not isinstance(current_at_start, str):
                raise DeploymentError("deployment recovery start target is invalid")
            start_target = Path(current_at_start)
            if (
                not start_target.is_absolute()
                or context.options.releases_dir.resolve() not in start_target.resolve().parents
            ):
                raise DeploymentError("deployment recovery start target is invalid")
        start_is_candidate = bool(start_target is not None and start_target.resolve() == candidate.resolve())
        if same_current != start_is_candidate:
            raise DeploymentError("deployment recovery same-current facts are inconsistent")
        if same_current:
            if context.previous_release is not None:
                raise DeploymentError("same-current recovery must not record a previous release")
            context.notes["same_current_recovery"] = True
        else:
            expected_previous = str(start_target) if start_target is not None else None
            if context.previous_release != expected_previous:
                raise DeploymentError("deployment recovery previous release is inconsistent")

        if current.is_symlink():
            actual_current = current.resolve()
        elif current.exists():
            raise DeploymentError("refusing transaction rollback: current exists and is not a symlink")
        else:
            actual_current = None
        if activation_state == "activated":
            if actual_current != candidate.resolve():
                raise DeploymentError("refusing stale activated rollback: current is not the recorded candidate")
            context.release_switched = not same_current
        elif activation_state == "rolling_back":
            if same_current:
                if actual_current != candidate.resolve():
                    raise DeploymentError("refusing stale same-current rollback: current no longer matches candidate")
                context.release_switched = False
            elif actual_current == candidate.resolve():
                context.release_switched = True
            elif start_target is None and actual_current is None:
                context.release_switched = False
            elif start_target is not None and actual_current == start_target.resolve():
                context.release_switched = False
            else:
                raise DeploymentError("refusing stale rolling-back transaction: current no longer matches facts")
        elif same_current:
            if actual_current != candidate.resolve():
                raise DeploymentError("refusing stale same-current recovery: current no longer matches candidate")
            context.release_switched = False
        elif actual_current == candidate.resolve():
            context.release_switched = True
        elif start_target is None and actual_current is None:
            context.release_switched = False
        elif start_target is not None and actual_current == start_target.resolve():
            context.release_switched = False
        else:
            raise DeploymentError("refusing stale prepared recovery: current no longer matches transaction facts")

    manifest_path = backup_root / "config" / "manifest.json"
    if manifest_path.is_file():
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise DeploymentError(f"config recovery manifest is invalid: {manifest_path}") from exc
        files = manifest.get("files")
        if not isinstance(files, dict):
            raise DeploymentError("config recovery manifest files must be an object")
        metadata_map = context.notes.setdefault("config_metadata", {})
        for raw_path, record in files.items():
            if not isinstance(record, dict):
                raise DeploymentError("config recovery record is invalid")
            path = Path(raw_path)
            if path.parent.resolve() != context.options.config_dir.resolve():
                raise DeploymentError("config recovery path escapes the managed config directory")
            if record.get("existed") is True:
                backup_name = record.get("backup")
                if not isinstance(backup_name, str) or Path(backup_name).name != backup_name:
                    raise DeploymentError("config recovery backup name is invalid")
                backup = backup_root / "config" / backup_name
                context.config_backups[path] = _require_file(backup, "config recovery backup").read_bytes()
            else:
                context.config_backups[path] = None
            metadata_map[str(path)] = record.get("metadata")
    return bool(
        context.release_switched
        or schema_version == 2
        and recovery.get("same_current") is True
        or manifest_path.is_file()
        or (_systemd_backup_dir(context) / "snapshot.complete").is_file()
    )
