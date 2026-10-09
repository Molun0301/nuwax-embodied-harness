from __future__ import annotations

import hashlib
import json
import os
import shutil
import stat
import subprocess
import tarfile
from pathlib import Path, PurePosixPath
from typing import Dict, Mapping, Optional

from .config import atomic_write
from .runner import source_user_command, source_user_environment


class BundleError(RuntimeError):
    pass


def _digest(path: Path) -> str:
    value = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def _copy_source_snapshot(
    source: Path,
    destination: Path,
    expected_revision: str,
    *,
    local_layout: bool = False,
) -> Optional[Dict[str, Dict[str, object]]]:
    if not source.is_dir():
        raise BundleError(f"bundle source directory is missing: source={source}")
    git_env = source_user_environment()
    head = subprocess.run(
        source_user_command(("git", "rev-parse", "HEAD")),
        cwd=str(source),
        env=git_env,
        capture_output=True,
        text=True,
        check=False,
        timeout=60,
    )
    actual_revision = head.stdout.strip()
    if head.returncode != 0 or not actual_revision:
        raise BundleError("bundle source must be a valid Git checkout")
    if expected_revision != actual_revision:
        raise BundleError(
            f"bundle source revision does not match Git HEAD: expected={expected_revision} actual={actual_revision}"
        )
    dirty = subprocess.run(
        source_user_command(("git", "status", "--porcelain", "--untracked-files=normal")),
        cwd=str(source),
        env=git_env,
        capture_output=True,
        text=True,
        check=False,
        timeout=60,
    )
    if dirty.returncode != 0 or dirty.stdout.strip():
        raise BundleError("bundle source Git checkout must be clean")
    index = subprocess.run(
        source_user_command(("git", "ls-files", "--stage", "-z")),
        cwd=str(source),
        env=git_env,
        capture_output=True,
        check=False,
        timeout=60,
    )
    if index.returncode != 0 or not index.stdout:
        raise BundleError("bundle source has no tracked release files")
    for record in index.stdout.split(b"\0"):
        if record and record.split(b" ", 1)[0] == b"160000":
            raise BundleError("bundle source must not contain Git submodules")

    layout: Dict[str, Dict[str, object]] = {}
    destination.mkdir(parents=True, exist_ok=True)
    process = subprocess.Popen(
        source_user_command(("git", "archive", "--format=tar", actual_revision)),
        cwd=str(source),
        env=git_env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    try:
        if process.stdout is None:
            raise BundleError("cannot read Git archive output")
        with tarfile.open(fileobj=process.stdout, mode="r|") as archive:
            for member in archive:
                relative = PurePosixPath(member.name)
                if relative.is_absolute() or ".." in relative.parts or relative.as_posix() != member.name.rstrip("/"):
                    raise BundleError(f"unsafe Git archive path: {member.name}")
                if local_layout and (
                    "__pycache__" in relative.parts or relative.suffix == ".pyc"
                    or any(part.endswith(".egg-info") for part in relative.parts)
                ):
                    raise BundleError(f"source artifact contains generated build/import state: {member.name}")
                target = destination.joinpath(*relative.parts)
                if member.isdir():
                    target.mkdir(parents=True, exist_ok=True)
                    os.chmod(target, 0o755)
                    if local_layout:
                        layout[relative.as_posix()] = {"type": "directory"}
                elif member.isfile():
                    source_file = archive.extractfile(member)
                    if source_file is None:
                        raise BundleError(f"cannot read Git archive member: {member.name}")
                    target.parent.mkdir(parents=True, exist_ok=True)
                    with source_file, target.open("xb") as output:
                        shutil.copyfileobj(source_file, output)
                    os.chmod(target, 0o755 if member.mode & stat.S_IXUSR else 0o644)
                    if local_layout:
                        layout[relative.as_posix()] = {
                            "type": "file", "size": member.size, "executable": bool(member.mode & stat.S_IXUSR),
                        }
                else:
                    raise BundleError(f"Git archive contains a link or special file: {member.name}")
        stderr = process.stderr.read() if process.stderr is not None else b""
        if process.wait(timeout=60) != 0:
            detail = stderr.decode("utf-8", errors="replace").strip()
            raise BundleError(f"cannot create Git source archive: {detail}")
    except Exception:
        process.kill()
        process.wait()
        raise
    finally:
        if process.stdout is not None:
            process.stdout.close()
        if process.stderr is not None:
            process.stderr.close()
    return layout if local_layout else None


def create_bundle(
    output: Path,
    *,
    wheelhouse: Path,
    cargo_vendor: Path,
    apt_repo: Path,
    vendor: Path,
    rustup_home: Path,
    cargo_bin: Path,
    source: Path,
    licenses: Path,
    signing_key: Path,
    metadata: Mapping[str, object],
    models: Optional[Path] = None,
) -> Path:
    output = output.resolve()
    if output.exists():
        raise BundleError(f"bundle output already exists: {output}")
    sources: Dict[str, Path] = {
        "wheelhouse": wheelhouse,
        "cargo-vendor": cargo_vendor,
        "apt-repo": apt_repo,
        "vendor": vendor,
        "rustup-home": rustup_home,
        "cargo-bin": cargo_bin,
        "licenses": licenses,
    }
    for name, source_dir in sources.items():
        if source_dir.is_symlink() or not source_dir.is_dir():
            raise BundleError(f"bundle source directory is missing or is a symlink: {name}={source_dir}")
    if source.is_symlink() or not source.is_dir():
        raise BundleError(f"bundle source snapshot is missing or is a symlink: {source}")
    if not (apt_repo / "Packages").is_file():
        raise BundleError("apt-repo must contain Packages metadata")
    for proxy in ("cargo", "rustc", "rustup"):
        path = cargo_bin / proxy
        if not path.is_file() or not os.access(path, os.X_OK):
            raise BundleError(f"cargo-bin must contain executable {proxy}: {path}")
    if signing_key.is_symlink() or not signing_key.is_file():
        raise BundleError(f"bundle signing key is missing or is a symlink: {signing_key}")
    if signing_key.stat().st_mode & (stat.S_IRWXG | stat.S_IRWXO):
        raise BundleError("bundle signing key must not be accessible by group or other users")

    staging = output.with_name(f".{output.name}.staging-{os.getpid()}")
    if staging.exists():
        shutil.rmtree(staging)
    staging.mkdir(parents=True)
    try:
        for name, source_dir in sources.items():
            shutil.copytree(source_dir, staging / name, symlinks=True)
        source_revision = metadata.get("source_revision")
        if not isinstance(source_revision, str) or not source_revision:
            raise BundleError("bundle metadata requires source_revision")
        _copy_source_snapshot(source, staging / "source", source_revision)
        if models is not None:
            if models.is_symlink() or not models.is_dir():
                raise BundleError(f"models directory is missing or is a symlink: {models}")
            shutil.copytree(models, staging / "models", symlinks=True)
        files: Dict[str, Dict[str, object]] = {}
        for path in sorted(staging.rglob("*")):
            if path.is_symlink():
                raise BundleError(f"bundle sources must not contain symlinks: {path}")
            if path.is_file() and path.name != "bundle-manifest.json":
                relative = path.relative_to(staging).as_posix()
                if relative in {"source/.env", "source/drivers/go2/data_plane.env"}:
                    raise BundleError(f"runtime secret must not enter offline bundle: {relative}")
                files[path.relative_to(staging).as_posix()] = {
                    "sha256": _digest(path),
                    "size": path.stat().st_size,
                }
        manifest = {
            "schema_version": 1,
            "signature_algorithm": "rsa-sha256",
            "metadata": dict(metadata),
            "files": files,
        }
        manifest_path = staging / "bundle-manifest.json"
        atomic_write(
            manifest_path,
            json.dumps(manifest, indent=2, sort_keys=True) + "\n",
            mode=0o644,
        )
        signature_path = staging / "bundle-manifest.sig"
        completed = subprocess.run(
            (
                "openssl",
                "dgst",
                "-sha256",
                "-sign",
                str(signing_key),
                "-out",
                str(signature_path),
                str(manifest_path),
            ),
            check=False,
            capture_output=True,
            timeout=60,
        )
        if completed.returncode != 0:
            detail = completed.stderr.decode("utf-8", errors="replace").strip()
            raise BundleError(f"cannot sign offline bundle manifest: {detail}")
        os.chmod(signature_path, 0o644)
        os.replace(staging, output)
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    return output
