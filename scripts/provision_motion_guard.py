#!/usr/bin/env python3
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import pwd
import stat
import sys
import tempfile
import uuid
from typing import Any, Dict, Iterable, List, Optional, Tuple


PROVISION_FIELDS = ("instance_id", "schema_version")
# Schema v4 is the single-user durable format. Legacy field tuples remain
# only to verify records that must migrate during the next guard boot.
EPOCH_FIELDS = (
    "schema_version",
    "generation",
    "boot_nonce_hex",
    "lease_state",
    "last_zero_ack_id",
    "actuator_symbols_sha256",
    "fenced",
    "startup_zero_confirmed",
    "recovery_fence_token_hex",
    "recovery_fence_correlation_sha256",
    "created_at",
)
V3_EPOCH_FIELDS = (
    "schema_version",
    "generation",
    "boot_nonce_hex",
    "lease_state",
    "last_zero_ack_id",
    "peer_uid_allowlist",
    "actuator_symbols_sha256",
    "fenced",
    "startup_zero_confirmed",
    "recovery_fence_token_hex",
    "recovery_fence_correlation_sha256",
    "created_at",
)
V2_EPOCH_FIELDS = tuple(
    field
    for field in V3_EPOCH_FIELDS
    if field not in {"recovery_fence_token_hex", "recovery_fence_correlation_sha256"}
)
V1_EPOCH_FIELDS = (
    "schema_version",
    "generation",
    "boot_nonce_hex",
    "safety_gate",
    "lease_state",
    "reset_generation",
    "last_zero_ack_id",
    "peer_uid_allowlist",
    "actuator_symbols_sha256",
    "fenced",
    "created_at",
)
MAX_GENERATION = (1 << 64) - 1


def _compact_json(value: Dict[str, Any]) -> str:
    return json.dumps(value, ensure_ascii=True, separators=(",", ":"))


def _checksum(body: Dict[str, Any]) -> str:
    digest = hashlib.sha256(_compact_json(body).encode("utf-8")).hexdigest()
    return f"sha256:{digest}"


def _seal(body: Dict[str, Any]) -> Dict[str, Any]:
    return {**body, "checksum": _checksum(body)}


def build_provisioned_record(instance_id: str) -> Dict[str, Any]:
    parsed = uuid.UUID(instance_id)
    if parsed.version != 4 or str(parsed) != instance_id:
        raise ValueError("instance_id must be a lowercase hyphenated UUIDv4")
    return _seal({"instance_id": instance_id, "schema_version": 1})


def build_initial_epoch_record(
    actuator_symbols_sha256: str,
    created_at: str,
) -> Dict[str, Any]:
    if not actuator_symbols_sha256.startswith("sha256:"):
        raise ValueError("actuator_symbols_sha256 must use sha256:<hex>")
    digest = actuator_symbols_sha256[len("sha256:") :]
    if len(digest) != 64 or any(ch not in "0123456789abcdef" for ch in digest):
        raise ValueError("actuator_symbols_sha256 must contain 64 lowercase hex chars")
    body = {
        "schema_version": 4,
        "generation": 0,
        "boot_nonce_hex": "",
        "lease_state": "Idle",
        "last_zero_ack_id": "",
        "actuator_symbols_sha256": actuator_symbols_sha256,
        "fenced": True,
        "startup_zero_confirmed": False,
        "recovery_fence_token_hex": "",
        "recovery_fence_correlation_sha256": "",
        "created_at": created_at,
    }
    return _seal(body)


def _validate_record(
    record: Dict[str, Any],
    expected_fields: Iterable[str],
) -> None:
    fields = tuple(expected_fields)
    if tuple(record) != (*fields, "checksum"):
        raise ValueError(f"record fields/order drift: got={tuple(record)}, expected={(*fields, 'checksum')}")
    body = {field: record[field] for field in fields}
    expected_checksum = _checksum(body)
    if record["checksum"] != expected_checksum:
        raise ValueError(f"checksum mismatch: got={record['checksum']}, expected={expected_checksum}")


def _load_record(path: Path, fields: Iterable[str]) -> Dict[str, Any]:
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"record must be a non-symlink regular file: {path}")
    raw = path.read_text(encoding="utf-8")

    def reject_duplicate_keys(pairs: List[Tuple[str, Any]]) -> Dict[str, Any]:
        result: Dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"duplicate JSON field {key!r}: {path}")
            result[key] = value
        return result

    record = json.loads(raw, object_pairs_hook=reject_duplicate_keys)
    if not isinstance(record, dict):
        raise ValueError(f"record is not an object: {path}")
    _validate_record(record, fields)
    return record


def _load_epoch_record(path: Path) -> Dict[str, Any]:
    preview = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(preview, dict):
        raise ValueError(f"record is not an object: {path}")
    schema_version = preview.get("schema_version")
    if schema_version == 1:
        return _load_record(path, V1_EPOCH_FIELDS)
    if schema_version == 2:
        return _load_record(path, V2_EPOCH_FIELDS)
    if schema_version == 3:
        return _load_record(path, V3_EPOCH_FIELDS)
    if schema_version == 4:
        return _load_record(path, EPOCH_FIELDS)
    raise ValueError("unsupported durable record schema version")


def _write_new_record(
    path: Path,
    record: Dict[str, Any],
    mode: int,
    uid: int,
    gid: int,
) -> None:
    payload = (_compact_json(record) + "\n").encode("utf-8")
    fd, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    temporary_path = Path(temporary_name)
    try:
        os.fchmod(fd, mode)
        os.fchown(fd, uid, gid)
        with os.fdopen(fd, "wb", closefd=True) as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        fd = -1
        os.link(temporary_path, path)
        temporary_path.unlink()
        directory_fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    finally:
        if fd >= 0:
            os.close(fd)
        temporary_path.unlink(missing_ok=True)


def _replace_record(
    path: Path,
    record: Dict[str, Any],
    mode: int,
    uid: int,
    gid: int,
) -> None:
    payload = (_compact_json(record) + "\n").encode("utf-8")
    fd, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    temporary_path: Optional[Path] = Path(temporary_name)
    try:
        os.fchmod(fd, mode)
        os.fchown(fd, uid, gid)
        with os.fdopen(fd, "wb", closefd=True) as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        fd = -1
        os.replace(str(temporary_path), path)
        temporary_path = None
        directory_fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    finally:
        if fd >= 0:
            os.close(fd)
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)


def _owner_ids(owner: str) -> Tuple[int, int]:
    account = pwd.getpwnam(owner)
    return account.pw_uid, account.pw_gid


def _manifest_digest(path: Path) -> str:
    data = path.read_bytes()
    if not data:
        raise ValueError(f"actuator symbols manifest is empty: {path}")
    return f"sha256:{hashlib.sha256(data).hexdigest()}"


def _ensure_state_dir(path: Path, uid: int, gid: int) -> None:
    path.mkdir(parents=True, exist_ok=True)
    os.chown(path, uid, gid)
    os.chmod(path, 0o700)


def _require_u64(record: Dict[str, Any], field: str) -> int:
    value = record.get(field)
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"epoch {field} must be an integer")
    if value < 0 or value > MAX_GENERATION:
        raise ValueError(f"epoch {field} is outside u64 range")
    return value


def _validate_epoch_semantics(epoch: Dict[str, Any]) -> None:
    if epoch.get("schema_version") != 4:
        raise ValueError("unsupported durable record schema version")
    _require_u64(epoch, "generation")
    boot_nonce_hex = epoch.get("boot_nonce_hex")
    if not isinstance(boot_nonce_hex, str):
        raise ValueError("epoch boot_nonce_hex must be a string")
    if boot_nonce_hex:
        try:
            nonce = bytes.fromhex(boot_nonce_hex)
        except ValueError as exc:
            raise ValueError("epoch boot_nonce_hex is not hexadecimal") from exc
        if len(nonce) != 16:
            raise ValueError("epoch boot_nonce_hex must be exactly 128 bits")
    if epoch.get("lease_state") not in {"Idle", "Active", "Pausing", "Revoking"}:
        raise ValueError("epoch lease_state is invalid")
    if not isinstance(epoch.get("last_zero_ack_id"), str):
        raise ValueError("epoch last_zero_ack_id must be a string")
    actuator_digest = epoch.get("actuator_symbols_sha256")
    if not isinstance(actuator_digest, str) or not actuator_digest.startswith("sha256:"):
        raise ValueError("epoch actuator_symbols_sha256 is invalid")
    digest = actuator_digest[len("sha256:") :]
    if len(digest) != 64 or any(ch not in "0123456789abcdef" for ch in digest):
        raise ValueError("epoch actuator_symbols_sha256 is invalid")
    if not isinstance(epoch.get("fenced"), bool):
        raise ValueError("epoch fenced must be a boolean")
    if not isinstance(epoch.get("startup_zero_confirmed"), bool):
        raise ValueError("epoch startup_zero_confirmed must be a boolean")
    token_hex = epoch.get("recovery_fence_token_hex")
    correlation_sha256 = epoch.get("recovery_fence_correlation_sha256")
    if not isinstance(token_hex, str) or not isinstance(correlation_sha256, str):
        raise ValueError("epoch recovery fence fields must be strings")
    if bool(token_hex) != bool(correlation_sha256):
        raise ValueError("epoch recovery fence token and correlation must be paired")
    if token_hex:
        try:
            token = bytes.fromhex(token_hex)
        except ValueError as exc:
            raise ValueError("epoch recovery_fence_token_hex is not hexadecimal") from exc
        if len(token) != 32:
            raise ValueError("epoch recovery_fence_token_hex must be exactly 256 bits")
        if len(correlation_sha256) != 64 or any(
            ch not in "0123456789abcdef" for ch in correlation_sha256
        ):
            raise ValueError("epoch recovery_fence_correlation_sha256 is invalid")
    if not isinstance(epoch.get("created_at"), str) or not epoch["created_at"]:
        raise ValueError("epoch created_at is required")


def _validate_legacy_epoch_for_guard_migration(epoch: Dict[str, Any]) -> None:
    version = epoch.get("schema_version")
    if version not in {1, 2, 3}:
        raise ValueError("epoch is not a v1-v3 migration input")
    _require_u64(epoch, "generation")
    if version == 1:
        _require_u64(epoch, "reset_generation")
    boot_nonce_hex = epoch.get("boot_nonce_hex")
    if not isinstance(boot_nonce_hex, str):
        raise ValueError(f"epoch v{version} boot_nonce_hex must be a string")
    if boot_nonce_hex:
        try:
            nonce = bytes.fromhex(boot_nonce_hex)
        except ValueError as exc:
            raise ValueError(f"epoch v{version} boot_nonce_hex is not hexadecimal") from exc
        if len(nonce) != 16:
            raise ValueError(f"epoch v{version} boot_nonce_hex must be exactly 128 bits")
    if epoch.get("lease_state") not in {"Idle", "Active", "Pausing", "Revoking"}:
        raise ValueError(f"epoch v{version} lease_state is invalid")
    if not isinstance(epoch.get("last_zero_ack_id"), str):
        raise ValueError(f"epoch v{version} last_zero_ack_id must be a string")
    actuator_digest = epoch.get("actuator_symbols_sha256")
    if not isinstance(actuator_digest, str) or not actuator_digest.startswith("sha256:"):
        raise ValueError(f"epoch v{version} actuator_symbols_sha256 is invalid")
    digest = actuator_digest[len("sha256:") :]
    if len(digest) != 64 or any(ch not in "0123456789abcdef" for ch in digest):
        raise ValueError(f"epoch v{version} actuator_symbols_sha256 is invalid")
    if not isinstance(epoch.get("fenced"), bool):
        raise ValueError(f"epoch v{version} fenced must be a boolean")
    if version >= 2 and not isinstance(epoch.get("startup_zero_confirmed"), bool):
        raise ValueError(f"epoch v{version} startup_zero_confirmed must be a boolean")
    if version == 3:
        token_hex = epoch.get("recovery_fence_token_hex")
        correlation = epoch.get("recovery_fence_correlation_sha256")
        if not isinstance(token_hex, str) or not isinstance(correlation, str):
            raise ValueError("epoch v3 recovery fence fields must be strings")
    if not isinstance(epoch.get("created_at"), str) or not epoch["created_at"]:
        raise ValueError(f"epoch v{version} created_at is required")


def _verify_path_metadata(
    path: Path,
    *,
    uid: int,
    gid: int,
    mode: int,
    directory: bool = False,
) -> None:
    metadata = path.lstat()
    expected_kind = stat.S_ISDIR if directory else stat.S_ISREG
    if path.is_symlink() or not expected_kind(metadata.st_mode):
        kind = "directory" if directory else "record"
        raise ValueError(f"{kind} must be a non-symlink {'directory' if directory else 'regular file'}: {path}")
    if metadata.st_uid != uid or metadata.st_gid != gid:
        raise ValueError(f"owner drift: {path}")
    if stat.S_IMODE(metadata.st_mode) != mode:
        raise ValueError(f"mode drift: {path}")


def _load_verified_records(
    state_dir: Path,
    manifest: Optional[Path],
    owner: str,
) -> Tuple[int, int, Dict[str, Any], Dict[str, Any]]:
    uid, gid = _owner_ids(owner)
    provision_path = state_dir / "provisioned.json"
    epoch_path = state_dir / "epoch.json"
    provisioned = _load_record(provision_path, PROVISION_FIELDS)
    epoch = _load_epoch_record(epoch_path)

    instance_id = str(uuid.UUID(provisioned["instance_id"]))
    if instance_id != provisioned["instance_id"] or uuid.UUID(instance_id).version != 4:
        raise ValueError("provisioned.instance_id is not a canonical UUIDv4")
    if provisioned["schema_version"] != 1:
        raise ValueError("unsupported durable record schema version")
    if manifest is not None and epoch["actuator_symbols_sha256"] != _manifest_digest(manifest):
        raise ValueError("epoch actuator hash does not match supplied manifest")

    _verify_path_metadata(state_dir, uid=uid, gid=gid, mode=0o700, directory=True)
    _verify_path_metadata(provision_path, uid=uid, gid=gid, mode=0o400)
    _verify_path_metadata(epoch_path, uid=uid, gid=gid, mode=0o600)
    return uid, gid, provisioned, epoch


def provision(
    state_dir: Path, manifest: Optional[Path], owner: str, *, backend: str = "unitree_sdk2py",
) -> None:
    if manifest is None and backend != "motion_control":
        raise ValueError("Go2 provisioning requires --actuator-symbols-file")
    uid, gid = _owner_ids(owner)
    _ensure_state_dir(state_dir, uid, gid)
    provision_path = state_dir / "provisioned.json"
    epoch_path = state_dir / "epoch.json"
    # 标准执行接口没有厂商清单，新实例只使用旧记录格式的内部兼容值。
    actuator_digest = _manifest_digest(manifest) if manifest is not None else "sha256:" + "0" * 64

    if provision_path.exists():
        _load_record(provision_path, PROVISION_FIELDS)
    else:
        record = build_provisioned_record(str(uuid.uuid4()))
        _write_new_record(provision_path, record, 0o400, uid, gid)

    if epoch_path.exists():
        epoch = _load_epoch_record(epoch_path)
        if epoch["schema_version"] in {1, 2, 3}:
            _validate_legacy_epoch_for_guard_migration(epoch)
        else:
            _validate_epoch_semantics(epoch)
        if manifest is not None and epoch["actuator_symbols_sha256"] != actuator_digest:
            raise ValueError("existing epoch actuator_symbols_sha256 does not match supplied manifest")
    else:
        created_at = datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")
        epoch = build_initial_epoch_record(actuator_digest, created_at)
        _write_new_record(epoch_path, epoch, 0o600, uid, gid)

    _verify_path_metadata(state_dir, uid=uid, gid=gid, mode=0o700, directory=True)
    _verify_path_metadata(provision_path, uid=uid, gid=gid, mode=0o400)
    _verify_path_metadata(epoch_path, uid=uid, gid=gid, mode=0o600)


def verify(state_dir: Path, manifest: Optional[Path], owner: str) -> None:
    _, _, _, epoch = _load_verified_records(state_dir, manifest, owner)
    if epoch.get("schema_version") != 4:
        raise ValueError("epoch has not yet been migrated by motion guard startup")
    _validate_epoch_semantics(epoch)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Provision NuwaX motion guard durable records")
    subparsers = parser.add_subparsers(dest="command", required=True)
    for command in ("provision", "verify"):
        child = subparsers.add_parser(command)
        child.add_argument(
            "--state-dir",
            type=Path,
            default=Path("/var/lib/nuwax/motion-guard"),
        )
        child.add_argument("--owner", default="nuwax")
        child.add_argument("--actuator-symbols-file", type=Path)
        child.add_argument(
            "--backend", choices=("unitree_sdk2py", "motion_control"),
            default=os.environ.get("NUWAX_MOTION_GUARD_BACKEND", "").strip() or "unitree_sdk2py",
            help="motion_control 使用标准接口，不需要厂商符号清单。",
        )
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.command == "provision":
            provision(args.state_dir, args.actuator_symbols_file, args.owner, backend=args.backend)
        else:
            verify(args.state_dir, args.actuator_symbols_file, args.owner)
    except (KeyError, OSError, ValueError, json.JSONDecodeError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    print(f"motion guard {args.command}: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
