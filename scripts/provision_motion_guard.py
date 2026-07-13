#!/usr/bin/env python3
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import pwd
import sys
import tempfile
import uuid
from typing import Any, Iterable


PROVISION_FIELDS = ("instance_id", "schema_version")
EPOCH_FIELDS = (
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
PEER_UID_ALLOWLIST = (
    "nuwax-motion-guard",
    "nuwax-motion-admin",
    "nuwax-geometry",
    "nuwax-gateway",
)


def _compact_json(value: dict[str, Any]) -> str:
    return json.dumps(value, ensure_ascii=True, separators=(",", ":"))


def _checksum(body: dict[str, Any]) -> str:
    digest = hashlib.sha256(_compact_json(body).encode("utf-8")).hexdigest()
    return f"sha256:{digest}"


def _seal(body: dict[str, Any]) -> dict[str, Any]:
    return {**body, "checksum": _checksum(body)}


def build_provisioned_record(instance_id: str) -> dict[str, Any]:
    parsed = uuid.UUID(instance_id)
    if parsed.version != 4 or str(parsed) != instance_id:
        raise ValueError("instance_id must be a lowercase hyphenated UUIDv4")
    return _seal({"instance_id": instance_id, "schema_version": 1})


def build_initial_epoch_record(
    actuator_symbols_sha256: str,
    created_at: str,
) -> dict[str, Any]:
    if not actuator_symbols_sha256.startswith("sha256:"):
        raise ValueError("actuator_symbols_sha256 must use sha256:<hex>")
    digest = actuator_symbols_sha256.removeprefix("sha256:")
    if len(digest) != 64 or any(ch not in "0123456789abcdef" for ch in digest):
        raise ValueError("actuator_symbols_sha256 must contain 64 lowercase hex chars")
    body = {
        "schema_version": 1,
        "generation": 0,
        "boot_nonce_hex": "",
        "safety_gate": "ResetRequired",
        "lease_state": "Idle",
        "reset_generation": 0,
        "last_zero_ack_id": "",
        "peer_uid_allowlist": list(PEER_UID_ALLOWLIST),
        "actuator_symbols_sha256": actuator_symbols_sha256,
        "fenced": True,
        "created_at": created_at,
    }
    return _seal(body)


def _validate_record(
    record: dict[str, Any],
    expected_fields: Iterable[str],
) -> None:
    fields = tuple(expected_fields)
    if tuple(record) != (*fields, "checksum"):
        raise ValueError(
            f"record fields/order drift: got={tuple(record)}, expected={(*fields, 'checksum')}"
        )
    body = {field: record[field] for field in fields}
    expected_checksum = _checksum(body)
    if record["checksum"] != expected_checksum:
        raise ValueError(
            f"checksum mismatch: got={record['checksum']}, expected={expected_checksum}"
        )


def _load_record(path: Path, fields: Iterable[str]) -> dict[str, Any]:
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"record must be a non-symlink regular file: {path}")
    raw = path.read_text(encoding="utf-8")

    def reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
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


def _write_new_record(
    path: Path,
    record: dict[str, Any],
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


def _owner_ids(owner: str) -> tuple[int, int]:
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


def provision(state_dir: Path, manifest: Path, owner: str) -> None:
    uid, gid = _owner_ids(owner)
    _ensure_state_dir(state_dir, uid, gid)
    provision_path = state_dir / "provisioned.json"
    epoch_path = state_dir / "epoch.json"
    actuator_digest = _manifest_digest(manifest)

    if provision_path.exists():
        _load_record(provision_path, PROVISION_FIELDS)
    else:
        record = build_provisioned_record(str(uuid.uuid4()))
        _write_new_record(provision_path, record, 0o400, uid, gid)

    if epoch_path.exists():
        epoch = _load_record(epoch_path, EPOCH_FIELDS)
        if epoch["actuator_symbols_sha256"] != actuator_digest:
            raise ValueError(
                "existing epoch actuator_symbols_sha256 does not match supplied manifest"
            )
    else:
        created_at = datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace(
            "+00:00", "Z"
        )
        epoch = build_initial_epoch_record(actuator_digest, created_at)
        _write_new_record(epoch_path, epoch, 0o600, uid, gid)

    verify(state_dir, manifest, owner)


def verify(state_dir: Path, manifest: Path | None, owner: str) -> None:
    uid, gid = _owner_ids(owner)
    provision_path = state_dir / "provisioned.json"
    epoch_path = state_dir / "epoch.json"
    provisioned = _load_record(provision_path, PROVISION_FIELDS)
    epoch = _load_record(epoch_path, EPOCH_FIELDS)

    instance_id = str(uuid.UUID(provisioned["instance_id"]))
    if instance_id != provisioned["instance_id"] or uuid.UUID(instance_id).version != 4:
        raise ValueError("provisioned.instance_id is not a canonical UUIDv4")
    if provisioned["schema_version"] != 1 or epoch["schema_version"] != 1:
        raise ValueError("unsupported durable record schema version")
    if epoch["peer_uid_allowlist"] != list(PEER_UID_ALLOWLIST):
        raise ValueError("epoch peer_uid_allowlist drift")
    if manifest is not None and epoch["actuator_symbols_sha256"] != _manifest_digest(manifest):
        raise ValueError("epoch actuator hash does not match supplied manifest")

    state_stat = state_dir.stat()
    if state_stat.st_uid != uid or state_stat.st_gid != gid:
        raise ValueError(f"state directory owner drift: {state_dir}")
    if state_stat.st_mode & 0o777 != 0o700:
        raise ValueError(f"state directory mode drift: {state_dir}")

    for path, mode in ((provision_path, 0o400), (epoch_path, 0o600)):
        stat_result = path.stat()
        if stat_result.st_uid != uid or stat_result.st_gid != gid:
            raise ValueError(f"record owner drift: {path}")
        if stat_result.st_mode & 0o777 != mode:
            raise ValueError(f"record mode drift: {path}")


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
        child.add_argument("--owner", default="nuwax-motion-guard")
        child.add_argument("--actuator-symbols-file", type=Path)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.command == "provision":
            if args.actuator_symbols_file is None:
                raise ValueError("provision requires --actuator-symbols-file")
            provision(args.state_dir, args.actuator_symbols_file, args.owner)
        else:
            verify(args.state_dir, args.actuator_symbols_file, args.owner)
    except (KeyError, OSError, ValueError, json.JSONDecodeError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    print(f"motion guard {args.command}: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
