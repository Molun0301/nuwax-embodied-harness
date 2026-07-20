from __future__ import annotations

from dataclasses import dataclass, replace
from datetime import datetime, timezone
import errno
import fcntl
import hashlib
import json
import os
from pathlib import Path
import secrets
import stat
import tempfile
from typing import Any, Dict, Mapping, Optional, Tuple
import uuid

from .constants import EPOCH_SCHEMA_VERSION, MAX_GENERATION, PROVISION_SCHEMA_VERSION


class EpochError(RuntimeError):
    pass


class SingleInstanceError(EpochError):
    pass


PROVISION_FIELDS = ("instance_id", "schema_version")
EPOCH_FIELDS = (
    "schema_version",
    "generation",
    "boot_nonce_hex",
    "lease_state",
    "last_zero_ack_id",
    "peer_uid_allowlist",
    "actuator_symbols_sha256",
    "fenced",
    "startup_zero_confirmed",
    "created_at",
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
EXPECTED_USERS = (
    "nuwax-motion-guard",
    "nuwax-navigation",
    "nuwax-gateway",
)
V1_EXPECTED_USER_SETS = (
    (
        "nuwax-motion-guard",
        "nuwax-motion-admin",
        "nuwax-navigation",
        "nuwax-gateway",
    ),
    (
        "nuwax-motion-guard",
        "nuwax-motion-admin",
        "nuwax-geometry",
        "nuwax-gateway",
    ),
)


def _canonical_bytes(document: Mapping[str, Any], fields: Tuple[str, ...]) -> bytes:
    try:
        body = {field: document[field] for field in fields}
    except KeyError as exc:
        raise EpochError(f"record is missing field {exc.args[0]}") from exc
    return json.dumps(
        body,
        ensure_ascii=True,
        separators=(",", ":"),
    ).encode("ascii")


def _checksum(document: Mapping[str, Any], fields: Tuple[str, ...]) -> str:
    return "sha256:" + hashlib.sha256(_canonical_bytes(document, fields)).hexdigest()


def _verify_checksum(
    document: Mapping[str, Any],
    *,
    fields: Tuple[str, ...],
    label: str,
) -> None:
    if set(document) != {*fields, "checksum"}:
        raise EpochError(f"{label} record fields drift")
    checksum = document.get("checksum")
    if not isinstance(checksum, str) or checksum != _checksum(document, fields):
        raise EpochError(f"{label} checksum mismatch")


@dataclass(frozen=True)
class ProvisionRecord:
    instance_id: str
    schema_version: int
    checksum: str = ""

    @classmethod
    def create(cls, *, instance_id: str) -> "ProvisionRecord":
        normalized = instance_id.strip()
        try:
            parsed = uuid.UUID(normalized)
        except ValueError as exc:
            raise EpochError("provision instance_id must be a UUIDv4") from exc
        if parsed.version != 4 or str(parsed) != normalized:
            raise EpochError("provision instance_id must be a canonical UUIDv4")
        raw = {
            "instance_id": normalized,
            "schema_version": PROVISION_SCHEMA_VERSION,
        }
        return cls(checksum=_checksum(raw, PROVISION_FIELDS), **raw)

    @classmethod
    def from_document(cls, document: Mapping[str, Any]) -> "ProvisionRecord":
        _verify_checksum(document, fields=PROVISION_FIELDS, label="provision")
        try:
            record = cls(
                schema_version=int(document["schema_version"]),
                instance_id=str(document["instance_id"]),
                checksum=str(document["checksum"]),
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise EpochError("invalid provision record") from exc
        if record.schema_version != PROVISION_SCHEMA_VERSION:
            raise EpochError("unsupported provision schema_version")
        try:
            parsed = uuid.UUID(record.instance_id)
        except ValueError as exc:
            raise EpochError("provision instance_id must be a UUIDv4") from exc
        if parsed.version != 4 or str(parsed) != record.instance_id:
            raise EpochError("provision instance_id must be a canonical UUIDv4")
        return record

    def to_document(self) -> Dict[str, Any]:
        return {
            "instance_id": self.instance_id,
            "schema_version": self.schema_version,
            "checksum": self.checksum,
        }


@dataclass(frozen=True)
class EpochRecord:
    schema_version: int
    generation: int
    boot_nonce_hex: str
    lease_state: str
    last_zero_ack_id: str
    peer_uid_allowlist: Tuple[str, ...]
    actuator_symbols_sha256: str
    fenced: bool
    startup_zero_confirmed: bool
    created_at: str
    checksum: str = ""

    @classmethod
    def initial(
        cls,
        *,
        generation: int = 0,
        actuator_symbols_sha256: str = "sha256:" + "0" * 64,
        created_at: str = "1970-01-01T00:00:00.000Z",
    ) -> "EpochRecord":
        raw = {
            "schema_version": EPOCH_SCHEMA_VERSION,
            "generation": generation,
            "boot_nonce_hex": "",
            "lease_state": "Idle",
            "last_zero_ack_id": "",
            "peer_uid_allowlist": EXPECTED_USERS,
            "actuator_symbols_sha256": actuator_symbols_sha256,
            "fenced": True,
            "startup_zero_confirmed": False,
            "created_at": created_at,
        }
        cls._validate_fields(raw, allow_empty_nonce=True)
        return cls(checksum=_checksum(raw, EPOCH_FIELDS), **raw)

    @classmethod
    def from_document(
        cls,
        document: Mapping[str, Any],
        *,
        allow_empty_nonce: bool = True,
    ) -> "EpochRecord":
        _verify_checksum(document, fields=EPOCH_FIELDS, label="epoch")
        try:
            raw = {
                "schema_version": int(document["schema_version"]),
                "generation": int(document["generation"]),
                "boot_nonce_hex": str(document["boot_nonce_hex"]),
                "lease_state": str(document["lease_state"]),
                "last_zero_ack_id": str(document["last_zero_ack_id"]),
                "peer_uid_allowlist": tuple(document["peer_uid_allowlist"]),
                "actuator_symbols_sha256": str(document["actuator_symbols_sha256"]),
                "fenced": bool(document["fenced"]),
                "startup_zero_confirmed": document["startup_zero_confirmed"],
                "created_at": str(document["created_at"]),
            }
        except (KeyError, TypeError, ValueError) as exc:
            raise EpochError("invalid epoch record") from exc
        cls._validate_fields(raw, allow_empty_nonce=allow_empty_nonce)
        return cls(checksum=str(document["checksum"]), **raw)

    @classmethod
    def from_v1_document(cls, document: Mapping[str, Any]) -> "EpochRecord":
        _verify_checksum(document, fields=V1_EPOCH_FIELDS, label="epoch v1")
        try:
            users = tuple(document["peer_uid_allowlist"])
            if users not in V1_EXPECTED_USER_SETS:
                raise EpochError("epoch v1 peer_uid_allowlist drift")
            raw = {
                "schema_version": EPOCH_SCHEMA_VERSION,
                "generation": int(document["generation"]),
                "boot_nonce_hex": str(document["boot_nonce_hex"]),
                "lease_state": str(document["lease_state"]),
                "last_zero_ack_id": str(document["last_zero_ack_id"]),
                "peer_uid_allowlist": EXPECTED_USERS,
                "actuator_symbols_sha256": str(
                    document["actuator_symbols_sha256"]
                ),
                "fenced": bool(document["fenced"]),
                "startup_zero_confirmed": False,
                "created_at": str(document["created_at"]),
            }
        except (KeyError, TypeError, ValueError) as exc:
            raise EpochError("invalid epoch v1 record") from exc
        cls._validate_fields(raw, allow_empty_nonce=True)
        return cls(checksum=_checksum(raw, EPOCH_FIELDS), **raw)

    @staticmethod
    def _validate_fields(
        raw: Mapping[str, Any],
        *,
        allow_empty_nonce: bool,
    ) -> None:
        if raw["schema_version"] != EPOCH_SCHEMA_VERSION:
            raise EpochError("unsupported epoch schema_version")
        generation = int(raw["generation"])
        if generation < 0 or generation > MAX_GENERATION:
            raise EpochError("generation is outside u64 range")
        nonce = str(raw["boot_nonce_hex"])
        if nonce or not allow_empty_nonce:
            try:
                nonce_bytes = bytes.fromhex(nonce)
            except ValueError as exc:
                raise EpochError("boot_nonce_hex is not valid hexadecimal") from exc
            if len(nonce_bytes) != 16:
                raise EpochError("boot_nonce_hex must be exactly 128 bits")
        if raw["lease_state"] not in {"Idle", "Active", "Pausing", "Revoking"}:
            raise EpochError("invalid lease_state")
        peer_uid_allowlist = tuple(raw["peer_uid_allowlist"])
        if peer_uid_allowlist != EXPECTED_USERS:
            raise EpochError("peer_uid_allowlist drift")
        digest = str(raw["actuator_symbols_sha256"])
        if not digest.startswith("sha256:") or len(digest) != 71:
            raise EpochError("actuator_symbols_sha256 is invalid")
        if any(character not in "0123456789abcdef" for character in digest[7:]):
            raise EpochError("actuator_symbols_sha256 is invalid")
        if not isinstance(raw["fenced"], bool):
            raise EpochError("fenced must be a boolean")
        if not isinstance(raw["startup_zero_confirmed"], bool):
            raise EpochError("startup_zero_confirmed must be a boolean")
        if not str(raw["created_at"]):
            raise EpochError("created_at is required")

    def with_updates(self, **updates: Any) -> "EpochRecord":
        provisional = replace(self, checksum="", **updates)
        document = provisional.to_document()
        document["checksum"] = _checksum(document, EPOCH_FIELDS)
        return EpochRecord.from_document(document, allow_empty_nonce=False)

    @property
    def boot_nonce_bytes(self) -> bytes:
        return bytes.fromhex(self.boot_nonce_hex)

    def to_document(self) -> Dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "generation": self.generation,
            "boot_nonce_hex": self.boot_nonce_hex,
            "lease_state": self.lease_state,
            "last_zero_ack_id": self.last_zero_ack_id,
            "peer_uid_allowlist": list(self.peer_uid_allowlist),
            "actuator_symbols_sha256": self.actuator_symbols_sha256,
            "fenced": self.fenced,
            "startup_zero_confirmed": self.startup_zero_confirmed,
            "created_at": self.created_at,
            "checksum": self.checksum,
        }


def atomic_write_record(path: Path, document: Mapping[str, Any], *, mode: int) -> None:
    path = Path(path)
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    encoded = (json.dumps(document, ensure_ascii=True, separators=(",", ":")) + "\n").encode("ascii")
    temp_path: Optional[str] = None
    try:
        fd, temp_path = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
        try:
            os.fchmod(fd, mode)
            stream = os.fdopen(fd, "wb", closefd=True)
            fd = -1
            with stream:
                stream.write(encoded)
                stream.flush()
                os.fsync(stream.fileno())
        finally:
            if fd >= 0:
                os.close(fd)
        os.replace(temp_path, path)
        temp_path = None
        os.chmod(path, mode)
        directory_fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    finally:
        if temp_path is not None:
            try:
                os.unlink(temp_path)
            except FileNotFoundError:
                pass


class EpochStore:
    def __init__(self, *, provision_path: Path, epoch_path: Path) -> None:
        self.provision_path = Path(provision_path)
        self.epoch_path = Path(epoch_path)

    @staticmethod
    def _load_document(path: Path, *, expected_mode: int, label: str) -> Dict[str, Any]:
        flags = os.O_RDONLY | os.O_CLOEXEC
        if hasattr(os, "O_NOFOLLOW"):
            flags |= os.O_NOFOLLOW
        fd = -1
        try:
            fd = os.open(path, flags)
            metadata = os.fstat(fd)
            if not stat.S_ISREG(metadata.st_mode):
                raise EpochError(f"{label} record is not a regular file")
            if metadata.st_uid != os.geteuid():
                raise EpochError(f"{label} record owner UID {metadata.st_uid} does not match guard UID {os.geteuid()}")
            actual_mode = stat.S_IMODE(metadata.st_mode)
            if actual_mode != expected_mode:
                raise EpochError(f"{label} record mode is {actual_mode:04o}, expected {expected_mode:04o}")
            stream = os.fdopen(fd, "r", encoding="ascii", closefd=True)
            fd = -1
            with stream:
                document = json.load(stream)
        except FileNotFoundError as exc:
            raise EpochError(f"{label} record is missing: {path}") from exc
        except OSError as exc:
            if exc.errno in {errno.ELOOP, errno.ENOTDIR}:
                raise EpochError(f"{label} record must be a non-symlink regular file") from exc
            raise EpochError(f"cannot read {label} record") from exc
        except (UnicodeError, json.JSONDecodeError) as exc:
            raise EpochError(f"invalid {label} JSON") from exc
        finally:
            if fd >= 0:
                os.close(fd)
        if not isinstance(document, dict):
            raise EpochError(f"invalid {label} JSON object")
        return document

    def load_provision(self) -> ProvisionRecord:
        document = self._load_document(
            self.provision_path,
            expected_mode=0o400,
            label="provision",
        )
        return ProvisionRecord.from_document(document)

    def load_epoch(self) -> EpochRecord:
        document = self._load_document(
            self.epoch_path,
            expected_mode=0o600,
            label="epoch",
        )
        if document.get("schema_version") == 1:
            backup_path = self.epoch_path.with_name(
                self.epoch_path.name + ".v1.bak"
            )
            if not backup_path.exists():
                atomic_write_record(backup_path, document, mode=0o600)
            migrated = EpochRecord.from_v1_document(document)
            self.persist(migrated)
            return migrated
        return EpochRecord.from_document(document)

    def persist(self, record: EpochRecord) -> None:
        EpochRecord.from_document(record.to_document(), allow_empty_nonce=False)
        atomic_write_record(self.epoch_path, record.to_document(), mode=0o600)

    def advance_boot(self) -> EpochRecord:
        self.load_provision()
        previous = self.load_epoch()
        if previous.generation == MAX_GENERATION:
            raise EpochError("generation overflow")
        advanced = previous.with_updates(
            generation=previous.generation + 1,
            boot_nonce_hex=secrets.token_hex(16),
            lease_state="Idle",
            fenced=True,
            startup_zero_confirmed=False,
            created_at=datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z"),
        )
        self.persist(advanced)
        return advanced


class SingleInstanceLock:
    def __init__(self, path: Path) -> None:
        self.path = Path(path)
        self._fd: Optional[int] = None

    def acquire(self) -> None:
        if self._fd is not None:
            return
        self.path.parent.mkdir(mode=0o750, parents=True, exist_ok=True)
        flags = os.O_RDWR | os.O_CREAT | os.O_CLOEXEC
        if hasattr(os, "O_NOFOLLOW"):
            flags |= os.O_NOFOLLOW
        try:
            fd = os.open(self.path, flags, 0o600)
        except OSError as exc:
            if exc.errno in {errno.ELOOP, errno.ENOTDIR}:
                raise SingleInstanceError(f"lock path must be a non-symlink regular file: {self.path}") from exc
            raise SingleInstanceError(f"cannot open guard lock {self.path}") from exc
        try:
            metadata = os.fstat(fd)
            if not stat.S_ISREG(metadata.st_mode):
                raise SingleInstanceError(f"lock path is not a regular file: {self.path}")
            if metadata.st_uid != os.geteuid():
                raise SingleInstanceError(f"lock owner UID {metadata.st_uid} does not match guard UID {os.geteuid()}")
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            os.close(fd)
            raise SingleInstanceError(f"another motion guard owns {self.path}") from exc
        except BaseException:
            os.close(fd)
            raise
        self._fd = fd

    def release(self) -> None:
        if self._fd is None:
            return
        fd, self._fd = self._fd, None
        try:
            fcntl.flock(fd, fcntl.LOCK_UN)
        finally:
            os.close(fd)

    def __enter__(self) -> "SingleInstanceLock":
        self.acquire()
        return self

    def __exit__(self, *_exc_info: object) -> None:
        self.release()
