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
from typing import Any, Dict, List, Mapping, Optional, Tuple
import uuid

from .constants import EPOCH_SCHEMA_VERSION, MAX_GENERATION, PROVISION_SCHEMA_VERSION


class EpochError(RuntimeError):
    pass


class SingleInstanceError(EpochError):
    pass


PROVISION_FIELDS = ("instance_id", "schema_version")
# Schema v4 is the first single-user durable format. Legacy field tuples are
# retained only to verify and migrate existing v1-v3 checksummed records.
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
    if field
    not in {"recovery_fence_token_hex", "recovery_fence_correlation_sha256"}
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

def _reject_duplicate_keys(pairs: List[Tuple[str, Any]]) -> Dict[str, Any]:
    document: Dict[str, Any] = {}
    for key, value in pairs:
        if key in document:
            raise ValueError(f"duplicate JSON key: {key}")
        document[key] = value
    return document


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


def _field(document: Mapping[str, Any], name: str, label: str) -> Any:
    try:
        return document[name]
    except KeyError as exc:
        raise EpochError(f"{label} record is missing field {name}") from exc


def _strict_int(value: Any, name: str, label: str) -> int:
    if type(value) is not int:
        raise EpochError(f"{label} field {name} must be a JSON integer")
    return value


def _strict_bool(value: Any, name: str, label: str) -> bool:
    if type(value) is not bool:
        raise EpochError(f"{label} field {name} must be a JSON boolean")
    return value


def _strict_str(value: Any, name: str, label: str) -> str:
    if type(value) is not str:
        raise EpochError(f"{label} field {name} must be a JSON string")
    return value


def _strict_list(value: Any, name: str, label: str) -> list:
    if type(value) is not list:
        raise EpochError(f"{label} field {name} must be a JSON array")
    return value


def _strict_string_list(value: Any, name: str, label: str) -> list:
    values = _strict_list(value, name, label)
    if any(type(item) is not str for item in values):
        raise EpochError(f"{label} field {name} must contain only JSON strings")
    return values


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
        schema_version = _strict_int(
            _field(document, "schema_version", "provision"),
            "schema_version",
            "provision",
        )
        instance_id = _strict_str(
            _field(document, "instance_id", "provision"),
            "instance_id",
            "provision",
        )
        checksum = _strict_str(
            _field(document, "checksum", "provision"), "checksum", "provision"
        )
        record = cls(
            schema_version=schema_version,
            instance_id=instance_id,
            checksum=checksum,
        )
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
    actuator_symbols_sha256: str
    fenced: bool
    startup_zero_confirmed: bool
    recovery_fence_token_hex: str
    recovery_fence_correlation_sha256: str
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
            "actuator_symbols_sha256": actuator_symbols_sha256,
            "fenced": True,
            "startup_zero_confirmed": False,
            "recovery_fence_token_hex": "",
            "recovery_fence_correlation_sha256": "",
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
        raw = {
            "schema_version": _strict_int(
                _field(document, "schema_version", "epoch"),
                "schema_version",
                "epoch",
            ),
            "generation": _strict_int(
                _field(document, "generation", "epoch"), "generation", "epoch"
            ),
            "boot_nonce_hex": _strict_str(
                _field(document, "boot_nonce_hex", "epoch"),
                "boot_nonce_hex",
                "epoch",
            ),
            "lease_state": _strict_str(
                _field(document, "lease_state", "epoch"), "lease_state", "epoch"
            ),
            "last_zero_ack_id": _strict_str(
                _field(document, "last_zero_ack_id", "epoch"),
                "last_zero_ack_id",
                "epoch",
            ),
            "actuator_symbols_sha256": _strict_str(
                _field(document, "actuator_symbols_sha256", "epoch"),
                "actuator_symbols_sha256",
                "epoch",
            ),
            "fenced": _strict_bool(
                _field(document, "fenced", "epoch"), "fenced", "epoch"
            ),
            "startup_zero_confirmed": _strict_bool(
                _field(document, "startup_zero_confirmed", "epoch"),
                "startup_zero_confirmed",
                "epoch",
            ),
            "recovery_fence_token_hex": _strict_str(
                _field(document, "recovery_fence_token_hex", "epoch"),
                "recovery_fence_token_hex",
                "epoch",
            ),
            "recovery_fence_correlation_sha256": _strict_str(
                _field(document, "recovery_fence_correlation_sha256", "epoch"),
                "recovery_fence_correlation_sha256",
                "epoch",
            ),
            "created_at": _strict_str(
                _field(document, "created_at", "epoch"), "created_at", "epoch"
            ),
        }
        if raw["schema_version"] != EPOCH_SCHEMA_VERSION:
            raise EpochError("unsupported epoch schema_version")
        cls._validate_fields(raw, allow_empty_nonce=allow_empty_nonce)
        return cls(checksum=document["checksum"], **raw)

    @classmethod
    def from_v1_document(cls, document: Mapping[str, Any]) -> "EpochRecord":
        _verify_checksum(document, fields=V1_EPOCH_FIELDS, label="epoch v1")
        label = "epoch v1"
        schema_version = _strict_int(_field(document, "schema_version", label), "schema_version", label)
        if schema_version != 1:
            raise EpochError("epoch v1 schema_version is invalid")
        _strict_string_list(_field(document, "peer_uid_allowlist", label), "peer_uid_allowlist", label)
        raw = {
            "schema_version": EPOCH_SCHEMA_VERSION,
            "generation": _strict_int(_field(document, "generation", label), "generation", label),
            "boot_nonce_hex": _strict_str(_field(document, "boot_nonce_hex", label), "boot_nonce_hex", label),
            "lease_state": _strict_str(_field(document, "lease_state", label), "lease_state", label),
            "last_zero_ack_id": _strict_str(_field(document, "last_zero_ack_id", label), "last_zero_ack_id", label),
            "actuator_symbols_sha256": _strict_str(_field(document, "actuator_symbols_sha256", label), "actuator_symbols_sha256", label),
            "fenced": _strict_bool(_field(document, "fenced", label), "fenced", label),
            "startup_zero_confirmed": False,
            "recovery_fence_token_hex": "",
            "recovery_fence_correlation_sha256": "",
            "created_at": _strict_str(_field(document, "created_at", label), "created_at", label),
        }
        _strict_int(_field(document, "reset_generation", label), "reset_generation", label)
        cls._validate_fields(raw, allow_empty_nonce=True)
        return cls(checksum=_checksum(raw, EPOCH_FIELDS), **raw)

    @classmethod
    def from_v2_document(cls, document: Mapping[str, Any]) -> "EpochRecord":
        _verify_checksum(document, fields=V2_EPOCH_FIELDS, label="epoch v2")
        label = "epoch v2"
        schema_version = _strict_int(_field(document, "schema_version", label), "schema_version", label)
        if schema_version != 2:
            raise EpochError("epoch v2 schema_version is invalid")
        _strict_string_list(_field(document, "peer_uid_allowlist", label), "peer_uid_allowlist", label)
        raw = {
            "schema_version": EPOCH_SCHEMA_VERSION,
            "generation": _strict_int(_field(document, "generation", label), "generation", label),
            "boot_nonce_hex": _strict_str(_field(document, "boot_nonce_hex", label), "boot_nonce_hex", label),
            "lease_state": _strict_str(_field(document, "lease_state", label), "lease_state", label),
            "last_zero_ack_id": _strict_str(_field(document, "last_zero_ack_id", label), "last_zero_ack_id", label),
            "actuator_symbols_sha256": _strict_str(_field(document, "actuator_symbols_sha256", label), "actuator_symbols_sha256", label),
            "fenced": _strict_bool(_field(document, "fenced", label), "fenced", label),
            "startup_zero_confirmed": _strict_bool(_field(document, "startup_zero_confirmed", label), "startup_zero_confirmed", label),
            "recovery_fence_token_hex": "",
            "recovery_fence_correlation_sha256": "",
            "created_at": _strict_str(_field(document, "created_at", label), "created_at", label),
        }
        cls._validate_fields(raw, allow_empty_nonce=True)
        return cls(checksum=_checksum(raw, EPOCH_FIELDS), **raw)

    @classmethod
    def from_v3_document(cls, document: Mapping[str, Any]) -> "EpochRecord":
        _verify_checksum(document, fields=V3_EPOCH_FIELDS, label="epoch v3")
        label = "epoch v3"
        schema_version = _strict_int(_field(document, "schema_version", label), "schema_version", label)
        if schema_version != 3:
            raise EpochError("epoch v3 schema_version is invalid")
        _strict_string_list(_field(document, "peer_uid_allowlist", label), "peer_uid_allowlist", label)
        raw = {
            "schema_version": EPOCH_SCHEMA_VERSION,
            "generation": _strict_int(_field(document, "generation", label), "generation", label),
            "boot_nonce_hex": _strict_str(_field(document, "boot_nonce_hex", label), "boot_nonce_hex", label),
            "lease_state": _strict_str(_field(document, "lease_state", label), "lease_state", label),
            "last_zero_ack_id": _strict_str(_field(document, "last_zero_ack_id", label), "last_zero_ack_id", label),
            "actuator_symbols_sha256": _strict_str(_field(document, "actuator_symbols_sha256", label), "actuator_symbols_sha256", label),
            "fenced": _strict_bool(_field(document, "fenced", label), "fenced", label),
            "startup_zero_confirmed": _strict_bool(_field(document, "startup_zero_confirmed", label), "startup_zero_confirmed", label),
            "recovery_fence_token_hex": _strict_str(_field(document, "recovery_fence_token_hex", label), "recovery_fence_token_hex", label),
            "recovery_fence_correlation_sha256": _strict_str(
                _field(document, "recovery_fence_correlation_sha256", label),
                "recovery_fence_correlation_sha256",
                label,
            ),
            "created_at": _strict_str(_field(document, "created_at", label), "created_at", label),
        }
        cls._validate_fields(raw, allow_empty_nonce=True)
        return cls(checksum=_checksum(raw, EPOCH_FIELDS), **raw)

    @staticmethod
    def _validate_fields(
        raw: Mapping[str, Any],
        *,
        allow_empty_nonce: bool,
    ) -> None:
        if type(raw["schema_version"]) is not int or raw["schema_version"] != EPOCH_SCHEMA_VERSION:
            raise EpochError("unsupported epoch schema_version")
        generation = raw["generation"]
        if type(generation) is not int or generation < 0 or generation > MAX_GENERATION:
            raise EpochError("generation is outside u64 range")
        nonce = raw["boot_nonce_hex"]
        if type(nonce) is not str:
            raise EpochError("boot_nonce_hex must be a JSON string")
        if nonce or not allow_empty_nonce:
            try:
                nonce_bytes = bytes.fromhex(nonce)
            except ValueError as exc:
                raise EpochError("boot_nonce_hex is not valid hexadecimal") from exc
            if len(nonce_bytes) != 16:
                raise EpochError("boot_nonce_hex must be exactly 128 bits")
        if type(raw["lease_state"]) is not str or raw["lease_state"] not in {"Idle", "Active", "Pausing", "Revoking"}:
            raise EpochError("invalid lease_state")
        digest = raw["actuator_symbols_sha256"]
        if type(digest) is not str or not digest.startswith("sha256:") or len(digest) != 71:
            raise EpochError("actuator_symbols_sha256 is invalid")
        if any(character not in "0123456789abcdef" for character in digest[7:]):
            raise EpochError("actuator_symbols_sha256 is invalid")
        if type(raw["fenced"]) is not bool:
            raise EpochError("fenced must be a boolean")
        if type(raw["startup_zero_confirmed"]) is not bool:
            raise EpochError("startup_zero_confirmed must be a boolean")
        token_hex = raw["recovery_fence_token_hex"]
        correlation_sha256 = raw["recovery_fence_correlation_sha256"]
        if type(token_hex) is not str or type(correlation_sha256) is not str:
            raise EpochError("recovery fence fields must be JSON strings")
        if bool(token_hex) != bool(correlation_sha256):
            raise EpochError("recovery fence token and correlation must be paired")
        if token_hex:
            try:
                token = bytes.fromhex(token_hex)
            except ValueError as exc:
                raise EpochError("recovery_fence_token_hex is not hexadecimal") from exc
            if len(token) != 32:
                raise EpochError("recovery_fence_token_hex must be exactly 256 bits")
            if len(correlation_sha256) != 64 or any(
                character not in "0123456789abcdef"
                for character in correlation_sha256
            ):
                raise EpochError("recovery_fence_correlation_sha256 is invalid")
        if type(raw["created_at"]) is not str or not raw["created_at"]:
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
            "actuator_symbols_sha256": self.actuator_symbols_sha256,
            "fenced": self.fenced,
            "startup_zero_confirmed": self.startup_zero_confirmed,
            "recovery_fence_token_hex": self.recovery_fence_token_hex,
            "recovery_fence_correlation_sha256": self.recovery_fence_correlation_sha256,
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
                document = json.load(stream, object_pairs_hook=_reject_duplicate_keys)
        except FileNotFoundError as exc:
            raise EpochError(f"{label} record is missing: {path}") from exc
        except OSError as exc:
            if exc.errno in {errno.ELOOP, errno.ENOTDIR}:
                raise EpochError(f"{label} record must be a non-symlink regular file") from exc
            raise EpochError(f"cannot read {label} record") from exc
        except (UnicodeError, json.JSONDecodeError, ValueError) as exc:
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
        schema_version = document.get("schema_version")
        if schema_version in {1, 2, 3}:
            migrated = {
                1: EpochRecord.from_v1_document,
                2: EpochRecord.from_v2_document,
                3: EpochRecord.from_v3_document,
            }[schema_version](document)
            backup_path = self.epoch_path.with_name(
                self.epoch_path.name + f".v{schema_version}.bak"
            )
            if not backup_path.exists():
                atomic_write_record(backup_path, document, mode=0o600)
            EpochRecord.from_document(
                migrated.to_document(),
                allow_empty_nonce=True,
            )
            atomic_write_record(
                self.epoch_path,
                migrated.to_document(),
                mode=0o600,
            )
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
