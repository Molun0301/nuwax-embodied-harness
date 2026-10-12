"""Dedicated lifecycle owner for bounded offline mapping CLI processes."""

from __future__ import annotations

import asyncio
import fcntl
import hashlib
import hmac
import json
import math
import os
import re
import select
import signal
import stat
import subprocess
import sys
import threading
import time
import uuid
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import (
    Any,
    BinaryIO,
    ClassVar,
    Dict,
    List,
    Mapping,
    Optional,
    Sequence,
    Tuple,
    Union,
)


REQUEST_SCHEMA_VERSION = 4
LEGACY_REQUEST_SCHEMA_VERSION = 3
ACTIVE_SCHEMA_VERSION = 2
LEGACY_ACTIVE_SCHEMA_VERSION = 1
STATE_ENVELOPE_SCHEMA_VERSION = 1
DEFAULT_MAX_LOG_BYTES = 64 * 1024
DEFAULT_STARTUP_TIMEOUT_SECONDS = 5.0
DEFAULT_TERMINATION_GRACE_SECONDS = 0.5
DEFAULT_KILL_GRACE_SECONDS = 1.0
DEFAULT_POLL_INTERVAL_SECONDS = 0.01
MAX_REQUEST_BYTES = 32 * 1024
MAX_STATE_BYTES = 256 * 1024
MAX_CANONICAL_MAP_JSON_BYTES = MAX_STATE_BYTES
MAX_MAP_GRID_DIMENSION = 0xFFFFFFFF
MAX_PGM_HEADER_BYTES = len(b"P5\n") + 10 + 1 + 10 + len(b"\n255\n")
MAX_MAP_GRID_CELLS = 8_000_000
MAX_PGM_FILE_BYTES = MAX_PGM_HEADER_BYTES + MAX_MAP_GRID_CELLS
MAX_MESSAGE_BYTES = 4096
MAX_PATH_BYTES = 1024
MAX_ITEMS = 64
MAX_REQUEST_RECORDS = 1024
MAX_STATE_ENTRIES = 1536
MAX_INPUT_ALIASES = 64
MAX_TIMEOUT_SECONDS = 24 * 60 * 60
MAX_CONFIGURED_LOG_BYTES = 96 * 1024
MIN_STATE_AUTH_KEY_BYTES = 32
MAX_STATE_AUTH_KEY_BYTES = 4096
OWNER_LOCK_NAME = ".nuwax-offline-cli-owner-v1.lock"
ACTIVE_JOURNAL_NAME = ".nuwax-offline-cli-active-v1.json"
LEGACY_REQUEST_FILE_PREFIX = ".nuwax-offline-cli-request-v1-"
REQUEST_FILE_PREFIX = ".nuwax-offline-cli-request-v2-"
TEMP_FILE_PREFIX = ".nuwax-offline-cli-temp-v1-"
INPUT_ALIAS_DIR_NAME = ".nuwax-offline-cli-input-aliases-v1"
INPUT_ALIAS_PREFIX = ".nuwax-offline-cli-input-v1-"

_REQUEST_ID_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.:-]{0,127}\Z")
_TOKEN_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_./:-]{0,127}\Z")
_SHA256_RE = re.compile(r"[0-9a-f]{64}\Z")
_OPERATION_ID_RE = re.compile(r"[0-9a-f]{32}\Z")
_STATE_MAC_RE = re.compile(r"[0-9a-f]{64}\Z")
_LEGACY_REQUEST_FILE_RE = re.compile(
    re.escape(LEGACY_REQUEST_FILE_PREFIX) + r"([0-9a-f]{64})\.json\Z"
)
_REQUEST_FILE_RE = re.compile(
    re.escape(REQUEST_FILE_PREFIX) + r"([A-Za-z0-9][A-Za-z0-9_.:-]{0,127})\.json\Z"
)
_INPUT_ALIAS_RE = re.compile(
    re.escape(INPUT_ALIAS_PREFIX)
    + r"([1-9][0-9]*)-([0-9a-f]{16})-([0-9a-f]{32})(\.jsonl|\.ndjson|\.db)\Z"
)
_FD_ALIAS_TARGET_RE = re.compile(r"/proc/self/fd/(0|[1-9][0-9]*)\Z")
_TEMP_NAME_RE = re.compile(r"\.nuwax-offline-cli-temp-v1-([1-9][0-9]*)-([0-9a-f]{16})-[0-9a-f]{32}\.json\Z")
_STATE_AUTH_DOMAIN = b"nuwax-offline-cli-owner-state-v1"
_LAUNCHER_CODE = r"""
import ctypes
import os
import signal
import sys

expected_parent_pid = int(sys.argv[1])
ready_fd = int(sys.argv[2])
gate_fd = int(sys.argv[3])
binary_fd = int(sys.argv[4])
input_alias_dir_fd = int(sys.argv[5])
input_file_fd = int(sys.argv[6])
output_parent_fd = int(sys.argv[7])
binary_argv0 = sys.argv[8]
if os.getppid() != expected_parent_pid:
    os._exit(124)
libc = ctypes.CDLL(None, use_errno=True)
if libc.prctl(1, signal.SIGKILL, 0, 0, 0) != 0:
    os._exit(124)
if os.getppid() != expected_parent_pid:
    os._exit(124)
try:
    if os.write(ready_fd, b"1") != 1:
        os._exit(124)
except OSError:
    os._exit(124)
finally:
    os.close(ready_fd)
try:
    gate = os.read(gate_fd, 1)
finally:
    os.close(gate_fd)
if gate != b"1":
    os._exit(125)
if os.getppid() != expected_parent_pid:
    os._exit(124)
os.fstat(input_alias_dir_fd)
os.fstat(input_file_fd)
os.fchdir(output_parent_fd)
os.close(output_parent_fd)
os.set_inheritable(binary_fd, False)
os.execve(binary_fd, [binary_argv0] + sys.argv[9:], os.environ)
"""


class OfflineCliOwnerError(RuntimeError):
    """Base class for the dedicated offline owner boundary."""


class OfflineCliValidationError(OfflineCliOwnerError, ValueError):
    """A typed request or configured filesystem boundary is invalid."""


class OfflineCliRequestConflict(OfflineCliOwnerError):
    """A request ID was permanently bound to a different typed request."""


class OfflineCliBusyError(OfflineCliOwnerError):
    """The single-capacity owner already has an active request."""


class OfflineCliCapacityError(OfflineCliOwnerError):
    """The bounded durable request ledger has reached its capacity."""


class OfflineCliNotFoundError(OfflineCliOwnerError, KeyError):
    """No durable request record exists for the supplied request ID."""


class OfflineCliRecoveryError(OfflineCliOwnerError):
    """Crash recovery could not prove that cleanup was safe and complete."""


class OfflineCliClosedError(OfflineCliOwnerError):
    """The owner is closing or already closed."""


class OfflineCliCommandKind(str, Enum):
    MAP = "map"
    POSE_FILL = "pose-fill"
    RENAME = "rename"
    SUMMARY = "summary"


class OfflineCliState(str, Enum):
    ACCEPTED = "accepted"
    STARTING = "starting"
    RUNNING = "running"
    CANCELLING = "cancelling"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELLED = "cancelled"
    TIMED_OUT = "timed_out"
    SHUTDOWN = "shutdown"


TERMINAL_STATES = frozenset(
    {
        OfflineCliState.SUCCEEDED,
        OfflineCliState.FAILED,
        OfflineCliState.CANCELLED,
        OfflineCliState.TIMED_OUT,
        OfflineCliState.SHUTDOWN,
    }
)


@dataclass(frozen=True)
class MapCommand:
    input_path: str
    output_path: str
    map_name: str
    frame_id: str
    theme: str = "default"
    resolution: float = 0.1
    kind: ClassVar[OfflineCliCommandKind] = OfflineCliCommandKind.MAP


@dataclass(frozen=True)
class PoseFillCommand:
    input_path: str
    output_path: str
    target: str = "lidar"
    pose_source: str = "odom"
    tolerance: float = 0.1
    streams: Optional[Tuple[str, ...]] = None
    seek: float = 0.0
    duration: Optional[float] = None
    kind: ClassVar[OfflineCliCommandKind] = OfflineCliCommandKind.POSE_FILL


@dataclass(frozen=True)
class RenameCommand:
    input_path: str
    output_path: str
    renames: Tuple[Tuple[str, str], ...] = ()
    drops: Tuple[str, ...] = ()
    seek: float = 0.0
    duration: Optional[float] = None
    kind: ClassVar[OfflineCliCommandKind] = OfflineCliCommandKind.RENAME


@dataclass(frozen=True)
class SummaryCommand:
    input_path: str
    output_path: str
    kind: ClassVar[OfflineCliCommandKind] = OfflineCliCommandKind.SUMMARY


OfflineCliCommand = Union[
    MapCommand,
    PoseFillCommand,
    RenameCommand,
    SummaryCommand,
]


@dataclass(frozen=True)
class OfflineCliRequest:
    request_id: str
    command: OfflineCliCommand
    timeout_seconds: float


@dataclass(frozen=True)
class OfflineCliReleaseProof:
    generation: str
    manifest_sha256: str
    artifact_sha256: str
    artifact_size: int
    artifact_path: str
    artifact_fd: int


@dataclass(frozen=True)
class LegacyOfflineCliStatus:
    schema_version: int
    request_id: str
    request_digest: str
    operation_id: str
    command_kind: OfflineCliCommandKind
    state: OfflineCliState
    terminal: bool
    dataset_sha256: str
    dataset_size: int
    dataset_format: str
    dataset_device: int
    dataset_inode: int
    submitted_at_unix_ns: int
    deadline_at_unix_ns: int
    started_at_unix_ns: Optional[int]
    completed_at_unix_ns: Optional[int]
    pid: Optional[int]
    process_start_token: Optional[int]
    pgid: Optional[int]
    sid: Optional[int]
    exit_code: Optional[int]
    stdout: str
    stderr: str
    stdout_truncated: bool
    stderr_truncated: bool
    output_relative_path: str
    output_size: Optional[int]
    output_sha256: Optional[str]
    receipt: Optional[Mapping[str, object]]
    release_generation: str
    release_manifest_sha256: str
    offline_cli_artifact_sha256: str
    offline_cli_artifact_size: int
    error_code: Optional[str]
    message: str

    @classmethod
    def from_record(cls, record: Mapping[str, object]) -> "LegacyOfflineCliStatus":
        _validate_record_shape(record)
        return cls(
            schema_version=int(record["schema_version"]),
            request_id=str(record["request_id"]),
            request_digest=str(record["request_digest"]),
            operation_id=str(record["operation_id"]),
            command_kind=OfflineCliCommandKind(str(record["command_kind"])),
            state=OfflineCliState(str(record["state"])),
            terminal=bool(record["terminal"]),
            dataset_sha256=str(record["dataset_sha256"]),
            dataset_size=int(record["dataset_size"]),
            dataset_format=str(record["dataset_format"]),
            dataset_device=int(record["dataset_device"]),
            dataset_inode=int(record["dataset_inode"]),
            submitted_at_unix_ns=int(record["submitted_at_unix_ns"]),
            deadline_at_unix_ns=int(record["deadline_at_unix_ns"]),
            started_at_unix_ns=_optional_int(record["started_at_unix_ns"]),
            completed_at_unix_ns=_optional_int(record["completed_at_unix_ns"]),
            pid=_optional_int(record["pid"]),
            process_start_token=_optional_int(record["process_start_token"]),
            pgid=_optional_int(record["pgid"]),
            sid=_optional_int(record["sid"]),
            exit_code=_optional_int(record["exit_code"]),
            stdout=str(record["stdout"]),
            stderr=str(record["stderr"]),
            stdout_truncated=bool(record["stdout_truncated"]),
            stderr_truncated=bool(record["stderr_truncated"]),
            output_relative_path=str(record["output_relative_path"]),
            output_size=_optional_int(record["output_size"]),
            output_sha256=_optional_text(record["output_sha256"]),
            receipt=(None if record["receipt"] is None else dict(record["receipt"])),
            release_generation=str(record["release_generation"]),
            release_manifest_sha256=str(record["release_manifest_sha256"]),
            offline_cli_artifact_sha256=str(record["offline_cli_artifact_sha256"]),
            offline_cli_artifact_size=int(record["offline_cli_artifact_size"]),
            error_code=_optional_text(record["error_code"]),
            message=str(record["message"]),
        )


@dataclass(frozen=True)
class OfflineCliStatus:
    schema_version: int
    request_id: str
    operation_id: str
    command_kind: OfflineCliCommandKind
    state: OfflineCliState
    terminal: bool
    dataset_size: int
    dataset_format: str
    dataset_device: int
    dataset_inode: int
    dataset_mtime_ns: int
    dataset_ctime_ns: int
    submitted_at_unix_ns: int
    deadline_at_unix_ns: int
    started_at_unix_ns: Optional[int]
    completed_at_unix_ns: Optional[int]
    pid: Optional[int]
    process_start_token: Optional[int]
    pgid: Optional[int]
    sid: Optional[int]
    exit_code: Optional[int]
    stdout: str
    stderr: str
    stdout_truncated: bool
    stderr_truncated: bool
    output_relative_path: str
    output_size: Optional[int]
    receipt: Optional[Mapping[str, object]]
    release_generation: str
    release_manifest_sha256: str
    offline_cli_artifact_sha256: str
    offline_cli_artifact_size: int
    error_code: Optional[str]
    message: str

    @classmethod
    def from_record(cls, record: Mapping[str, object]) -> Union["OfflineCliStatus", LegacyOfflineCliStatus]:
        if record.get("schema_version") == LEGACY_REQUEST_SCHEMA_VERSION:
            return LegacyOfflineCliStatus.from_record(record)
        _validate_record_shape(record)
        return cls(
            schema_version=int(record["schema_version"]),
            request_id=str(record["request_id"]),
            operation_id=str(record["operation_id"]),
            command_kind=OfflineCliCommandKind(str(record["command_kind"])),
            state=OfflineCliState(str(record["state"])),
            terminal=bool(record["terminal"]),
            dataset_size=int(record["dataset_size"]),
            dataset_format=str(record["dataset_format"]),
            dataset_device=int(record["dataset_device"]),
            dataset_inode=int(record["dataset_inode"]),
            dataset_mtime_ns=int(record["dataset_mtime_ns"]),
            dataset_ctime_ns=int(record["dataset_ctime_ns"]),
            submitted_at_unix_ns=int(record["submitted_at_unix_ns"]),
            deadline_at_unix_ns=int(record["deadline_at_unix_ns"]),
            started_at_unix_ns=_optional_int(record["started_at_unix_ns"]),
            completed_at_unix_ns=_optional_int(record["completed_at_unix_ns"]),
            pid=_optional_int(record["pid"]),
            process_start_token=_optional_int(record["process_start_token"]),
            pgid=_optional_int(record["pgid"]),
            sid=_optional_int(record["sid"]),
            exit_code=_optional_int(record["exit_code"]),
            stdout=str(record["stdout"]),
            stderr=str(record["stderr"]),
            stdout_truncated=bool(record["stdout_truncated"]),
            stderr_truncated=bool(record["stderr_truncated"]),
            output_relative_path=str(record["output_relative_path"]),
            output_size=_optional_int(record["output_size"]),
            receipt=(None if record["receipt"] is None else dict(record["receipt"])),
            release_generation=str(record["release_generation"]),
            release_manifest_sha256=str(record["release_manifest_sha256"]),
            offline_cli_artifact_sha256=str(record["offline_cli_artifact_sha256"]),
            offline_cli_artifact_size=int(record["offline_cli_artifact_size"]),
            error_code=_optional_text(record["error_code"]),
            message=str(record["message"]),
        )


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


def _validate_release_proof(proof: OfflineCliReleaseProof) -> OfflineCliReleaseProof:
    if type(proof) is not OfflineCliReleaseProof:
        raise OfflineCliValidationError("release_proof must be an OfflineCliReleaseProof")
    for value, label in (
        (proof.generation, "release generation"),
        (proof.manifest_sha256, "release manifest sha256"),
        (proof.artifact_sha256, "offline CLI artifact sha256"),
    ):
        if not isinstance(value, str) or not _SHA256_RE.fullmatch(value):
            raise OfflineCliValidationError("%s is not a canonical SHA-256 digest" % label)
    if (
        isinstance(proof.artifact_size, bool)
        or not isinstance(proof.artifact_size, int)
        or proof.artifact_size <= 0
        or isinstance(proof.artifact_fd, bool)
        or not isinstance(proof.artifact_fd, int)
        or proof.artifact_fd <= 2
    ):
        raise OfflineCliValidationError("release artifact size or retained FD is invalid")
    if not Path(proof.artifact_path).is_absolute():
        raise OfflineCliValidationError("release artifact path must be absolute")
    return proof


def _optional_int(value: object) -> Optional[int]:
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, int):
        raise OfflineCliRecoveryError("durable status contains a non-integer field")
    return value


def _optional_text(value: object) -> Optional[str]:
    if value is None:
        return None
    if not isinstance(value, str):
        raise OfflineCliRecoveryError("durable status contains a non-string field")
    return value


def _require_finite(
    value: object,
    name: str,
    *,
    minimum: float,
    maximum: float,
    allow_zero: bool = True,
) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise OfflineCliValidationError("%s must be a finite number" % name)
    normalized = float(value)
    if not math.isfinite(normalized):
        raise OfflineCliValidationError("%s must be a finite number" % name)
    if normalized < minimum or normalized > maximum:
        raise OfflineCliValidationError("%s is outside the supported range" % name)
    if not allow_zero and normalized == 0.0:
        raise OfflineCliValidationError("%s must be positive" % name)
    return normalized


def _require_identity(value: object, name: str) -> str:
    if not isinstance(value, str) or not value or value.strip() != value:
        raise OfflineCliValidationError("%s must be a non-empty canonical string" % name)
    if "\x00" in value or "\n" in value or "\r" in value:
        raise OfflineCliValidationError("%s contains a forbidden control character" % name)
    return value


def _safe_map_segment(map_name: str) -> str:
    return "map-" + map_name.encode("utf-8").hex()


def _require_token(value: object, name: str) -> str:
    if not isinstance(value, str):
        raise OfflineCliValidationError("%s must be a string" % name)
    normalized = value.strip()
    if not _TOKEN_RE.fullmatch(normalized) or normalized.startswith("-"):
        raise OfflineCliValidationError("%s contains unsupported characters" % name)
    return normalized


def _require_items(values: Sequence[str], name: str) -> Tuple[str, ...]:
    if isinstance(values, (str, bytes)) or len(values) > MAX_ITEMS:
        raise OfflineCliValidationError("%s exceeds the bounded item count" % name)
    normalized = tuple(_require_token(value, name) for value in values)
    if len(set(normalized)) != len(normalized):
        raise OfflineCliValidationError("%s contains duplicate values" % name)
    return normalized


def _relative_parts(value: object, name: str) -> Tuple[str, ...]:
    if not isinstance(value, str):
        raise OfflineCliValidationError("%s must be a relative path string" % name)
    if "\x00" in value or not value or len(value.encode("utf-8")) > MAX_PATH_BYTES:
        raise OfflineCliValidationError("%s is empty or exceeds the path limit" % name)
    if os.path.isabs(value):
        raise OfflineCliValidationError("%s must be relative to its configured root" % name)
    parts = tuple(value.split("/"))
    if any(
        not part
        or part in {".", ".."}
        or len(part.encode("utf-8")) > 255
        or any(ord(character) < 32 or ord(character) == 127 for character in part)
        for part in parts
    ):
        raise OfflineCliValidationError("%s contains traversal or invalid components" % name)
    if any(part.startswith("-") for part in parts):
        raise OfflineCliValidationError(
            "%s must not contain an option-like component" % name
        )
    return parts


def _require_extension(path: str, allowed: Sequence[str], name: str) -> None:
    suffix = Path(path).suffix
    if suffix not in allowed:
        raise OfflineCliValidationError("%s must use one of these extensions: %s" % (name, ", ".join(allowed)))


def _command_contract(command: OfflineCliCommand) -> Tuple[Dict[str, object], List[str]]:
    if type(command) is MapCommand:
        input_path = "/".join(_relative_parts(command.input_path, "map input_path"))
        output_path = "/".join(_relative_parts(command.output_path, "map output_path"))
        _require_extension(input_path, (".jsonl", ".ndjson", ".db"), "map input_path")
        map_name = _require_identity(command.map_name, "map map_name")
        frame_id = _require_identity(command.frame_id, "map frame_id")
        if Path(output_path).name != _safe_map_segment(map_name):
            raise OfflineCliValidationError("map output_path must end with the canonical map segment")
        theme = _require_token(command.theme, "map theme")
        resolution = _require_finite(
            command.resolution,
            "map resolution",
            minimum=1e-9,
            maximum=1e6,
            allow_zero=False,
        )
        payload = {
            "kind": command.kind.value,
            "input_path": input_path,
            "output_path": output_path,
            "map_name": map_name,
            "frame_id": frame_id,
            "theme": theme,
            "resolution": resolution,
        }
        argv = [
            command.kind.value,
            "--bag",
            input_path,
            "--output-parent",
            ".",
            "--map-name",
            map_name,
            "--frame-id",
            frame_id,
            "--release-generation",
            "{release_generation}",
            "--release-manifest-sha256",
            "{release_manifest_sha256}",
            "--artifact-sha256",
            "{artifact_sha256}",
            "--theme",
            theme,
            "--resolution",
            repr(resolution),
        ]
        return payload, argv

    if type(command) is PoseFillCommand:
        input_path = "/".join(_relative_parts(command.input_path, "pose-fill input_path"))
        output_path = "/".join(_relative_parts(command.output_path, "pose-fill output_path"))
        _require_extension(input_path, (".jsonl", ".ndjson", ".db"), "pose-fill input_path")
        expected_outputs = (".db",) if Path(input_path).suffix == ".db" else (".jsonl", ".ndjson")
        _require_extension(output_path, expected_outputs, "pose-fill output_path")
        target = _require_token(command.target, "pose-fill target")
        pose_source = _require_token(command.pose_source, "pose-fill pose_source")
        tolerance = _require_finite(
            command.tolerance,
            "pose-fill tolerance",
            minimum=0.0,
            maximum=MAX_TIMEOUT_SECONDS,
        )
        seek = _require_finite(
            command.seek,
            "pose-fill seek",
            minimum=0.0,
            maximum=MAX_TIMEOUT_SECONDS,
        )
        duration = None
        if command.duration is not None:
            duration = _require_finite(
                command.duration,
                "pose-fill duration",
                minimum=1e-9,
                maximum=MAX_TIMEOUT_SECONDS,
                allow_zero=False,
            )
        streams = None
        if command.streams is not None:
            streams = _require_items(command.streams, "pose-fill streams")
            if not streams:
                raise OfflineCliValidationError(
                    "pose-fill streams must not be an empty selection"
                )
        if Path(input_path).suffix != ".db" and (streams is not None or seek != 0.0 or duration is not None):
            raise OfflineCliValidationError("pose-fill native stream options require a .db input")
        payload = {
            "kind": command.kind.value,
            "input_path": input_path,
            "output_path": output_path,
            "target": target,
            "pose_source": pose_source,
            "tolerance": tolerance,
            "streams": list(streams) if streams is not None else None,
            "seek": seek,
            "duration": duration,
        }
        argv = [
            command.kind.value,
            "--input",
            input_path,
            "--output",
            output_path,
            "--target",
            target,
            "--pose-source",
            pose_source,
            "--tolerance",
            repr(tolerance),
        ]
        if streams is not None:
            argv.extend(["--streams", ",".join(streams)])
        if seek != 0.0:
            argv.extend(["--seek", repr(seek)])
        if duration is not None:
            argv.extend(["--duration", repr(duration)])
        return payload, argv

    if type(command) is RenameCommand:
        input_path = "/".join(_relative_parts(command.input_path, "rename input_path"))
        output_path = "/".join(_relative_parts(command.output_path, "rename output_path"))
        _require_extension(input_path, (".jsonl", ".ndjson", ".db"), "rename input_path")
        expected_outputs = (".db",) if Path(input_path).suffix == ".db" else (".jsonl", ".ndjson")
        _require_extension(output_path, expected_outputs, "rename output_path")
        if len(command.renames) > MAX_ITEMS:
            raise OfflineCliValidationError("rename pairs exceed the bounded item count")
        renames: List[Tuple[str, str]] = []
        sources = set()
        for pair in command.renames:
            if not isinstance(pair, tuple) or len(pair) != 2:
                raise OfflineCliValidationError("rename pairs must be typed two-tuples")
            source = _require_token(pair[0], "rename source")
            destination = _require_token(pair[1], "rename destination")
            if "=" in source or "=" in destination or source in sources:
                raise OfflineCliValidationError("rename pairs contain a duplicate or invalid source")
            sources.add(source)
            renames.append((source, destination))
        drops = _require_items(command.drops, "rename drops")
        if not renames and not drops:
            raise OfflineCliValidationError("rename requires at least one rename or drop")
        seek = _require_finite(
            command.seek,
            "rename seek",
            minimum=0.0,
            maximum=MAX_TIMEOUT_SECONDS,
        )
        duration = None
        if command.duration is not None:
            duration = _require_finite(
                command.duration,
                "rename duration",
                minimum=1e-9,
                maximum=MAX_TIMEOUT_SECONDS,
                allow_zero=False,
            )
        if Path(input_path).suffix != ".db" and (seek != 0.0 or duration is not None):
            raise OfflineCliValidationError("rename native timeline options require a .db input")
        payload = {
            "kind": command.kind.value,
            "input_path": input_path,
            "output_path": output_path,
            "renames": [list(pair) for pair in renames],
            "drops": list(drops),
            "seek": seek,
            "duration": duration,
        }
        argv = [command.kind.value, "--bag", input_path, "--output", output_path]
        for source, destination in renames:
            argv.extend(["--rename", "%s=%s" % (source, destination)])
        for stream in drops:
            argv.extend(["--drop", stream])
        if seek != 0.0:
            argv.extend(["--seek", repr(seek)])
        if duration is not None:
            argv.extend(["--duration", repr(duration)])
        return payload, argv

    if type(command) is SummaryCommand:
        input_path = "/".join(_relative_parts(command.input_path, "summary input_path"))
        output_path = "/".join(_relative_parts(command.output_path, "summary output_path"))
        _require_extension(input_path, (".jsonl", ".ndjson", ".db"), "summary input_path")
        _require_extension(output_path, (".json",), "summary output_path")
        payload = {
            "kind": command.kind.value,
            "input_path": input_path,
            "output_path": output_path,
        }
        argv = [
            command.kind.value,
            "--bag",
            input_path,
            "--output",
            output_path,
        ]
        return payload, argv

    raise OfflineCliValidationError("command is not one of the six typed variants")


def _request_contract(
    request: OfflineCliRequest,
    *,
    release_generation: str,
    release_manifest_sha256: str,
    offline_cli_artifact_sha256: str,
    offline_cli_artifact_size: int,
    dataset_size: int,
    dataset_format: str,
    dataset_device: int,
    dataset_inode: int,
    dataset_mtime_ns: int,
    dataset_ctime_ns: int,
) -> Tuple[Dict[str, object], List[str], float]:
    if not isinstance(request.request_id, str):
        raise OfflineCliValidationError("request_id must be a string")
    request_id = request.request_id.strip()
    if not _REQUEST_ID_RE.fullmatch(request_id):
        raise OfflineCliValidationError("request_id is not canonical")
    timeout = _require_finite(
        request.timeout_seconds,
        "timeout_seconds",
        minimum=1e-3,
        maximum=MAX_TIMEOUT_SECONDS,
        allow_zero=False,
    )
    command_payload, argv = _command_contract(request.command)
    payload = {
        "schema_version": REQUEST_SCHEMA_VERSION,
        "request_id": request_id,
        "timeout_seconds": timeout,
        "release_generation": release_generation,
        "release_manifest_sha256": release_manifest_sha256,
        "offline_cli_artifact_sha256": offline_cli_artifact_sha256,
        "offline_cli_artifact_size": offline_cli_artifact_size,
        "dataset_size": dataset_size,
        "dataset_format": dataset_format,
        "dataset_device": dataset_device,
        "dataset_inode": dataset_inode,
        "dataset_mtime_ns": dataset_mtime_ns,
        "dataset_ctime_ns": dataset_ctime_ns,
        "command": command_payload,
    }
    encoded = json.dumps(
        payload,
        allow_nan=False,
        ensure_ascii=True,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")
    if len(encoded) > MAX_REQUEST_BYTES:
        raise OfflineCliValidationError("typed request exceeds the bounded size")
    if type(request.command) is MapCommand:
        argv.extend(["--request-id", request_id])
    return payload, argv, timeout


def _durable_command_contract(payload: object) -> Dict[str, object]:
    if not isinstance(payload, dict):
        raise OfflineCliRecoveryError("durable typed command is not an object")
    try:
        kind = OfflineCliCommandKind(payload.get("kind"))
        if kind == OfflineCliCommandKind.MAP:
            expected = {"kind", "input_path", "output_path", "map_name", "frame_id", "theme", "resolution"}
            if set(payload) != expected:
                raise OfflineCliRecoveryError("durable map command shape is invalid")
            command: OfflineCliCommand = MapCommand(
                input_path=payload["input_path"],
                output_path=payload["output_path"],
                map_name=payload["map_name"],
                frame_id=payload["frame_id"],
                theme=payload["theme"],
                resolution=payload["resolution"],
            )
        elif kind == OfflineCliCommandKind.POSE_FILL:
            expected = {
                "kind",
                "input_path",
                "output_path",
                "target",
                "pose_source",
                "tolerance",
                "streams",
                "seek",
                "duration",
            }
            if set(payload) != expected:
                raise OfflineCliRecoveryError("durable pose-fill command shape is invalid")
            raw_streams = payload["streams"]
            if raw_streams is not None and not isinstance(raw_streams, list):
                raise OfflineCliRecoveryError("durable pose-fill streams are invalid")
            command = PoseFillCommand(
                input_path=payload["input_path"],
                output_path=payload["output_path"],
                target=payload["target"],
                pose_source=payload["pose_source"],
                tolerance=payload["tolerance"],
                streams=(None if raw_streams is None else tuple(raw_streams)),
                seek=payload["seek"],
                duration=payload["duration"],
            )
        elif kind == OfflineCliCommandKind.RENAME:
            expected = {
                "kind",
                "input_path",
                "output_path",
                "renames",
                "drops",
                "seek",
                "duration",
            }
            if set(payload) != expected:
                raise OfflineCliRecoveryError("durable rename command shape is invalid")
            raw_renames = payload["renames"]
            raw_drops = payload["drops"]
            if not isinstance(raw_renames, list) or not isinstance(raw_drops, list):
                raise OfflineCliRecoveryError("durable rename items are invalid")
            renames: List[Tuple[str, str]] = []
            for pair in raw_renames:
                if not isinstance(pair, list) or len(pair) != 2:
                    raise OfflineCliRecoveryError("durable rename pair is invalid")
                renames.append((pair[0], pair[1]))
            command = RenameCommand(
                input_path=payload["input_path"],
                output_path=payload["output_path"],
                renames=tuple(renames),
                drops=tuple(raw_drops),
                seek=payload["seek"],
                duration=payload["duration"],
            )
        else:
            expected = {"kind", "input_path", "output_path"}
            if set(payload) != expected:
                raise OfflineCliRecoveryError("durable summary command shape is invalid")
            command = SummaryCommand(
                input_path=payload["input_path"],
                output_path=payload["output_path"],
            )
        normalized, _ = _command_contract(command)
    except OfflineCliRecoveryError:
        raise
    except (KeyError, TypeError, ValueError, OfflineCliValidationError) as error:
        raise OfflineCliRecoveryError("durable typed command is invalid") from error
    if normalized != payload:
        raise OfflineCliRecoveryError("durable typed command is not canonical")
    return normalized


def _directory_flags() -> int:
    required = ("O_DIRECTORY", "O_NOFOLLOW", "O_CLOEXEC")
    if os.name != "posix" or any(not hasattr(os, name) for name in required):
        raise OfflineCliValidationError("offline CLI ownership requires POSIX no-follow directory operations")
    return os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC


def _same_identity(left: os.stat_result, right: os.stat_result) -> bool:
    return left.st_dev == right.st_dev and left.st_ino == right.st_ino


def _open_absolute_directory(path: Path, label: str) -> int:
    normalized = Path(os.path.abspath(os.path.expanduser(str(path))))
    if not normalized.is_absolute():
        raise OfflineCliValidationError("%s root must be absolute" % label)
    current = os.open(os.sep, _directory_flags())
    try:
        for component in normalized.parts[1:]:
            next_descriptor = os.open(component, _directory_flags(), dir_fd=current)
            os.close(current)
            current = next_descriptor
        metadata = os.fstat(current)
        if not stat.S_ISDIR(metadata.st_mode):
            raise OfflineCliValidationError("%s root is not a directory" % label)
        return current
    except OSError as error:
        os.close(current)
        raise OfflineCliValidationError(
            "%s path contains a missing, symbolic, or invalid directory" % label
        ) from error
    except Exception:
        os.close(current)
        raise


@dataclass
class _ResolvedInput:
    path: Path
    parent_fd: int
    file_fd: int
    metadata: os.stat_result

    def verify_current_identity(self) -> None:
        opened = os.fstat(self.file_fd)
        named = os.stat(self.path.name, dir_fd=self.parent_fd, follow_symlinks=False)
        if (
            not _same_identity(opened, self.metadata)
            or not _same_identity(named, self.metadata)
            or not stat.S_ISREG(opened.st_mode)
            or not stat.S_ISREG(named.st_mode)
            or opened.st_size != self.metadata.st_size
            or opened.st_mtime_ns != self.metadata.st_mtime_ns
            or opened.st_ctime_ns != self.metadata.st_ctime_ns
            or named.st_size != self.metadata.st_size
            or named.st_mtime_ns != self.metadata.st_mtime_ns
            or named.st_ctime_ns != self.metadata.st_ctime_ns
        ):
            raise OfflineCliValidationError("input identity changed before process release")

    def observe_input(self) -> Tuple[int, str, int, int, int, int]:
        self.verify_current_identity()
        metadata = self.metadata
        formats = {".jsonl": "jsonl", ".ndjson": "ndjson", ".db": "memory2_sqlite"}
        suffix = self.path.suffix
        if suffix not in formats:
            raise OfflineCliValidationError("input format is unsupported")
        if not 0 < metadata.st_size < (1 << 64) or any(
               not 0 <= value < (1 << 64) for value in
               (metadata.st_dev, metadata.st_ino)) or any(
               not -(1 << 63) <= value < (1 << 63) for value in
               (metadata.st_mtime_ns, metadata.st_ctime_ns)):
            raise OfflineCliValidationError("input observation is outside the supported integer bounds")
        return (metadata.st_size, formats[suffix], metadata.st_dev, metadata.st_ino,
                metadata.st_mtime_ns, metadata.st_ctime_ns)

    def verify_input_observation(
        self,
        expected_size: int,
        expected_format: str,
        expected_device: int,
        expected_inode: int,
        expected_mtime_ns: int,
        expected_ctime_ns: int,
    ) -> None:
        self.verify_current_identity()
        metadata = self.metadata
        formats = {".jsonl": "jsonl", ".ndjson": "ndjson", ".db": "memory2_sqlite"}
        observed = (metadata.st_size, formats.get(self.path.suffix), metadata.st_dev,
                    metadata.st_ino, metadata.st_mtime_ns, metadata.st_ctime_ns)
        if observed != (expected_size, expected_format, expected_device,
                        expected_inode, expected_mtime_ns, expected_ctime_ns):
            raise OfflineCliValidationError("input observation does not match the durable request")

    def close(self) -> None:
        os.close(self.file_fd)
        os.close(self.parent_fd)


@dataclass
class _ResolvedOutput:
    path: Path
    parent_fd: int
    parent_metadata: os.stat_result

    def verify_parent_identity(self) -> None:
        current = os.stat(self.path.parent, follow_symlinks=False)
        if not _same_identity(current, self.parent_metadata) or not stat.S_ISDIR(current.st_mode):
            raise OfflineCliValidationError("output parent identity changed")

    def inspect_final(self) -> int:
        self.verify_parent_identity()
        flags = os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW
        descriptor = os.open(self.path.name, flags, dir_fd=self.parent_fd)
        try:
            metadata = os.fstat(descriptor)
            if (
                not stat.S_ISREG(metadata.st_mode)
                or metadata.st_nlink != 1
                or metadata.st_size <= 0
            ):
                raise OfflineCliOwnerError(
                    "durable CLI output is missing, empty, linked, or non-regular"
                )
            return metadata.st_size
        finally:
            os.close(descriptor)

    def close(self) -> None:
        os.close(self.parent_fd)


class _StableRoot:
    def __init__(self, path: Union[str, os.PathLike[str]], label: str) -> None:
        expanded = Path(os.path.expanduser(os.fspath(path)))
        if not expanded.is_absolute():
            raise OfflineCliValidationError("%s root must be absolute" % label)
        self.path = Path(os.path.abspath(expanded))
        self.label = label
        if self.path == Path(os.sep):
            raise OfflineCliValidationError("%s root must not be the filesystem root" % label)
        descriptor = _open_absolute_directory(self.path, label)
        try:
            metadata = os.fstat(descriptor)
            if label == "state" and (
                metadata.st_uid != os.geteuid()
                or stat.S_IMODE(metadata.st_mode) & 0o022
            ):
                raise OfflineCliValidationError(
                    "state root must be owner-controlled and not group or world writable"
                )
        except Exception:
            os.close(descriptor)
            raise
        self.fd = descriptor
        self.metadata = metadata

    def verify_current_identity(self) -> None:
        current = os.stat(self.path, follow_symlinks=False)
        if not _same_identity(current, self.metadata) or not stat.S_ISDIR(current.st_mode):
            raise OfflineCliValidationError("%s root identity changed" % self.label)

    def _open_parent(self, parts: Sequence[str]) -> int:
        descriptor = os.dup(self.fd)
        try:
            for component in parts[:-1]:
                next_descriptor = os.open(
                    component,
                    _directory_flags(),
                    dir_fd=descriptor,
                )
                os.close(descriptor)
                descriptor = next_descriptor
            return descriptor
        except Exception:
            os.close(descriptor)
            raise

    def resolve_input(self, relative_path: str) -> _ResolvedInput:
        self.verify_current_identity()
        parts = _relative_parts(relative_path, "input path")
        parent_fd = self._open_parent(parts)
        try:
            file_fd = os.open(
                parts[-1],
                os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW,
                dir_fd=parent_fd,
            )
            metadata = os.fstat(file_fd)
            if not stat.S_ISREG(metadata.st_mode):
                raise OfflineCliValidationError("input path is not a regular file")
            return _ResolvedInput(
                path=self.path.joinpath(*parts),
                parent_fd=parent_fd,
                file_fd=file_fd,
                metadata=metadata,
            )
        except Exception:
            os.close(parent_fd)
            raise

    def resolve_output(self, relative_path: str) -> _ResolvedOutput:
        self.verify_current_identity()
        parts = _relative_parts(relative_path, "output path")
        parent_fd = self._open_parent(parts)
        try:
            try:
                os.stat(parts[-1], dir_fd=parent_fd, follow_symlinks=False)
            except FileNotFoundError:
                pass
            else:
                raise OfflineCliValidationError("output path already exists")
            return _ResolvedOutput(
                path=self.path.joinpath(*parts),
                parent_fd=parent_fd,
                parent_metadata=os.fstat(parent_fd),
            )
        except Exception:
            os.close(parent_fd)
            raise

    def close(self) -> None:
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1


def _roots_overlap(left: _StableRoot, right: _StableRoot) -> bool:
    if _same_identity(left.metadata, right.metadata):
        return True
    common = Path(os.path.commonpath((str(left.path), str(right.path))))
    return common == left.path or common == right.path


@dataclass(frozen=True)
class _ProcessInfo:
    pid: int
    state: str
    pgrp: int
    session: int
    start_token: int


class _ProbeKind(str, Enum):
    RUNNING = "running"
    ABSENT = "absent"
    UNVERIFIABLE = "unverifiable"


def _parse_proc_stat(pid: int, text: str) -> Optional[_ProcessInfo]:
    close = text.rfind(")")
    if close < 0:
        return None
    fields = text[close + 1 :].split()
    if len(fields) < 20:
        return None
    try:
        state = fields[0]
        pgrp = int(fields[2])
        session = int(fields[3])
        start_token = int(fields[19])
    except (TypeError, ValueError):
        return None
    if len(state) != 1 or pgrp <= 0 or session <= 0 or start_token <= 0:
        return None
    return _ProcessInfo(pid, state, pgrp, session, start_token)


def _probe_process(pid: int) -> Tuple[_ProbeKind, Optional[_ProcessInfo]]:
    if isinstance(pid, bool) or not isinstance(pid, int) or pid <= 0:
        return _ProbeKind.UNVERIFIABLE, None
    process_dir = Path("/proc") / str(pid)
    try:
        text = (process_dir / "stat").read_text(encoding="utf-8")
    except FileNotFoundError:
        try:
            process_dir.lstat()
        except FileNotFoundError:
            return _ProbeKind.ABSENT, None
        except OSError:
            return _ProbeKind.UNVERIFIABLE, None
        return _ProbeKind.UNVERIFIABLE, None
    except (OSError, UnicodeError):
        return _ProbeKind.UNVERIFIABLE, None
    info = _parse_proc_stat(pid, text)
    if info is None:
        return _ProbeKind.UNVERIFIABLE, None
    return _ProbeKind.RUNNING, info


def _group_members(pgid: int, sid: int) -> List[_ProcessInfo]:
    try:
        for entry in Path("/proc").iterdir():
            if not entry.name.isdigit():
                continue
            pid = int(entry.name)
            kind, info = _probe_process(pid)
            if (
                kind == _ProbeKind.RUNNING
                and info is not None
                and info.pgrp == pgid
                and info.session == sid
            ):
                return [info]
    except OSError as error:
        raise OfflineCliRecoveryError("cannot scan /proc for process group") from error
    return []


def _owner_is_proven_dead(pid: int, start_token: int) -> Optional[bool]:
    kind, info = _probe_process(pid)
    if kind == _ProbeKind.ABSENT:
        return True
    if kind == _ProbeKind.UNVERIFIABLE or info is None:
        return None
    return info.start_token != start_token or info.state == "Z"


def _validate_durable_map_receipt(
    receipt: object,
    *,
    command: Mapping[str, object],
    record: Mapping[str, object],
) -> None:
    legacy = record["schema_version"] == LEGACY_REQUEST_SCHEMA_VERSION
    expected = {
        "map_name", "frame_id", "output_directory", "bundle_size",
        "dataset_size", "dataset_format", "dataset_device", "dataset_inode",
        "offline_cli_artifact_size", "files", "release_generation",
        "release_manifest_sha256", "offline_cli_artifact_sha256", "request_id",
    }
    expected |= ({"manifest_sha256", "dataset_sha256", "request_digest"} if legacy else
                 {"schema_version", "dataset_mtime_ns", "dataset_ctime_ns"})
    if not isinstance(receipt, dict) or set(receipt) != expected:
        raise OfflineCliRecoveryError("durable map receipt field set is not exact")
    bound_fields = (
        "dataset_size", "dataset_format", "dataset_device", "dataset_inode",
        "offline_cli_artifact_size", "release_generation", "release_manifest_sha256",
        "offline_cli_artifact_sha256", "request_id",
    ) + (("dataset_sha256", "request_digest") if legacy else
         ("dataset_mtime_ns", "dataset_ctime_ns"))
    if (
        receipt["map_name"] != command["map_name"]
        or receipt["frame_id"] != command["frame_id"]
        or receipt["bundle_size"] != record["output_size"]
        or any(receipt[field] != record[field] for field in bound_fields)
        or (legacy and receipt["manifest_sha256"] != record["output_sha256"])
        or (not legacy and (type(receipt["schema_version"]) is not int or receipt["schema_version"] != 2))
    ):
        raise OfflineCliRecoveryError("durable map receipt identity is inconsistent")
    output_directory = receipt["output_directory"]
    if not isinstance(output_directory, str) or not os.path.isabs(output_directory):
        raise OfflineCliRecoveryError("durable map receipt output directory is invalid")
    output_parts = _relative_parts(command["output_path"], "durable map output path")
    if Path(os.path.normpath(output_directory)).parts[-len(output_parts) :] != output_parts:
        raise OfflineCliRecoveryError("durable map receipt output directory disagrees with its command")
    files = receipt["files"]
    if not isinstance(files, list) or len(files) != 2:
        raise OfflineCliRecoveryError("durable map receipt files are not exact")
    expected_paths = ("asset.json", "map.pgm")
    for item, expected_path in zip(files, expected_paths):
        if not isinstance(item, dict) or set(item) != ({"path", "size", "sha256"} if legacy else {"path", "size"}):
            raise OfflineCliRecoveryError("durable map receipt file shape is invalid")
        if item["path"] != expected_path:
            raise OfflineCliRecoveryError("durable map receipt files are not byte-sorted")
        if type(item["size"]) is not int or item["size"] <= 0:
            raise OfflineCliRecoveryError("durable map receipt file size is invalid")
        if legacy and (not isinstance(item["sha256"], str) or not _SHA256_RE.fullmatch(item["sha256"])):
            raise OfflineCliRecoveryError("durable legacy map receipt alias is invalid")


def _validate_record_shape(record: Mapping[str, object]) -> None:
    required = {
        "schema_version",
        "request_id",
        "request_digest",
        "operation_id",
        "command_kind",
        "command",
        "state",
        "terminal",
        "dataset_sha256",
        "dataset_size",
        "dataset_format",
        "dataset_device",
        "dataset_inode",
        "timeout_seconds",
        "submitted_at_unix_ns",
        "deadline_at_unix_ns",
        "started_at_unix_ns",
        "completed_at_unix_ns",
        "pid",
        "process_start_token",
        "pgid",
        "sid",
        "exit_code",
        "stdout",
        "stderr",
        "stdout_truncated",
        "stderr_truncated",
        "output_relative_path",
        "output_size",
        "output_sha256",
        "receipt",
        "release_generation",
        "release_manifest_sha256",
        "offline_cli_artifact_sha256",
        "offline_cli_artifact_size",
        "error_code",
        "message",
    }
    if not isinstance(record, dict):
        raise OfflineCliRecoveryError("durable request record is not an object")
    legacy = record.get("schema_version") == LEGACY_REQUEST_SCHEMA_VERSION
    if not legacy:
        required -= {"request_digest", "dataset_sha256", "output_sha256"}
        required |= {"dataset_mtime_ns", "dataset_ctime_ns"}
    if set(record) != required:
        raise OfflineCliRecoveryError("durable request record has unknown or missing fields")
    if record.get("schema_version") not in {REQUEST_SCHEMA_VERSION, LEGACY_REQUEST_SCHEMA_VERSION}:
        raise OfflineCliRecoveryError("durable request record schema is unsupported")
    request_id = record.get("request_id")
    digest = record.get("request_digest")
    operation_id = record.get("operation_id")
    if not isinstance(request_id, str) or not _REQUEST_ID_RE.fullmatch(request_id):
        raise OfflineCliRecoveryError("durable request ID is invalid")
    if legacy and (not isinstance(digest, str) or not _SHA256_RE.fullmatch(digest)):
        raise OfflineCliRecoveryError("durable request digest is invalid")
    if not isinstance(operation_id, str) or not _OPERATION_ID_RE.fullmatch(operation_id):
        raise OfflineCliRecoveryError("durable operation ID is invalid")
    command = _durable_command_contract(record.get("command"))
    try:
        timeout = _require_finite(
            record.get("timeout_seconds"),
            "durable timeout_seconds",
            minimum=1e-3,
            maximum=MAX_TIMEOUT_SECONDS,
            allow_zero=False,
        )
    except OfflineCliValidationError as error:
        raise OfflineCliRecoveryError("durable timeout is invalid") from error
    request_payload = {
        "schema_version": record["schema_version"],
        "request_id": request_id,
        "timeout_seconds": timeout,
        "release_generation": record["release_generation"],
        "release_manifest_sha256": record["release_manifest_sha256"],
        "offline_cli_artifact_sha256": record["offline_cli_artifact_sha256"],
        "offline_cli_artifact_size": record["offline_cli_artifact_size"],
        "dataset_size": record["dataset_size"],
        "dataset_format": record["dataset_format"],
        "dataset_device": record["dataset_device"],
        "dataset_inode": record["dataset_inode"],
        "command": command,
    }
    if legacy:
        request_payload["dataset_sha256"] = record["dataset_sha256"]
    else:
        for field in ("dataset_mtime_ns", "dataset_ctime_ns"):
            value = record[field]
            if type(value) is not int or not -(1 << 63) <= value < (1 << 63):
                raise OfflineCliRecoveryError("durable input timestamp is outside signed i64")
            request_payload[field] = value
    encoded = json.dumps(
        request_payload,
        allow_nan=False,
        ensure_ascii=True,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")
    if len(encoded) > MAX_REQUEST_BYTES:
        raise OfflineCliRecoveryError("durable request exceeds its bounded size")
    try:
        command_kind = OfflineCliCommandKind(record.get("command_kind"))
        state = OfflineCliState(record.get("state"))
    except (TypeError, ValueError) as error:
        raise OfflineCliRecoveryError("durable request enum is invalid") from error
    if command_kind.value != command["kind"]:
        raise OfflineCliRecoveryError("durable command kind disagrees with its payload")
    if (
        not isinstance(record.get("terminal"), bool)
        or bool(record.get("terminal")) != (state in TERMINAL_STATES)
    ):
        raise OfflineCliRecoveryError("durable terminal classification is inconsistent")
    for field_name in ("submitted_at_unix_ns", "deadline_at_unix_ns"):
        value = record.get(field_name)
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise OfflineCliRecoveryError("durable timestamp is invalid")
    submitted = int(record["submitted_at_unix_ns"])
    deadline = int(record["deadline_at_unix_ns"])
    if deadline != submitted + int(timeout * 1_000_000_000):
        raise OfflineCliRecoveryError("durable deadline does not match its timeout")
    for field_name in ("stdout", "stderr", "output_relative_path", "message"):
        if not isinstance(record.get(field_name), str):
            raise OfflineCliRecoveryError("durable text field is invalid")
    if record["output_relative_path"] != command["output_path"]:
        raise OfflineCliRecoveryError("durable output path disagrees with its command")
    if len(str(record["message"]).encode("utf-8", errors="replace")) > MAX_MESSAGE_BYTES:
        raise OfflineCliRecoveryError("durable status message exceeds its bound")
    for field_name in ("stdout", "stderr"):
        encoded_log = json.dumps(record[field_name], ensure_ascii=True).encode("utf-8")
        if len(encoded_log) > MAX_CONFIGURED_LOG_BYTES:
            raise OfflineCliRecoveryError("durable process log exceeds its bound")
    for field_name in (
        "release_generation",
        "release_manifest_sha256",
        "offline_cli_artifact_sha256",
    ) + (("dataset_sha256",) if legacy else ()):
        if not isinstance(record.get(field_name), str) or not _SHA256_RE.fullmatch(str(record[field_name])):
            raise OfflineCliRecoveryError("durable release proof field is invalid")
    for field_name in (
        "offline_cli_artifact_size",
        "dataset_size",
        "dataset_device",
        "dataset_inode",
    ):
        value = record.get(field_name)
        if isinstance(value, bool) or not isinstance(value, int) or value < (1 if legacy or field_name in {"offline_cli_artifact_size", "dataset_size"} else 0) or (not legacy and value >= (1 << 64)):
            raise OfflineCliRecoveryError("durable byte or file identity field is invalid")
    if record.get("dataset_format") not in {"jsonl", "ndjson", "memory2_sqlite"}:
        raise OfflineCliRecoveryError("durable dataset format is invalid")
    receipt = record.get("receipt")
    if receipt is not None and not isinstance(receipt, dict):
        raise OfflineCliRecoveryError("durable receipt is invalid")
    for field_name in ("stdout_truncated", "stderr_truncated"):
        if not isinstance(record.get(field_name), bool):
            raise OfflineCliRecoveryError("durable truncation field is invalid")
    for field_name in (
        "started_at_unix_ns",
        "completed_at_unix_ns",
        "pid",
        "process_start_token",
        "pgid",
        "sid",
        "exit_code",
        "output_size",
    ):
        _optional_int(record.get(field_name))
    for field_name in ("started_at_unix_ns", "completed_at_unix_ns"):
        value = record.get(field_name)
        if value is not None and int(value) <= 0:
            raise OfflineCliRecoveryError("durable optional timestamp is invalid")
    process_values = tuple(
        record.get(field_name)
        for field_name in ("pid", "process_start_token", "pgid", "sid")
    )
    if any(value is not None for value in process_values):
        if any(
            value is None or isinstance(value, bool) or not isinstance(value, int) or value <= 0
            for value in process_values
        ):
            raise OfflineCliRecoveryError("durable process identity is incomplete")
        if record["pid"] != record["pgid"] or record["pid"] != record["sid"]:
            raise OfflineCliRecoveryError("durable process is not an isolated session leader")
    output_digest = record.get("output_sha256")
    if output_digest is not None and (
        not isinstance(output_digest, str) or not _SHA256_RE.fullmatch(output_digest)
    ):
        raise OfflineCliRecoveryError("durable output digest is invalid")
    error_code = record.get("error_code")
    if error_code is not None and (
        not isinstance(error_code, str)
        or not error_code
        or len(error_code.encode("utf-8")) > 128
    ):
        raise OfflineCliRecoveryError("durable error code is invalid")
    if state == OfflineCliState.ACCEPTED:
        if (
            any(value is not None for value in process_values)
            or record["started_at_unix_ns"] is not None
        ):
            raise OfflineCliRecoveryError("accepted durable request has process identity")
    elif state == OfflineCliState.STARTING:
        if (
            any(value is None for value in process_values)
            or record["started_at_unix_ns"] is not None
        ):
            raise OfflineCliRecoveryError("starting durable request identity is invalid")
    elif state in {OfflineCliState.RUNNING, OfflineCliState.CANCELLING}:
        if (
            any(value is None for value in process_values)
            or record["started_at_unix_ns"] is None
        ):
            raise OfflineCliRecoveryError("running durable request identity is invalid")
    if state in TERMINAL_STATES:
        if record["completed_at_unix_ns"] is None:
            raise OfflineCliRecoveryError("terminal durable request has no completion time")
        if state == OfflineCliState.SUCCEEDED:
            if (
                record["started_at_unix_ns"] is None
                or record["exit_code"] != 0
                or isinstance(record["output_size"], bool)
                or not isinstance(record["output_size"], int)
                or record["output_size"] <= 0
                or (legacy and output_digest is None)
                or error_code is not None
            ):
                raise OfflineCliRecoveryError("successful durable request proof is incomplete")
            if command_kind == OfflineCliCommandKind.MAP:
                _validate_durable_map_receipt(receipt, command=command, record=record)
            elif receipt is not None:
                raise OfflineCliRecoveryError("non-map durable success contains a map receipt")
        elif (
            record["output_size"] is not None
            or output_digest is not None
            or receipt is not None
            or error_code is None
        ):
            raise OfflineCliRecoveryError("failed durable request contains success proof")
    elif (
        record["completed_at_unix_ns"] is not None
        or record["exit_code"] is not None
        or record["output_size"] is not None
        or output_digest is not None
        or receipt is not None
        or error_code is not None
    ):
        raise OfflineCliRecoveryError("nonterminal durable request contains terminal fields")


def _canonical_json_bytes(payload: object) -> bytes:
    return json.dumps(
        payload,
        allow_nan=False,
        ensure_ascii=True,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")


def _require_state_name(name: object) -> str:
    if not isinstance(name, str) or (
        name != ACTIVE_JOURNAL_NAME
        and _REQUEST_FILE_RE.fullmatch(name) is None
        and _LEGACY_REQUEST_FILE_RE.fullmatch(name) is None
    ):
        raise OfflineCliRecoveryError("state entry name is outside the authenticated domain")
    return name


def _state_entry_mac(auth_key: bytes, name: str, payload_bytes: bytes) -> str:
    authenticated = (
        _STATE_AUTH_DOMAIN
        + b"\x00"
        + name.encode("ascii")
        + b"\x00"
        + payload_bytes
    )
    return hmac.new(auth_key, authenticated, hashlib.sha256).hexdigest()


def _encode_state_entry(
    auth_key: bytes,
    name: str,
    payload: Mapping[str, object],
) -> bytes:
    logical_name = _require_state_name(name)
    if not isinstance(payload, dict):
        payload = dict(payload)
    payload_bytes = _canonical_json_bytes(payload)
    envelope = {
        "schema_version": STATE_ENVELOPE_SCHEMA_VERSION,
        "logical_name": logical_name,
        "payload": payload,
        "mac": _state_entry_mac(auth_key, logical_name, payload_bytes),
    }
    return _canonical_json_bytes(envelope)


class _OwnerStateStore:
    def __init__(
        self,
        root: _StableRoot,
        *,
        owner_pid: int,
        owner_start_token: int,
        state_auth_key: bytes,
    ) -> None:
        self._root = root
        self._owner_pid = owner_pid
        self._owner_start_token = owner_start_token
        self._state_auth_key = state_auth_key
        self._lock_fd = -1
        self._input_alias_dir_fd = -1

    def acquire_lock(self) -> None:
        flags = os.O_RDWR | os.O_CREAT | os.O_CLOEXEC | os.O_NOFOLLOW
        descriptor = os.open(OWNER_LOCK_NAME, flags, 0o600, dir_fd=self._root.fd)
        try:
            metadata = os.fstat(descriptor)
            if (
                not stat.S_ISREG(metadata.st_mode)
                or metadata.st_nlink != 1
                or metadata.st_uid != os.geteuid()
                or stat.S_IMODE(metadata.st_mode) != 0o600
            ):
                raise OfflineCliRecoveryError(
                    "owner lock ownership, mode, or link count is invalid"
                )
            try:
                fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as error:
                raise OfflineCliRecoveryError("another offline CLI owner holds the state root") from error
            self._lock_fd = descriptor
        except Exception:
            os.close(descriptor)
            raise
        self._open_input_alias_directory()

    def _open_input_alias_directory(self) -> None:
        created = False
        try:
            os.mkdir(INPUT_ALIAS_DIR_NAME, 0o700, dir_fd=self._root.fd)
            created = True
        except FileExistsError:
            pass
        except OSError as error:
            raise OfflineCliRecoveryError(
                "cannot create the private input alias directory"
            ) from error
        if created:
            os.fsync(self._root.fd)
        descriptor = -1
        try:
            descriptor = os.open(
                INPUT_ALIAS_DIR_NAME,
                _directory_flags(),
                dir_fd=self._root.fd,
            )
            metadata = os.fstat(descriptor)
            if (
                not stat.S_ISDIR(metadata.st_mode)
                or metadata.st_uid != os.geteuid()
                or stat.S_IMODE(metadata.st_mode) & 0o077
            ):
                raise OfflineCliRecoveryError(
                    "private input alias directory ownership or mode is invalid"
                )
        except Exception:
            if descriptor >= 0:
                os.close(descriptor)
            raise
        self._input_alias_dir_fd = descriptor

    def input_alias_dir_fd(self) -> int:
        if self._input_alias_dir_fd < 0:
            raise OfflineCliClosedError("private input alias directory is closed")
        return self._input_alias_dir_fd

    def list_input_aliases(self) -> Tuple[str, ...]:
        names: List[str] = []
        with os.scandir(self.input_alias_dir_fd()) as entries:
            for entry in entries:
                if len(names) >= MAX_INPUT_ALIASES:
                    raise OfflineCliCapacityError(
                        "private input alias directory exceeds its capacity"
                    )
                names.append(entry.name)
        return tuple(names)

    def create_input_alias(
        self,
        resolved_input: _ResolvedInput,
        operation_id: str,
    ) -> str:
        if not _OPERATION_ID_RE.fullmatch(operation_id):
            raise OfflineCliRecoveryError("input alias operation ID is invalid")
        if len(self.list_input_aliases()) >= MAX_INPUT_ALIASES:
            raise OfflineCliCapacityError("private input alias directory is full")
        extension = resolved_input.path.suffix
        if extension not in {".jsonl", ".ndjson", ".db"}:
            raise OfflineCliRecoveryError("input alias extension is unsupported")
        start_text = "%016x" % self._owner_start_token
        if len(start_text) != 16:
            raise OfflineCliRecoveryError("owner start token cannot be encoded in an alias")
        name = "%s%d-%s-%s%s" % (
            INPUT_ALIAS_PREFIX,
            self._owner_pid,
            start_text,
            operation_id,
            extension,
        )
        target = "/proc/self/fd/%d" % resolved_input.file_fd
        descriptor = self.input_alias_dir_fd()
        created = False
        try:
            os.symlink(target, name, dir_fd=descriptor)
            created = True
            metadata = os.stat(name, dir_fd=descriptor, follow_symlinks=False)
            if not stat.S_ISLNK(metadata.st_mode) or metadata.st_nlink != 1:
                raise OfflineCliRecoveryError("private input alias is not one symbolic link")
            if os.readlink(name, dir_fd=descriptor) != target:
                raise OfflineCliRecoveryError("private input alias target is invalid")
            os.fsync(descriptor)
            return name
        except Exception as error:
            cleanup_error: Optional[BaseException] = None
            if created:
                try:
                    os.unlink(name, dir_fd=descriptor)
                    os.fsync(descriptor)
                except OSError as failure:
                    cleanup_error = failure
            if cleanup_error is not None:
                raise OfflineCliRecoveryError(
                    "input alias creation failed and cleanup is unproven"
                ) from error
            raise

    def remove_input_alias(self, name: str) -> None:
        if _INPUT_ALIAS_RE.fullmatch(name) is None:
            raise OfflineCliRecoveryError("refusing to remove a malformed input alias")
        descriptor = self.input_alias_dir_fd()
        metadata = os.stat(name, dir_fd=descriptor, follow_symlinks=False)
        if not stat.S_ISLNK(metadata.st_mode) or metadata.st_nlink != 1:
            raise OfflineCliRecoveryError("private input alias is not one symbolic link")
        target = os.readlink(name, dir_fd=descriptor)
        if _FD_ALIAS_TARGET_RE.fullmatch(target) is None:
            raise OfflineCliRecoveryError("private input alias target is malformed")
        os.unlink(name, dir_fd=descriptor)
        os.fsync(descriptor)

    def cleanup_stale_input_aliases(self) -> None:
        for name in self.list_input_aliases():
            matched = _INPUT_ALIAS_RE.fullmatch(name)
            if matched is None:
                continue
            owner_pid = int(matched.group(1))
            owner_start = int(matched.group(2), 16)
            if (
                owner_pid == self._owner_pid
                and owner_start == self._owner_start_token
            ):
                dead = True
            else:
                dead = _owner_is_proven_dead(owner_pid, owner_start)
            if dead is None:
                raise OfflineCliRecoveryError(
                    "cannot verify private input alias owner identity"
                )
            if dead:
                self.remove_input_alias(name)

    def request_name(self, request_id: str) -> str:
        if not isinstance(request_id, str) or not _REQUEST_ID_RE.fullmatch(request_id):
            raise OfflineCliRecoveryError("request ID is not canonical")
        return "%s%s.json" % (REQUEST_FILE_PREFIX, request_id)

    def read_json(self, name: str) -> Optional[Dict[str, object]]:
        logical_name = _require_state_name(name)
        try:
            descriptor = os.open(
                name,
                os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW,
                dir_fd=self._root.fd,
            )
        except FileNotFoundError:
            return None
        try:
            metadata = os.fstat(descriptor)
            if (
                not stat.S_ISREG(metadata.st_mode)
                or metadata.st_nlink != 1
                or metadata.st_uid != os.geteuid()
                or stat.S_IMODE(metadata.st_mode) != 0o600
                or metadata.st_size <= 0
                or metadata.st_size > MAX_STATE_BYTES
            ):
                raise OfflineCliRecoveryError(
                    "state entry ownership, mode, link count, or size is invalid"
                )
            chunks = bytearray()
            while True:
                chunk = os.read(descriptor, min(64 * 1024, MAX_STATE_BYTES + 1 - len(chunks)))
                if not chunk:
                    break
                chunks.extend(chunk)
                if len(chunks) > MAX_STATE_BYTES:
                    raise OfflineCliRecoveryError("state entry exceeds the bounded size")
            completed = os.fstat(descriptor)
            if (
                not _same_identity(metadata, completed)
                or completed.st_nlink != metadata.st_nlink
                or completed.st_size != metadata.st_size
                or completed.st_mtime_ns != metadata.st_mtime_ns
                or completed.st_ctime_ns != metadata.st_ctime_ns
                or len(chunks) != metadata.st_size
            ):
                raise OfflineCliRecoveryError("state entry changed while it was read")
        finally:
            os.close(descriptor)
        raw = bytes(chunks)
        try:
            envelope = json.loads(raw.decode("utf-8"))
        except (UnicodeError, json.JSONDecodeError) as error:
            raise OfflineCliRecoveryError("state entry is not valid JSON") from error
        if not isinstance(envelope, dict) or set(envelope) != {
            "schema_version",
            "logical_name",
            "payload",
            "mac",
        }:
            raise OfflineCliRecoveryError("state entry authentication envelope is invalid")
        if envelope.get("schema_version") != STATE_ENVELOPE_SCHEMA_VERSION:
            raise OfflineCliRecoveryError("state entry authentication schema is unsupported")
        if envelope.get("logical_name") != logical_name:
            raise OfflineCliRecoveryError("state entry logical name does not match its file")
        payload = envelope.get("payload")
        mac = envelope.get("mac")
        if not isinstance(payload, dict) or not isinstance(mac, str) or _STATE_MAC_RE.fullmatch(mac) is None:
            raise OfflineCliRecoveryError("state entry authenticated payload or MAC is invalid")
        payload_bytes = _canonical_json_bytes(payload)
        expected_mac = _state_entry_mac(
            self._state_auth_key,
            logical_name,
            payload_bytes,
        )
        if not hmac.compare_digest(mac, expected_mac):
            raise OfflineCliRecoveryError("state entry authentication failed")
        expected = _encode_state_entry(self._state_auth_key, logical_name, payload)
        if not hmac.compare_digest(raw, expected):
            raise OfflineCliRecoveryError("state entry is not the exact canonical envelope")
        return payload

    def write_json(self, name: str, payload: Mapping[str, object]) -> None:
        logical_name = _require_state_name(name)
        encoded = _encode_state_entry(self._state_auth_key, logical_name, payload)
        if len(encoded) > MAX_STATE_BYTES:
            raise OfflineCliOwnerError("state entry exceeds the bounded size")
        temporary = "%s%d-%016x-%s.json" % (
            TEMP_FILE_PREFIX,
            self._owner_pid,
            self._owner_start_token,
            uuid.uuid4().hex,
        )
        descriptor = os.open(
            temporary,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC | os.O_NOFOLLOW,
            0o600,
            dir_fd=self._root.fd,
        )
        installed = False
        try:
            view = memoryview(encoded)
            while view:
                written = os.write(descriptor, view)
                if written <= 0:
                    raise OSError("state write accepted zero bytes")
                view = view[written:]
            metadata = os.fstat(descriptor)
            if (
                not stat.S_ISREG(metadata.st_mode)
                or metadata.st_nlink != 1
                or metadata.st_uid != os.geteuid()
                or stat.S_IMODE(metadata.st_mode) != 0o600
                or metadata.st_size != len(encoded)
            ):
                raise OfflineCliRecoveryError(
                    "temporary state entry ownership, mode, link count, or size is invalid"
                )
            os.fsync(descriptor)
            os.close(descriptor)
            descriptor = -1
            os.replace(
                temporary,
                name,
                src_dir_fd=self._root.fd,
                dst_dir_fd=self._root.fd,
            )
            installed = True
            os.fsync(self._root.fd)
        finally:
            if descriptor >= 0:
                os.close(descriptor)
            if not installed:
                try:
                    os.unlink(temporary, dir_fd=self._root.fd)
                except FileNotFoundError:
                    pass

    def remove(self, name: str) -> None:
        try:
            metadata = os.stat(name, dir_fd=self._root.fd, follow_symlinks=False)
        except FileNotFoundError:
            return
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
            raise OfflineCliRecoveryError(
                "refusing to remove a non-regular or linked state entry"
            )
        os.unlink(name, dir_fd=self._root.fd)
        os.fsync(self._root.fd)

    def list_names(self) -> Tuple[str, ...]:
        names: List[str] = []
        with os.scandir(self._root.fd) as entries:
            for entry in entries:
                if len(names) >= MAX_STATE_ENTRIES:
                    raise OfflineCliCapacityError(
                        "offline owner state root exceeds its bounded entry count"
                    )
                names.append(entry.name)
        return tuple(names)

    def request_names(self) -> Tuple[str, ...]:
        names = sorted(
            name for name in self.list_names()
            if _REQUEST_FILE_RE.fullmatch(name) or _LEGACY_REQUEST_FILE_RE.fullmatch(name)
        )
        if len(names) > MAX_REQUEST_RECORDS:
            raise OfflineCliCapacityError(
                "offline owner durable request ledger exceeds its capacity"
            )
        return tuple(names)

    def require_request_capacity(self) -> None:
        if len(self.request_names()) >= MAX_REQUEST_RECORDS:
            raise OfflineCliCapacityError(
                "offline owner durable request ledger is full"
            )

    def cleanup_stale_temps(self) -> None:
        names = [name for name in self.list_names() if name.startswith(TEMP_FILE_PREFIX)]
        if len(names) > 256:
            raise OfflineCliRecoveryError("too many offline owner temporary state files")
        removed = False
        for name in names:
            matched = _TEMP_NAME_RE.fullmatch(name)
            if matched is None:
                continue
            owner_pid = int(matched.group(1))
            owner_start = int(matched.group(2), 16)
            dead = _owner_is_proven_dead(owner_pid, owner_start)
            if dead is None:
                raise OfflineCliRecoveryError("cannot verify stale state file owner identity")
            if not dead:
                continue
            metadata = os.stat(name, dir_fd=self._root.fd, follow_symlinks=False)
            if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
                raise OfflineCliRecoveryError(
                    "stale owner temporary state entry is not one regular file"
                )
            os.unlink(name, dir_fd=self._root.fd)
            removed = True
        if removed:
            os.fsync(self._root.fd)

    def close(self) -> None:
        if self._input_alias_dir_fd >= 0:
            os.close(self._input_alias_dir_fd)
            self._input_alias_dir_fd = -1
        if self._lock_fd >= 0:
            try:
                fcntl.flock(self._lock_fd, fcntl.LOCK_UN)
            finally:
                os.close(self._lock_fd)
                self._lock_fd = -1
        self._state_auth_key = b""


def _bounded_text(value: object, maximum_bytes: int = MAX_MESSAGE_BYTES) -> str:
    text = str(value).replace("\x00", "\\0")
    encoded = text.encode("utf-8", errors="replace")
    if len(encoded) <= maximum_bytes:
        return text
    return encoded[:maximum_bytes].decode("utf-8", errors="ignore")


def _bounded_log_text(data: bytes, maximum_json_bytes: int) -> Tuple[str, bool]:
    decoded = data.decode("utf-8", errors="replace")
    encoded = json.dumps(decoded, ensure_ascii=True).encode("utf-8")
    if len(encoded) <= maximum_json_bytes:
        return decoded, False

    low = 0
    high = len(decoded)
    while low < high:
        middle = (low + high + 1) // 2
        candidate = json.dumps(decoded[:middle], ensure_ascii=True).encode("utf-8")
        if len(candidate) <= maximum_json_bytes:
            low = middle
        else:
            high = middle - 1
    return decoded[:low], True


class _BoundedCapture:
    def __init__(self, stream: BinaryIO, maximum_bytes: int, name: str) -> None:
        self._stream = stream
        self._maximum_bytes = maximum_bytes
        self._buffer = bytearray()
        self._truncated = False
        self._error: Optional[str] = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._drain,
            name=name,
            daemon=False,
        )

    def start(self) -> None:
        self._thread.start()

    def _drain(self) -> None:
        descriptor = self._stream.fileno()
        try:
            while not self._stop.is_set():
                try:
                    readable, _, _ = select.select((descriptor,), (), (), 0.05)
                except InterruptedError:
                    continue
                if not readable:
                    continue
                try:
                    chunk = os.read(descriptor, 64 * 1024)
                except InterruptedError:
                    continue
                if not chunk:
                    return
                with self._lock:
                    remaining = self._maximum_bytes - len(self._buffer)
                    if remaining > 0:
                        self._buffer.extend(chunk[:remaining])
                    if len(chunk) > remaining:
                        self._truncated = True
        except (OSError, ValueError) as error:
            if not self._stop.is_set():
                with self._lock:
                    self._error = _bounded_text(error)
        finally:
            try:
                self._stream.close()
            except OSError:
                pass

    def finish(self, timeout_seconds: float) -> None:
        self._thread.join(timeout_seconds)
        if self._thread.is_alive():
            self._stop.set()
            self._thread.join(0.2)
        if self._thread.is_alive():
            raise OfflineCliOwnerError("bounded log drain did not stop")

    def abort(self) -> None:
        self._stop.set()
        try:
            self._stream.close()
        except OSError:
            pass
        if self._thread.ident is not None:
            self._thread.join(0.2)

    def snapshot(self) -> Tuple[str, bool, Optional[str]]:
        with self._lock:
            raw = bytes(self._buffer)
            truncated = self._truncated
            error = self._error
        text, serialization_truncated = _bounded_log_text(
            raw,
            self._maximum_bytes,
        )
        return text, truncated or serialization_truncated, error


@dataclass
class _Operation:
    request_id: str
    record: Dict[str, object]
    input_alias_name: str
    resolved_input: _ResolvedInput
    resolved_output: _ResolvedOutput
    process: subprocess.Popen
    process_start_token: int
    pgid: int
    sid: int
    monotonic_deadline: float
    stdout_capture: _BoundedCapture
    stderr_capture: _BoundedCapture
    wake: threading.Event = field(default_factory=threading.Event)
    desired_terminal: Optional[OfflineCliState] = None
    desired_error_code: Optional[str] = None
    desired_message: Optional[str] = None
    monitor: Optional[threading.Thread] = None

    def close_resolved_paths(self) -> None:
        self.resolved_input.close()
        self.resolved_output.close()


def _validate_active_journal(active: Mapping[str, object]) -> None:
    required = {
        "schema_version",
        "owner_pid",
        "owner_start_token",
        "request_id",
        "request_digest",
        "operation_id",
        "pid",
        "process_start_token",
        "pgid",
        "sid",
    }
    if not isinstance(active, dict):
        raise OfflineCliRecoveryError("active journal is not an object")
    legacy = active.get("schema_version") == LEGACY_ACTIVE_SCHEMA_VERSION
    if not legacy:
        required.remove("request_digest")
    if set(active) != required:
        raise OfflineCliRecoveryError("active journal has unknown or missing fields")
    if active.get("schema_version") not in {ACTIVE_SCHEMA_VERSION, LEGACY_ACTIVE_SCHEMA_VERSION}:
        raise OfflineCliRecoveryError("active journal schema is unsupported")
    for field_name in (
        "owner_pid",
        "owner_start_token",
        "pid",
        "process_start_token",
        "pgid",
        "sid",
    ):
        value = active.get(field_name)
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise OfflineCliRecoveryError("active journal process identity is invalid")
    request_id = active.get("request_id")
    digest = active.get("request_digest")
    operation_id = active.get("operation_id")
    if not isinstance(request_id, str) or not _REQUEST_ID_RE.fullmatch(request_id):
        raise OfflineCliRecoveryError("active journal request ID is invalid")
    if legacy and (not isinstance(digest, str) or not _SHA256_RE.fullmatch(digest)):
        raise OfflineCliRecoveryError("active journal request digest is invalid")
    if not isinstance(operation_id, str) or not _OPERATION_ID_RE.fullmatch(operation_id):
        raise OfflineCliRecoveryError("active journal operation ID is invalid")
    if active["pid"] != active["pgid"] or active["pid"] != active["sid"]:
        raise OfflineCliRecoveryError("active journal is not an isolated process group")


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


def _open_and_read_stable_regular_file(
    directory_fd: int, name: str, label: str, maximum_bytes: int
) -> Tuple[bytes, int, os.stat_result]:
    descriptor = os.open(
        name,
        os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW,
        dir_fd=directory_fd,
    )
    try:
        opened = os.fstat(descriptor)
        if not stat.S_ISREG(opened.st_mode) or opened.st_nlink != 1:
            raise OfflineCliOwnerError("%s is not a single-link regular file" % label)
        if opened.st_size < 0 or opened.st_size > maximum_bytes:
            raise OfflineCliOwnerError(
                "%s exceeds the %d-byte inspection limit" % (label, maximum_bytes)
            )
        payload = bytearray()
        while len(payload) < opened.st_size:
            chunk = os.read(
                descriptor,
                min(64 * 1024, opened.st_size - len(payload)),
            )
            if not chunk:
                break
            payload.extend(chunk)
        if os.read(descriptor, 1):
            raise OfflineCliOwnerError("%s grew while it was being inspected" % label)
        completed = os.fstat(descriptor)
        if (
            not _same_identity(opened, completed)
            or completed.st_nlink != 1
            or completed.st_size != opened.st_size
            or completed.st_mtime_ns != opened.st_mtime_ns
            or completed.st_ctime_ns != opened.st_ctime_ns
            or len(payload) != opened.st_size
        ):
            raise OfflineCliOwnerError("%s changed while it was being inspected" % label)
        return bytes(payload), descriptor, opened
    except Exception:
        os.close(descriptor)
        raise


def _exact_json_object(
    value: object, expected_fields: frozenset, label: str
) -> Mapping[str, object]:
    if not isinstance(value, dict) or set(value) != expected_fields:
        raise OfflineCliOwnerError("%s field set is not exact" % label)
    return value


def _finite_json_number(value: object) -> bool:
    if type(value) not in (int, float):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


def _parse_strict_pgm_p5_header(header: bytes) -> Tuple[int, int, int]:
    first_newline = header.find(b"\n")
    second_newline = header.find(b"\n", first_newline + 1)
    third_newline = header.find(b"\n", second_newline + 1)
    if (
        first_newline < 0
        or second_newline < 0
        or third_newline < 0
        or third_newline + 1 != len(header)
        or header[: first_newline + 1] != b"P5\n"
    ):
        raise OfflineCliOwnerError("map raster is not strict PGM P5")
    dimensions = header[first_newline + 1 : second_newline]
    if re.fullmatch(rb"[1-9][0-9]* [1-9][0-9]*", dimensions) is None:
        raise OfflineCliOwnerError("map raster dimensions are not canonical PGM")
    width_text, height_text = dimensions.split(b" ", 1)
    maxval_text = header[second_newline + 1 : third_newline]
    if maxval_text != b"255":
        raise OfflineCliOwnerError("map raster maxval must be exactly 255")
    width = int(width_text)
    height = int(height_text)
    if width > MAX_MAP_GRID_DIMENSION or height > MAX_MAP_GRID_DIMENSION:
        raise OfflineCliOwnerError("map raster dimensions exceed the frozen u32 grid boundary")
    cells = width * height
    if cells > MAX_MAP_GRID_CELLS:
        raise OfflineCliOwnerError("map raster dimensions exceed the platform byte boundary")
    return width, height, 255


def _open_and_inspect_stable_pgm_p5(
    directory_fd: int, name: str, label: str
) -> Tuple[int, int, int, int, int, os.stat_result]:
    descriptor = os.open(
        name,
        os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW,
        dir_fd=directory_fd,
    )
    try:
        opened = os.fstat(descriptor)
        if not stat.S_ISREG(opened.st_mode) or opened.st_nlink != 1:
            raise OfflineCliOwnerError("%s is not a single-link regular file" % label)
        if opened.st_size < 0 or opened.st_size > MAX_PGM_FILE_BYTES:
            raise OfflineCliOwnerError("%s exceeds the frozen grid byte boundary" % label)
        prefix = bytearray()
        third_newline = -1
        while third_newline < 0 and len(prefix) < MAX_PGM_HEADER_BYTES + 1:
            chunk = os.read(
                descriptor,
                MAX_PGM_HEADER_BYTES + 1 - len(prefix),
            )
            if not chunk:
                break
            prefix.extend(chunk)
            first_newline = prefix.find(b"\n")
            second_newline = prefix.find(b"\n", first_newline + 1)
            third_newline = prefix.find(b"\n", second_newline + 1)
        if third_newline < 0 or third_newline + 1 > MAX_PGM_HEADER_BYTES:
            raise OfflineCliOwnerError("%s header exceeds the canonical PGM limit" % label)
        header_size = third_newline + 1
        width, height, maxval = _parse_strict_pgm_p5_header(bytes(prefix[:header_size]))
        raster_size = width * height
        expected_size = header_size + raster_size
        if opened.st_size != expected_size:
            raise OfflineCliOwnerError(
                "%s payload length disagrees with PGM dimensions" % label
            )
        completed = os.fstat(descriptor)
        if (
            not _same_identity(opened, completed)
            or completed.st_nlink != 1
            or completed.st_size != opened.st_size
            or completed.st_mtime_ns != opened.st_mtime_ns
            or completed.st_ctime_ns != opened.st_ctime_ns
        ):
            raise OfflineCliOwnerError("%s changed while it was being inspected" % label)
        return (
            width,
            height,
            maxval,
            opened.st_size,
            descriptor,
            opened,
        )
    except Exception:
        os.close(descriptor)
        raise


def _inspect_map_bundle(
    resolved: _ResolvedOutput,
    command: Mapping[str, object],
    request_id: str,
    dataset_size: int,
    dataset_format: str,
    dataset_device: int,
    dataset_inode: int,
    dataset_mtime_ns: int,
    dataset_ctime_ns: int,
    release_generation: str,
    release_manifest_sha256: str,
    artifact_sha256: str,
    artifact_size: int,
) -> Tuple[int, Mapping[str, object]]:
    resolved.verify_parent_identity()
    final_directory_fd = os.open(
        resolved.path.name,
        _directory_flags(),
        dir_fd=resolved.parent_fd,
    )
    member_descriptors: List[int] = []
    try:
        opened_directory = os.fstat(final_directory_fd)
        names = {entry.name for entry in os.scandir(final_directory_fd)}
        if names != {"manifest.json", "asset.json", "map.pgm"}:
            raise OfflineCliOwnerError("map bundle member set is not exact")
        member_bytes: Dict[str, bytes] = {}
        member_metadata: Dict[str, os.stat_result] = {}
        for name, label in (
            ("manifest.json", "map manifest"),
            ("asset.json", "map asset"),
        ):
            payload, descriptor, opened = _open_and_read_stable_regular_file(
                final_directory_fd,
                name,
                label,
                MAX_CANONICAL_MAP_JSON_BYTES,
            )
            member_descriptors.append(descriptor)
            member_bytes[name] = payload
            member_metadata[name] = opened
        (
            pgm_width,
            pgm_height,
            pgm_maxval,
            map_size,
            map_descriptor,
            map_metadata,
        ) = _open_and_inspect_stable_pgm_p5(
            final_directory_fd,
            "map.pgm",
            "map raster",
        )
        member_descriptors.append(map_descriptor)
        member_metadata["map.pgm"] = map_metadata
        manifest_bytes = member_bytes["manifest.json"]
        asset_bytes = member_bytes["asset.json"]
        completed_directory = os.fstat(final_directory_fd)
        named_directory = os.stat(
            resolved.path.name,
            dir_fd=resolved.parent_fd,
            follow_symlinks=False,
        )
        if (
            not _same_identity(opened_directory, completed_directory)
            or not _same_identity(opened_directory, named_directory)
            or not stat.S_ISDIR(completed_directory.st_mode)
            or not stat.S_ISDIR(named_directory.st_mode)
            or completed_directory.st_mtime_ns != opened_directory.st_mtime_ns
            or completed_directory.st_ctime_ns != opened_directory.st_ctime_ns
        ):
            raise OfflineCliOwnerError("map bundle directory changed while it was being inspected")
        for name, opened in member_metadata.items():
            named = os.stat(name, dir_fd=final_directory_fd, follow_symlinks=False)
            if (
                not _same_identity(opened, named)
                or not stat.S_ISREG(named.st_mode)
                or named.st_nlink != 1
                or named.st_size != opened.st_size
                or named.st_mtime_ns != opened.st_mtime_ns
                or named.st_ctime_ns != opened.st_ctime_ns
            ):
                raise OfflineCliOwnerError("map bundle member changed during inspection: %s" % name)
    finally:
        for descriptor in member_descriptors:
            os.close(descriptor)
        os.close(final_directory_fd)
    manifest = json.loads(manifest_bytes.decode("utf-8"))
    if manifest_bytes != _canonical_json_bytes(manifest):
        raise OfflineCliOwnerError("map manifest is not canonical JSON")
    expected_manifest_keys = {"schema_version", "map_name", "frame_id", "asset_path", "files", "request_id"}
    if not isinstance(manifest, dict) or set(manifest) != expected_manifest_keys:
        raise OfflineCliOwnerError("map manifest field set is not exact")
    if (
        manifest["schema_version"] != "nuwax.mapping.offline-map-manifest.v2"
        or manifest["map_name"] != command["map_name"]
        or manifest["frame_id"] != command["frame_id"]
        or manifest["asset_path"] != "asset.json"
        or manifest["request_id"] != request_id
    ):
        raise OfflineCliOwnerError("map manifest identity is inconsistent")
    files = manifest["files"]
    if not isinstance(files, list) or not all(isinstance(item, dict) for item in files) or [item.get("path") for item in files] != ["asset.json", "map.pgm"]:
        raise OfflineCliOwnerError("map manifest files are not exact and sorted")
    verified_files: List[Mapping[str, object]] = []
    member_proofs = {
        "asset.json": len(asset_bytes),
        "map.pgm": map_size,
    }
    total_size = len(manifest_bytes)
    for item in files:
        if not isinstance(item, dict) or set(item) != {"path", "size"}:
            raise OfflineCliOwnerError("map manifest file proof shape is invalid")
        member_size = member_proofs[str(item["path"])]
        if type(item["size"]) is not int or item["size"] != member_size:
            raise OfflineCliOwnerError("map bundle member proof mismatch")
        total_size += member_size
        verified_files.append(dict(item))
    asset = json.loads(asset_bytes.decode("utf-8"))
    if asset_bytes != _canonical_json_bytes(asset):
        raise OfflineCliOwnerError("map asset is not canonical JSON")
    asset = _exact_json_object(
        asset,
        frozenset(
            {"schema_version", "map_name", "frame_id", "source", "grid", "producer", "request_id"}
        ),
        "map asset",
    )
    if (
        asset["schema_version"] != "nuwax.mapping.offline-map-asset.v2"
        or asset["map_name"] != command["map_name"]
        or asset["frame_id"] != command["frame_id"]
        or asset["request_id"] != request_id
    ):
        raise OfflineCliOwnerError("map asset identity is inconsistent")

    source = _exact_json_object(
        asset["source"],
        frozenset({"dataset_size", "input_format", "dataset_device", "dataset_inode", "dataset_mtime_ns", "dataset_ctime_ns"}),
        "map asset source",
    )
    if any(type(source[field]) is not int for field in source if field != "input_format") or source != {
        "dataset_size": dataset_size,
        "input_format": dataset_format,
        "dataset_device": dataset_device,
        "dataset_inode": dataset_inode,
        "dataset_mtime_ns": dataset_mtime_ns,
        "dataset_ctime_ns": dataset_ctime_ns,
    }:
        raise OfflineCliOwnerError("map asset source proof does not match admitted input observation")

    grid = _exact_json_object(
        asset["grid"],
        frozenset(
            {"width", "height", "resolution_m", "origin_xy_yaw", "encoding", "path"}
        ),
        "map asset grid",
    )
    origin = grid["origin_xy_yaw"]
    if (
        type(grid["width"]) is not int
        or not 1 <= grid["width"] <= 0xFFFFFFFF
        or type(grid["height"]) is not int
        or not 1 <= grid["height"] <= 0xFFFFFFFF
        or not _finite_json_number(grid["resolution_m"])
        or grid["resolution_m"] <= 0
        or not isinstance(origin, list)
        or len(origin) != 3
        or not all(_finite_json_number(value) for value in origin)
        or grid["encoding"] != "pgm-p5-cost-inverted-v1"
        or grid["path"] != "map.pgm"
    ):
        raise OfflineCliOwnerError("map asset grid is invalid")
    if (
        pgm_width != grid["width"]
        or pgm_height != grid["height"]
        or pgm_maxval != 255
        or map_size != len(
            ("P5\n%d %d\n255\n" % (grid["width"], grid["height"])).encode("ascii")
        )
        + grid["width"] * grid["height"]
    ):
        raise OfflineCliOwnerError("map raster proof disagrees with grid metadata")

    producer = _exact_json_object(
        asset["producer"],
        frozenset(
            {
                "role",
                "release_generation",
                "release_manifest_sha256",
                "artifact_sha256",
            }
        ),
        "map asset producer",
    )
    if producer != {
        "role": "offline_mapping_cli",
        "release_generation": release_generation,
        "release_manifest_sha256": release_manifest_sha256,
        "artifact_sha256": artifact_sha256,
    } or artifact_size <= 0 or any(
        _SHA256_RE.fullmatch(value) is None
        for value in (
            release_generation,
            release_manifest_sha256,
            artifact_sha256,
        )
    ):
        raise OfflineCliOwnerError("map asset producer proof is inconsistent")
    receipt: Mapping[str, object] = {
        "schema_version": 2,
        "map_name": command["map_name"],
        "frame_id": command["frame_id"],
        "output_directory": str(resolved.path),
        "bundle_size": total_size,
        "dataset_size": dataset_size,
        "dataset_format": dataset_format,
        "dataset_device": dataset_device,
        "dataset_inode": dataset_inode,
        "dataset_mtime_ns": dataset_mtime_ns,
        "dataset_ctime_ns": dataset_ctime_ns,
        "offline_cli_artifact_size": artifact_size,
        "files": verified_files,
        "release_generation": release_generation,
        "release_manifest_sha256": release_manifest_sha256,
        "offline_cli_artifact_sha256": artifact_sha256,
        "request_id": request_id,
    }
    return total_size, receipt


def _inspect_output(
    resolved: _ResolvedOutput,
    relative_path: str,
) -> Tuple[int, Optional[Mapping[str, object]]]:
    size = resolved.inspect_final()
    if Path(relative_path).suffix == ".db":
        for suffix in ("-journal", "-shm", "-wal"):
            try:
                metadata = os.stat(
                    resolved.path.name + suffix,
                    dir_fd=resolved.parent_fd,
                    follow_symlinks=False,
                )
            except FileNotFoundError:
                continue
            if stat.S_ISREG(metadata.st_mode):
                raise OfflineCliOwnerError("durable SQLite output still has a companion file")
            raise OfflineCliOwnerError("durable SQLite companion path is not a regular file")
    return size, None


class OfflineMappingCliOwner:
    """四个离线建图命令的单任务进程宿主。"""

    def __init__(
        self,
        binary_path: Union[str, os.PathLike[str]],
        *,
        release_proof: OfflineCliReleaseProof,
        input_root: Union[str, os.PathLike[str]],
        output_root: Union[str, os.PathLike[str]],
        state_root: Union[str, os.PathLike[str]],
        state_auth_key: bytes,
        max_log_bytes: int = DEFAULT_MAX_LOG_BYTES,
        startup_timeout_seconds: float = DEFAULT_STARTUP_TIMEOUT_SECONDS,
        termination_grace_seconds: float = DEFAULT_TERMINATION_GRACE_SECONDS,
        kill_grace_seconds: float = DEFAULT_KILL_GRACE_SECONDS,
        poll_interval_seconds: float = DEFAULT_POLL_INTERVAL_SECONDS,
    ) -> None:
        if (
            type(state_auth_key) is not bytes
            or len(state_auth_key) < MIN_STATE_AUTH_KEY_BYTES
            or len(state_auth_key) > MAX_STATE_AUTH_KEY_BYTES
        ):
            raise OfflineCliValidationError(
                "state_auth_key must be external bytes within the supported secret size"
            )
        if (
            isinstance(max_log_bytes, bool)
            or not isinstance(max_log_bytes, int)
            or max_log_bytes < 2
            or max_log_bytes > MAX_CONFIGURED_LOG_BYTES
        ):
            raise OfflineCliValidationError("max_log_bytes is outside the bounded range")
        self._release_proof = _validate_release_proof(release_proof)
        self._release_generation = self._release_proof.generation
        self._release_manifest_sha256 = self._release_proof.manifest_sha256
        self._artifact_sha256 = self._release_proof.artifact_sha256
        self._artifact_size = self._release_proof.artifact_size
        self._max_log_bytes = max_log_bytes
        self._startup_timeout_seconds = _require_finite(
            startup_timeout_seconds,
            "startup_timeout_seconds",
            minimum=1e-3,
            maximum=60.0,
            allow_zero=False,
        )
        self._termination_grace_seconds = _require_finite(
            termination_grace_seconds,
            "termination_grace_seconds",
            minimum=0.0,
            maximum=60.0,
        )
        self._kill_grace_seconds = _require_finite(
            kill_grace_seconds,
            "kill_grace_seconds",
            minimum=1e-3,
            maximum=60.0,
            allow_zero=False,
        )
        self._poll_interval_seconds = _require_finite(
            poll_interval_seconds,
            "poll_interval_seconds",
            minimum=1e-3,
            maximum=1.0,
            allow_zero=False,
        )

        self._condition = threading.Condition(threading.RLock())
        self._active: Optional[_Operation] = None
        self._volatile_records: Dict[str, Dict[str, object]] = {}
        self._record_names: Dict[str, str] = {}
        self._closing = False
        self._resources_closed = False
        self._poisoned: Optional[str] = None
        self._binary_path = Path()
        self._binary_parent_fd = -1
        self._binary_fd = -1
        self._binary_parent_metadata: Optional[os.stat_result] = None
        self._binary_metadata: Optional[os.stat_result] = None
        self._input_root: Optional[_StableRoot] = None
        self._output_root: Optional[_StableRoot] = None
        self._state_root: Optional[_StableRoot] = None
        self._state_store: Optional[_OwnerStateStore] = None

        owner_pid = os.getpid()
        kind, owner_info = _probe_process(owner_pid)
        if kind != _ProbeKind.RUNNING or owner_info is None or owner_info.start_token <= 0 or owner_info.state == "Z":
            raise OfflineCliRecoveryError("cannot prove current owner process identity")
        self._owner_pid = owner_pid
        self._owner_start_token = owner_info.start_token

        try:
            self._open_binary(binary_path, self._release_proof)
            self._input_root = _StableRoot(input_root, "input")
            self._output_root = _StableRoot(output_root, "output")
            self._state_root = _StableRoot(state_root, "state")
            if _roots_overlap(self._state_root, self._input_root) or _roots_overlap(
                self._state_root,
                self._output_root,
            ):
                raise OfflineCliValidationError(
                    "state root must not overlap the input or output root"
                )
            self._state_store = _OwnerStateStore(
                self._state_root,
                owner_pid=self._owner_pid,
                owner_start_token=self._owner_start_token,
                state_auth_key=state_auth_key,
            )
            self._state_store.acquire_lock()
            self._index_request_records()
            self._state_store.cleanup_stale_temps()
            self._recover_stale_active()
            self._recover_orphaned_records()
            self._state_store.cleanup_stale_input_aliases()
        except Exception:
            self._close_resources_unlocked()
            raise

    def _open_binary(
        self,
        binary_path: Union[str, os.PathLike[str]],
        proof: OfflineCliReleaseProof,
    ) -> None:
        expanded = Path(os.path.expanduser(os.fspath(binary_path)))
        if not expanded.is_absolute() or not expanded.name:
            raise OfflineCliValidationError("binary path must be an absolute file path")
        path = Path(os.path.abspath(expanded))
        if path != Path(proof.artifact_path):
            raise OfflineCliValidationError("binary path does not match the verified release artifact")
        parent_fd = _open_absolute_directory(path.parent, "binary parent")
        descriptor = -1
        try:
            descriptor = os.open(
                path.name,
                os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW,
                dir_fd=parent_fd,
            )
            metadata = os.fstat(descriptor)
            proof_metadata = os.fstat(proof.artifact_fd)
            if (
                not _same_identity(metadata, proof_metadata)
                or metadata.st_size != proof.artifact_size
                or _sha256_fd(proof.artifact_fd) != proof.artifact_sha256
            ):
                raise OfflineCliValidationError("binary does not match the retained release artifact FD")
            if not stat.S_ISREG(metadata.st_mode) or metadata.st_mode & 0o111 == 0:
                raise OfflineCliValidationError("mapping CLI binary is not a regular executable")
            if not os.access(str(path), os.X_OK, follow_symlinks=False):
                raise OfflineCliValidationError("mapping CLI binary is not executable")
            self._binary_path = path
            self._binary_parent_fd = parent_fd
            self._binary_fd = descriptor
            self._binary_parent_metadata = os.fstat(parent_fd)
            self._binary_metadata = metadata
        except OSError as error:
            if descriptor >= 0:
                os.close(descriptor)
            os.close(parent_fd)
            raise OfflineCliValidationError("mapping CLI binary path is not a stable regular file") from error
        except Exception:
            if descriptor >= 0:
                os.close(descriptor)
            os.close(parent_fd)
            raise

    def _verify_binary_identity(self) -> None:
        if self._binary_parent_metadata is None or self._binary_metadata is None:
            raise OfflineCliClosedError("mapping CLI binary identity is unavailable")
        current_parent_fd = _open_absolute_directory(
            self._binary_path.parent,
            "binary parent",
        )
        try:
            current_parent = os.fstat(current_parent_fd)
            current_binary = os.stat(
                self._binary_path.name,
                dir_fd=current_parent_fd,
                follow_symlinks=False,
            )
            if not _same_identity(current_parent, self._binary_parent_metadata):
                raise OfflineCliValidationError("mapping CLI binary parent changed")
            if (
                not _same_identity(current_binary, self._binary_metadata)
                or not stat.S_ISREG(current_binary.st_mode)
                or current_binary.st_size != self._binary_metadata.st_size
                or current_binary.st_mtime_ns != self._binary_metadata.st_mtime_ns
                or current_binary.st_ctime_ns != self._binary_metadata.st_ctime_ns
            ):
                raise OfflineCliValidationError("mapping CLI binary identity changed")
        finally:
            os.close(current_parent_fd)

    def _close_resources_unlocked(self) -> None:
        store = self._state_store
        self._state_store = None
        if store is not None:
            store.close()
        for attribute in ("_state_root", "_output_root", "_input_root"):
            root = getattr(self, attribute)
            setattr(self, attribute, None)
            if root is not None:
                root.close()
        if self._binary_fd >= 0:
            os.close(self._binary_fd)
            self._binary_fd = -1
        if self._binary_parent_fd >= 0:
            os.close(self._binary_parent_fd)
            self._binary_parent_fd = -1
        self._resources_closed = True

    def _require_store(self) -> _OwnerStateStore:
        if self._state_store is None:
            raise OfflineCliClosedError("offline CLI owner resources are closed")
        return self._state_store

    def _assert_owner_process_locked(self) -> None:
        if os.getpid() != self._owner_pid:
            raise OfflineCliRecoveryError(
                "offline CLI owner must not be used by a forked process"
            )
        kind, info = _probe_process(self._owner_pid)
        if (
            kind != _ProbeKind.RUNNING
            or info is None
            or info.state == "Z"
            or info.start_token != self._owner_start_token
        ):
            raise OfflineCliRecoveryError(
                "current offline CLI owner process identity is unprovable"
            )

    def _ensure_startable_locked(self) -> None:
        self._assert_owner_process_locked()
        if self._closing or self._resources_closed:
            raise OfflineCliClosedError("offline CLI owner is closing or closed")
        if self._poisoned is not None:
            raise OfflineCliRecoveryError(self._poisoned)

    def _poison_locked(self, message: object) -> None:
        if self._poisoned is None:
            self._poisoned = _bounded_text(message)

    def _index_request_records(self) -> None:
        store = self._require_store()
        for name in store.request_names():
            record = store.read_json(name)
            if record is None:
                raise OfflineCliRecoveryError("durable request disappeared during indexing")
            _validate_record_shape(record)
            request_id = str(record["request_id"])
            legacy = record["schema_version"] == LEGACY_REQUEST_SCHEMA_VERSION
            if ((legacy and _LEGACY_REQUEST_FILE_RE.fullmatch(name) is None)
                    or (not legacy and name != store.request_name(request_id))):
                raise OfflineCliRecoveryError("durable request filename/schema mismatch")
            if request_id in self._record_names:
                raise OfflineCliRecoveryError("multiple authenticated records have the same request ID")
            self._record_names[request_id] = name

    def _write_record_locked(self, record: Mapping[str, object]) -> None:
        try:
            _validate_record_shape(record)
            if record["schema_version"] != REQUEST_SCHEMA_VERSION:
                raise OfflineCliRecoveryError("legacy request identities are read-only")
            request_id = str(record["request_id"])
            name = self._require_store().request_name(request_id)
            self._require_store().write_json(name, record)
            self._record_names[request_id] = name
        except Exception as error:
            self._poison_locked("durable request state write failed: %s" % error)
            raise

    def _write_recovered_record(self, previous: Mapping[str, object], record: Mapping[str, object]) -> None:
        if record["schema_version"] != LEGACY_REQUEST_SCHEMA_VERSION:
            self._write_record_locked(record)
            return
        allowed = {"state", "terminal", "completed_at_unix_ns", "error_code", "message"}
        if (previous["terminal"] or record["state"] != OfflineCliState.FAILED.value
                or not record["terminal"]
                or any(record[key] != value for key, value in previous.items() if key not in allowed)):
            raise OfflineCliRecoveryError("legacy recovery attempted to change the original request identity")
        _validate_record_shape(record)
        name = self._record_names[str(record["request_id"])]
        self._require_store().write_json(name, record)

    def _read_record_locked(self, request_id: str) -> Optional[Dict[str, object]]:
        volatile = self._volatile_records.get(request_id)
        if volatile is not None:
            return dict(volatile)
        store = self._require_store()
        name = self._record_names.get(request_id, store.request_name(request_id))
        record = store.read_json(name)
        if record is None:
            return None
        _validate_record_shape(record)
        if record["request_id"] != request_id:
            raise OfflineCliRecoveryError("durable request filename identity mismatch")
        return record

    def _verified_group_members(
        self,
        *,
        pid: int,
        process_start_token: int,
        pgid: int,
        sid: int,
    ) -> List[_ProcessInfo]:
        if pid != pgid or pid != sid or pid <= 0:
            raise OfflineCliRecoveryError("process is not its isolated group and session leader")
        if pgid == os.getpgrp() or sid == os.getsid(0):
            raise OfflineCliRecoveryError("refusing to signal the owner process group")
        kind, info = _probe_process(pid)
        members = _group_members(pgid, sid)
        if kind == _ProbeKind.UNVERIFIABLE:
            raise OfflineCliRecoveryError("cannot verify child process identity")
        if kind == _ProbeKind.RUNNING and info is not None:
            if info.start_token != process_start_token:
                if members:
                    raise OfflineCliRecoveryError("child PID was reused while the recorded group still exists")
                return []
            if info.pgrp != pgid or info.session != sid:
                raise OfflineCliRecoveryError("child process identity left its recorded group")
        if not members:
            try:
                os.killpg(pgid, 0)
            except ProcessLookupError:
                pass
            except PermissionError as error:
                raise OfflineCliRecoveryError(
                    "cannot prove ownership of the recorded process group"
                ) from error
            else:
                raise OfflineCliRecoveryError(
                    "recorded process group exists without verifiable members"
                )
        return members

    def _wait_group_empty(
        self,
        *,
        pid: int,
        process_start_token: int,
        pgid: int,
        sid: int,
        timeout_seconds: float,
        process: Optional[subprocess.Popen],
    ) -> bool:
        deadline = time.monotonic() + timeout_seconds
        while True:
            if process is not None:
                process.poll()
            members = self._verified_group_members(
                pid=pid,
                process_start_token=process_start_token,
                pgid=pgid,
                sid=sid,
            )
            if not members:
                if process is not None:
                    try:
                        process.wait(timeout=0)
                    except subprocess.TimeoutExpired:
                        pass
                return True
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False
            time.sleep(min(self._poll_interval_seconds, remaining))

    def _terminate_identity(
        self,
        *,
        pid: int,
        process_start_token: int,
        pgid: int,
        sid: int,
        process: Optional[subprocess.Popen],
    ) -> None:
        for sig, timeout_seconds in (
            (signal.SIGTERM, self._termination_grace_seconds),
            (signal.SIGKILL, self._kill_grace_seconds),
        ):
            members = self._verified_group_members(
                pid=pid,
                process_start_token=process_start_token,
                pgid=pgid,
                sid=sid,
            )
            if not members:
                break
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                pass
            except PermissionError as error:
                raise OfflineCliRecoveryError("permission denied while terminating offline process group") from error
            if self._wait_group_empty(
                pid=pid,
                process_start_token=process_start_token,
                pgid=pgid,
                sid=sid,
                timeout_seconds=timeout_seconds,
                process=process,
            ):
                break
        else:
            members = self._verified_group_members(
                pid=pid,
                process_start_token=process_start_token,
                pgid=pgid,
                sid=sid,
            )
            if members:
                raise OfflineCliRecoveryError("offline process group survived SIGKILL")
        if not self._wait_group_empty(
            pid=pid,
            process_start_token=process_start_token,
            pgid=pgid,
            sid=sid,
            timeout_seconds=self._kill_grace_seconds,
            process=process,
        ):
            raise OfflineCliRecoveryError("offline process group cleanup is unproven")
        if process is not None:
            try:
                process.wait(timeout=self._kill_grace_seconds)
            except subprocess.TimeoutExpired as error:
                raise OfflineCliRecoveryError("direct CLI child was not reaped") from error

    def _cleanup_map_staging(
        self,
        record: Mapping[str, object],
        pid: int,
        process_start_token: int,
        output_parent_fd: Optional[int] = None,
    ) -> None:
        if record["command_kind"] != OfflineCliCommandKind.MAP.value:
            return
        command = record["command"]
        if not isinstance(command, Mapping):
            raise OfflineCliRecoveryError("durable map command is unavailable for staging cleanup")
        output_parts = _relative_parts(
            str(command["output_path"]),
            "durable map output path",
        )
        final_name = os.fsencode(output_parts[-1])
        expected = re.compile(
            re.escape(".nuwax-mapping-cli-stage-v2-bundle-")
            + str(pid)
            + "-"
            + ("%016x" % process_start_token)
            + r"-[0-9a-f]{16}\Z"
        )

        def require_bound(parent_fd: int, name: bytes, metadata: os.stat_result) -> None:
            current = os.stat(name, dir_fd=parent_fd, follow_symlinks=False)
            if not _same_identity(current, metadata) or stat.S_IFMT(current.st_mode) != stat.S_IFMT(metadata.st_mode):
                raise OfflineCliRecoveryError("MAP staging entry identity changed")

        def remove_tree(parent_fd: int, name: str) -> None:
            try:
                child_fd = os.open(name, _directory_flags(), dir_fd=parent_fd)
            except NotADirectoryError:
                os.unlink(name, dir_fd=parent_fd)
                return
            try:
                with os.scandir(child_fd) as entries:
                    child_names = [entry.name for entry in entries]
                for child_name in child_names:
                    remove_tree(child_fd, child_name)
            finally:
                os.close(child_fd)
            os.rmdir(name, dir_fd=parent_fd)

        owned_parent_fd = -1
        try:
            if output_parent_fd is None:
                if self._output_root is None:
                    raise OfflineCliClosedError("configured output root is unavailable")
                self._output_root.verify_current_identity()
                owned_parent_fd = self._output_root._open_parent(output_parts)
                parent_fd = owned_parent_fd
            else:
                parent_fd = output_parent_fd
            with os.scandir(parent_fd) as entries:
                names = [os.fsencode(entry.name) for entry in entries if expected.fullmatch(entry.name)]
            for name in names:
                wrapper_fd = -1
                contents_fd = -1
                try:
                    # Legacy, unbound, foreign, symlink and unknown shapes stay intact.
                    try:
                        wrapper_fd = os.open(name, _directory_flags(), dir_fd=parent_fd)
                        wrapper_metadata = os.fstat(wrapper_fd)
                        with os.scandir(wrapper_fd) as entries:
                            wrapper_names = [os.fsencode(entry.name) for entry in entries]
                        if wrapper_names != [final_name]:
                            continue
                        contents_fd = os.open(final_name, _directory_flags(), dir_fd=wrapper_fd)
                    except OSError:
                        continue
                    contents_metadata = os.fstat(contents_fd)

                    def require_area() -> None:
                        require_bound(parent_fd, name, wrapper_metadata)
                        require_bound(wrapper_fd, final_name, contents_metadata)

                    with os.scandir(contents_fd) as entries:
                        content_names = [os.fsencode(entry.name) for entry in entries]
                    if content_names not in ([], [b"payload"]):
                        continue
                    payload_metadata = None
                    if content_names:
                        payload_metadata = os.stat(b"payload", dir_fd=contents_fd, follow_symlinks=False)
                        if not stat.S_ISDIR(payload_metadata.st_mode):
                            continue
                    require_area()

                    if payload_metadata is not None:
                        remove_tree(contents_fd, b"payload")
                        os.fsync(contents_fd)
                    # Never recursively remove the outer directories: a newly
                    # inserted sibling or replacement payload must be retained.
                    require_area()
                    os.rmdir(final_name, dir_fd=wrapper_fd)
                    os.fsync(wrapper_fd)
                    require_bound(parent_fd, name, wrapper_metadata)
                    os.rmdir(name, dir_fd=parent_fd)
                    os.fsync(parent_fd)
                finally:
                    if contents_fd >= 0:
                        os.close(contents_fd)
                    if wrapper_fd >= 0:
                        os.close(wrapper_fd)
        except BaseException as error:
            raise OfflineCliRecoveryError("MAP staging cleanup is unproven") from error
        finally:
            if owned_parent_fd >= 0:
                os.close(owned_parent_fd)

    def _recover_stale_active(self) -> None:
        store = self._require_store()
        active = store.read_json(ACTIVE_JOURNAL_NAME)
        if active is None:
            return
        _validate_active_journal(active)
        dead = _owner_is_proven_dead(
            int(active["owner_pid"]),
            int(active["owner_start_token"]),
        )
        if dead is None:
            raise OfflineCliRecoveryError("cannot verify prior owner identity")
        if not dead:
            raise OfflineCliRecoveryError("prior offline CLI owner is still alive")

        request_id = str(active["request_id"])
        record = self._read_record_locked(request_id)
        if record is None:
            raise OfflineCliRecoveryError("active journal has no durable request record")
        _validate_record_shape(record)
        exact_fields = (
            "request_id",
            "operation_id",
            "pid",
            "process_start_token",
            "pgid",
            "sid",
        )
        legacy = active["schema_version"] == LEGACY_ACTIVE_SCHEMA_VERSION
        if (legacy != (record["schema_version"] == LEGACY_REQUEST_SCHEMA_VERSION)
                or (legacy and record["request_digest"] != active["request_digest"])
                or any(record[field] != active[field] for field in exact_fields)):
            raise OfflineCliRecoveryError("active journal and request identity disagree")

        members = self._verified_group_members(
            pid=int(active["pid"]),
            process_start_token=int(active["process_start_token"]),
            pgid=int(active["pgid"]),
            sid=int(active["sid"]),
        )
        if members:
            self._terminate_identity(
                pid=int(active["pid"]),
                process_start_token=int(active["process_start_token"]),
                pgid=int(active["pgid"]),
                sid=int(active["sid"]),
                process=None,
            )
        self._cleanup_map_staging(
            record,
            int(active["pid"]),
            int(active["process_start_token"]),
        )
        if not bool(record["terminal"]):
            previous = dict(record)
            record.update(
                {
                    "state": OfflineCliState.FAILED.value,
                    "terminal": True,
                    "completed_at_unix_ns": time.time_ns(),
                    "error_code": "owner_crash_outcome_unknown",
                    "message": (
                        "prior owner crashed after durable launch authorization; "
                        "the process group was cleaned, but a published output must "
                        "not be retried or activated automatically"
                    ),
                }
            )
            self._write_recovered_record(previous, record)
        store.remove(ACTIVE_JOURNAL_NAME)

    def _recover_orphaned_records(self) -> None:
        store = self._require_store()
        for name in store.request_names():
            record = store.read_json(name)
            if record is None:
                raise OfflineCliRecoveryError(
                    "durable request record disappeared during recovery"
                )
            _validate_record_shape(record)
            expected_name = self._record_names.get(str(record["request_id"]))
            if name != expected_name:
                raise OfflineCliRecoveryError(
                    "durable request filename identity mismatch"
                )
            if bool(record["terminal"]):
                continue
            if record["pid"] is not None:
                members = self._verified_group_members(
                    pid=int(record["pid"]),
                    process_start_token=int(record["process_start_token"]),
                    pgid=int(record["pgid"]),
                    sid=int(record["sid"]),
                )
                if members:
                    self._terminate_identity(
                        pid=int(record["pid"]),
                        process_start_token=int(record["process_start_token"]),
                        pgid=int(record["pgid"]),
                        sid=int(record["sid"]),
                        process=None,
                    )
                self._cleanup_map_staging(
                    record,
                    int(record["pid"]),
                    int(record["process_start_token"]),
                )
            previous = dict(record)
            record.update(
                {
                    "state": OfflineCliState.FAILED.value,
                    "terminal": True,
                    "completed_at_unix_ns": time.time_ns(),
                    "error_code": "owner_crash_recovered",
                    "message": (
                        "prior owner crashed without a recoverable active process journal"
                    ),
                }
            )
            self._write_recovered_record(previous, record)

    def _await_launcher_identity(
        self,
        process: subprocess.Popen,
        ready_fd: int,
        request_deadline: float,
    ) -> _ProcessInfo:
        startup_deadline = time.monotonic() + self._startup_timeout_seconds
        deadline = min(startup_deadline, request_deadline)
        while True:
            if process.poll() is not None:
                raise OfflineCliOwnerError(
                    "gated CLI launcher exited before the parent-death handshake"
                )
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                if request_deadline <= startup_deadline:
                    raise TimeoutError(
                        "request deadline expired during the parent-death handshake"
                    )
                raise OfflineCliOwnerError("gated CLI parent-death handshake timed out")
            try:
                readable, _, _ = select.select(
                    (ready_fd,),
                    (),
                    (),
                    min(self._poll_interval_seconds, remaining),
                )
            except InterruptedError:
                continue
            if not readable:
                continue
            marker = os.read(ready_fd, 2)
            if marker != b"1":
                raise OfflineCliOwnerError(
                    "gated CLI parent-death handshake marker is invalid"
                )
            kind, info = _probe_process(process.pid)
            if (
                kind != _ProbeKind.RUNNING
                or info is None
                or info.state == "Z"
                or info.pgrp != process.pid
                or info.session != process.pid
            ):
                raise OfflineCliOwnerError(
                    "gated CLI identity is invalid after the parent-death handshake"
                )
            return info

    def _terminate_unidentified_launcher(self, process: subprocess.Popen) -> None:
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=self._termination_grace_seconds)
            except subprocess.TimeoutExpired:
                process.kill()
        try:
            process.wait(timeout=self._kill_grace_seconds)
        except subprocess.TimeoutExpired as error:
            raise OfflineCliRecoveryError("gated CLI launcher was not reaped") from error

    def _launch_locked(
        self,
        *,
        record: Dict[str, object],
        argv: Sequence[str],
        monotonic_deadline: float,
        resolved_input: _ResolvedInput,
        resolved_output: _ResolvedOutput,
    ) -> _Operation:
        output_option = (
            "--output-parent"
            if record["command_kind"] == OfflineCliCommandKind.MAP.value
            else "--output"
        )
        if (
            len(argv) < 5
            or argv[1] not in {"--bag", "--input"}
            or argv[3] != output_option
        ):
            raise OfflineCliOwnerError("internal typed CLI argument layout is invalid")

        ready_read = -1
        ready_write = -1
        gate_read = -1
        gate_write = -1
        input_alias_name: Optional[str] = None
        process: Optional[subprocess.Popen] = None
        stdout_capture: Optional[_BoundedCapture] = None
        stderr_capture: Optional[_BoundedCapture] = None
        operation: Optional[_Operation] = None
        journaled = False
        cleanup_error: Optional[BaseException] = None
        process_cleanup_proven = False
        try:
            store = self._require_store()
            inherited_fds = (
                self._binary_fd,
                store.input_alias_dir_fd(),
                resolved_input.file_fd,
                resolved_output.parent_fd,
            )
            if any(descriptor <= 2 for descriptor in inherited_fds):
                raise OfflineCliValidationError(
                    "offline CLI launch requires reserved standard descriptors"
                )
            input_alias_name = store.create_input_alias(
                resolved_input,
                str(record["operation_id"]),
            )
            stable_input = "/proc/self/fd/%d/%s" % (
                store.input_alias_dir_fd(),
                input_alias_name,
            )
            process_argv = [
                argv[0],
                argv[1],
                stable_input,
                argv[3],
                "."
                if record["command_kind"] == OfflineCliCommandKind.MAP.value
                else resolved_output.path.name,
            ]
            process_argv.extend(argv[5:])
            gate_read, gate_write = os.pipe2(os.O_CLOEXEC)
            ready_read, ready_write = os.pipe2(os.O_CLOEXEC)
            if any(
                descriptor <= 2
                for descriptor in (ready_read, ready_write, gate_read, gate_write)
            ):
                raise OfflineCliValidationError(
                    "offline CLI handshake conflicts with a standard descriptor"
                )
            from core.settings import component_logging_environment

            process = subprocess.Popen(
                [
                    sys.executable,
                    "-c",
                    _LAUNCHER_CODE,
                    str(self._owner_pid),
                    str(ready_write),
                    str(gate_read),
                    str(self._binary_fd),
                    str(store.input_alias_dir_fd()),
                    str(resolved_input.file_fd),
                    str(resolved_output.parent_fd),
                    str(self._binary_path),
                ]
                + process_argv,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=component_logging_environment(),
                cwd=os.sep,
                close_fds=True,
                pass_fds=(
                    ready_write,
                    gate_read,
                    self._binary_fd,
                    store.input_alias_dir_fd(),
                    resolved_input.file_fd,
                    resolved_output.parent_fd,
                ),
                start_new_session=True,
            )
            os.close(ready_write)
            ready_write = -1
            os.close(gate_read)
            gate_read = -1
            info = self._await_launcher_identity(
                process,
                ready_read,
                monotonic_deadline,
            )
            os.close(ready_read)
            ready_read = -1
            if process.stdout is None or process.stderr is None:
                raise OfflineCliOwnerError("CLI launcher pipes are unavailable")
            stdout_capture = _BoundedCapture(
                process.stdout,
                self._max_log_bytes,
                "offline-cli-stdout-%s" % record["operation_id"],
            )
            stderr_capture = _BoundedCapture(
                process.stderr,
                self._max_log_bytes,
                "offline-cli-stderr-%s" % record["operation_id"],
            )
            stdout_capture.start()
            stderr_capture.start()
            operation = _Operation(
                request_id=str(record["request_id"]),
                record=record,
                input_alias_name=input_alias_name,
                resolved_input=resolved_input,
                resolved_output=resolved_output,
                process=process,
                process_start_token=info.start_token,
                pgid=info.pgrp,
                sid=info.session,
                monotonic_deadline=monotonic_deadline,
                stdout_capture=stdout_capture,
                stderr_capture=stderr_capture,
            )
            record.update(
                {
                    "state": OfflineCliState.STARTING.value,
                    "pid": process.pid,
                    "process_start_token": info.start_token,
                    "pgid": info.pgrp,
                    "sid": info.session,
                    "message": "gated CLI launcher identity is durable",
                }
            )
            self._write_record_locked(record)
            self._require_store().write_json(
                ACTIVE_JOURNAL_NAME,
                {
                    "schema_version": ACTIVE_SCHEMA_VERSION,
                    "owner_pid": self._owner_pid,
                    "owner_start_token": self._owner_start_token,
                    "request_id": record["request_id"],
                    "operation_id": record["operation_id"],
                    "pid": process.pid,
                    "process_start_token": info.start_token,
                    "pgid": info.pgrp,
                    "sid": info.session,
                },
            )
            journaled = True
            self._active = operation

            resolved_input.verify_current_identity()
            resolved_output.verify_parent_identity()
            if self._input_root is None or self._output_root is None:
                raise OfflineCliClosedError("configured roots are unavailable")
            self._input_root.verify_current_identity()
            self._output_root.verify_current_identity()
            self._verify_binary_identity()
            if time.monotonic() >= monotonic_deadline:
                raise TimeoutError("request deadline expired before CLI release")
            if os.write(gate_write, b"1") != 1:
                raise OfflineCliOwnerError("gated CLI launcher release was incomplete")
            os.close(gate_write)
            gate_write = -1

            record.update(
                {
                    "state": OfflineCliState.RUNNING.value,
                    "started_at_unix_ns": time.time_ns(),
                    "message": "offline CLI process is running",
                }
            )
            self._write_record_locked(record)
            operation.monitor = threading.Thread(
                target=self._monitor_operation,
                args=(operation,),
                name="offline-cli-owner-%s" % record["operation_id"],
                daemon=False,
            )
            operation.monitor.start()
            return operation
        except BaseException as error:
            if ready_read >= 0:
                os.close(ready_read)
            if ready_write >= 0:
                os.close(ready_write)
            if gate_read >= 0:
                os.close(gate_read)
            if gate_write >= 0:
                os.close(gate_write)
            try:
                if operation is not None:
                    self._terminate_identity(
                        pid=operation.process.pid,
                        process_start_token=operation.process_start_token,
                        pgid=operation.pgid,
                        sid=operation.sid,
                        process=operation.process,
                    )
                elif process is not None:
                    self._terminate_unidentified_launcher(process)
                process_cleanup_proven = True
            except BaseException as terminate_error:
                cleanup_error = terminate_error
            for capture in (stdout_capture, stderr_capture):
                if capture is None:
                    continue
                try:
                    capture.finish(self._kill_grace_seconds)
                except BaseException as capture_error:
                    capture.abort()
                    if cleanup_error is None:
                        cleanup_error = capture_error
            if stdout_capture is not None:
                stdout, truncated, _ = stdout_capture.snapshot()
                record["stdout"] = stdout
                record["stdout_truncated"] = truncated
            if stderr_capture is not None:
                stderr, truncated, _ = stderr_capture.snapshot()
                record["stderr"] = stderr
                record["stderr_truncated"] = truncated
            if process is not None:
                record["exit_code"] = process.returncode
            if operation is not None and process_cleanup_proven:
                try:
                    self._cleanup_map_staging(
                        operation.record,
                        operation.process.pid,
                        operation.process_start_token,
                        operation.resolved_output.parent_fd,
                    )
                except BaseException as staging_error:
                    if cleanup_error is None:
                        cleanup_error = staging_error
            if input_alias_name is not None and process_cleanup_proven:
                try:
                    self._require_store().remove_input_alias(input_alias_name)
                except BaseException as alias_error:
                    if cleanup_error is None:
                        cleanup_error = alias_error
            if self._active is operation:
                self._active = None
            if journaled and cleanup_error is None:
                try:
                    self._require_store().remove(ACTIVE_JOURNAL_NAME)
                except BaseException as journal_error:
                    cleanup_error = journal_error
            self._condition.notify_all()
            if cleanup_error is not None:
                self._poison_locked("CLI launch cleanup is unproven: %s" % cleanup_error)
                raise OfflineCliRecoveryError("CLI launch failed and cleanup is unproven") from error
            if isinstance(error, OfflineCliRecoveryError):
                self._poison_locked(error)
            raise

    def _finish_unstarted_locked(
        self,
        record: Dict[str, object],
        error: BaseException,
    ) -> OfflineCliStatus:
        state = OfflineCliState.TIMED_OUT if isinstance(error, TimeoutError) else OfflineCliState.FAILED
        record.update(
            {
                "state": state.value,
                "terminal": True,
                "completed_at_unix_ns": time.time_ns(),
                "error_code": ("startup_deadline" if state == OfflineCliState.TIMED_OUT else "startup_failed"),
                "message": _bounded_text(error),
            }
        )
        self._write_record_locked(record)
        return OfflineCliStatus.from_record(record)

    def start(self, request: OfflineCliRequest) -> OfflineCliStatus:
        if type(request) is not OfflineCliRequest:
            raise OfflineCliValidationError("request must be an OfflineCliRequest")
        command, argv = _command_contract(request.command)
        if not isinstance(request.request_id, str):
            raise OfflineCliValidationError("request_id must be a string")
        request_id = request.request_id.strip()
        if not _REQUEST_ID_RE.fullmatch(request_id):
            raise OfflineCliValidationError("request_id is not canonical")

        with self._condition:
            self._ensure_startable_locked()
            if self._input_root is None or self._output_root is None:
                raise OfflineCliClosedError("configured roots are unavailable")
            try:
                existing = self._read_record_locked(request_id)
            except Exception as error:
                self._poison_locked("durable request lookup failed: %s" % error)
                raise
            if existing is not None and existing["schema_version"] == LEGACY_REQUEST_SCHEMA_VERSION:
                raise OfflineCliRequestConflict("legacy request is read-only; query its recorded status instead of resubmitting")
            resolved_input = self._input_root.resolve_input(str(command["input_path"]))
            try:
                (
                    dataset_size,
                    dataset_format,
                    dataset_device,
                    dataset_inode,
                    dataset_mtime_ns,
                    dataset_ctime_ns,
                ) = resolved_input.observe_input()
                payload, argv, timeout_seconds = _request_contract(
                    request,
                    release_generation=self._release_generation,
                    release_manifest_sha256=self._release_manifest_sha256,
                    offline_cli_artifact_sha256=self._artifact_sha256,
                    offline_cli_artifact_size=self._artifact_size,
                    dataset_size=dataset_size,
                    dataset_format=dataset_format,
                    dataset_device=dataset_device,
                    dataset_inode=dataset_inode,
                    dataset_mtime_ns=dataset_mtime_ns,
                    dataset_ctime_ns=dataset_ctime_ns,
                )
            except BaseException:
                resolved_input.close()
                raise
            argv = [
                value.format(
                    release_generation=self._release_generation,
                    release_manifest_sha256=self._release_manifest_sha256,
                    artifact_sha256=self._artifact_sha256,
                )
                for value in argv
            ]
            if existing is not None:
                resolved_input.close()
                if any(existing[key] != value for key, value in payload.items()):
                    raise OfflineCliRequestConflict("request_id is permanently bound to another typed request")
                if not bool(existing["terminal"]) and (
                    self._active is None or self._active.request_id != request_id
                ):
                    self._poison_locked(
                        "durable request is nonterminal without its active operation"
                    )
                    raise OfflineCliRecoveryError(self._poisoned)
                return OfflineCliStatus.from_record(existing)
            if self._active is not None:
                resolved_input.close()
                raise OfflineCliBusyError("another offline CLI request is active")
            try:
                self._require_store().require_request_capacity()
            except BaseException:
                resolved_input.close()
                raise

            submitted_at = time.time_ns()
            monotonic_deadline = time.monotonic() + timeout_seconds
            record: Dict[str, object] = {
                "schema_version": REQUEST_SCHEMA_VERSION,
                "request_id": request_id,
                "operation_id": uuid.uuid4().hex,
                "command_kind": str(command["kind"]),
                "command": command,
                "state": OfflineCliState.ACCEPTED.value,
                "terminal": False,
                "dataset_size": int(payload["dataset_size"]),
                "dataset_format": str(payload["dataset_format"]),
                "dataset_device": int(payload["dataset_device"]),
                "dataset_inode": int(payload["dataset_inode"]),
                "dataset_mtime_ns": int(payload["dataset_mtime_ns"]),
                "dataset_ctime_ns": int(payload["dataset_ctime_ns"]),
                "timeout_seconds": timeout_seconds,
                "submitted_at_unix_ns": submitted_at,
                "deadline_at_unix_ns": submitted_at + int(timeout_seconds * 1_000_000_000),
                "started_at_unix_ns": None,
                "completed_at_unix_ns": None,
                "pid": None,
                "process_start_token": None,
                "pgid": None,
                "sid": None,
                "exit_code": None,
                "stdout": "",
                "stderr": "",
                "stdout_truncated": False,
                "stderr_truncated": False,
                "output_relative_path": str(command["output_path"]),
                "output_size": None,
                "receipt": None,
                "release_generation": str(payload["release_generation"]),
                "release_manifest_sha256": str(payload["release_manifest_sha256"]),
                "offline_cli_artifact_sha256": str(payload["offline_cli_artifact_sha256"]),
                "offline_cli_artifact_size": int(payload["offline_cli_artifact_size"]),
                "error_code": None,
                "message": "offline CLI request accepted",
            }
            try:
                self._write_record_locked(record)
            except BaseException:
                resolved_input.close()
                raise

            resolved_output: Optional[_ResolvedOutput] = None
            launched = False
            try:
                resolved_output = self._output_root.resolve_output(str(command["output_path"]))
                self._verify_binary_identity()
                operation = self._launch_locked(
                    record=record,
                    argv=argv,
                    monotonic_deadline=monotonic_deadline,
                    resolved_input=resolved_input,
                    resolved_output=resolved_output,
                )
                launched = True
                return OfflineCliStatus.from_record(operation.record)
            except BaseException as error:
                if isinstance(error, (KeyboardInterrupt, SystemExit)):
                    raise
                if isinstance(error, OfflineCliRecoveryError) or self._poisoned is not None:
                    raise
                return self._finish_unstarted_locked(record, error)
            finally:
                if not launched:
                    if resolved_input is not None:
                        resolved_input.close()
                    if resolved_output is not None:
                        resolved_output.close()

    def _request_stop_locked(
        self,
        operation: _Operation,
        terminal_state: OfflineCliState,
        error_code: str,
        message: str,
    ) -> None:
        if terminal_state not in {
            OfflineCliState.CANCELLED,
            OfflineCliState.TIMED_OUT,
            OfflineCliState.SHUTDOWN,
            OfflineCliState.FAILED,
        }:
            raise OfflineCliOwnerError("invalid offline CLI stop intent")
        if operation.desired_terminal is not None:
            return
        operation.desired_terminal = terminal_state
        operation.desired_error_code = error_code
        operation.desired_message = _bounded_text(message)
        operation.record.update(
            {
                "state": OfflineCliState.CANCELLING.value,
                "message": operation.desired_message,
            }
        )
        operation.wake.set()
        self._write_record_locked(operation.record)

    def _finalize_operation(
        self,
        operation: _Operation,
        *,
        state: OfflineCliState,
        error_code: Optional[str],
        message: str,
        cleanup_proven: bool,
        output_size: Optional[int],
        receipt: Optional[Mapping[str, object]],
    ) -> None:
        stdout, stdout_truncated, stdout_error = operation.stdout_capture.snapshot()
        stderr, stderr_truncated, stderr_error = operation.stderr_capture.snapshot()
        if stdout_error is not None or stderr_error is not None:
            state = OfflineCliState.FAILED
            error_code = "log_capture_failed"
            message = "bounded log capture failed: %s" % (stdout_error if stdout_error is not None else stderr_error)
            output_size = None
            receipt = None
        if not cleanup_proven:
            state = OfflineCliState.FAILED
            error_code = "cleanup_unproven"
            output_size = None
            receipt = None

        operation.record.update(
            {
                "state": state.value,
                "terminal": True,
                "completed_at_unix_ns": time.time_ns(),
                "exit_code": operation.process.returncode,
                "stdout": stdout,
                "stderr": stderr,
                "stdout_truncated": stdout_truncated,
                "stderr_truncated": stderr_truncated,
                "output_size": output_size,
                "receipt": (None if receipt is None else dict(receipt)),
                "error_code": error_code,
                "message": _bounded_text(message),
            }
        )

        with self._condition:
            durable = False
            journal_removed = False
            try:
                self._write_record_locked(operation.record)
                durable = True
                if cleanup_proven:
                    self._require_store().remove(ACTIVE_JOURNAL_NAME)
                    journal_removed = True
            except Exception as error:
                self._poison_locked("terminal state cleanup failed: %s" % error)
            if not cleanup_proven:
                self._poison_locked(message)
            if not durable or (cleanup_proven and not journal_removed):
                self._volatile_records[operation.request_id] = dict(operation.record)
            if self._active is operation:
                self._active = None
            self._condition.notify_all()

    def _monitor_operation(self, operation: _Operation) -> None:
        state = OfflineCliState.FAILED
        error_code: Optional[str] = "process_failed"
        message = "offline CLI process failed"
        cleanup_proven = True
        process_cleanup_proven = False
        output_size: Optional[int] = None
        receipt: Optional[Mapping[str, object]] = None
        try:
            while True:
                exit_code = operation.process.poll()
                members = self._verified_group_members(
                    pid=operation.process.pid,
                    process_start_token=operation.process_start_token,
                    pgid=operation.pgid,
                    sid=operation.sid,
                )
                if exit_code is not None:
                    if members:
                        with self._condition:
                            self._request_stop_locked(
                                operation,
                                OfflineCliState.FAILED,
                                "descendant_process_survived",
                                "direct CLI child exited while its process group remained",
                            )
                        self._terminate_identity(
                            pid=operation.process.pid,
                            process_start_token=operation.process_start_token,
                            pgid=operation.pgid,
                            sid=operation.sid,
                            process=operation.process,
                        )
                    break

                with self._condition:
                    if operation.desired_terminal is None and time.monotonic() >= operation.monotonic_deadline:
                        self._request_stop_locked(
                            operation,
                            OfflineCliState.TIMED_OUT,
                            "deadline_exceeded",
                            "offline CLI request exceeded its deadline",
                        )
                    desired = operation.desired_terminal
                if desired is not None:
                    self._terminate_identity(
                        pid=operation.process.pid,
                        process_start_token=operation.process_start_token,
                        pgid=operation.pgid,
                        sid=operation.sid,
                        process=operation.process,
                    )
                    break
                operation.wake.wait(self._poll_interval_seconds)
                operation.wake.clear()

            process_cleanup_proven = True

            with self._condition:
                desired = operation.desired_terminal
                desired_code = operation.desired_error_code
                desired_message = operation.desired_message
            if desired is not None:
                state = desired
                error_code = desired_code
                message = desired_message or "offline CLI request was stopped"
            elif operation.process.returncode != 0:
                state = OfflineCliState.FAILED
                error_code = "nonzero_exit"
                message = "offline CLI exited with status %s" % operation.process.returncode
            else:
                operation.resolved_input.verify_input_observation(
                    int(operation.record["dataset_size"]),
                    str(operation.record["dataset_format"]),
                    int(operation.record["dataset_device"]),
                    int(operation.record["dataset_inode"]),
                    int(operation.record["dataset_mtime_ns"]),
                    int(operation.record["dataset_ctime_ns"]),
                )
                if self._input_root is None or self._output_root is None:
                    raise OfflineCliClosedError("configured roots are unavailable")
                self._input_root.verify_current_identity()
                self._output_root.verify_current_identity()
                self._verify_binary_identity()
                if operation.record["command_kind"] == OfflineCliCommandKind.MAP.value:
                    output_size, receipt = _inspect_map_bundle(
                        operation.resolved_output,
                        operation.record["command"],
                        operation.request_id,
                        int(operation.record["dataset_size"]),
                        str(operation.record["dataset_format"]),
                        int(operation.record["dataset_device"]),
                        int(operation.record["dataset_inode"]),
                        int(operation.record["dataset_mtime_ns"]),
                        int(operation.record["dataset_ctime_ns"]),
                        self._release_generation,
                        self._release_manifest_sha256,
                        self._artifact_sha256,
                        self._artifact_size,
                    )
                else:
                    output_size, receipt = _inspect_output(
                        operation.resolved_output,
                        str(operation.record["output_relative_path"]),
                    )
                state = OfflineCliState.SUCCEEDED
                error_code = None
                message = "offline CLI output was durably published and verified"
        except BaseException as error:
            state = OfflineCliState.FAILED
            error_code = "owner_monitor_failed"
            message = "offline CLI supervision failed: %s" % error
            output_size = None
            try:
                self._terminate_identity(
                    pid=operation.process.pid,
                    process_start_token=operation.process_start_token,
                    pgid=operation.pgid,
                    sid=operation.sid,
                    process=operation.process,
                )
                process_cleanup_proven = True
            except BaseException as cleanup_error:
                cleanup_proven = False
                message = "%s; cleanup failed: %s" % (message, cleanup_error)

        for capture in (operation.stdout_capture, operation.stderr_capture):
            try:
                capture.finish(self._kill_grace_seconds)
            except BaseException as capture_error:
                capture.abort()
                cleanup_proven = False
                state = OfflineCliState.FAILED
                error_code = "log_drain_failed"
                message = "bounded log drain failed: %s" % capture_error
                output_size = None
        if process_cleanup_proven:
            try:
                self._cleanup_map_staging(
                    operation.record,
                    operation.process.pid,
                    operation.process_start_token,
                    operation.resolved_output.parent_fd,
                )
            except BaseException as staging_error:
                cleanup_proven = False
                message = "%s; MAP staging cleanup failed: %s" % (
                    message,
                    staging_error,
                )
            try:
                self._require_store().remove_input_alias(operation.input_alias_name)
            except BaseException as alias_error:
                cleanup_proven = False
                message = "%s; input alias cleanup failed: %s" % (
                    message,
                    alias_error,
                )
        try:
            operation.close_resolved_paths()
        except OSError as close_error:
            cleanup_proven = False
            message = "%s; stable path descriptor close failed: %s" % (
                message,
                close_error,
            )
        self._finalize_operation(
            operation,
            state=state,
            error_code=error_code,
            message=message,
            cleanup_proven=cleanup_proven,
            output_size=output_size,
            receipt=receipt,
        )

    @staticmethod
    def _validate_request_id(request_id: object) -> str:
        if not isinstance(request_id, str) or not _REQUEST_ID_RE.fullmatch(request_id):
            raise OfflineCliValidationError("request_id is not canonical")
        return request_id

    def get_status(self, request_id: str) -> OfflineCliStatus:
        normalized = self._validate_request_id(request_id)
        with self._condition:
            self._assert_owner_process_locked()
            if self._resources_closed:
                raise OfflineCliClosedError("offline CLI owner is closed")
            try:
                record = self._read_record_locked(normalized)
            except Exception as error:
                self._poison_locked("durable request lookup failed: %s" % error)
                raise
            if record is None:
                raise OfflineCliNotFoundError(normalized)
            return OfflineCliStatus.from_record(record)

    def wait(
        self,
        request_id: str,
        timeout: Optional[float] = None,
    ) -> OfflineCliStatus:
        normalized = self._validate_request_id(request_id)
        wait_timeout: Optional[float] = None
        if timeout is not None:
            wait_timeout = _require_finite(
                timeout,
                "wait timeout",
                minimum=0.0,
                maximum=MAX_TIMEOUT_SECONDS,
            )
        deadline = None if wait_timeout is None else time.monotonic() + wait_timeout
        with self._condition:
            while True:
                self._assert_owner_process_locked()
                if self._resources_closed:
                    raise OfflineCliClosedError("offline CLI owner is closed")
                try:
                    record = self._read_record_locked(normalized)
                except Exception as error:
                    self._poison_locked("durable request lookup failed: %s" % error)
                    raise
                if record is None:
                    raise OfflineCliNotFoundError(normalized)
                status = OfflineCliStatus.from_record(record)
                if status.terminal:
                    return status
                if self._active is None or self._active.request_id != normalized:
                    raise OfflineCliRecoveryError("durable request is nonterminal without its active owner")
                if deadline is None:
                    self._condition.wait()
                    continue
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise TimeoutError("waiting for offline CLI request timed out")
                self._condition.wait(remaining)

    def execute(self, request: OfflineCliRequest) -> OfflineCliStatus:
        status = self.start(request)
        if status.terminal:
            return status
        return self.wait(status.request_id)

    async def execute_async(self, request: OfflineCliRequest) -> OfflineCliStatus:
        loop = asyncio.get_running_loop()
        start_task = loop.run_in_executor(None, self.start, request)
        try:
            status = await asyncio.shield(start_task)
        except asyncio.CancelledError:
            status = await asyncio.shield(start_task)
            if not status.terminal:
                cancel_task = loop.run_in_executor(None, self.cancel, status.request_id)
                await asyncio.shield(cancel_task)
            raise
        if status.terminal:
            return status
        try:
            wait_task = loop.run_in_executor(None, self.wait, status.request_id)
            return await asyncio.shield(wait_task)
        except asyncio.CancelledError:
            cancel_task = loop.run_in_executor(None, self.cancel, status.request_id)
            await asyncio.shield(cancel_task)
            await asyncio.shield(wait_task)
            raise

    def cancel(self, request_id: str) -> OfflineCliStatus:
        normalized = self._validate_request_id(request_id)
        with self._condition:
            self._assert_owner_process_locked()
            if self._resources_closed:
                raise OfflineCliClosedError("offline CLI owner is closed")
            try:
                record = self._read_record_locked(normalized)
            except Exception as error:
                self._poison_locked("durable request lookup failed: %s" % error)
                raise
            if record is None:
                raise OfflineCliNotFoundError(normalized)
            status = OfflineCliStatus.from_record(record)
            if status.terminal:
                return status
            operation = self._active
            if operation is None or operation.request_id != normalized:
                raise OfflineCliRecoveryError("durable request is nonterminal without its active owner")
            try:
                self._request_stop_locked(
                    operation,
                    OfflineCliState.CANCELLED,
                    "cancelled",
                    "offline CLI request was cancelled",
                )
            finally:
                operation.wake.set()
        return self.wait(normalized)

    def shutdown(self) -> None:
        operation: Optional[_Operation]
        stop_error: Optional[BaseException] = None
        with self._condition:
            self._assert_owner_process_locked()
            if self._resources_closed:
                return
            self._closing = True
            operation = self._active
            if operation is not None:
                try:
                    self._request_stop_locked(
                        operation,
                        OfflineCliState.SHUTDOWN,
                        "owner_shutdown",
                        "offline CLI owner is shutting down",
                    )
                except BaseException as error:
                    stop_error = error
                finally:
                    operation.wake.set()

        if operation is not None and operation.monitor is not None:
            join_timeout = self._termination_grace_seconds + 3 * self._kill_grace_seconds + 2.0
            operation.monitor.join(join_timeout)
            if operation.monitor.is_alive():
                with self._condition:
                    self._poison_locked("offline CLI monitor did not stop during shutdown")
                raise OfflineCliRecoveryError("offline CLI monitor did not stop during shutdown")

        with self._condition:
            if self._active is not None:
                self._poison_locked("offline CLI operation did not settle during shutdown")
                raise OfflineCliRecoveryError("offline CLI operation did not settle during shutdown")
            poison = self._poisoned
            self._close_resources_unlocked()
        if stop_error is not None:
            raise OfflineCliRecoveryError("shutdown state transition failed: %s" % stop_error) from stop_error
        if poison is not None:
            raise OfflineCliRecoveryError(poison)

    def __enter__(self) -> "OfflineMappingCliOwner":
        with self._condition:
            self._ensure_startable_locked()
        return self

    def __exit__(
        self,
        exc_type: Optional[type],
        exc_value: Optional[BaseException],
        traceback: Any,
    ) -> None:
        self.shutdown()
