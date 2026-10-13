"""Crash-consistent storage for geometry mapping-backend checkpoints."""

from __future__ import annotations

from services.geometry.checkpoint_payload import FrozenPayload, payload_digest, payload_chunks, MAX_PLANAR_ARCHIVE_BYTES
from services.geometry.planar_archive import read_planar_structure, assign_legacy_planar_archive, MAX_GRID
from services.geometry.fine_map_archive import MAX_FINE_ARCHIVE_BYTES, validate_fine_archive
from services.geometry.dense_cloud import MAX_DENSE_ARCHIVE_BYTES, validate_dense_archive, read_dense_metadata

import fcntl
import ctypes
import errno
import hashlib
import json
import math
import os
import re
import stat
import struct
import time
import uuid
import numpy as np
from dataclasses import dataclass, field, replace
from types import MappingProxyType
from pathlib import Path
from typing import (
    Callable,
    Dict,
    FrozenSet,
    List,
    Mapping,
    Optional,
    Sequence,
    Set,
    Tuple,
    Union,
)


MAPPING_BACKEND_CHECKPOINT_SCHEMA_VERSION = 1
ASSIGNED_CHECKPOINT_SCHEMA_VERSION = 2
MAX_MAPPING_BACKEND_CHECKPOINT_BYTES = 64 * 1024 * 1024
DEFAULT_ORPHAN_GRACE_SECONDS = 24 * 60 * 60
DEFAULT_STORE_LOCK_TIMEOUT_SECONDS = 30.0
SUPPORTED_MAPPING_BACKEND_KINDS = frozenset({"raytracing", "voxel", "slam_toolbox"})
CLOUD_MAP_PAYLOAD_SCHEMA_VERSION = 1
MAX_CLOUD_MAP_VOXEL_SIZE = 1_048_576.0

PUBLISH_FAULT_PHASES = (
    "payload_temp_written",
    "payload_file_fsynced",
    "object_installed",
    "objects_directory_fsynced",
    "manifest_temp_written",
    "manifest_file_fsynced",
    "manifest_installed",
    "manifests_directory_fsynced",
    "selector_temp_written",
    "selector_file_fsynced",
    "selector_installed",
    "backend_directory_fsynced",
)

_SHA256_RE = re.compile(r"[0-9a-f]{64}\Z")
_TEMP_PATTERNS = (
    re.compile(r"\.tmp-checkpoint-[0-9a-f]{32}\.bin\Z"),
    re.compile(r"\.tmp-manifest-[0-9a-f]{32}\.json\Z"),
    re.compile(r"\.tmp-checkpoint-v[12]-[0-9a-f]{32}\.json\Z"),
)
_OBJECT_NAME_RE = re.compile(r"(?:sha256-[0-9a-f]{64}|id-[0-9a-f]{32}(?:-(?:backend|occupancy|cloud))?)\.bin\Z")
_MANIFEST_NAME_RE = re.compile(r"(?:sha256-[0-9a-f]{64}|id-[0-9a-f]{32})\.json\Z")
_MAX_JSON_BYTES = 1024 * 1024
_U32_MAX = (1 << 32) - 1
_U64_MAX = (1 << 64) - 1


class MappingBackendCheckpointStoreError(Exception):
    """Base class for checkpoint persistence failures."""


class MappingBackendCheckpointValidationError(
    MappingBackendCheckpointStoreError, ValueError
):
    """A checkpoint or persisted checkpoint artifact is invalid."""


class MappingBackendCheckpointNotFoundError(
    MappingBackendCheckpointStoreError, FileNotFoundError
):
    """The stable selector for a requested checkpoint does not exist."""


def _freeze(value):
    if isinstance(value, Mapping):
        return MappingProxyType({key: _freeze(item) for key, item in value.items()})
    if isinstance(value, (list, tuple)):
        return tuple(_freeze(item) for item in value)
    return value


@dataclass(frozen=True)
class CheckpointMemberIdentity:
    checkpoint_id: str
    role: str
    occupancy_receipt: Mapping[str, object]
    cloud_receipt: Optional[Mapping[str, object]]
    frame_id: str
    frame_generation: int = 0
    producer_generation: int = 0

    def __post_init__(self):
        object.__setattr__(self, "occupancy_receipt", _freeze(self.occupancy_receipt))
        object.__setattr__(self, "cloud_receipt", _freeze(self.cloud_receipt))


@dataclass(frozen=True)
class CheckpointArtifactReference:
    id: str
    size: int
    identity_schema_version: int
    member_identity: Optional[CheckpointMemberIdentity] = None
    legacy_alias: bytes = b""


@dataclass(frozen=True)
class MappingBackendCheckpoint:
    """Checkpoint data shaped like the GeometryRuntime protobuf message.

    Receipt mappings use the protobuf field names. ``commit_id.boot_nonce`` is
    exactly 16 bytes at this API boundary and is stored as lowercase hex in the
    JSON manifest.
    """

    schema_version: int
    backend_kind: str
    source_session_id: str
    source_session_generation: int
    content_sha256: bytes
    payload_size: int
    created_at_ns: int
    payload: Union[bytes, FrozenPayload]
    occupancy_receipt: Mapping[str, object]
    cloud_receipt: Optional[Mapping[str, object]]
    fine_archive: Optional[Union[bytes, FrozenPayload]] = None
    dense_archive: Optional[Union[bytes, FrozenPayload]] = None
    checkpoint_id: str = ""
    target_kind: str = ""
    backend_payload_schema_version: int = 1
    canonical_occupancy_data: Optional[Union[bytes, FrozenPayload]] = None
    canonical_cloud_xyz: Optional[Union[bytes, FrozenPayload]] = None
    fine_reference: Optional[CheckpointArtifactReference] = None
    dense_reference: Optional[CheckpointArtifactReference] = None
    cloud_identity: Optional[CheckpointMemberIdentity] = None
    backend_identity: Optional[CheckpointMemberIdentity] = None
    occupancy_identity: Optional[CheckpointMemberIdentity] = None
    graph_reference: Optional[CheckpointArtifactReference] = None
    _admission: object = field(default=None, init=False, repr=False, compare=False)

    def __post_init__(self):
        object.__setattr__(self, "occupancy_receipt", _freeze(self.occupancy_receipt))
        object.__setattr__(self, "cloud_receipt", _freeze(self.cloud_receipt))


@dataclass(frozen=True)
class StoredMappingBackendCheckpoint:
    checkpoint: MappingBackendCheckpoint
    occupancy_asset: Mapping[str, object]
    cloud_asset: Optional[Mapping[str, object]]
    selector_path: Path
    manifest_sha256: str
    manifest_path: Path
    object_path: Path
    identity: Optional[Tuple[str, str]] = None
    legacy_alias: Optional[str] = None
    member_paths: Mapping[str, Path] = field(default_factory=dict)

    def __post_init__(self):
        object.__setattr__(self, "occupancy_asset", _freeze(self.occupancy_asset))
        object.__setattr__(self, "cloud_asset", _freeze(self.cloud_asset))
        object.__setattr__(self, "member_paths", _freeze(self.member_paths))


@dataclass(frozen=True)
class CheckpointCleanupResult:
    removed_temp_files: Tuple[Path, ...]
    removed_manifests: Tuple[Path, ...]
    removed_objects: Tuple[Path, ...]
    issues: Tuple[str, ...] = ()


FaultInjector = Callable[[str], None]


class MappingBackendCheckpointStore:
    """Persist one current checkpoint per backend/envelope generation."""

    def __init__(
        self,
        version_dir: Union[Path, str],
        *,
        max_payload_bytes: int = MAX_MAPPING_BACKEND_CHECKPOINT_BYTES,
        orphan_grace_seconds: float = DEFAULT_ORPHAN_GRACE_SECONDS,
        lock_timeout_seconds: float = DEFAULT_STORE_LOCK_TIMEOUT_SECONDS,
        clock_ns: Callable[[], int] = time.time_ns,
    ) -> None:
        self._version_dir = Path(version_dir).expanduser().absolute()
        self._max_payload_bytes = _require_integer(
            max_payload_bytes,
            "max_payload_bytes",
            minimum=1,
            maximum=MAX_MAPPING_BACKEND_CHECKPOINT_BYTES,
        )
        if isinstance(orphan_grace_seconds, bool) or not isinstance(
            orphan_grace_seconds, (int, float)
        ):
            raise MappingBackendCheckpointValidationError(
                "orphan_grace_seconds must be a non-negative number"
            )
        normalized_grace_seconds = float(orphan_grace_seconds)
        maximum_grace_seconds = _U64_MAX / 1_000_000_000
        if (
            not math.isfinite(normalized_grace_seconds)
            or normalized_grace_seconds < 0
            or normalized_grace_seconds > maximum_grace_seconds
        ):
            raise MappingBackendCheckpointValidationError(
                "orphan_grace_seconds is outside the supported finite range"
            )
        self._orphan_grace_ns = int(normalized_grace_seconds * 1_000_000_000)
        self._lock_timeout_seconds = _require_positive_finite_duration(
            lock_timeout_seconds,
            "lock_timeout_seconds",
        )
        self._clock_ns = clock_ns

    def _payload_limit(self, backend_kind):
        if backend_kind == "slam_toolbox" and self._max_payload_bytes == MAX_MAPPING_BACKEND_CHECKPOINT_BYTES:
            return MAX_PLANAR_ARCHIVE_BYTES
        return self._max_payload_bytes

    @property
    def version_dir(self) -> Path:
        return self._version_dir

    @classmethod
    def validate_checkpoint_bundle(
        cls,
        checkpoint: MappingBackendCheckpoint,
        *,
        occupancy_asset: Mapping[str, object],
        cloud_asset: Optional[Mapping[str, object]],
        max_payload_bytes: int = MAX_MAPPING_BACKEND_CHECKPOINT_BYTES,
    ) -> Tuple[
        MappingBackendCheckpoint,
        Mapping[str, object],
        Mapping[str, object],
    ]:
        """Validate one in-memory export bundle without touching the filesystem."""

        if checkpoint.schema_version == 2 and checkpoint._admission is not None:
            old_occupancy, old_cloud = checkpoint._admission
            payload_limit = (MAX_PLANAR_ARCHIVE_BYTES if checkpoint.backend_kind == "slam_toolbox"
                and max_payload_bytes == MAX_MAPPING_BACKEND_CHECKPOINT_BYTES else max_payload_bytes)
            if occupancy_asset == old_occupancy and cloud_asset == old_cloud and checkpoint.payload_size <= payload_limit:
                return checkpoint, old_occupancy, old_cloud
        validator = cls(Path.cwd(), max_payload_bytes=max_payload_bytes)
        validated_checkpoint = validator._validate_checkpoint(checkpoint)
        validated_occupancy_asset = _validate_occupancy_asset(
            occupancy_asset,
            sha256_encoding="none" if checkpoint.schema_version == 2 else "bytes",
        )
        validated_cloud_asset = _validate_optional_cloud(cloud_asset, validated_checkpoint.backend_kind, sha256_encoding="none" if checkpoint.schema_version == 2 else "bytes")
        _validate_asset_bindings(
            validated_checkpoint,
            validated_occupancy_asset,
            validated_cloud_asset,
        )
        if checkpoint.schema_version == 2:
            _validate_canonical_members(validated_checkpoint, validated_occupancy_asset, validated_cloud_asset)
            validated_occupancy_asset = _freeze(validated_occupancy_asset)
            validated_cloud_asset = _freeze(validated_cloud_asset)
            object.__setattr__(validated_checkpoint, "_admission", (validated_occupancy_asset, validated_cloud_asset))
        return (
            validated_checkpoint,
            validated_occupancy_asset,
            validated_cloud_asset,
        )

    def publish(
        self,
        checkpoint: MappingBackendCheckpoint,
        *,
        occupancy_asset: Mapping[str, object],
        cloud_asset: Optional[Mapping[str, object]],
        fault_injector: Optional[FaultInjector] = None,
    ) -> StoredMappingBackendCheckpoint:
        """Validate and publish object, manifest, then stable selector."""

        if checkpoint.schema_version == 2:
            return self._publish_v2(checkpoint, occupancy_asset, cloud_asset, fault_injector)
        validated = self._validate_checkpoint(checkpoint, legacy_write=True)
        validated_occupancy_asset = _validate_occupancy_asset(
            occupancy_asset, sha256_encoding="bytes"
        )
        validated_cloud_asset = _validate_optional_cloud(cloud_asset, validated.backend_kind, sha256_encoding="bytes")
        _validate_asset_bindings(
            validated,
            validated_occupancy_asset,
            validated_cloud_asset,
        )
        access = self._ensure_layout(validated.backend_kind, exclusive=True)
        try:
            paths = access.paths
            current = self._load_optional(
                validated.backend_kind,
                validated.schema_version,
                access=access,
            )
            self._cleanup_paths(
                access,
                current=current,
                now_ns=self._validated_now_ns(),
            )

            payload_temp_name: Optional[str] = (
                f".tmp-checkpoint-{uuid.uuid4().hex}.bin"
            )
            payload_temp_identity: Optional[_FileIdentity] = None
            manifest_temp_name: Optional[str] = None
            manifest_temp_identity: Optional[_FileIdentity] = None
            selector_temp_name: Optional[str] = None
            selector_temp_identity: Optional[_FileIdentity] = None
            try:
                payload_temp_identity = self._write_temp_file(
                    access.objects_fd,
                    payload_temp_name,
                    paths.objects / payload_temp_name,
                    validated.payload,
                    written_phase="payload_temp_written",
                    fsynced_phase="payload_file_fsynced",
                    fault_injector=fault_injector,
                )
                reopened_payload = _read_regular_file_at(
                    access.objects_fd,
                    payload_temp_name,
                    paths.objects / payload_temp_name,
                    maximum_bytes=self._payload_limit(validated.backend_kind),
                    description="temporary checkpoint payload",
                )
                reopened_sha256 = payload_digest(reopened_payload)
                if reopened_sha256 != validated.content_sha256:
                    raise MappingBackendCheckpointValidationError(
                        "temporary checkpoint payload SHA-256 changed after fsync"
                    )
                if len(reopened_payload) != validated.payload_size:
                    raise MappingBackendCheckpointValidationError(
                        "temporary checkpoint payload size changed after fsync"
                    )

                object_sha256 = validated.content_sha256.hex()
                object_name = f"sha256-{object_sha256}.bin"
                object_path = paths.objects / object_name
                self._install_immutable(
                    access.objects_fd,
                    payload_temp_name,
                    object_name,
                    object_path,
                    validated.payload,
                    temp_identity=payload_temp_identity,
                    maximum_bytes=self._payload_limit(validated.backend_kind),
                    description="checkpoint object",
                )
                payload_temp_name = None
                _inject_fault(fault_injector, "object_installed")
                _fsync_directory_fd(access.objects_fd, paths.objects)
                _inject_fault(fault_injector, "objects_directory_fsynced")

                if validated.fine_archive is not None:
                    self._publish_pointcloud_archive(access, validated.fine_archive, fault_injector, MAX_FINE_ARCHIVE_BYTES)
                if validated.dense_archive is not None:
                    self._publish_pointcloud_archive(access, validated.dense_archive, fault_injector, MAX_DENSE_ARCHIVE_BYTES)

                manifest_document = self._manifest_document(
                    validated,
                    validated_occupancy_asset,
                    validated_cloud_asset,
                )
                manifest_bytes = _encode_json(manifest_document)
                manifest_temp_name = f".tmp-manifest-{uuid.uuid4().hex}.json"
                manifest_temp_identity = self._write_temp_file(
                    access.manifests_fd,
                    manifest_temp_name,
                    paths.manifests / manifest_temp_name,
                    manifest_bytes,
                    written_phase="manifest_temp_written",
                    fsynced_phase="manifest_file_fsynced",
                    fault_injector=fault_injector,
                )
                reopened_manifest = _read_regular_file_at(
                    access.manifests_fd,
                    manifest_temp_name,
                    paths.manifests / manifest_temp_name,
                    maximum_bytes=_MAX_JSON_BYTES,
                    description="temporary checkpoint manifest",
                )
                if reopened_manifest != manifest_bytes:
                    raise MappingBackendCheckpointValidationError(
                        "temporary checkpoint manifest changed after fsync"
                    )
                manifest_sha256 = hashlib.sha256(reopened_manifest).hexdigest()
                manifest_relative = _manifest_relative_path(manifest_sha256)
                manifest_name = f"sha256-{manifest_sha256}.json"
                manifest_path = paths.manifests / manifest_name
                self._install_immutable(
                    access.manifests_fd,
                    manifest_temp_name,
                    manifest_name,
                    manifest_path,
                    reopened_manifest,
                    temp_identity=manifest_temp_identity,
                    maximum_bytes=_MAX_JSON_BYTES,
                    description="checkpoint manifest",
                )
                manifest_temp_name = None
                _inject_fault(fault_injector, "manifest_installed")
                _fsync_directory_fd(access.manifests_fd, paths.manifests)
                _inject_fault(fault_injector, "manifests_directory_fsynced")

                selector_document = {
                    "schema_version": validated.schema_version,
                    "backend_kind": validated.backend_kind,
                    "manifest": {
                        "sha256": manifest_sha256,
                        "path": manifest_relative,
                    },
                }
                selector_bytes = _encode_json(selector_document)
                selector_temp_name = (
                    f".tmp-checkpoint-v{validated.schema_version}-"
                    f"{uuid.uuid4().hex}.json"
                )
                selector_temp_identity = self._write_temp_file(
                    access.backend_fd,
                    selector_temp_name,
                    paths.backend / selector_temp_name,
                    selector_bytes,
                    written_phase="selector_temp_written",
                    fsynced_phase="selector_file_fsynced",
                    fault_injector=fault_injector,
                )
                reopened_selector = _read_regular_file_at(
                    access.backend_fd,
                    selector_temp_name,
                    paths.backend / selector_temp_name,
                    maximum_bytes=_MAX_JSON_BYTES,
                    description="temporary checkpoint selector",
                )
                if reopened_selector != selector_bytes:
                    raise MappingBackendCheckpointValidationError(
                        "temporary checkpoint selector changed after fsync"
                    )
                _verify_access_layout(access)
                _replace_regular_file_at(
                    access.backend_fd,
                    selector_temp_name,
                    paths.selector.name,
                    paths.selector,
                    expected_source=selector_temp_identity,
                )
                selector_temp_name = None
                _inject_fault(fault_injector, "selector_installed")
                _fsync_directory_fd(access.backend_fd, paths.backend)
                _verify_access_layout(access)
                _inject_fault(fault_injector, "backend_directory_fsynced")
            finally:
                for directory_fd, temp_name, identity in (
                    (
                        access.objects_fd,
                        payload_temp_name,
                        payload_temp_identity,
                    ),
                    (
                        access.manifests_fd,
                        manifest_temp_name,
                        manifest_temp_identity,
                    ),
                    (
                        access.backend_fd,
                        selector_temp_name,
                        selector_temp_identity,
                    ),
                ):
                    if temp_name is not None and identity is not None:
                        _unlink_at_if_same(directory_fd, temp_name, identity)

            stored = self._load_optional(
                validated.backend_kind,
                validated.schema_version,
                access=access,
            )
            if stored is None:
                raise MappingBackendCheckpointValidationError(
                    "published checkpoint selector disappeared under the store lock"
                )
            return stored
        finally:
            access.close()

    def load(
        self,
        backend_kind: str,
        *,
        schema_version: int = MAPPING_BACKEND_CHECKPOINT_SCHEMA_VERSION,
        expected_checkpoint_id: Optional[str] = None,
    ) -> StoredMappingBackendCheckpoint:
        """Load only through the backend's stable selector."""

        kind = _validate_backend_kind(backend_kind)
        schema = _validate_schema_version(schema_version)
        if expected_checkpoint_id is not None:
            if schema != ASSIGNED_CHECKPOINT_SCHEMA_VERSION:
                raise MappingBackendCheckpointValidationError(
                    "assigned checkpoint expectation requires schema 2"
                )
            _validate_assigned_id(expected_checkpoint_id, "expected checkpoint id")
        access = self._existing_layout(kind, schema, exclusive=False)
        try:
            stored = self._load_optional(kind, schema, access=access)
            if stored is None:
                raise MappingBackendCheckpointNotFoundError(
                    "mapping backend checkpoint selector does not exist: "
                    f"{access.paths.selector}"
                )
            if (expected_checkpoint_id is not None
                    and stored.checkpoint.checkpoint_id != expected_checkpoint_id):
                raise MappingBackendCheckpointValidationError(
                    "checkpoint selector does not match the expected assigned identity"
                )
            return stored
        finally:
            access.close()

    def cleanup(
        self,
        backend_kind: str,
        *,
        schema_version: int = MAPPING_BACKEND_CHECKPOINT_SCHEMA_VERSION,
        now_ns: Optional[int] = None,
    ) -> CheckpointCleanupResult:
        """Remove old temps and unreachable immutable files after the grace period."""

        kind = _validate_backend_kind(backend_kind)
        schema = _validate_schema_version(schema_version)
        if not _backend_directory_exists(self._version_dir, kind):
            return CheckpointCleanupResult((), (), ())
        access = self._existing_layout(kind, schema, exclusive=True)
        try:
            current = None
            cleanup_now_ns = (
                self._validated_now_ns()
                if now_ns is None
                else _require_integer(now_ns, "now_ns", minimum=0, maximum=_U64_MAX)
            )
            return self._cleanup_paths(
                access,
                current=current,
                now_ns=cleanup_now_ns,
            )
        finally:
            access.close()

    def _publish_pointcloud_archive(self, access, payload, fault_injector, maximum_bytes):
        """点云资产先于引用它的清单落盘，失败只留下可回收的无引用对象。"""
        name = f".tmp-checkpoint-{uuid.uuid4().hex}.bin"
        identity = None
        try:
            identity = self._write_temp_file(
                access.objects_fd, name, access.paths.objects / name, payload,
                written_phase="payload_temp_written", fsynced_phase="payload_file_fsynced",
                fault_injector=fault_injector,
            )
            final_name = f"sha256-{payload_digest(payload).hex()}.bin"
            self._install_immutable(access.objects_fd, name, final_name, access.paths.objects / final_name,
                payload, temp_identity=identity, maximum_bytes=maximum_bytes, description="点云地图资产")
            name = None
            _fsync_directory_fd(access.objects_fd, access.paths.objects)
        finally:
            if name is not None and identity is not None:
                _unlink_at_if_same(access.objects_fd, name, identity)

    def _validate_checkpoint(
        self, checkpoint: MappingBackendCheckpoint, *, legacy_write: bool = False
    ) -> MappingBackendCheckpoint:
        if not isinstance(checkpoint, MappingBackendCheckpoint):
            raise MappingBackendCheckpointValidationError(
                "checkpoint must be a MappingBackendCheckpoint"
            )
        schema_version = _validate_schema_version(checkpoint.schema_version)
        backend_kind = _validate_backend_kind(checkpoint.backend_kind)
        source_session_id = _validate_session_id(checkpoint.source_session_id)
        source_session_generation = _require_integer(
            checkpoint.source_session_generation,
            "source_session_generation",
            minimum=1,
            maximum=_U64_MAX,
        )
        created_at_ns = _require_integer(
            checkpoint.created_at_ns,
            "created_at_ns",
            minimum=1,
            maximum=_U64_MAX,
        )
        payload = checkpoint.payload
        if not isinstance(payload, (bytes, FrozenPayload)):
            raise MappingBackendCheckpointValidationError("检查点载荷必须是字节串或二维私有只读文件")
        payload_size = _require_integer(
            checkpoint.payload_size,
            "payload_size",
            minimum=1,
            maximum=self._payload_limit(backend_kind),
        )
        if len(payload) != payload_size:
            raise MappingBackendCheckpointValidationError(
                f"payload_size mismatch: declared={payload_size} actual={len(payload)}"
            )
        if schema_version == 2:
            _validate_assigned_id(checkpoint.checkpoint_id, "checkpoint_id")
            planar = backend_kind == "slam_toolbox"
            if checkpoint.target_kind != ("planar_graph" if planar else ""):
                raise MappingBackendCheckpointValidationError("checkpoint target kind is invalid")
            if checkpoint.content_sha256 != b"" or type(checkpoint.backend_payload_schema_version) is not int or checkpoint.backend_payload_schema_version != (2 if planar else 1):
                raise MappingBackendCheckpointValidationError("v2 cannot carry a legacy digest or unsupported backend payload version")
            content_sha256 = b""
            if not planar:
                _validate_backend_payload_header(payload, backend_kind)
        else:
            if checkpoint.graph_reference is not None or checkpoint.checkpoint_id or checkpoint.cloud_identity is not None or checkpoint.backend_identity is not None or checkpoint.occupancy_identity is not None:
                raise MappingBackendCheckpointValidationError("legacy checkpoint cannot carry assigned identity")
            content_sha256 = _require_sha256_bytes(checkpoint.content_sha256)
            if legacy_write and payload_digest(payload) != content_sha256:
                raise MappingBackendCheckpointValidationError("content_sha256 does not match checkpoint payload")
        occupancy_receipt = _validate_occupancy_receipt(
            checkpoint.occupancy_receipt,
            source_session_id=source_session_id,
            source_session_generation=source_session_generation,
            backend_kind=backend_kind,
            boot_nonce_encoding="bytes",
        )
        cloud_receipt = None
        if backend_kind == "slam_toolbox":
            if checkpoint.cloud_receipt is not None:
                raise MappingBackendCheckpointValidationError("二维图不能携带三维收据")
        else:
            cloud_receipt = _validate_cloud_receipt(checkpoint.cloud_receipt,
                source_session_generation=source_session_generation, boot_nonce_encoding="bytes")
            _validate_receipt_order(occupancy_receipt, cloud_receipt)
        if checkpoint.fine_archive is not None and schema_version == 1:
            if backend_kind == "slam_toolbox":
                raise MappingBackendCheckpointValidationError("二维检查点不能携带三维精细资产")
            try:
                validate_fine_archive(checkpoint.fine_archive, session=source_session_id,
                    generation=source_session_generation, cloud_revision=cloud_receipt["cloud_revision"],
                    source_timestamp_ns=cloud_receipt["source_timestamp_ns"])
            except (ValueError, TypeError) as error:
                raise MappingBackendCheckpointValidationError(str(error)) from error
        if checkpoint.dense_archive is not None and schema_version == 1:
            if backend_kind == "slam_toolbox":
                raise MappingBackendCheckpointValidationError("二维检查点不能携带完整三维点云")
            try:
                validate_dense_archive(checkpoint.dense_archive, session=source_session_id,
                    generation=source_session_generation, cloud_revision=cloud_receipt["cloud_revision"],
                    source_timestamp_ns=cloud_receipt["source_timestamp_ns"])
            except (ValueError, TypeError) as error:
                raise MappingBackendCheckpointValidationError(str(error)) from error
        validated = replace(checkpoint, schema_version=schema_version, backend_kind=backend_kind,
            source_session_id=source_session_id, source_session_generation=source_session_generation,
            content_sha256=content_sha256, payload_size=payload_size, created_at_ns=created_at_ns,
            occupancy_receipt=occupancy_receipt, cloud_receipt=cloud_receipt)
        if schema_version == 2:
            _validate_checkpoint_artifacts(validated)
        return validated


    def _manifest_document(
        self,
        checkpoint: MappingBackendCheckpoint,
        occupancy_asset: Mapping[str, object],
        cloud_asset: Optional[Mapping[str, object]],
    ) -> Dict[str, object]:
        document = {
            "schema_version": checkpoint.schema_version,
            "backend_kind": checkpoint.backend_kind,
            "source_session_id": checkpoint.source_session_id,
            "source_session_generation": checkpoint.source_session_generation,
            "created_at_ns": checkpoint.created_at_ns,
            "payload": {
                "sha256": checkpoint.content_sha256.hex(),
                "path": _object_relative_path(checkpoint.content_sha256.hex()),
                "size": checkpoint.payload_size,
            },
            "canonical_receipts": {
                "occupancy": _receipt_document(checkpoint.occupancy_receipt),
                "cloud": _receipt_document(checkpoint.cloud_receipt) if checkpoint.cloud_receipt is not None else None,
            },
            "canonical_assets": {
                "occupancy": _asset_document(
                    occupancy_asset, sha256_field="raw_data_sha256"
                ),
                "cloud": _asset_document(cloud_asset, sha256_field="xyz_sha256") if cloud_asset is not None else None,
            },
        }
        if checkpoint.fine_archive is not None:
            sha = payload_digest(checkpoint.fine_archive).hex()
            document["fine_archive"] = {"sha256": sha, "path": _object_relative_path(sha), "size": len(checkpoint.fine_archive)}
        if checkpoint.dense_archive is not None:
            sha = payload_digest(checkpoint.dense_archive).hex()
            document["dense_archive"] = {"sha256": sha, "path": _object_relative_path(sha), "size": len(checkpoint.dense_archive)}
        return document

    def _load_optional(
        self,
        backend_kind: str,
        schema_version: int,
        *,
        access: "_StoreAccess",
    ) -> Optional[StoredMappingBackendCheckpoint]:
        if schema_version == 2:
            return self._load_v2_optional(access)
        paths = access.paths
        try:
            os.stat(
                paths.selector.name,
                dir_fd=access.backend_fd,
                follow_symlinks=False,
            )
        except FileNotFoundError:
            return None
        selector_bytes = _read_regular_file_at(
            access.backend_fd,
            paths.selector.name,
            paths.selector,
            maximum_bytes=_MAX_JSON_BYTES,
            description="checkpoint selector",
        )
        selector = _parse_canonical_json(selector_bytes, "checkpoint selector")
        _require_exact_fields(
            selector,
            ("schema_version", "backend_kind", "manifest"),
            "checkpoint selector",
        )
        if _validate_schema_version(selector["schema_version"]) != schema_version:
            raise MappingBackendCheckpointValidationError(
                "checkpoint selector schema_version does not match its filename"
            )
        if _validate_backend_kind(selector["backend_kind"]) != backend_kind:
            raise MappingBackendCheckpointValidationError(
                "checkpoint selector backend_kind does not match its directory"
            )
        manifest_sha256, manifest_relative = _validate_manifest_reference(
            selector["manifest"], "checkpoint selector manifest"
        )
        return self._load_referenced_checkpoint(
            access,
            manifest_sha256=manifest_sha256,
            manifest_relative=manifest_relative,
        )

    def _load_referenced_checkpoint(
        self,
        access: "_StoreAccess",
        *,
        manifest_sha256: str,
        manifest_relative: str,
    ) -> StoredMappingBackendCheckpoint:
        paths = access.paths
        expected_manifest_relative = _manifest_relative_path(manifest_sha256)
        if manifest_relative != expected_manifest_relative:
            raise MappingBackendCheckpointValidationError(
                "checkpoint manifest path is not canonical for its SHA-256"
            )
        manifest_path = paths.backend / expected_manifest_relative
        manifest_bytes = _read_regular_file_at(
            access.manifests_fd,
            manifest_path.name,
            manifest_path,
            maximum_bytes=_MAX_JSON_BYTES,
            description="checkpoint manifest",
        )
        manifest = _parse_canonical_json(manifest_bytes, "checkpoint manifest")
        _require_exact_fields(
            manifest,
            (
                "schema_version",
                "backend_kind",
                "source_session_id",
                "source_session_generation",
                "created_at_ns",
                "payload",
                "canonical_receipts",
                "canonical_assets",
            ) + tuple(key for key in ("fine_archive", "dense_archive") if key in manifest),
            "checkpoint manifest",
        )
        schema_version = _validate_schema_version(manifest["schema_version"])
        backend_kind = _validate_backend_kind(manifest["backend_kind"])
        if schema_version != paths.schema_version or backend_kind != paths.backend_kind:
            raise MappingBackendCheckpointValidationError(
                "checkpoint manifest identity does not match selector location"
            )
        payload_sha256, payload_relative, payload_size = _validate_payload_reference(
            manifest["payload"]
        )
        object_path = paths.backend / payload_relative
        payload = _read_regular_file_at(
            access.objects_fd,
            object_path.name,
            object_path,
            maximum_bytes=self._payload_limit(backend_kind),
            description="checkpoint object",
        )
        if len(payload) != payload_size:
            raise MappingBackendCheckpointValidationError(
                "checkpoint object size does not match manifest"
            )
        # Historical names are opaque aliases, never recomputed on read.
        content_sha256 = bytes.fromhex(payload_sha256)
        receipts = _require_mapping(
            manifest["canonical_receipts"], "canonical_receipts"
        )
        _require_exact_fields(
            receipts, ("occupancy", "cloud"), "canonical_receipts"
        )
        source_session_id = _validate_session_id(manifest["source_session_id"])
        source_session_generation = _require_integer(
            manifest["source_session_generation"],
            "source_session_generation",
            minimum=1,
            maximum=_U64_MAX,
        )
        occupancy_receipt = _validate_occupancy_receipt(
            receipts["occupancy"],
            source_session_id=source_session_id,
            source_session_generation=source_session_generation,
            backend_kind=backend_kind,
            boot_nonce_encoding="hex",
        )
        cloud_receipt = None
        if backend_kind == "slam_toolbox":
            if receipts["cloud"] is not None:
                raise MappingBackendCheckpointValidationError("二维图清单不能携带三维收据")
        else:
            cloud_receipt = _validate_cloud_receipt(receipts["cloud"],
                source_session_generation=source_session_generation, boot_nonce_encoding="hex")
            _validate_receipt_order(occupancy_receipt, cloud_receipt)
        assets = _require_mapping(manifest["canonical_assets"], "canonical_assets")
        _require_exact_fields(assets, ("occupancy", "cloud"), "canonical_assets")
        occupancy_asset = _validate_occupancy_asset(
            assets["occupancy"], sha256_encoding="hex"
        )
        cloud_asset = _validate_optional_cloud(assets["cloud"], backend_kind, sha256_encoding="hex")
        fine_archive = None
        if "fine_archive" in manifest:
            fine_sha, fine_relative, fine_size = _validate_payload_reference(manifest["fine_archive"])
            fine_path = paths.backend / fine_relative
            fine_archive = _read_regular_file_at(access.objects_fd, fine_path.name, fine_path,
                maximum_bytes=MAX_FINE_ARCHIVE_BYTES, description="精细地图资产")
            if len(fine_archive) != fine_size:
                raise MappingBackendCheckpointValidationError("精细地图资产长度或摘要与清单不符")
        dense_archive = None
        if "dense_archive" in manifest:
            dense_sha, dense_relative, dense_size = _validate_payload_reference(manifest["dense_archive"])
            dense_path = paths.backend / dense_relative
            dense_archive = _read_regular_file_at(access.objects_fd, dense_path.name, dense_path,
                maximum_bytes=MAX_DENSE_ARCHIVE_BYTES, description="完整点云资产")
            if len(dense_archive) != dense_size:
                raise MappingBackendCheckpointValidationError("完整点云资产长度或摘要与清单不符")
        checkpoint = self._validate_checkpoint(
            MappingBackendCheckpoint(
                schema_version=schema_version,
                backend_kind=backend_kind,
                source_session_id=source_session_id,
                source_session_generation=source_session_generation,
                content_sha256=content_sha256,
                payload_size=payload_size,
                created_at_ns=manifest["created_at_ns"],
                payload=payload,
                occupancy_receipt=occupancy_receipt,
                cloud_receipt=cloud_receipt,
                fine_archive=fine_archive,
                dense_archive=dense_archive,
            )
        )
        _validate_asset_bindings(checkpoint, occupancy_asset, cloud_asset)
        return StoredMappingBackendCheckpoint(
            checkpoint=checkpoint,
            occupancy_asset=occupancy_asset,
            cloud_asset=cloud_asset,
            selector_path=paths.selector,
            manifest_sha256=manifest_sha256,
            manifest_path=manifest_path,
            object_path=object_path,
            legacy_alias=manifest_sha256,
            member_paths={"backend_state": object_path,
                **({"fine_map": fine_path} if fine_archive is not None else {}),
                **({"dense_cloud": dense_path} if dense_archive is not None else {})},
        )

    def _ensure_layout(
        self, backend_kind: str, *, exclusive: bool
    ) -> "_StoreAccess":
        return _open_store_access(
            self._version_dir,
            backend_kind,
            MAPPING_BACKEND_CHECKPOINT_SCHEMA_VERSION,
            create=True,
            exclusive=exclusive,
            lock_timeout_seconds=self._lock_timeout_seconds,
        )

    def _existing_layout(
        self,
        backend_kind: str,
        schema_version: int,
        *,
        exclusive: bool,
    ) -> "_StoreAccess":
        return _open_store_access(
            self._version_dir,
            backend_kind,
            schema_version,
            create=False,
            exclusive=exclusive,
            lock_timeout_seconds=self._lock_timeout_seconds,
        )

    def _write_temp_file(
        self,
        directory_fd: int,
        name: str,
        path: Path,
        data: bytes,
        *,
        written_phase: str,
        fsynced_phase: str,
        fault_injector: Optional[FaultInjector],
    ) -> "_FileIdentity":
        fd: Optional[int] = None
        identity: Optional[_FileIdentity] = None
        try:
            fd = os.open(
                name,
                os.O_WRONLY | os.O_CREAT | os.O_EXCL | _no_follow_flag(),
                0o600,
                dir_fd=directory_fd,
            )
            metadata = os.fstat(fd)
            if not stat.S_ISREG(metadata.st_mode):
                raise MappingBackendCheckpointValidationError(
                    f"checkpoint temporary path is not a regular file: {path}"
                )
            identity = _FileIdentity(metadata.st_dev, metadata.st_ino)
            with os.fdopen(fd, "wb", closefd=True) as stream:
                fd = None
                for chunk in payload_chunks(data):
                    stream.write(chunk)
                stream.flush()
                _inject_fault(fault_injector, written_phase)
                os.fsync(stream.fileno())
                _inject_fault(fault_injector, fsynced_phase)
            return identity
        except Exception:
            if identity is not None:
                _unlink_at_if_same(directory_fd, name, identity)
            raise
        finally:
            if fd is not None:
                os.close(fd)

    def _install_immutable(
        self,
        directory_fd: int,
        temp_name: str,
        final_name: str,
        final_path: Path,
        expected: bytes,
        *,
        temp_identity: "_FileIdentity",
        maximum_bytes: int,
        description: str,
    ) -> None:
        try:
            os.stat(final_name, dir_fd=directory_fd, follow_symlinks=False)
        except FileNotFoundError:
            _replace_regular_file_at(
                directory_fd,
                temp_name,
                final_name,
                final_path,
                expected_source=temp_identity,
            )
            return
        existing = _read_regular_file_at(
            directory_fd,
            final_name,
            final_path,
            maximum_bytes=maximum_bytes,
            description=description,
        )
        if existing != expected:
            raise MappingBackendCheckpointValidationError(
                f"existing immutable {description} content does not match its name"
            )
        _unlink_at_if_same(directory_fd, temp_name, temp_identity)

    def _cleanup_paths(
        self,
        access: "_StoreAccess",
        *,
        current: Optional[StoredMappingBackendCheckpoint],
        now_ns: int,
    ) -> CheckpointCleanupResult:
        paths = access.paths
        cutoff_ns = now_ns - self._orphan_grace_ns
        referenced_manifests, referenced_objects, issues = self._live_references(access)
        if issues:
            # An uncertain live selector must never make unrelated files collectible.
            return CheckpointCleanupResult((), (), (), tuple(issues))

        removed_temps = _remove_old_matching(
            access.backend_fd,
            paths.backend,
            cutoff_ns=cutoff_ns,
            name_matches=lambda name: _TEMP_PATTERNS[2].fullmatch(name) is not None,
            preserved_names=frozenset(),
        )
        removed_temps += _remove_old_matching(
            access.objects_fd,
            paths.objects,
            cutoff_ns=cutoff_ns,
            name_matches=lambda name: _TEMP_PATTERNS[0].fullmatch(name) is not None,
            preserved_names=frozenset(),
        )
        removed_temps += _remove_old_matching(
            access.manifests_fd,
            paths.manifests,
            cutoff_ns=cutoff_ns,
            name_matches=lambda name: _TEMP_PATTERNS[1].fullmatch(name) is not None,
            preserved_names=frozenset(),
        )
        removed_manifests = _remove_old_matching(
            access.manifests_fd,
            paths.manifests,
            cutoff_ns=cutoff_ns,
            name_matches=lambda name: _MANIFEST_NAME_RE.fullmatch(name) is not None,
            preserved_names=frozenset(referenced_manifests),
        )
        removed_objects = _remove_old_matching(
            access.objects_fd,
            paths.objects,
            cutoff_ns=cutoff_ns,
            name_matches=lambda name: _OBJECT_NAME_RE.fullmatch(name) is not None or re.fullmatch(r"id-[0-9a-f]{32}\.json", name) is not None,
            preserved_names=frozenset(referenced_objects),
        )
        if removed_temps:
            _fsync_directory_fd(access.backend_fd, paths.backend)
            _fsync_directory_fd(access.objects_fd, paths.objects)
            _fsync_directory_fd(access.manifests_fd, paths.manifests)
        if removed_manifests:
            _fsync_directory_fd(access.manifests_fd, paths.manifests)
        if removed_objects:
            _fsync_directory_fd(access.objects_fd, paths.objects)
        return CheckpointCleanupResult(
            removed_temp_files=removed_temps,
            removed_manifests=removed_manifests,
            removed_objects=removed_objects,
        )

    def _publish_v2(self, checkpoint, occupancy_asset, cloud_asset, fault_injector):
        checkpoint, occupancy_asset, cloud_asset = self.validate_checkpoint_bundle(
            checkpoint, occupancy_asset=occupancy_asset, cloud_asset=cloud_asset,
            max_payload_bytes=self._max_payload_bytes)
        access = _open_store_access(self._version_dir, checkpoint.backend_kind, 2,
            create=True, exclusive=True, lock_timeout_seconds=self._lock_timeout_seconds)
        try:
            document, members = _v2_manifest(checkpoint, occupancy_asset, cloud_asset)
            manifest_name = f"id-{checkpoint.checkpoint_id}.json"
            manifest_path = access.paths.manifests / manifest_name
            manifest_bytes = _encode_json(document)
            if _file_exists_at(access.manifests_fd, manifest_name):
                existing = _parse_canonical_json(_read_regular_file_at(access.manifests_fd,
                    manifest_name, manifest_path, maximum_bytes=_MAX_JSON_BYTES,
                    description="checkpoint manifest"), "checkpoint manifest")
                if existing != document:
                    raise MappingBackendCheckpointValidationError("checkpoint ID reused with different metadata")
            # Immutable companions bind one retained artifact ID to its typed
            # descriptor without scanning historical checkpoint manifests.
            companions = {}
            for reference in (checkpoint.graph_reference, checkpoint.fine_reference, checkpoint.dense_reference):
                if reference is None:
                    continue
                name = f"id-{reference.id}.json"
                data = _encode_json(_artifact_document(reference))
                companions[name] = data
                if _file_exists_at(access.objects_fd, name):
                    existing = _parse_canonical_json(_read_regular_file_at(access.objects_fd, name,
                        access.paths.objects / name, maximum_bytes=_MAX_JSON_BYTES,
                        description="artifact descriptor"), "artifact descriptor")
                    if _artifact_from_document(existing, reference.member_identity.role) != reference:
                        raise MappingBackendCheckpointValidationError("artifact ID reused with different descriptor")
                elif _file_exists_at(access.objects_fd, f"id-{reference.id}.bin"):
                    raise MappingBackendCheckpointValidationError("existing artifact lacks its immutable descriptor")
            for role, (name, payload, maximum) in members.items():
                if _file_exists_at(access.objects_fd, name):
                    if checkpoint.backend_kind == "slam_toolbox":
                        metadata = os.stat(name, dir_fd=access.objects_fd, follow_symlinks=False)
                        _validate_owned_regular_file(metadata, access.paths.objects / name)
                        if metadata.st_size != len(payload):
                            raise MappingBackendCheckpointValidationError("assigned planar member size conflict")
                    else:
                        _compare_file_payload_at(access.objects_fd, name, access.paths.objects / name,
                            payload, maximum_bytes=maximum, description=role)
            for name, data in companions.items():
                if not _file_exists_at(access.objects_fd, name):
                    self._publish_assigned_file(access.objects_fd, access.paths.objects, name,
                        data, _MAX_JSON_BYTES, "manifest", fault_injector)
            for role, (name, payload, maximum) in members.items():
                if not _file_exists_at(access.objects_fd, name):
                    self._publish_assigned_file(access.objects_fd, access.paths.objects, name,
                        payload, maximum, "payload", fault_injector,
                        compare_existing=checkpoint.backend_kind != "slam_toolbox")
            _fsync_directory_fd(access.objects_fd, access.paths.objects)
            _inject_fault(fault_injector, "objects_directory_fsynced")
            if not _file_exists_at(access.manifests_fd, manifest_name):
                self._publish_assigned_file(access.manifests_fd, access.paths.manifests,
                    manifest_name, manifest_bytes, _MAX_JSON_BYTES, "manifest", fault_injector)
            _fsync_directory_fd(access.manifests_fd, access.paths.manifests)
            _inject_fault(fault_injector, "manifests_directory_fsynced")
            selector = _encode_json({"schema_version": 2, "backend_kind": checkpoint.backend_kind,
                "manifest": {"checkpoint_id": checkpoint.checkpoint_id, "path": f"manifests/{manifest_name}"}})
            temp = f".tmp-checkpoint-v2-{uuid.uuid4().hex}.json"
            identity = None
            try:
                identity = self._write_temp_file(access.backend_fd, temp, access.paths.backend / temp,
                    selector, written_phase="selector_temp_written", fsynced_phase="selector_file_fsynced",
                    fault_injector=fault_injector)
                _verify_access_layout(access)
                _replace_regular_file_at(access.backend_fd, temp, access.paths.selector.name,
                    access.paths.selector, expected_source=identity)
                temp = None
                _inject_fault(fault_injector, "selector_installed")
                _fsync_directory_fd(access.backend_fd, access.paths.backend)
                _verify_access_layout(access)
                _inject_fault(fault_injector, "backend_directory_fsynced")
            finally:
                if temp is not None and identity is not None:
                    _unlink_at_if_same(access.backend_fd, temp, identity)
            return StoredMappingBackendCheckpoint(checkpoint, occupancy_asset, cloud_asset,
                access.paths.selector, "", manifest_path, access.paths.objects / members["planar_graph" if checkpoint.backend_kind == "slam_toolbox" else "backend_state"][0],
                identity=(checkpoint.checkpoint_id, "manifest"),
                member_paths={role: access.paths.objects / data[0] for role, data in members.items()})
        finally:
            access.close()

    def _publish_assigned_file(self, directory_fd, directory_path, name, payload, maximum, kind, fault_injector, *, compare_existing=True):
        if not isinstance(payload, (bytes, FrozenPayload)) or not 1 <= len(payload) <= maximum:
            raise MappingBackendCheckpointValidationError("immutable member size/type exceeds its role budget")
        temp = f".tmp-{'checkpoint' if kind == 'payload' else 'manifest'}-{uuid.uuid4().hex}.{'bin' if kind == 'payload' else 'json'}"
        identity = None
        try:
            identity = self._write_temp_file(directory_fd, temp, directory_path / temp, payload,
                written_phase=f"{kind}_temp_written", fsynced_phase=f"{kind}_file_fsynced",
                fault_injector=fault_injector)
            metadata = os.stat(temp, dir_fd=directory_fd, follow_symlinks=False)
            _require_file_identity(metadata, identity, "temporary member was substituted")
            if not stat.S_ISREG(metadata.st_mode) or metadata.st_size != len(payload):
                raise MappingBackendCheckpointValidationError("temporary member size changed after fsync")
            # No transient second hard link can strand an interrupted install.
            try:
                _rename_noreplace_at(directory_fd, temp, name)
            except FileExistsError:
                if not compare_existing:
                    raise MappingBackendCheckpointValidationError("unexpected assigned planar install collision")
                _compare_file_payload_at(directory_fd, name, directory_path / name, payload,
                    maximum_bytes=maximum, description="immutable member")
                _unlink_at_if_same(directory_fd, temp, identity)
            else:
                final = os.stat(name, dir_fd=directory_fd, follow_symlinks=False)
                _require_file_identity(final, identity, "installed immutable member was substituted")
            temp = None
            _inject_fault(fault_injector, "object_installed" if kind == "payload" else "manifest_installed")
        finally:
            if temp is not None and identity is not None:
                _unlink_at_if_same(directory_fd, temp, identity)

    def _load_v2_optional(self, access):
        if not _file_exists_at(access.backend_fd, access.paths.selector.name):
            return None
        selector = _parse_canonical_json(_read_regular_file_at(access.backend_fd,
            access.paths.selector.name, access.paths.selector, maximum_bytes=_MAX_JSON_BYTES,
            description="checkpoint selector"), "checkpoint selector")
        checkpoint_id, name = _v2_selector(selector, access.paths.backend_kind)
        path = access.paths.manifests / name
        document = _parse_canonical_json(_read_regular_file_at(access.manifests_fd, name, path,
            maximum_bytes=_MAX_JSON_BYTES, description="checkpoint manifest"), "checkpoint manifest")
        checkpoint, occupancy, cloud, references = _v2_manifest_metadata(document,
            checkpoint_id, access.paths.backend_kind)
        for reference in (checkpoint.graph_reference, checkpoint.fine_reference, checkpoint.dense_reference):
            if reference is not None:
                companion = f"id-{reference.id}.json"
                descriptor = _parse_canonical_json(_read_regular_file_at(access.objects_fd, companion,
                    access.paths.objects / companion, maximum_bytes=_MAX_JSON_BYTES,
                    description="artifact descriptor"), "artifact descriptor")
                if _artifact_from_document(descriptor, reference.member_identity.role) != reference:
                    raise MappingBackendCheckpointValidationError("artifact descriptor differs from checkpoint manifest")
        values = {}
        member_paths = {}
        for role, reference in references.items():
            member_path = access.paths.backend / reference["path"]
            maximum = _member_maximum(role, self._payload_limit(checkpoint.backend_kind))
            value = _read_regular_file_at(access.objects_fd, member_path.name, member_path,
                maximum_bytes=maximum, description=role)
            if len(value) != reference["size"]:
                raise MappingBackendCheckpointValidationError(f"{role} size does not match its reference")
            values[role] = value
            member_paths[role] = member_path
        primary = "planar_graph" if checkpoint.backend_kind == "slam_toolbox" else "backend_state"
        checkpoint = replace(checkpoint, payload=values[primary],
            canonical_occupancy_data=values["canonical_occupancy"], canonical_cloud_xyz=values.get("canonical_cloud"),
            fine_archive=values.get("fine_map"), dense_archive=values.get("dense_cloud"))
        checkpoint, occupancy, cloud = self.validate_checkpoint_bundle(checkpoint,
            occupancy_asset=occupancy, cloud_asset=cloud, max_payload_bytes=self._max_payload_bytes)
        return StoredMappingBackendCheckpoint(checkpoint, occupancy, cloud, access.paths.selector,
            "", path, member_paths[primary], identity=(checkpoint_id, "manifest"), member_paths=member_paths)

    def _live_references(self, access):
        manifests, objects, issues = set(), set(), []
        for schema in (1, 2):
            name = f"checkpoint-v{schema}.json"
            if not _file_exists_at(access.backend_fd, name):
                continue
            try:
                selector = _parse_canonical_json(_read_regular_file_at(access.backend_fd, name,
                    access.paths.backend / name, maximum_bytes=_MAX_JSON_BYTES,
                    description="live checkpoint selector"), "live checkpoint selector")
                if schema == 2:
                    checkpoint_id, manifest_name = _v2_selector(selector, access.paths.backend_kind)
                else:
                    _require_exact_fields(selector, ("schema_version", "backend_kind", "manifest"), "legacy selector")
                    if selector["schema_version"] != 1 or selector["backend_kind"] != access.paths.backend_kind:
                        raise MappingBackendCheckpointValidationError("legacy live selector identity mismatch")
                    _, relative = _validate_manifest_reference(selector["manifest"], "legacy manifest reference")
                    manifest_name = Path(relative).name
                document = _parse_canonical_json(_read_regular_file_at(access.manifests_fd, manifest_name,
                    access.paths.manifests / manifest_name, maximum_bytes=_MAX_JSON_BYTES,
                    description="live checkpoint manifest"), "live checkpoint manifest")
                if schema == 2:
                    _, _, _, references = _v2_manifest_metadata(document, checkpoint_id, access.paths.backend_kind)
                    names = {Path(item["path"]).name for item in references.values()}
                    names.update(f"id-{item['artifact']['id']}.json" for item in references.values() if "artifact" in item)
                else:
                    if document.get("schema_version") != 1 or document.get("backend_kind") != access.paths.backend_kind:
                        raise MappingBackendCheckpointValidationError("legacy live manifest identity mismatch")
                    names = {Path(_validate_payload_reference(document["payload"])[1]).name}
                    for field_name in ("fine_archive", "dense_archive"):
                        if field_name in document:
                            names.add(Path(_validate_payload_reference(document[field_name])[1]).name)
                manifests.add(manifest_name)
                objects.update(names)
            except (MappingBackendCheckpointStoreError, KeyError, TypeError, ValueError, OSError) as error:
                issues.append(f"preserved checkpoint objects: {name}: {error}")
        return manifests, objects, issues

    def _validated_now_ns(self) -> int:
        return _require_integer(
            self._clock_ns(), "clock_ns result", minimum=0, maximum=_U64_MAX
        )


@dataclass(frozen=True)
class _StorePaths:
    backend_kind: str
    schema_version: int
    backend: Path
    objects: Path
    manifests: Path
    selector: Path


@dataclass
class _FileIdentity:
    device: int
    inode: int


@dataclass
class _StoreAccess:
    paths: _StorePaths
    version_fd: int
    mapping_fd: int
    backend_fd: int
    objects_fd: int
    manifests_fd: int
    locked: bool = False

    def close(self) -> None:
        if self.locked:
            try:
                fcntl.flock(self.backend_fd, fcntl.LOCK_UN)
            except OSError:
                # Closing the directory descriptor also releases flock. Keep
                # closing every retained descriptor if an explicit unlock is
                # interrupted or the filesystem has already failed.
                pass
            finally:
                self.locked = False
        for field_name in (
            "manifests_fd",
            "objects_fd",
            "backend_fd",
            "mapping_fd",
            "version_fd",
        ):
            descriptor = getattr(self, field_name)
            if descriptor >= 0:
                try:
                    os.close(descriptor)
                except OSError:
                    pass
                finally:
                    setattr(self, field_name, -1)


def _open_store_access(
    version_dir: Path,
    backend_kind: str,
    schema_version: int,
    *,
    create: bool,
    exclusive: bool,
    lock_timeout_seconds: float,
) -> _StoreAccess:
    _require_secure_directory_features()
    mapping_root = version_dir / "mapping_backend"
    backend = mapping_root / backend_kind
    objects = backend / "objects"
    manifests = backend / "manifests"
    paths = _StorePaths(
        backend_kind=backend_kind,
        schema_version=schema_version,
        backend=backend,
        objects=objects,
        manifests=manifests,
        selector=backend / f"checkpoint-v{schema_version}.json",
    )
    descriptors: List[int] = []
    try:
        version_fd = _open_absolute_directory_without_symlinks(
            version_dir, "map version directory"
        )
        descriptors.append(version_fd)
        mapping_fd = _open_managed_child_directory(
            version_fd,
            "mapping_backend",
            mapping_root,
            create=create,
        )
        descriptors.append(mapping_fd)
        backend_fd = _open_managed_child_directory(
            mapping_fd,
            backend_kind,
            backend,
            create=create,
        )
        descriptors.append(backend_fd)
        objects_fd = _open_managed_child_directory(
            backend_fd,
            "objects",
            objects,
            create=create,
        )
        descriptors.append(objects_fd)
        manifests_fd = _open_managed_child_directory(
            backend_fd,
            "manifests",
            manifests,
            create=create,
        )
        descriptors.append(manifests_fd)
        access = _StoreAccess(
            paths=paths,
            version_fd=version_fd,
            mapping_fd=mapping_fd,
            backend_fd=backend_fd,
            objects_fd=objects_fd,
            manifests_fd=manifests_fd,
        )
        _acquire_directory_lock(
            access.backend_fd,
            exclusive=exclusive,
            timeout_seconds=lock_timeout_seconds,
        )
        access.locked = True
        _verify_access_layout(access)
        return access
    except Exception:
        for descriptor in reversed(descriptors):
            try:
                os.close(descriptor)
            except OSError:
                pass
        raise


def _backend_directory_exists(version_dir: Path, backend_kind: str) -> bool:
    _require_secure_directory_features()
    descriptors: List[int] = []
    try:
        version_fd = _open_absolute_directory_without_symlinks(
            version_dir, "map version directory"
        )
        descriptors.append(version_fd)
        try:
            mapping_fd = _open_managed_child_directory(
                version_fd,
                "mapping_backend",
                version_dir / "mapping_backend",
                create=False,
            )
        except MappingBackendCheckpointNotFoundError:
            return False
        descriptors.append(mapping_fd)
        try:
            backend_fd = _open_managed_child_directory(
                mapping_fd,
                backend_kind,
                version_dir / "mapping_backend" / backend_kind,
                create=False,
            )
        except MappingBackendCheckpointNotFoundError:
            return False
        descriptors.append(backend_fd)
        return True
    finally:
        for descriptor in reversed(descriptors):
            try:
                os.close(descriptor)
            except OSError:
                pass


def _acquire_directory_lock(
    descriptor: int, *, exclusive: bool, timeout_seconds: float
) -> None:
    operation = fcntl.LOCK_EX if exclusive else fcntl.LOCK_SH
    deadline = time.monotonic() + timeout_seconds
    while True:
        try:
            fcntl.flock(descriptor, operation | fcntl.LOCK_NB)
            return
        except BlockingIOError as exc:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                mode = "exclusive" if exclusive else "shared"
                raise MappingBackendCheckpointStoreError(
                    f"timed out acquiring {mode} mapping checkpoint store lock"
                ) from exc
            time.sleep(min(0.01, remaining))


def _open_absolute_directory_without_symlinks(path: Path, description: str) -> int:
    if not path.is_absolute():
        raise MappingBackendCheckpointValidationError(
            f"{description} must use an absolute path"
        )
    components = path.parts
    if not components or components[0] != os.path.sep:
        raise MappingBackendCheckpointValidationError(
            f"{description} has an invalid absolute path shape: {path}"
        )
    flags = _directory_open_flags()
    current_fd = os.open(os.path.sep, flags)
    try:
        for component in components[1:]:
            _validate_path_component(component, description)
            try:
                next_fd = os.open(component, flags, dir_fd=current_fd)
            except FileNotFoundError as exc:
                raise MappingBackendCheckpointNotFoundError(
                    f"{description} does not exist: {path}"
                ) from exc
            except OSError as exc:
                raise MappingBackendCheckpointValidationError(
                    f"{description} contains a symlink or non-directory component: {path}"
                ) from exc
            os.close(current_fd)
            current_fd = next_fd
        _require_directory_fd(current_fd, path, managed=False)
        return current_fd
    except Exception:
        os.close(current_fd)
        raise


def _open_managed_child_directory(
    parent_fd: int,
    name: str,
    path: Path,
    *,
    create: bool,
) -> int:
    _validate_path_component(name, "managed checkpoint directory")
    try:
        descriptor = os.open(name, _directory_open_flags(), dir_fd=parent_fd)
    except FileNotFoundError as exc:
        if not create:
            raise MappingBackendCheckpointNotFoundError(
                f"checkpoint storage directory does not exist: {path}"
            ) from exc
        try:
            os.mkdir(name, mode=0o700, dir_fd=parent_fd)
        except FileExistsError:
            pass
        _fsync_directory_fd(parent_fd, path.parent)
        try:
            descriptor = os.open(name, _directory_open_flags(), dir_fd=parent_fd)
        except OSError as open_exc:
            raise MappingBackendCheckpointValidationError(
                f"cannot securely open created checkpoint directory: {path}"
            ) from open_exc
    except OSError as exc:
        raise MappingBackendCheckpointValidationError(
            f"checkpoint storage path is a symlink or non-directory: {path}"
        ) from exc
    try:
        _require_directory_fd(descriptor, path, managed=True)
    except Exception:
        os.close(descriptor)
        raise
    return descriptor


def _require_directory_fd(descriptor: int, path: Path, *, managed: bool) -> None:
    metadata = os.fstat(descriptor)
    if not stat.S_ISDIR(metadata.st_mode):
        raise MappingBackendCheckpointValidationError(
            f"checkpoint path is not a directory: {path}"
        )
    if managed:
        if metadata.st_uid != os.geteuid():
            raise MappingBackendCheckpointValidationError(
                f"managed checkpoint directory has an unexpected owner: {path}"
            )
        if stat.S_IMODE(metadata.st_mode) & 0o022:
            raise MappingBackendCheckpointValidationError(
                f"managed checkpoint directory is group/world writable: {path}"
            )


def _verify_access_layout(access: _StoreAccess) -> None:
    reopened_version_fd = _open_absolute_directory_without_symlinks(
        access.paths.backend.parent.parent,
        "map version directory",
    )
    try:
        _require_same_inode(
            os.fstat(reopened_version_fd),
            os.fstat(access.version_fd),
            "map version directory was rebound during checkpoint operation",
        )
    finally:
        os.close(reopened_version_fd)
    _require_named_directory_inode(
        access.version_fd,
        "mapping_backend",
        access.mapping_fd,
        "mapping_backend directory",
    )
    _require_named_directory_inode(
        access.mapping_fd,
        access.paths.backend_kind,
        access.backend_fd,
        "mapping backend directory",
    )
    _require_named_directory_inode(
        access.backend_fd,
        "objects",
        access.objects_fd,
        "checkpoint objects directory",
    )
    _require_named_directory_inode(
        access.backend_fd,
        "manifests",
        access.manifests_fd,
        "checkpoint manifests directory",
    )


def _require_named_directory_inode(
    parent_fd: int,
    name: str,
    child_fd: int,
    description: str,
) -> None:
    try:
        named = os.stat(name, dir_fd=parent_fd, follow_symlinks=False)
    except OSError as exc:
        raise MappingBackendCheckpointValidationError(
            f"{description} was removed or rebound during checkpoint operation"
        ) from exc
    if not stat.S_ISDIR(named.st_mode):
        raise MappingBackendCheckpointValidationError(
            f"{description} was replaced by a non-directory"
        )
    _require_same_inode(
        named,
        os.fstat(child_fd),
        f"{description} was rebound during checkpoint operation",
    )


def _require_same_inode(left: os.stat_result, right: os.stat_result, message: str) -> None:
    if (left.st_dev, left.st_ino) != (right.st_dev, right.st_ino):
        raise MappingBackendCheckpointValidationError(message)


def _validate_path_component(value: str, description: str) -> None:
    if not value or value in {".", ".."} or os.path.sep in value:
        raise MappingBackendCheckpointValidationError(
            f"{description} contains an invalid path component: {value!r}"
        )
    if os.path.altsep is not None and os.path.altsep in value:
        raise MappingBackendCheckpointValidationError(
            f"{description} contains an invalid path component: {value!r}"
        )


def _require_secure_directory_features() -> None:
    required_dir_fd_functions = (os.open, os.mkdir, os.stat, os.unlink)
    if (
        not hasattr(os, "O_NOFOLLOW")
        or not hasattr(os, "O_DIRECTORY")
        or any(function not in os.supports_dir_fd for function in required_dir_fd_functions)
        or os.listdir not in os.supports_fd
    ):
        raise MappingBackendCheckpointValidationError(
            "secure checkpoint storage requires Linux no-follow dir_fd operations"
        )


def _directory_open_flags() -> int:
    flags = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW
    if hasattr(os, "O_CLOEXEC"):
        flags |= os.O_CLOEXEC
    return flags


_MEMBER_ROLES = frozenset({"backend_state", "canonical_occupancy", "canonical_cloud", "fine_map", "dense_cloud", "planar_graph"})


def _validate_assigned_id(value, name):
    if not isinstance(value, str) or re.fullmatch(r"[0-9a-f]{32}", value) is None or value == "0" * 32:
        raise MappingBackendCheckpointValidationError(f"{name} must be an assigned 32-lowercase-hex ID")
    return value


def _validate_member_identity(identity, *, checkpoint=None, role=None, frame=None):
    if not isinstance(identity, CheckpointMemberIdentity):
        raise MappingBackendCheckpointValidationError("member identity is required")
    _validate_assigned_id(identity.checkpoint_id, "member checkpoint_id")
    if identity.role not in _MEMBER_ROLES or (role is not None and identity.role != role):
        raise MappingBackendCheckpointValidationError("member role mismatch")
    _validate_frame_id(identity.frame_id, "member frame_id")
    for key in ("frame_generation", "producer_generation"):
        _require_integer(getattr(identity, key), key, minimum=0, maximum=_U64_MAX)
    if identity.role != "fine_map" and (identity.frame_generation or identity.producer_generation):
        raise MappingBackendCheckpointValidationError("non-fine member cannot carry fine root generations")
    if identity.role == "fine_map" and not identity.producer_generation:
        raise MappingBackendCheckpointValidationError("fine member producer generation is required")
    occupancy = identity.occupancy_receipt
    _validate_occupancy_receipt(occupancy, source_session_id=occupancy.get("session_id"),
        source_session_generation=occupancy.get("session_generation"), backend_kind=occupancy.get("backend_kind"),
        boot_nonce_encoding="bytes")
    if occupancy["backend_kind"] == "slam_toolbox":
        if identity.cloud_receipt is not None:
            raise MappingBackendCheckpointValidationError("planar identity cannot carry cloud receipt")
    else:
        cloud = _validate_cloud_receipt(identity.cloud_receipt,
            source_session_generation=occupancy["session_generation"], boot_nonce_encoding="bytes")
        _validate_receipt_order(occupancy, cloud)
    if checkpoint is not None and (identity.checkpoint_id != checkpoint.checkpoint_id
        or identity.occupancy_receipt != checkpoint.occupancy_receipt
        or identity.cloud_receipt != checkpoint.cloud_receipt):
        raise MappingBackendCheckpointValidationError("member source identity differs from parent checkpoint")
    if frame is not None and identity.frame_id != frame:
        raise MappingBackendCheckpointValidationError("member frame differs from canonical frame")
    return identity


def validate_artifact_reference(reference, *, checkpoint=None, role=None, frame=None, maximum_bytes=MAX_PLANAR_ARCHIVE_BYTES):
    if not isinstance(reference, CheckpointArtifactReference):
        raise MappingBackendCheckpointValidationError("artifact reference type is invalid")
    _require_integer(reference.size, "artifact size", minimum=1, maximum=maximum_bytes)
    if type(reference.identity_schema_version) is not int:
        raise MappingBackendCheckpointValidationError("artifact identity schema must be an integer")
    if reference.identity_schema_version == 2:
        _validate_assigned_id(reference.id, "artifact id")
        if reference.legacy_alias != b"":
            raise MappingBackendCheckpointValidationError("assigned artifact cannot carry an active legacy digest")
        _validate_member_identity(reference.member_identity, checkpoint=checkpoint, role=role, frame=frame)
        if reference.member_identity.role not in ("fine_map", "dense_cloud", "planar_graph"):
            raise MappingBackendCheckpointValidationError("member role is not transferable")
        role_maximum = {"fine_map": MAX_FINE_ARCHIVE_BYTES, "dense_cloud": MAX_DENSE_ARCHIVE_BYTES,
            "planar_graph": MAX_PLANAR_ARCHIVE_BYTES}[reference.member_identity.role]
        if reference.size > role_maximum:
            raise MappingBackendCheckpointValidationError("artifact size exceeds its member role budget")
    elif reference.identity_schema_version in (0, 1):
        if not isinstance(reference.id, str) or not reference.id or len(reference.id.encode("utf-8")) > 128:
            raise MappingBackendCheckpointValidationError("legacy artifact ID must be nonempty and at most 128 bytes")
        if reference.member_identity is not None or not isinstance(reference.legacy_alias, bytes) or len(reference.legacy_alias) != 32 or not any(reference.legacy_alias):
            raise MappingBackendCheckpointValidationError("legacy artifact reference is ambiguous or incomplete")
        if checkpoint is not None and checkpoint.schema_version == 2:
            raise MappingBackendCheckpointValidationError("v2 checkpoint requires assigned artifact references")
    else:
        raise MappingBackendCheckpointValidationError("unsupported artifact identity schema")
    return reference


def _validate_checkpoint_artifacts(checkpoint):
    if checkpoint.backend_kind == "slam_toolbox":
        if any(value is not None for value in (checkpoint.fine_archive, checkpoint.dense_archive,
                checkpoint.fine_reference, checkpoint.dense_reference, checkpoint.cloud_identity,
                checkpoint.backend_identity, checkpoint.canonical_cloud_xyz)):
            raise MappingBackendCheckpointValidationError("planar checkpoint cannot carry cloud members")
        validate_artifact_reference(checkpoint.graph_reference, checkpoint=checkpoint, role="planar_graph")
        if not isinstance(checkpoint.payload, FrozenPayload) or len(checkpoint.payload) != checkpoint.graph_reference.size:
            raise MappingBackendCheckpointValidationError("planar graph requires its retained FrozenPayload")
        return
    if checkpoint.graph_reference is not None:
        raise MappingBackendCheckpointValidationError("cloud checkpoint cannot carry graph reference")
    if (checkpoint.fine_reference is not None and checkpoint.dense_reference is not None
            and checkpoint.fine_reference.id == checkpoint.dense_reference.id):
        raise MappingBackendCheckpointValidationError("distinct checkpoint members cannot reuse one artifact ID")
    for role, payload, reference, maximum in (
        ("fine_map", checkpoint.fine_archive, checkpoint.fine_reference, MAX_FINE_ARCHIVE_BYTES),
        ("dense_cloud", checkpoint.dense_archive, checkpoint.dense_reference, MAX_DENSE_ARCHIVE_BYTES)):
        if (payload is None) != (reference is None):
            raise MappingBackendCheckpointValidationError("archive bytes and retained reference must both be present")
        if payload is None:
            continue
        validate_artifact_reference(reference, checkpoint=checkpoint, role=role, maximum_bytes=maximum)
        if not isinstance(payload, (bytes, FrozenPayload)) or len(payload) != reference.size:
            raise MappingBackendCheckpointValidationError("artifact bytes do not match declared size")
        keywords = dict(session=checkpoint.source_session_id, generation=checkpoint.source_session_generation,
            cloud_revision=checkpoint.cloud_receipt["cloud_revision"],
            source_timestamp_ns=checkpoint.cloud_receipt["source_timestamp_ns"])
        if role == "fine_map":
            if payload[:8] not in (b"NUWAXAR2", b"NUWAXAR3"):
                raise MappingBackendCheckpointValidationError("assigned fine member requires AR2 or AR3")
            validate_fine_archive(payload, **keywords,
                frame_generation=reference.member_identity.frame_generation,
                producer_generation=reference.member_identity.producer_generation)
        else:
            validate_dense_archive(payload, **keywords, frame_id=reference.member_identity.frame_id)


def _validate_canonical_members(checkpoint, occupancy, cloud):
    if checkpoint.backend_kind == "slam_toolbox":
        cells = checkpoint.canonical_occupancy_data
        if not isinstance(cells, (bytes, FrozenPayload)) or len(cells) != occupancy["raw_data_size"] or not 1 <= len(cells) <= MAX_GRID:
            raise MappingBackendCheckpointValidationError("exact canonical planar occupancy bytes are required")
        _validate_member_identity(checkpoint.occupancy_identity, checkpoint=checkpoint,
            role="canonical_occupancy", frame=occupancy["frame_id"])
        _validate_member_identity(checkpoint.graph_reference.member_identity, checkpoint=checkpoint,
            role="planar_graph", frame=occupancy["frame_id"])
        try:
            read_planar_structure(checkpoint.payload, checkpoint_id=checkpoint.checkpoint_id,
                session_id=checkpoint.source_session_id, generation=checkpoint.source_session_generation,
                timestamp_ns=checkpoint.occupancy_receipt["source_timestamp_ns"], occupancy=occupancy, cells=cells)
        except (ValueError, TypeError, KeyError) as error:
            raise MappingBackendCheckpointValidationError(str(error)) from error
        return
    for value, size, name in ((checkpoint.canonical_occupancy_data, occupancy["raw_data_size"], "occupancy"),
                             (checkpoint.canonical_cloud_xyz, cloud["xyz_size"], "cloud")):
        maximum = _member_maximum("canonical_" + name, MAX_MAPPING_BACKEND_CHECKPOINT_BYTES)
        if not isinstance(value, (bytes, FrozenPayload)) or len(value) != size or not 1 <= size <= maximum:
            raise MappingBackendCheckpointValidationError(f"exact canonical {name} bytes are required")
    for chunk in payload_chunks(checkpoint.canonical_occupancy_data):
        values = np.frombuffer(chunk, dtype=np.uint8)
        if ((values > 100) & (values != 255)).any():
            raise MappingBackendCheckpointValidationError("canonical occupancy values must be [-1, 100]")
    xyz = checkpoint.canonical_cloud_xyz
    for offset in range(0, len(xyz), 65536 * 12):
        if not np.isfinite(np.frombuffer(xyz[offset:offset + 65536 * 12], dtype="<f4")).all():
            raise MappingBackendCheckpointValidationError("canonical cloud contains non-finite XYZ")
    _validate_member_identity(checkpoint.backend_identity, checkpoint=checkpoint,
        role="backend_state", frame=occupancy["frame_id"])
    _validate_member_identity(checkpoint.occupancy_identity, checkpoint=checkpoint,
        role="canonical_occupancy", frame=occupancy["frame_id"])
    _validate_member_identity(checkpoint.cloud_identity, checkpoint=checkpoint,
        role="canonical_cloud", frame=cloud["frame_id"])
    for role, reference in (("fine_map", checkpoint.fine_reference), ("dense_cloud", checkpoint.dense_reference)):
        if reference is not None:
            _validate_member_identity(reference.member_identity, checkpoint=checkpoint, role=role, frame=cloud["frame_id"])


def assign_legacy_cloud_checkpoint(stored_checkpoint, *, canonical_occupancy_data,
                                   canonical_cloud_xyz, checkpoint_id, dense_artifact_id=None):
    """Convert an actual legacy cloud owner; allocation/publication belong to importer."""
    if not isinstance(stored_checkpoint, StoredMappingBackendCheckpoint):
        raise TypeError("cloud import requires a stored checkpoint owner")
    previous = stored_checkpoint.checkpoint
    if (previous.schema_version != 1 or previous.backend_kind not in {"voxel", "raytracing"}
            or previous.cloud_receipt is None or stored_checkpoint.cloud_asset is None
            or previous.fine_archive is not None):
        raise ValueError("cloud import requires original receipts and no fine archive")
    _validate_assigned_id(checkpoint_id, "checkpoint_id")
    if (previous.dense_archive is None) != (dense_artifact_id is None):
        raise ValueError("dense import bytes and owned artifact ID must both be present")
    occupancy = {key: value for key, value in stored_checkpoint.occupancy_asset.items()
                 if key != "raw_data_sha256"}
    cloud = {key: value for key, value in stored_checkpoint.cloud_asset.items() if key != "xyz_sha256"}
    cloud["payload_schema_version"] = 2
    def member(role, frame):
        return CheckpointMemberIdentity(checkpoint_id, role, previous.occupancy_receipt,
            previous.cloud_receipt, frame)
    dense_reference = None
    if previous.dense_archive is not None:
        _validate_assigned_id(dense_artifact_id, "dense artifact id")
        dense_reference = CheckpointArtifactReference(dense_artifact_id, len(previous.dense_archive),
            2, member("dense_cloud", cloud["frame_id"]))
    checkpoint = MappingBackendCheckpoint(schema_version=2, backend_kind=previous.backend_kind,
        source_session_id=previous.source_session_id, source_session_generation=previous.source_session_generation,
        content_sha256=b"", payload_size=previous.payload_size, created_at_ns=previous.created_at_ns,
        payload=previous.payload, occupancy_receipt=previous.occupancy_receipt, cloud_receipt=previous.cloud_receipt,
        checkpoint_id=checkpoint_id, backend_payload_schema_version=1,
        canonical_occupancy_data=canonical_occupancy_data, canonical_cloud_xyz=canonical_cloud_xyz,
        backend_identity=member("backend_state", occupancy["frame_id"]),
        occupancy_identity=member("canonical_occupancy", occupancy["frame_id"]),
        cloud_identity=member("canonical_cloud", cloud["frame_id"]),
        dense_archive=previous.dense_archive, dense_reference=dense_reference)
    return MappingBackendCheckpointStore.validate_checkpoint_bundle(checkpoint,
        occupancy_asset=occupancy, cloud_asset=cloud)


def assign_legacy_planar_checkpoint(stored_checkpoint, *, checkpoint_id, artifact_id):
    """Build an assigned candidate from a read-only legacy checkpoint owner.

    IDs belong to the outer save/import candidate. This never publishes, changes
    the old asset, or invents a canonical commit receipt.
    """
    if not isinstance(stored_checkpoint, StoredMappingBackendCheckpoint):
        raise TypeError("planar import requires a stored checkpoint owner")
    previous = stored_checkpoint.checkpoint
    if (previous.schema_version != 1 or previous.backend_kind != "slam_toolbox"
            or previous.cloud_receipt is not None or stored_checkpoint.cloud_asset is not None
            or previous.fine_archive is not None or previous.dense_archive is not None):
        raise ValueError("planar import requires a legacy planar checkpoint")
    _validate_assigned_id(checkpoint_id, "checkpoint_id")
    _validate_assigned_id(artifact_id, "artifact_id")
    occupancy = {key: value for key, value in stored_checkpoint.occupancy_asset.items() if key != "raw_data_sha256"}
    grid_size = occupancy["raw_data_size"]
    if type(grid_size) is not int or not 1 <= grid_size <= MAX_GRID or grid_size > len(previous.payload):
        raise ValueError("legacy planar canonical grid size exceeds budget")
    payload = assign_legacy_planar_archive(previous.payload, checkpoint_id)
    graph_identity = CheckpointMemberIdentity(checkpoint_id, "planar_graph", previous.occupancy_receipt,
        None, occupancy["frame_id"])
    checkpoint = MappingBackendCheckpoint(schema_version=2, backend_kind="slam_toolbox",
        source_session_id=previous.source_session_id, source_session_generation=previous.source_session_generation,
        content_sha256=b"", payload_size=len(payload), created_at_ns=previous.created_at_ns,
        payload=payload, occupancy_receipt=previous.occupancy_receipt, cloud_receipt=None,
        checkpoint_id=checkpoint_id, target_kind="planar_graph", backend_payload_schema_version=2,
        canonical_occupancy_data=payload[len(payload) - grid_size:],
        graph_reference=CheckpointArtifactReference(artifact_id, len(payload), 2, graph_identity),
        occupancy_identity=CheckpointMemberIdentity(checkpoint_id, "canonical_occupancy", previous.occupancy_receipt,
            None, occupancy["frame_id"]))
    checkpoint, occupancy, cloud = MappingBackendCheckpointStore.validate_checkpoint_bundle(checkpoint,
        occupancy_asset=occupancy, cloud_asset=None)
    return checkpoint, occupancy, cloud


def _validate_backend_payload_header(payload, kind):
    if len(payload) < 16 or payload[:8] != b"NWMBCKP\0":
        raise MappingBackendCheckpointValidationError("backend payload magic/header is invalid")
    version, stored_kind, flags, reserved = struct.unpack("<IBBH", payload[8:16])
    if version != 1 or stored_kind != {"voxel": 0, "raytracing": 1}[kind] or flags or reserved:
        raise MappingBackendCheckpointValidationError("backend payload version/kind/flags mismatch")
    if kind == "voxel":
        if len(payload) < 48:
            raise MappingBackendCheckpointValidationError("voxel backend metadata is truncated")
        capacity, count, candidate_count, received = struct.unpack("<QQQQ", payload[16:48])
        retained_capacity = min(capacity, (MAX_MAPPING_BACKEND_CHECKPOINT_BYTES - 48) // 24)
        if not capacity or count > retained_capacity or candidate_count > retained_capacity or len(payload) != 48 + count * 24:
            raise MappingBackendCheckpointValidationError("voxel backend size/count mismatch")
        for offset in range(48, len(payload), 65536 * 24):
            if not np.isfinite(np.frombuffer(payload[offset:offset + 65536 * 24], dtype="<f8")).all():
                raise MappingBackendCheckpointValidationError("voxel backend contains non-finite coordinates")
    else:
        if len(payload) < 96:
            raise MappingBackendCheckpointValidationError("ray backend metadata is truncated")
        map_size, occupancy_size = struct.unpack("<QQ", payload[80:96])
        if not map_size or len(payload) != 96 + map_size + occupancy_size:
            raise MappingBackendCheckpointValidationError("ray backend member lengths mismatch")


def _identity_document(identity):
    return {"checkpoint_id": identity.checkpoint_id, "role": identity.role,
        "occupancy_receipt": _receipt_document(identity.occupancy_receipt),
        "cloud_receipt": _receipt_document(identity.cloud_receipt) if identity.cloud_receipt is not None else None,
        "frame_id": identity.frame_id, "frame_generation": identity.frame_generation,
        "producer_generation": identity.producer_generation}


def _identity_from_document(value):
    value = _require_mapping(value, "member identity")
    _require_exact_fields(value, ("checkpoint_id", "role", "occupancy_receipt", "cloud_receipt", "frame_id",
        "frame_generation", "producer_generation"), "member identity")
    receipt = _require_mapping(value["occupancy_receipt"], "member occupancy receipt")
    occupancy = _validate_occupancy_receipt(receipt, source_session_id=receipt.get("session_id"),
        source_session_generation=receipt.get("session_generation"), backend_kind=receipt.get("backend_kind"), boot_nonce_encoding="hex")
    cloud = None if value["cloud_receipt"] is None else _validate_cloud_receipt(value["cloud_receipt"],
        source_session_generation=occupancy["session_generation"], boot_nonce_encoding="hex")
    return _validate_member_identity(CheckpointMemberIdentity(value["checkpoint_id"], value["role"], occupancy,
        cloud, value["frame_id"], value["frame_generation"], value["producer_generation"]))


def _artifact_document(reference):
    return {"id": reference.id, "size": reference.size, "identity_schema_version": 2,
        "member_identity": _identity_document(reference.member_identity)}


def _artifact_from_document(value, role):
    value = _require_mapping(value, "artifact reference")
    _require_exact_fields(value, ("id", "size", "identity_schema_version", "member_identity"), "artifact reference")
    return validate_artifact_reference(CheckpointArtifactReference(value["id"], value["size"],
        value["identity_schema_version"], _identity_from_document(value["member_identity"])), role=role,
        maximum_bytes=_member_maximum(role, MAX_MAPPING_BACKEND_CHECKPOINT_BYTES))


def _member_maximum(role, backend_maximum):
    return {"planar_graph": MAX_PLANAR_ARCHIVE_BYTES, "backend_state": backend_maximum, "canonical_occupancy": MAX_MAPPING_BACKEND_CHECKPOINT_BYTES,
        "canonical_cloud": 128 * 1024 * 1024, "fine_map": MAX_FINE_ARCHIVE_BYTES,
        "dense_cloud": MAX_DENSE_ARCHIVE_BYTES}[role]


def _v2_manifest(checkpoint, occupancy, cloud):
    members, references = {}, {}
    planar = checkpoint.backend_kind == "slam_toolbox"
    entries = (("planar_graph", None, checkpoint.payload, checkpoint.graph_reference.member_identity, checkpoint.graph_reference),
        ("canonical_occupancy", "occupancy", checkpoint.canonical_occupancy_data, checkpoint.occupancy_identity, None)) if planar else (
        ("backend_state", "backend", checkpoint.payload, checkpoint.backend_identity, None),
        ("canonical_occupancy", "occupancy", checkpoint.canonical_occupancy_data, checkpoint.occupancy_identity, None),
        ("canonical_cloud", "cloud", checkpoint.canonical_cloud_xyz, checkpoint.cloud_identity, None),
        ("fine_map", None, checkpoint.fine_archive, checkpoint.fine_reference.member_identity if checkpoint.fine_reference else None, checkpoint.fine_reference),
        ("dense_cloud", None, checkpoint.dense_archive, checkpoint.dense_reference.member_identity if checkpoint.dense_reference else None, checkpoint.dense_reference))
    for role, suffix, payload, identity, artifact in entries:
        if payload is None:
            continue
        name = f"id-{checkpoint.checkpoint_id}-{suffix}.bin" if suffix else f"id-{artifact.id}.bin"
        references[role] = {"identity": _identity_document(identity), "role": role,
            "path": f"objects/{name}", "size": len(payload)}
        if artifact is not None:
            references[role]["artifact"] = _artifact_document(artifact)
        members[role] = (name, payload, _member_maximum(role, MAX_MAPPING_BACKEND_CHECKPOINT_BYTES))
    return {"schema_version": 2, "checkpoint_id": checkpoint.checkpoint_id,
        "backend_payload_schema_version": checkpoint.backend_payload_schema_version,
        "backend_kind": checkpoint.backend_kind, "target_kind": checkpoint.target_kind, "source_session_id": checkpoint.source_session_id,
        "source_session_generation": checkpoint.source_session_generation, "created_at_ns": checkpoint.created_at_ns,
        "canonical_receipts": {"occupancy": _receipt_document(checkpoint.occupancy_receipt), "cloud": _receipt_document(checkpoint.cloud_receipt) if cloud is not None else None},
        "canonical_assets": {"occupancy": dict(occupancy), "cloud": dict(cloud) if cloud is not None else None}, "members": references}, members


def _v2_selector(selector, backend_kind):
    _require_exact_fields(selector, ("schema_version", "backend_kind", "manifest"), "checkpoint selector")
    if type(selector["schema_version"]) is not int or selector["schema_version"] != 2 or selector["backend_kind"] != backend_kind:
        raise MappingBackendCheckpointValidationError("v2 selector identity mismatch")
    reference = _require_mapping(selector["manifest"], "manifest reference")
    _require_exact_fields(reference, ("checkpoint_id", "path"), "manifest reference")
    checkpoint_id = _validate_assigned_id(reference["checkpoint_id"], "checkpoint_id")
    name = f"id-{checkpoint_id}.json"
    if reference["path"] != f"manifests/{name}":
        raise MappingBackendCheckpointValidationError("assigned manifest path is not canonical")
    return checkpoint_id, name


def _v2_manifest_metadata(document, checkpoint_id, backend_kind):
    _require_exact_fields(document, ("schema_version", "checkpoint_id", "backend_payload_schema_version", "backend_kind",
        "target_kind", "source_session_id", "source_session_generation", "created_at_ns", "canonical_receipts", "canonical_assets", "members"), "v2 manifest")
    if type(document["schema_version"]) is not int or document["schema_version"] != 2 or document["checkpoint_id"] != checkpoint_id or document["backend_kind"] != backend_kind:
        raise MappingBackendCheckpointValidationError("v2 manifest does not match selector")
    planar = backend_kind == "slam_toolbox"
    if type(document["backend_payload_schema_version"]) is not int or document["backend_payload_schema_version"] != (2 if planar else 1):
        raise MappingBackendCheckpointValidationError("v2 backend payload generation is not supported")
    receipts = _require_mapping(document["canonical_receipts"], "canonical receipts")
    _require_exact_fields(receipts, ("occupancy", "cloud"), "canonical receipts")
    occupancy_receipt = _validate_occupancy_receipt(receipts["occupancy"], source_session_id=document["source_session_id"],
        source_session_generation=document["source_session_generation"], backend_kind=backend_kind, boot_nonce_encoding="hex")
    if planar and receipts["cloud"] is not None:
        raise MappingBackendCheckpointValidationError("planar manifest cannot carry cloud receipt")
    cloud_receipt = None if planar else _validate_cloud_receipt(receipts["cloud"], source_session_generation=document["source_session_generation"], boot_nonce_encoding="hex")
    if not planar:
        _validate_receipt_order(occupancy_receipt, cloud_receipt)
    assets = _require_mapping(document["canonical_assets"], "canonical assets")
    _require_exact_fields(assets, ("occupancy", "cloud"), "canonical assets")
    occupancy = _validate_occupancy_asset(assets["occupancy"], sha256_encoding="none")
    cloud = _validate_optional_cloud(assets["cloud"], backend_kind, sha256_encoding="none")
    members = _require_mapping(document["members"], "checkpoint members")
    required = {"planar_graph", "canonical_occupancy"} if planar else {"backend_state", "canonical_occupancy", "canonical_cloud"}
    optional = set() if planar else {"fine_map", "dense_cloud"}
    if not required <= set(members) or set(members) - required - optional:
        raise MappingBackendCheckpointValidationError("v2 manifest members are incomplete or unknown")
    checkpoint = MappingBackendCheckpoint(2, backend_kind, document["source_session_id"], document["source_session_generation"],
        b"", members["planar_graph" if planar else "backend_state"]["size"], document["created_at_ns"], b"", occupancy_receipt, cloud_receipt,
        checkpoint_id=checkpoint_id, target_kind=document["target_kind"], backend_payload_schema_version=document["backend_payload_schema_version"])
    refs = {}
    paths = set()
    for role, member in members.items():
        member = _require_mapping(member, "checkpoint member")
        archive = role in {"planar_graph", "fine_map", "dense_cloud"}
        _require_exact_fields(member, ("identity", "role", "path", "size") + (("artifact",) if archive else ()), "checkpoint member")
        identity = _identity_from_document(member["identity"])
        _validate_member_identity(identity, checkpoint=checkpoint, role=role, frame=occupancy["frame_id"])
        if member["role"] != role:
            raise MappingBackendCheckpointValidationError("manifest member role mismatch")
        if archive:
            reference = _artifact_from_document(member["artifact"], role)
            if reference.member_identity != identity or reference.size != member["size"]:
                raise MappingBackendCheckpointValidationError("artifact identity differs from manifest member")
            refs[role] = reference
            expected_path = f"objects/id-{reference.id}.bin"
        else:
            suffix = {"backend_state": "backend", "canonical_occupancy": "occupancy", "canonical_cloud": "cloud"}[role]
            expected_path = f"objects/id-{checkpoint_id}-{suffix}.bin"
        _require_integer(member["size"], "member size", minimum=1, maximum=_member_maximum(role, MAX_MAPPING_BACKEND_CHECKPOINT_BYTES))
        if member["path"] != expected_path or expected_path in paths:
            raise MappingBackendCheckpointValidationError("member path is unsafe, duplicated or noncanonical")
        paths.add(expected_path)
    checkpoint = replace(checkpoint, graph_reference=refs.get("planar_graph"), fine_reference=refs.get("fine_map"), dense_reference=refs.get("dense_cloud"),
        cloud_identity=None if planar else _identity_from_document(members["canonical_cloud"]["identity"]),
        backend_identity=None if planar else _identity_from_document(members["backend_state"]["identity"]),
        occupancy_identity=_identity_from_document(members["canonical_occupancy"]["identity"]))
    _validate_asset_bindings(checkpoint, occupancy, cloud)
    return checkpoint, occupancy, cloud, members


def _rename_noreplace_at(directory_fd, source_name, destination_name):
    """Linux atomic immutable install; never fall back to replacing a name."""
    rename = getattr(ctypes.CDLL(None, use_errno=True), "renameat2", None)
    if rename is None:
        raise MappingBackendCheckpointStoreError("assigned checkpoint publication requires renameat2(RENAME_NOREPLACE)")
    rename.argtypes = (ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint)
    rename.restype = ctypes.c_int
    if rename(directory_fd, os.fsencode(source_name), directory_fd, os.fsencode(destination_name), 1) != 0:
        error = ctypes.get_errno()
        if error == errno.EEXIST:
            raise FileExistsError(error, os.strerror(error), destination_name)
        raise OSError(error, os.strerror(error), destination_name)


def _file_exists_at(directory_fd, name):
    try:
        os.stat(name, dir_fd=directory_fd, follow_symlinks=False)
        return True
    except FileNotFoundError:
        return False


def _compare_file_payload_at(directory_fd, name, path, expected, *, maximum_bytes, description):
    _validate_path_component(name, description)
    fd = os.open(name, os.O_RDONLY | _no_follow_flag(), dir_fd=directory_fd)
    try:
        metadata = os.fstat(fd)
        _validate_owned_regular_file(metadata, path)
        if metadata.st_size != len(expected) or metadata.st_size > maximum_bytes:
            raise MappingBackendCheckpointValidationError(f"same-ID {description} size conflict")
        with os.fdopen(fd, "rb", closefd=True) as stream:
            fd = -1
            for chunk in payload_chunks(expected):
                if stream.read(len(chunk)) != chunk:
                    raise MappingBackendCheckpointValidationError(f"same-ID {description} byte conflict")
            if stream.read(1):
                raise MappingBackendCheckpointValidationError(f"same-ID {description} changed during comparison")
            after = os.fstat(stream.fileno())
            if (metadata.st_dev, metadata.st_ino, metadata.st_size, metadata.st_mtime_ns, metadata.st_ctime_ns) != (
                after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns, after.st_ctime_ns):
                raise MappingBackendCheckpointValidationError(f"same-ID {description} mutated during comparison")
    finally:
        if fd >= 0:
            os.close(fd)


def _validate_owned_regular_file(metadata, path):
    if not stat.S_ISREG(metadata.st_mode) or metadata.st_uid != os.geteuid() or metadata.st_nlink != 1 or stat.S_IMODE(metadata.st_mode) & 0o022:
        raise MappingBackendCheckpointValidationError(f"member must be a private owned single-link regular file: {path}")


def _validate_schema_version(value: object) -> int:
    schema_version = _require_integer(
        value,
        "schema_version",
        minimum=1,
        maximum=_U32_MAX,
    )
    if schema_version not in (1, 2):
        raise MappingBackendCheckpointValidationError(
            f"unsupported mapping backend checkpoint schema_version: {schema_version}"
        )
    return schema_version


def _validate_backend_kind(value: object) -> str:
    if not isinstance(value, str) or value not in SUPPORTED_MAPPING_BACKEND_KINDS:
        raise MappingBackendCheckpointValidationError(
            "backend_kind must be exactly 'voxel' or 'raytracing'"
        )
    return value


def _validate_session_id(value: object) -> str:
    if not isinstance(value, str) or not value or value.strip() != value:
        raise MappingBackendCheckpointValidationError(
            "source_session_id must be a non-empty, trimmed string"
        )
    if len(value) > 256 or any(ord(character) < 0x20 for character in value):
        raise MappingBackendCheckpointValidationError(
            "source_session_id contains invalid characters or is too long"
        )
    return value


def _validate_occupancy_receipt(
    value: object,
    *,
    source_session_id: str,
    source_session_generation: int,
    backend_kind: str,
    boot_nonce_encoding: str,
) -> Dict[str, object]:
    receipt = _require_mapping(value, "occupancy_receipt")
    fields = (
        "commit_id",
        "session_id",
        "session_generation",
        "occupancy_revision",
        "source_timestamp_ns",
        "backend_kind",
        "backend_generation",
    )
    _require_exact_fields(receipt, fields, "occupancy_receipt")
    session_id = _validate_session_id(receipt["session_id"])
    session_generation = _require_integer(
        receipt["session_generation"],
        "occupancy_receipt.session_generation",
        minimum=1,
        maximum=_U64_MAX,
    )
    receipt_backend_kind = _validate_backend_kind(receipt["backend_kind"])
    if session_id != source_session_id:
        raise MappingBackendCheckpointValidationError(
            "occupancy_receipt.session_id does not match source_session_id"
        )
    if session_generation != source_session_generation:
        raise MappingBackendCheckpointValidationError(
            "occupancy_receipt.session_generation does not match source session"
        )
    if receipt_backend_kind != backend_kind:
        raise MappingBackendCheckpointValidationError(
            "occupancy_receipt.backend_kind does not match checkpoint backend_kind"
        )
    return {
        "commit_id": _validate_commit_id(
            receipt["commit_id"], boot_nonce_encoding=boot_nonce_encoding
        ),
        "session_id": session_id,
        "session_generation": session_generation,
        "occupancy_revision": _require_integer(
            receipt["occupancy_revision"],
            "occupancy_receipt.occupancy_revision",
            minimum=1,
            maximum=_U64_MAX,
        ),
        "source_timestamp_ns": _require_integer(
            receipt["source_timestamp_ns"],
            "occupancy_receipt.source_timestamp_ns",
            minimum=0,
            maximum=_U64_MAX,
        ),
        "backend_kind": receipt_backend_kind,
        "backend_generation": _require_integer(
            receipt["backend_generation"],
            "occupancy_receipt.backend_generation",
            minimum=1,
            maximum=_U64_MAX,
        ),
    }


def _validate_cloud_receipt(
    value: object,
    *,
    source_session_generation: int,
    boot_nonce_encoding: str,
) -> Dict[str, object]:
    receipt = _require_mapping(value, "cloud_receipt")
    fields = (
        "commit_id",
        "session_generation",
        "occupancy_revision",
        "cloud_revision",
        "source_timestamp_ns",
        "point_count",
    )
    _require_exact_fields(receipt, fields, "cloud_receipt")
    session_generation = _require_integer(
        receipt["session_generation"],
        "cloud_receipt.session_generation",
        minimum=1,
        maximum=_U64_MAX,
    )
    if session_generation != source_session_generation:
        raise MappingBackendCheckpointValidationError(
            "cloud_receipt.session_generation does not match source session"
        )
    return {
        "commit_id": _validate_commit_id(
            receipt["commit_id"], boot_nonce_encoding=boot_nonce_encoding
        ),
        "session_generation": session_generation,
        "occupancy_revision": _require_integer(
            receipt["occupancy_revision"],
            "cloud_receipt.occupancy_revision",
            minimum=1,
            maximum=_U64_MAX,
        ),
        "cloud_revision": _require_integer(
            receipt["cloud_revision"],
            "cloud_receipt.cloud_revision",
            minimum=1,
            maximum=_U64_MAX,
        ),
        "source_timestamp_ns": _require_integer(
            receipt["source_timestamp_ns"],
            "cloud_receipt.source_timestamp_ns",
            minimum=0,
            maximum=_U64_MAX,
        ),
        "point_count": _require_integer(
            receipt["point_count"],
            "cloud_receipt.point_count",
            minimum=1,
            maximum=_U32_MAX,
        ),
    }


def _validate_commit_id(
    value: object, *, boot_nonce_encoding: str
) -> Dict[str, object]:
    commit_id = _require_mapping(value, "commit_id")
    _require_exact_fields(commit_id, ("boot_nonce", "sequence"), "commit_id")
    boot_nonce_value = commit_id["boot_nonce"]
    if boot_nonce_encoding == "bytes":
        boot_nonce = _require_bytes(boot_nonce_value, "commit_id.boot_nonce")
        if len(boot_nonce) != 16:
            raise MappingBackendCheckpointValidationError(
                "commit_id.boot_nonce must contain exactly 16 bytes"
            )
    elif boot_nonce_encoding == "hex":
        if (
            not isinstance(boot_nonce_value, str)
            or len(boot_nonce_value) != 32
            or not re.fullmatch(r"[0-9a-f]{32}", boot_nonce_value)
        ):
            raise MappingBackendCheckpointValidationError(
                "persisted commit_id.boot_nonce must be 32 lowercase hex characters"
            )
        boot_nonce = bytes.fromhex(boot_nonce_value)
    else:
        raise AssertionError(f"unsupported boot nonce encoding: {boot_nonce_encoding}")
    if not any(boot_nonce):
        raise MappingBackendCheckpointValidationError(
            "commit_id.boot_nonce must not be all zero"
        )
    return {
        "boot_nonce": boot_nonce,
        "sequence": _require_integer(
            commit_id["sequence"],
            "commit_id.sequence",
            minimum=1,
            maximum=_U64_MAX,
        ),
    }


def _validate_receipt_order(
    occupancy_receipt: Mapping[str, object], cloud_receipt: Mapping[str, object]
) -> None:
    occupancy_commit = _require_mapping(
        occupancy_receipt["commit_id"], "occupancy_receipt.commit_id"
    )
    cloud_commit = _require_mapping(
        cloud_receipt["commit_id"], "cloud_receipt.commit_id"
    )
    if cloud_commit["boot_nonce"] != occupancy_commit["boot_nonce"]:
        raise MappingBackendCheckpointValidationError(
            "canonical occupancy and cloud receipts must share a boot nonce"
        )
    if int(cloud_commit["sequence"]) > int(occupancy_commit["sequence"]):
        raise MappingBackendCheckpointValidationError(
            "canonical cloud receipt commit sequence cannot lead occupancy"
        )
    if int(cloud_receipt["occupancy_revision"]) > int(
        occupancy_receipt["occupancy_revision"]
    ):
        raise MappingBackendCheckpointValidationError(
            "canonical cloud receipt occupancy_revision cannot lead occupancy"
        )
    same_sequence = int(cloud_commit["sequence"]) == int(
        occupancy_commit["sequence"]
    )
    same_occupancy_revision = int(cloud_receipt["occupancy_revision"]) == int(
        occupancy_receipt["occupancy_revision"]
    )
    if same_sequence != same_occupancy_revision:
        raise MappingBackendCheckpointValidationError(
            "canonical receipt sequence/revision ordering is inconsistent"
        )
    if same_sequence and int(cloud_receipt["source_timestamp_ns"]) != int(
        occupancy_receipt["source_timestamp_ns"]
    ):
        raise MappingBackendCheckpointValidationError(
            "canonical receipts with one commit id must share a timestamp"
        )


def _validate_occupancy_asset(
    value: object, *, sha256_encoding: str
) -> Dict[str, object]:
    asset = _require_mapping(value, "occupancy_asset")
    fields = (
        "raw_data_sha256",
        "raw_data_size",
        "width",
        "height",
        "resolution",
        "origin_x",
        "origin_y",
        "frame_id",
        "source_occupancy_revision",
        "timestamp_ns",
    )
    if sha256_encoding == "none":
        fields = tuple(key for key in fields if key != "raw_data_sha256")
    _require_exact_fields(asset, fields, "occupancy_asset")
    width = _require_integer(
        asset["width"], "occupancy_asset.width", minimum=1, maximum=_U32_MAX
    )
    height = _require_integer(
        asset["height"], "occupancy_asset.height", minimum=1, maximum=_U32_MAX
    )
    raw_data_size = _require_integer(
        asset["raw_data_size"],
        "occupancy_asset.raw_data_size",
        minimum=1,
        maximum=_U64_MAX,
    )
    expected_size = width * height
    if raw_data_size != expected_size:
        raise MappingBackendCheckpointValidationError(
            "occupancy_asset.raw_data_size must equal width * height"
        )
    return {
        **({"raw_data_sha256": _validate_asset_sha256(asset["raw_data_sha256"],
            "occupancy_asset.raw_data_sha256", encoding=sha256_encoding)} if sha256_encoding != "none" else {}),
        "raw_data_size": raw_data_size,
        "width": width,
        "height": height,
        "resolution": _require_finite_number(
            asset["resolution"], "occupancy_asset.resolution", positive=True
        ),
        "origin_x": _require_finite_number(
            asset["origin_x"], "occupancy_asset.origin_x"
        ),
        "origin_y": _require_finite_number(
            asset["origin_y"], "occupancy_asset.origin_y"
        ),
        "frame_id": _validate_frame_id(asset["frame_id"], "occupancy_asset.frame_id"),
        "source_occupancy_revision": _require_integer(
            asset["source_occupancy_revision"],
            "occupancy_asset.source_occupancy_revision",
            minimum=1,
            maximum=_U64_MAX,
        ),
        "timestamp_ns": _require_integer(
            asset["timestamp_ns"],
            "occupancy_asset.timestamp_ns",
            minimum=0,
            maximum=_U64_MAX,
        ),
    }


def _validate_cloud_asset(
    value: object, *, sha256_encoding: str
) -> Dict[str, object]:
    asset = _require_mapping(value, "cloud_asset")
    fields = (
        "xyz_sha256",
        "xyz_size",
        "frame_id",
        "source_cloud_revision",
        "point_count",
        "voxel_size",
        "payload_schema_version",
    )
    if sha256_encoding == "none":
        fields = tuple(key for key in fields if key != "xyz_sha256")
    _require_exact_fields(asset, fields, "cloud_asset")
    point_count = _require_integer(
        asset["point_count"],
        "cloud_asset.point_count",
        minimum=1,
        maximum=_U32_MAX,
    )
    xyz_size = _require_integer(
        asset["xyz_size"],
        "cloud_asset.xyz_size",
        minimum=0,
        maximum=_U64_MAX,
    )
    if xyz_size != point_count * 12:
        raise MappingBackendCheckpointValidationError(
            "cloud_asset.xyz_size must equal point_count * 12"
        )
    return {
        **({"xyz_sha256": _validate_asset_sha256(asset["xyz_sha256"],
            "cloud_asset.xyz_sha256", encoding=sha256_encoding)} if sha256_encoding != "none" else {}),
        "xyz_size": xyz_size,
        "frame_id": _validate_frame_id(asset["frame_id"], "cloud_asset.frame_id"),
        "source_cloud_revision": _require_integer(
            asset["source_cloud_revision"],
            "cloud_asset.source_cloud_revision",
            minimum=1,
            maximum=_U64_MAX,
        ),
        "point_count": point_count,
        "voxel_size": _require_finite_number(
            asset["voxel_size"],
            "cloud_asset.voxel_size",
            positive=True,
            maximum=MAX_CLOUD_MAP_VOXEL_SIZE,
        ),
        "payload_schema_version": _require_exact_integer(
            asset["payload_schema_version"],
            "cloud_asset.payload_schema_version",
            expected=2 if sha256_encoding == "none" else CLOUD_MAP_PAYLOAD_SCHEMA_VERSION,
        ),
    }


def _validate_optional_cloud(value, backend_kind, *, sha256_encoding):
    if backend_kind == "slam_toolbox":
        if value is not None:
            raise MappingBackendCheckpointValidationError("二维完整图不能携带三维资产")
        return None
    return _validate_cloud_asset(value, sha256_encoding=sha256_encoding)


def _validate_asset_bindings(
    checkpoint: MappingBackendCheckpoint,
    occupancy_asset: Mapping[str, object],
    cloud_asset: Optional[Mapping[str, object]],
) -> None:
    if int(occupancy_asset["source_occupancy_revision"]) != int(
        checkpoint.occupancy_receipt["occupancy_revision"]
    ):
        raise MappingBackendCheckpointValidationError(
            "occupancy asset revision does not match canonical occupancy receipt"
        )
    if int(occupancy_asset["timestamp_ns"]) != int(
        checkpoint.occupancy_receipt["source_timestamp_ns"]
    ):
        raise MappingBackendCheckpointValidationError(
            "occupancy asset timestamp does not match canonical occupancy receipt"
        )
    if checkpoint.backend_kind == "slam_toolbox":
        if cloud_asset is not None or checkpoint.cloud_receipt is not None:
            raise MappingBackendCheckpointValidationError("二维完整图不能携带三维资产")
        if checkpoint.schema_version == 1:
            if checkpoint.payload[:8] != b"NWS2D001":
                raise MappingBackendCheckpointValidationError("legacy planar envelope requires legacy archive structure")
            try:
                read_planar_structure(checkpoint.payload, session_id=checkpoint.source_session_id,
                    generation=checkpoint.source_session_generation,
                    timestamp_ns=checkpoint.occupancy_receipt["source_timestamp_ns"], occupancy=occupancy_asset)
            except (ValueError, TypeError, KeyError) as error:
                raise MappingBackendCheckpointValidationError(str(error)) from error
        return
    if int(cloud_asset["source_cloud_revision"]) != int(
        checkpoint.cloud_receipt["cloud_revision"]
    ):
        raise MappingBackendCheckpointValidationError(
            "cloud asset revision does not match canonical cloud receipt"
        )
    if int(cloud_asset["point_count"]) != int(
        checkpoint.cloud_receipt["point_count"]
    ):
        raise MappingBackendCheckpointValidationError(
            "cloud asset point_count does not match canonical cloud receipt"
        )
    if cloud_asset["frame_id"] != occupancy_asset["frame_id"]:
        raise MappingBackendCheckpointValidationError(
            "canonical occupancy and cloud assets must use the same frame_id"
        )
    if checkpoint.dense_archive is not None and read_dense_metadata(checkpoint.dense_archive)[0]["frame_id"] != cloud_asset["frame_id"]:
        raise MappingBackendCheckpointValidationError("完整点云与体素地图坐标系不一致")


def _asset_document(
    asset: Mapping[str, object], *, sha256_field: str
) -> Dict[str, object]:
    document = dict(asset)
    document[sha256_field] = _require_bytes(
        document[sha256_field], sha256_field
    ).hex()
    return document


def _validate_asset_sha256(value: object, name: str, *, encoding: str) -> bytes:
    if encoding == "bytes":
        digest = _require_bytes(value, name)
        if len(digest) != 32:
            raise MappingBackendCheckpointValidationError(
                f"{name} must contain exactly 32 bytes"
            )
        return digest
    if encoding == "hex":
        return bytes.fromhex(_validate_sha256_hex(value, name))
    raise AssertionError(f"unsupported asset SHA-256 encoding: {encoding}")


def _require_finite_number(
    value: object,
    name: str,
    *,
    positive: bool = False,
    maximum: Optional[float] = None,
) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise MappingBackendCheckpointValidationError(f"{name} must be a finite number")
    normalized = float(value)
    if (
        not math.isfinite(normalized)
        or (positive and normalized <= 0.0)
        or (maximum is not None and normalized > maximum)
    ):
        qualifier = "finite and positive" if positive else "finite"
        if maximum is not None:
            qualifier += f" and at most {maximum}"
        raise MappingBackendCheckpointValidationError(f"{name} must be {qualifier}")
    return normalized


def _require_positive_finite_duration(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise MappingBackendCheckpointValidationError(
            f"{name} must be a finite positive number"
        )
    normalized = float(value)
    if not math.isfinite(normalized) or normalized <= 0.0 or normalized > 3600.0:
        raise MappingBackendCheckpointValidationError(
            f"{name} must be in the range (0, 3600]"
        )
    return normalized


def _validate_frame_id(value: object, name: str) -> str:
    if not isinstance(value, str) or not value or value.strip() != value:
        raise MappingBackendCheckpointValidationError(
            f"{name} must be a non-empty, trimmed string"
        )
    if len(value) > 256 or any(ord(character) < 0x20 for character in value):
        raise MappingBackendCheckpointValidationError(
            f"{name} contains invalid characters or is too long"
        )
    return value


def _receipt_document(receipt: Mapping[str, object]) -> Dict[str, object]:
    document = dict(receipt)
    commit_id = dict(_require_mapping(document["commit_id"], "commit_id"))
    commit_id["boot_nonce"] = _require_bytes(
        commit_id["boot_nonce"], "commit_id.boot_nonce"
    ).hex()
    document["commit_id"] = commit_id
    return document


def _validate_manifest_reference(value: object, description: str) -> Tuple[str, str]:
    reference = _require_mapping(value, description)
    _require_exact_fields(reference, ("sha256", "path"), description)
    sha256 = _validate_sha256_hex(reference["sha256"], f"{description}.sha256")
    path = _require_string(reference["path"], f"{description}.path")
    expected_path = _manifest_relative_path(sha256)
    if path != expected_path:
        raise MappingBackendCheckpointValidationError(
            f"{description}.path is not canonical for its SHA-256"
        )
    return sha256, path


def _validate_payload_reference(value: object) -> Tuple[str, str, int]:
    reference = _require_mapping(value, "checkpoint manifest payload")
    _require_exact_fields(
        reference, ("sha256", "path", "size"), "checkpoint manifest payload"
    )
    sha256 = _validate_sha256_hex(reference["sha256"], "payload.sha256")
    path = _require_string(reference["path"], "payload.path")
    if path != _object_relative_path(sha256):
        raise MappingBackendCheckpointValidationError(
            "payload.path is not canonical for its SHA-256"
        )
    size = _require_integer(
        reference["size"],
        "payload.size",
        minimum=1,
        maximum=_U64_MAX,
    )
    return sha256, path, size


def _object_relative_path(sha256: str) -> str:
    return f"objects/sha256-{sha256}.bin"


def _manifest_relative_path(sha256: str) -> str:
    return f"manifests/sha256-{sha256}.json"


def _require_mapping(value: object, description: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise MappingBackendCheckpointValidationError(
            f"{description} must be a JSON object"
        )
    if not all(isinstance(key, str) for key in value):
        raise MappingBackendCheckpointValidationError(
            f"{description} field names must be strings"
        )
    return value


def _require_exact_fields(
    value: Mapping[str, object], fields: Sequence[str], description: str
) -> None:
    expected = set(fields)
    actual = set(value)
    if actual != expected:
        missing = sorted(expected - actual)
        unknown = sorted(actual - expected)
        raise MappingBackendCheckpointValidationError(
            f"{description} fields are invalid: missing={missing} unknown={unknown}"
        )


def _require_integer(
    value: object,
    name: str,
    *,
    minimum: int,
    maximum: int,
) -> int:
    if type(value) is not int or not minimum <= value <= maximum:
        raise MappingBackendCheckpointValidationError(
            f"{name} must be an integer in [{minimum}, {maximum}]"
        )
    return value


def _require_exact_integer(value: object, name: str, *, expected: int) -> int:
    normalized = _require_integer(
        value,
        name,
        minimum=expected,
        maximum=expected,
    )
    return normalized


def _require_string(value: object, name: str) -> str:
    if not isinstance(value, str):
        raise MappingBackendCheckpointValidationError(f"{name} must be a string")
    return value


def _require_bytes(value: object, name: str) -> bytes:
    if not isinstance(value, bytes):
        raise MappingBackendCheckpointValidationError(f"{name} must be bytes")
    return value


def _require_sha256_bytes(value: object) -> bytes:
    digest = _require_bytes(value, "content_sha256")
    if len(digest) != 32:
        raise MappingBackendCheckpointValidationError(
            "content_sha256 must contain exactly 32 bytes"
        )
    return digest


def _validate_sha256_hex(value: object, name: str) -> str:
    if not isinstance(value, str) or _SHA256_RE.fullmatch(value) is None:
        raise MappingBackendCheckpointValidationError(
            f"{name} must contain 64 lowercase hex characters"
        )
    return value


def _encode_json(document: Mapping[str, object]) -> bytes:
    return (
        json.dumps(
            document,
            ensure_ascii=True,
            allow_nan=False,
            sort_keys=True,
            separators=(",", ":"),
        )
        + "\n"
    ).encode("ascii")


def _parse_canonical_json(data: bytes, description: str) -> Dict[str, object]:
    def reject_duplicate_pairs(
        pairs: List[Tuple[str, object]],
    ) -> Dict[str, object]:
        result: Dict[str, object] = {}
        for key, value in pairs:
            if key in result:
                raise MappingBackendCheckpointValidationError(
                    f"{description} contains duplicate field {key!r}"
                )
            result[key] = value
        return result

    def reject_constant(value: str) -> object:
        raise MappingBackendCheckpointValidationError(
            f"{description} contains non-finite JSON number {value}"
        )

    try:
        decoded = data.decode("ascii")
        value = json.loads(
            decoded,
            object_pairs_hook=reject_duplicate_pairs,
            parse_constant=reject_constant,
        )
    except MappingBackendCheckpointValidationError:
        raise
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise MappingBackendCheckpointValidationError(
            f"{description} is not valid canonical JSON"
        ) from exc
    if not isinstance(value, dict):
        raise MappingBackendCheckpointValidationError(
            f"{description} must contain a JSON object"
        )
    if _encode_json(value) != data:
        raise MappingBackendCheckpointValidationError(
            f"{description} is not encoded as canonical JSON"
        )
    return value


def _read_regular_file_at(
    directory_fd: int,
    name: str,
    path: Path,
    *,
    maximum_bytes: int,
    description: str,
) -> bytes:
    _validate_path_component(name, description)
    flags = os.O_RDONLY | _no_follow_flag()
    if hasattr(os, "O_CLOEXEC"):
        flags |= os.O_CLOEXEC
    try:
        fd = os.open(name, flags, dir_fd=directory_fd)
    except OSError as exc:
        raise MappingBackendCheckpointValidationError(
            f"cannot open {description} as a regular non-symlink file: {path}"
        ) from exc
    try:
        metadata = os.fstat(fd)
        _validate_owned_regular_file(metadata, path)
        if metadata.st_size > maximum_bytes:
            raise MappingBackendCheckpointValidationError(
                f"{description} exceeds maximum size {maximum_bytes}: {metadata.st_size}"
            )
        with os.fdopen(fd, "rb", closefd=True) as stream:
            fd = -1
            if maximum_bytes > MAX_MAPPING_BACKEND_CHECKPOINT_BYTES:
                data = FrozenPayload(iter(lambda: stream.read(1024 * 1024), b""), maximum_bytes=maximum_bytes)
            else:
                data = stream.read(maximum_bytes + 1)
            after = os.fstat(stream.fileno())
            if (metadata.st_dev, metadata.st_ino, metadata.st_size, metadata.st_mtime_ns, metadata.st_ctime_ns) != (
                after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns, after.st_ctime_ns):
                raise MappingBackendCheckpointValidationError(f"{description} mutated while freezing")
        if len(data) != metadata.st_size or len(data) > maximum_bytes:
            raise MappingBackendCheckpointValidationError(
                f"{description} exceeds maximum size {maximum_bytes}"
            )
        return data
    finally:
        if fd >= 0:
            os.close(fd)


def _replace_regular_file_at(
    directory_fd: int,
    source_name: str,
    destination_name: str,
    destination_path: Path,
    *,
    expected_source: _FileIdentity,
) -> None:
    _validate_path_component(source_name, "checkpoint temporary filename")
    _validate_path_component(destination_name, "checkpoint final filename")
    try:
        source_metadata = os.stat(
            source_name,
            dir_fd=directory_fd,
            follow_symlinks=False,
        )
    except OSError as exc:
        raise MappingBackendCheckpointValidationError(
            f"checkpoint temporary file disappeared before replace: {source_name}"
        ) from exc
    if not stat.S_ISREG(source_metadata.st_mode):
        raise MappingBackendCheckpointValidationError(
            f"checkpoint temporary path is not a regular file: {source_name}"
        )
    _require_file_identity(
        source_metadata,
        expected_source,
        "checkpoint temporary file was substituted before replace",
    )
    try:
        metadata = os.stat(
            destination_name,
            dir_fd=directory_fd,
            follow_symlinks=False,
        )
    except FileNotFoundError:
        metadata = None
    if metadata is not None and (
        stat.S_ISLNK(metadata.st_mode) or not stat.S_ISREG(metadata.st_mode)
    ):
        raise MappingBackendCheckpointValidationError(
            f"refusing to replace non-regular or symlink path: {destination_path}"
        )
    os.replace(
        source_name,
        destination_name,
        src_dir_fd=directory_fd,
        dst_dir_fd=directory_fd,
    )


def _fsync_directory_fd(descriptor: int, path: Path) -> None:
    metadata = os.fstat(descriptor)
    if not stat.S_ISDIR(metadata.st_mode):
        raise MappingBackendCheckpointValidationError(
            f"cannot fsync non-directory checkpoint path: {path}"
        )
    os.fsync(descriptor)


def _remove_old_matching(
    directory_fd: int,
    directory_path: Path,
    *,
    cutoff_ns: int,
    name_matches: Callable[[str], bool],
    preserved_names: FrozenSet[str],
) -> Tuple[Path, ...]:
    removed: List[Path] = []
    for name in os.listdir(directory_fd):
        if name in preserved_names or not name_matches(name):
            continue
        try:
            metadata = os.stat(name, dir_fd=directory_fd, follow_symlinks=False)
        except FileNotFoundError:
            continue
        if metadata.st_mtime_ns >= cutoff_ns:
            continue
        if not (stat.S_ISREG(metadata.st_mode) or stat.S_ISLNK(metadata.st_mode)):
            continue
        try:
            os.unlink(name, dir_fd=directory_fd)
        except FileNotFoundError:
            continue
        removed.append(directory_path / name)
    return tuple(sorted(removed))


def _unlink_at_if_same(
    directory_fd: int, name: str, expected: _FileIdentity
) -> None:
    try:
        metadata = os.stat(name, dir_fd=directory_fd, follow_symlinks=False)
    except FileNotFoundError:
        return
    _require_file_identity(
        metadata,
        expected,
        "checkpoint temporary file was substituted before cleanup",
    )
    try:
        os.unlink(name, dir_fd=directory_fd)
    except FileNotFoundError:
        pass


def _require_file_identity(
    metadata: os.stat_result, expected: _FileIdentity, message: str
) -> None:
    if (metadata.st_dev, metadata.st_ino) != (expected.device, expected.inode):
        raise MappingBackendCheckpointValidationError(message)


def _inject_fault(fault_injector: Optional[FaultInjector], phase: str) -> None:
    if phase not in PUBLISH_FAULT_PHASES:
        raise AssertionError(f"unknown checkpoint publication phase: {phase}")
    if fault_injector is not None:
        fault_injector(phase)


def _no_follow_flag() -> int:
    if not hasattr(os, "O_NOFOLLOW"):
        raise MappingBackendCheckpointValidationError(
            "secure checkpoint storage requires O_NOFOLLOW"
        )
    return os.O_NOFOLLOW


__all__ = [
    "ASSIGNED_CHECKPOINT_SCHEMA_VERSION",
    "CheckpointArtifactReference",
    "CheckpointCleanupResult",
    "CheckpointMemberIdentity",
    "DEFAULT_ORPHAN_GRACE_SECONDS",
    "MAPPING_BACKEND_CHECKPOINT_SCHEMA_VERSION",
    "MAX_MAPPING_BACKEND_CHECKPOINT_BYTES",
    "MappingBackendCheckpoint",
    "MappingBackendCheckpointNotFoundError",
    "MappingBackendCheckpointStore",
    "MappingBackendCheckpointStoreError",
    "MappingBackendCheckpointValidationError",
    "PUBLISH_FAULT_PHASES",
    "SUPPORTED_MAPPING_BACKEND_KINDS",
    "StoredMappingBackendCheckpoint",
    "validate_artifact_reference",
]
