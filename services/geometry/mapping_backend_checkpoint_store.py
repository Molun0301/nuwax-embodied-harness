"""Crash-consistent storage for geometry mapping-backend checkpoints."""

from __future__ import annotations

from services.geometry.checkpoint_payload import FrozenPayload, payload_digest, payload_chunks, MAX_PLANAR_ARCHIVE_BYTES
from services.geometry.fine_map_archive import MAX_FINE_ARCHIVE_BYTES, validate_fine_archive

import fcntl
import hashlib
import json
import math
import os
import re
import stat
import time
import uuid
from dataclasses import dataclass
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
    re.compile(r"\.tmp-checkpoint-v1-[0-9a-f]{32}\.json\Z"),
)
_OBJECT_NAME_RE = re.compile(r"sha256-([0-9a-f]{64})\.bin\Z")
_MANIFEST_NAME_RE = re.compile(r"sha256-([0-9a-f]{64})\.json\Z")
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


@dataclass(frozen=True)
class StoredMappingBackendCheckpoint:
    checkpoint: MappingBackendCheckpoint
    occupancy_asset: Mapping[str, object]
    cloud_asset: Optional[Mapping[str, object]]
    selector_path: Path
    manifest_sha256: str
    manifest_path: Path
    object_path: Path


@dataclass(frozen=True)
class CheckpointCleanupResult:
    removed_temp_files: Tuple[Path, ...]
    removed_manifests: Tuple[Path, ...]
    removed_objects: Tuple[Path, ...]


FaultInjector = Callable[[str], None]


class MappingBackendCheckpointStore:
    """Persist one current content-addressed checkpoint per backend and schema."""

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

        validator = cls(Path.cwd(), max_payload_bytes=max_payload_bytes)
        validated_checkpoint = validator._validate_checkpoint(checkpoint)
        validated_occupancy_asset = _validate_occupancy_asset(
            occupancy_asset,
            sha256_encoding="bytes",
        )
        validated_cloud_asset = _validate_optional_cloud(cloud_asset, validated_checkpoint.backend_kind, sha256_encoding="bytes")
        _validate_asset_bindings(
            validated_checkpoint,
            validated_occupancy_asset,
            validated_cloud_asset,
        )
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

        validated = self._validate_checkpoint(checkpoint)
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
                    self._publish_fine_archive(access, validated.fine_archive, fault_injector)

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
    ) -> StoredMappingBackendCheckpoint:
        """Load only through the backend's stable selector."""

        kind = _validate_backend_kind(backend_kind)
        schema = _validate_schema_version(schema_version)
        access = self._existing_layout(kind, schema, exclusive=False)
        try:
            stored = self._load_optional(kind, schema, access=access)
            if stored is None:
                raise MappingBackendCheckpointNotFoundError(
                    "mapping backend checkpoint selector does not exist: "
                    f"{access.paths.selector}"
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
            current = self._load_optional(kind, schema, access=access)
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

    def _publish_fine_archive(self, access, payload, fault_injector):
        """精细块资产先于引用它的清单落盘，失败只留下可回收的无引用对象。"""
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
                payload, temp_identity=identity, maximum_bytes=MAX_FINE_ARCHIVE_BYTES, description="精细地图资产")
            name = None
            _fsync_directory_fd(access.objects_fd, access.paths.objects)
        finally:
            if name is not None and identity is not None:
                _unlink_at_if_same(access.objects_fd, name, identity)

    def _validate_checkpoint(
        self, checkpoint: MappingBackendCheckpoint
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
        if not isinstance(payload, bytes) and not (backend_kind == "slam_toolbox" and isinstance(payload, FrozenPayload)):
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
        content_sha256 = _require_sha256_bytes(checkpoint.content_sha256)
        actual_sha256 = payload_digest(payload)
        if actual_sha256 != content_sha256:
            raise MappingBackendCheckpointValidationError(
                "content_sha256 does not match checkpoint payload"
            )
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
        if checkpoint.fine_archive is not None:
            if backend_kind == "slam_toolbox":
                raise MappingBackendCheckpointValidationError("二维检查点不能携带三维精细资产")
            try:
                validate_fine_archive(checkpoint.fine_archive, session=source_session_id,
                    generation=source_session_generation, cloud_revision=cloud_receipt["cloud_revision"],
                    source_timestamp_ns=cloud_receipt["source_timestamp_ns"])
            except (ValueError, TypeError) as error:
                raise MappingBackendCheckpointValidationError(str(error)) from error
        return MappingBackendCheckpoint(
            schema_version=schema_version,
            backend_kind=backend_kind,
            source_session_id=source_session_id,
            source_session_generation=source_session_generation,
            content_sha256=content_sha256,
            payload_size=payload_size,
            created_at_ns=created_at_ns,
            payload=payload,
            occupancy_receipt=occupancy_receipt,
            cloud_receipt=cloud_receipt,
            fine_archive=checkpoint.fine_archive,
        )

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
        return document

    def _load_optional(
        self,
        backend_kind: str,
        schema_version: int,
        *,
        access: "_StoreAccess",
    ) -> Optional[StoredMappingBackendCheckpoint]:
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
        if hashlib.sha256(manifest_bytes).hexdigest() != manifest_sha256:
            raise MappingBackendCheckpointValidationError(
                "checkpoint manifest SHA-256 does not match selector"
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
            ) + (("fine_archive",) if "fine_archive" in manifest else ()),
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
        content_sha256 = payload_digest(payload)
        if content_sha256.hex() != payload_sha256:
            raise MappingBackendCheckpointValidationError(
                "checkpoint object SHA-256 does not match manifest"
            )
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
            if len(fine_archive) != fine_size or payload_digest(fine_archive).hex() != fine_sha:
                raise MappingBackendCheckpointValidationError("精细地图资产长度或摘要与清单不符")
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
        referenced_manifests: Set[str] = set()
        referenced_objects: Set[str] = set()
        if current is not None:
            referenced_manifests.add(current.manifest_path.name)
            referenced_objects.add(current.object_path.name)
            if current.checkpoint.fine_archive is not None:
                referenced_objects.add(f"sha256-{payload_digest(current.checkpoint.fine_archive).hex()}.bin")

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
            name_matches=lambda name: _OBJECT_NAME_RE.fullmatch(name) is not None,
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


def _validate_schema_version(value: object) -> int:
    schema_version = _require_integer(
        value,
        "schema_version",
        minimum=1,
        maximum=_U32_MAX,
    )
    if schema_version != MAPPING_BACKEND_CHECKPOINT_SCHEMA_VERSION:
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
        "raw_data_sha256": _validate_asset_sha256(
            asset["raw_data_sha256"],
            "occupancy_asset.raw_data_sha256",
            encoding=sha256_encoding,
        ),
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
        "xyz_sha256": _validate_asset_sha256(
            asset["xyz_sha256"],
            "cloud_asset.xyz_sha256",
            encoding=sha256_encoding,
        ),
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
            expected=CLOUD_MAP_PAYLOAD_SCHEMA_VERSION,
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
        if not stat.S_ISREG(metadata.st_mode):
            raise MappingBackendCheckpointValidationError(
                f"{description} must be a regular file: {path}"
            )
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
        if len(data) > maximum_bytes:
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
    "CheckpointCleanupResult",
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
]
