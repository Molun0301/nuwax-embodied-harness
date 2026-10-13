from __future__ import annotations

import ctypes
import errno
import hashlib
import json
import os
import shlex
import shutil
import stat
import uuid
from dataclasses import dataclass, replace
from pathlib import Path, PurePosixPath
from typing import TYPE_CHECKING, Callable, Dict, Iterable, List, Mapping, Optional, Tuple
from urllib.parse import quote

from contracts.map_workspace import (
    MAP_GEOREFERENCE_RELATIVE_PATH,
    AssetIdentityV2,
    MapAssetBundle,
    MapGeometryAssetBinding,
    MapGeometryAssetType,
    MapGeoreferenceBindingV1,
    MapGeoreferenceV1,
)
from contracts.naming import build_safe_dir_segment

if TYPE_CHECKING:
    from services.geometry.mapping_backend_checkpoint_store import CheckpointMemberIdentity

_BUNDLE_FILE_NAME = "bundle.json"
_MANIFEST_FILE_NAME = "manifest.json"
_MANIFEST_SCHEMA_VERSION = 1
_STAGING_PREFIX = ".current.tmp-"
_PLANAR_EXCHANGE_JOURNAL = ".planar-exchange-v1.json"
_ASSIGNED_EXCHANGE_JOURNAL = ".assigned-exchange-v1.json"
_MAX_GEOREFERENCE_JSON_BYTES = 64 * 1024
_MAX_MANIFEST_JSON_BYTES = 4 * 1024 * 1024
_MAX_BUNDLE_JSON_BYTES = 64 * 1024 * 1024
_MAX_LEGACY_METADATA_JSON_BYTES = 4 * 1024 * 1024
_MAX_LEGACY_VERSION_JSON_BYTES = 64 * 1024 * 1024
_MAX_CHECKPOINT_JSON_BYTES = 4 * 1024 * 1024
_MAX_NAV2_YAML_BYTES = 4 * 1024 * 1024
_MAX_MANIFEST_FILE_ENTRIES = 8192
_MAX_DIRECTORY_ENTRIES = _MAX_MANIFEST_FILE_ENTRIES + 128
_MAX_JSON_NUMBER_TOKEN_BYTES = 128
_FILE_READ_CHUNK_BYTES = 1024 * 1024
_FILE_PREFIX_BYTES = 8192
_VALIDATED_SNAPSHOT_DOMAIN = b"nuwax.mapping.validated-map-asset-snapshot.v1\0"


class MapAssetBundleError(RuntimeError):
    """Base error for finalized map asset storage."""


class MapAssetBundleValidationError(MapAssetBundleError):
    """A finalized or candidate bundle failed integrity validation."""


@dataclass(frozen=True, init=False)
class _AssignedActivationRead:
    """One process-local stable read owns both metadata and retained body bytes."""

    _repository: object
    bundle: MapAssetBundle
    root: Path
    stored_checkpoint: object

    def __init__(self, *args, **kwargs):
        raise TypeError("activation reads must come from the asset repository")

    def __reduce_ex__(self, protocol):
        raise TypeError("activation reads cannot be serialized or copied")

    def require_owner(self, repository, root):
        if repository is not self._repository or Path(root).absolute() != self.root:
            raise MapAssetBundleValidationError("activation read belongs to another repository/root")
        return self.bundle, self.stored_checkpoint


class MapAssetCommitUncertainError(MapAssetBundleError):
    """目录交换尚未完成持久对账；暂存目录仍由交换日志拥有。"""
    preserve_staging = True


class MapAlreadyFinalizedError(MapAssetBundleError):
    """A map already has a different immutable current bundle."""

    def __init__(self, map_name: str, existing_sha256: str, candidate_sha256: str) -> None:
        self.map_name = map_name
        self.existing_sha256 = existing_sha256
        self.candidate_sha256 = candidate_sha256
        super().__init__("地图 %s 已 finalized；修改地图必须使用新的 map_name。" % map_name)


class AssignedMapAlreadyFinalizedError(MapAlreadyFinalizedError):
    """Assigned publication conflict; IDs never occupy legacy digest fields."""

    def __init__(self, map_name: str, existing_identity: Optional[AssetIdentityV2],
                 candidate_identity: AssetIdentityV2, *, existing_kind: str = "assigned") -> None:
        self.map_name = map_name
        self.existing_asset_identity = existing_identity
        self.candidate_asset_identity = candidate_identity
        self.existing_identity_kind = existing_kind
        MapAssetBundleError.__init__(self, "Map %s already owns a different finalized %s asset" % (map_name, existing_kind))


class MapAssetMigrationError(MapAssetBundleError):
    """Legacy version storage cannot be migrated without guessing."""


@dataclass(frozen=True)
class ValidatedMapAssetBundleSnapshot:
    """一次稳定目录读取中验证完成的资产；无地理锚点时参考原文为空字节。"""

    bundle: MapAssetBundle
    georeference_bytes: bytes
    repository_device: int
    repository_inode: int
    repository_fingerprint: str
    repository_observations: tuple = ()
    assigned_source_member: Optional[CheckpointMemberIdentity] = None


@dataclass(frozen=True)
class AssignedMapAssetExchange:
    """Retained predecessor proof; catalog adoption must precede settlement."""

    previous_identity: AssetIdentityV2
    candidate_identity: AssetIdentityV2
    committed: bool
    catalog_created_at: str
    map_directory: Tuple[int, int]


class MapAssetBundleRepository:
    """Store one immutable, integrity-checked ``current/`` bundle per map name."""

    def __init__(
        self,
        root_dir: str,
        *,
        fault_injector: Optional[Callable[[str], None]] = None,
    ) -> None:
        self._root_dir = Path(root_dir).expanduser()
        self._root_dir.mkdir(parents=True, exist_ok=True)
        self._fault_injector = fault_injector
        self._staging_owners: Dict[Path, Tuple[int, int]] = {}

    def create_staging_directory(self, map_name: str) -> Path:
        map_dir = self._map_dir(map_name)
        map_dir.mkdir(parents=True, exist_ok=True)
        self._require_real_directory(map_dir, "地图目录")
        staging_dir = map_dir / (_STAGING_PREFIX + uuid.uuid4().hex)
        staging_dir.mkdir(mode=0o700)
        owned = staging_dir.stat()
        self._staging_owners[staging_dir.absolute()] = (owned.st_dev, owned.st_ino)
        _fsync_directory(map_dir)
        return staging_dir

    def discard_staging_directory(self, map_name: str, staging_dir: Path) -> None:
        resolved = self._require_staging_directory(map_name, staging_dir, must_exist=False)
        owner = self._staging_owners.get(resolved)
        if not resolved.exists() and not resolved.is_symlink():
            self._staging_owners.pop(resolved, None)
            return
        if owner is None:
            raise MapAssetBundleValidationError("staging cleanup requires its original directory owner")
        self._discard_owned_staging(map_name, resolved, owner)

    def _discard_owned_staging(self, map_name, staging_dir, owner):
        resolved = self._require_staging_directory(map_name, staging_dir, must_exist=False)
        flags = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC
        parent_fd = os.open(resolved.parent, flags)
        staging_fd = cleanup_fd = -1
        cleanup_name = ".current.cleanup-" + uuid.uuid4().hex
        cleanup_created = False
        retired = False
        try:
            try:
                staging_fd = os.open(resolved.name, flags, dir_fd=parent_fd)
            except FileNotFoundError:
                self._staging_owners.pop(resolved, None)
                return
            except OSError as exc:
                raise MapAssetBundleValidationError("staging directory was replaced; refusing unowned cleanup") from exc
            opened = os.fstat(staging_fd)
            if (opened.st_dev, opened.st_ino) != tuple(owner):
                raise MapAssetBundleValidationError("staging directory was replaced; refusing unowned cleanup")
            self._inject_fault("before_staging_quarantine")
            os.mkdir(cleanup_name, mode=0o700, dir_fd=parent_fd)
            cleanup_created = True
            cleanup_fd = os.open(cleanup_name, flags, dir_fd=parent_fd)
            # Move first into an exclusively owned namespace. If somebody replaced
            # the public staging name after open, preserve/restore that entry;
            # not one byte is deleted until the moved inode matches our owner.
            _rename_directory_noreplace(parent_fd, resolved.name, "retired",
                target_parent_descriptor=cleanup_fd)
            retired = True
            moved = os.stat("retired", dir_fd=cleanup_fd, follow_symlinks=False)
            if not stat.S_ISDIR(moved.st_mode) or (moved.st_dev, moved.st_ino) != tuple(owner):
                try:
                    _rename_directory_noreplace(cleanup_fd, "retired", resolved.name,
                        target_parent_descriptor=parent_fd)
                    retired = False
                except OSError:
                    # A new public entry appeared meanwhile. Both it and the
                    # unowned quarantine entry must survive for reconciliation.
                    pass
                raise MapAssetBundleValidationError("staging directory was replaced; refusing unowned cleanup")
            os.fsync(parent_fd)
            os.fsync(cleanup_fd)
            self._inject_fault("after_staging_quarantine")
            # The public staging path is no longer a deletion target. Walk the
            # originally pinned directory, never a newly opened replacement.
            _remove_pinned_directory_contents(staging_fd)
            named = os.stat("retired", dir_fd=cleanup_fd, follow_symlinks=False)
            _require_same_file_identity(opened, named, "retired staging")
            os.rmdir("retired", dir_fd=cleanup_fd)
            retired = False
            self._staging_owners.pop(resolved, None)
            os.fsync(cleanup_fd)
        finally:
            if staging_fd >= 0:
                os.close(staging_fd)
            if cleanup_fd >= 0:
                os.close(cleanup_fd)
            if cleanup_created and not retired:
                os.rmdir(cleanup_name, dir_fd=parent_fd)
                os.fsync(parent_fd)
            os.close(parent_fd)

    def get_map_asset_bundle(
        self,
        map_name: str,
        *,
        migrate_legacy: bool = True,
        expected_asset_identity: Optional[AssetIdentityV2] = None,
        _for_activation: bool = False,
    ) -> Optional[MapAssetBundle]:
        _require_typed_asset_identity(expected_asset_identity, optional=True)
        self._recover_planar_exchange(map_name)
        current_dir = self._current_dir(map_name, expected_asset_identity=expected_asset_identity)
        if current_dir.exists() or current_dir.is_symlink():
            return self._load_validated_bundle(map_name, current_dir, expected_asset_identity=expected_asset_identity,
                _for_activation=_for_activation)
        if expected_asset_identity is not None or not migrate_legacy:
            return None
        return self._migrate_legacy_latest(map_name)

    def _find_legacy_cloud_import(self, map_name, provenance):
        """Discover only small published-version documents; admit one exact owner later."""
        matches = []
        for path in self._map_dir(map_name).iterdir():
            if not path.name.startswith("assigned-id-") or path.is_symlink() or not path.is_dir():
                continue
            try:
                manifest = _parse_json_object(_read_regular_file(path / _MANIFEST_FILE_NAME,
                    "import manifest", maximum_bytes=_MAX_MANIFEST_JSON_BYTES), "import manifest")
                if (manifest.get("schema_version") != 2 or manifest.get("map_name") != map_name
                        or manifest.get("legacy_cloud_import_source") != provenance):
                    continue
                identity = AssetIdentityV2.model_validate(manifest.get("asset_identity"))
                if (manifest.get("asset_identity") != identity.model_dump()
                        or path.name != "assigned-id-" + identity.asset_id):
                    continue
                matches.append(identity)
            except (OSError, ValueError, MapAssetBundleError):
                # Unrelated/incomplete siblings never gate legacy activation.
                continue
        return matches

    def import_legacy_cloud_bundle(self, map_name, *, catalog_created_at, map_directory_owner):
        """Explicit activation import of eligible schema1 current; old files stay untouched.

        Caller holds the existing executor guard and commits catalog ownership.
        None retains legacy activation for unsupported or ambiguous sources.
        """
        self._recover_planar_exchange(map_name)
        current = self._map_dir(map_name) / "current"
        if not current.exists():
            return None
        from services.geometry.mapping_backend_checkpoint_store import (
            MappingBackendCheckpointStore, assign_legacy_cloud_checkpoint)
        with _StableDirectoryTree(current) as tree:
            manifest = _parse_json_object(tree.require_file(_MANIFEST_FILE_NAME, "legacy manifest")
                .read(_MAX_MANIFEST_JSON_BYTES), "legacy manifest")
            if manifest.get("schema_version") != 1:
                return None
            document = _parse_json_object(tree.require_file(_BUNDLE_FILE_NAME, "legacy bundle")
                .read(_MAX_BUNDLE_JSON_BYTES), "legacy bundle")
            bundle = MapAssetBundle.model_validate(document)
            geometry = bundle.geometry_asset
            source = geometry.metadata.get("mapping_backend_checkpoint") if geometry is not None else None
            if (bundle.identity_schema_version != 1 or geometry is None or bundle.snapshot is None
                    or bundle.snapshot.occupancy_grid is None or not geometry.pcd_path
                    or not isinstance(source, dict) or source.get("present") is not True
                    or source.get("schema_version") != 1 or source.get("backend_kind") not in {"voxel", "raytracing"}):
                return None
            if bundle.map_name != map_name:
                raise MapAssetBundleValidationError("legacy cloud map name differs")
            entries = manifest.get("files", [])
            expected = {entry["path"]: entry["size"] for entry in entries}
            if len(expected) != len(entries) or set(tree.files) != set(expected) | {_MANIFEST_FILE_NAME}:
                raise MapAssetBundleValidationError("legacy cloud inventory differs")
            for relative, size in expected.items():
                if tree.require_file(relative, "legacy member").size() != size:
                    raise MapAssetBundleValidationError("legacy cloud member size differs")
            checkpoint_manifest = _parse_json_object(tree.require_file(source["manifest_path"], "checkpoint manifest")
                .read(_MAX_CHECKPOINT_JSON_BYTES), "checkpoint manifest")
            receipts = checkpoint_manifest.get("canonical_receipts")
            if (checkpoint_manifest.get("fine_archive") is not None or not isinstance(receipts, dict)
                    or not receipts.get("occupancy") or not receipts.get("cloud")):
                return None
            raw_catalog = _parse_json_object(_read_regular_file(self._map_dir(map_name) / "metadata.json",
                "legacy catalog", maximum_bytes=_MAX_LEGACY_METADATA_JSON_BYTES), "legacy catalog")
            root_state = os.fstat(tree.root.descriptor)
            provenance = {"map_name": map_name, "source_reference": {"kind": "current"},
                "catalog_created_at": catalog_created_at,
                "map_directory_owner": list(map_directory_owner),
                "source_directory_owner": [root_state.st_dev, root_state.st_ino],
                "stored_manifest_alias": raw_catalog.get("asset_manifest_sha256"),
                "stored_checkpoint_alias": source.get("manifest_sha256"),
                "files": [list(item) for item in _assigned_observations(tree)]}
            matches = self._find_legacy_cloud_import(map_name, provenance)
            if len(matches) > 1:
                return None
            if matches:
                selected = self.get_map_asset_bundle(map_name, migrate_legacy=False,
                    expected_asset_identity=matches[0], _for_activation=True)
                if (type(selected) is not _AssignedActivationRead
                        or selected.bundle.metadata.get("legacy_cloud_import_source") != provenance):
                    raise MapAssetBundleValidationError("published cloud import source changed")
                tree.verify_all()
                return selected
            stored = MappingBackendCheckpointStore(current).load(source["backend_kind"], schema_version=1)
            if stored.checkpoint.fine_archive is not None:
                return None
            if (stored.manifest_path.relative_to(current).as_posix() != source["manifest_path"]
                    or stored.manifest_sha256 != source.get("manifest_sha256")
                    or stored.checkpoint.content_sha256.hex() != source.get("object_sha256")):
                raise MapAssetBundleValidationError("legacy cloud checkpoint references differ")
            grid = bundle.snapshot.occupancy_grid
            occupancy = stored.occupancy_asset
            import struct
            import numpy as np
            from services.geometry.pointcloud_asset_stream import open_pointcloud_chunks
            if (grid.width != occupancy["width"] or grid.height != occupancy["height"]
                    or struct.pack("<f", grid.resolution_m) != struct.pack("<f", occupancy["resolution"])
                    or grid.origin.position.x != occupancy["origin_x"] or grid.origin.position.y != occupancy["origin_y"]
                    or grid.frame_id.strip().lstrip("/") != occupancy["frame_id"]):
                return None  # Original legacy activation retains its own admission decision.
            if geometry.frame_id != occupancy["frame_id"] or bundle.frame_id != occupancy["frame_id"]:
                return None  # Assigned frame contract is stricter; preserve legacy activation.
            from datetime import datetime, timezone
            timestamp = grid.timestamp
            if not isinstance(timestamp, datetime):
                return None  # Original legacy activation retains its own admission decision.
            timestamp = timestamp.replace(tzinfo=timezone.utc) if timestamp.tzinfo is None else timestamp.astimezone(timezone.utc)
            delta = timestamp - datetime(1970, 1, 1, tzinfo=timezone.utc)
            timestamp_ns = delta.days * 86400000000000 + delta.seconds * 1000000000 + delta.microseconds * 1000
            if abs(timestamp_ns - occupancy["timestamp_ns"]) >= 1000:
                return None  # Original legacy activation retains its own admission decision.
            cells = bytes(value if value >= 0 else value + 256 for value in grid.data)
            managed = tree.require_file(geometry.pcd_path, "canonical legacy PCD")
            managed.verify_named_entry()
            from services.geometry.checkpoint_payload import CHUNK_BYTES, FrozenPayload
            with open_pointcloud_chunks(Path("/proc/self/fd/%d" % managed.descriptor), format_hint=".pcd") as (header, chunks):
                xyz_size = header.points * 12
                if xyz_size <= CHUNK_BYTES:
                    xyz = b"".join(np.asarray(points, dtype="<f4", order="C").tobytes()
                        for points in chunks)
                else:
                    def xyz_chunks():
                        for points in chunks:
                            points = np.asarray(points, dtype="<f4", order="C")
                            for start in range(0, len(points), CHUNK_BYTES // 12):
                                yield points[start:start + CHUNK_BYTES // 12].tobytes()
                    xyz = FrozenPayload(xyz_chunks(), maximum_bytes=xyz_size)
            managed.verify_named_entry()
            if (header.points > 0 and header.points * 12 != len(xyz)
                    or stored.cloud_asset["frame_id"] != occupancy["frame_id"]):
                return None  # Original legacy activation retains its own admission decision.
            if len(cells) != occupancy["raw_data_size"] or len(xyz) != stored.cloud_asset["xyz_size"]:
                return None  # Original legacy activation retains its own admission decision.
            # These IDs are allocated for a genuine new immutable import owner,
            # only after published-version reuse and source preparation.
            identity = AssetIdentityV2(asset_id=uuid.uuid4().hex)
            checkpoint, occupancy, cloud = assign_legacy_cloud_checkpoint(stored,
                canonical_occupancy_data=cells, canonical_cloud_xyz=xyz,
                checkpoint_id=uuid.uuid4().hex,
                dense_artifact_id=uuid.uuid4().hex if stored.checkpoint.dense_archive is not None else None)
            staging = self.create_staging_directory(map_name)
            try:
                assigned = MappingBackendCheckpointStore(staging).publish(checkpoint,
                    occupancy_asset=occupancy, cloud_asset=cloud)
                assigned_source = {"present": True, "schema_version": 2, "backend_kind": checkpoint.backend_kind,
                    "checkpoint_id": checkpoint.checkpoint_id,
                    "manifest_path": assigned.manifest_path.relative_to(staging).as_posix()}
                for relative, member in tree.files.items():
                    if (relative in {_BUNDLE_FILE_NAME, _MANIFEST_FILE_NAME}
                            or relative.startswith("mapping_backend/") or relative.endswith("/geometry_asset.json")):
                        continue
                    target = _contained_path(staging, relative)
                    target.parent.mkdir(parents=True, exist_ok=True)
                    with target.open("xb") as output:
                        for offset in range(0, member.size(), _FILE_READ_CHUNK_BYTES):
                            output.write(member.slice(offset, offset + _FILE_READ_CHUNK_BYTES))
                geometry = geometry.model_copy(update={"metadata": {
                    **dict(geometry.metadata), "mapping_backend_checkpoint": assigned_source}})
                georeference = bundle.georeference
                if georeference is not None:
                    georeference = georeference.model_copy(update={"raw_sha256": None})
                values = {key: getattr(bundle, key) for key in type(bundle).model_fields}
                values.update(identity_schema_version=2, asset_identity=identity, asset_state="candidate",
                    manifest_sha256=None, geometry_asset=geometry, georeference=georeference,
                    metadata={**dict(bundle.metadata), "mapping_backend_checkpoint": assigned_source,
                        "legacy_cloud_import_source": provenance})
                candidate = MapAssetBundle(**values)
                tree.verify_all()
                self.finalize_assigned_map_asset_bundle(candidate, staging_dir=staging, asset_identity=identity,
                    _independent_import=True)
            except BaseException:
                if staging.exists():
                    self.discard_staging_directory(map_name, staging)
                raise
        return self.get_map_asset_bundle(map_name, migrate_legacy=False,
            expected_asset_identity=identity, _for_activation=True)

    def import_legacy_planar_bundle(self, map_name):
        """Import the committed legacy current once; leave its files untouched.

        The caller commits the returned assigned identity using the existing
        catalog owner transaction. Unselected versions are never guessed.
        """
        current = self._map_dir(map_name) / "current"
        if not current.exists():
            return None
        from services.geometry.mapping_backend_checkpoint_store import (
            MappingBackendCheckpointStore, assign_legacy_planar_checkpoint)
        with _StableDirectoryTree(current) as tree:
            manifest = _parse_json_object(tree.require_file(_MANIFEST_FILE_NAME, "legacy manifest")
                .read(_MAX_MANIFEST_JSON_BYTES), "legacy manifest")
            if manifest.get("schema_version") != 1:
                return None
            document = _parse_json_object(tree.require_file(_BUNDLE_FILE_NAME, "legacy bundle")
                .read(_MAX_BUNDLE_JSON_BYTES), "legacy bundle")
            geometry = document.get("geometry_asset")
            if not isinstance(geometry, dict) or geometry.get("metadata", {}).get("target_kind") != "planar_graph":
                return None
            bundle = MapAssetBundle.model_validate(document)
            if bundle.map_name != map_name:
                raise MapAssetBundleValidationError("legacy planar map name differs")
            # Historical digest strings are aliases only. Retain inventory and
            # regular-file ownership/size checks without content hashing.
            entries = manifest.get("files", [])
            expected = {entry["path"]: entry["size"] for entry in entries}
            if set(tree.files) != set(expected) | {_MANIFEST_FILE_NAME}:
                raise MapAssetBundleValidationError("legacy planar inventory differs")
            for relative, size in expected.items():
                if tree.require_file(relative, "legacy member").size() != size:
                    raise MapAssetBundleValidationError("legacy planar member size differs")
            stored = MappingBackendCheckpointStore(current).load("slam_toolbox", schema_version=1)
            checkpoint, occupancy, cloud = assign_legacy_planar_checkpoint(stored,
                checkpoint_id=uuid.uuid4().hex, artifact_id=uuid.uuid4().hex)
            identity = AssetIdentityV2(asset_id=uuid.uuid4().hex)
            staging = self.create_staging_directory(map_name)
            try:
                assigned = MappingBackendCheckpointStore(staging).publish(checkpoint,
                    occupancy_asset=occupancy, cloud_asset=cloud)
                object_path = assigned.object_path.relative_to(staging).as_posix()
                source = {"present": True, "schema_version": 2, "backend_kind": "slam_toolbox",
                    "checkpoint_id": checkpoint.checkpoint_id,
                    "manifest_path": assigned.manifest_path.relative_to(staging).as_posix()}
                old_graph = bundle.geometry_asset.mapping_database_path
                for relative, managed in tree.files.items():
                    if (relative in {_BUNDLE_FILE_NAME, _MANIFEST_FILE_NAME, old_graph}
                            or relative.startswith("mapping_backend/") or relative.endswith("/geometry_asset.json")):
                        continue
                    target = _contained_path(staging, relative)
                    target.parent.mkdir(parents=True, exist_ok=True)
                    with target.open("xb") as output:
                        for offset in range(0, managed.size(), _FILE_READ_CHUNK_BYTES):
                            output.write(managed.slice(offset, offset + _FILE_READ_CHUNK_BYTES))
                asset_geometry = bundle.geometry_asset
                assets = [asset.model_copy(update={"path": object_path})
                    if asset.asset_type == MapGeometryAssetType.PLANAR_GRAPH else asset
                    for asset in asset_geometry.assets]
                asset_geometry = asset_geometry.model_copy(update={"mapping_database_path": object_path,
                    "assets": assets, "metadata": {**dict(asset_geometry.metadata), "mapping_backend_checkpoint": source}})
                georeference = bundle.georeference
                if georeference is not None:
                    georeference = georeference.model_copy(update={"raw_sha256": None})
                values = {key: getattr(bundle, key) for key in type(bundle).model_fields}
                values.update(identity_schema_version=2, asset_identity=identity, asset_state="candidate",
                    manifest_sha256=None, geometry_asset=asset_geometry, georeference=georeference,
                    metadata={**dict(bundle.metadata), "mapping_backend_checkpoint": source})
                candidate = MapAssetBundle(**values)
                tree.verify_all()
                return self.finalize_assigned_map_asset_bundle(candidate,
                    staging_dir=staging, asset_identity=identity)
            except BaseException:
                if staging.exists():
                    self.discard_staging_directory(map_name, staging)
                raise

    def get_validated_map_asset_bundle_snapshot(
        self,
        map_name: str,
        *,
        expected_manifest_sha256: Optional[str] = None,
        expected_asset_identity: Optional[AssetIdentityV2] = None,
        _previous: Optional[ValidatedMapAssetBundleSnapshot] = None,
    ) -> Optional[ValidatedMapAssetBundleSnapshot]:
        """只读加载 modern current，并在同一稳定目录视图中返回地理参考原文。"""

        _require_typed_asset_identity(expected_asset_identity, optional=True)
        if expected_manifest_sha256 is not None and expected_asset_identity is not None:
            raise MapAssetBundleValidationError("mixed asset identity expectations are forbidden")
        if (self._map_dir(map_name) / _PLANAR_EXCHANGE_JOURNAL).exists():
            raise MapAssetBundleValidationError("二维资产交换尚未完成持久对账")
        # 列表请求只保留上一份已验证快照；每次复用仍核对当前目录和成员。
        if (_previous is not None and _previous.bundle.map_name == map_name
                and _previous.bundle.asset_identity == expected_asset_identity
                and _previous.bundle.manifest_sha256 == expected_manifest_sha256
                and self.map_asset_bundle_snapshot_is_current(map_name, _previous)):
            return _previous
        current_dir = self._current_dir(map_name, expected_asset_identity=expected_asset_identity)
        if not current_dir.exists() and not current_dir.is_symlink():
            return None
        with _StableDirectoryTree(current_dir) as tree:
            manifest_sha256, bundle = self._validate_open_directory(
                map_name,
                tree,
                expected_manifest_sha256=expected_manifest_sha256,
                expected_asset_identity=expected_asset_identity,
            )
            if bundle.identity_schema_version == 2:
                georeference_bytes = (tree.require_file(bundle.georeference.relative_path, "georeference")
                    .read(_MAX_GEOREFERENCE_JSON_BYTES) if bundle.georeference is not None else b"")
                source = self._assigned_checkpoint_source(tree, bundle.geometry_asset)
                checkpoint, _occupancy, _cloud, _members = self._validate_assigned_source_document(source)
                tree.verify_all()
                repository_stat = os.fstat(tree.root.descriptor)
                return ValidatedMapAssetBundleSnapshot(bundle, georeference_bytes,
                    repository_stat.st_dev, repository_stat.st_ino, "", _assigned_observations(tree),
                    checkpoint.graph_reference.member_identity if checkpoint.backend_kind == "slam_toolbox"
                    else checkpoint.cloud_identity)
            georeference_bytes = b""
            if bundle.georeference is not None:
                georeference_bytes, _size, georeference_sha256 = tree.require_file(
                    bundle.georeference.relative_path,
                    "地理参考文件",
                ).read_and_hash(_MAX_GEOREFERENCE_JSON_BYTES)
                if georeference_sha256 != bundle.georeference.raw_sha256:
                    raise MapAssetBundleValidationError("地理参考原文摘要在稳定读取期间发生变化。")
            tree.verify_all()
            if tree.root is None:
                raise MapAssetBundleValidationError("地图资产目录描述符已关闭。")
            repository_stat = os.fstat(tree.root.descriptor)
            validated_bundle = bundle.model_copy(
                update={"manifest_sha256": manifest_sha256},
                deep=True,
            )
            repository_fingerprint = _validated_snapshot_fingerprint(
                tree,
                validated_bundle,
                georeference_bytes,
            )
            repository_observations = _assigned_observations(tree)
        return ValidatedMapAssetBundleSnapshot(
            bundle=validated_bundle,
            georeference_bytes=georeference_bytes,
            repository_device=repository_stat.st_dev,
            repository_inode=repository_stat.st_ino,
            repository_fingerprint=repository_fingerprint,
            repository_observations=repository_observations,
        )

    def validate_map_asset_bundle_snapshot_identity(
        self,
        map_name: str,
        snapshot: ValidatedMapAssetBundleSnapshot,
    ) -> None:
        """从可信 current 命名项重建快照，并拒绝解析期间的任何变化。"""

        assigned = snapshot.bundle.identity_schema_version == 2
        refreshed = self.get_validated_map_asset_bundle_snapshot(map_name,
            expected_manifest_sha256=None if assigned else str(snapshot.bundle.manifest_sha256 or ""),
            expected_asset_identity=snapshot.bundle.asset_identity if assigned else None)
        if assigned:
            same = (refreshed is not None and refreshed.repository_device == snapshot.repository_device
                and refreshed.repository_inode == snapshot.repository_inode
                and refreshed.repository_observations == snapshot.repository_observations
                and refreshed.georeference_bytes == snapshot.georeference_bytes)
        else:
            same = refreshed is not None and refreshed == snapshot
        if not same:
            raise MapAssetBundleValidationError("finalized 地图 repository snapshot 已被替换。")

    def map_asset_bundle_snapshot_is_current(self, map_name, snapshot) -> bool:
        """Check an admitted snapshot using only bounded inode metadata."""
        if snapshot.bundle.map_name != map_name or not snapshot.repository_observations:
            return False
        try:
            with _StableDirectoryTree(self._current_dir(map_name)) as tree:
                current = os.fstat(tree.root.descriptor)
                return ((current.st_dev, current.st_ino) ==
                    (snapshot.repository_device, snapshot.repository_inode)
                    and _assigned_observations(tree) == snapshot.repository_observations)
        except (OSError, MapAssetBundleValidationError):
            return False

    def finalize_map_asset_bundle(
        self,
        bundle: MapAssetBundle,
        *,
        staging_dir: Path,
        expected_previous_manifest_sha256: Optional[str] = None,
    ) -> MapAssetBundle:
        if bundle.identity_schema_version != 1:
            raise MapAssetBundleValidationError("assigned candidates require the explicit assigned finalize API")
        if expected_previous_manifest_sha256 is not None:
            return self._replace_planar_bundle(bundle, staging_dir=staging_dir, expected_previous_manifest_sha256=expected_previous_manifest_sha256)
        map_name = str(bundle.map_name or "").strip()
        staging = self._require_staging_directory(map_name, staging_dir, must_exist=True)
        current = self._current_dir(map_name)
        try:
            self._write_candidate_bundle(bundle, staging)
            candidate_manifest = self._write_manifest(map_name, staging)
            candidate_sha256 = _sha256_bytes(candidate_manifest)
            resolve_existing = False
            committed_bundle: Optional[MapAssetBundle] = None
            with _StableDirectoryTree(staging) as candidate_tree:
                manifest_sha256, validated_bundle = self._validate_open_directory(
                    map_name,
                    candidate_tree,
                    expected_manifest_sha256=candidate_sha256,
                )
                _fsync_tree(candidate_tree)
                candidate_tree.verify_all()
                self._inject_fault("before_rename")
                if _entry_exists_at(candidate_tree.parent_descriptor, current.name):
                    resolve_existing = True
                else:
                    try:
                        _rename_directory_noreplace(
                            candidate_tree.parent_descriptor,
                            staging.name,
                            current.name,
                        )
                    except OSError as exc:
                        if exc.errno not in {errno.EEXIST, errno.ENOTEMPTY}:
                            raise
                        resolve_existing = True
                    else:
                        candidate_tree.rebind_root_after_rename(current.name)
                        os.fsync(candidate_tree.parent_descriptor)
                        committed_bundle = validated_bundle.model_copy(
                            update={"manifest_sha256": manifest_sha256},
                            deep=True,
                        )
                        self._inject_fault("after_rename")
            if resolve_existing:
                return self._resolve_existing_current(
                    map_name,
                    current,
                    staging,
                    candidate_sha256,
                )
            if committed_bundle is None:
                raise MapAssetBundleValidationError("地图资产目录提交结果不确定。")
            return committed_bundle
        except Exception:
            if staging.exists() or staging.is_symlink():
                self.discard_staging_directory(map_name, staging)
            raise

    def finalize_assigned_map_asset_bundle(
        self,
        bundle: MapAssetBundle,
        *,
        staging_dir: Path,
        asset_identity: AssetIdentityV2,
        expected_previous_identity: Optional[AssetIdentityV2] = None,
        _independent_import: bool = False,
    ) -> MapAssetBundle:
        """Publish an explicitly assigned owner. Legacy/default writers stay v1.

        IDs are supplied once by the candidate owner. Planar retry checks its
        retained metadata/inventory without comparing graph bodies. Legacy
        imports publish an independent version; the catalog selects it later.
        """
        _require_typed_asset_identity(asset_identity)
        _require_typed_asset_identity(expected_previous_identity, optional=True)
        if (bundle.identity_schema_version != 2 or bundle.asset_identity != asset_identity
                or bundle.asset_state not in {"candidate", "finalized"}):
            raise MapAssetBundleValidationError("candidate does not own the exact assigned asset identity")
        map_name = bundle.map_name
        independent = _independent_import or (expected_previous_identity is not None
            and self.has_assigned_version(map_name, expected_previous_identity))
        if expected_previous_identity is not None and not independent:
            return self._replace_assigned_cloud_bundle(bundle, staging_dir=staging_dir,
                asset_identity=asset_identity, expected_previous_identity=expected_previous_identity)
        staging = self._require_staging_directory(map_name, staging_dir, must_exist=True)
        owner = self._staging_owners[staging]
        current = self._assigned_version_dir(map_name, asset_identity) if independent else self._current_dir(map_name)
        if not independent and current.exists():
            with _StableDirectoryTree(current) as tree:
                header = _parse_json_object(tree.require_file(_MANIFEST_FILE_NAME, "current manifest")
                    .read(_MAX_MANIFEST_JSON_BYTES), "current manifest")
                independent = header.get("schema_version") == 1
        if independent:
            current = self._assigned_version_dir(map_name, asset_identity)
        try:
            finalized = bundle.model_copy(update={"asset_state": "finalized"})
            # Validate the source generation/descriptors before any outer body
            # serialization. Checkpoint JSON is small and body reads are deferred.
            with _StableDirectoryTree(staging) as tree:
                source = self._assigned_checkpoint_source(tree, finalized.geometry_asset)
                tree.verify_all()
            self._write_assigned_candidate_bundle(finalized, staging)
            self._write_assigned_manifest(finalized, staging, source,
                import_source=finalized.metadata["legacy_cloud_import_source"] if _independent_import else None)
            with _StableDirectoryTree(staging) as candidate_tree:
                _unused, validated = self._validate_open_directory(map_name, candidate_tree,
                    expected_asset_identity=asset_identity)
                _fsync_tree(candidate_tree)
                candidate_tree.verify_all()
                self._inject_fault("before_rename")
                try:
                    _rename_directory_noreplace(candidate_tree.parent_descriptor, staging.name, current.name)
                except OSError as exc:
                    if exc.errno not in {errno.EEXIST, errno.ENOTEMPTY}:
                        raise
                    with _StableDirectoryTree(current) as previous_tree:
                        # An occupied legacy namespace is an explicit conflict,
                        # not permission to enter its hashing/identity reader.
                        prior_manifest = _parse_json_object(previous_tree.require_file(
                            _MANIFEST_FILE_NAME, "current manifest").read(_MAX_MANIFEST_JSON_BYTES), "current manifest")
                        version = prior_manifest.get("schema_version")
                        if type(version) is not int or version not in (1, 2):
                            raise MapAssetBundleValidationError("unsupported occupied asset manifest version")
                        if version == 1:
                            raise AssignedMapAlreadyFinalizedError(map_name, None, asset_identity, existing_kind="legacy")
                        try:
                            prior_identity = AssetIdentityV2.model_validate(prior_manifest.get("asset_identity"))
                        except Exception as error:
                            raise MapAssetBundleValidationError("occupied assigned asset identity is invalid") from error
                        if prior_identity != asset_identity:
                            raise AssignedMapAlreadyFinalizedError(map_name, prior_identity, asset_identity)
                        _unused, existing = self._validate_open_directory(map_name, previous_tree,
                            expected_asset_identity=asset_identity)
                        if set(previous_tree.files) != set(candidate_tree.files):
                            raise MapAssetBundleValidationError("assigned asset ID reused with different inventory")
                        if source != self._assigned_checkpoint_source(previous_tree, existing.geometry_asset):
                            raise MapAssetBundleValidationError("assigned asset ID reused with different checkpoint source")
                        if source["manifest"]["backend_kind"] != "slam_toolbox":
                            for relative, candidate_file in candidate_tree.files.items():
                                if not candidate_file.streaming_equals(previous_tree.files[relative]):
                                    raise MapAssetBundleValidationError("assigned asset ID reused with different body: %s" % relative)
                        previous_tree.verify_all()
                        candidate_tree.verify_all()
                        return existing
                candidate_tree.rebind_root_after_rename(current.name)
                os.fsync(candidate_tree.parent_descriptor)
                self._inject_fault("after_rename")
                return validated
        finally:
            # A post-rename failure leaves the durable current owner discoverable
            # to restart/catalog reconciliation. Only this original staging inode
            # may be removed; a replacement directory belongs to somebody else.
            self._discard_owned_staging(map_name, staging, owner)

    @staticmethod
    def _assigned_catalog_owner(asset):
        if asset is None or asset.identity_schema_version != 2 or not asset.finalized:
            raise MapAssetBundleValidationError("assigned replacement requires its finalized catalog owner")
        return (asset.map_name, asset.created_at.isoformat(), asset.asset_identity)

    @staticmethod
    def _assigned_directory_owner(path):
        metadata = path.lstat()
        if not stat.S_ISDIR(metadata.st_mode):
            raise MapAssetBundleValidationError("assigned exchange directory was replaced")
        return [metadata.st_dev, metadata.st_ino]

    def _read_assigned_exchange(self, map_name):
        path = self._map_dir(map_name) / _ASSIGNED_EXCHANGE_JOURNAL
        if not path.exists() and not path.is_symlink():
            return None
        record = _parse_json_object(_read_regular_file(path, "assigned exchange journal",
            maximum_bytes=8192), "assigned exchange journal")
        fields = {"schema_version", "map_name", "previous_identity", "candidate_identity", "staging",
            "previous_directory", "candidate_directory", "map_directory", "catalog_created_at"}
        if (set(record) != fields or type(record["schema_version"]) is not int
                or record["schema_version"] != 1 or record["map_name"] != map_name
                or not isinstance(record["catalog_created_at"], str)):
            raise MapAssetBundleValidationError("invalid assigned exchange journal")
        for name in ("previous_directory", "candidate_directory", "map_directory"):
            value = record[name]
            if not isinstance(value, list) or len(value) != 2 or any(type(v) is not int or v < 0 for v in value):
                raise MapAssetBundleValidationError("invalid assigned exchange directory owner")
        try:
            previous = AssetIdentityV2.model_validate(record["previous_identity"])
            candidate = AssetIdentityV2.model_validate(record["candidate_identity"])
        except Exception as exc:
            raise MapAssetBundleValidationError("invalid assigned exchange identity") from exc
        if previous == candidate or record["previous_directory"] == record["candidate_directory"]:
            raise MapAssetBundleValidationError("assigned replacement requires distinct owners")
        staging = self._require_staging_directory(map_name, path.parent / str(record["staging"]), must_exist=False)
        if staging.name != record["staging"]:
            raise MapAssetBundleValidationError("invalid assigned exchange staging name")
        return record

    def _inspect_assigned_exchange(self, map_name, record, catalog):
        map_dir = self._map_dir(map_name)
        if self._assigned_directory_owner(map_dir) != record["map_directory"]:
            raise MapAssetBundleValidationError("assigned exchange map creation directory changed")
        asset = catalog.get_map_asset(map_name)
        owner = self._assigned_catalog_owner(asset)
        if owner[:2] != (map_name, record["catalog_created_at"]):
            raise MapAssetBundleValidationError("assigned exchange catalog creation owner changed")
        previous = AssetIdentityV2.model_validate(record["previous_identity"])
        candidate = AssetIdentityV2.model_validate(record["candidate_identity"])
        current = self._current_dir(map_name)
        current_owner = self._assigned_directory_owner(current)
        committed = current_owner == record["candidate_directory"]
        if current_owner != record["candidate_directory" if committed else "previous_directory"]:
            raise MapAssetBundleValidationError("assigned exchange current directory changed")
        identity = candidate if committed else previous
        manifest = _parse_json_object(_read_regular_file(current / _MANIFEST_FILE_NAME,
            "assigned exchange current manifest", maximum_bytes=_MAX_MANIFEST_JSON_BYTES),
            "assigned exchange current manifest")
        if (type(manifest.get("schema_version")) is not int or manifest.get("schema_version") != 2
                or manifest.get("map_name") != map_name
                or manifest.get("asset_identity") != identity.model_dump()
                or self._assigned_directory_owner(current) != current_owner):
            raise MapAssetBundleValidationError("assigned exchange current identity changed")
        if owner[2] not in ((previous, candidate) if committed else (previous,)):
            raise MapAssetBundleValidationError("assigned exchange catalog asset owner changed")
        staging = map_dir / record["staging"]
        if staging.exists() or staging.is_symlink():
            retired = record["previous_directory" if committed else "candidate_directory"]
            if self._assigned_directory_owner(staging) != retired:
                raise MapAssetBundleValidationError("assigned exchange staging directory was replaced; refusing unowned cleanup")
        return AssignedMapAssetExchange(previous, candidate, committed, record["catalog_created_at"],
            tuple(record["map_directory"]))

    def get_assigned_map_asset_exchange(self, map_name) -> Optional[AssignedMapAssetExchange]:
        """Read exact predecessor proof without clearing it or relaxing getters."""
        from services.mapping.catalog import MapCatalogRepository
        catalog = MapCatalogRepository(str(self._root_dir))
        with catalog.map_activation_guard():
            record = self._read_assigned_exchange(map_name)
            return None if record is None else self._inspect_assigned_exchange(map_name, record, catalog)

    def settle_assigned_map_asset_exchange(self, map_name, *, expected_previous_identity, asset_identity):
        """Release a committed/aborted exchange only after its catalog is durable."""
        from services.mapping.catalog import MapCatalogRepository
        _require_typed_asset_identity(expected_previous_identity)
        _require_typed_asset_identity(asset_identity)
        catalog = MapCatalogRepository(str(self._root_dir))
        map_dir = self._map_dir(map_name)
        with catalog.map_activation_guard():
            record = self._read_assigned_exchange(map_name)
            if record is None:
                return
            proof = self._inspect_assigned_exchange(map_name, record, catalog)
            if (proof.previous_identity != expected_previous_identity or proof.candidate_identity != asset_identity):
                raise MapAssetBundleValidationError("assigned exchange settlement identity changed")
            wanted = proof.candidate_identity if proof.committed else proof.previous_identity
            if catalog.get_map_asset(map_name).asset_identity != wanted:
                raise MapAssetBundleValidationError("assigned exchange catalog adoption is not complete")
            # A previous metadata rename may have succeeded before fsync failed.
            descriptor, _size = _open_regular_file(map_dir / "metadata.json", "assigned catalog metadata")
            try:
                os.fsync(descriptor)
            finally:
                os.close(descriptor)
            _fsync_directory(map_dir)
        staging = map_dir / record["staging"]
        retired = record["previous_directory" if proof.committed else "candidate_directory"]
        self._discard_owned_staging(map_name, staging, retired)
        with catalog.map_activation_guard():
            latest = self._read_assigned_exchange(map_name)
            if latest is None:
                return
            if latest != record or self._inspect_assigned_exchange(map_name, latest, catalog) != proof:
                raise MapAssetBundleValidationError("assigned exchange changed during settlement")
            if catalog.get_map_asset(map_name).asset_identity != wanted:
                raise MapAssetBundleValidationError("assigned exchange catalog changed during settlement")
            journal = map_dir / _ASSIGNED_EXCHANGE_JOURNAL
            tombstone = map_dir / (".assigned-exchange-cleared-" + uuid.uuid4().hex)
            try:
                self._inject_fault("before_assigned_journal_clear")
                os.replace(journal, tombstone)
                _fsync_directory(map_dir)
            except BaseException:
                # Preserve the only predecessor proof if journal removal did not
                # durably finish. link never overwrites a newly owned journal.
                try:
                    if not journal.exists() and tombstone.exists():
                        os.link(tombstone, journal, follow_symlinks=False)
                        tombstone.unlink()
                        _fsync_directory(map_dir)
                except BaseException:  # noqa: BLE001, S110 - retain the original failed durable-clear result
                    pass
                raise
        tombstone.unlink(missing_ok=True)
        _fsync_directory(map_dir)

    def _replace_assigned_cloud_bundle(self, bundle, *, staging_dir, asset_identity, expected_previous_identity):
        from services.mapping.catalog import MapCatalogRepository
        map_name = bundle.map_name
        staging = self._require_staging_directory(map_name, staging_dir, must_exist=True)
        owner = self._staging_owners[staging]
        current = self._current_dir(map_name)
        catalog = MapCatalogRepository(str(self._root_dir))
        preserve_staging = False
        try:
            captured = self._assigned_catalog_owner(catalog.get_map_asset(map_name))
            map_owner = self._assigned_directory_owner(current.parent)
            if captured[2] not in (expected_previous_identity, asset_identity):
                raise AssignedMapAlreadyFinalizedError(map_name, captured[2], asset_identity)
            pending = self._read_assigned_exchange(map_name)
            if pending is not None and pending["staging"] == staging.name:
                preserve_staging = True
                raise MapAssetCommitUncertainError("retained assigned exchange must be reconciled before reusing its staging")
            finalized = bundle.model_copy(update={"asset_state": "finalized"})
            with _StableDirectoryTree(staging) as tree:
                source = self._assigned_checkpoint_source(tree, finalized.geometry_asset)
                tree.verify_all()
            self._write_assigned_candidate_bundle(finalized, staging)
            self._write_assigned_manifest(finalized, staging, source)
            with _StableDirectoryTree(staging) as candidate_tree, _StableDirectoryTree(current) as previous_tree:
                _unused, validated = self._validate_open_directory(map_name, candidate_tree,
                    expected_asset_identity=asset_identity)
                header = _parse_json_object(previous_tree.require_file(_MANIFEST_FILE_NAME,
                    "assigned previous manifest").read(_MAX_MANIFEST_JSON_BYTES), "assigned previous manifest")
                if type(header.get("schema_version")) is int and header["schema_version"] == 1:
                    raise AssignedMapAlreadyFinalizedError(map_name, None, asset_identity, existing_kind="legacy")
                try:
                    old_identity = AssetIdentityV2.model_validate(header.get("asset_identity"))
                except Exception as exc:
                    raise MapAssetBundleValidationError("invalid assigned previous identity") from exc
                if old_identity not in (expected_previous_identity, asset_identity):
                    raise AssignedMapAlreadyFinalizedError(map_name, old_identity, asset_identity)
                if old_identity != asset_identity and captured[2] != expected_previous_identity:
                    raise MapAssetBundleValidationError("assigned predecessor differs from the catalog owner")
                _unused, previous = self._validate_open_directory(map_name, previous_tree,
                    expected_asset_identity=old_identity)
                # Both source generations must be explicitly admitted cloud-v2.
                old_source = self._assigned_checkpoint_source(previous_tree, previous.geometry_asset)
                if old_identity == asset_identity:
                    if set(previous_tree.files) != set(candidate_tree.files):
                        raise MapAssetBundleValidationError("assigned asset ID reused with different inventory")
                    if source != old_source:
                        raise MapAssetBundleValidationError("assigned asset ID reused with different checkpoint source")
                    if source["manifest"]["backend_kind"] != "slam_toolbox":
                        for relative, candidate_file in candidate_tree.files.items():
                            if not candidate_file.streaming_equals(previous_tree.files[relative]):
                                raise MapAssetBundleValidationError(f"assigned asset ID reused with different body: {relative}")
                elif (asset_identity == expected_previous_identity
                        or source["manifest"]["checkpoint_id"] == old_source["manifest"]["checkpoint_id"]):
                    raise MapAssetBundleValidationError("assigned replacement requires a fresh asset and checkpoint owner")
                _fsync_tree(candidate_tree)
                candidate_tree.verify_all()
                previous_tree.verify_all()
                with catalog.map_activation_guard():
                    if (self._assigned_catalog_owner(catalog.get_map_asset(map_name)) != captured
                            or self._assigned_directory_owner(current.parent) != map_owner
                            or [os.fstat(candidate_tree.parent_descriptor).st_dev,
                                os.fstat(candidate_tree.parent_descriptor).st_ino] != map_owner
                            or [os.fstat(previous_tree.parent_descriptor).st_dev,
                                os.fstat(previous_tree.parent_descriptor).st_ino] != map_owner):
                        raise MapAssetBundleValidationError("assigned replacement catalog creation owner changed")
                    if catalog.get_map_activation() is not None or catalog.get_relocalization_intent() is not None:
                        raise MapAssetBundleValidationError("activation or relocalization fences assigned exchange")
                    candidate_tree.verify_all()
                    previous_tree.verify_all()
                    pending = self._read_assigned_exchange(map_name)
                    if pending is not None:
                        proof = self._inspect_assigned_exchange(map_name, pending, catalog)
                        if (not proof.committed or proof.previous_identity != expected_previous_identity
                                or proof.candidate_identity != asset_identity or old_identity != asset_identity):
                            raise MapAssetCommitUncertainError("another assigned exchange requires catalog reconciliation")
                    if old_identity == asset_identity:
                        return previous
                    self._inject_fault("before_assigned_exchange")
                    candidate_tree.verify_all()
                    previous_tree.verify_all()
                    if (self._assigned_catalog_owner(catalog.get_map_asset(map_name)) != captured
                            or self._assigned_directory_owner(current.parent) != map_owner):
                        raise MapAssetBundleValidationError("assigned replacement catalog creation owner changed")
                    old_stat = os.fstat(previous_tree.root.descriptor)
                    new_stat = os.fstat(candidate_tree.root.descriptor)
                    record = {"schema_version": 1, "map_name": map_name,
                        "previous_identity": expected_previous_identity.model_dump(),
                        "candidate_identity": asset_identity.model_dump(), "staging": staging.name,
                        "previous_directory": [old_stat.st_dev, old_stat.st_ino],
                        "candidate_directory": [new_stat.st_dev, new_stat.st_ino],
                        "map_directory": map_owner, "catalog_created_at": captured[1]}
                    preserve_staging = True
                    _atomic_write_relative(current.parent, _ASSIGNED_EXCHANGE_JOURNAL, _canonical_json_bytes(record))
                    self._inject_fault("after_assigned_exchange_journal")
                    candidate_tree.verify_all()
                    previous_tree.verify_all()
                    if (self._assigned_catalog_owner(catalog.get_map_asset(map_name)) != captured
                            or self._assigned_directory_owner(current.parent) != map_owner):
                        raise MapAssetBundleValidationError("assigned replacement catalog creation owner changed")
                    if catalog.get_map_activation() is not None or catalog.get_relocalization_intent() is not None:
                        raise MapAssetBundleValidationError("activation or relocalization fences assigned exchange")
                    _exchange_directories(candidate_tree.parent_descriptor, staging.name, current.name)
                    candidate_tree.rebind_root_after_rename(current.name)
                    previous_tree.rebind_root_after_rename(staging.name)
                    self._inject_fault("before_assigned_directory_fsync")
                    os.fsync(candidate_tree.parent_descriptor)
                    self._inject_fault("after_assigned_exchange")
                return validated
        except BaseException as exc:
            if preserve_staging:
                if not isinstance(exc, Exception):
                    exc.preserve_staging = True
                    raise
                raise MapAssetCommitUncertainError(f"assigned exchange requires exact journal reconciliation: {exc}") from exc
            raise
        finally:
            if not preserve_staging:
                self._discard_owned_staging(map_name, staging, owner)

    def _write_assigned_candidate_bundle(self, bundle, staging):
        if bundle.georeference is not None:
            relative = bundle.georeference.relative_path
            path = _contained_path(staging, relative)
            if not path.exists() and not path.is_symlink():
                _atomic_write_relative(staging, relative, _model_json_bytes(bundle.georeference.contract),
                    create_parents=True)
        geometry = bundle.geometry_asset
        if geometry is None:
            raise MapAssetBundleValidationError("assigned bundle requires geometry_asset")
        relative = PurePosixPath("geometry", build_safe_dir_segment(geometry.geometry_stack), "geometry_asset.json").as_posix()
        _atomic_write_relative(staging, relative, _model_json_bytes(geometry), create_parents=True)
        _atomic_write_relative(staging, _BUNDLE_FILE_NAME, _model_json_bytes(bundle))

    def _write_assigned_manifest(self, bundle, directory, source, *, import_source=None):
        with _StableDirectoryTree(directory) as tree:
            entries = [{"path": path, "size": managed.size()}
                       for path, managed in sorted(tree.files.items()) if path != _MANIFEST_FILE_NAME]
            tree.verify_all()
        document = {"schema_version": 2, "map_name": bundle.map_name, "bundle_path": _BUNDLE_FILE_NAME,
            "asset_identity": bundle.asset_identity.model_dump(mode="json"),
            "checkpoint_source": source, "files": entries}
        if import_source is not None:
            document["legacy_cloud_import_source"] = import_source
        payload = _canonical_json_bytes(document)
        if len(entries) > _MAX_MANIFEST_FILE_ENTRIES or len(payload) > _MAX_MANIFEST_JSON_BYTES:
            raise MapAssetBundleValidationError("assigned manifest exceeds its budget")
        _atomic_write_relative(directory, _MANIFEST_FILE_NAME, payload)

    def _validate_assigned_open_directory(self, map_name, tree, manifest, *, expected_asset_identity=None,
                                          _for_activation=False):
        fields = {"schema_version", "map_name", "bundle_path", "asset_identity", "checkpoint_source", "files"}
        if "legacy_cloud_import_source" in manifest:
            fields.add("legacy_cloud_import_source")
        if set(manifest) != fields:
            raise MapAssetBundleValidationError("assigned manifest has missing or mixed-generation fields")
        if manifest["map_name"] != map_name or manifest["bundle_path"] != _BUNDLE_FILE_NAME:
            raise MapAssetBundleValidationError("assigned manifest map/path mismatch")
        try:
            identity = AssetIdentityV2.model_validate(manifest["asset_identity"])
        except Exception as exc:
            raise MapAssetBundleValidationError("invalid assigned asset identity") from exc
        if expected_asset_identity is not None and identity != expected_asset_identity:
            raise MapAssetBundleValidationError("assigned asset differs from the expected owner")
        entries = manifest["files"]
        if not isinstance(entries, list) or len(entries) > _MAX_MANIFEST_FILE_ENTRIES:
            raise MapAssetBundleValidationError("invalid assigned file inventory")
        files = {}
        for entry in entries:
            if not isinstance(entry, dict) or set(entry) != {"path", "size"}:
                raise MapAssetBundleValidationError("assigned file descriptor has missing or legacy fields")
            relative = _validate_relative_path(entry["path"], "assigned member path")
            if relative != entry["path"] or relative in files or relative == _MANIFEST_FILE_NAME:
                raise MapAssetBundleValidationError("assigned inventory contains duplicate/noncanonical path")
            if type(entry["size"]) is not int or entry["size"] < 0:
                raise MapAssetBundleValidationError("assigned member size is invalid")
            files[relative] = entry["size"]
        if set(tree.files) != set(files) | {_MANIFEST_FILE_NAME} or _BUNDLE_FILE_NAME not in files:
            raise MapAssetBundleValidationError("assigned manifest inventory differs from directory")
        # Decode/validate all source identities before reading any payload body.
        self._validate_assigned_source_document(manifest["checkpoint_source"])
        for path, size in files.items():
            if tree.files[path].size() != size:
                raise MapAssetBundleValidationError("assigned member size differs: %s" % path)
        document = _parse_json_object(tree.require_file(_BUNDLE_FILE_NAME, "assigned bundle").read(_MAX_BUNDLE_JSON_BYTES), "assigned bundle")
        if (type(document.get("identity_schema_version")) is not int or document["identity_schema_version"] != 2
                or document.get("asset_identity") != manifest["asset_identity"]
                or document.get("asset_state") != "finalized" or document.get("manifest_sha256") is not None):
            raise MapAssetBundleValidationError("assigned bundle has missing or mixed-generation identity")
        try:
            bundle = MapAssetBundle.model_validate(document)
        except Exception as exc:
            raise MapAssetBundleValidationError("invalid assigned bundle contract") from exc
        if bundle.map_name != map_name:
            raise MapAssetBundleValidationError("assigned bundle map mismatch")
        if ("legacy_cloud_import_source" in manifest
                and (not isinstance(manifest["legacy_cloud_import_source"], dict)
                    or bundle.metadata.get("legacy_cloud_import_source") != manifest["legacy_cloud_import_source"])):
            raise MapAssetBundleValidationError("assigned import provenance differs from its bundle")
        source = self._assigned_checkpoint_source(tree, bundle.geometry_asset)
        if source != manifest["checkpoint_source"]:
            raise MapAssetBundleValidationError("assigned checkpoint source/member descriptors changed")
        geometry = bundle.geometry_asset
        geometry_path = PurePosixPath("geometry", build_safe_dir_segment(geometry.geometry_stack), "geometry_asset.json").as_posix()
        observed_geometry = _parse_json_object(tree.require_file(geometry_path, "geometry descriptor").read(_MAX_LEGACY_METADATA_JSON_BYTES), "geometry descriptor")
        if observed_geometry != geometry.model_dump(mode="json", exclude_none=True):
            raise MapAssetBundleValidationError("geometry descriptor differs from assigned bundle")
        if bundle.georeference is not None:
            raw = tree.require_file(bundle.georeference.relative_path, "georeference").read(_MAX_GEOREFERENCE_JSON_BYTES)
            try:
                contract = MapGeoreferenceV1.model_validate(_parse_json_object(raw, "georeference"))
            except Exception as exc:
                raise MapAssetBundleValidationError("invalid assigned georeference") from exc
            if contract != bundle.georeference.contract:
                raise MapAssetBundleValidationError("assigned georeference contract differs")
        elif any(path == "georeference" or path.startswith("georeference/") for path in files):
            raise MapAssetBundleValidationError("unbound georeference file")
        self._validate_geometry_assets(tree, geometry, set(files), assigned=True)
        stored = None
        if _for_activation:
            from services.geometry.mapping_backend_checkpoint_store import MappingBackendCheckpointStore
            descriptor = source["manifest"]
            try:
                # Match the direct runtime loader's canonical-path semantics.
                # The original open tree still verifies its named entries below.
                stored = MappingBackendCheckpointStore(tree.path.resolve(strict=True)).load(
                    descriptor["backend_kind"], schema_version=2,
                    expected_checkpoint_id=descriptor["checkpoint_id"])
            except Exception as exc:
                raise MapAssetBundleValidationError("invalid retained activation checkpoint") from exc
        else:
            self._validate_assigned_checkpoint_bodies(tree, source)
        tree.verify_all()
        if stored is not None:
            result = object.__new__(_AssignedActivationRead)
            for name, value in (("_repository", self), ("bundle", bundle),
                                ("root", tree.path), ("stored_checkpoint", stored)):
                object.__setattr__(result, name, value)
            return result
        return bundle

    @staticmethod
    def _validate_assigned_source_document(source):
        from services.geometry.mapping_backend_checkpoint_store import _v2_manifest_metadata
        if not isinstance(source, dict) or set(source) != {"selector_path", "manifest_path", "manifest"}:
            raise MapAssetBundleValidationError("assigned checkpoint source descriptor is incomplete")
        document = source["manifest"]
        if not isinstance(document, dict) or type(document.get("schema_version")) is not int or document["schema_version"] != 2:
            raise MapAssetBundleValidationError("assigned outer asset requires checkpoint schema 2")
        kind, checkpoint_id = document.get("backend_kind"), document.get("checkpoint_id")
        if kind not in {"voxel", "raytracing", "slam_toolbox"}:
            raise MapAssetBundleValidationError("unsupported assigned checkpoint backend")
        root = "mapping_backend/%s/" % kind
        if (source["selector_path"] != root + "checkpoint-v2.json"
                or source["manifest_path"] != root + "manifests/id-%s.json" % checkpoint_id):
            raise MapAssetBundleValidationError("assigned checkpoint source paths are not canonical")
        try:
            return _v2_manifest_metadata(document, checkpoint_id, kind)
        except Exception as exc:
            raise MapAssetBundleValidationError("invalid assigned checkpoint source/member metadata") from exc

    def _assigned_checkpoint_source(self, tree, geometry):
        from services.geometry.mapping_backend_checkpoint_store import _v2_selector, _artifact_from_document
        if geometry is None:
            raise MapAssetBundleValidationError("assigned asset requires checkpoint geometry")
        raw = geometry.metadata.get("mapping_backend_checkpoint")
        required = {"present", "schema_version", "backend_kind", "checkpoint_id", "manifest_path"}
        if (not isinstance(raw, dict) or set(raw) != required or raw["present"] is not True
                or type(raw["schema_version"]) is not int or raw["schema_version"] != 2):
            raise MapAssetBundleValidationError("assigned asset requires explicit checkpoint-v2 source, without digest aliases")
        kind, checkpoint_id = raw["backend_kind"], raw["checkpoint_id"]
        if kind not in {"voxel", "raytracing", "slam_toolbox"}:
            raise MapAssetBundleValidationError("unsupported assigned checkpoint backend")
        root = "mapping_backend/%s/" % kind
        source = {"selector_path": root + "checkpoint-v2.json", "manifest_path": raw["manifest_path"]}
        if raw["manifest_path"] != root + "manifests/id-%s.json" % checkpoint_id:
            raise MapAssetBundleValidationError("checkpoint manifest path differs from its assigned ID")
        if any(path.startswith("mapping_backend/") and ("checkpoint-v1.json" in path or "/sha256-" in path)
               for path in tree.files):
            raise MapAssetBundleValidationError("mixed legacy/assigned checkpoint inventory")
        selector = _parse_json_object(tree.require_file(source["selector_path"], "assigned checkpoint selector").read(_MAX_CHECKPOINT_JSON_BYTES), "assigned checkpoint selector")
        try:
            selected, _name = _v2_selector(selector, kind)
        except Exception as exc:
            raise MapAssetBundleValidationError("invalid assigned checkpoint selector") from exc
        if selected != checkpoint_id:
            raise MapAssetBundleValidationError("checkpoint selector differs from declared assigned source")
        source["manifest"] = _parse_json_object(tree.require_file(source["manifest_path"], "assigned checkpoint manifest").read(_MAX_CHECKPOINT_JSON_BYTES), "assigned checkpoint manifest")
        checkpoint, _occupancy, _cloud, members = self._validate_assigned_source_document(source)
        for role, member in members.items():
            relative = root + member["path"]
            if tree.require_file(relative, role).size() != member["size"]:
                raise MapAssetBundleValidationError("checkpoint member size differs from retained descriptor")
            if "artifact" in member:
                companion = root + "objects/id-%s.json" % member["artifact"]["id"]
                descriptor = _parse_json_object(tree.require_file(companion, "artifact companion").read(_MAX_CHECKPOINT_JSON_BYTES), "artifact companion")
                try:
                    retained = _artifact_from_document(descriptor, role)
                except Exception as exc:
                    raise MapAssetBundleValidationError("invalid assigned artifact companion") from exc
                expected = (checkpoint.graph_reference if role == "planar_graph" else
                    checkpoint.fine_reference if role == "fine_map" else checkpoint.dense_reference)
                if retained != expected:
                    raise MapAssetBundleValidationError("assigned artifact companion differs from retained descriptor")
        return source

    def _validate_assigned_checkpoint_bodies(self, tree, source):
        from services.geometry.checkpoint_payload import FrozenPayload
        from services.geometry.mapping_backend_checkpoint_store import MappingBackendCheckpointStore
        # A borrowed immutable descriptor view satisfies the existing bounded
        # semantic validators. It never materializes/copies a whole member and
        # cannot escape this open, state-checked directory lifetime.
        class BorrowedPayload(FrozenPayload):
            def __init__(self, managed):
                self.managed = managed
            def __len__(self):
                return self.managed.size()
            def __getitem__(self, key):
                if isinstance(key, slice):
                    start, stop, step = key.indices(len(self))
                    if step != 1:
                        raise ValueError("borrowed payload only supports contiguous bounded slices")
                    return self.managed.slice(start, stop)
                index = key if key >= 0 else len(self) + key
                if not 0 <= index < len(self):
                    raise IndexError(index)
                return self.managed.slice(index, index + 1)[0]
            def chunks(self):
                for offset in range(0, len(self), _FILE_READ_CHUNK_BYTES):
                    yield self.managed.slice(offset, offset + _FILE_READ_CHUNK_BYTES)
        checkpoint, occupancy, cloud, members = self._validate_assigned_source_document(source)
        root = "mapping_backend/%s/" % checkpoint.backend_kind
        bodies = {role: BorrowedPayload(tree.require_file(root + descriptor["path"], role))
                  for role, descriptor in members.items()}
        checkpoint = replace(checkpoint, payload=bodies["planar_graph" if checkpoint.backend_kind == "slam_toolbox" else "backend_state"],
            canonical_occupancy_data=bodies["canonical_occupancy"], canonical_cloud_xyz=bodies.get("canonical_cloud"),
            fine_archive=bodies.get("fine_map"), dense_archive=bodies.get("dense_cloud"))
        try:
            MappingBackendCheckpointStore.validate_checkpoint_bundle(checkpoint,
                occupancy_asset=occupancy, cloud_asset=cloud)
        except Exception as exc:
            raise MapAssetBundleValidationError("invalid assigned checkpoint member body") from exc

    def _recover_planar_exchange(self, map_name):
        from services.mapping.catalog import MapCatalogRepository
        map_dir = self._map_dir(map_name)
        journal = map_dir / _PLANAR_EXCHANGE_JOURNAL
        if not journal.exists() and not journal.is_symlink():
            return
        with MapCatalogRepository(str(self._root_dir)).map_activation_guard():
            if not journal.exists() and not journal.is_symlink():
                return
            record = _parse_json_object(_read_regular_file(journal, "二维目录交换日志", maximum_bytes=4096), "二维目录交换日志")
            if set(record) != {"schema_version", "previous", "candidate", "staging", "previous_directory", "candidate_directory"} or record["schema_version"] != 1:
                raise MapAssetBundleValidationError("二维目录交换日志字段非法")
            previous = _validate_sha256(record["previous"], "原清单摘要")
            candidate = _validate_sha256(record["candidate"], "新清单摘要")
            staging = self._require_staging_directory(map_name, map_dir / str(record["staging"]), must_exist=False)
            current = self._load_validated_bundle(map_name, self._current_dir(map_name))
            if current.manifest_sha256 not in {previous, candidate}:
                raise MapAssetBundleValidationError("二维目录交换当前身份无法确定，保留全部资源")
            identity = record["previous_directory" if current.manifest_sha256 == candidate else "candidate_directory"]
            if not isinstance(identity, list) or len(identity) != 2 or any(isinstance(v, bool) or not isinstance(v, int) or v < 0 for v in identity):
                raise MapAssetBundleValidationError("二维交换目录身份非法")
            if staging.exists() or staging.is_symlink():
                metadata = staging.lstat()
                if not stat.S_ISDIR(metadata.st_mode) or [metadata.st_dev, metadata.st_ino] != identity:
                    raise MapAssetBundleValidationError("二维交换退役目录被替换，拒绝清理")
            # 先证明current引用持久，再删除只属于本次事务的另一目录。
            _fsync_directory(map_dir)
            if staging.exists():
                self._discard_owned_staging(map_name, staging, identity)
            journal.unlink()
            _fsync_directory(map_dir)

    def _replace_planar_bundle(self, bundle, *, staging_dir, expected_previous_manifest_sha256):
        from services.mapping.catalog import MapCatalogRepository
        expected = _validate_sha256(expected_previous_manifest_sha256, "二维续建原清单摘要")
        map_name = bundle.map_name
        self._recover_planar_exchange(map_name)
        staging = self._require_staging_directory(map_name, staging_dir, must_exist=True)
        current = self._current_dir(map_name)
        journal = current.parent / _PLANAR_EXCHANGE_JOURNAL
        if bundle.geometry_asset is None or bundle.geometry_asset.metadata.get("target_kind") != "planar_graph":
            raise MapAssetBundleValidationError("只有明确二维完整图续建允许替换current")
        preserve_staging = False
        try:
            self._write_candidate_bundle(bundle, staging)
            candidate_sha = _sha256_bytes(self._write_manifest(map_name, staging))
            with _StableDirectoryTree(staging) as candidate_tree:
                manifest_sha, validated = self._validate_open_directory(map_name, candidate_tree, expected_manifest_sha256=candidate_sha)
                _fsync_tree(candidate_tree)
                catalog = MapCatalogRepository(str(self._root_dir))
                with catalog.map_activation_guard():
                    if catalog.get_map_activation() is not None or catalog.get_relocalization_intent() is not None:
                        raise MapAssetBundleValidationError("地图激活或重定位事务尚未收尾，不能交换二维资产")
                    with _StableDirectoryTree(current) as previous_tree:
                        previous_sha, previous = self._validate_open_directory(map_name, previous_tree)
                        if previous_sha != expected:
                            if previous_sha == candidate_sha:
                                return previous
                            raise MapAlreadyFinalizedError(map_name, previous_sha, candidate_sha)
                        if previous.geometry_asset is None or previous.geometry_asset.metadata.get("target_kind") != "planar_graph":
                            raise MapAssetBundleValidationError("二维续建不能覆盖其它类型地图")
                        old_checkpoint = previous.geometry_asset.metadata.get("mapping_backend_checkpoint") or {}
                        new_checkpoint = validated.geometry_asset.metadata.get("mapping_backend_checkpoint") or {}
                        if old_checkpoint.get("object_sha256") == new_checkpoint.get("object_sha256"):
                            return previous
                        candidate_tree.verify_all()
                        previous_tree.verify_all()
                        self._inject_fault("before_planar_exchange")
                        old_stat, new_stat = current.stat(), staging.stat()
                        record = {"schema_version": 1, "previous": previous_sha, "candidate": candidate_sha, "staging": staging.name,
                            "previous_directory": [old_stat.st_dev, old_stat.st_ino], "candidate_directory": [new_stat.st_dev, new_stat.st_ino]}
                        _atomic_write_relative(current.parent, _PLANAR_EXCHANGE_JOURNAL, _canonical_json_bytes(record))
                        preserve_staging = True
                        _exchange_directories(candidate_tree.parent_descriptor, staging.name, current.name)
                        candidate_tree.rebind_root_after_rename(current.name)
                        previous_tree.rebind_root_after_rename(staging.name)
                        self._inject_fault("before_planar_directory_fsync")
                        os.fsync(candidate_tree.parent_descriptor)
                        self._inject_fault("after_planar_exchange")
                        committed = validated.model_copy(update={"manifest_sha256": manifest_sha}, deep=True)
                    self._discard_owned_staging(map_name, staging, (old_stat.st_dev, old_stat.st_ino))
                    journal.unlink()
                    os.fsync(candidate_tree.parent_descriptor)
                    preserve_staging = False
            return committed
        except Exception as error:
            if preserve_staging or journal.exists():
                preserve_staging = True
                raise MapAssetCommitUncertainError("二维资产目录交换需要按日志对账：%s" % error) from error
            raise
        finally:
            if not preserve_staging and (staging.exists() or staging.is_symlink()):
                self.discard_staging_directory(map_name, staging)

    def current_directory(self, map_name: str) -> Path:
        """Resolve the committed bundle root; callers still load through validation."""

        return self._current_dir(map_name)

    def _write_candidate_bundle(
        self,
        bundle: MapAssetBundle,
        staging_dir: Path,
    ) -> MapAssetBundle:
        georeference = bundle.georeference
        if georeference is not None:
            georeference_path = _contained_path(staging_dir, georeference.relative_path)
            if not georeference_path.exists() and not georeference_path.is_symlink():
                _atomic_write_relative(
                    staging_dir,
                    georeference.relative_path,
                    _model_json_bytes(georeference.contract),
                    create_parents=True,
                )
            georeference_payload = _read_regular_file(
                georeference_path,
                "地理参考文件",
                maximum_bytes=_MAX_GEOREFERENCE_JSON_BYTES,
            )
            georeference = self._validate_georeference_asset(
                georeference,
                payload=georeference_payload,
                raw_sha256=_sha256_bytes(georeference_payload),
                require_bound_digest=False,
            )

        geometry_asset = bundle.geometry_asset
        if geometry_asset is not None:
            geometry_relative = PurePosixPath(
                "geometry",
                build_safe_dir_segment(geometry_asset.geometry_stack),
                "geometry_asset.json",
            ).as_posix()
            geometry_asset = geometry_asset.model_copy(
                update={
                    "metadata": {
                        **dict(geometry_asset.metadata),
                        "geometry_asset_path": geometry_relative,
                        "geometry_asset_root": str(PurePosixPath(geometry_relative).parent),
                    }
                },
                deep=True,
            )
            _atomic_write_relative(
                staging_dir,
                geometry_relative,
                _model_json_bytes(geometry_asset),
                create_parents=True,
            )
        candidate = bundle.model_copy(
            update={
                "geometry_asset": geometry_asset,
                "georeference": georeference,
                "manifest_sha256": None,
            },
            deep=True,
        )
        _atomic_write_relative(
            staging_dir,
            _BUNDLE_FILE_NAME,
            _model_json_bytes(candidate),
        )
        return candidate

    def _write_manifest(self, map_name: str, directory: Path) -> bytes:
        files = []
        with _StableDirectoryTree(directory) as tree:
            for relative_path, managed_file in tree.files.items():
                if relative_path == _MANIFEST_FILE_NAME:
                    continue
                size, sha256 = managed_file.hash()
                files.append(
                    {
                        "path": relative_path,
                        "size": size,
                        "sha256": sha256,
                    }
                )
            tree.verify_all()
        files.sort(key=lambda item: str(item["path"]))
        if len(files) > _MAX_MANIFEST_FILE_ENTRIES:
            raise MapAssetBundleValidationError("地图资产清单文件项数量超过上限。")
        document = {
            "schema_version": _MANIFEST_SCHEMA_VERSION,
            "map_name": map_name,
            "bundle_path": _BUNDLE_FILE_NAME,
            "files": files,
        }
        payload = _canonical_json_bytes(document)
        if len(payload) > _MAX_MANIFEST_JSON_BYTES:
            raise MapAssetBundleValidationError("地图资产清单超过大小上限。")
        _atomic_write_relative(directory, _MANIFEST_FILE_NAME, payload)
        return payload

    def _resolve_existing_current(
        self,
        map_name: str,
        current_dir: Path,
        staging_dir: Path,
        candidate_sha256: str,
    ) -> MapAssetBundle:
        existing = self._load_validated_bundle(map_name, current_dir)
        existing_sha256 = str(existing.manifest_sha256 or "")
        self.discard_staging_directory(map_name, staging_dir)
        if existing_sha256 == candidate_sha256:
            return existing
        raise MapAlreadyFinalizedError(map_name, existing_sha256, candidate_sha256)

    def _load_validated_bundle(
        self,
        map_name: str,
        directory: Path,
        *,
        expected_manifest_sha256: Optional[str] = None,
        expected_asset_identity: Optional[AssetIdentityV2] = None,
        _for_activation: bool = False,
    ) -> MapAssetBundle:
        manifest_sha256, bundle = self._validate_directory(
            map_name,
            directory,
            expected_manifest_sha256=expected_manifest_sha256,
            expected_asset_identity=expected_asset_identity,
            _for_activation=_for_activation,
        )
        if type(bundle) is _AssignedActivationRead or bundle.identity_schema_version == 2:
            return bundle
        return bundle.model_copy(update={"manifest_sha256": manifest_sha256}, deep=True)

    def _validate_directory(
        self,
        map_name: str,
        directory: Path,
        *,
        expected_manifest_sha256: Optional[str] = None,
        expected_asset_identity: Optional[AssetIdentityV2] = None,
        _for_activation: bool = False,
    ) -> Tuple[Optional[str], MapAssetBundle]:
        with _StableDirectoryTree(directory) as tree:
            result = self._validate_open_directory(
                map_name,
                tree,
                expected_manifest_sha256=expected_manifest_sha256,
                expected_asset_identity=expected_asset_identity,
                _for_activation=_for_activation,
            )
            tree.verify_all()
            return result

    def _validate_open_directory(
        self,
        map_name: str,
        tree: _StableDirectoryTree,
        *,
        expected_manifest_sha256: Optional[str] = None,
        expected_asset_identity: Optional[AssetIdentityV2] = None,
        _for_activation: bool = False,
    ) -> Tuple[Optional[str], MapAssetBundle]:
        _require_typed_asset_identity(expected_asset_identity, optional=True)
        manifest_file = tree.require_file(_MANIFEST_FILE_NAME, "地图资产清单")
        header = _parse_json_object(manifest_file.read(_MAX_MANIFEST_JSON_BYTES), "地图资产清单")
        if type(header.get("schema_version")) is not int or header["schema_version"] not in {1, 2}:
            raise MapAssetBundleValidationError("missing or unsupported explicit manifest schema")
        if header["schema_version"] == 2:
            if expected_manifest_sha256 is not None:
                raise MapAssetBundleValidationError("assigned manifest cannot use a legacy digest expectation")
            return None, self._validate_assigned_open_directory(map_name, tree, header,
                expected_asset_identity=expected_asset_identity, _for_activation=_for_activation)
        if expected_asset_identity is not None or "asset_identity" in header:
            raise MapAssetBundleValidationError("legacy manifest cannot carry an assigned identity")
        manifest_payload, _manifest_size, manifest_sha256 = manifest_file.read_and_hash(_MAX_MANIFEST_JSON_BYTES)
        manifest_sha256 = _sha256_bytes(manifest_payload)
        if expected_manifest_sha256 and manifest_sha256 != expected_manifest_sha256:
            raise MapAssetBundleValidationError("地图资产清单摘要与提交摘要不一致。")
        manifest = _parse_json_object(manifest_payload, "地图资产清单")
        if set(manifest) != {"schema_version", "map_name", "bundle_path", "files"}:
            raise MapAssetBundleValidationError("地图资产清单字段不完整或包含未知字段。")
        if manifest.get("schema_version") != _MANIFEST_SCHEMA_VERSION:
            raise MapAssetBundleValidationError("不支持的地图资产清单 schema_version。")
        if str(manifest.get("map_name") or "") != map_name:
            raise MapAssetBundleValidationError("地图资产清单 map_name 不匹配。")
        if manifest.get("bundle_path") != _BUNDLE_FILE_NAME:
            raise MapAssetBundleValidationError("地图资产清单 bundle_path 非法。")

        entries = manifest.get("files")
        if not isinstance(entries, list):
            raise MapAssetBundleValidationError("地图资产清单 files 必须是数组。")
        if len(entries) > _MAX_MANIFEST_FILE_ENTRIES:
            raise MapAssetBundleValidationError("地图资产清单文件项数量超过上限。")
        expected_files: Dict[str, Mapping[str, object]] = {}
        for entry in entries:
            if not isinstance(entry, dict) or set(entry) != {"path", "size", "sha256"}:
                raise MapAssetBundleValidationError("地图资产清单文件项非法。")
            relative = _validate_relative_path(entry.get("path"), "地图资产清单文件路径")
            if relative == _MANIFEST_FILE_NAME or relative in expected_files:
                raise MapAssetBundleValidationError("地图资产清单包含重复或保留路径。")
            size = entry.get("size")
            if isinstance(size, bool) or not isinstance(size, int) or size < 0:
                raise MapAssetBundleValidationError("地图资产清单文件大小非法。")
            sha256 = _validate_sha256(entry.get("sha256"), "地图资产清单文件摘要")
            expected_files[relative] = {"size": size, "sha256": sha256}

        actual_files = {
            relative: managed for relative, managed in tree.files.items() if relative != _MANIFEST_FILE_NAME
        }
        if set(actual_files) != set(expected_files):
            raise MapAssetBundleValidationError("地图资产清单与目录中的文件集合不一致。")

        bundle_file = tree.require_file(_BUNDLE_FILE_NAME, "地图资产包")
        bundle_payload, bundle_size, bundle_sha256 = bundle_file.read_and_hash(_MAX_BUNDLE_JSON_BYTES)
        _validate_manifested_file_observation(
            _BUNDLE_FILE_NAME,
            expected_files.get(_BUNDLE_FILE_NAME),
            bundle_size,
            bundle_sha256,
        )
        bundle_document = _parse_json_object(bundle_payload, "地图资产包")
        if bundle_document.get("manifest_sha256") is not None and bundle_document.get("manifest_sha256") != "":
            raise MapAssetBundleValidationError("bundle.json 不得自引用 manifest_sha256。")
        georeference_document = bundle_document.get("georeference")
        if georeference_document is not None and (
            not isinstance(georeference_document, dict)
            or set(georeference_document) != {"schema_version", "relative_path", "raw_sha256", "contract"}
        ):
            raise MapAssetBundleValidationError("地图资产包地理参考绑定字段不完整或包含未知字段。")
        try:
            bundle = MapAssetBundle.model_validate(bundle_document)
        except Exception as exc:
            raise MapAssetBundleValidationError("地图资产包契约非法。") from exc
        if bundle.identity_schema_version != 1:
            raise MapAssetBundleValidationError("legacy manifest cannot wrap an assigned bundle")
        if bundle.map_name != map_name:
            raise MapAssetBundleValidationError("地图资产包 map_name 不匹配。")
        validated_paths = {_BUNDLE_FILE_NAME}
        if bundle.georeference is not None:
            georeference_file = tree.require_file(
                bundle.georeference.relative_path,
                "地理参考文件",
            )
            georeference_payload, georeference_size, georeference_sha256 = georeference_file.read_and_hash(
                _MAX_GEOREFERENCE_JSON_BYTES
            )
            _validate_manifested_file_observation(
                bundle.georeference.relative_path,
                expected_files.get(bundle.georeference.relative_path),
                georeference_size,
                georeference_sha256,
            )
            georeference = self._validate_georeference_asset(
                bundle.georeference,
                payload=georeference_payload,
                raw_sha256=georeference_sha256,
                manifested_paths=set(expected_files),
                require_bound_digest=True,
            )
            bundle = bundle.model_copy(update={"georeference": georeference}, deep=True)
            validated_paths.add(georeference.relative_path)
        elif any(path == "georeference" or path.startswith("georeference/") for path in expected_files):
            raise MapAssetBundleValidationError("无地理锚点的资产不得包含未绑定的地理参考文件。")
        for relative, metadata in expected_files.items():
            if relative in validated_paths:
                continue
            size, sha256 = actual_files[relative].hash()
            _validate_manifested_file_observation(relative, metadata, size, sha256)
        self._validate_geometry_assets(tree, bundle.geometry_asset, set(expected_files))
        tree.verify_all()
        return manifest_sha256, bundle

    def _validate_georeference_asset(
        self,
        binding: MapGeoreferenceBindingV1,
        *,
        payload: bytes,
        raw_sha256: str,
        manifested_paths: Optional[set[str]] = None,
        require_bound_digest: bool,
    ) -> MapGeoreferenceBindingV1:
        relative_path = binding.relative_path
        if relative_path != MAP_GEOREFERENCE_RELATIVE_PATH:
            raise MapAssetBundleValidationError("地理参考文件路径不是第一版固定路径。")
        if manifested_paths is not None and relative_path not in manifested_paths:
            raise MapAssetBundleValidationError("地理参考文件未被顶层清单绑定。")
        if not payload:
            raise MapAssetBundleValidationError("地理参考文件不能为空。")
        if len(payload) > _MAX_GEOREFERENCE_JSON_BYTES:
            raise MapAssetBundleValidationError("地理参考文件超过 64 KiB 上限。")

        if require_bound_digest and binding.raw_sha256 is None:
            raise MapAssetBundleValidationError("地图资产包缺少地理参考原文 SHA-256。")
        if _sha256_bytes(payload) != raw_sha256:
            raise MapAssetBundleValidationError("地理参考原文读取摘要不一致。")
        if binding.raw_sha256 is not None and binding.raw_sha256 != raw_sha256:
            raise MapAssetBundleValidationError("地理参考原文 SHA-256 与资产包绑定不一致。")

        document = _parse_json_object(payload, "地理参考文件")
        try:
            contract = MapGeoreferenceV1.model_validate(document)
        except Exception as exc:
            raise MapAssetBundleValidationError("地理参考合同非法。") from exc
        if contract.model_dump(mode="json") != binding.contract.model_dump(mode="json"):
            raise MapAssetBundleValidationError("地理参考原文与资产包中的类型化合同不一致。")
        return binding.model_copy(update={"raw_sha256": raw_sha256}, deep=True)

    def _validate_geometry_assets(
        self,
        tree: _StableDirectoryTree,
        geometry_asset: Optional[MapGeometryAssetBinding],
        manifested_paths: set[str],
        *,
        assigned: bool = False,
    ) -> None:
        if geometry_asset is None:
            raise MapAssetBundleValidationError("地图资产包缺少 geometry_asset。")
        # 二维导航资产可以没有点云或原生后端历史状态；只验证实际声明的文件。
        database_quality = geometry_asset.quality_metrics.get("database") or {}
        if not isinstance(database_quality, dict):
            raise MapAssetBundleValidationError("建图数据库质量描述非法。")
        database_required = database_quality.get("required", True) is not False
        database_declared = (
            database_required
            or database_quality.get("present") is True
            or geometry_asset.mapping_database_path in manifested_paths
            or any(
                asset.asset_type in {MapGeometryAssetType.MAPPING_DATABASE, MapGeometryAssetType.PLANAR_GRAPH}
                for asset in geometry_asset.assets
            )
        )
        referenced = [asset.path for asset in geometry_asset.assets]
        if database_declared:
            referenced.append(geometry_asset.mapping_database_path)
        pcd_paths = {
            asset.path for asset in geometry_asset.assets
            if asset.asset_type == MapGeometryAssetType.PCD
        }
        if geometry_asset.pcd_path:
            pcd_paths.add(geometry_asset.pcd_path)
        referenced.extend(sorted(pcd_paths))
        for relative in dict.fromkeys(referenced):
            normalized = _validate_relative_path(relative, "几何资产路径")
            if normalized not in manifested_paths:
                raise MapAssetBundleValidationError("几何资产未被顶层清单绑定：%s" % normalized)
            tree.require_file(normalized, "几何资产").size()

        if database_declared:
            database_file = tree.require_file(geometry_asset.mapping_database_path, "建图数据库")
            if database_file.size() == 0:
                raise MapAssetBundleValidationError("建图数据库不能为空。")
        for pcd_path in sorted(pcd_paths):
            self._validate_pcd(tree.require_file(pcd_path, "PCD 地图"))
        navigation_assets = [
            asset for asset in geometry_asset.assets if asset.asset_type == MapGeometryAssetType.NAVIGATION_MAP
        ]
        if not navigation_assets:
            raise MapAssetBundleValidationError("地图资产包缺少导航地图资产。")
        for asset in navigation_assets:
            self._validate_nav2_yaml(tree, asset.path, manifested_paths)
        if not assigned:
            self._validate_backend_checkpoint(tree, geometry_asset, manifested_paths)

    def _validate_backend_checkpoint(
        self,
        tree: _StableDirectoryTree,
        geometry_asset: MapGeometryAssetBinding,
        manifested_paths: set[str],
    ) -> None:
        raw_checkpoint = geometry_asset.metadata.get("mapping_backend_checkpoint")
        if raw_checkpoint is None:
            return
        if not isinstance(raw_checkpoint, dict):
            raise MapAssetBundleValidationError("backend checkpoint 描述非法。")
        if raw_checkpoint.get("present") is False:
            return
        if raw_checkpoint.get("present") is not True:
            raise MapAssetBundleValidationError("backend checkpoint 缺少明确的存在状态。")
        backend_kind = str(raw_checkpoint.get("backend_kind") or "").strip()
        schema_version = raw_checkpoint.get("schema_version")
        if not backend_kind or isinstance(schema_version, bool) or not isinstance(schema_version, int):
            raise MapAssetBundleValidationError("backend checkpoint 身份字段非法。")
        manifest_relative = _validate_relative_path(
            raw_checkpoint.get("manifest_path"),
            "backend checkpoint manifest 路径",
        )
        object_relative = _validate_relative_path(
            raw_checkpoint.get("object_path"),
            "backend checkpoint object 路径",
        )
        selector_relative = PurePosixPath(
            "mapping_backend",
            backend_kind,
            "checkpoint-v%d.json" % schema_version,
        ).as_posix()
        for relative in (selector_relative, manifest_relative, object_relative):
            if relative not in manifested_paths:
                raise MapAssetBundleValidationError("backend checkpoint 文件未被顶层清单绑定：%s" % relative)

        manifest_payload, _manifest_size, manifest_sha256 = tree.require_file(
            manifest_relative,
            "backend checkpoint manifest",
        ).read_and_hash(_MAX_CHECKPOINT_JSON_BYTES)
        expected_manifest_sha256 = _validate_sha256(
            raw_checkpoint.get("manifest_sha256"),
            "backend checkpoint manifest SHA-256",
        )
        expected_object_sha256 = _validate_sha256(
            raw_checkpoint.get("object_sha256"),
            "backend checkpoint object SHA-256",
        )
        if manifest_sha256 != expected_manifest_sha256:
            raise MapAssetBundleValidationError("backend checkpoint manifest SHA-256 不匹配。")
        _object_size, object_sha256 = tree.require_file(
            object_relative,
            "backend checkpoint object",
        ).hash()
        if object_sha256 != expected_object_sha256:
            raise MapAssetBundleValidationError("backend checkpoint object SHA-256 不匹配。")

        selector = _parse_json_object(
            tree.require_file(
                selector_relative,
                "backend checkpoint selector",
            ).read_and_hash(_MAX_CHECKPOINT_JSON_BYTES)[0],
            "backend checkpoint selector",
        )
        selector_manifest = selector.get("manifest")
        backend_root = PurePosixPath("mapping_backend", backend_kind)
        try:
            selector_manifest_path = PurePosixPath(manifest_relative).relative_to(backend_root).as_posix()
        except ValueError as exc:
            raise MapAssetBundleValidationError("backend checkpoint manifest 不属于目标 backend。") from exc
        if (
            selector.get("schema_version") != schema_version
            or str(selector.get("backend_kind") or "") != backend_kind
            or not isinstance(selector_manifest, dict)
            or selector_manifest.get("sha256") != expected_manifest_sha256
            or selector_manifest.get("path") != selector_manifest_path
        ):
            raise MapAssetBundleValidationError("backend checkpoint selector 与资产包身份不一致。")
        try:
            from services.geometry.mapping_backend_checkpoint_store import (
                MappingBackendCheckpointStore,
                MappingBackendCheckpointStoreError,
            )

            stored = MappingBackendCheckpointStore(tree.path).load(
                backend_kind,
                schema_version=schema_version,
            )
        except MappingBackendCheckpointStoreError as exc:
            raise MapAssetBundleValidationError("backend checkpoint 内层契约不可加载。") from exc
        if (
            stored.manifest_sha256 != expected_manifest_sha256
            or stored.checkpoint.content_sha256.hex() != expected_object_sha256
        ):
            raise MapAssetBundleValidationError("backend checkpoint 加载结果与资产包身份不一致。")
        tree.verify_all()

    @staticmethod
    def _validate_pcd(managed_file: _ManagedRegularFile) -> None:
        # 复用激活/显示链的PCD解析器，从已持有的文件描述符读取，避免另建解析规则。
        import numpy as np
        from services.geometry.pointcloud_asset_stream import open_pointcloud_chunks

        managed_file.verify_named_entry()
        try:
            with open_pointcloud_chunks(
                Path("/proc/self/fd/%d" % managed_file.descriptor), format_hint=".pcd",
            ) as (header, chunks):
                point_count = 0
                for points in chunks:
                    if not np.isfinite(points).all():
                        raise MapAssetBundleValidationError("PCD 地图包含非有限点。")
                    point_count += len(points)
        except Exception as exc:
            raise MapAssetBundleValidationError("PCD 地图不可读取：%s" % managed_file.relative_path) from exc
        managed_file.verify_named_entry()
        if not point_count:
            raise MapAssetBundleValidationError("PCD 地图没有有效有限点。")
        if header.points > 0 and point_count != header.points:
            raise MapAssetBundleValidationError("PCD 地图点数与头部声明不一致。")

    @staticmethod
    def _validate_nav2_yaml(
        tree: _StableDirectoryTree,
        yaml_relative: str,
        manifested_paths: set[str],
    ) -> None:
        text = (
            tree.require_file(
                yaml_relative,
                "Nav2 地图 YAML",
            )
            .read(_MAX_NAV2_YAML_BYTES)
            .decode("utf-8")
        )
        image_values: List[str] = []
        for raw_line in text.splitlines():
            line = raw_line.strip()
            if not line or line.startswith("#") or ":" not in line:
                continue
            key, value = line.split(":", 1)
            if key.strip() != "image":
                continue
            try:
                tokens = shlex.split(value.strip(), comments=True, posix=True)
            except ValueError as exc:
                raise MapAssetBundleValidationError("Nav2 地图 image 字段非法。") from exc
            if len(tokens) != 1:
                raise MapAssetBundleValidationError("Nav2 地图必须包含一个 image 路径。")
            image_values.append(tokens[0])
        if len(image_values) != 1:
            raise MapAssetBundleValidationError("Nav2 地图必须包含唯一的 image 字段。")
        image_value = image_values[0]
        image_path = PurePosixPath(image_value)
        if image_path.is_absolute():
            raise MapAssetBundleValidationError("Nav2 地图 image 必须位于资产包内。")
        combined = PurePosixPath(yaml_relative).parent / image_path
        normalized = _validate_relative_path(combined.as_posix(), "Nav2 地图 image 路径")
        if normalized not in manifested_paths:
            raise MapAssetBundleValidationError("Nav2 地图 image 未被顶层清单绑定。")
        pgm_header = tree.require_file(normalized, "Nav2 地图图像").prefix(2)
        if pgm_header not in {b"P2", b"P5"}:
            raise MapAssetBundleValidationError("Nav2 地图图像不是有效的 PGM。")

    def _regular_files(self, directory: Path) -> Iterable[Tuple[str, Path]]:
        def walk(parent: Path, relative_parent: PurePosixPath) -> Iterable[Tuple[str, Path]]:
            with os.scandir(parent) as entries:
                ordered = sorted(entries, key=lambda entry: entry.name)
            for entry in ordered:
                relative = relative_parent / entry.name
                entry_stat = entry.stat(follow_symlinks=False)
                if stat.S_ISLNK(entry_stat.st_mode):
                    raise MapAssetBundleValidationError("地图资产包不得包含符号链接：%s" % relative)
                path = Path(entry.path)
                if stat.S_ISDIR(entry_stat.st_mode):
                    yield from walk(path, relative)
                elif stat.S_ISREG(entry_stat.st_mode):
                    yield relative.as_posix(), path
                else:
                    raise MapAssetBundleValidationError("地图资产包包含非普通文件：%s" % relative)

        yield from walk(directory, PurePosixPath())

    def _migrate_legacy_latest(self, map_name: str) -> Optional[MapAssetBundle]:
        map_dir = self._map_dir(map_name)
        metadata_path = map_dir / "metadata.json"
        versions_root = map_dir / "versions"
        if not metadata_path.exists() or not versions_root.exists():
            return None
        metadata = _parse_json_object(
            _read_regular_file(
                metadata_path,
                "旧地图 metadata.json",
                maximum_bytes=_MAX_LEGACY_METADATA_JSON_BYTES,
            ),
            "旧地图 metadata.json",
        )
        if str(metadata.get("map_name") or "") != map_name:
            raise MapAssetMigrationError("旧地图 metadata.json 的 map_name 不匹配。")
        selected = self._select_legacy_version(versions_root, metadata.get("latest_version_id"))
        if selected is None:
            return None
        staging = self.create_staging_directory(map_name)
        try:
            self._copy_legacy_tree(selected, staging)
            legacy_document = _parse_json_object(
                _read_regular_file(
                    selected / "version.json",
                    "旧地图 version.json",
                    maximum_bytes=_MAX_LEGACY_VERSION_JSON_BYTES,
                ),
                "旧地图 version.json",
            )
            bundle = self._convert_legacy_bundle(map_name, selected, legacy_document)
            return self.finalize_map_asset_bundle(bundle, staging_dir=staging)
        except Exception:
            if staging.exists() or staging.is_symlink():
                self.discard_staging_directory(map_name, staging)
            raise

    def _select_legacy_version(self, versions_root: Path, latest_version_id: object) -> Optional[Path]:
        explicit = str(latest_version_id or "").strip()
        if explicit:
            candidates = (
                versions_root / build_safe_dir_segment(explicit),
                versions_root / quote(explicit, safe=""),
            )
            for candidate in candidates:
                if self._legacy_version_complete(candidate):
                    return candidate
            raise MapAssetMigrationError("metadata.json 指向的旧地图版本不完整，拒绝猜测其它版本。")
        complete = [
            path
            for path in sorted(versions_root.iterdir(), key=lambda item: item.name)
            if self._legacy_version_complete(path)
        ]
        if not complete:
            return None
        if len(complete) != 1:
            raise MapAssetMigrationError("旧地图存在多个候选版本且没有明确 latest 指针，拒绝猜测。")
        return complete[0]

    @staticmethod
    def _legacy_version_complete(version_dir: Path) -> bool:
        if not version_dir.is_dir() or version_dir.is_symlink():
            return False
        version_path = version_dir / "version.json"
        if not version_path.is_file() or version_path.is_symlink():
            return False
        try:
            document = _parse_json_object(
                _read_regular_file(
                    version_path,
                    "旧地图 version.json",
                    maximum_bytes=_MAX_LEGACY_VERSION_JSON_BYTES,
                ),
                "旧地图 version.json",
            )
        except Exception:
            return False
        return isinstance(document.get("snapshot"), dict) and bool(document.get("map_name"))

    def _copy_legacy_tree(self, source: Path, target: Path) -> None:
        for relative, source_path in self._regular_files(source):
            if relative in {
                "version.json",
                "bundle.json",
                "manifest.json",
                "geometry_asset.json",
            } or relative.endswith("/geometry_asset.json"):
                continue
            target_path = _contained_path(target, relative)
            target_path.parent.mkdir(parents=True, exist_ok=True)
            current_parent = target_path.parent
            while current_parent != target:
                current_parent.chmod(0o700)
                current_parent = current_parent.parent
            shutil.copyfile(source_path, target_path)
            target_path.chmod(0o600)

    def _convert_legacy_bundle(
        self,
        map_name: str,
        source_dir: Path,
        document: Mapping[str, object],
    ) -> MapAssetBundle:
        georeference_payload = document.get("georeference")
        georeference = None
        if georeference_payload is not None:
            if not isinstance(georeference_payload, dict) or set(georeference_payload) != {
                "schema_version", "relative_path", "raw_sha256", "contract",
            }:
                raise MapAssetMigrationError("旧地图地理参考绑定不完整，拒绝推测锚点或坐标变换。")
            try:
                georeference = MapGeoreferenceBindingV1.model_validate(georeference_payload)
                georeference_path = _contained_path(source_dir, georeference.relative_path)
                georeference_bytes = _read_regular_file(
                    georeference_path,
                    "旧地图地理参考文件",
                    maximum_bytes=_MAX_GEOREFERENCE_JSON_BYTES,
                )
                georeference = self._validate_georeference_asset(
                    georeference,
                    payload=georeference_bytes,
                    raw_sha256=_sha256_bytes(georeference_bytes),
                    require_bound_digest=True,
                )
            except Exception as exc:
                raise MapAssetMigrationError("旧地图地理参考无法精确验证，拒绝迁移。") from exc

        geometry_payload = document.get("geometry_asset")
        geometry_asset = None
        if isinstance(geometry_payload, dict):
            normalized_geometry = dict(geometry_payload)
            normalized_geometry.pop("version_id", None)
            normalized_geometry["map_name"] = map_name
            for key in ("mapping_database_path", "pcd_path"):
                if normalized_geometry.get(key):
                    normalized_geometry[key] = self._legacy_relative_path(
                        source_dir,
                        normalized_geometry[key],
                    )
            assets = []
            for raw_asset in normalized_geometry.get("assets") or []:
                if not isinstance(raw_asset, dict):
                    raise MapAssetMigrationError("旧地图 geometry assets 非法。")
                asset = dict(raw_asset)
                asset_type = str(asset.get("asset_type") or "")
                if asset_type.startswith("nuwax_mapping_"):
                    continue
                asset["path"] = self._legacy_relative_path(source_dir, asset.get("path"))
                assets.append(asset)
            normalized_geometry["assets"] = assets
            normalized_geometry["metadata"] = self._normalize_legacy_metadata(
                source_dir,
                normalized_geometry.get("metadata") or {},
            )
            try:
                geometry_asset = MapGeometryAssetBinding.model_validate(normalized_geometry)
            except Exception as exc:
                raise MapAssetMigrationError("旧地图几何资产无法转换为 current/ 契约。") from exc

        snapshot = document.get("snapshot")
        if not isinstance(snapshot, dict):
            raise MapAssetMigrationError("旧地图缺少 snapshot。")
        snapshot_payload = dict(snapshot)
        legacy_source_name = str(
            document.get("source_name")
            or snapshot_payload.get("source_name")
            or "legacy_map_asset"
        ).strip()
        legacy_snapshot_token = str(
            document.get("source_version_id")
            or snapshot_payload.get("version_id")
            or ""
        ).strip()
        legacy_source_epoch = snapshot_payload.get("source_epoch") or json.dumps(
            ["legacy-map-snapshot-v2", map_name, legacy_source_name,
             legacy_snapshot_token or source_dir.name], ensure_ascii=False, separators=(",", ":")
        )
        try:
            legacy_update_sequence = max(
                1,
                int(snapshot_payload.get("update_sequence") or snapshot_payload.get("revision") or 1),
            )
        except (TypeError, ValueError) as exc:
            raise MapAssetMigrationError(
                "旧地图 snapshot revision 非法。"
            ) from exc
        for layer_key in ("occupancy_grid", "cost_map", "semantic_map"):
            layer_payload = snapshot_payload.get(layer_key)
            if isinstance(layer_payload, dict):
                normalized_layer = dict(layer_payload)
                normalized_layer["map_id"] = map_name
                snapshot_payload[layer_key] = normalized_layer
        snapshot_payload.pop("version_id", None)
        snapshot_payload.pop("revision", None)
        snapshot_payload.pop("source_snapshot_id", None)
        snapshot_payload["map_name"] = map_name
        snapshot_payload["source_name"] = legacy_source_name
        snapshot_payload["source_epoch"] = legacy_source_epoch
        snapshot_payload["update_sequence"] = legacy_update_sequence
        snapshot_payload["metadata"] = self._normalize_legacy_metadata(
            source_dir,
            snapshot_payload.get("metadata") or {},
        )
        snapshot_payload.pop("content_sha256", None)
        snapshot_payload["schema_version"] = 2
        bundle_payload = {
            "map_name": map_name,
            "source_name": legacy_source_name,
            "frame_id": document.get("frame_id"),
            "snapshot": snapshot_payload,
            "geometry_asset": geometry_asset,
            "georeference": georeference,
            "metadata": self._normalize_legacy_metadata(source_dir, document.get("metadata") or {}),
        }
        if document.get("timestamp") is not None:
            bundle_payload["timestamp"] = document.get("timestamp")
        try:
            return MapAssetBundle.model_validate(bundle_payload)
        except Exception as exc:
            raise MapAssetMigrationError("旧地图版本无法转换为 current/ 资产包。") from exc

    def _legacy_relative_path(self, source_dir: Path, value: object) -> str:
        raw = str(value or "").strip()
        if not raw:
            raise MapAssetMigrationError("旧地图包含空资产路径。")
        candidate = Path(raw).expanduser()
        if candidate.is_absolute():
            try:
                relative = candidate.resolve(strict=False).relative_to(source_dir.resolve(strict=True))
            except (OSError, ValueError) as exc:
                raise MapAssetMigrationError("旧地图资产路径越过版本目录：%s" % raw) from exc
            return _validate_relative_path(relative.as_posix(), "旧地图资产路径")
        return _validate_relative_path(PurePosixPath(raw).as_posix(), "旧地图资产路径")

    def _normalize_legacy_metadata(self, source_dir: Path, value: object) -> object:
        if isinstance(value, dict):
            normalized: Dict[str, object] = {}
            for raw_key, raw_value in value.items():
                key = str(raw_key)
                if key in {
                    "version_id",
                    "revision",
                    "map_version_id",
                    "platform_map_version_id",
                    "platform_map_version_revision",
                    "seed_map_version_id",
                    "seed_map_version_revision",
                    "latest_version_id",
                    "latest_revision",
                    "source_snapshot_id",
                    "source_version_id",
                    "snapshot_revision",
                }:
                    continue
                if key.endswith("_path") and isinstance(raw_value, str):
                    try:
                        normalized[key] = self._legacy_relative_path(source_dir, raw_value)
                    except MapAssetMigrationError:
                        continue
                else:
                    normalized[key] = self._normalize_legacy_metadata(source_dir, raw_value)
            return normalized
        if isinstance(value, list):
            return [self._normalize_legacy_metadata(source_dir, item) for item in value]
        return value

    def _require_staging_directory(
        self,
        map_name: str,
        staging_dir: Path,
        *,
        must_exist: bool,
    ) -> Path:
        map_dir = self._map_dir(map_name).absolute()
        candidate = Path(staging_dir).expanduser().absolute()
        if candidate.parent != map_dir or not candidate.name.startswith(_STAGING_PREFIX):
            raise MapAssetBundleValidationError("staging 目录不属于目标地图。")
        if must_exist:
            self._require_real_directory(candidate, "地图 staging 目录")
            observed = candidate.stat()
            owner = (observed.st_dev, observed.st_ino)
            if candidate in self._staging_owners and self._staging_owners[candidate] != owner:
                raise MapAssetBundleValidationError("staging owner has been replaced")
            self._staging_owners[candidate] = owner
        return candidate

    @staticmethod
    def _require_real_directory(path: Path, description: str) -> None:
        try:
            path_stat = path.lstat()
        except FileNotFoundError as exc:
            raise MapAssetBundleValidationError("%s不存在。" % description) from exc
        if stat.S_ISLNK(path_stat.st_mode) or not stat.S_ISDIR(path_stat.st_mode):
            raise MapAssetBundleValidationError("%s必须是真实目录。" % description)

    def _assigned_version_dir(self, map_name, asset_identity):
        _require_typed_asset_identity(asset_identity)
        return self._map_dir(map_name) / ("assigned-id-" + asset_identity.asset_id)

    def _current_dir(self, map_name: str, *, expected_asset_identity=None) -> Path:
        # Only the caller's retained identity or the committed catalog chooses
        # an independent version. Uncommitted sibling directories are ignored.
        identity = expected_asset_identity
        if identity is None:
            from services.mapping.catalog import MapCatalogRepository
            asset = MapCatalogRepository(str(self._root_dir)).get_map_asset(map_name)
            if asset is not None and asset.identity_schema_version == 2:
                identity = asset.asset_identity
        if identity is not None:
            selected = self._assigned_version_dir(map_name, identity)
            if selected.exists() or selected.is_symlink():
                return selected
        return self._map_dir(map_name) / "current"

    def has_assigned_version(self, map_name, asset_identity):
        path = self._assigned_version_dir(map_name, asset_identity)
        return path.exists() or path.is_symlink()

    def _map_dir(self, map_name: str) -> Path:
        resolved_name = str(map_name or "").strip()
        primary = self._root_dir / build_safe_dir_segment(resolved_name)
        if not primary.exists():
            legacy = self._root_dir / quote(resolved_name, safe="")
            if legacy.exists():
                return legacy
        return primary

    def _inject_fault(self, phase: str) -> None:
        if self._fault_injector is not None:
            self._fault_injector(phase)


class _ManagedRegularFile:
    def __init__(
        self,
        *,
        descriptor: int,
        parent_descriptor: int,
        name: str,
        relative_path: str,
        initial_stat: os.stat_result,
    ) -> None:
        self.descriptor = descriptor
        self.parent_descriptor = parent_descriptor
        self.name = name
        self.relative_path = relative_path
        self.initial_stat = initial_stat
        self._size: Optional[int] = None
        self._sha256: Optional[str] = None
        self._payload: Optional[bytes] = None
        self._prefix = b""

    def close(self) -> None:
        if self.descriptor >= 0:
            os.close(self.descriptor)
            self.descriptor = -1

    def _begin_observation(self) -> os.stat_result:
        current = os.fstat(self.descriptor)
        _validate_regular_file_stat(current, self.relative_path)
        _require_same_file_state(self.initial_stat, current, self.relative_path)
        return current

    def _finish_observation(
        self,
        before: os.stat_result,
        *,
        observed_size: int,
    ) -> os.stat_result:
        after = os.fstat(self.descriptor)
        _validate_regular_file_stat(after, self.relative_path)
        _require_same_file_state(before, after, self.relative_path)
        if observed_size != int(after.st_size):
            raise MapAssetBundleValidationError("地图资产文件读取长度发生变化：%s" % self.relative_path)
        self.verify_named_entry()
        return after

    def verify_named_entry(self) -> None:
        try:
            named = os.stat(
                self.name,
                dir_fd=self.parent_descriptor,
                follow_symlinks=False,
            )
        except OSError as exc:
            raise MapAssetBundleValidationError("地图资产文件命名项不可验证：%s" % self.relative_path) from exc
        _validate_regular_file_stat(named, self.relative_path)
        current = os.fstat(self.descriptor)
        _require_same_file_identity(current, named, self.relative_path)
        _require_same_file_state(self.initial_stat, current, self.relative_path)

    def read_and_hash(self, maximum_bytes: int) -> Tuple[bytes, int, str]:
        before = self._begin_observation()
        if int(before.st_size) > maximum_bytes:
            raise MapAssetBundleValidationError("地图资产文件超过大小上限：%s" % self.relative_path)
        os.lseek(self.descriptor, 0, os.SEEK_SET)
        digest = hashlib.sha256()
        chunks = []
        observed_size = 0
        while True:
            remaining = maximum_bytes + 1 - observed_size
            if remaining <= 0:
                raise MapAssetBundleValidationError("地图资产文件超过大小上限：%s" % self.relative_path)
            chunk = os.read(
                self.descriptor,
                min(_FILE_READ_CHUNK_BYTES, remaining),
            )
            if not chunk:
                break
            observed_size += len(chunk)
            if observed_size > maximum_bytes:
                raise MapAssetBundleValidationError("地图资产文件超过大小上限：%s" % self.relative_path)
            digest.update(chunk)
            chunks.append(chunk)
        self._finish_observation(before, observed_size=observed_size)
        payload = b"".join(chunks)
        sha256 = digest.hexdigest()
        self._record_observation(payload, observed_size, sha256)
        return payload, observed_size, sha256

    def hash(self) -> Tuple[int, str]:
        if self._size is not None and self._sha256 is not None:
            self.verify_named_entry()
            return self._size, self._sha256
        before = self._begin_observation()
        os.lseek(self.descriptor, 0, os.SEEK_SET)
        digest = hashlib.sha256()
        prefix = bytearray()
        observed_size = 0
        while True:
            chunk = os.read(self.descriptor, _FILE_READ_CHUNK_BYTES)
            if not chunk:
                break
            observed_size += len(chunk)
            digest.update(chunk)
            if len(prefix) < _FILE_PREFIX_BYTES:
                prefix.extend(chunk[: _FILE_PREFIX_BYTES - len(prefix)])
        self._finish_observation(before, observed_size=observed_size)
        sha256 = digest.hexdigest()
        self._prefix = bytes(prefix)
        self._record_observation(None, observed_size, sha256)
        return observed_size, sha256

    def read(self, maximum_bytes: int) -> bytes:
        before = self._begin_observation()
        if before.st_size > maximum_bytes:
            raise MapAssetBundleValidationError("file exceeds read budget: %s" % self.relative_path)
        chunks = []
        offset = 0
        while offset < before.st_size:
            chunk = os.pread(self.descriptor, min(_FILE_READ_CHUNK_BYTES, before.st_size - offset), offset)
            if not chunk:
                raise MapAssetBundleValidationError("short file read: %s" % self.relative_path)
            chunks.append(chunk)
            offset += len(chunk)
        self._finish_observation(before, observed_size=offset)
        return b"".join(chunks)

    def slice(self, start: int, stop: int) -> bytes:
        before = self._begin_observation()
        stop = min(stop, before.st_size)
        payload = os.pread(self.descriptor, max(0, stop - start), start)
        if len(payload) != max(0, stop - start):
            raise MapAssetBundleValidationError("short file slice: %s" % self.relative_path)
        self._finish_observation(before, observed_size=before.st_size)
        return payload

    def prefix(self, size: int) -> bytes:
        return self.slice(0, size)

    def size(self) -> int:
        current = self._begin_observation()
        self.verify_named_entry()
        return int(current.st_size)

    def streaming_equals(self, other: _ManagedRegularFile) -> bool:
        size = self.size()
        if size != other.size():
            return False
        for offset in range(0, size, _FILE_READ_CHUNK_BYTES):
            if self.slice(offset, offset + _FILE_READ_CHUNK_BYTES) != other.slice(offset, offset + _FILE_READ_CHUNK_BYTES):
                return False
        self.verify_named_entry()
        other.verify_named_entry()
        return True

    def _record_observation(
        self,
        payload: Optional[bytes],
        size: int,
        sha256: str,
    ) -> None:
        if self._size is not None and self._size != size:
            raise MapAssetBundleValidationError("地图资产文件重复读取大小不一致：%s" % self.relative_path)
        if self._sha256 is not None and self._sha256 != sha256:
            raise MapAssetBundleValidationError("地图资产文件重复读取摘要不一致：%s" % self.relative_path)
        self._size = size
        self._sha256 = sha256
        if payload is not None:
            self._payload = payload
            self._prefix = payload[:_FILE_PREFIX_BYTES]


class _ManagedDirectory:
    def __init__(
        self,
        *,
        descriptor: int,
        parent_descriptor: int,
        name: str,
        relative_path: str,
        initial_stat: os.stat_result,
    ) -> None:
        self.descriptor = descriptor
        self.parent_descriptor = parent_descriptor
        self.name = name
        self.relative_path = relative_path
        self.initial_stat = initial_stat

    def close(self) -> None:
        if self.descriptor >= 0:
            os.close(self.descriptor)
            self.descriptor = -1

    def verify_named_entry(self) -> None:
        try:
            named = os.stat(
                self.name,
                dir_fd=self.parent_descriptor,
                follow_symlinks=False,
            )
        except OSError as exc:
            raise MapAssetBundleValidationError("地图资产目录命名项不可验证：%s" % self.relative_path) from exc
        current = os.fstat(self.descriptor)
        _validate_directory_stat(named, self.relative_path)
        _validate_directory_stat(current, self.relative_path)
        _require_same_file_identity(current, named, self.relative_path)
        _require_same_directory_state(self.initial_stat, current, self.relative_path)

    def refresh_after_rename(self, name: str, relative_path: str) -> None:
        self.name = name
        self.relative_path = relative_path
        self.initial_stat = os.fstat(self.descriptor)
        self.verify_named_entry()


class _StableDirectoryTree:
    def __init__(self, directory: Path) -> None:
        self.path = Path(directory).absolute()
        self.parent_descriptor = -1
        self.root: Optional[_ManagedDirectory] = None
        self.directories: List[_ManagedDirectory] = []
        self.files: Dict[str, _ManagedRegularFile] = {}
        self._entry_count = 0
        self._open()

    def __enter__(self) -> "_StableDirectoryTree":
        return self

    def __exit__(self, _exc_type, _exc, _traceback) -> None:
        self.close()

    def close(self) -> None:
        for managed_file in self.files.values():
            managed_file.close()
        self.files.clear()
        for directory in reversed(self.directories):
            directory.close()
        self.directories.clear()
        self.root = None
        if self.parent_descriptor >= 0:
            os.close(self.parent_descriptor)
            self.parent_descriptor = -1

    def require_file(self, relative_path: str, description: str) -> _ManagedRegularFile:
        normalized = _validate_relative_path(relative_path, description)
        managed = self.files.get(normalized)
        if managed is None:
            raise MapAssetBundleValidationError("%s不存在：%s" % (description, normalized))
        return managed

    def verify_all(self) -> None:
        for managed_file in self.files.values():
            managed_file.verify_named_entry()
        for directory in reversed(self.directories):
            directory.verify_named_entry()
        if self.root is None:
            raise MapAssetBundleValidationError("地图资产目录描述符已关闭。")
        try:
            absolute_stat = os.stat(self.path, follow_symlinks=False)
        except OSError as exc:
            raise MapAssetBundleValidationError("地图资产目录路径不可验证。") from exc
        _require_same_file_identity(
            os.fstat(self.root.descriptor),
            absolute_stat,
            str(self.path),
        )

    def rebind_root_after_rename(self, name: str) -> None:
        if self.root is None:
            raise MapAssetBundleValidationError("地图资产目录描述符已关闭。")
        self.path = self.path.with_name(name)
        self.root.refresh_after_rename(name, name)
        self.verify_all()

    def _open(self) -> None:
        if not self.path.name or self.path.name in {".", ".."}:
            raise MapAssetBundleValidationError("地图资产目录名称非法。")
        directory_flags = (
            os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_CLOEXEC", 0)
        )
        root_descriptor = -1
        try:
            self.parent_descriptor = os.open(self.path.parent, directory_flags)
            named = os.stat(
                self.path.name,
                dir_fd=self.parent_descriptor,
                follow_symlinks=False,
            )
            root_descriptor = os.open(
                self.path.name,
                directory_flags,
                dir_fd=self.parent_descriptor,
            )
            opened = os.fstat(root_descriptor)
            _validate_directory_stat(named, str(self.path))
            _validate_directory_stat(opened, str(self.path))
            _require_same_file_identity(named, opened, str(self.path))
            root = _ManagedDirectory(
                descriptor=root_descriptor,
                parent_descriptor=self.parent_descriptor,
                name=self.path.name,
                relative_path=self.path.name,
                initial_stat=opened,
            )
            self.root = root
            self.directories.append(root)
            root_descriptor = -1
            self._scan_directory(root, PurePosixPath())
        except Exception:
            if root_descriptor >= 0:
                os.close(root_descriptor)
            self.close()
            raise

    def _scan_directory(
        self,
        directory: _ManagedDirectory,
        relative_parent: PurePosixPath,
    ) -> None:
        directory_flags = (
            os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_CLOEXEC", 0)
        )
        file_flags = (
            os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NONBLOCK", 0)
        )
        with os.scandir(directory.descriptor) as iterator:
            entries = sorted(iterator, key=lambda entry: entry.name)
        for entry in entries:
            self._entry_count += 1
            if self._entry_count > _MAX_DIRECTORY_ENTRIES:
                raise MapAssetBundleValidationError("地图资产目录项数量超过上限。")
            relative = relative_parent / entry.name
            relative_text = relative.as_posix()
            entry_stat = entry.stat(follow_symlinks=False)
            if stat.S_ISLNK(entry_stat.st_mode):
                raise MapAssetBundleValidationError("地图资产包不得包含符号链接：%s" % relative_text)
            if stat.S_ISDIR(entry_stat.st_mode):
                child_descriptor = os.open(
                    entry.name,
                    directory_flags,
                    dir_fd=directory.descriptor,
                )
                try:
                    child_stat = os.fstat(child_descriptor)
                    _validate_directory_stat(child_stat, relative_text)
                    _require_same_file_identity(entry_stat, child_stat, relative_text)
                except Exception:
                    os.close(child_descriptor)
                    raise
                child = _ManagedDirectory(
                    descriptor=child_descriptor,
                    parent_descriptor=directory.descriptor,
                    name=entry.name,
                    relative_path=relative_text,
                    initial_stat=child_stat,
                )
                self.directories.append(child)
                self._scan_directory(child, relative)
                continue
            if not stat.S_ISREG(entry_stat.st_mode):
                raise MapAssetBundleValidationError("地图资产包包含非普通文件：%s" % relative_text)
            descriptor = os.open(
                entry.name,
                file_flags,
                dir_fd=directory.descriptor,
            )
            try:
                opened_stat = os.fstat(descriptor)
                _validate_regular_file_stat(opened_stat, relative_text)
                _require_same_file_identity(entry_stat, opened_stat, relative_text)
            except Exception:
                os.close(descriptor)
                raise
            self.files[relative_text] = _ManagedRegularFile(
                descriptor=descriptor,
                parent_descriptor=directory.descriptor,
                name=entry.name,
                relative_path=relative_text,
                initial_stat=opened_stat,
            )


def _validated_snapshot_fingerprint(
    tree: _StableDirectoryTree,
    bundle: MapAssetBundle,
    georeference_bytes: bytes,
) -> str:
    """绑定一次完整验证所得的类型化合同、原文与全部目录成员观察。"""

    digest = hashlib.sha256()
    digest.update(_VALIDATED_SNAPSHOT_DOMAIN)
    canonical_bundle = json.dumps(
        bundle.model_dump(mode="json"),
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    for field in (canonical_bundle, georeference_bytes):
        digest.update(len(field).to_bytes(8, "big"))
        digest.update(field)
    for relative_path in sorted(tree.files):
        managed = tree.files[relative_path]
        size, sha256 = managed.hash()
        current = os.fstat(managed.descriptor)
        metadata = json.dumps(
            {
                "device": int(current.st_dev),
                "inode": int(current.st_ino),
                "mode": int(current.st_mode),
                "nlink": int(current.st_nlink),
                "size": size,
                "mtime_ns": int(current.st_mtime_ns),
                "ctime_ns": int(current.st_ctime_ns),
                "sha256": sha256,
            },
            sort_keys=True,
            separators=(",", ":"),
        ).encode("ascii")
        path_bytes = relative_path.encode("utf-8")
        digest.update(len(path_bytes).to_bytes(8, "big"))
        digest.update(path_bytes)
        digest.update(len(metadata).to_bytes(8, "big"))
        digest.update(metadata)
    tree.verify_all()
    return digest.hexdigest()


def _validate_regular_file_stat(path_stat: os.stat_result, description: str) -> None:
    if not stat.S_ISREG(path_stat.st_mode):
        raise MapAssetBundleValidationError("%s必须是普通文件。" % description)
    if int(path_stat.st_nlink) != 1:
        raise MapAssetBundleValidationError("%s必须只有一个硬链接。" % description)


def _validate_directory_stat(path_stat: os.stat_result, description: str) -> None:
    if not stat.S_ISDIR(path_stat.st_mode):
        raise MapAssetBundleValidationError("%s必须是真实目录。" % description)


def _require_same_file_identity(
    expected: os.stat_result,
    actual: os.stat_result,
    description: str,
) -> None:
    if (int(expected.st_dev), int(expected.st_ino)) != (
        int(actual.st_dev),
        int(actual.st_ino),
    ):
        raise MapAssetBundleValidationError("%s在校验期间被替换。" % description)


def _require_same_file_state(
    expected: os.stat_result,
    actual: os.stat_result,
    description: str,
) -> None:
    _require_same_file_identity(expected, actual, description)
    expected_state = (
        int(expected.st_nlink),
        int(expected.st_size),
        int(expected.st_mtime_ns),
        int(expected.st_ctime_ns),
    )
    actual_state = (
        int(actual.st_nlink),
        int(actual.st_size),
        int(actual.st_mtime_ns),
        int(actual.st_ctime_ns),
    )
    if expected_state != actual_state:
        raise MapAssetBundleValidationError("%s在校验期间发生变化。" % description)


def _require_same_directory_state(
    expected: os.stat_result,
    actual: os.stat_result,
    description: str,
) -> None:
    _require_same_file_identity(expected, actual, description)
    expected_state = (
        int(expected.st_nlink),
        int(expected.st_mtime_ns),
        int(expected.st_ctime_ns),
    )
    actual_state = (
        int(actual.st_nlink),
        int(actual.st_mtime_ns),
        int(actual.st_ctime_ns),
    )
    if expected_state != actual_state:
        raise MapAssetBundleValidationError("%s在校验期间发生变化。" % description)


def _validate_relative_path(value: object, description: str) -> str:
    raw = str(value or "").strip()
    if not raw or "\\" in raw:
        raise MapAssetBundleValidationError("%s必须是规范的 POSIX 相对路径。" % description)
    path = PurePosixPath(raw)
    if path.is_absolute() or raw != path.as_posix() or any(part in {"", ".", ".."} for part in path.parts):
        raise MapAssetBundleValidationError("%s必须位于地图资产包内。" % description)
    return path.as_posix()


def _contained_path(root: Path, relative: object) -> Path:
    normalized = _validate_relative_path(relative, "地图资产路径")
    candidate = root.joinpath(*PurePosixPath(normalized).parts)
    try:
        candidate.absolute().relative_to(root.absolute())
    except ValueError as exc:
        raise MapAssetBundleValidationError("地图资产路径越过资产包目录。") from exc
    return candidate


def _read_regular_file(
    path: Path,
    description: str,
    *,
    maximum_bytes: Optional[int] = None,
) -> bytes:
    descriptor, expected_size = _open_regular_file(path, description)
    try:
        before = os.fstat(descriptor)
        if maximum_bytes is not None and expected_size > maximum_bytes:
            raise MapAssetBundleValidationError("%s超过大小上限。" % description)
        chunks = []
        observed_size = 0
        while True:
            read_size = _FILE_READ_CHUNK_BYTES
            if maximum_bytes is not None:
                remaining = maximum_bytes + 1 - observed_size
                if remaining <= 0:
                    raise MapAssetBundleValidationError("%s超过大小上限。" % description)
                read_size = min(read_size, remaining)
            chunk = os.read(descriptor, read_size)
            if not chunk:
                break
            observed_size += len(chunk)
            if maximum_bytes is not None and observed_size > maximum_bytes:
                raise MapAssetBundleValidationError("%s超过大小上限。" % description)
            chunks.append(chunk)
        after = os.fstat(descriptor)
        _validate_regular_file_stat(after, description)
        _require_same_file_state(before, after, description)
        if observed_size != expected_size:
            raise MapAssetBundleValidationError("%s在校验期间发生变化。" % description)
        try:
            named = os.stat(path, follow_symlinks=False)
        except OSError as exc:
            raise MapAssetBundleValidationError("%s命名项不可验证。" % description) from exc
        _validate_regular_file_stat(named, description)
        _require_same_file_identity(after, named, description)
        return b"".join(chunks)
    finally:
        os.close(descriptor)


def _open_regular_file(path: Path, description: str) -> Tuple[int, int]:
    try:
        descriptor = os.open(
            path,
            os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NONBLOCK", 0),
        )
    except FileNotFoundError as exc:
        raise MapAssetBundleValidationError("%s不存在：%s" % (description, path)) from exc
    except OSError as exc:
        raise MapAssetBundleValidationError("无法打开%s：%s" % (description, path)) from exc
    try:
        path_stat = os.fstat(descriptor)
        _validate_regular_file_stat(path_stat, description)
    except Exception:
        os.close(descriptor)
        raise
    return descriptor, int(path_stat.st_size)


def _regular_file_size(path: Path, description: str) -> int:
    descriptor, size = _open_regular_file(path, description)
    os.close(descriptor)
    return size


def _hash_regular_file(path: Path, description: str) -> Tuple[int, str]:
    descriptor, expected_size = _open_regular_file(path, description)
    digest = hashlib.sha256()
    observed_size = 0
    try:
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            observed_size += len(chunk)
            digest.update(chunk)
    finally:
        os.close(descriptor)
    if observed_size != expected_size:
        raise MapAssetBundleValidationError("%s在校验期间发生变化：%s" % (description, path))
    return observed_size, digest.hexdigest()


def _read_regular_file_prefix(path: Path, description: str, size: int) -> bytes:
    descriptor, _expected_size = _open_regular_file(path, description)
    try:
        return os.read(descriptor, max(0, int(size)))
    finally:
        os.close(descriptor)


def _parse_json_object(payload: bytes, description: str) -> Dict[str, object]:
    def reject_duplicates(pairs):
        result: Dict[str, object] = {}
        for key, value in pairs:
            if key in result:
                raise MapAssetBundleValidationError("%s 包含重复 JSON key：%s" % (description, key))
            result[key] = value
        return result

    def reject_constant(value: str) -> object:
        raise MapAssetBundleValidationError("%s 包含非有限 JSON 数值：%s" % (description, value))

    def parse_integer(value: str) -> int:
        if len(value) > _MAX_JSON_NUMBER_TOKEN_BYTES:
            raise MapAssetBundleValidationError("%s 包含过长 JSON 整数。" % description)
        if value == "-0":
            raise MapAssetBundleValidationError("%s 包含整数形式的负零。" % description)
        return int(value)

    def parse_float(value: str) -> float:
        if len(value) > _MAX_JSON_NUMBER_TOKEN_BYTES:
            raise MapAssetBundleValidationError("%s 包含过长 JSON 浮点数。" % description)
        parsed = float(value)
        if parsed == 0.0 and value.startswith("-"):
            raise MapAssetBundleValidationError("%s 包含负零浮点数。" % description)
        return parsed

    try:
        value = json.loads(
            payload.decode("utf-8"),
            object_pairs_hook=reject_duplicates,
            parse_constant=reject_constant,
            parse_int=parse_integer,
            parse_float=parse_float,
        )
    except MapAssetBundleValidationError:
        raise
    except Exception as exc:
        raise MapAssetBundleValidationError("%s 不是有效 JSON。" % description) from exc
    if not isinstance(value, dict):
        raise MapAssetBundleValidationError("%s 必须是 JSON object。" % description)
    return value


def _validate_sha256(value: object, description: str) -> str:
    resolved = str(value or "").strip().lower()
    if len(resolved) != 64 or any(char not in "0123456789abcdef" for char in resolved):
        raise MapAssetBundleValidationError("%s非法。" % description)
    return resolved


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _canonical_json_bytes(document: Mapping[str, object]) -> bytes:
    return (
        json.dumps(document, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n"
    ).encode("utf-8")


def _model_json_bytes(model) -> bytes:
    return _canonical_json_bytes(model.model_dump(mode="json", exclude_none=True))


def _validate_manifested_file_observation(
    relative_path: str,
    metadata: Optional[Mapping[str, object]],
    observed_size: int,
    observed_sha256: str,
) -> None:
    if metadata is None:
        raise MapAssetBundleValidationError("地图资产文件未被顶层清单绑定：%s" % relative_path)
    if observed_size != metadata.get("size") or observed_sha256 != metadata.get("sha256"):
        raise MapAssetBundleValidationError("地图资产文件大小或 SHA-256 不匹配：%s" % relative_path)


def _entry_exists_at(parent_descriptor: int, name: str) -> bool:
    try:
        os.stat(name, dir_fd=parent_descriptor, follow_symlinks=False)
    except FileNotFoundError:
        return False
    return True


def _exchange_directories(parent_descriptor, source_name, target_name):
    if any(not name or "/" in name or name in {".", ".."} for name in (source_name, target_name)):
        raise MapAssetBundleValidationError("二维资产交换名称非法")
    libc = ctypes.CDLL(None, use_errno=True)
    function = getattr(libc, "renameat2", None)
    if function is None:
        raise OSError(errno.ENOTSUP, "二维资产原子交换要求renameat2")
    function.argtypes = (ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint)
    function.restype = ctypes.c_int
    if function(parent_descriptor, os.fsencode(source_name), parent_descriptor, os.fsencode(target_name), 2) != 0:
        number = ctypes.get_errno()
        raise OSError(number, os.strerror(number))


def _rename_directory_noreplace(
    parent_descriptor: int,
    source_name: str,
    target_name: str,
    *,
    target_parent_descriptor: Optional[int] = None,
) -> None:
    if (
        not source_name
        or not target_name
        or "/" in source_name
        or "/" in target_name
        or source_name in {".", ".."}
        or target_name in {".", ".."}
    ):
        raise MapAssetBundleValidationError("地图资产目录提交名称非法。")
    libc = ctypes.CDLL(None, use_errno=True)
    renameat2 = getattr(libc, "renameat2", None)
    if renameat2 is None:
        raise OSError(errno.ENOTSUP, "renameat2 不可用，拒绝降级为可覆盖提交")
    renameat2.argtypes = (
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_uint,
    )
    renameat2.restype = ctypes.c_int
    result = renameat2(
        parent_descriptor,
        os.fsencode(source_name),
        parent_descriptor if target_parent_descriptor is None else target_parent_descriptor,
        os.fsencode(target_name),
        1,
    )
    if result != 0:
        error_number = ctypes.get_errno()
        raise OSError(error_number, os.strerror(error_number))


def _atomic_write_relative(
    root: Path,
    relative_path: str,
    payload: bytes,
    *,
    create_parents: bool = False,
) -> None:
    normalized = _validate_relative_path(relative_path, "地图资产写入路径")
    parts = PurePosixPath(normalized).parts
    directory_flags = (
        os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_CLOEXEC", 0)
    )
    descriptors: List[int] = []
    directory_states: List[Tuple[int, int, str, os.stat_result]] = []
    temporary_name = ".%s.%s.tmp" % (parts[-1], uuid.uuid4().hex)
    temporary_descriptor = -1
    try:
        current_descriptor = os.open(Path(root).absolute(), directory_flags)
        descriptors.append(current_descriptor)
        for part in parts[:-1]:
            try:
                named = os.stat(
                    part,
                    dir_fd=current_descriptor,
                    follow_symlinks=False,
                )
            except FileNotFoundError as exc:
                if not create_parents:
                    raise MapAssetBundleValidationError("地图资产写入目录不存在：%s" % part) from exc
                os.mkdir(part, mode=0o700, dir_fd=current_descriptor)
                named = os.stat(
                    part,
                    dir_fd=current_descriptor,
                    follow_symlinks=False,
                )
            _validate_directory_stat(named, part)
            child_descriptor = os.open(
                part,
                directory_flags,
                dir_fd=current_descriptor,
            )
            try:
                opened = os.fstat(child_descriptor)
                _validate_directory_stat(opened, part)
                _require_same_file_identity(named, opened, part)
            except Exception:
                os.close(child_descriptor)
                raise
            directory_states.append((current_descriptor, child_descriptor, part, opened))
            descriptors.append(child_descriptor)
            current_descriptor = child_descriptor

        temporary_descriptor = os.open(
            temporary_name,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_CLOEXEC", 0),
            0o600,
            dir_fd=current_descriptor,
        )
        view = memoryview(payload)
        written = 0
        while written < len(view):
            count = os.write(temporary_descriptor, view[written:])
            if count <= 0:
                raise OSError(errno.EIO, "地图资产临时文件写入失败")
            written += count
        os.fsync(temporary_descriptor)
        temporary_stat = os.fstat(temporary_descriptor)
        _validate_regular_file_stat(temporary_stat, normalized)
        os.close(temporary_descriptor)
        temporary_descriptor = -1
        os.replace(
            temporary_name,
            parts[-1],
            src_dir_fd=current_descriptor,
            dst_dir_fd=current_descriptor,
        )
        final_stat = os.stat(
            parts[-1],
            dir_fd=current_descriptor,
            follow_symlinks=False,
        )
        _validate_regular_file_stat(final_stat, normalized)
        _require_same_file_identity(temporary_stat, final_stat, normalized)
        os.fsync(current_descriptor)
        for parent_descriptor, child_descriptor, part, opened in reversed(directory_states):
            named = os.stat(
                part,
                dir_fd=parent_descriptor,
                follow_symlinks=False,
            )
            _validate_directory_stat(named, part)
            _require_same_file_identity(opened, named, part)
            _require_same_file_identity(opened, os.fstat(child_descriptor), part)
    finally:
        if temporary_descriptor >= 0:
            os.close(temporary_descriptor)
        if descriptors:
            try:
                os.unlink(temporary_name, dir_fd=descriptors[-1])
            except FileNotFoundError:
                pass
            except OSError:
                pass
        for descriptor in reversed(descriptors):
            os.close(descriptor)


def _atomic_write_bytes(path: Path, payload: bytes) -> None:
    temporary = path.with_name(".%s.%s.tmp" % (path.name, uuid.uuid4().hex))
    descriptor = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    try:
        with os.fdopen(descriptor, "wb", closefd=True) as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        descriptor = -1
        os.replace(temporary, path)
        _fsync_directory(path.parent)
    finally:
        if descriptor >= 0:
            os.close(descriptor)
        try:
            temporary.unlink(missing_ok=True)
        except OSError:
            pass


def _fsync_tree(tree: _StableDirectoryTree) -> None:
    for managed_file in tree.files.values():
        managed_file.verify_named_entry()
        os.fsync(managed_file.descriptor)
        managed_file.verify_named_entry()
    for directory in reversed(tree.directories):
        directory.verify_named_entry()
        os.fsync(directory.descriptor)
        directory.verify_named_entry()
    tree.verify_all()


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _require_typed_asset_identity(value, *, optional=False):
    if value is None and optional:
        return
    if type(value) is not AssetIdentityV2:
        raise MapAssetBundleValidationError("expected a typed AssetIdentityV2 owner, not a digest/string")


def _assigned_observations(tree):
    observations = []
    for relative, managed in sorted(tree.files.items()):
        managed.verify_named_entry()
        state = os.fstat(managed.descriptor)
        observations.append((relative, state.st_dev, state.st_ino, state.st_mode, state.st_nlink,
            state.st_size, state.st_mtime_ns, state.st_ctime_ns))
    tree.verify_all()
    return tuple(observations)


def _remove_pinned_directory_contents(directory_fd):
    """Retire children only through an already owned, quarantined directory FD."""
    with os.scandir(directory_fd) as iterator:
        entries = list(iterator)
    flags = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC
    for entry in entries:
        named = entry.stat(follow_symlinks=False)
        if stat.S_ISDIR(named.st_mode):
            child_fd = os.open(entry.name, flags, dir_fd=directory_fd)
            try:
                _require_same_file_identity(named, os.fstat(child_fd), entry.name)
                _remove_pinned_directory_contents(child_fd)
                _require_same_file_identity(named,
                    os.stat(entry.name, dir_fd=directory_fd, follow_symlinks=False), entry.name)
                os.rmdir(entry.name, dir_fd=directory_fd)
            finally:
                os.close(child_fd)
        else:
            # Symlinks are unlinked as names, never followed.
            _require_same_file_identity(named,
                os.stat(entry.name, dir_fd=directory_fd, follow_symlinks=False), entry.name)
            os.unlink(entry.name, dir_fd=directory_fd)
