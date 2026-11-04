from __future__ import annotations

import ctypes
import errno
import json
import os
import shlex
import shutil
import stat
import uuid
from dataclasses import dataclass, replace
from pathlib import Path, PurePosixPath
from typing import TYPE_CHECKING, Callable, Dict, List, Mapping, Optional, Tuple

from contracts.map_workspace import (
    AssetIdentityV2,
    MapAssetBundle,
    MapGeometryAssetBinding,
    MapGeometryAssetType,
    MapGeoreferenceV1,
)
from contracts.naming import build_safe_dir_segment

if TYPE_CHECKING:
    from services.geometry.mapping_backend_checkpoint_store import CheckpointMemberIdentity

_BUNDLE_FILE_NAME = "bundle.json"
_MANIFEST_FILE_NAME = "manifest.json"
_STAGING_PREFIX = ".current.tmp-"
_ASSIGNED_EXCHANGE_JOURNAL = ".assigned-exchange-v1.json"
_MAX_GEOREFERENCE_JSON_BYTES = 64 * 1024
_MAX_MANIFEST_JSON_BYTES = 4 * 1024 * 1024
_MAX_BUNDLE_JSON_BYTES = 64 * 1024 * 1024
_MAX_METADATA_JSON_BYTES = 4 * 1024 * 1024
_MAX_CHECKPOINT_JSON_BYTES = 4 * 1024 * 1024
_MAX_NAV2_YAML_BYTES = 4 * 1024 * 1024
_MAX_MANIFEST_FILE_ENTRIES = 8192
_MAX_DIRECTORY_ENTRIES = _MAX_MANIFEST_FILE_ENTRIES + 128
_MAX_JSON_NUMBER_TOKEN_BYTES = 128
_FILE_READ_CHUNK_BYTES = 1024 * 1024
_FILE_PREFIX_BYTES = 8192


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
    """地图已有定稿资产的发布冲突。"""


class AssignedMapAlreadyFinalizedError(MapAlreadyFinalizedError):
    """Assigned publication conflict; IDs never occupy legacy digest fields."""

    def __init__(self, map_name: str, existing_identity: Optional[AssetIdentityV2],
                 candidate_identity: AssetIdentityV2, *, existing_kind: str = "assigned") -> None:
        self.map_name = map_name
        self.existing_asset_identity = existing_identity
        self.candidate_asset_identity = candidate_identity
        self.existing_identity_kind = existing_kind
        MapAssetBundleError.__init__(self, "Map %s already owns a different finalized %s asset" % (map_name, existing_kind))


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
    map_directory: str


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
        expected_asset_identity: Optional[AssetIdentityV2] = None,
        _for_activation: bool = False,
    ) -> Optional[MapAssetBundle]:
        _require_typed_asset_identity(expected_asset_identity, optional=True)
        current_dir = self._current_dir(map_name)
        if current_dir.exists() or current_dir.is_symlink():
            return self._load_validated_bundle(map_name, current_dir, expected_asset_identity=expected_asset_identity,
                _for_activation=_for_activation)
        return None

    def get_validated_map_asset_bundle_snapshot(
        self,
        map_name: str,
        *,
        expected_asset_identity: Optional[AssetIdentityV2] = None,
        _previous: Optional[ValidatedMapAssetBundleSnapshot] = None,
    ) -> Optional[ValidatedMapAssetBundleSnapshot]:
        """只读加载 modern current，并在同一稳定目录视图中返回地理参考原文。"""

        _require_typed_asset_identity(expected_asset_identity, optional=True)
        # 列表请求只保留上一份已验证快照；每次复用仍核对当前目录和成员。
        if (_previous is not None and _previous.bundle.map_name == map_name
                and _previous.bundle.asset_identity == expected_asset_identity
                and self.map_asset_bundle_snapshot_is_current(map_name, _previous)):
            return _previous
        current_dir = self._current_dir(map_name)
        if not current_dir.exists() and not current_dir.is_symlink():
            return None
        with _StableDirectoryTree(current_dir) as tree:
            _unused, bundle = self._validate_open_directory(
                map_name,
                tree,
                expected_asset_identity=expected_asset_identity,
            )
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


    def finalize_assigned_map_asset_bundle(
        self,
        bundle: MapAssetBundle,
        *,
        staging_dir: Path,
        asset_identity: AssetIdentityV2,
        expected_previous_identity: Optional[AssetIdentityV2] = None,
    ) -> MapAssetBundle:
        """发布调用方持有的分配式资产身份。"""
        _require_typed_asset_identity(asset_identity)
        _require_typed_asset_identity(expected_previous_identity, optional=True)
        if (bundle.identity_schema_version != 2 or bundle.asset_identity != asset_identity
                or bundle.asset_state not in {"candidate", "finalized"}):
            raise MapAssetBundleValidationError("candidate does not own the exact assigned asset identity")
        map_name = bundle.map_name
        if expected_previous_identity is not None:
            return self._replace_assigned_cloud_bundle(bundle, staging_dir=staging_dir,
                asset_identity=asset_identity, expected_previous_identity=expected_previous_identity)
        staging = self._require_staging_directory(map_name, staging_dir, must_exist=True)
        owner = self._staging_owners[staging]
        current = self._current_dir(map_name)
        try:
            finalized = bundle.model_copy(update={"asset_state": "finalized"})
            # Validate the source generation/descriptors before any outer body
            # serialization. Checkpoint JSON is small and body reads are deferred.
            with _StableDirectoryTree(staging) as tree:
                source = self._assigned_checkpoint_source(tree, finalized.geometry_asset)
                tree.verify_all()
            self._write_assigned_candidate_bundle(finalized, staging)
            self._write_assigned_manifest(finalized, staging, source)
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
                        prior_manifest = _parse_json_object(previous_tree.require_file(
                            _MANIFEST_FILE_NAME, "current manifest").read(_MAX_MANIFEST_JSON_BYTES), "current manifest")
                        version = prior_manifest.get("schema_version")
                        if type(version) is not int or version != 2:
                            raise MapAssetBundleValidationError("unsupported occupied asset manifest version")
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
        return path.name

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
            if not isinstance(value, str):
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
        if current_owner != record["previous_directory"]:
            raise MapAssetBundleValidationError("assigned exchange current directory changed")
        manifest = _parse_json_object(_read_regular_file(current / _MANIFEST_FILE_NAME,
            "assigned exchange current manifest", maximum_bytes=_MAX_MANIFEST_JSON_BYTES),
            "assigned exchange current manifest")
        # 交换前后 current 的名字不变，清单中的资产标识决定交换是否完成。
        committed = manifest.get("asset_identity") == candidate.model_dump()
        identity = candidate if committed else previous
        if (type(manifest.get("schema_version")) is not int or manifest.get("schema_version") != 2
                or manifest.get("map_name") != map_name
                or manifest.get("asset_identity") != identity.model_dump()
                or self._assigned_directory_owner(current) != current_owner):
            raise MapAssetBundleValidationError("assigned exchange current identity changed")
        if owner[2] not in ((previous, candidate) if committed else (previous,)):
            raise MapAssetBundleValidationError("assigned exchange catalog asset owner changed")
        staging = map_dir / record["staging"]
        if staging.exists() or staging.is_symlink():
            if self._assigned_directory_owner(staging) != record["candidate_directory"]:
                raise MapAssetBundleValidationError("assigned exchange staging directory was replaced; refusing unowned cleanup")
        return AssignedMapAssetExchange(previous, candidate, committed, record["catalog_created_at"],
            record["map_directory"])

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
        if staging.exists():
            shutil.rmtree(staging)
        self._staging_owners.pop(staging.absolute(), None)
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
                            or self._assigned_directory_owner(current.parent) != map_owner):
                        raise MapAssetBundleValidationError("assigned replacement catalog creation owner changed")
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
                    record = {"schema_version": 1, "map_name": map_name,
                        "previous_identity": expected_previous_identity.model_dump(),
                        "candidate_identity": asset_identity.model_dump(), "staging": staging.name,
                        "previous_directory": current.name,
                        "candidate_directory": staging.name,
                        "map_directory": map_owner, "catalog_created_at": captured[1]}
                    preserve_staging = True
                    _atomic_write_relative(current.parent, _ASSIGNED_EXCHANGE_JOURNAL, _canonical_json_bytes(record))
                    self._inject_fault("after_assigned_exchange_journal")
                    candidate_tree.verify_all()
                    previous_tree.verify_all()
                    if (self._assigned_catalog_owner(catalog.get_map_asset(map_name)) != captured
                            or self._assigned_directory_owner(current.parent) != map_owner):
                        raise MapAssetBundleValidationError("assigned replacement catalog creation owner changed")
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

    def _write_assigned_manifest(self, bundle, directory, source):
        with _StableDirectoryTree(directory) as tree:
            entries = [{"path": path, "size": managed.size()}
                       for path, managed in sorted(tree.files.items()) if path != _MANIFEST_FILE_NAME]
            tree.verify_all()
        document = {"schema_version": 2, "map_name": bundle.map_name, "bundle_path": _BUNDLE_FILE_NAME,
            "asset_identity": bundle.asset_identity.model_dump(mode="json"),
            "checkpoint_source": source, "files": entries}
        payload = _canonical_json_bytes(document)
        if len(entries) > _MAX_MANIFEST_FILE_ENTRIES or len(payload) > _MAX_MANIFEST_JSON_BYTES:
            raise MapAssetBundleValidationError("assigned manifest exceeds its budget")
        _atomic_write_relative(directory, _MANIFEST_FILE_NAME, payload)

    def _validate_assigned_open_directory(self, map_name, tree, manifest, *, expected_asset_identity=None,
                                          _for_activation=False):
        fields = {"schema_version", "map_name", "bundle_path", "asset_identity", "checkpoint_source", "files"}
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
                or document.get("asset_state") != "finalized"):
            raise MapAssetBundleValidationError("assigned bundle has missing or mixed-generation identity")
        try:
            bundle = MapAssetBundle.model_validate(document)
        except Exception as exc:
            raise MapAssetBundleValidationError("invalid assigned bundle contract") from exc
        if bundle.map_name != map_name:
            raise MapAssetBundleValidationError("assigned bundle map mismatch")
        source = self._assigned_checkpoint_source(tree, bundle.geometry_asset)
        if source != manifest["checkpoint_source"]:
            raise MapAssetBundleValidationError("assigned checkpoint source/member descriptors changed")
        geometry = bundle.geometry_asset
        geometry_path = PurePosixPath("geometry", build_safe_dir_segment(geometry.geometry_stack), "geometry_asset.json").as_posix()
        observed_geometry = _parse_json_object(tree.require_file(geometry_path, "geometry descriptor").read(_MAX_METADATA_JSON_BYTES), "geometry descriptor")
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
        self._validate_geometry_assets(tree, geometry, set(files))
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



    def current_directory(self, map_name: str) -> Path:
        """Resolve the committed bundle root; callers still load through validation."""

        return self._current_dir(map_name)




    def _load_validated_bundle(
        self,
        map_name: str,
        directory: Path,
        *,
        expected_asset_identity: Optional[AssetIdentityV2] = None,
        _for_activation: bool = False,
    ) -> MapAssetBundle:
        _unused, bundle = self._validate_directory(
            map_name,
            directory,
            expected_asset_identity=expected_asset_identity,
            _for_activation=_for_activation,
        )
        return bundle

    def _validate_directory(
        self,
        map_name: str,
        directory: Path,
        *,
        expected_asset_identity: Optional[AssetIdentityV2] = None,
        _for_activation: bool = False,
    ) -> Tuple[Optional[str], MapAssetBundle]:
        with _StableDirectoryTree(directory) as tree:
            result = self._validate_open_directory(
                map_name,
                tree,
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
        expected_asset_identity: Optional[AssetIdentityV2] = None,
        _for_activation: bool = False,
    ) -> Tuple[Optional[str], MapAssetBundle]:
        _require_typed_asset_identity(expected_asset_identity, optional=True)
        manifest_file = tree.require_file(_MANIFEST_FILE_NAME, "地图资产清单")
        header = _parse_json_object(manifest_file.read(_MAX_MANIFEST_JSON_BYTES), "地图资产清单")
        if type(header.get("schema_version")) is not int or header["schema_version"] != 2:
            raise MapAssetBundleValidationError("missing or unsupported explicit manifest schema")
        return None, self._validate_assigned_open_directory(map_name, tree, header,
            expected_asset_identity=expected_asset_identity, _for_activation=_for_activation)

    def _validate_geometry_assets(
        self,
        tree: _StableDirectoryTree,
        geometry_asset: Optional[MapGeometryAssetBinding],
        manifested_paths: set[str],
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


    def _current_dir(self, map_name: str) -> Path:
        return self._map_dir(map_name) / "current"


    def _map_dir(self, map_name: str) -> Path:
        return self._root_dir / build_safe_dir_segment(str(map_name or "").strip())

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






def _canonical_json_bytes(document: Mapping[str, object]) -> bytes:
    return (
        json.dumps(document, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n"
    ).encode("utf-8")


def _model_json_bytes(model) -> bytes:
    return _canonical_json_bytes(model.model_dump(mode="json", exclude_none=True))




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
