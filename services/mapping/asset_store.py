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
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Callable, Dict, Iterable, List, Mapping, Optional, Tuple
from urllib.parse import quote

from contracts.map_workspace import (
    MAP_GEOREFERENCE_RELATIVE_PATH,
    MapAssetBundle,
    MapGeometryAssetBinding,
    MapGeometryAssetType,
    MapGeoreferenceBindingV1,
    MapGeoreferenceV1,
)
from contracts.naming import build_safe_dir_segment
from contracts.runtime_views import map_content_sha256

_BUNDLE_FILE_NAME = "bundle.json"
_MANIFEST_FILE_NAME = "manifest.json"
_MANIFEST_SCHEMA_VERSION = 1
_STAGING_PREFIX = ".current.tmp-"
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


class MapAlreadyFinalizedError(MapAssetBundleError):
    """A map already has a different immutable current bundle."""

    def __init__(self, map_name: str, existing_sha256: str, candidate_sha256: str) -> None:
        self.map_name = map_name
        self.existing_sha256 = existing_sha256
        self.candidate_sha256 = candidate_sha256
        super().__init__("地图 %s 已 finalized；修改地图必须使用新的 map_name。" % map_name)


class MapAssetMigrationError(MapAssetBundleError):
    """Legacy version storage cannot be migrated without guessing."""


@dataclass(frozen=True)
class ValidatedMapAssetBundleSnapshot:
    """一次稳定目录读取中验证完成的 finalized 资产与地理参考原文。"""

    bundle: MapAssetBundle
    georeference_bytes: bytes
    repository_device: int
    repository_inode: int
    repository_fingerprint: str


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

    def create_staging_directory(self, map_name: str) -> Path:
        map_dir = self._map_dir(map_name)
        map_dir.mkdir(parents=True, exist_ok=True)
        self._require_real_directory(map_dir, "地图目录")
        staging_dir = map_dir / (_STAGING_PREFIX + uuid.uuid4().hex)
        staging_dir.mkdir(mode=0o700)
        _fsync_directory(map_dir)
        return staging_dir

    def discard_staging_directory(self, map_name: str, staging_dir: Path) -> None:
        resolved = self._require_staging_directory(map_name, staging_dir, must_exist=False)
        if resolved.is_symlink():
            resolved.unlink()
        elif resolved.exists():
            shutil.rmtree(resolved)

    def get_map_asset_bundle(
        self,
        map_name: str,
        *,
        migrate_legacy: bool = True,
    ) -> Optional[MapAssetBundle]:
        current_dir = self._current_dir(map_name)
        if current_dir.exists() or current_dir.is_symlink():
            return self._load_validated_bundle(map_name, current_dir)
        if not migrate_legacy:
            return None
        return self._migrate_legacy_latest(map_name)

    def get_validated_map_asset_bundle_snapshot(
        self,
        map_name: str,
        *,
        expected_manifest_sha256: str,
    ) -> Optional[ValidatedMapAssetBundleSnapshot]:
        """只读加载 modern current，并在同一稳定目录视图中返回地理参考原文。"""

        current_dir = self._current_dir(map_name)
        if not current_dir.exists() and not current_dir.is_symlink():
            return None
        with _StableDirectoryTree(current_dir) as tree:
            manifest_sha256, bundle = self._validate_open_directory(
                map_name,
                tree,
                expected_manifest_sha256=expected_manifest_sha256,
            )
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
        return ValidatedMapAssetBundleSnapshot(
            bundle=validated_bundle,
            georeference_bytes=georeference_bytes,
            repository_device=repository_stat.st_dev,
            repository_inode=repository_stat.st_ino,
            repository_fingerprint=repository_fingerprint,
        )

    def validate_map_asset_bundle_snapshot_identity(
        self,
        map_name: str,
        snapshot: ValidatedMapAssetBundleSnapshot,
    ) -> None:
        """从可信 current 命名项重建快照，并拒绝解析期间的任何变化。"""

        refreshed = self.get_validated_map_asset_bundle_snapshot(
            map_name,
            expected_manifest_sha256=str(snapshot.bundle.manifest_sha256 or ""),
        )
        if refreshed is None or refreshed != snapshot:
            raise MapAssetBundleValidationError("finalized 地图 repository snapshot 已被替换。")

    def finalize_map_asset_bundle(
        self,
        bundle: MapAssetBundle,
        *,
        staging_dir: Path,
    ) -> MapAssetBundle:
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

    def current_directory(self, map_name: str) -> Path:
        """Return the fixed bundle root; callers must still load through validation."""

        return self._current_dir(map_name)

    def _write_candidate_bundle(
        self,
        bundle: MapAssetBundle,
        staging_dir: Path,
    ) -> MapAssetBundle:
        georeference = bundle.georeference
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
    ) -> MapAssetBundle:
        manifest_sha256, bundle = self._validate_directory(
            map_name,
            directory,
            expected_manifest_sha256=expected_manifest_sha256,
        )
        return bundle.model_copy(update={"manifest_sha256": manifest_sha256}, deep=True)

    def _validate_directory(
        self,
        map_name: str,
        directory: Path,
        *,
        expected_manifest_sha256: Optional[str] = None,
    ) -> Tuple[str, MapAssetBundle]:
        with _StableDirectoryTree(directory) as tree:
            result = self._validate_open_directory(
                map_name,
                tree,
                expected_manifest_sha256=expected_manifest_sha256,
            )
            tree.verify_all()
            return result

    def _validate_open_directory(
        self,
        map_name: str,
        tree: _StableDirectoryTree,
        *,
        expected_manifest_sha256: Optional[str] = None,
    ) -> Tuple[str, MapAssetBundle]:
        manifest_file = tree.require_file(_MANIFEST_FILE_NAME, "地图资产清单")
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
        if not isinstance(georeference_document, dict) or set(georeference_document) != {
            "schema_version",
            "relative_path",
            "raw_sha256",
            "contract",
        }:
            raise MapAssetBundleValidationError("地图资产包地理参考绑定字段不完整或包含未知字段。")
        try:
            bundle = MapAssetBundle.model_validate(bundle_document)
        except Exception as exc:
            raise MapAssetBundleValidationError("地图资产包契约非法。") from exc
        if bundle.map_name != map_name:
            raise MapAssetBundleValidationError("地图资产包 map_name 不匹配。")
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
        for relative, metadata in expected_files.items():
            if relative in {_BUNDLE_FILE_NAME, bundle.georeference.relative_path}:
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
    ) -> None:
        if geometry_asset is None:
            raise MapAssetBundleValidationError("地图资产包缺少 geometry_asset。")
        referenced = [geometry_asset.mapping_database_path]
        if not geometry_asset.pcd_path:
            raise MapAssetBundleValidationError("地图资产包缺少 PCD 点云资产。")
        referenced.append(geometry_asset.pcd_path)
        referenced.extend(asset.path for asset in geometry_asset.assets)
        for relative in referenced:
            normalized = _validate_relative_path(relative, "几何资产路径")
            if normalized not in manifested_paths:
                raise MapAssetBundleValidationError("几何资产未被顶层清单绑定：%s" % normalized)
            tree.require_file(normalized, "几何资产").size()

        database_file = tree.require_file(
            geometry_asset.mapping_database_path,
            "建图数据库",
        )
        if database_file.size() <= 4096:
            raise MapAssetBundleValidationError("建图数据库缺失或过小。")
        self._validate_pcd(tree.require_file(geometry_asset.pcd_path, "PCD 地图"))
        navigation_assets = [
            asset for asset in geometry_asset.assets if asset.asset_type == MapGeometryAssetType.NAVIGATION_MAP
        ]
        if not navigation_assets:
            raise MapAssetBundleValidationError("地图资产包缺少 Nav2 地图资产。")
        for asset in geometry_asset.assets:
            if asset.asset_type == MapGeometryAssetType.NAVIGATION_MAP:
                self._validate_nav2_yaml(tree, asset.path, manifested_paths)
        self._validate_backend_checkpoint(
            tree,
            geometry_asset,
            manifested_paths,
        )

    def _validate_backend_checkpoint(
        self,
        tree: _StableDirectoryTree,
        geometry_asset: MapGeometryAssetBinding,
        manifested_paths: set[str],
    ) -> None:
        raw_checkpoint = geometry_asset.metadata.get("mapping_backend_checkpoint")
        if not isinstance(raw_checkpoint, dict) or raw_checkpoint.get("present") is not True:
            raise MapAssetBundleValidationError("地图资产包缺少 backend checkpoint。")
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
        header = managed_file.prefix(_FILE_PREFIX_BYTES)
        if b"FIELDS x y z" not in header or b"DATA " not in header:
            raise MapAssetBundleValidationError("PCD 地图头部非法。")

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
            .read_and_hash(_MAX_NAV2_YAML_BYTES)[0]
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
        if not isinstance(georeference_payload, dict) or set(georeference_payload) != {
            "schema_version",
            "relative_path",
            "raw_sha256",
            "contract",
        }:
            raise MapAssetMigrationError("旧地图没有完整地理参考绑定，拒绝推测锚点或坐标变换。")
        try:
            georeference = MapGeoreferenceBindingV1.model_validate(georeference_payload)
            georeference_path = _contained_path(
                source_dir,
                georeference.relative_path,
            )
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
        legacy_epoch_preimage = "%s\0%s\0%s" % (
            map_name,
            legacy_source_name,
            legacy_snapshot_token,
        )
        try:
            legacy_update_sequence = max(
                1,
                int(snapshot_payload.get("revision") or 1),
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
        snapshot_payload["source_epoch"] = "legacy:%s" % hashlib.sha256(
            legacy_epoch_preimage.encode("utf-8")
        ).hexdigest()
        snapshot_payload["update_sequence"] = legacy_update_sequence
        snapshot_payload["metadata"] = self._normalize_legacy_metadata(
            source_dir,
            snapshot_payload.get("metadata") or {},
        )
        canonical_frame_id = snapshot_payload.get("frame_id")
        if not canonical_frame_id:
            for layer_key in ("occupancy_grid", "cost_map", "semantic_map"):
                layer_payload = snapshot_payload.get(layer_key)
                if isinstance(layer_payload, dict) and layer_payload.get("frame_id"):
                    canonical_frame_id = layer_payload.get("frame_id")
                    break
        snapshot_payload["content_sha256"] = map_content_sha256(
            map_name=map_name,
            frame_id=canonical_frame_id,
            occupancy_grid=snapshot_payload.get("occupancy_grid"),
            cost_map=snapshot_payload.get("cost_map"),
            semantic_map=snapshot_payload.get("semantic_map"),
        )
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

    def prefix(self, size: int) -> bytes:
        if self._size is None or self._sha256 is None:
            self.hash()
        if size <= len(self._prefix):
            self.verify_named_entry()
            return self._prefix[:size]
        payload, _size, _sha256 = self.read_and_hash(size)
        return payload[:size]

    def size(self) -> int:
        if self._size is None:
            self.hash()
        self.verify_named_entry()
        return int(self._size or 0)

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


def _rename_directory_noreplace(
    parent_descriptor: int,
    source_name: str,
    target_name: str,
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
        parent_descriptor,
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
