from __future__ import annotations

import errno
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import shlex
import shutil
import stat
from typing import Callable, Dict, Iterable, List, Mapping, Optional, Tuple
from urllib.parse import quote
import uuid

from contracts.map_workspace import MapAssetBundle, MapGeometryAssetBinding, MapGeometryAssetType
from contracts.naming import build_safe_dir_segment


_BUNDLE_FILE_NAME = "bundle.json"
_MANIFEST_FILE_NAME = "manifest.json"
_MANIFEST_SCHEMA_VERSION = 1
_STAGING_PREFIX = ".current.tmp-"


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
            self._validate_directory(map_name, staging, expected_manifest_sha256=candidate_sha256)
            _fsync_tree(staging)
            self._inject_fault("before_rename")

            if current.exists() or current.is_symlink():
                return self._resolve_existing_current(
                    map_name,
                    current,
                    staging,
                    candidate_sha256,
                )
            try:
                os.rename(staging, current)
            except OSError as exc:
                if exc.errno not in {errno.EEXIST, errno.ENOTEMPTY} and not current.exists():
                    raise
                return self._resolve_existing_current(
                    map_name,
                    current,
                    staging,
                    candidate_sha256,
                )
            _fsync_directory(current.parent)
            self._inject_fault("after_rename")
            return self._load_validated_bundle(
                map_name,
                current,
                expected_manifest_sha256=candidate_sha256,
            )
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
            geometry_path = _contained_path(staging_dir, geometry_relative)
            geometry_path.parent.mkdir(parents=True, exist_ok=True)
            _atomic_write_bytes(geometry_path, _model_json_bytes(geometry_asset))
        candidate = bundle.model_copy(
            update={"geometry_asset": geometry_asset, "manifest_sha256": None},
            deep=True,
        )
        _atomic_write_bytes(staging_dir / _BUNDLE_FILE_NAME, _model_json_bytes(candidate))
        return candidate

    def _write_manifest(self, map_name: str, directory: Path) -> bytes:
        files = []
        for relative_path, absolute_path in self._regular_files(directory):
            if relative_path == _MANIFEST_FILE_NAME:
                continue
            size, sha256 = _hash_regular_file(
                absolute_path,
                "地图资产文件",
            )
            files.append(
                {
                    "path": relative_path,
                    "size": size,
                    "sha256": sha256,
                }
            )
        files.sort(key=lambda item: str(item["path"]))
        document = {
            "schema_version": _MANIFEST_SCHEMA_VERSION,
            "map_name": map_name,
            "bundle_path": _BUNDLE_FILE_NAME,
            "files": files,
        }
        payload = _canonical_json_bytes(document)
        _atomic_write_bytes(directory / _MANIFEST_FILE_NAME, payload)
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
        self._require_real_directory(directory, "地图资产包目录")
        manifest_path = directory / _MANIFEST_FILE_NAME
        manifest_payload = _read_regular_file(manifest_path, "地图资产清单")
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
            relative: path
            for relative, path in self._regular_files(directory)
            if relative != _MANIFEST_FILE_NAME
        }
        if set(actual_files) != set(expected_files):
            raise MapAssetBundleValidationError("地图资产清单与目录中的文件集合不一致。")
        for relative, metadata in expected_files.items():
            size, sha256 = _hash_regular_file(
                actual_files[relative],
                "地图资产文件",
            )
            if size != metadata["size"] or sha256 != metadata["sha256"]:
                raise MapAssetBundleValidationError("地图资产文件大小或 SHA-256 不匹配：%s" % relative)

        bundle_payload = _read_regular_file(directory / _BUNDLE_FILE_NAME, "地图资产包")
        bundle_document = _parse_json_object(bundle_payload, "地图资产包")
        if bundle_document.get("manifest_sha256") is not None and bundle_document.get("manifest_sha256") != "":
            raise MapAssetBundleValidationError("bundle.json 不得自引用 manifest_sha256。")
        try:
            bundle = MapAssetBundle.model_validate(bundle_document)
        except Exception as exc:
            raise MapAssetBundleValidationError("地图资产包契约非法。") from exc
        if bundle.map_name != map_name:
            raise MapAssetBundleValidationError("地图资产包 map_name 不匹配。")
        self._validate_geometry_assets(directory, bundle.geometry_asset, set(expected_files))
        return manifest_sha256, bundle

    def _validate_geometry_assets(
        self,
        directory: Path,
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
            _regular_file_size(_contained_path(directory, normalized), "几何资产")

        database_path = _contained_path(directory, geometry_asset.mapping_database_path)
        if _regular_file_size(database_path, "建图数据库") <= 4096:
            raise MapAssetBundleValidationError("建图数据库缺失或过小。")
        self._validate_pcd(_contained_path(directory, geometry_asset.pcd_path))
        navigation_assets = [
            asset
            for asset in geometry_asset.assets
            if asset.asset_type == MapGeometryAssetType.NAVIGATION_MAP
        ]
        if not navigation_assets:
            raise MapAssetBundleValidationError("地图资产包缺少 Nav2 地图资产。")
        for asset in geometry_asset.assets:
            if asset.asset_type == MapGeometryAssetType.NAVIGATION_MAP:
                self._validate_nav2_yaml(directory, asset.path, manifested_paths)
        self._validate_backend_checkpoint(
            directory,
            geometry_asset,
            manifested_paths,
        )

    def _validate_backend_checkpoint(
        self,
        directory: Path,
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
                raise MapAssetBundleValidationError(
                    "backend checkpoint 文件未被顶层清单绑定：%s" % relative
                )

        manifest_payload = _read_regular_file(
            _contained_path(directory, manifest_relative),
            "backend checkpoint manifest",
        )
        expected_manifest_sha256 = _validate_sha256(
            raw_checkpoint.get("manifest_sha256"),
            "backend checkpoint manifest SHA-256",
        )
        expected_object_sha256 = _validate_sha256(
            raw_checkpoint.get("object_sha256"),
            "backend checkpoint object SHA-256",
        )
        if _sha256_bytes(manifest_payload) != expected_manifest_sha256:
            raise MapAssetBundleValidationError("backend checkpoint manifest SHA-256 不匹配。")
        _object_size, object_sha256 = _hash_regular_file(
            _contained_path(directory, object_relative),
            "backend checkpoint object",
        )
        if object_sha256 != expected_object_sha256:
            raise MapAssetBundleValidationError("backend checkpoint object SHA-256 不匹配。")

        selector = _parse_json_object(
            _read_regular_file(
                _contained_path(directory, selector_relative),
                "backend checkpoint selector",
            ),
            "backend checkpoint selector",
        )
        selector_manifest = selector.get("manifest")
        backend_root = PurePosixPath("mapping_backend", backend_kind)
        try:
            selector_manifest_path = PurePosixPath(manifest_relative).relative_to(
                backend_root
            ).as_posix()
        except ValueError as exc:
            raise MapAssetBundleValidationError(
                "backend checkpoint manifest 不属于目标 backend。"
            ) from exc
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

            stored = MappingBackendCheckpointStore(directory).load(
                backend_kind,
                schema_version=schema_version,
            )
        except MappingBackendCheckpointStoreError as exc:
            raise MapAssetBundleValidationError(
                "backend checkpoint 内层契约不可加载。"
            ) from exc
        if (
            stored.manifest_sha256 != expected_manifest_sha256
            or stored.checkpoint.content_sha256.hex() != expected_object_sha256
        ):
            raise MapAssetBundleValidationError(
                "backend checkpoint 加载结果与资产包身份不一致。"
            )

    @staticmethod
    def _validate_pcd(path: Path) -> None:
        header = _read_regular_file_prefix(path, "PCD 地图", 8192)
        if b"FIELDS x y z" not in header or b"DATA " not in header:
            raise MapAssetBundleValidationError("PCD 地图头部非法。")

    @staticmethod
    def _validate_nav2_yaml(root: Path, yaml_relative: str, manifested_paths: set[str]) -> None:
        yaml_path = _contained_path(root, yaml_relative)
        text = _read_regular_file(yaml_path, "Nav2 地图 YAML").decode("utf-8")
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
        pgm_header = _read_regular_file_prefix(
            _contained_path(root, normalized),
            "Nav2 地图图像",
            2,
        )
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
            _read_regular_file(metadata_path, "旧地图 metadata.json"),
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
                _read_regular_file(selected / "version.json", "旧地图 version.json"),
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
            document = _parse_json_object(version_path.read_bytes(), "旧地图 version.json")
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
        snapshot_payload["metadata"] = self._normalize_legacy_metadata(
            source_dir,
            snapshot_payload.get("metadata") or {},
        )
        bundle_payload = {
            "map_name": map_name,
            "source_name": document.get("source_name") or snapshot_payload.get("source_name"),
            "source_snapshot_id": document.get("source_version_id") or snapshot_payload.get("version_id"),
            "frame_id": document.get("frame_id"),
            "snapshot": snapshot_payload,
            "geometry_asset": geometry_asset,
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


def _read_regular_file(path: Path, description: str) -> bytes:
    descriptor, _size = _open_regular_file(path, description)
    try:
        chunks = []
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            chunks.append(chunk)
        return b"".join(chunks)
    finally:
        os.close(descriptor)


def _open_regular_file(path: Path, description: str) -> Tuple[int, int]:
    try:
        descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0))
    except FileNotFoundError as exc:
        raise MapAssetBundleValidationError("%s不存在：%s" % (description, path)) from exc
    except OSError as exc:
        raise MapAssetBundleValidationError("无法打开%s：%s" % (description, path)) from exc
    path_stat = os.fstat(descriptor)
    if not stat.S_ISREG(path_stat.st_mode):
        os.close(descriptor)
        raise MapAssetBundleValidationError("%s必须是普通文件：%s" % (description, path))
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

    try:
        value = json.loads(
            payload.decode("utf-8"),
            object_pairs_hook=reject_duplicates,
            parse_constant=reject_constant,
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


def _fsync_tree(root: Path) -> None:
    directories = [root]
    for current_root, directory_names, file_names in os.walk(root, followlinks=False):
        current = Path(current_root)
        for name in file_names:
            path = current / name
            path_stat = path.lstat()
            if stat.S_ISLNK(path_stat.st_mode) or not stat.S_ISREG(path_stat.st_mode):
                raise MapAssetBundleValidationError("地图资产包包含非普通文件：%s" % path)
            descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0))
            try:
                os.fsync(descriptor)
            finally:
                os.close(descriptor)
        for name in directory_names:
            directory = current / name
            if directory.is_symlink():
                raise MapAssetBundleValidationError("地图资产包不得包含符号链接：%s" % directory)
            directories.append(directory)
    for directory in reversed(directories):
        _fsync_directory(directory)


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
