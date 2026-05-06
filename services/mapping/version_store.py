from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
from urllib.parse import quote

from contracts.map_workspace import MapVersion
from contracts.runtime_views import MapSnapshot
from typing import Dict, Optional, Tuple


class MapVersionRepository:
    """平台地图版本仓储。"""

    def __init__(self, root_dir: str) -> None:
        self._root_dir = Path(root_dir).expanduser()
        self._root_dir.mkdir(parents=True, exist_ok=True)

    def list_map_versions(self, map_name: str, *, limit: Optional[int] = None) -> Tuple[MapVersion, ...]:
        versions_root = self._versions_root(map_name)
        if not versions_root.exists():
            return tuple()
        items = []
        for version_dir in versions_root.iterdir():
            if not version_dir.is_dir():
                continue
            version_path = version_dir / "version.json"
            if not version_path.exists():
                continue
            items.append(MapVersion.model_validate_json(version_path.read_text(encoding="utf-8")))
        items.sort(key=lambda item: item.revision, reverse=True)
        if limit is not None:
            items = items[: max(0, limit)]
        return tuple(items)

    def get_map_version(self, map_name: str, version_id: str) -> Optional[MapVersion]:
        version_path = self._version_path(map_name, version_id)
        if not version_path.exists():
            return None
        return MapVersion.model_validate_json(version_path.read_text(encoding="utf-8"))

    def get_latest_map_version(self, map_name: str) -> Optional[MapVersion]:
        versions = self.list_map_versions(map_name, limit=1)
        return versions[0] if versions else None

    def save_existing_map_version(self, version: MapVersion) -> MapVersion:
        """覆盖写回已存在的平台地图版本。"""

        version_path = self._version_path(version.map_name, version.version_id)
        version_path.parent.mkdir(parents=True, exist_ok=True)
        self._write_text_atomic(version_path, version.model_dump_json(indent=2))
        self.write_bundle_manifest(version)
        return version

    def save_map_version(
        self,
        *,
        map_name: str,
        snapshot: MapSnapshot,
        metadata: Optional[Dict[str, object]] = None,
    ) -> MapVersion:
        latest = self.get_latest_map_version(map_name)
        revision = 1 if latest is None else latest.revision + 1
        version = MapVersion(
            map_name=str(map_name or "").strip(),
            version_id=self._build_version_id(revision),
            revision=revision,
            source_name=snapshot.source_name,
            source_version_id=snapshot.version_id,
            frame_id=self._resolve_map_frame_id(snapshot),
            snapshot=snapshot.model_copy(deep=True),
            metadata=dict(metadata or {}),
        )
        version_dir = self._versions_root(map_name) / version.version_id
        version_dir.mkdir(parents=True, exist_ok=True)
        self._write_text_atomic(
            self._version_path(map_name, version.version_id),
            version.model_dump_json(indent=2),
        )
        self.write_bundle_manifest(version)
        return version

    def bundle_manifest_path(self, map_name: str, version_id: str) -> str:
        """返回地图版本 bundle manifest 路径。"""

        return str((self._version_path(map_name, version_id).parent / "bundle_manifest.json").resolve(strict=False))

    def write_bundle_manifest(self, version: MapVersion, *, memory_checkpoint: Optional[Dict[str, object]] = None) -> Dict[str, object]:
        """写入地图版本 bundle manifest。"""

        version_dir = self._version_path(version.map_name, version.version_id).parent
        version_dir.mkdir(parents=True, exist_ok=True)
        manifest = self.build_bundle_manifest(version, memory_checkpoint=memory_checkpoint)
        self._write_text_atomic(
            version_dir / "bundle_manifest.json",
            json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True),
        )
        return manifest

    def build_bundle_manifest(self, version: MapVersion, *, memory_checkpoint: Optional[Dict[str, object]] = None) -> Dict[str, object]:
        """构建地图版本可校验资产清单。"""

        version_path = self._version_path(version.map_name, version.version_id)
        files = [self._describe_file(version_path, role="version_json", required=True)]
        geometry_asset = version.geometry_asset
        if geometry_asset is not None:
            geometry_asset_path_value = str(geometry_asset.metadata.get("geometry_asset_path") or "").strip()
            if geometry_asset_path_value:
                files.append(self._describe_file(Path(geometry_asset_path_value).expanduser(), role="geometry_asset_json", required=True))
            files.append(self._describe_file(Path(geometry_asset.rtabmap_db_path).expanduser(), role="rtabmap_database", required=True))
            if geometry_asset.pcd_path:
                files.append(self._describe_file(Path(geometry_asset.pcd_path).expanduser(), role="pcd", required=False))
            nav2_map = geometry_asset.metadata.get("nav2_map") if isinstance(geometry_asset.metadata.get("nav2_map"), dict) else {}
            for key, role, required in (
                ("yaml_path", "nav2_map_yaml", True),
                ("image_path", "nav2_map_image", False),
            ):
                value = str(nav2_map.get(key) or "").strip()
                if value:
                    files.append(self._describe_file(Path(value).expanduser(), role=role, required=required))
        manifest_payload = {
            "schema_version": "map_version_bundle_manifest.v1",
            "map_name": version.map_name,
            "version_id": version.version_id,
            "revision": version.revision,
            "source_name": version.source_name,
            "source_version_id": version.source_version_id,
            "frame_id": version.frame_id,
            "files": files,
            "memory_checkpoint": dict(memory_checkpoint or version.metadata.get("memory_checkpoint") or {}),
            "metadata": {
                "geometry_stack": geometry_asset.geometry_stack if geometry_asset is not None else None,
                "backend_map_version_id": version.metadata.get("backend_map_version_id"),
                "rtabmap_session_id": version.metadata.get("rtabmap_session_id"),
                "rtabmap_run_id": version.metadata.get("rtabmap_run_id"),
            },
        }
        manifest_payload["bundle_hash"] = hashlib.sha256(
            json.dumps(manifest_payload, ensure_ascii=False, sort_keys=True).encode("utf-8")
        ).hexdigest()
        missing_required = [item for item in files if item.get("required") and not item.get("present")]
        manifest_payload["complete"] = not missing_required
        manifest_payload["missing_required_files"] = missing_required
        return manifest_payload

    def _describe_file(self, path: Path, *, role: str, required: bool) -> Dict[str, object]:
        resolved = path.expanduser().resolve(strict=False)
        payload: Dict[str, object] = {
            "role": role,
            "path": str(resolved),
            "required": bool(required),
            "present": resolved.exists() and resolved.is_file(),
        }
        if not payload["present"]:
            return payload
        stat = resolved.stat()
        payload.update({
            "size_bytes": int(stat.st_size),
            "mtime_ns": int(stat.st_mtime_ns),
        })
        digest = hashlib.sha256()
        with resolved.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
        payload["sha256"] = digest.hexdigest()
        return payload

    def _write_text_atomic(self, path: Path, content: str) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp_path = path.with_name(".%s.tmp" % path.name)
        tmp_path.write_text(content, encoding="utf-8")
        os.replace(tmp_path, path)

    def _build_version_id(self, revision: int) -> str:
        return f"mapver_{revision:06d}"

    def _resolve_map_frame_id(self, snapshot: MapSnapshot) -> Optional[str]:
        for item in (snapshot.occupancy_grid, snapshot.cost_map, snapshot.semantic_map):
            if item is None:
                continue
            frame_id = str(getattr(item, "frame_id", "") or "").strip()
            if frame_id:
                return frame_id
        return None

    def _version_path(self, map_name: str, version_id: str) -> Path:
        return self._versions_root(map_name) / quote(str(version_id or "").strip(), safe="") / "version.json"

    def _versions_root(self, map_name: str) -> Path:
        safe_map_name = quote(str(map_name or "").strip(), safe="")
        return self._root_dir / safe_map_name / "versions"
