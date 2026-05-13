from __future__ import annotations

from pathlib import Path
from urllib.parse import quote

from contracts.map_workspace import MapVersion
from contracts.runtime_views import MapSnapshot
from services.mapping.catalog import _to_safe_dir_segment
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

        self._version_path(version.map_name, version.version_id).parent.mkdir(parents=True, exist_ok=True)
        self._version_path(version.map_name, version.version_id).write_text(
            version.model_dump_json(indent=2),
            encoding="utf-8",
        )
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
        self._version_path(map_name, version.version_id).write_text(
            version.model_dump_json(indent=2),
            encoding="utf-8",
        )
        return version

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
        versions_root = self._versions_root(map_name)
        safe_version = _to_safe_dir_segment(version_id)
        primary = versions_root / safe_version / "version.json"
        if not primary.exists():
            # 兼容旧版本写出的 URL 编码 version_id 目录（含特殊字符时才有差异）。
            legacy = versions_root / quote(str(version_id or "").strip(), safe="") / "version.json"
            if legacy.exists():
                return legacy
        return primary

    def _versions_root(self, map_name: str) -> Path:
        safe_map_name = _to_safe_dir_segment(map_name)
        primary = self._root_dir / safe_map_name / "versions"
        # 同 catalog.py 中 _map_dir 的兼容策略：旧目录走 URL 编码名时不破坏读取。
        if not primary.exists():
            legacy_root = self._root_dir / quote(str(map_name or "").strip(), safe="")
            legacy_versions = legacy_root / "versions"
            if legacy_versions.exists():
                return legacy_versions
        return primary
