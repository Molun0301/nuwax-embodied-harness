from __future__ import annotations

import os
from pathlib import Path
from urllib.parse import quote

from contracts.map_workspace import MapVersion
from contracts.naming import build_safe_dir_segment
from contracts.runtime_views import MapSnapshot
from datetime import datetime, timezone
from typing import Dict, Optional, Tuple


def _atomic_write_text(path: Path, text: str) -> None:
    """原子写文本：同目录写 .tmp 再 os.replace 替换，避免写一半崩溃留下损坏文件。"""
    tmp_path = path.with_name(path.name + ".tmp")
    tmp_path.write_text(text, encoding="utf-8")
    os.replace(tmp_path, path)


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
        _atomic_write_text(
            self._version_path(version.map_name, version.version_id),
            version.model_dump_json(indent=2),
        )
        return version

    def save_map_version(
        self,
        *,
        map_name: str,
        snapshot: MapSnapshot,
        metadata: Optional[Dict[str, object]] = None,
        version_id_override: Optional[str] = None,
    ) -> MapVersion:
        existing = None
        override_id = str(version_id_override or "").strip()
        if override_id:
            existing = self.get_map_version(map_name, override_id)
        if existing is not None:
            version = existing.model_copy(
                update={
                    "timestamp": datetime.now(timezone.utc),
                    "source_name": snapshot.source_name,
                    "source_version_id": snapshot.version_id,
                    "frame_id": self._resolve_map_frame_id(snapshot) or existing.frame_id,
                    "snapshot": snapshot.model_copy(deep=True),
                    "metadata": {
                        **dict(existing.metadata or {}),
                        **dict(metadata or {}),
                    },
                },
                deep=True,
            )
        else:
            latest = self.get_latest_map_version(map_name)
            if override_id:
                version_id = override_id
                try:
                    revision = int(override_id.split("_", 1)[1])
                except (IndexError, ValueError):
                    revision = 1 if latest is None else latest.revision + 1
            else:
                revision = 1 if latest is None else latest.revision + 1
                version_id = self._build_version_id(revision)
            version = MapVersion(
                map_name=str(map_name or "").strip(),
                version_id=version_id,
                revision=revision,
                source_name=snapshot.source_name,
                source_version_id=snapshot.version_id,
                frame_id=self._resolve_map_frame_id(snapshot),
                snapshot=snapshot.model_copy(deep=True),
                metadata=dict(metadata or {}),
            )
        version_dir = self._versions_root(map_name) / version.version_id
        version_dir.mkdir(parents=True, exist_ok=True)
        _atomic_write_text(
            self._version_path(map_name, version.version_id),
            version.model_dump_json(indent=2),
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
        safe_version = build_safe_dir_segment(version_id)
        primary = versions_root / safe_version / "version.json"
        if not primary.exists():
            # 兼容旧版本写出的 URL 编码 version_id 目录（含特殊字符时才有差异）。
            legacy = versions_root / quote(str(version_id or "").strip(), safe="") / "version.json"
            if legacy.exists():
                return legacy
        return primary

    def _versions_root(self, map_name: str) -> Path:
        safe_map_name = build_safe_dir_segment(map_name)
        primary = self._root_dir / safe_map_name / "versions"
        # 同 catalog.py 中 _map_dir 的兼容策略：旧目录走 URL 编码名时不破坏读取。
        if not primary.exists():
            legacy_root = self._root_dir / quote(str(map_name or "").strip(), safe="")
            legacy_versions = legacy_root / "versions"
            if legacy_versions.exists():
                return legacy_versions
        return primary
