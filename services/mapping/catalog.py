from __future__ import annotations

import json
import unicodedata
from pathlib import Path
import shutil
from urllib.parse import quote

from contracts.map_workspace import MapAsset
from typing import List, Optional


class MapNameCollisionError(ValueError):
    """两个不同 map_name 经 _to_safe_dir_segment 规整后落到同一个目录。

    场景：name_a="办公区/A"、name_b="办公区_A" 都会被规整为 "办公区_A"。
    第二个 name 写入时本类被抛出，避免悄悄覆盖已有地图资产。
    """

    def __init__(self, *, requested_name: str, existing_name: str, safe_segment: str, path: str) -> None:
        self.requested_name = requested_name
        self.existing_name = existing_name
        self.safe_segment = safe_segment
        self.path = path
        super().__init__(
            "地图名称 %r 与已存在地图 %r 在文件系统目录段 %r 上冲突；"
            "请改用差异更明显的 map_name。" % (requested_name, existing_name, safe_segment)
        )


# 文件系统中不允许出现的字符（POSIX/常见 FS 取交集，再补常规危险符号），
# 用单字符 `_` 替换；保留中文/Unicode 字母数字，避免 URL 编码导致的 sqlite 路径
# 不可达问题（参见 docs/开发文档/ROS全链路修复-2026-05-13/worker2-rtabmap/feadback.md）。
_FORBIDDEN_DIR_CHARS = set('/\\\x00:*?"<>|')


def _to_safe_dir_segment(raw_name: str) -> str:
    """把任意 map_name / version_id 规整成单层目录名。

    规则：
    - strip + NFC 归一化，避免 macOS NFD 与 Linux NFC 不一致；
    - 控制字符与文件系统保留符号 → `_`；
    - 不做 URL/percent 编码，保留中文等 Unicode 字母数字，让 rtabmap、SQLite
      在该路径上直接 open 而不会因为 `%XX` 解释歧义打不开数据库。
    - 整段去重连续 `_`，避免污染显示。
    """

    candidate = unicodedata.normalize("NFC", str(raw_name or "").strip())
    if not candidate:
        # 空字符串永远不允许落到文件系统：返回单字符占位避免 mkdir/sqlite 异常。
        return "_"
    chars: List[str] = []
    for ch in candidate:
        if ch in _FORBIDDEN_DIR_CHARS or unicodedata.category(ch).startswith("C"):
            chars.append("_")
        else:
            chars.append(ch)
    normalized = "".join(chars)
    while "__" in normalized:
        normalized = normalized.replace("__", "_")
    return normalized.strip("_") or "_"


class MapCatalogRepository:
    """地图资产目录仓储。"""

    def __init__(self, root_dir: str) -> None:
        self._root_dir = Path(root_dir).expanduser()
        self._root_dir.mkdir(parents=True, exist_ok=True)
        self._active_map_path = self._root_dir / "active_map.json"

    @property
    def root_dir(self) -> Path:
        return self._root_dir

    def get_map_asset(self, map_name: str) -> Optional[MapAsset]:
        metadata_path = self._metadata_path(map_name)
        if not metadata_path.exists():
            return None
        return MapAsset.model_validate_json(metadata_path.read_text(encoding="utf-8"))

    def list_map_assets(self) -> List[MapAsset]:
        assets: List[MapAsset] = []
        for map_dir in sorted(self._root_dir.iterdir(), key=lambda item: item.name):
            if not map_dir.is_dir():
                continue
            metadata_path = map_dir / "metadata.json"
            if not metadata_path.exists():
                continue
            assets.append(MapAsset.model_validate_json(metadata_path.read_text(encoding="utf-8")))
        return assets

    def save_map_asset(self, asset: MapAsset) -> MapAsset:
        # 写入前先做 safe-segment 碰撞检查：如果目标目录已存在 metadata.json
        # 且其 map_name 与当前请求不同，说明两个不同 map_name 经 _to_safe_dir_segment
        # 规整后落到同一目录（例如 "办公区/A" 与 "办公区_A"），拒绝静默覆盖。
        map_dir = self._map_dir(asset.map_name)
        existing_metadata_path = map_dir / "metadata.json"
        if existing_metadata_path.exists():
            try:
                existing_asset = MapAsset.model_validate_json(existing_metadata_path.read_text(encoding="utf-8"))
            except Exception:
                existing_asset = None
            if existing_asset is not None and existing_asset.map_name != asset.map_name:
                raise MapNameCollisionError(
                    requested_name=asset.map_name,
                    existing_name=existing_asset.map_name,
                    safe_segment=_to_safe_dir_segment(asset.map_name),
                    path=str(existing_metadata_path),
                )
        map_dir.mkdir(parents=True, exist_ok=True)
        (map_dir / "snapshots").mkdir(parents=True, exist_ok=True)
        (map_dir / "versions").mkdir(parents=True, exist_ok=True)
        (map_dir / "artifacts").mkdir(parents=True, exist_ok=True)
        self._metadata_path(asset.map_name).write_text(
            asset.model_dump_json(indent=2),
            encoding="utf-8",
        )
        return asset

    def delete_map_asset(self, map_name: str) -> bool:
        map_dir = self._map_dir(map_name)
        if not map_dir.exists():
            return False
        shutil.rmtree(map_dir)
        if self.get_active_map_name() == str(map_name or "").strip():
            self.clear_active_map_name()
        return True

    def get_active_map_name(self) -> Optional[str]:
        if not self._active_map_path.exists():
            return None
        try:
            payload = json.loads(self._active_map_path.read_text(encoding="utf-8"))
        except Exception:
            return None
        map_name = str(payload.get("map_name", "")).strip()
        return map_name or None

    def set_active_map_name(self, map_name: str) -> None:
        resolved_name = str(map_name or "").strip()
        if not resolved_name:
            self.clear_active_map_name()
            return
        self._active_map_path.write_text(
            json.dumps({"map_name": resolved_name}, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

    def clear_active_map_name(self) -> None:
        if self._active_map_path.exists():
            self._active_map_path.unlink()

    def _metadata_path(self, map_name: str) -> Path:
        return self._map_dir(map_name) / "metadata.json"

    def _map_dir(self, map_name: str) -> Path:
        safe_name = _to_safe_dir_segment(map_name)
        primary = self._root_dir / safe_name
        # 向后兼容：旧版本曾用 urllib.parse.quote(map_name, safe="") 写出 URL
        # 编码目录（如 `%E5%8A%9E%E5%85%AC%E5%AE%A4`），sqlite 在带百分号路径上
        # 反复 FATAL `unable to open database file`。这里：当原名目录不存在但
        # 编码目录存在时，沿用旧目录路径以读老资产；新写入永远走原名分支。
        if not primary.exists():
            legacy = self._root_dir / quote(str(map_name or "").strip(), safe="")
            if legacy.exists():
                return legacy
        return primary
