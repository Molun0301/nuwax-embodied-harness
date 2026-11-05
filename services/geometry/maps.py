"""地图目录的显示信息与禁行区文件。"""
from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import shutil
import tempfile
import threading
from typing import Any, List, Mapping, Optional, Sequence
import unicodedata

from core.errors import GatewayError


@dataclass(frozen=True)
class MapInfo:
    name: str
    saved: bool
    display_name: str
    created_at: Optional[str]
    updated_at: Optional[str]
    georeference: Optional[dict]
    size_bytes: int


class MapNameError(ValueError):
    """入口地图名不合法。"""


def validate_map_name(name: str) -> str:
    name = unicodedata.normalize("NFC", name).strip()
    if not 1 <= len(name) <= 64 or name.startswith(".") or any(
        c == "/" or unicodedata.category(c) == "Cc" for c in name
    ):
        raise MapNameError("地图名须为 1–64 个字符，不能以点开头或含斜杠、控制字符")
    return name


class MapStore:
    def __init__(self, root: str) -> None:
        self._root = Path(root)
        self._lock = threading.RLock()

    @property
    def root(self) -> Path:
        return self._root

    def list(self) -> List[MapInfo]:
        with self._lock:
            if not self._root.exists():
                return []
            maps = [self.get(path.name) for path in self._root.iterdir() if path.is_dir()]
            return sorted((info for info in maps if info is not None),
                          key=lambda info: info.updated_at or "", reverse=True)

    def get(self, name: str) -> Optional[MapInfo]:
        with self._lock:
            path = self._root / name
            saved = (path / "geometry").is_dir()
            if not saved and not (path / "meta.json").is_file() and not (path / "memory").is_dir():
                return None
            meta = self.read_meta(name)
            size = sum(file.stat().st_size for file in path.rglob("*") if file.is_file())
            return MapInfo(name, saved, meta.get("display_name", name), meta.get("created_at"),
                           meta.get("updated_at"), meta.get("georeference"), size)

    def _read_json(self, name, filename, expected, missing):
        path = self._root / name / filename
        try:
            with path.open(encoding="utf-8") as stream:
                value = json.load(stream)
        except FileNotFoundError:
            return missing
        if not isinstance(value, expected):
            raise ValueError(f"{path} 的 JSON 内容类型错误")
        return value

    def read_meta(self, name: str) -> dict:
        with self._lock:
            return self._read_json(name, "meta.json", dict, {})

    def _write_json(self, name, filename, value):
        directory = self._root / name
        directory.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=directory,
                                         prefix=f".{filename}.", delete=False) as stream:
            temporary = stream.name
            try:
                json.dump(value, stream, ensure_ascii=False)
                stream.flush()
                os.fsync(stream.fileno())
                os.replace(temporary, directory / filename)
            finally:
                if os.path.exists(temporary):
                    os.unlink(temporary)

    def write_meta(self, name: str, meta: Mapping[str, Any]) -> None:
        with self._lock:
            self._write_json(name, "meta.json", dict(meta))

    def delete(self, name: str) -> None:
        with self._lock:
            try:
                shutil.rmtree(self._root / name)
            except FileNotFoundError as error:
                raise GatewayError("地图不存在", error_code="map_not_found", http_status=404) from error

    def read_forbidden_zones(self, name: str) -> List[dict]:
        with self._lock:
            return self._read_json(name, "forbidden_zones.json", list, [])

    def write_forbidden_zones(self, name: str, zones: Sequence[Mapping[str, Any]]) -> None:
        with self._lock:
            self._write_json(name, "forbidden_zones.json", [dict(zone) for zone in zones])
