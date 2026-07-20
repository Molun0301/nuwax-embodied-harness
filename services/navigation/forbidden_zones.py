from __future__ import annotations

import json
import os
from pathlib import Path
import secrets
import tempfile
import threading
from typing import Callable, Optional, Sequence, Union

from pydantic import ValidationError

from contracts.forbidden_zones import (
    ForbiddenZone,
    ForbiddenZoneMapEntry,
    ForbiddenZonesEnvelope,
    ReplaceForbiddenZonesRequest,
)
from contracts.base import utc_now
from core.errors import GatewayError


DEFAULT_FORBIDDEN_ZONES_PATH = Path("runtime_data/navigation/forbidden_zones.json")
FORBIDDEN_ZONES_PATH_ENV = "NUWAX_FORBIDDEN_ZONES_PATH"


class ForbiddenZoneStore:
    """按地图名称保存禁行区，并通过同目录原子替换保证文件完整性。"""

    def __init__(
        self,
        path: Optional[Union[str, os.PathLike[str]]] = None,
        *,
        signature_factory: Optional[Callable[[], str]] = None,
    ) -> None:
        configured_path = path
        if configured_path is None:
            configured_path = os.environ.get(FORBIDDEN_ZONES_PATH_ENV) or DEFAULT_FORBIDDEN_ZONES_PATH
        self._path = Path(configured_path).expanduser()
        self._lock = threading.RLock()
        self._signature_factory = signature_factory or (
            lambda: secrets.token_hex(32)
        )

    @property
    def path(self) -> Path:
        return self._path

    def load(self) -> ForbiddenZonesEnvelope:
        with self._lock:
            return self._load_unlocked()

    def list_for_map(
        self,
        map_version_id: str = "",
        *,
        map_name: Optional[str] = None,
    ) -> ForbiddenZonesEnvelope:
        request = ReplaceForbiddenZonesRequest(
            map_name=str(map_name or ""),
            map_version_id=map_version_id,
            zones=[],
        )
        with self._lock:
            document = self._load_unlocked()
            maps = dict(document.maps)
            storage_key = request.map_name or request.map_version_id
            entry = maps.get(storage_key)
            if entry is None and request.map_name:
                legacy_key, legacy_entry = self._find_legacy_entry(
                    maps,
                    request.map_version_id,
                )
                if legacy_entry is not None:
                    entry = legacy_entry.model_copy(
                        update={
                            "map_name": request.map_name,
                            "map_version_id": "",
                        },
                        deep=True,
                    )
                    maps.pop(legacy_key, None)
                    maps[storage_key] = entry
                    self._write_unlocked(ForbiddenZonesEnvelope(maps=maps))
            if entry is None:
                entry = ForbiddenZoneMapEntry(
                    map_name=request.map_name,
                    map_version_id=request.map_version_id,
                )
            projected_entry = entry.model_copy(
                update={
                    "map_name": request.map_name or entry.map_name,
                    "map_version_id": request.map_version_id or entry.map_version_id,
                },
                deep=True,
            )
            return ForbiddenZonesEnvelope(maps={storage_key: projected_entry})

    def replace(
        self,
        *,
        map_name: Optional[str] = None,
        map_version_id: str = "",
        zones: Sequence[ForbiddenZone],
        expected_generation: Optional[int] = None,
    ) -> ForbiddenZonesEnvelope:
        request = ReplaceForbiddenZonesRequest(
            map_name=str(map_name or ""),
            map_version_id=map_version_id,
            expected_generation=expected_generation,
            zones=list(zones),
        )
        with self._lock:
            document = self._load_unlocked()
            maps = dict(document.maps)
            storage_key = request.map_name or request.map_version_id
            previous = maps.get(storage_key)
            legacy_key = None
            if previous is None and request.map_name:
                legacy_key, legacy_entry = self._find_legacy_entry(
                    maps,
                    request.map_version_id,
                )
                if legacy_entry is not None:
                    previous = legacy_entry
            current_generation = previous.generation if previous is not None else 0
            if (
                request.expected_generation is not None
                and request.expected_generation != current_generation
            ):
                raise GatewayError(
                    "禁行区已被其他编辑器更新，请重新加载后再保存。",
                    error_code="forbidden_zones_generation_conflict",
                    http_status=409,
                    details={
                        "map_name": request.map_name or None,
                        "expected_generation": request.expected_generation,
                        "current_generation": current_generation,
                    },
                )
            entry = ForbiddenZoneMapEntry(
                map_name=request.map_name,
                map_version_id="" if request.map_name else request.map_version_id,
                generation=current_generation + 1,
                signature=self._signature_factory(),
                updated_at=utc_now(),
                zones=request.zones,
            )
            if legacy_key is not None and legacy_key != storage_key:
                maps.pop(legacy_key, None)
            maps[storage_key] = entry
            updated_document = ForbiddenZonesEnvelope(maps=maps)
            self._write_unlocked(updated_document)
            projected_entry = entry.model_copy(
                update={"map_version_id": request.map_version_id},
                deep=True,
            )
            return ForbiddenZonesEnvelope(maps={storage_key: projected_entry})

    def delete_for_map(self, map_name: str) -> ForbiddenZonesEnvelope:
        """删除与地图资产绑定的禁行区，并返回供实时流清屏的墓碑。"""

        resolved_map_name = str(map_name or "").strip()
        if not resolved_map_name:
            raise GatewayError(
                "map_name 不能为空。",
                error_code="map_name_required",
                http_status=400,
            )
        with self._lock:
            document = self._load_unlocked()
            maps = dict(document.maps)
            previous = maps.pop(resolved_map_name, None)
            if previous is None:
                return ForbiddenZonesEnvelope()
            self._write_unlocked(ForbiddenZonesEnvelope(maps=maps))
            tombstone = previous.model_copy(
                update={
                    "generation": previous.generation + 1,
                    "signature": "",
                    "updated_at": utc_now(),
                    "zones": [],
                },
                deep=True,
            )
            return ForbiddenZonesEnvelope(
                maps={resolved_map_name: tombstone}
            )

    @staticmethod
    def _find_legacy_entry(
        maps: dict[str, ForbiddenZoneMapEntry],
        map_version_id: str,
    ) -> tuple[Optional[str], Optional[ForbiddenZoneMapEntry]]:
        exact = maps.get(map_version_id) if map_version_id else None
        if exact is not None and not exact.map_name:
            return map_version_id, exact
        candidates = [
            (key, entry)
            for key, entry in maps.items()
            if not entry.map_name
        ]
        if len(candidates) == 1:
            return candidates[0]
        return None, None

    def _load_unlocked(self) -> ForbiddenZonesEnvelope:
        if not self._path.exists():
            return ForbiddenZonesEnvelope()
        try:
            payload = json.loads(self._path.read_text(encoding="utf-8"))
            return ForbiddenZonesEnvelope.model_validate(payload)
        except (json.JSONDecodeError, ValidationError, TypeError, ValueError) as exc:
            raise GatewayError(
                "禁行区存储文件格式无效。",
                error_code="forbidden_zones_store_invalid",
                http_status=500,
                details={"path": str(self._path)},
            ) from exc
        except OSError as exc:
            raise GatewayError(
                "无法读取禁行区存储文件。",
                error_code="forbidden_zones_store_read_failed",
                http_status=500,
                details={"path": str(self._path)},
            ) from exc

    def _write_unlocked(self, document: ForbiddenZonesEnvelope) -> None:
        encoded = (
            json.dumps(
                document.model_dump(mode="json"),
                ensure_ascii=False,
                separators=(",", ":"),
                sort_keys=True,
            )
            + "\n"
        ).encode("utf-8")
        self._path.parent.mkdir(parents=True, exist_ok=True)
        temp_path: Optional[str] = None
        try:
            fd, temp_path = tempfile.mkstemp(
                prefix=f".{self._path.name}.",
                suffix=".tmp",
                dir=self._path.parent,
            )
            try:
                # The production directory is setgid to nuwax-motion-clients;
                # keep group-read permission so the nuwax-geometry runtime can
                # consume the same atomically replaced document.
                os.fchmod(fd, 0o640)
                with os.fdopen(fd, "wb", closefd=True) as stream:
                    fd = -1
                    stream.write(encoded)
                    stream.flush()
                    os.fsync(stream.fileno())
            finally:
                if fd >= 0:
                    os.close(fd)
            os.replace(temp_path, self._path)
            temp_path = None
            directory_fd = os.open(self._path.parent, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        except OSError as exc:
            raise GatewayError(
                "无法原子写入禁行区存储文件。",
                error_code="forbidden_zones_store_write_failed",
                http_status=500,
                details={"path": str(self._path)},
            ) from exc
        finally:
            if temp_path is not None:
                try:
                    os.unlink(temp_path)
                except FileNotFoundError:
                    pass


__all__ = [
    "DEFAULT_FORBIDDEN_ZONES_PATH",
    "FORBIDDEN_ZONES_PATH_ENV",
    "ForbiddenZoneStore",
]
