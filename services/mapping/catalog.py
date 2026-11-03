from __future__ import annotations

from contextlib import contextmanager
import fcntl
import json
import logging
import os
from pathlib import Path
import shutil
import tempfile
import threading
from urllib.parse import quote
import weakref

from contracts.map_workspace import MapAsset
from contracts.naming import build_safe_dir_segment
from typing import Iterator, List, Optional, Tuple

LOGGER = logging.getLogger("nuwax_robot_bridge.mapping.catalog")


class _MapActivationGuardState:
    def __init__(self) -> None:
        self.mutex = threading.RLock()
        self.local = threading.local()


_ACTIVATION_GUARD_REGISTRY_LOCK = threading.Lock()
_ACTIVATION_GUARD_REGISTRY: weakref.WeakValueDictionary[
    Tuple[int, str], _MapActivationGuardState
] = weakref.WeakValueDictionary()
_ACTIVATION_EXECUTOR_GUARD_REGISTRY_LOCK = threading.Lock()
_ACTIVATION_EXECUTOR_GUARD_REGISTRY: weakref.WeakValueDictionary[
    Tuple[int, str], _MapActivationGuardState
] = weakref.WeakValueDictionary()


def _map_activation_guard_state_for_root(root_dir: Path) -> _MapActivationGuardState:
    root_key = (os.getpid(), os.path.normcase(str(root_dir.resolve())))
    with _ACTIVATION_GUARD_REGISTRY_LOCK:
        state = _ACTIVATION_GUARD_REGISTRY.get(root_key)
        if state is None:
            state = _MapActivationGuardState()
            _ACTIVATION_GUARD_REGISTRY[root_key] = state
        return state


def _map_activation_executor_guard_state_for_root(
    root_dir: Path,
) -> _MapActivationGuardState:
    root_key = (os.getpid(), os.path.normcase(str(root_dir.resolve())))
    with _ACTIVATION_EXECUTOR_GUARD_REGISTRY_LOCK:
        state = _ACTIVATION_EXECUTOR_GUARD_REGISTRY.get(root_key)
        if state is None:
            state = _MapActivationGuardState()
            _ACTIVATION_EXECUTOR_GUARD_REGISTRY[root_key] = state
        return state


def _fsync_directory(path: Path) -> None:
    flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
    directory_fd = os.open(str(path), flags)
    try:
        os.fsync(directory_fd)
    finally:
        os.close(directory_fd)


def _atomic_write_text(path: Path, text: str, *, adoption_guard=None) -> None:
    """原子且持久地替换文本文件。"""

    descriptor, temporary_name = tempfile.mkstemp(
        dir=str(path.parent),
        prefix=".%s." % path.name,
        suffix=".tmp",
    )
    temporary_path = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        if adoption_guard is None:
            os.replace(temporary_path, path)
        else:
            # Fence only the visible catalog replacement. File/directory fsync and
            # extension callbacks must not hold runtime/provider ownership locks.
            with adoption_guard:
                os.replace(temporary_path, path)
        _fsync_directory(path.parent)
    finally:
        temporary_path.unlink(missing_ok=True)


class MapNameCollisionError(ValueError):
    """两个不同 map_name 经 build_safe_dir_segment 规整后落到同一个目录。

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


class MapCatalogRepository:
    """地图资产目录仓储。"""

    def __init__(self, root_dir: str) -> None:
        self._root_dir = Path(root_dir).expanduser()
        self._root_dir.mkdir(parents=True, exist_ok=True)
        self._active_map_path = self._root_dir / "active_map.json"
        self._activation_lock_path = self._root_dir / ".map_activation.lock"
        self._activation_executor_lock_path = (
            self._root_dir / ".map_activation_executor.lock"
        )
        self._activation_guard_state = _map_activation_guard_state_for_root(
            self._root_dir
        )
        self._activation_executor_guard_state = (
            _map_activation_executor_guard_state_for_root(self._root_dir)
        )

    @property
    def root_dir(self) -> Path:
        return self._root_dir

    @contextmanager
    def map_activation_guard(self) -> Iterator[None]:
        """Serialize one activation transaction across threads and processes."""

        state = _map_activation_guard_state_for_root(self._root_dir)
        self._activation_guard_state = state
        with state.mutex:
            depth = int(getattr(state.local, "depth", 0) or 0)
            if depth:
                state.local.depth = depth + 1
                try:
                    yield
                finally:
                    state.local.depth = depth
                return
            with self._activation_lock_path.open("a+b") as lock_stream:
                fcntl.flock(lock_stream.fileno(), fcntl.LOCK_EX)
                state.local.depth = 1
                try:
                    yield
                finally:
                    state.local.depth = 0
                    fcntl.flock(lock_stream.fileno(), fcntl.LOCK_UN)

    @contextmanager
    def map_activation_executor_guard(self) -> Iterator[None]:
        """串行执行地图写操作，读取入口不取执行锁。"""

        state = _map_activation_executor_guard_state_for_root(self._root_dir)
        self._activation_executor_guard_state = state
        with state.mutex:
            depth = int(getattr(state.local, "depth", 0) or 0)
            if depth:
                state.local.depth = depth + 1
                try:
                    yield
                finally:
                    state.local.depth = depth
                return
            with self._activation_executor_lock_path.open("a+b") as lock_stream:
                fcntl.flock(lock_stream.fileno(), fcntl.LOCK_EX)
                state.local.depth = 1
                try:
                    yield
                finally:
                    state.local.depth = 0
                    fcntl.flock(lock_stream.fileno(), fcntl.LOCK_UN)

    def get_map_asset(self, map_name: str) -> Optional[MapAsset]:
        metadata_path = self._metadata_path(map_name)
        if not metadata_path.exists():
            return None
        return self._load_map_asset(metadata_path)

    def list_map_assets(self) -> List[MapAsset]:
        assets: List[MapAsset] = []
        for map_dir in sorted(self._root_dir.iterdir(), key=lambda item: item.name):
            if not map_dir.is_dir():
                continue
            metadata_path = map_dir / "metadata.json"
            if not metadata_path.exists():
                continue
            assets.append(self._load_map_asset(metadata_path))
        return assets

    def save_map_asset(self, asset: MapAsset, *, adoption_guard=None) -> MapAsset:
        # 写入前先做 safe-segment 碰撞检查：如果目标目录已存在 metadata.json
        # 且其 map_name 与当前请求不同，说明两个不同 map_name 经 build_safe_dir_segment
        # 规整后落到同一目录（例如 "办公区/A" 与 "办公区_A"），拒绝静默覆盖。
        map_dir = self._map_dir(asset.map_name)
        existing_metadata_path = map_dir / "metadata.json"
        if existing_metadata_path.exists():
            try:
                existing_asset = self._load_map_asset(existing_metadata_path)
            except Exception:
                LOGGER.exception("读取既有地图目录记录失败 path=%s。", existing_metadata_path)
                raise
            if existing_asset is not None and existing_asset.map_name != asset.map_name:
                raise MapNameCollisionError(
                    requested_name=asset.map_name,
                    existing_name=existing_asset.map_name,
                    safe_segment=build_safe_dir_segment(asset.map_name),
                    path=str(existing_metadata_path),
                )
        map_dir.mkdir(parents=True, exist_ok=True)
        _atomic_write_text(
            self._metadata_path(asset.map_name),
            asset.model_dump_json(indent=2),
            adoption_guard=adoption_guard,
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
            LOGGER.exception("读取活动地图记录失败 path=%s。", self._active_map_path)
            raise
        map_name = str(payload.get("map_name", "")).strip()
        return map_name or None

    def set_active_map_name(self, map_name: str) -> None:
        resolved_name = str(map_name or "").strip()
        if not resolved_name:
            self.clear_active_map_name()
            return
        _atomic_write_text(
            self._active_map_path,
            json.dumps({"map_name": resolved_name}, ensure_ascii=False, indent=2),
        )

    def clear_active_map_name(self) -> None:
        if self._active_map_path.exists():
            self._active_map_path.unlink()

    def _metadata_path(self, map_name: str) -> Path:
        return self._map_dir(map_name) / "metadata.json"

    def _load_map_asset(self, metadata_path: Path) -> MapAsset:
        payload = json.loads(metadata_path.read_text(encoding="utf-8"))
        if not isinstance(payload, dict):
            raise ValueError("地图 metadata.json 必须是 JSON object。")
        payload.pop("latest_version_id", None)
        payload.pop("latest_revision", None)
        if payload.get("identity_schema_version") == 2:
            # Assigned reads validate persisted authority; they never infer an ID,
            # compute a manifest digest, or mutate finalized metadata during load.
            return MapAsset.model_validate(payload)
        current_dir = metadata_path.parent / "current"
        manifest_path = current_dir / "manifest.json"
        if "finalized" not in payload:
            payload["finalized"] = bool(current_dir.is_dir() and manifest_path.is_file())
        if payload.get("finalized") and not payload.get("finalized_at") and current_dir.is_dir():
            payload["finalized_at"] = payload.get("updated_at")
        return MapAsset.model_validate(payload)

    def _map_dir(self, map_name: str) -> Path:
        safe_name = build_safe_dir_segment(map_name)
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
