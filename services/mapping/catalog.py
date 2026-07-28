from __future__ import annotations

from contextlib import contextmanager
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile
import threading
from urllib.parse import quote
import uuid
import weakref

from contracts.map_workspace import MapAsset
from contracts.naming import build_safe_dir_segment
from typing import Dict, Iterator, List, Optional, Sequence, Tuple


_ACTIVATION_JOURNAL_SCHEMA_VERSION = 2
_ACTIVATION_RECEIPT_SCHEMA_VERSION = 1
_ACTIVATION_JOURNAL_PHASES = frozenset(
    {
        "prepared",
        "backend_dispatching",
        "backend_applied",
        "reconciling",
        "recovering",
    }
)


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


def _get_map_activation_guard_state(root_dir: Path) -> _MapActivationGuardState:
    root_key = (os.getpid(), os.path.normcase(str(root_dir.resolve())))
    with _ACTIVATION_GUARD_REGISTRY_LOCK:
        state = _ACTIVATION_GUARD_REGISTRY.get(root_key)
        if state is None:
            state = _MapActivationGuardState()
            _ACTIVATION_GUARD_REGISTRY[root_key] = state
        return state


def _get_map_activation_executor_guard_state(
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


def _atomic_write_text(path: Path, text: str) -> None:
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
        self._activation_journal_path = self._root_dir / "map_activation.json"
        self._activation_receipt_path = self._root_dir / "map_activation_receipt.json"
        self._activation_lock_path = self._root_dir / ".map_activation.lock"
        self._activation_executor_lock_path = (
            self._root_dir / ".map_activation_executor.lock"
        )
        self._activation_guard_state = _get_map_activation_guard_state(
            self._root_dir
        )
        self._activation_executor_guard_state = (
            _get_map_activation_executor_guard_state(self._root_dir)
        )

    @property
    def root_dir(self) -> Path:
        return self._root_dir

    @contextmanager
    def map_activation_guard(self) -> Iterator[None]:
        """Serialize one activation transaction across threads and processes."""

        state = _get_map_activation_guard_state(self._root_dir)
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
        """Serialize activation executors without blocking short journal reads."""

        state = _get_map_activation_executor_guard_state(self._root_dir)
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

    def save_map_asset(self, asset: MapAsset) -> MapAsset:
        # 写入前先做 safe-segment 碰撞检查：如果目标目录已存在 metadata.json
        # 且其 map_name 与当前请求不同，说明两个不同 map_name 经 build_safe_dir_segment
        # 规整后落到同一目录（例如 "办公区/A" 与 "办公区_A"），拒绝静默覆盖。
        map_dir = self._map_dir(asset.map_name)
        existing_metadata_path = map_dir / "metadata.json"
        if existing_metadata_path.exists():
            try:
                existing_asset = self._load_map_asset(existing_metadata_path)
            except Exception:
                existing_asset = None
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
        _atomic_write_text(
            self._active_map_path,
            json.dumps({"map_name": resolved_name}, ensure_ascii=False, indent=2),
        )

    def clear_active_map_name(self) -> None:
        if self._active_map_path.exists():
            self._active_map_path.unlink()

    def begin_map_activation(
        self,
        *,
        previous_map_name: Optional[str],
        target_map_name: str,
        target_manifest_sha256: str,
        load_memory_history: bool,
        reuse_completed_request_id: bool = True,
    ) -> Dict[str, object]:
        with self.map_activation_guard():
            target_name = str(target_map_name or "").strip()
            manifest_sha256 = str(target_manifest_sha256 or "").strip().lower()
            if not target_name:
                raise ValueError("地图激活日志 target_map_name 不能为空。")
            if len(manifest_sha256) != 64 or any(
                character not in "0123456789abcdef" for character in manifest_sha256
            ):
                raise ValueError("地图激活日志 target_manifest_sha256 非法。")
            existing = self.get_map_activation()
            if existing is not None:
                if (
                    existing["target_map_name"] != target_name
                    or existing["target_manifest_sha256"] != manifest_sha256
                ):
                    raise ValueError(
                        "已有其它地图的激活恢复任务：%s。"
                        % existing["target_map_name"]
                    )
                return existing
            activation_request_id = "mapload:%s" % uuid.uuid4().hex
            if reuse_completed_request_id:
                receipt = self.get_last_successful_map_activation_receipt()
                if receipt is not None and (
                    receipt["target_map_name"] == target_name
                    and receipt["target_manifest_sha256"] == manifest_sha256
                ):
                    activation_request_id = str(receipt["activation_request_id"])
            payload: Dict[str, object] = {
                "schema_version": _ACTIVATION_JOURNAL_SCHEMA_VERSION,
                "previous_map_name": str(previous_map_name or "").strip() or None,
                "target_map_name": target_name,
                "target_manifest_sha256": manifest_sha256,
                "activation_request_id": activation_request_id,
                "load_memory_history": bool(load_memory_history),
                "phase": "prepared",
            }
            _atomic_write_text(
                self._activation_journal_path,
                json.dumps(payload, ensure_ascii=False, indent=2),
            )
            return payload

    def get_last_successful_map_activation_receipt(
        self,
    ) -> Optional[Dict[str, object]]:
        with self.map_activation_guard():
            if not self._activation_receipt_path.exists():
                return None
            try:
                payload = json.loads(
                    self._activation_receipt_path.read_text(encoding="utf-8")
                )
            except Exception as exc:
                raise ValueError("地图激活成功凭据不是有效 JSON。") from exc
            required_fields = {
                "schema_version",
                "target_map_name",
                "target_manifest_sha256",
                "activation_request_id",
            }
            if not isinstance(payload, dict) or set(payload) != required_fields:
                raise ValueError("地图激活成功凭据字段不完整或包含未知字段。")
            if payload.get("schema_version") != _ACTIVATION_RECEIPT_SCHEMA_VERSION:
                raise ValueError("不支持的地图激活成功凭据 schema_version。")
            target_name = str(payload.get("target_map_name") or "").strip()
            manifest_sha256 = str(
                payload.get("target_manifest_sha256") or ""
            ).strip().lower()
            activation_request_id = str(
                payload.get("activation_request_id") or ""
            ).strip()
            if not target_name:
                raise ValueError("地图激活成功凭据 target_map_name 不能为空。")
            if len(manifest_sha256) != 64 or any(
                character not in "0123456789abcdef" for character in manifest_sha256
            ):
                raise ValueError("地图激活成功凭据 target_manifest_sha256 非法。")
            request_suffix = activation_request_id.removeprefix("mapload:")
            if (
                not activation_request_id.startswith("mapload:")
                or len(request_suffix) != 32
                or any(character not in "0123456789abcdef" for character in request_suffix)
            ):
                raise ValueError("地图激活成功凭据 activation_request_id 非法。")
            return {
                "schema_version": _ACTIVATION_RECEIPT_SCHEMA_VERSION,
                "target_map_name": target_name,
                "target_manifest_sha256": manifest_sha256,
                "activation_request_id": activation_request_id,
            }

    def get_map_activation(self) -> Optional[Dict[str, object]]:
        with self.map_activation_guard():
            if not self._activation_journal_path.exists():
                return None
            try:
                payload = json.loads(
                    self._activation_journal_path.read_text(encoding="utf-8")
                )
            except Exception as exc:
                raise ValueError("地图激活日志不是有效 JSON。") from exc
            required_fields = {
                "schema_version",
                "previous_map_name",
                "target_map_name",
                "target_manifest_sha256",
                "activation_request_id",
                "load_memory_history",
                "phase",
            }
            if not isinstance(payload, dict) or set(payload) != required_fields:
                raise ValueError("地图激活日志字段不完整或包含未知字段。")
            if payload.get("schema_version") != _ACTIVATION_JOURNAL_SCHEMA_VERSION:
                raise ValueError("不支持的地图激活日志 schema_version。")
            previous_name = str(payload.get("previous_map_name") or "").strip() or None
            target_name = str(payload.get("target_map_name") or "").strip()
            manifest_sha256 = str(
                payload.get("target_manifest_sha256") or ""
            ).strip().lower()
            activation_request_id = str(
                payload.get("activation_request_id") or ""
            ).strip()
            load_memory_history = payload.get("load_memory_history")
            phase = str(payload.get("phase") or "").strip()
            if not target_name:
                raise ValueError("地图激活日志 target_map_name 不能为空。")
            if len(manifest_sha256) != 64 or any(
                character not in "0123456789abcdef" for character in manifest_sha256
            ):
                raise ValueError("地图激活日志 target_manifest_sha256 非法。")
            request_suffix = activation_request_id.removeprefix("mapload:")
            if (
                not activation_request_id.startswith("mapload:")
                or len(request_suffix) != 32
                or any(character not in "0123456789abcdef" for character in request_suffix)
            ):
                raise ValueError("地图激活日志 activation_request_id 非法。")
            if not isinstance(load_memory_history, bool):
                raise ValueError("地图激活日志 load_memory_history 非法。")
            if phase not in _ACTIVATION_JOURNAL_PHASES:
                raise ValueError("地图激活日志 phase 非法。")
            return {
                "schema_version": _ACTIVATION_JOURNAL_SCHEMA_VERSION,
                "previous_map_name": previous_name,
                "target_map_name": target_name,
                "target_manifest_sha256": manifest_sha256,
                "activation_request_id": activation_request_id,
                "load_memory_history": load_memory_history,
                "phase": phase,
            }

    def update_map_activation_phase(
        self,
        phase: str,
        *,
        target_map_name: str,
        target_manifest_sha256: str,
        activation_request_id: str,
        expected_phases: Sequence[str],
    ) -> Dict[str, object]:
        with self.map_activation_guard():
            resolved_phase = str(phase or "").strip()
            allowed_phases = {
                str(item or "").strip() for item in expected_phases
            }
            if resolved_phase not in _ACTIVATION_JOURNAL_PHASES:
                raise ValueError("地图激活日志 phase 非法。")
            if not allowed_phases or not allowed_phases.issubset(
                _ACTIVATION_JOURNAL_PHASES
            ):
                raise ValueError("地图激活日志 expected_phases 非法。")
            payload = self.get_map_activation()
            if payload is None:
                raise ValueError("地图激活日志不存在。")
            self._require_map_activation_identity(
                payload,
                target_map_name=target_map_name,
                target_manifest_sha256=target_manifest_sha256,
                activation_request_id=activation_request_id,
            )
            if payload["phase"] not in allowed_phases:
                raise ValueError(
                    "地图激活日志 phase 已变化：%s。" % payload["phase"]
                )
            payload["phase"] = resolved_phase
            _atomic_write_text(
                self._activation_journal_path,
                json.dumps(payload, ensure_ascii=False, indent=2),
            )
            return payload

    def rotate_map_activation_request_id(
        self,
        *,
        target_map_name: str,
        target_manifest_sha256: str,
        activation_request_id: str,
    ) -> Dict[str, object]:
        """Replace one prepared journal request ID without clearing its fence."""

        with self.map_activation_guard():
            payload = self.get_map_activation()
            if payload is None:
                raise ValueError("地图激活日志不存在。")
            self._require_map_activation_identity(
                payload,
                target_map_name=target_map_name,
                target_manifest_sha256=target_manifest_sha256,
                activation_request_id=activation_request_id,
            )
            if payload["phase"] != "prepared":
                raise ValueError(
                    "地图激活日志 phase 已变化：%s。" % payload["phase"]
                )
            rotated_request_id = "mapload:%s" % uuid.uuid4().hex
            while rotated_request_id == payload["activation_request_id"]:
                rotated_request_id = "mapload:%s" % uuid.uuid4().hex
            payload["activation_request_id"] = rotated_request_id
            _atomic_write_text(
                self._activation_journal_path,
                json.dumps(payload, ensure_ascii=False, indent=2),
            )
            return payload

    def clear_map_activation(
        self,
        *,
        target_map_name: str,
        target_manifest_sha256: str,
        activation_request_id: str,
        expected_phases: Sequence[str],
    ) -> None:
        with self.map_activation_guard():
            payload = self.get_map_activation()
            if payload is None:
                return
            self._require_map_activation_identity(
                payload,
                target_map_name=target_map_name,
                target_manifest_sha256=target_manifest_sha256,
                activation_request_id=activation_request_id,
            )
            allowed_phases = {
                str(item or "").strip() for item in expected_phases
            }
            if not allowed_phases or not allowed_phases.issubset(
                _ACTIVATION_JOURNAL_PHASES
            ):
                raise ValueError("地图激活日志 expected_phases 非法。")
            if payload["phase"] not in allowed_phases:
                raise ValueError(
                    "地图激活日志 phase 已变化：%s。" % payload["phase"]
                )
            tombstone_path = self._activation_journal_path.with_name(
                ".map_activation.cleared.%s"
                % str(payload["activation_request_id"]).removeprefix("mapload:")
            )
            os.replace(self._activation_journal_path, tombstone_path)
            try:
                _fsync_directory(self._activation_journal_path.parent)
            except Exception:
                os.replace(tombstone_path, self._activation_journal_path)
                try:
                    _fsync_directory(self._activation_journal_path.parent)
                except Exception:
                    pass
                raise
            try:
                tombstone_path.unlink(missing_ok=True)
                _fsync_directory(self._activation_journal_path.parent)
            except Exception:
                # The journal removal is already durable; a hidden tombstone is inert.
                pass

    def complete_map_activation_durable(
        self,
        *,
        target_map_name: str,
        target_manifest_sha256: str,
        activation_request_id: str,
        expected_phases: Sequence[str],
    ) -> Dict[str, object]:
        """Persist successful activation identity before durably clearing its journal."""

        with self.map_activation_guard():
            allowed_phases = {
                str(item or "").strip() for item in expected_phases
            }
            if not allowed_phases or not allowed_phases.issubset(
                _ACTIVATION_JOURNAL_PHASES
            ):
                raise ValueError("地图激活日志 expected_phases 非法。")
            payload = self.get_map_activation()
            if payload is None:
                receipt = self.get_last_successful_map_activation_receipt()
                if receipt is None:
                    raise ValueError("地图激活日志不存在。")
                self._require_map_activation_identity(
                    receipt,
                    target_map_name=target_map_name,
                    target_manifest_sha256=target_manifest_sha256,
                    activation_request_id=activation_request_id,
                )
                return receipt
            self._require_map_activation_identity(
                payload,
                target_map_name=target_map_name,
                target_manifest_sha256=target_manifest_sha256,
                activation_request_id=activation_request_id,
            )
            if payload["phase"] not in allowed_phases:
                raise ValueError(
                    "地图激活日志 phase 已变化：%s。" % payload["phase"]
                )
            receipt: Dict[str, object] = {
                "schema_version": _ACTIVATION_RECEIPT_SCHEMA_VERSION,
                "target_map_name": str(payload["target_map_name"]),
                "target_manifest_sha256": str(
                    payload["target_manifest_sha256"]
                ),
                "activation_request_id": str(payload["activation_request_id"]),
            }
            _atomic_write_text(
                self._activation_receipt_path,
                json.dumps(receipt, ensure_ascii=False, indent=2),
            )
            self.clear_map_activation(
                target_map_name=target_map_name,
                target_manifest_sha256=target_manifest_sha256,
                activation_request_id=activation_request_id,
                expected_phases=expected_phases,
            )
            return receipt

    @staticmethod
    def _require_map_activation_identity(
        payload: Dict[str, object],
        *,
        target_map_name: str,
        target_manifest_sha256: str,
        activation_request_id: str,
    ) -> None:
        expected_identity = (
            str(target_map_name or "").strip(),
            str(target_manifest_sha256 or "").strip().lower(),
            str(activation_request_id or "").strip(),
        )
        actual_identity = (
            str(payload.get("target_map_name") or "").strip(),
            str(payload.get("target_manifest_sha256") or "").strip().lower(),
            str(payload.get("activation_request_id") or "").strip(),
        )
        if actual_identity != expected_identity:
            raise ValueError("地图激活日志目标身份已变化。")

    def _metadata_path(self, map_name: str) -> Path:
        return self._map_dir(map_name) / "metadata.json"

    def _load_map_asset(self, metadata_path: Path) -> MapAsset:
        payload = json.loads(metadata_path.read_text(encoding="utf-8"))
        if not isinstance(payload, dict):
            raise ValueError("地图 metadata.json 必须是 JSON object。")
        payload.pop("latest_version_id", None)
        payload.pop("latest_revision", None)
        current_dir = metadata_path.parent / "current"
        manifest_path = current_dir / "manifest.json"
        if "finalized" not in payload:
            payload["finalized"] = bool(current_dir.is_dir() and manifest_path.is_file())
        if payload.get("finalized") and not payload.get("asset_manifest_sha256") and manifest_path.is_file():
            payload["asset_manifest_sha256"] = hashlib.sha256(manifest_path.read_bytes()).hexdigest()
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
