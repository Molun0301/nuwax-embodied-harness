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
_ACTIVATION_RECEIPT_SCHEMA_VERSION = 2
_RELOCALIZATION_RECEIPT_SCHEMA_VERSION = 1
_RELOCALIZATION_INTENT_SCHEMA_VERSION = 1
_RELOCALIZATION_INTENT_PHASES = frozenset(
    {
        "prepared",
        "backend_dispatching",
        "backend_outcome_unknown",
        "backend_applied_pending_receipt",
    }
)
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
        self._relocalization_receipt_path = (
            self._root_dir / "map_relocalization_receipt.json"
        )
        self._relocalization_intent_path = (
            self._root_dir / "map_relocalization.json"
        )
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
                    receipt.get("trusted_identity") is True
                    and receipt["target_map_name"] == target_name
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
            if not isinstance(payload, dict):
                raise ValueError("地图激活成功凭据必须是 JSON object。")
            schema_version = payload.get("schema_version")
            if isinstance(schema_version, bool) or not isinstance(
                schema_version, int
            ):
                raise ValueError("地图激活成功凭据 schema_version 非法。")
            if schema_version == 1:
                return self._validate_legacy_map_activation_receipt(payload)
            if schema_version != _ACTIVATION_RECEIPT_SCHEMA_VERSION:
                raise ValueError("不支持的地图激活成功凭据 schema_version。")
            return self._validate_map_activation_receipt_v2(payload)

    @staticmethod
    def _validate_legacy_map_activation_receipt(
        payload: Dict[str, object],
    ) -> Dict[str, object]:
        required_fields = {
            "schema_version",
            "target_map_name",
            "target_manifest_sha256",
            "activation_request_id",
        }
        if set(payload) != required_fields:
            raise ValueError("旧版地图激活成功凭据字段不完整或包含未知字段。")
        target_name = MapCatalogRepository._require_bounded_string(
            payload.get("target_map_name"),
            "地图激活成功凭据 target_map_name",
            max_length=256,
        )
        manifest_sha256 = MapCatalogRepository._require_sha256(
            payload.get("target_manifest_sha256"),
            "地图激活成功凭据 target_manifest_sha256",
        )
        activation_request_id = MapCatalogRepository._require_activation_request_id(
            payload.get("activation_request_id")
        )
        if not target_name:
            raise ValueError("地图激活成功凭据 target_map_name 不能为空。")
        return {
            "schema_version": 1,
            "trusted_identity": False,
            "target_map_name": target_name,
            "target_manifest_sha256": manifest_sha256,
            "activation_request_id": activation_request_id,
        }

    @staticmethod
    def _validate_map_activation_receipt_v2(
        payload: Dict[str, object],
    ) -> Dict[str, object]:
        required_fields = {
            "schema_version",
            "target_map_name",
            "target_manifest_sha256",
            "activation_request_id",
            "activation_receipt_schema_version",
            "activation_receipt_disposition",
            "activation_receipt_disposition_name",
            "runtime_generation",
            "session_id",
            "session_generation",
            "backend_generation",
            "mapping_backend_kind",
            "activation_request_identity_sha256",
            "activation_identity_sha256",
            "target_receipt_sha256",
            "asset_bundle_sha256",
            "relocalization_target_available",
            "cloud_source_revision",
            "cloud_source_sha256",
            "source_policy_version",
            "source_policy_sha256",
            "deadline_policy_version",
            "deadline_policy_sha256",
            "algorithm_options_version",
            "algorithm_options_sha256",
        }
        if set(payload) != required_fields:
            raise ValueError("地图激活成功凭据字段不完整或包含未知字段。")
        target_name = MapCatalogRepository._require_bounded_string(
            payload.get("target_map_name"),
            "地图激活成功凭据 target_map_name",
            max_length=256,
        )
        receipt = dict(payload)
        receipt["schema_version"] = _ACTIVATION_RECEIPT_SCHEMA_VERSION
        receipt["target_map_name"] = target_name
        receipt["target_manifest_sha256"] = MapCatalogRepository._require_sha256(
            payload.get("target_manifest_sha256"),
            "地图激活成功凭据 target_manifest_sha256",
        )
        receipt["activation_request_id"] = (
            MapCatalogRepository._require_activation_request_id(
                payload.get("activation_request_id")
            )
        )
        session_id = MapCatalogRepository._require_bounded_string(
            payload.get("session_id"),
            "地图激活成功凭据 session_id",
            max_length=128,
        )
        if session_id != receipt["activation_request_id"]:
            raise ValueError("地图激活成功凭据 session_id 与请求身份不一致。")
        receipt["session_id"] = session_id
        receipt["mapping_backend_kind"] = (
            MapCatalogRepository._require_bounded_string(
                payload.get("mapping_backend_kind"),
                "地图激活成功凭据 mapping_backend_kind",
                max_length=64,
            )
        )
        receipt["activation_receipt_disposition_name"] = (
            MapCatalogRepository._require_bounded_string(
                payload.get("activation_receipt_disposition_name"),
                "地图激活成功凭据 activation_receipt_disposition_name",
                max_length=32,
            )
        )
        if (
            not receipt["mapping_backend_kind"]
            or receipt["activation_receipt_disposition_name"]
            not in {"sealed", "sealed_replay"}
        ):
            raise ValueError("地图激活成功凭据 sealed 身份非法。")
        positive_fields = (
            "runtime_generation",
            "session_generation",
            "backend_generation",
            "cloud_source_revision",
            "source_policy_version",
            "deadline_policy_version",
            "algorithm_options_version",
        )
        for field_name in positive_fields:
            value = MapCatalogRepository._require_positive_int(
                payload.get(field_name),
                "地图激活成功凭据 %s" % field_name,
            )
            receipt[field_name] = value
        receipt_schema_version = MapCatalogRepository._require_positive_int(
            payload.get("activation_receipt_schema_version"),
            "地图激活成功凭据 activation_receipt_schema_version",
        )
        if receipt_schema_version != 1:
            raise ValueError("地图激活成功凭据 wire schema_version 非法。")
        receipt["activation_receipt_schema_version"] = receipt_schema_version
        disposition = MapCatalogRepository._require_positive_int(
            payload.get("activation_receipt_disposition"),
            "地图激活成功凭据 activation_receipt_disposition",
        )
        expected_disposition = {
            "sealed": 2,
            "sealed_replay": 3,
        }[receipt["activation_receipt_disposition_name"]]
        if disposition != expected_disposition:
            raise ValueError("地图激活成功凭据 disposition 身份不一致。")
        receipt["activation_receipt_disposition"] = disposition
        digest_fields = (
            "activation_request_identity_sha256",
            "activation_identity_sha256",
            "target_receipt_sha256",
            "asset_bundle_sha256",
            "cloud_source_sha256",
            "source_policy_sha256",
            "deadline_policy_sha256",
            "algorithm_options_sha256",
        )
        for field_name in digest_fields:
            receipt[field_name] = MapCatalogRepository._require_sha256(
                payload.get(field_name),
                "地图激活成功凭据 %s" % field_name,
            )
        if payload.get("relocalization_target_available") is not True:
            raise ValueError("地图激活成功凭据缺少 sealed 重定位目标。")
        receipt["relocalization_target_available"] = True
        receipt["trusted_identity"] = True
        return receipt

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
            request_suffix = (
                activation_request_id[len("mapload:") :]
                if activation_request_id.startswith("mapload:")
                else activation_request_id
            )
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
                % str(payload["activation_request_id"])[len("mapload:") :]
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
        activation_receipt: Dict[str, object],
        expected_phases: Sequence[str],
    ) -> Dict[str, object]:
        """先持久化完整 sealed 激活凭据，再清除恢复日志。"""

        with self.map_activation_guard():
            allowed_phases = {
                str(item or "").strip() for item in expected_phases
            }
            if not allowed_phases or not allowed_phases.issubset(
                _ACTIVATION_JOURNAL_PHASES
            ):
                raise ValueError("地图激活日志 expected_phases 非法。")
            receipt_payload = {
                **dict(activation_receipt or {}),
                "schema_version": _ACTIVATION_RECEIPT_SCHEMA_VERSION,
                "target_map_name": target_map_name,
                "target_manifest_sha256": target_manifest_sha256,
                "activation_request_id": activation_request_id,
            }
            receipt = self._validate_map_activation_receipt_v2(receipt_payload)
            receipt.pop("trusted_identity", None)
            payload = self.get_map_activation()
            if payload is None:
                existing = self.get_last_successful_map_activation_receipt()
                if existing is None:
                    raise ValueError("地图激活日志不存在。")
                if existing.get("trusted_identity") is not True:
                    raise ValueError("旧版地图激活凭据不能作为完整身份复用。")
                if (
                    self._map_activation_receipt_immutable_identity(existing)
                    != self._map_activation_receipt_immutable_identity(receipt)
                ):
                    raise ValueError("相同激活请求已有不同的 durable 身份凭据。")
                return existing
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
            existing = self.get_last_successful_map_activation_receipt()
            if existing is not None:
                existing_request_id = str(
                    existing.get("activation_request_id") or ""
                ).strip()
                if existing_request_id == activation_request_id:
                    if (
                        self._map_activation_receipt_immutable_identity(existing)
                        != self._map_activation_receipt_immutable_identity(receipt)
                    ):
                        raise ValueError(
                            "相同激活请求已有不同的 durable 身份凭据。"
                        )
                    receipt = dict(existing)
                    receipt.pop("trusted_identity", None)
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
            return {**receipt, "trusted_identity": True}

    @staticmethod
    def _map_activation_receipt_immutable_identity(
        receipt: Dict[str, object],
    ) -> Tuple[object, ...]:
        fields = (
            "target_map_name",
            "target_manifest_sha256",
            "activation_request_id",
            "activation_receipt_schema_version",
            "runtime_generation",
            "session_id",
            "session_generation",
            "backend_generation",
            "mapping_backend_kind",
            "activation_request_identity_sha256",
            "activation_identity_sha256",
            "target_receipt_sha256",
            "asset_bundle_sha256",
            "relocalization_target_available",
            "cloud_source_revision",
            "cloud_source_sha256",
            "source_policy_version",
            "source_policy_sha256",
            "deadline_policy_version",
            "deadline_policy_sha256",
            "algorithm_options_version",
            "algorithm_options_sha256",
        )
        return tuple(receipt.get(field_name) for field_name in fields)

    def begin_relocalization_durable(
        self,
        *,
        request_id: str,
        logical_timeout_ms: int,
        activation_receipt: Dict[str, object],
    ) -> Dict[str, object]:
        """在 RPC dispatch 前持久化绑定 sealed target 的重定位 intent。"""

        with self.map_activation_guard():
            activation = self._require_trusted_activation_receipt(
                activation_receipt
            )
            resolved_request_id = self._require_relocalization_request_id(
                request_id
            )
            timeout_ms = self._require_positive_int(
                logical_timeout_ms,
                "重定位 intent logical_timeout_ms",
            )
            if timeout_ms < 250 or timeout_ms > 30_000:
                raise ValueError("重定位 intent logical_timeout_ms 非法。")
            expected = self._relocalization_intent_identity(
                request_id=resolved_request_id,
                logical_timeout_ms=timeout_ms,
                activation_receipt=activation,
            )
            existing = self.get_relocalization_intent()
            if existing is not None:
                if self._relocalization_intent_identity_from_payload(existing) != expected:
                    raise ValueError("已有不同身份的重定位 durable intent。")
                return existing
            payload = {
                "schema_version": _RELOCALIZATION_INTENT_SCHEMA_VERSION,
                "phase": "prepared",
                **dict(expected),
                "terminal_receipt": None,
            }
            _atomic_write_text(
                self._relocalization_intent_path,
                json.dumps(payload, ensure_ascii=False, indent=2),
            )
            return payload

    def get_relocalization_intent(self) -> Optional[Dict[str, object]]:
        with self.map_activation_guard():
            if not self._relocalization_intent_path.exists():
                return None
            try:
                payload = json.loads(
                    self._relocalization_intent_path.read_text(encoding="utf-8")
                )
            except Exception as exc:
                raise ValueError("重定位 durable intent 不是有效 JSON。") from exc
            return self._validate_relocalization_intent(payload)

    def mark_relocalization_backend_dispatching(
        self,
        *,
        request_id: str,
    ) -> Dict[str, object]:
        """在跨越 provider RPC 边界前持久化 dispatch owner。"""

        with self.map_activation_guard():
            payload = self._require_relocalization_intent_request(request_id)
            if payload["phase"] != "prepared":
                raise ValueError("重定位 intent 不能重复进入 backend_dispatching。")
            payload["phase"] = "backend_dispatching"
            _atomic_write_text(
                self._relocalization_intent_path,
                json.dumps(payload, ensure_ascii=False, indent=2),
            )
            return payload

    def abort_relocalization_before_apply(self, *, request_id: str) -> None:
        """仅在 provider 明确拒绝且未应用时清除 durable owner。"""

        with self.map_activation_guard():
            payload = self._require_relocalization_intent_request(request_id)
            if payload["phase"] not in {"prepared", "backend_dispatching"}:
                raise ValueError("结果未知或已应用的重定位 intent 不能撤销。")
            self._clear_relocalization_intent(request_id)

    def mark_relocalization_outcome_unknown(
        self,
        *,
        request_id: str,
    ) -> Dict[str, object]:
        with self.map_activation_guard():
            payload = self._require_relocalization_intent_request(request_id)
            if payload["phase"] == "backend_applied_pending_receipt":
                return payload
            payload["phase"] = "backend_outcome_unknown"
            _atomic_write_text(
                self._relocalization_intent_path,
                json.dumps(payload, ensure_ascii=False, indent=2),
            )
            return payload

    def mark_relocalization_backend_applied(
        self,
        *,
        relocalization_receipt: Dict[str, object],
    ) -> Dict[str, object]:
        with self.map_activation_guard():
            receipt = self._validate_relocalization_receipt(
                dict(relocalization_receipt or {})
            )
            payload = self._require_relocalization_intent_request(
                str(receipt["request_id"])
            )
            self._require_relocalization_receipt_matches_intent(receipt, payload)
            if payload["phase"] not in {
                "backend_dispatching",
                "backend_outcome_unknown",
                "backend_applied_pending_receipt",
            }:
                raise ValueError("重定位 terminal 不能从 prepared intent 写入。")
            existing_terminal = payload.get("terminal_receipt")
            if existing_terminal is not None:
                validated_existing = self._validate_relocalization_receipt(
                    existing_terminal
                )
                if (
                    self._relocalization_receipt_immutable_identity(validated_existing)
                    != self._relocalization_receipt_immutable_identity(receipt)
                ):
                    raise ValueError("重定位 pending terminal 身份冲突。")
                return payload
            payload["phase"] = "backend_applied_pending_receipt"
            payload["terminal_receipt"] = receipt
            _atomic_write_text(
                self._relocalization_intent_path,
                json.dumps(payload, ensure_ascii=False, indent=2),
            )
            return payload

    @staticmethod
    def _relocalization_intent_identity(
        *,
        request_id: str,
        logical_timeout_ms: int,
        activation_receipt: Dict[str, object],
    ) -> Dict[str, object]:
        return {
            "request_id": request_id,
            "logical_timeout_ms": logical_timeout_ms,
            "target_map_name": activation_receipt["target_map_name"],
            "target_manifest_sha256": activation_receipt[
                "target_manifest_sha256"
            ],
            "activation_request_id": activation_receipt[
                "activation_request_id"
            ],
            "runtime_generation": activation_receipt["runtime_generation"],
            "session_id": activation_receipt["session_id"],
            "session_generation": activation_receipt["session_generation"],
            "backend_generation": activation_receipt["backend_generation"],
            "activation_identity_sha256": activation_receipt[
                "activation_identity_sha256"
            ],
            "target_receipt_sha256": activation_receipt[
                "target_receipt_sha256"
            ],
            "asset_bundle_sha256": activation_receipt["asset_bundle_sha256"],
        }

    @staticmethod
    def _relocalization_intent_identity_from_payload(
        payload: Dict[str, object],
    ) -> Dict[str, object]:
        fields = (
            "request_id",
            "logical_timeout_ms",
            "target_map_name",
            "target_manifest_sha256",
            "activation_request_id",
            "runtime_generation",
            "session_id",
            "session_generation",
            "backend_generation",
            "activation_identity_sha256",
            "target_receipt_sha256",
            "asset_bundle_sha256",
        )
        return {field_name: payload[field_name] for field_name in fields}

    def _validate_relocalization_intent(
        self,
        payload: Dict[str, object],
    ) -> Dict[str, object]:
        required_fields = {
            "schema_version",
            "phase",
            "request_id",
            "logical_timeout_ms",
            "target_map_name",
            "target_manifest_sha256",
            "activation_request_id",
            "runtime_generation",
            "session_id",
            "session_generation",
            "backend_generation",
            "activation_identity_sha256",
            "target_receipt_sha256",
            "asset_bundle_sha256",
            "terminal_receipt",
        }
        if not isinstance(payload, dict) or set(payload) != required_fields:
            raise ValueError("重定位 durable intent 字段不完整或包含未知字段。")
        if payload.get("schema_version") != _RELOCALIZATION_INTENT_SCHEMA_VERSION:
            raise ValueError("不支持的重定位 durable intent schema_version。")
        phase = self._require_bounded_string(
            payload.get("phase"), "重定位 durable intent phase", max_length=64
        )
        if phase not in _RELOCALIZATION_INTENT_PHASES:
            raise ValueError("重定位 durable intent phase 非法。")
        normalized = dict(payload)
        normalized["phase"] = phase
        normalized["request_id"] = self._require_relocalization_request_id(
            payload.get("request_id")
        )
        normalized["logical_timeout_ms"] = self._require_positive_int(
            payload.get("logical_timeout_ms"),
            "重定位 durable intent logical_timeout_ms",
        )
        normalized["target_map_name"] = self._require_bounded_string(
            payload.get("target_map_name"),
            "重定位 durable intent target_map_name",
            max_length=256,
        )
        normalized["target_manifest_sha256"] = self._require_sha256(
            payload.get("target_manifest_sha256"),
            "重定位 durable intent target_manifest_sha256",
        )
        normalized["activation_request_id"] = self._require_activation_request_id(
            payload.get("activation_request_id")
        )
        normalized["session_id"] = self._require_bounded_string(
            payload.get("session_id"),
            "重定位 durable intent session_id",
            max_length=128,
        )
        for field_name in (
            "runtime_generation",
            "session_generation",
            "backend_generation",
        ):
            normalized[field_name] = self._require_positive_int(
                payload.get(field_name),
                "重定位 durable intent %s" % field_name,
            )
        for field_name in (
            "activation_identity_sha256",
            "target_receipt_sha256",
            "asset_bundle_sha256",
        ):
            normalized[field_name] = self._require_sha256(
                payload.get(field_name),
                "重定位 durable intent %s" % field_name,
            )
        terminal = payload.get("terminal_receipt")
        if terminal is not None:
            terminal = self._validate_relocalization_receipt(terminal)
        if phase == "backend_applied_pending_receipt" and terminal is None:
            raise ValueError("重定位 applied pending intent 缺少 terminal receipt。")
        if phase != "backend_applied_pending_receipt" and terminal is not None:
            raise ValueError("重定位 intent 提前包含 terminal receipt。")
        normalized["terminal_receipt"] = terminal
        return normalized

    def _require_relocalization_intent_request(
        self,
        request_id: str,
    ) -> Dict[str, object]:
        payload = self.get_relocalization_intent()
        if payload is None:
            raise ValueError("重定位 durable intent 不存在。")
        if payload["request_id"] != self._require_relocalization_request_id(request_id):
            raise ValueError("重定位 durable intent request_id 已变化。")
        return payload

    @staticmethod
    def _require_relocalization_receipt_matches_intent(
        receipt: Dict[str, object],
        intent: Dict[str, object],
    ) -> None:
        pairs = (
            ("request_id", "request_id"),
            ("logical_timeout_ms", "logical_timeout_ms"),
            ("map_name", "target_map_name"),
            ("asset_manifest_sha256", "target_manifest_sha256"),
            ("runtime_generation", "runtime_generation"),
            ("session_id", "session_id"),
            ("session_generation", "session_generation"),
            ("backend_generation", "backend_generation"),
            ("activation_identity_sha256", "activation_identity_sha256"),
            ("target_receipt_sha256", "target_receipt_sha256"),
            ("asset_bundle_sha256", "asset_bundle_sha256"),
        )
        if any(receipt[left] != intent[right] for left, right in pairs):
            raise ValueError("重定位 terminal receipt 与 durable intent 不一致。")

    def _require_trusted_activation_receipt(
        self,
        activation_receipt: Dict[str, object],
    ) -> Dict[str, object]:
        receipt = dict(activation_receipt or {})
        if receipt.get("trusted_identity") is not True:
            raise ValueError("重定位必须绑定 trusted activation receipt。")
        comparable = dict(receipt)
        comparable.pop("trusted_identity", None)
        validated = self._validate_map_activation_receipt_v2(comparable)
        current = self.get_last_successful_map_activation_receipt()
        if current is None or current.get("trusted_identity") is not True:
            raise ValueError("当前没有 trusted activation receipt。")
        if (
            self._map_activation_receipt_immutable_identity(current)
            != self._map_activation_receipt_immutable_identity(validated)
        ):
            raise ValueError("重定位 target 与当前 activation receipt 不一致。")
        return current

    def get_last_successful_relocalization_receipt(
        self,
    ) -> Optional[Dict[str, object]]:
        with self.map_activation_guard():
            if not self._relocalization_receipt_path.exists():
                return None
            try:
                payload = json.loads(
                    self._relocalization_receipt_path.read_text(encoding="utf-8")
                )
            except Exception as exc:
                raise ValueError("重定位终态凭据不是有效 JSON。") from exc
            return self._validate_relocalization_receipt(payload)

    def validate_relocalization_terminal(
        self,
        receipt: Dict[str, object],
    ) -> Dict[str, object]:
        """返回有界规范 terminal receipt 的防御性副本。"""

        return self._validate_relocalization_receipt(dict(receipt or {}))

    def relocalization_terminals_match(
        self,
        first: Dict[str, object],
        second: Dict[str, object],
    ) -> bool:
        """比较两个规范 terminal receipt 的不可变身份。"""

        validated_first = self._validate_relocalization_receipt(dict(first or {}))
        validated_second = self._validate_relocalization_receipt(dict(second or {}))
        return self._relocalization_receipt_immutable_identity(
            validated_first
        ) == self._relocalization_receipt_immutable_identity(validated_second)

    def reconcile_relocalization_known_terminal(
        self,
        *,
        relocalization_receipt: Dict[str, object],
        activation_receipt: Dict[str, object],
    ) -> Dict[str, object]:
        """在单一短锁内幂等收敛已校验的重定位终态。"""

        with self.map_activation_guard():
            self._require_trusted_activation_receipt(activation_receipt)
            receipt = self._validate_relocalization_receipt(
                dict(relocalization_receipt or {})
            )
            intent = self._require_relocalization_intent_request(
                str(receipt["request_id"])
            )
            self._require_relocalization_receipt_matches_intent(receipt, intent)
            identity = self._relocalization_receipt_immutable_identity(receipt)
            existing = self.get_last_successful_relocalization_receipt()
            if existing is not None and existing["request_id"] == receipt["request_id"]:
                if self._relocalization_receipt_immutable_identity(existing) != identity:
                    raise ValueError("相同重定位 request_id 已绑定不同终态身份。")
                return existing
            if intent["phase"] == "prepared":
                raise ValueError("重定位 terminal 不能从 prepared intent 写入。")
            if intent["phase"] not in {
                "backend_dispatching",
                "backend_outcome_unknown",
                "backend_applied_pending_receipt",
            }:
                raise ValueError("重定位 durable intent phase 非法。")
            terminal = intent.get("terminal_receipt")
            if terminal is None:
                intent["phase"] = "backend_applied_pending_receipt"
                intent["terminal_receipt"] = receipt
                _atomic_write_text(
                    self._relocalization_intent_path,
                    json.dumps(intent, ensure_ascii=False, indent=2),
                )
                intent = self._require_relocalization_intent_request(
                    str(receipt["request_id"])
                )
                terminal = intent.get("terminal_receipt")
            validated_terminal = self._validate_relocalization_receipt(terminal)
            if self._relocalization_receipt_immutable_identity(validated_terminal) != identity:
                raise ValueError("重定位 pending terminal 身份冲突。")
            if intent["phase"] != "backend_applied_pending_receipt":
                raise ValueError("重定位 durable intent 尚未记录 applied terminal。")
            _atomic_write_text(
                self._relocalization_receipt_path,
                json.dumps(receipt, ensure_ascii=False, indent=2),
            )
            durable = self.get_last_successful_relocalization_receipt()
            if (
                durable is None
                or self._relocalization_receipt_immutable_identity(durable) != identity
            ):
                raise ValueError("重定位终态凭据持久化后身份不一致。")
            return durable

    def complete_relocalization_durable(
        self,
        *,
        relocalization_receipt: Dict[str, object],
        activation_receipt: Dict[str, object],
    ) -> Dict[str, object]:
        """对账 trusted target 后持久化 terminal receipt 并清除 intent。"""

        with self.map_activation_guard():
            self._require_trusted_activation_receipt(activation_receipt)
            receipt = self._validate_relocalization_receipt(
                dict(relocalization_receipt or {})
            )
            intent = self._require_relocalization_intent_request(
                str(receipt["request_id"])
            )
            if intent["phase"] != "backend_applied_pending_receipt":
                raise ValueError("重定位 durable intent 尚未记录 applied terminal。")
            self._require_relocalization_receipt_matches_intent(receipt, intent)
            terminal = self._validate_relocalization_receipt(
                intent["terminal_receipt"]
            )
            if (
                self._relocalization_receipt_immutable_identity(terminal)
                != self._relocalization_receipt_immutable_identity(receipt)
            ):
                raise ValueError("重定位 terminal receipt 与 pending 身份冲突。")
            existing = self.get_last_successful_relocalization_receipt()
            if existing is not None and existing["request_id"] == receipt["request_id"]:
                if (
                    self._relocalization_receipt_immutable_identity(existing)
                    != self._relocalization_receipt_immutable_identity(receipt)
                ):
                    raise ValueError("相同重定位 request_id 已绑定不同终态身份。")
                receipt = existing
            else:
                _atomic_write_text(
                    self._relocalization_receipt_path,
                    json.dumps(receipt, ensure_ascii=False, indent=2),
                )
            return receipt

    def settle_relocalization_durable(
        self,
        *,
        request_id: str,
        activation_receipt: Dict[str, object],
    ) -> Dict[str, object]:
        """仅在 terminal receipt 与会话同步均完成后清除 pending intent。"""

        with self.map_activation_guard():
            self._require_trusted_activation_receipt(activation_receipt)
            intent = self._require_relocalization_intent_request(request_id)
            if intent["phase"] != "backend_applied_pending_receipt":
                raise ValueError("重定位 durable intent 尚未进入 applied pending。")
            terminal = self._validate_relocalization_receipt(
                intent["terminal_receipt"]
            )
            durable = self.get_last_successful_relocalization_receipt()
            if (
                durable is None
                or self._relocalization_receipt_immutable_identity(durable)
                != self._relocalization_receipt_immutable_identity(terminal)
            ):
                raise ValueError("重定位 terminal receipt 尚未 durable。")
            self._clear_relocalization_intent(request_id)
            return durable

    def _clear_relocalization_intent(self, request_id: str) -> None:
        payload = self._require_relocalization_intent_request(request_id)
        tombstone = self._relocalization_intent_path.with_name(
            ".map_relocalization.cleared.%s" % payload["request_id"].replace(":", "_")
        )
        os.replace(self._relocalization_intent_path, tombstone)
        try:
            _fsync_directory(self._relocalization_intent_path.parent)
        except Exception:
            os.replace(tombstone, self._relocalization_intent_path)
            try:
                _fsync_directory(self._relocalization_intent_path.parent)
            except Exception:
                pass
            raise
        try:
            tombstone.unlink(missing_ok=True)
            _fsync_directory(self._relocalization_intent_path.parent)
        except Exception:
            pass

    @staticmethod
    def _relocalization_receipt_immutable_identity(
        receipt: Dict[str, object],
    ) -> Tuple[object, ...]:
        fields = (
            "request_id",
            "request_identity_sha256",
            "runtime_generation",
            "session_id",
            "map_name",
            "session_generation",
            "backend_generation",
            "activation_identity_sha256",
            "target_receipt_sha256",
            "target_cloud_revision",
            "target_cloud_sha256",
            "asset_manifest_sha256",
            "asset_bundle_sha256",
            "source_policy_version",
            "source_policy_sha256",
            "deadline_policy_version",
            "deadline_policy_sha256",
            "logical_timeout_ms",
            "algorithm_options_version",
            "algorithm_options_sha256",
            "candidate_sequence",
            "candidate_payload_sha256",
            "evidence_sha256",
            "correction_revision",
            "terminal_proof_sha256",
        )
        return tuple(receipt.get(field_name) for field_name in fields)

    @staticmethod
    def _validate_relocalization_receipt(
        payload: Dict[str, object],
    ) -> Dict[str, object]:
        required_fields = {
            "schema_version",
            "request_id",
            "request_identity_sha256",
            "runtime_generation",
            "session_id",
            "map_name",
            "session_generation",
            "backend_generation",
            "activation_identity_sha256",
            "target_receipt_sha256",
            "target_cloud_revision",
            "target_cloud_sha256",
            "asset_manifest_sha256",
            "asset_bundle_sha256",
            "source_policy_version",
            "source_policy_sha256",
            "deadline_policy_version",
            "deadline_policy_sha256",
            "logical_timeout_ms",
            "algorithm_options_version",
            "algorithm_options_sha256",
            "disposition",
            "disposition_name",
            "candidate_sequence",
            "candidate_payload_sha256",
            "evidence_sha256",
            "correction_revision",
            "terminal_proof_sha256",
        }
        if not isinstance(payload, dict) or set(payload) != required_fields:
            raise ValueError("重定位终态凭据字段不完整或包含未知字段。")
        schema_version = payload.get("schema_version")
        if (
            isinstance(schema_version, bool)
            or not isinstance(schema_version, int)
            or schema_version != _RELOCALIZATION_RECEIPT_SCHEMA_VERSION
        ):
            raise ValueError("不支持的重定位终态凭据 schema_version。")
        receipt = dict(payload)
        string_limits = {
            "request_id": 128,
            "session_id": 128,
            "map_name": 256,
            "disposition_name": 32,
        }
        for field_name, max_length in string_limits.items():
            receipt[field_name] = MapCatalogRepository._require_bounded_string(
                payload.get(field_name),
                "重定位终态凭据 %s" % field_name,
                max_length=max_length,
            )
        if (
            len(receipt["request_id"]) > 128
            or any(
                not (
                    character.isascii()
                    and (character.isalnum() or character in "._:-")
                )
                for character in receipt["request_id"]
            )
        ):
            raise ValueError("重定位终态凭据 request_id 非法。")
        if receipt["disposition_name"] not in {"applied", "already_applied"}:
            raise ValueError("重定位终态凭据不是成功终态。")
        positive_fields = (
            "runtime_generation",
            "session_generation",
            "backend_generation",
            "target_cloud_revision",
            "source_policy_version",
            "deadline_policy_version",
            "logical_timeout_ms",
            "algorithm_options_version",
            "candidate_sequence",
            "correction_revision",
        )
        for field_name in positive_fields:
            receipt[field_name] = MapCatalogRepository._require_positive_int(
                payload.get(field_name),
                "重定位终态凭据 %s" % field_name,
            )
        disposition = MapCatalogRepository._require_positive_int(
            payload.get("disposition"),
            "重定位终态凭据 disposition",
        )
        expected_disposition = {
            "applied": 1,
            "already_applied": 2,
        }[receipt["disposition_name"]]
        if disposition != expected_disposition:
            raise ValueError("重定位终态凭据 disposition 身份不一致。")
        receipt["disposition"] = disposition
        digest_fields = (
            "request_identity_sha256",
            "activation_identity_sha256",
            "target_receipt_sha256",
            "target_cloud_sha256",
            "asset_manifest_sha256",
            "asset_bundle_sha256",
            "source_policy_sha256",
            "deadline_policy_sha256",
            "algorithm_options_sha256",
            "candidate_payload_sha256",
            "evidence_sha256",
            "terminal_proof_sha256",
        )
        for field_name in digest_fields:
            receipt[field_name] = MapCatalogRepository._require_sha256(
                payload.get(field_name),
                "重定位终态凭据 %s" % field_name,
            )
        return receipt

    @staticmethod
    def _require_bounded_string(
        value: object,
        field_name: str,
        *,
        max_length: int,
    ) -> str:
        if not isinstance(value, str):
            raise ValueError("%s 非法。" % field_name)
        resolved = value.strip()
        if not resolved or len(resolved) > max_length:
            raise ValueError("%s 非法。" % field_name)
        return resolved

    @staticmethod
    def _require_sha256(value: object, field_name: str) -> str:
        if not isinstance(value, str):
            raise ValueError("%s 非法。" % field_name)
        digest = value.strip().lower()
        if len(digest) != 64 or any(
            character not in "0123456789abcdef" for character in digest
        ):
            raise ValueError("%s 非法。" % field_name)
        return digest

    @staticmethod
    def _require_positive_int(value: object, field_name: str) -> int:
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError("%s 非法。" % field_name)
        if value <= 0 or value > 0x7FFF_FFFF_FFFF_FFFF:
            raise ValueError("%s 非法。" % field_name)
        return value

    @staticmethod
    def _require_relocalization_request_id(value: object) -> str:
        request_id = MapCatalogRepository._require_bounded_string(
            value,
            "重定位 request_id",
            max_length=128,
        )
        if any(
            not (
                character.isascii()
                and (character.isalnum() or character in "._:-")
            )
            for character in request_id
        ):
            raise ValueError("重定位 request_id 非法。")
        return request_id

    @staticmethod
    def _require_activation_request_id(value: object) -> str:
        activation_request_id = MapCatalogRepository._require_bounded_string(
            value,
            "地图激活成功凭据 activation_request_id",
            max_length=40,
        )
        request_suffix = (
            activation_request_id[len("mapload:") :]
            if activation_request_id.startswith("mapload:")
            else activation_request_id
        )
        if (
            not activation_request_id.startswith("mapload:")
            or len(request_suffix) != 32
            or any(
                character not in "0123456789abcdef"
                for character in request_suffix
            )
        ):
            raise ValueError("地图激活成功凭据 activation_request_id 非法。")
        return activation_request_id

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
