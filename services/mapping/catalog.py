from __future__ import annotations

from contextlib import contextmanager
import fcntl
import json
import math
import os
from pathlib import Path
import shutil
import tempfile
import threading
from urllib.parse import quote
import uuid
import weakref

from contracts.map_workspace import MapAsset, AssetIdentityV2
from contracts.map_activation import (
    AssignedActivationReceipt,
    _integer as _assigned_integer,
    _request as _admit_assigned_activation_request,
    _source_session as _assigned_source_session,
    _text as _assigned_text,
)
from contracts.map_relocalization import (
    AssignedRelocalizationQuery,
    AssignedRelocalizationRequest,
    AssignedRelocalizationTerminal,
    _normalize as _normalize_assigned_metadata,
    current_assigned_relocalization_policy,
    current_assigned_planar_relocalization_policy,
)
from contracts.naming import build_safe_dir_segment
from typing import Dict, Iterator, List, Optional, Sequence, Tuple


_ACTIVATION_JOURNAL_SCHEMA_VERSION = 2
_ACTIVATION_RECEIPT_SCHEMA_VERSION = 2
_RELOCALIZATION_RECEIPT_SCHEMA_VERSION = 1
_RELOCALIZATION_INTENT_SCHEMA_VERSION = 1
# Durable envelopes are separate from the native request/result schema tag.
_ASSIGNED_RELOCALIZATION_DURABLE_SCHEMA_VERSION = 2
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

_ASSIGNED_ACTIVATION_JOURNAL_PHASES = _ACTIVATION_JOURNAL_PHASES | frozenset(
    {"predecessor_stop_dispatching", "predecessor_stopped"}
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

    @staticmethod
    def _assigned_asset_identity(value: AssetIdentityV2) -> Dict[str, object]:
        if not isinstance(value, AssetIdentityV2):
            raise ValueError("assigned activation requires typed AssetIdentityV2")
        return {"asset_id": AssetIdentityV2(asset_id=value.asset_id).asset_id}

    @staticmethod
    def _assigned_checkpoint_id(value: object) -> str:
        if (not isinstance(value, str) or len(value) != 32 or value == "0" * 32
                or any(c not in "0123456789abcdef" for c in value)):
            raise ValueError("assigned checkpoint ID is missing or invalid")
        return value

    @staticmethod
    def _assigned_activation_predecessor(value):
        fields = {"session_id", "map_name", "session_generation", "runtime_generation", "mapping_backend_kind"}
        if not isinstance(value, dict) or set(value) != fields:
            raise ValueError("assigned activation predecessor fields are invalid")
        if (not isinstance(value["mapping_backend_kind"], str)
                or value["mapping_backend_kind"] not in {"voxel", "raytracing", "slam_toolbox"}):
            raise ValueError("assigned activation predecessor backend is invalid")
        return {
            "session_id": _assigned_source_session(value["session_id"]),
            "map_name": _assigned_text(value["map_name"], "predecessor map name", 256),
            "session_generation": _assigned_integer(value["session_generation"], "predecessor session generation"),
            "runtime_generation": _assigned_integer(value["runtime_generation"], "predecessor runtime generation"),
            "mapping_backend_kind": value["mapping_backend_kind"],
        }

    @staticmethod
    def _assigned_activation_policy(value, backend_kind=None):
        # Admission keeps the policy actually returned by the backend. It never
        # replaces an absent or unsupported policy with locally current values.
        policy = _normalize_assigned_metadata(value, "policy")
        expected = (current_assigned_planar_relocalization_policy() if backend_kind == "slam_toolbox"
            else current_assigned_relocalization_policy())
        if policy != expected:
            raise ValueError("assigned activation policy parameters differ")
        return policy

    @staticmethod
    def _validate_assigned_activation_evidence(payload, activation_request):
        fields = {"activation_receipt_schema_version", "activation_receipt_v2",
                  "activation_receipt_disposition", "activation_receipt_disposition_name",
                  "relocalization_target_available", "relocalization_policy_v2"}
        if not isinstance(payload, dict) or set(payload) != fields:
            raise ValueError("assigned activation backend evidence fields are invalid")
        if type(payload["activation_receipt_schema_version"]) is not int or payload["activation_receipt_schema_version"] != 2:
            raise ValueError("assigned activation backend evidence requires native receipt 2")
        owner = AssignedActivationReceipt.admit(payload["activation_receipt_v2"])
        receipt = owner.to_dict()
        if receipt["request"] != activation_request:
            raise ValueError("assigned activation backend evidence request differs")
        disposition = payload["activation_receipt_disposition"]
        name = payload["activation_receipt_disposition_name"]
        if (type(disposition) is not int or not isinstance(name, str)
                or (disposition, name) not in {(2, "sealed"), (3, "sealed_replay")}):
            raise ValueError("assigned activation backend evidence is not sealed")
        if type(payload["relocalization_target_available"]) is not bool:
            raise ValueError("assigned target availability must be explicit")
        result = {
            "activation_receipt_schema_version": 2, "activation_receipt_v2": receipt,
            "activation_receipt_disposition": disposition, "activation_receipt_disposition_name": name,
            "relocalization_target_available": payload["relocalization_target_available"],
            "relocalization_policy_v2": MapCatalogRepository._assigned_activation_policy(
                payload["relocalization_policy_v2"], receipt["activation"]["backend_kind"]),
        }
        # Existing native metadata admission owns all field limits. In particular,
        # source session text has no new journal-specific length/alphabet gate.
        return result

    @staticmethod
    def _same_assigned_activation_evidence(left, right):
        # Both labels acknowledge the same installed owner. A state query may
        # report sealed after a replay ACK; retain the first admitted label.
        labels = {"activation_receipt_disposition", "activation_receipt_disposition_name"}
        return ({key: value for key, value in left.items() if key not in labels}
                == {key: value for key, value in right.items() if key not in labels})

    @staticmethod
    def _validate_assigned_activation_journal(payload: Dict[str, object]) -> Dict[str, object]:
        fields = {"schema_version", "previous_map_name", "target_map_name", "target_asset_identity",
                  "target_checkpoint_id", "activation_request_id", "load_memory_history", "phase"}
        extended = {"activation_request", "predecessor", "backend_evidence"}
        if (not isinstance(payload, dict) or set(payload) not in (fields, fields | extended)
                or type(payload["schema_version"]) is not int or payload["schema_version"] != 3):
            raise ValueError("assigned activation journal schema/fields are invalid")
        raw_asset = payload["target_asset_identity"]
        if not isinstance(raw_asset, dict) or set(raw_asset) != {"asset_id"}:
            raise ValueError("assigned activation journal asset identity is missing")
        asset = AssetIdentityV2(**raw_asset)
        previous = payload["previous_map_name"]
        if previous is not None:
            previous = MapCatalogRepository._require_bounded_string(previous, "previous map name", max_length=256)
        target = MapCatalogRepository._require_bounded_string(payload["target_map_name"], "target map name", max_length=256)
        full_request = "activation_request" in payload
        phases = _ASSIGNED_ACTIVATION_JOURNAL_PHASES if full_request else _ACTIVATION_JOURNAL_PHASES
        if (type(payload["load_memory_history"]) is not bool or not isinstance(payload["phase"], str)
                or payload["phase"] not in phases):
            raise ValueError("assigned activation journal flags/phase are invalid")
        result = {"schema_version": 3, "previous_map_name": previous, "target_map_name": target,
                  "target_asset_identity": {"asset_id": asset.asset_id},
                  "target_checkpoint_id": MapCatalogRepository._assigned_checkpoint_id(payload["target_checkpoint_id"]),
                  "activation_request_id": payload["activation_request_id"],
                  "load_memory_history": payload["load_memory_history"], "phase": payload["phase"]}
        if not full_request:
            result["activation_request_id"] = MapCatalogRepository._require_activation_request_id(payload["activation_request_id"])
            return result
        request = _admit_assigned_activation_request(payload["activation_request"])
        MapCatalogRepository._require_assigned_activation_identity(result,
            target_map_name=request["map_name"], target_asset_identity=AssetIdentityV2(**request["asset_identity"]),
            target_checkpoint_id=request["checkpoint_id"], activation_request_id=request["activation_request_id"])
        predecessor = (None if payload["predecessor"] is None
                       else MapCatalogRepository._assigned_activation_predecessor(payload["predecessor"]))
        if payload["phase"] in {"predecessor_stop_dispatching", "predecessor_stopped"} and predecessor is None:
            raise ValueError("assigned predecessor stop phase requires the retained predecessor")
        evidence = (None if payload["backend_evidence"] is None
                    else MapCatalogRepository._validate_assigned_activation_evidence(payload["backend_evidence"], request))
        if payload["phase"] == "backend_applied" and evidence is None:
            raise ValueError("assigned backend_applied phase requires durable backend evidence")
        if evidence is not None and payload["phase"] not in {"backend_applied", "reconciling", "recovering"}:
            raise ValueError("assigned backend evidence cannot regress to dispatch")
        result.update(activation_request=request, predecessor=predecessor, backend_evidence=evidence)
        return result

    @staticmethod
    def _validate_assigned_activation_receipt(payload: Dict[str, object]) -> Dict[str, object]:
        fields = {"schema_version", "target_map_name", "target_asset_identity", "target_checkpoint_id",
                  "activation_request_id", "activation_receipt_schema_version", "activation_receipt_disposition",
                  "activation_receipt_disposition_name", "relocalization_target_available", "activation_receipt_v2"}
        extended = {"activation_request", "relocalization_policy_v2"}
        if (not isinstance(payload, dict) or set(payload) not in (fields, fields | extended)
                or type(payload["schema_version"]) is not int or payload["schema_version"] != 3):
            raise ValueError("assigned durable receipt schema/fields are invalid")
        if type(payload["activation_receipt_schema_version"]) is not int or payload["activation_receipt_schema_version"] != 2:
            raise ValueError("assigned durable receipt requires native wire receipt 2")
        raw_asset = payload["target_asset_identity"]
        if not isinstance(raw_asset, dict) or set(raw_asset) != {"asset_id"}:
            raise ValueError("assigned durable receipt asset identity is missing")
        asset = AssetIdentityV2(**raw_asset)
        owner = AssignedActivationReceipt.admit(payload["activation_receipt_v2"])
        target = MapCatalogRepository._require_bounded_string(payload["target_map_name"], "target map name", max_length=256)
        full_request = "activation_request" in payload
        request = _admit_assigned_activation_request(payload["activation_request"]) if full_request else None
        request_id = (request["activation_request_id"] if full_request
                      else MapCatalogRepository._require_activation_request_id(payload["activation_request_id"]))
        checkpoint_id = MapCatalogRepository._assigned_checkpoint_id(payload["target_checkpoint_id"])
        if (owner.asset_identity != asset or owner.map_name != target or owner.request_id != request_id
                or payload["activation_request_id"] != request_id or owner.checkpoint_id != checkpoint_id
                or (owner.to_dict()["request"] != request if full_request else owner.session_id != request_id)):
            raise ValueError("assigned durable receipt differs from its transaction target")
        disposition = payload["activation_receipt_disposition"]
        name = payload["activation_receipt_disposition_name"]
        if type(disposition) is not int or not isinstance(name, str) or (disposition, name) not in {(2, "sealed"), (3, "sealed_replay")}:
            raise ValueError("assigned durable receipt is not sealed")
        if type(payload["relocalization_target_available"]) is not bool:
            raise ValueError("assigned target availability must be explicit")
        result = {"schema_version": 3, "target_map_name": target, "target_asset_identity": {"asset_id": asset.asset_id},
                  "target_checkpoint_id": checkpoint_id, "activation_request_id": request_id,
                  "activation_receipt_schema_version": 2, "activation_receipt_disposition": disposition,
                  "activation_receipt_disposition_name": name,
                  "relocalization_target_available": payload["relocalization_target_available"],
                  "activation_receipt_v2": owner.to_dict(), "trusted_identity": True}
        if full_request:
            evidence = MapCatalogRepository._validate_assigned_activation_evidence(
                {key: payload[key] for key in ("activation_receipt_schema_version", "activation_receipt_v2",
                    "activation_receipt_disposition", "activation_receipt_disposition_name",
                    "relocalization_target_available", "relocalization_policy_v2")}, request)
            result.update(activation_request=request, relocalization_policy_v2=evidence["relocalization_policy_v2"])
        return result

    @staticmethod
    def _require_assigned_activation_identity(payload, *, target_map_name, target_asset_identity,
                                              target_checkpoint_id, activation_request_id, activation_request=None):
        expected = (target_map_name, MapCatalogRepository._assigned_asset_identity(target_asset_identity),
                    MapCatalogRepository._assigned_checkpoint_id(target_checkpoint_id), activation_request_id)
        actual = (payload.get("target_map_name"), payload.get("target_asset_identity"),
                  payload.get("target_checkpoint_id"), payload.get("activation_request_id"))
        if payload.get("schema_version") != 3 or actual != expected:
            raise ValueError("assigned activation transaction identity changed")
        if activation_request is not None or "activation_request" in payload:
            if activation_request is None or "activation_request" not in payload:
                raise ValueError("assigned activation requires its retained whole request")
            if _admit_assigned_activation_request(activation_request) != payload["activation_request"]:
                raise ValueError("assigned activation whole request changed")

    @staticmethod
    def _assigned_activation_immutable_identity(receipt):
        return (receipt.get("schema_version"), receipt.get("target_map_name"), receipt.get("target_asset_identity"),
                receipt.get("target_checkpoint_id"), receipt.get("activation_request_id"),
                receipt.get("activation_receipt_schema_version"), receipt.get("activation_receipt_v2"),
                receipt.get("relocalization_target_available"), receipt.get("activation_request"),
                receipt.get("relocalization_policy_v2"))

    def begin_assigned_map_activation(self, *, previous_map_name, target_map_name,
                                     target_asset_identity: AssetIdentityV2, target_checkpoint_id: str,
                                     load_memory_history: bool, reuse_completed_request_id: bool = True,
                                     activation_request=None, predecessor=None):
        """Retain a full assigned request before any caller-owned backend dispatch."""
        request = None if activation_request is None else _admit_assigned_activation_request(activation_request)
        if request is None and predecessor is not None:
            raise ValueError("assigned predecessor requires the retained whole request")
        candidate = {
            "schema_version": 3, "previous_map_name": previous_map_name, "target_map_name": target_map_name,
            "target_asset_identity": self._assigned_asset_identity(target_asset_identity),
            "target_checkpoint_id": target_checkpoint_id,
            "activation_request_id": request["activation_request_id"] if request is not None else "mapload:" + uuid.uuid4().hex,
            "load_memory_history": load_memory_history, "phase": "prepared"}
        if request is not None:
            candidate.update(activation_request=request, predecessor=predecessor, backend_evidence=None)
            # A caller-owned full request is never regenerated from a receipt.
            reuse_completed_request_id = False
        candidate = self._validate_assigned_activation_journal(candidate)
        with self.map_activation_guard():
            existing = self.get_map_activation()
            if existing is not None:
                ignored = {"phase", "backend_evidence"}
                if request is None:
                    ignored.add("activation_request_id")
                comparable = {key: value for key, value in existing.items() if key not in ignored}
                expected = {key: value for key, value in candidate.items() if key not in ignored}
                if comparable != expected:
                    raise ValueError("another activation transaction or conflicting retry is pending")
                return existing
            if reuse_completed_request_id:
                receipt = self.get_last_successful_map_activation_receipt()
                if (receipt is not None and receipt.get("schema_version") == 3
                        and "activation_request" not in receipt
                        and receipt["target_map_name"] == candidate["target_map_name"]
                        and receipt["target_asset_identity"] == candidate["target_asset_identity"]
                        and receipt["target_checkpoint_id"] == candidate["target_checkpoint_id"]):
                    candidate["activation_request_id"] = receipt["activation_request_id"]
            _atomic_write_text(self._activation_journal_path, json.dumps(candidate, ensure_ascii=False, indent=2))
            return candidate

    def update_assigned_map_activation_phase(self, phase, *, target_map_name, target_asset_identity,
                                            target_checkpoint_id, activation_request_id, expected_phases,
                                            activation_request=None):
        if not isinstance(phase, str) or phase not in _ASSIGNED_ACTIVATION_JOURNAL_PHASES:
            raise ValueError("unsupported assigned activation phase")
        allowed = set(expected_phases)
        if not allowed or not allowed.issubset(_ASSIGNED_ACTIVATION_JOURNAL_PHASES):
            raise ValueError("unsupported assigned activation expected phases")
        with self.map_activation_guard():
            payload = self.get_map_activation()
            if payload is None:
                raise ValueError("assigned activation journal is absent")
            self._require_assigned_activation_identity(payload, target_map_name=target_map_name,
                target_asset_identity=target_asset_identity, target_checkpoint_id=target_checkpoint_id,
                activation_request_id=activation_request_id, activation_request=activation_request)
            if payload["phase"] not in allowed:
                raise ValueError("assigned activation phase changed")
            if "activation_request" in payload:
                order = {"prepared": 0, "predecessor_stop_dispatching": 1, "predecessor_stopped": 2,
                         "backend_dispatching": 3, "backend_applied": 4, "reconciling": 5, "recovering": 5}
                if order[phase] < order[payload["phase"]]:
                    raise ValueError("assigned activation phase cannot regress")
                if phase in {"backend_applied", "reconciling", "recovering"} and payload["backend_evidence"] is None:
                    raise ValueError("assigned adoption phase requires durable backend evidence")
                if (phase == "backend_dispatching" and payload["predecessor"] is not None
                        and payload["phase"] not in {"predecessor_stopped", "backend_dispatching"}):
                    raise ValueError("assigned backend dispatch requires its predecessor stopped phase")
            payload["phase"] = phase
            payload = self._validate_assigned_activation_journal(payload)
            _atomic_write_text(self._activation_journal_path, json.dumps(payload, ensure_ascii=False, indent=2))
            return payload

    def record_assigned_map_activation_evidence(self, *, target_map_name, target_asset_identity,
                                              target_checkpoint_id, activation_request_id, activation_request,
                                              activation_receipt: AssignedActivationReceipt,
                                              relocalization_policy_v2, relocalization_target_available,
                                              activation_receipt_disposition, activation_receipt_disposition_name,
                                              expected_phases):
        """Fsync exact backend ACK metadata before application adoption begins."""
        if not isinstance(activation_receipt, AssignedActivationReceipt):
            raise ValueError("backend evidence requires an admitted assigned activation receipt owner")
        request = _admit_assigned_activation_request(activation_request)
        evidence = self._validate_assigned_activation_evidence({
            "activation_receipt_schema_version": 2, "activation_receipt_v2": activation_receipt.to_dict(),
            "activation_receipt_disposition": activation_receipt_disposition,
            "activation_receipt_disposition_name": activation_receipt_disposition_name,
            "relocalization_target_available": relocalization_target_available,
            "relocalization_policy_v2": relocalization_policy_v2}, request)
        allowed = set(expected_phases)
        if not allowed or not allowed.issubset(_ASSIGNED_ACTIVATION_JOURNAL_PHASES):
            raise ValueError("unsupported assigned activation expected phases")
        with self.map_activation_guard():
            payload = self.get_map_activation()
            if payload is None:
                raise ValueError("assigned activation journal is absent")
            self._require_assigned_activation_identity(payload, target_map_name=target_map_name,
                target_asset_identity=target_asset_identity, target_checkpoint_id=target_checkpoint_id,
                activation_request_id=activation_request_id, activation_request=request)
            if payload["phase"] not in allowed:
                raise ValueError("assigned activation phase changed")
            if payload["backend_evidence"] is not None:
                if not self._same_assigned_activation_evidence(payload["backend_evidence"], evidence):
                    raise ValueError("assigned activation has conflicting backend evidence")
                # A prior atomic replacement may have succeeded before its
                # directory fsync raised. An exact retry must settle durability
                # before allowing adoption, without replacing retained evidence.
                _fsync_directory(self._activation_journal_path.parent)
                return payload
            if payload["phase"] != "backend_dispatching":
                raise ValueError("assigned backend evidence requires the retained dispatch phase")
            payload.update(backend_evidence=evidence, phase="backend_applied")
            _atomic_write_text(self._activation_journal_path, json.dumps(payload, ensure_ascii=False, indent=2))
            return payload

    def rotate_assigned_map_activation_request_id(self, *, target_map_name, target_asset_identity,
                                                 target_checkpoint_id, activation_request_id):
        with self.map_activation_guard():
            payload = self.get_map_activation()
            if payload is None:
                raise ValueError("assigned activation journal is absent")
            if "activation_request" in payload:
                raise ValueError("retained whole assigned activation request cannot rotate")
            self._require_assigned_activation_identity(payload, target_map_name=target_map_name,
                target_asset_identity=target_asset_identity, target_checkpoint_id=target_checkpoint_id,
                activation_request_id=activation_request_id)
            if payload["phase"] != "prepared":
                raise ValueError("only prepared assigned activation may rotate request identity")
            replacement = "mapload:" + uuid.uuid4().hex
            while replacement == activation_request_id:
                replacement = "mapload:" + uuid.uuid4().hex
            payload["activation_request_id"] = replacement
            _atomic_write_text(self._activation_journal_path, json.dumps(payload, ensure_ascii=False, indent=2))
            return payload

    def clear_assigned_map_activation(self, *, target_map_name, target_asset_identity,
                                     target_checkpoint_id, activation_request_id, expected_phases,
                                     activation_request=None):
        allowed = set(expected_phases)
        if not allowed or not allowed.issubset(_ASSIGNED_ACTIVATION_JOURNAL_PHASES):
            raise ValueError("unsupported assigned activation expected phases")
        with self.map_activation_guard():
            payload = self.get_map_activation()
            if payload is None:
                return
            self._require_assigned_activation_identity(payload, target_map_name=target_map_name,
                target_asset_identity=target_asset_identity, target_checkpoint_id=target_checkpoint_id,
                activation_request_id=activation_request_id, activation_request=activation_request)
            if payload["phase"] not in allowed:
                raise ValueError("assigned activation phase changed")
            if "activation_request" in payload:
                evidence = payload["backend_evidence"]
                if evidence is None:
                    if payload["phase"] != "prepared":
                        raise ValueError("cannot clear unresolved assigned backend ownership")
                else:
                    receipt = self.get_last_successful_map_activation_receipt()
                    if (receipt is None or receipt.get("activation_request") != payload["activation_request"]
                            or not self._same_assigned_activation_evidence(evidence,
                                {key: receipt.get(key) for key in evidence})):
                        raise ValueError("assigned activation clear requires its exact durable evidence")
            self._clear_current_activation_journal(payload)

    def complete_assigned_map_activation_durable(self, *, target_map_name, target_asset_identity,
                                               target_checkpoint_id, activation_request_id,
                                               activation_receipt: AssignedActivationReceipt,
                                               expected_phases, idempotent_replay=False,
                                               relocalization_target_available=False, activation_request=None,
                                               retain_journal=False):
        if not isinstance(activation_receipt, AssignedActivationReceipt):
            raise ValueError("completion requires an admitted assigned activation receipt owner")
        if type(idempotent_replay) is not bool or type(retain_journal) is not bool:
            raise ValueError("idempotent replay and retain journal flags must be explicit")
        allowed = set(expected_phases)
        if not allowed or not allowed.issubset(_ASSIGNED_ACTIVATION_JOURNAL_PHASES):
            raise ValueError("unsupported assigned activation expected phases")
        candidate = {
            "schema_version": 3, "target_map_name": target_map_name,
            "target_asset_identity": self._assigned_asset_identity(target_asset_identity),
            "target_checkpoint_id": target_checkpoint_id, "activation_request_id": activation_request_id,
            "activation_receipt_schema_version": 2, "activation_receipt_disposition": 3 if idempotent_replay else 2,
            "activation_receipt_disposition_name": "sealed_replay" if idempotent_replay else "sealed",
            "relocalization_target_available": relocalization_target_available,
            "activation_receipt_v2": activation_receipt.to_dict()}
        with self.map_activation_guard():
            payload = self.get_map_activation()
            existing = self.get_last_successful_map_activation_receipt()
            if payload is not None:
                self._require_assigned_activation_identity(payload, target_map_name=target_map_name,
                    target_asset_identity=target_asset_identity, target_checkpoint_id=target_checkpoint_id,
                    activation_request_id=activation_request_id, activation_request=activation_request)
                if payload["phase"] not in allowed:
                    raise ValueError("assigned activation phase changed")
            if activation_request is not None:
                request = _admit_assigned_activation_request(activation_request)
                if payload is not None:
                    evidence = payload["backend_evidence"]
                    if evidence is None:
                        raise ValueError("assigned completion requires durable backend evidence")
                elif existing is not None and existing.get("activation_request") == request:
                    evidence = {key: existing[key] for key in (
                        "activation_receipt_schema_version", "activation_receipt_v2", "activation_receipt_disposition",
                        "activation_receipt_disposition_name", "relocalization_target_available", "relocalization_policy_v2")}
                else:
                    raise ValueError("assigned activation journal/receipt is absent or differs")
                if (evidence["activation_receipt_v2"] != candidate["activation_receipt_v2"]
                        or evidence["relocalization_target_available"] != relocalization_target_available):
                    raise ValueError("assigned completion has conflicting backend evidence")
                # Preserve the original ACK disposition and policy across recovery.
                candidate.update(evidence)
                candidate["activation_request"] = request
            receipt = self._validate_assigned_activation_receipt(candidate)
            receipt.pop("trusted_identity")
            if existing is not None and existing.get("activation_request_id") == activation_request_id:
                if self._assigned_activation_immutable_identity(existing) != self._assigned_activation_immutable_identity(receipt):
                    raise ValueError("same activation request has a conflicting durable receipt")
                receipt = {key: value for key, value in existing.items() if key != "trusted_identity"}
            if payload is None:
                if existing is None or self._assigned_activation_immutable_identity(existing) != self._assigned_activation_immutable_identity(receipt):
                    raise ValueError("assigned activation journal/receipt is absent or differs")
                return existing
            # Receipt fsync precedes journal removal, preserving crash reconciliation.
            _atomic_write_text(self._activation_receipt_path, json.dumps(receipt, ensure_ascii=False, indent=2))
            if not retain_journal:
                self._clear_current_activation_journal(payload)
            return {**receipt, "trusted_identity": True}

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
                if existing.get("schema_version") == 3:
                    raise ValueError("assigned activation journal requires typed assigned APIs")
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
                    and receipt.get("schema_version") == _ACTIVATION_RECEIPT_SCHEMA_VERSION
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
            if schema_version == 3:
                return self._validate_assigned_activation_receipt(payload)
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
        planar = payload.get("target_kind") == "planar_graph"
        if planar:
            required_fields.update({"target_kind", "target_graph_sha256"})
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
        target_available = payload.get("relocalization_target_available")
        if not isinstance(target_available, bool):
            raise ValueError("地图激活成功凭据重定位资格必须是布尔值。")
        positive_fields = (
            "runtime_generation",
            "session_generation",
            "backend_generation",
            "source_policy_version",
            "deadline_policy_version",
            "algorithm_options_version",
        )
        if target_available and not planar:
            positive_fields += ("cloud_source_revision",)
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
            "asset_bundle_sha256",
            "source_policy_sha256",
            "deadline_policy_sha256",
            "algorithm_options_sha256",
        )
        if target_available:
            digest_fields += ("target_receipt_sha256", "target_graph_sha256" if planar else "cloud_source_sha256")
        for field_name in digest_fields:
            receipt[field_name] = MapCatalogRepository._require_sha256(
                payload.get(field_name),
                "地图激活成功凭据 %s" % field_name,
            )
        if not target_available:
            for field_name, empty_value in (
                ("target_receipt_sha256", ""),
                ("cloud_source_sha256", ""),
                ("cloud_source_revision", 0),
            ):
                if payload.get(field_name) != empty_value:
                    raise ValueError("二维地图激活凭据不能携带不完整的三维目标身份。")
                receipt[field_name] = empty_value
        if planar and (receipt["mapping_backend_kind"] != "slam_toolbox" or not target_available or payload.get("cloud_source_revision") != 0 or payload.get("cloud_source_sha256") != ""):
            raise ValueError("二维完整图激活凭据不能冒充三维目标")
        receipt["relocalization_target_available"] = target_available
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
            if isinstance(payload, dict) and payload.get("schema_version") == 3:
                return self._validate_assigned_activation_journal(payload)
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
            self._clear_current_activation_journal(payload)

    def _clear_current_activation_journal(self, payload):
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
        return tuple(receipt.get(field_name) for field_name in fields + ("target_kind", "target_graph_sha256", "planar_seed"))

    @staticmethod
    def _require_assigned_relocalization_request(
        request: AssignedRelocalizationRequest,
    ) -> AssignedRelocalizationRequest:
        if not isinstance(request, AssignedRelocalizationRequest):
            raise ValueError("assigned relocalization requires an admitted request owner")
        return AssignedRelocalizationRequest.from_dict(request.to_dict())

    def _require_assigned_relocalization_ownership(
        self, request: AssignedRelocalizationRequest,
        activation_receipt: AssignedActivationReceipt,
        target_asset_identity: AssetIdentityV2,
    ) -> None:
        """Check retained owners under the journal guard, without loading payloads.

        Callers publishing/removing asset metadata must share this guard (as the
        normal workspace delete path already does). Direct repository asset
        save/delete calls do not provide atomic exclusion with this check.
        """
        if not isinstance(activation_receipt, AssignedActivationReceipt):
            raise ValueError("assigned relocalization requires a typed activation receipt")
        asset_identity = self._assigned_asset_identity(target_asset_identity)
        if (request.asset_identity.asset_id != asset_identity["asset_id"]
                or not request.activation_receipt.exact_replay_of(activation_receipt)):
            raise ValueError("assigned request differs from activation/asset ownership")
        current = self.get_last_successful_map_activation_receipt()
        if (current is None or current.get("schema_version") != 3
                or current.get("trusted_identity") is not True
                or current.get("relocalization_target_available") is not True):
            raise ValueError("assigned relocalization requires a current trusted sealed target")
        owner = AssignedActivationReceipt.admit(current["activation_receipt_v2"])
        if not owner.exact_replay_of(activation_receipt):
            raise ValueError("assigned activation ownership changed or was retired")
        if self.get_map_activation() is not None:
            raise ValueError("an activation transition fences assigned relocalization")
        metadata_path = self._metadata_path(owner.map_name)
        if not metadata_path.is_file():
            raise ValueError("assigned finalized asset ownership changed or was retired")
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        if not isinstance(metadata, dict) or metadata.get("identity_schema_version") != 2:
            # Do not call the legacy loader: it can reconstruct a manifest SHA.
            raise ValueError("assigned finalized asset ownership changed or was retired")
        asset = MapAsset.model_validate(metadata)
        if (asset.map_name != owner.map_name or not asset.finalized
                or asset.identity_schema_version != 2
                or asset.asset_identity != target_asset_identity):
            raise ValueError("assigned finalized asset ownership changed or was retired")

    @staticmethod
    def _assigned_relocalization_receipt(
        terminal: AssignedRelocalizationTerminal,
    ) -> Dict[str, object]:
        if not isinstance(terminal, AssignedRelocalizationTerminal):
            raise ValueError("assigned completion requires an admitted terminal owner")
        terminal = AssignedRelocalizationTerminal.from_dict(terminal.to_dict())
        if terminal.disposition not in {1, 2}:
            raise ValueError("assigned successful receipt requires an applied terminal")
        return {"schema_version": _ASSIGNED_RELOCALIZATION_DURABLE_SCHEMA_VERSION,
                "request_id": terminal.request_id,
                "assigned_terminal_v2": terminal.to_dict()}

    @staticmethod
    def _validate_assigned_relocalization_receipt(payload) -> Dict[str, object]:
        fields = {"schema_version", "request_id", "assigned_terminal_v2"}
        if (not isinstance(payload, dict) or set(payload) != fields
                or type(payload["schema_version"]) is not int
                or payload["schema_version"] != _ASSIGNED_RELOCALIZATION_DURABLE_SCHEMA_VERSION):
            raise ValueError("assigned relocalization receipt schema/fields are invalid")
        terminal = AssignedRelocalizationTerminal.from_dict(payload["assigned_terminal_v2"])
        if payload["request_id"] != terminal.request_id:
            raise ValueError("assigned receipt request identity differs")
        return MapCatalogRepository._assigned_relocalization_receipt(terminal)

    @staticmethod
    def _validate_assigned_relocalization_intent(payload) -> Dict[str, object]:
        fields = {"schema_version", "phase", "request_id", "assigned_request_v2", "terminal_receipt"}
        if (not isinstance(payload, dict) or set(payload) != fields
                or type(payload["schema_version"]) is not int
                or payload["schema_version"] != _ASSIGNED_RELOCALIZATION_DURABLE_SCHEMA_VERSION):
            raise ValueError("assigned relocalization intent schema/fields are invalid")
        request = AssignedRelocalizationRequest.from_dict(payload["assigned_request_v2"])
        phase = payload["phase"]
        if (payload["request_id"] != request.request_id or not isinstance(phase, str)
                or phase not in _RELOCALIZATION_INTENT_PHASES):
            raise ValueError("assigned relocalization intent identity/phase is invalid")
        receipt = payload["terminal_receipt"]
        if (phase == "backend_applied_pending_receipt") != (receipt is not None):
            raise ValueError("assigned pending phase and terminal presence differ")
        if receipt is not None:
            receipt = MapCatalogRepository._validate_assigned_relocalization_receipt(receipt)
            terminal = AssignedRelocalizationTerminal.from_dict(receipt["assigned_terminal_v2"])
            if not request.exact_replay_of(terminal.request):
                raise ValueError("assigned pending terminal differs from the whole request")
        return {"schema_version": _ASSIGNED_RELOCALIZATION_DURABLE_SCHEMA_VERSION,
                "phase": phase, "request_id": request.request_id,
                "assigned_request_v2": request.to_dict(), "terminal_receipt": receipt}

    def get_assigned_relocalization_intent(self) -> Optional[Dict[str, object]]:
        """Reopen only the explicit assigned lane; never adopt legacy intents."""
        payload = self.get_relocalization_intent()
        if payload is not None and payload["schema_version"] != _ASSIGNED_RELOCALIZATION_DURABLE_SCHEMA_VERSION:
            raise ValueError("legacy pending intent requires its original recovery lifecycle")
        return payload

    def _require_assigned_relocalization_intent(self, request) -> Dict[str, object]:
        payload = self.get_assigned_relocalization_intent()
        if payload is None:
            raise ValueError("assigned relocalization intent is absent")
        stored = AssignedRelocalizationRequest.from_dict(payload["assigned_request_v2"])
        if not request.exact_replay_of(stored):
            raise ValueError("assigned relocalization whole request identity changed")
        return payload

    def reopen_assigned_relocalization_query(self, *, request_id: str) -> AssignedRelocalizationQuery:
        """Read the durable normalized request, even when its live owner retired.

        Not-found, pending and version-skew outcomes confer no permission to
        redispatch or release this owner. No legacy SHA alias can be recovered
        here: legacy pending intents never retained request_identity_sha256.
        """
        with self.map_activation_guard():
            payload = self.get_assigned_relocalization_intent()
            if payload is None or payload["request_id"] != request_id:
                raise ValueError("assigned relocalization query owner is absent or changed")
            return AssignedRelocalizationQuery.from_dict({
                "schema_version": 2, "request_id": request_id,
                "assigned_request_v2": payload["assigned_request_v2"]})

    def begin_assigned_relocalization_durable(
        self, *, assigned_request: AssignedRelocalizationRequest,
        activation_receipt: AssignedActivationReceipt, target_asset_identity: AssetIdentityV2,
    ) -> Dict[str, object]:
        request = self._require_assigned_relocalization_request(assigned_request)
        with self.map_activation_guard():
            self._require_assigned_relocalization_ownership(request, activation_receipt, target_asset_identity)
            existing = self.get_relocalization_intent()
            if existing is not None:
                return self._require_assigned_relocalization_intent(request)
            receipt = self.get_last_successful_relocalization_receipt()
            terminal = None
            if receipt is not None and receipt["request_id"] == request.request_id:
                receipt = self._validate_assigned_relocalization_receipt(receipt)
                completed = AssignedRelocalizationTerminal.from_dict(receipt["assigned_terminal_v2"])
                if not request.exact_replay_of(completed.request):
                    raise ValueError("same assigned request ID has a conflicting durable receipt")
                # A settled retry can only re-synchronize the retained terminal.
                # It must never become prepared or dispatch the backend again.
                terminal = receipt
            payload = {"schema_version": _ASSIGNED_RELOCALIZATION_DURABLE_SCHEMA_VERSION,
                       "phase": "backend_applied_pending_receipt" if terminal else "prepared",
                       "request_id": request.request_id, "assigned_request_v2": request.to_dict(),
                       "terminal_receipt": terminal}
            _atomic_write_text(self._relocalization_intent_path, json.dumps(payload, ensure_ascii=False, indent=2))
            return payload

    def _update_assigned_relocalization_phase(
        self, phase, *, assigned_request, activation_receipt, target_asset_identity,
    ) -> Dict[str, object]:
        request = self._require_assigned_relocalization_request(assigned_request)
        with self.map_activation_guard():
            self._require_assigned_relocalization_ownership(request, activation_receipt, target_asset_identity)
            payload = self._require_assigned_relocalization_intent(request)
            if phase == "backend_dispatching":
                if payload["phase"] != "prepared":
                    raise ValueError("assigned relocalization cannot dispatch twice")
            elif phase == "backend_outcome_unknown":
                if payload["phase"] in {"backend_outcome_unknown", "backend_applied_pending_receipt"}:
                    return payload
                if payload["phase"] != "backend_dispatching":
                    raise ValueError("assigned prepared owner has not crossed dispatch")
            else:
                raise ValueError("unsupported assigned relocalization transition")
            payload["phase"] = phase
            _atomic_write_text(self._relocalization_intent_path, json.dumps(payload, ensure_ascii=False, indent=2))
            return payload

    def mark_assigned_relocalization_backend_dispatching(
        self, *, assigned_request, activation_receipt, target_asset_identity,
    ) -> Dict[str, object]:
        """Persist the single dispatch owner before crossing the provider boundary."""
        return self._update_assigned_relocalization_phase("backend_dispatching",
            assigned_request=assigned_request, activation_receipt=activation_receipt,
            target_asset_identity=target_asset_identity)

    def mark_assigned_relocalization_outcome_unknown(
        self, *, assigned_request, activation_receipt, target_asset_identity,
    ) -> Dict[str, object]:
        return self._update_assigned_relocalization_phase("backend_outcome_unknown",
            assigned_request=assigned_request, activation_receipt=activation_receipt,
            target_asset_identity=target_asset_identity)

    def abort_assigned_relocalization_before_apply(
        self, *, assigned_request, activation_receipt, target_asset_identity,
        terminal: Optional[AssignedRelocalizationTerminal] = None,
    ) -> None:
        """Release prepared intent or an exact, explicitly rejected dispatch.

        Unknown/applied owners remain fenced; their retirement is a separate
        lifecycle operation and is intentionally not inferred from query misses.
        """
        request = self._require_assigned_relocalization_request(assigned_request)
        if terminal is not None:
            if not isinstance(terminal, AssignedRelocalizationTerminal):
                raise ValueError("assigned rejection requires an admitted terminal")
            terminal = AssignedRelocalizationTerminal.from_dict(terminal.to_dict())
            if not request.exact_replay_of(terminal.request) or terminal.disposition in {1, 2}:
                raise ValueError("assigned rejection is applied or belongs to another request")
        with self.map_activation_guard():
            self._require_assigned_relocalization_ownership(request, activation_receipt, target_asset_identity)
            payload = self._require_assigned_relocalization_intent(request)
            if (payload["phase"] not in {"prepared", "backend_dispatching"}
                    or payload["phase"] == "backend_dispatching" and terminal is None):
                raise ValueError("assigned unknown/applied dispatch cannot be aborted")
            self._clear_current_relocalization_intent(payload)

    def mark_assigned_relocalization_backend_applied(
        self, *, relocalization_terminal: AssignedRelocalizationTerminal,
        activation_receipt: AssignedActivationReceipt, target_asset_identity: AssetIdentityV2,
    ) -> Dict[str, object]:
        receipt = self._assigned_relocalization_receipt(relocalization_terminal)
        request = relocalization_terminal.request
        with self.map_activation_guard():
            self._require_assigned_relocalization_ownership(request, activation_receipt, target_asset_identity)
            payload = self._require_assigned_relocalization_intent(request)
            if payload["phase"] == "prepared":
                raise ValueError("assigned terminal cannot precede backend dispatch")
            if payload["terminal_receipt"] is not None:
                if payload["terminal_receipt"] != receipt:
                    raise ValueError("assigned pending terminal conflicts with retained exact terminal")
                return payload
            payload.update(phase="backend_applied_pending_receipt", terminal_receipt=receipt)
            _atomic_write_text(self._relocalization_intent_path, json.dumps(payload, ensure_ascii=False, indent=2))
            return payload

    def complete_assigned_relocalization_durable(
        self, *, relocalization_terminal: AssignedRelocalizationTerminal,
        activation_receipt: AssignedActivationReceipt, target_asset_identity: AssetIdentityV2,
    ) -> Dict[str, object]:
        """Fsync the exact successful receipt; keep intent until session sync."""
        receipt = self._assigned_relocalization_receipt(relocalization_terminal)
        request = relocalization_terminal.request
        with self.map_activation_guard():
            self._require_assigned_relocalization_ownership(request, activation_receipt, target_asset_identity)
            existing = self.get_last_successful_relocalization_receipt()
            if existing is not None and existing["request_id"] == request.request_id and existing != receipt:
                raise ValueError("same assigned request ID has a conflicting durable terminal")
            intent = self.get_assigned_relocalization_intent()
            if intent is None:
                if existing != receipt:
                    raise ValueError("assigned relocalization intent/receipt is absent or differs")
                _atomic_write_text(self._relocalization_receipt_path, json.dumps(receipt, ensure_ascii=False, indent=2))
                return existing
            intent = self._require_assigned_relocalization_intent(request)
            if intent["phase"] != "backend_applied_pending_receipt" or intent["terminal_receipt"] != receipt:
                raise ValueError("assigned terminal is not retained by its pending intent")
            # 相同可见内容也需重新确认持久化：前次 rename 后目录 fsync 可能失败。
            _atomic_write_text(self._relocalization_receipt_path, json.dumps(receipt, ensure_ascii=False, indent=2))
            durable = self.get_last_successful_relocalization_receipt()
            if durable != receipt:
                raise ValueError("assigned terminal receipt changed after durable write")
            return durable

    def reconcile_assigned_relocalization_known_terminal(
        self, *, relocalization_terminal, activation_receipt, target_asset_identity,
    ) -> Dict[str, object]:
        """Retain and persist one exact terminal within the existing short guard."""
        with self.map_activation_guard():
            if self.get_assigned_relocalization_intent() is not None:
                self.mark_assigned_relocalization_backend_applied(
                    relocalization_terminal=relocalization_terminal,
                    activation_receipt=activation_receipt, target_asset_identity=target_asset_identity)
            return self.complete_assigned_relocalization_durable(
                relocalization_terminal=relocalization_terminal,
                activation_receipt=activation_receipt, target_asset_identity=target_asset_identity)

    def settle_assigned_relocalization_durable(
        self, *, assigned_request, activation_receipt, target_asset_identity,
    ) -> Dict[str, object]:
        """Caller has synchronized the session; release only the exact durable owner."""
        request = self._require_assigned_relocalization_request(assigned_request)
        with self.map_activation_guard():
            self._require_assigned_relocalization_ownership(request, activation_receipt, target_asset_identity)
            durable = self.get_last_successful_relocalization_receipt()
            if durable is None:
                raise ValueError("assigned terminal receipt is not durable")
            durable = self._validate_assigned_relocalization_receipt(durable)
            terminal = AssignedRelocalizationTerminal.from_dict(durable["assigned_terminal_v2"])
            if not request.exact_replay_of(terminal.request):
                raise ValueError("assigned durable receipt belongs to another whole request")
            intent = self.get_assigned_relocalization_intent()
            if intent is None:
                return durable
            intent = self._require_assigned_relocalization_intent(request)
            if intent["phase"] != "backend_applied_pending_receipt" or intent["terminal_receipt"] != durable:
                raise ValueError("assigned pending terminal does not match durable receipt")
            self._clear_current_relocalization_intent(intent)
            return durable

    def settle_assigned_relocalization_query(
        self, *, query: AssignedRelocalizationQuery, response: Dict[str, object],
        replacement_activation: Optional[AssignedActivationReceipt] = None,
    ) -> Dict[str, object]:
        """Release an exact authoritative query terminal without installing it.

        A negative dispatch reply is deliberately insufficient here: it may be
        a new pre-admission rejection rather than the old uncertain attempt's
        retained terminal. Pending/not-found/conflict queries never release.
        Applied results require a different currently durable sealed activation;
        deletion without such a receipt still needs a native retirement fence.
        """
        if not isinstance(query, AssignedRelocalizationQuery):
            raise ValueError("assigned recovery requires the retained whole query")
        terminal = query.validate_response(response)
        if terminal is None:
            raise ValueError("assigned recovery requires an authoritative terminal query")
        request = query.request
        applied = terminal.disposition in {1, 2}
        receipt = self._assigned_relocalization_receipt(terminal) if applied else None
        with self.map_activation_guard():
            if applied:
                if (not isinstance(replacement_activation, AssignedActivationReceipt)
                        or replacement_activation.exact_replay_of(request.activation_receipt)):
                    raise ValueError("applied recovery requires a distinct sealed replacement owner")
                current = self.get_last_successful_map_activation_receipt()
                if (current is None or current.get("schema_version") != 3
                        or current.get("trusted_identity") is not True
                        or not AssignedActivationReceipt.admit(current["activation_receipt_v2"])
                            .exact_replay_of(replacement_activation)):
                    raise ValueError("assigned replacement activation is not the current durable owner")
            existing = self.get_last_successful_relocalization_receipt()
            if existing is not None and existing["request_id"] == request.request_id:
                if not applied or existing != receipt:
                    raise ValueError("queried terminal conflicts with durable successful receipt")
            intent = self.get_assigned_relocalization_intent()
            if intent is not None:
                intent = self._require_assigned_relocalization_intent(request)
                if intent["phase"] == "prepared":
                    raise ValueError("query retirement cannot precede backend dispatch")
                if intent["terminal_receipt"] is not None and intent["terminal_receipt"] != receipt:
                    raise ValueError("queried terminal conflicts with retained pending terminal")
                self._clear_current_relocalization_intent(intent)
            # Do not overwrite a later successful receipt or install old geometry.
            return {"assigned_request_v2": request.to_dict(),
                    "assigned_terminal_v2": terminal.to_dict(), "retired": applied}

    def begin_relocalization_durable(
        self,
        *,
        request_id: str,
        logical_timeout_ms: int,
        activation_receipt: Dict[str, object],
        planar_seed: Optional[List[float]] = None,
    ) -> Dict[str, object]:
        """在 RPC dispatch 前持久化绑定 sealed target 的重定位 intent。"""

        with self.map_activation_guard():
            activation = self._require_trusted_activation_receipt(
                activation_receipt
            )
            if activation.get("relocalization_target_available") is not True:
                raise ValueError("当前地图没有已封存的重定位目标。")
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
                planar_seed=planar_seed,
            )
            existing = self.get_relocalization_intent()
            if existing is not None:
                if existing.get("schema_version") == _ASSIGNED_RELOCALIZATION_DURABLE_SCHEMA_VERSION:
                    raise ValueError("assigned relocalization intent requires typed assigned APIs")
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
            if isinstance(payload, dict) and payload.get("schema_version") == _ASSIGNED_RELOCALIZATION_DURABLE_SCHEMA_VERSION:
                return self._validate_assigned_relocalization_intent(payload)
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
    def _validate_planar_seed(value):
        if not isinstance(value, (list, tuple)) or len(value) != 3 or any(isinstance(v, bool) or not isinstance(v, (float,int)) or not math.isfinite(v) for v in value) or abs(value[2]) > math.pi:
            raise ValueError("二维初始位置必须为有限的x、y、偏航角")
        return [float(v) for v in value]

    @staticmethod
    def _relocalization_intent_identity(
        *,
        request_id: str,
        logical_timeout_ms: int,
        activation_receipt: Dict[str, object],
        planar_seed: Optional[List[float]] = None,
    ) -> Dict[str, object]:
        if activation_receipt.get("target_kind") == "planar_graph":
            planar_seed = MapCatalogRepository._validate_planar_seed(planar_seed)
        elif planar_seed is not None:
            raise ValueError("三维重定位不能携带二维初值")
        return {
            **({"planar_seed": planar_seed} if planar_seed is not None else {}),
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
        if "planar_seed" in payload:
            fields += ("planar_seed",)
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
        if isinstance(payload, dict) and "planar_seed" in payload:
            required_fields.add("planar_seed")
            self._validate_planar_seed(payload["planar_seed"])
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
        if payload.get("schema_version") == _ASSIGNED_RELOCALIZATION_DURABLE_SCHEMA_VERSION:
            raise ValueError("assigned relocalization intent requires typed assigned APIs")
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
        if any(receipt[left] != intent[right] for left, right in pairs) or ("planar_seed" in intent and receipt.get("planar_seed") != intent["planar_seed"]):
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
            if isinstance(payload, dict) and payload.get("schema_version") == _ASSIGNED_RELOCALIZATION_DURABLE_SCHEMA_VERSION:
                return self._validate_assigned_relocalization_receipt(payload)
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
        self._clear_current_relocalization_intent(payload)

    def _clear_current_relocalization_intent(self, payload: Dict[str, object]) -> None:
        # Both lanes use the same tombstone/fsync rollback protocol. Callers
        # hold map_activation_guard and validate their own typed exact owner.
        if self.get_relocalization_intent() != payload:
            raise ValueError("relocalization intent changed before release")
        tombstone = self._relocalization_intent_path.with_name(
            ".map_relocalization.cleared.%s" % payload["request_id"].replace(":", "_")
        )
        try:
            os.replace(self._relocalization_intent_path, tombstone)
            _fsync_directory(self._relocalization_intent_path.parent)
        except BaseException:
            # rename 本身也可能已完成后被取消。只恢复精确原内容，link 的
            # no-replace 语义不会覆盖并发写入的其它拥有方。
            try:
                if (not self._relocalization_intent_path.exists()
                        and not self._relocalization_intent_path.is_symlink()
                        and tombstone.is_file() and not tombstone.is_symlink()
                        and json.loads(tombstone.read_text(encoding="utf-8")) == payload):
                    os.link(tombstone, self._relocalization_intent_path, follow_symlinks=False)
                    tombstone.unlink()
                    _fsync_directory(self._relocalization_intent_path.parent)
            except BaseException:
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
        return tuple(receipt.get(field_name) for field_name in fields + ("target_kind", "target_graph_sha256", "planar_seed"))

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
        planar = isinstance(payload, dict) and payload.get("target_kind") == "planar_graph"
        if planar:
            required_fields.update({"target_kind", "target_graph_sha256", "planar_seed"})
            MapCatalogRepository._validate_planar_seed(payload.get("planar_seed"))
            MapCatalogRepository._require_sha256(payload.get("target_graph_sha256"), "二维图摘要")
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
        if planar:
            positive_fields = tuple(k for k in positive_fields if k not in {"target_cloud_revision", "candidate_sequence"})
            if payload.get("target_cloud_revision") != 0 or payload.get("candidate_sequence") != 0 or payload.get("target_cloud_sha256") != "" or payload.get("candidate_payload_sha256") != "":
                raise ValueError("二维重定位不能携带三维候选凭据")
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
        if planar:
            digest_fields = tuple(k for k in digest_fields if k not in {"target_cloud_sha256", "candidate_payload_sha256"})
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
        if payload.get("schema_version") == 3:
            raise ValueError("assigned activation journal requires typed assigned APIs")
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
