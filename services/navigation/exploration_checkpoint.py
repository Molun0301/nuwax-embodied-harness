"""单层探索暂停检查点；依附原地图仓储，保存和读取都核对完整资产。"""
from __future__ import annotations

import heapq
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Literal, Optional, Union

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_serializer, model_validator

from contracts.geometry import Pose
from contracts.map_identity import AssetIdentityV2
from contracts.navigation import ExploreAreaRequest
from services.mapping.asset_store import (
    MapAssetBundleRepository,
    MapAssetBundleValidationError,
    ValidatedMapAssetBundleSnapshot,
)
from services.mapping.catalog import MapCatalogRepository, _atomic_write_text, _fsync_directory
from services.navigation.file_identity import LegacyFileIndex, read_record_text


class ExplorationCheckpoint(BaseModel):
    """保存经过停车确认的源进度和当时实际完成的地图资产身份。"""

    model_config = ConfigDict(extra="forbid", frozen=True, allow_inf_nan=False)
    schema_version: Literal[1, 2] = 2
    checkpoint_id: str = Field(min_length=1, max_length=128)
    created_at: datetime = Field(default_factory=lambda: datetime.now(timezone.utc))
    pipeline: Literal["slam_2d", "lio_3d"]
    request: ExploreAreaRequest
    asset_manifest_sha256: Optional[str] = Field(default=None, pattern=r"^[0-9a-f]{64}$")
    asset_identity: Optional[AssetIdentityV2] = None
    mapping_checkpoint_id: Optional[str] = None
    geometry_progress_json: str = Field(min_length=1, max_length=2_097_152)
    localization_seed: Optional[Pose] = None

    @field_validator("schema_version", mode="before")
    @classmethod
    def _check_version(cls, value):
        if type(value) is not int or value not in (1, 2):
            raise ValueError("探索检查点版本必须为整数 1 或 2")
        return value

    @field_validator("mapping_checkpoint_id", mode="before")
    @classmethod
    def _check_mapping_checkpoint_id(cls, value):
        if value is not None and (not isinstance(value, str) or len(value) != 32
                or any(character not in "0123456789abcdef" for character in value)
                or value == "0" * 32):
            raise ValueError("建图检查点身份必须为非零的三十二位小写十六进制 ID")
        return value

    @model_serializer(mode="wrap")
    def _serialize_identity(self, handler):
        document = handler(self)
        if self.schema_version == 1:
            document.pop("asset_identity", None)
            document.pop("mapping_checkpoint_id", None)
        else:
            document.pop("asset_manifest_sha256", None)
        return document

    @model_validator(mode="after")
    def _check_identity(self):
        if self.schema_version == 1:
            if self.asset_manifest_sha256 is None or self.asset_identity is not None or self.mapping_checkpoint_id is not None:
                raise ValueError("旧探索检查点必须且只能绑定原始地图清单摘要")
        elif self.asset_identity is None or self.mapping_checkpoint_id is None or self.asset_manifest_sha256 is not None:
            raise ValueError("新版探索检查点必须绑定分配身份资产与建图检查点，不能混用清单摘要")
        value = json.loads(self.geometry_progress_json)
        if not isinstance(value, dict) or type(value.get("schema_version")) is not int or value["schema_version"] not in (1, 2) or value.get("map_name") != self.request.map_name or value.get("request_id") != self.request.request_id:
            raise ValueError("探索进度与冻结请求的地图或原任务身份不一致")
        if self.schema_version == 2:
            for name in ("map_name", "map_frame_id", "origin_session_id"):
                if not isinstance(value.get(name), str) or not value[name].strip():
                    raise ValueError("新版探索进度缺少源地图、坐标系或会话身份")
            for name in ("origin_session_generation", "origin_frame_generation"):
                if type(value.get(name)) is not int or value[name] <= 0:
                    raise ValueError("新版探索进度缺少有效源会话或坐标代次")
        if self.request.resume_progress_json is not None or "motion_admission" in self.request.metadata:
            raise ValueError("持久探索请求不能携带旧运动准入或嵌套恢复载荷")
        if self.localization_seed is not None and self.localization_seed.frame_id != value.get("map_frame_id"):
            raise ValueError("探索定位提示与已保存地图坐标不一致")
        return self


class ExplorationCheckpointStore:
    """业务标识直接寻址；旧文件通过有界内存的派生索引读取。"""

    def __init__(self, root: Union[str, Path]):
        self.maps = MapCatalogRepository(str(root))
        self.assets = MapAssetBundleRepository(str(root))
        self.root = Path(root) / "exploration_progress"
        self._files = LegacyFileIndex(self.root, lambda path: self._read(path).checkpoint_id,
            discovery_forbidden=lambda: bool(getattr(self.maps._activation_guard_state.local, "depth", 0)))

    def _path(self, checkpoint_id: str) -> Path:
        return self._files.resolve(checkpoint_id)

    @staticmethod
    def _read(path: Path) -> ExplorationCheckpoint:
        envelope = json.loads(read_record_text(path))
        if not isinstance(envelope, dict) or not isinstance(envelope.get("payload"), str):
            raise ValueError("探索检查点封装无效")  # noqa: TRY004 -- 持久记录准入保持 ValueError 接口
        legacy = set(envelope) == {"payload", "sha256"}
        current = (set(envelope) == {"schema_version", "payload"}
                   and type(envelope["schema_version"]) is int and envelope["schema_version"] == 2)
        if not legacy and not current:
            raise ValueError("探索检查点封装版本无效")
        # 旧本地摘要仅作为输入兼容字段忽略；原生进度及地图资产绑定保持原样。
        document = json.loads(envelope["payload"])
        if not isinstance(document, dict):
            raise ValueError("探索检查点载荷必须是对象")
        if "schema_version" not in document:
            document["schema_version"] = 1
        return ExplorationCheckpoint.model_validate(document)

    def _validate_catalog_identity(self, value: ExplorationCheckpoint) -> None:
        asset = self.maps.get_map_asset(value.request.map_name)
        if asset is None or not asset.finalized or asset.identity_schema_version != 2:
            raise ValueError("检查点绑定的完整地图已缺失或改变")
        if asset.asset_identity != value.asset_identity:
            raise ValueError("检查点绑定的 assigned 地图身份已改变")

    def validate_asset(self, value: ExplorationCheckpoint, *,
                       _previous: Optional[ValidatedMapAssetBundleSnapshot] = None) -> ValidatedMapAssetBundleSnapshot:
        if value.schema_version != 2:
            raise ValueError("旧探索检查点依赖已退役的摘要身份，仅支持读取，不能恢复或重新保存。")
        self._validate_catalog_identity(value)
        try:
            snapshot = self.assets.get_validated_map_asset_bundle_snapshot(value.request.map_name,
                expected_asset_identity=value.asset_identity, _previous=_previous)
        except MapAssetBundleValidationError as error:
            raise ValueError("检查点绑定的完整地图已缺失或改变") from error
        if snapshot is None or snapshot.bundle.geometry_asset is None:
            raise ValueError("检查点没有可恢复的几何资产")
        kind = snapshot.bundle.geometry_asset.metadata.get("target_kind")
        if (value.pipeline == "slam_2d") != (kind == "planar_graph"):
            raise ValueError("检查点地图资产与二维/三维链路不一致")
        progress = json.loads(value.geometry_progress_json)
        source = snapshot.assigned_source_member
        if (snapshot.bundle.identity_schema_version != 2
                or snapshot.bundle.asset_identity != value.asset_identity
                or snapshot.bundle.map_name != progress["map_name"]
                or snapshot.bundle.frame_id != progress["map_frame_id"]
                or source is None or source.role != ("planar_graph" if value.pipeline == "slam_2d" else "canonical_cloud")
                or source.checkpoint_id != value.mapping_checkpoint_id
                or source.frame_id != progress["map_frame_id"]
                or source.occupancy_receipt.get("session_id") != progress["origin_session_id"]
                or source.occupancy_receipt.get("session_generation") != progress["origin_session_generation"]):
            raise ValueError("探索进度与完整地图的原始建图检查点、会话或坐标不一致")
        # 原进度坐标代次属于运行态；规范地图成员固定为 0，不能拿来比较。
        # 外层 metadata 摘要不是源凭据，源身份只取已验证的内部清单。
        return snapshot

    @staticmethod
    def _record_observation(path: Path):
        try:
            value = path.lstat()
        except FileNotFoundError:
            return None
        return (value.st_dev, value.st_ino, value.st_mode, value.st_size,
            value.st_mtime_ns, value.st_ctime_ns)

    def save(self, value: ExplorationCheckpoint) -> None:
        # 在锁外准入并冻结小型记录；原始进度字符串不解析后重写。
        if not isinstance(value, ExplorationCheckpoint):
            raise TypeError("探索检查点必须使用完整类型契约")
        value = ExplorationCheckpoint.model_validate({name: getattr(value, name)
            for name in ExplorationCheckpoint.model_fields})
        if value.schema_version != 2:
            raise ValueError("旧探索检查点依赖已退役的摘要身份，仅支持读取，不能恢复或重新保存。")
        payload = value.model_dump_json()
        value = ExplorationCheckpoint.model_validate_json(payload)
        snapshot = self.validate_asset(value)
        path = self._path(value.checkpoint_id)
        observed = self._record_observation(path)
        if observed is not None:
            previous = self._read(path)
            if previous != value:
                raise ValueError("探索检查点已存在且内容不同")
            if self._record_observation(path) != observed:
                raise ValueError("探索检查点读取期间已改变，请重试")
        envelope = json.dumps({"schema_version": 2, "payload": payload}, ensure_ascii=False)
        with self._files.write_guard(), self.maps.map_activation_guard():
            if self._path(value.checkpoint_id) != path or self._record_observation(path) != observed:
                raise ValueError("探索检查点拥有方已改变，请重试")
            if isinstance(snapshot, ValidatedMapAssetBundleSnapshot):
                self._validate_catalog_identity(value)
                if not self.assets.map_asset_bundle_snapshot_is_current(value.request.map_name, snapshot):
                    raise ValueError("检查点绑定的完整地图在保存前已改变")
            elif value.schema_version == 2:
                raise ValueError("新版探索检查点缺少已验证的完整地图快照")
            self._files.prepare_parent(path)
            if observed is not None:
                # 覆盖上次 rename 成功但目录 fsync 失败；不改写原封装或 created_at。
                _fsync_directory(path.parent)
                return
            _atomic_write_text(path, envelope)

    def load(self, checkpoint_id: str, *, validate_asset: bool = True) -> ExplorationCheckpoint:
        path = self._path(checkpoint_id)
        value = self._read(path)
        if value.checkpoint_id != checkpoint_id:
            raise ValueError("探索检查点身份与索引不符")
        if validate_asset:
            self.validate_asset(value)
        return value

    def list(self, limit: int = 50) -> list[dict]:
        self._files.refresh()
        records = []
        previous_snapshot = None
        paths = heapq.nlargest(max(1, min(limit, 100)), self._files.iter_paths(), key=lambda path: (path.stat().st_mtime_ns, str(path)))
        for path in paths:
            value = self._read(path)
            if path != self._path(value.checkpoint_id):
                raise ValueError("探索检查点文件名与身份不符")
            available, reason = True, None
            try:
                previous_snapshot = self.validate_asset(value, _previous=previous_snapshot)
            except ValueError as error:
                previous_snapshot = None
                available, reason = False, str(error)
            identity = ({"schema_version": 2, "asset_identity": value.asset_identity.model_dump(),
                         "mapping_checkpoint_id": value.mapping_checkpoint_id} if value.schema_version == 2
                        else {"asset_manifest_sha256": value.asset_manifest_sha256})
            records.append({"checkpoint_id": value.checkpoint_id, "request_id": value.request.request_id, "map_name": value.request.map_name,
                "created_at": value.created_at.isoformat(), "pipeline": value.pipeline, "available": available, "unavailable_reason": reason,
                **identity})
        return records
