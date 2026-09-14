"""单层探索暂停检查点；依附原地图仓储，保存和读取都核对完整资产。"""
from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
from typing import Literal, Optional

from pydantic import BaseModel, ConfigDict, Field, model_validator

from contracts.geometry import Pose
from contracts.navigation import ExploreAreaRequest
from services.mapping.asset_store import MapAssetBundleRepository
from services.mapping.catalog import MapCatalogRepository, _atomic_write_text


class ExplorationCheckpoint(BaseModel):
    """保存经过停车确认的源进度和当时实际完成的地图资产身份。"""

    model_config = ConfigDict(extra="forbid", frozen=True, allow_inf_nan=False)
    schema_version: Literal[1] = 1
    checkpoint_id: str = Field(min_length=1, max_length=128)
    created_at: datetime = Field(default_factory=lambda: datetime.now(timezone.utc))
    pipeline: Literal["slam_2d", "lio_3d"]
    request: ExploreAreaRequest
    asset_manifest_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    geometry_progress_json: str = Field(min_length=1, max_length=2_097_152)
    localization_seed: Optional[Pose] = None

    @model_validator(mode="after")
    def _check_identity(self):
        value = json.loads(self.geometry_progress_json)
        if value.get("schema_version") != 1 or value.get("map_name") != self.request.map_name or value.get("request_id") != self.request.request_id:
            raise ValueError("探索进度与冻结请求的地图或原任务身份不一致")
        if self.request.resume_progress_json is not None or "motion_admission" in self.request.metadata:
            raise ValueError("持久探索请求不能携带旧运动准入或嵌套恢复载荷")
        if self.localization_seed is not None and self.localization_seed.frame_id != value.get("map_frame_id"):
            raise ValueError("探索定位提示与已保存地图坐标不一致")
        return self


class ExplorationCheckpointStore:
    """文件名使用业务标识摘要；外部调用不提供文件路径。"""

    def __init__(self, root: str | Path):
        self.maps = MapCatalogRepository(str(root))
        self.assets = MapAssetBundleRepository(str(root))
        self.root = Path(root) / "exploration_progress"

    def _path(self, checkpoint_id: str) -> Path:
        if not checkpoint_id or len(checkpoint_id) > 128:
            raise ValueError("探索检查点标识无效")
        return self.root / (hashlib.sha256(checkpoint_id.encode()).hexdigest() + ".json")

    def validate_asset(self, value: ExplorationCheckpoint) -> None:
        asset = self.maps.get_map_asset(value.request.map_name)
        if asset is None or not asset.finalized or asset.asset_manifest_sha256 != value.asset_manifest_sha256:
            raise ValueError("检查点绑定的完整地图已缺失或改变")
        snapshot = self.assets.get_validated_map_asset_bundle_snapshot(value.request.map_name, expected_manifest_sha256=value.asset_manifest_sha256)
        if snapshot is None or snapshot.bundle.geometry_asset is None:
            raise ValueError("检查点没有可恢复的几何资产")
        kind = snapshot.bundle.geometry_asset.metadata.get("target_kind")
        if (value.pipeline == "slam_2d") != (kind == "planar_graph"):
            raise ValueError("检查点地图资产与二维/三维链路不一致")

    def save(self, value: ExplorationCheckpoint) -> None:
        with self.maps.map_activation_guard():
            self.validate_asset(value)
            self.root.mkdir(parents=True, exist_ok=True)
            path = self._path(value.checkpoint_id)
            if path.exists():
                previous = self.load(value.checkpoint_id, validate_asset=False)
                if previous != value:
                    raise ValueError("探索检查点已存在且内容不同")
                return
            payload = value.model_dump_json()
            envelope = {"payload": payload, "sha256": hashlib.sha256(payload.encode()).hexdigest()}
            _atomic_write_text(path, json.dumps(envelope, ensure_ascii=False))

    def load(self, checkpoint_id: str, *, validate_asset: bool = True) -> ExplorationCheckpoint:
        path = self._path(checkpoint_id)
        if path.stat().st_size > 4_194_304:
            raise ValueError("探索检查点超过四兆字节")
        envelope = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(envelope, dict) or set(envelope) != {"payload", "sha256"} or not isinstance(envelope["payload"], str):
            raise ValueError("探索检查点封装无效")
        if hashlib.sha256(envelope["payload"].encode()).hexdigest() != envelope["sha256"]:
            raise ValueError("探索检查点摘要不匹配")
        value = ExplorationCheckpoint.model_validate_json(envelope["payload"])
        if value.checkpoint_id != checkpoint_id:
            raise ValueError("探索检查点身份与索引不符")
        if validate_asset:
            self.validate_asset(value)
        return value

    def list(self, limit: int = 50) -> list[dict]:
        records = []
        for path in sorted(self.root.glob("*.json"), key=lambda p: p.stat().st_mtime, reverse=True)[:max(1, min(limit, 100))]:
            if path.stat().st_size > 4_194_304:
                raise ValueError("探索检查点超过四兆字节")
            envelope = json.loads(path.read_text(encoding="utf-8"))
            value = ExplorationCheckpoint.model_validate_json(envelope["payload"])
            if path != self._path(value.checkpoint_id):
                raise ValueError("探索检查点文件名与身份不符")
            value = self.load(value.checkpoint_id, validate_asset=False)
            available, reason = True, None
            try:
                self.validate_asset(value)
            except ValueError as error:
                available, reason = False, str(error)
            records.append({"checkpoint_id": value.checkpoint_id, "request_id": value.request.request_id, "map_name": value.request.map_name,
                "created_at": value.created_at.isoformat(), "pipeline": value.pipeline, "available": available, "unavailable_reason": reason,
                "asset_manifest_sha256": value.asset_manifest_sha256})
        return records
