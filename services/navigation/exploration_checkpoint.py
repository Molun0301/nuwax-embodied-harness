"""单层探索暂停检查点；依附原地图仓储，保存和读取都核对完整资产。"""
from __future__ import annotations

from datetime import datetime, timezone
import heapq
import json
from pathlib import Path
from typing import Literal, Optional

from pydantic import BaseModel, ConfigDict, Field, model_validator

from contracts.geometry import Pose
from contracts.navigation import ExploreAreaRequest
from services.mapping.asset_store import MapAssetBundleRepository
from services.mapping.catalog import MapCatalogRepository, _atomic_write_text
from services.navigation.file_identity import LegacyFileIndex, read_record_text


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
    """业务标识直接寻址；旧文件通过有界内存的派生索引读取。"""

    def __init__(self, root: str | Path):
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
            raise ValueError("探索检查点封装无效")
        legacy = set(envelope) == {"payload", "sha256"}
        current = (set(envelope) == {"schema_version", "payload"}
                   and type(envelope["schema_version"]) is int and envelope["schema_version"] == 2)
        if not legacy and not current:
            raise ValueError("探索检查点封装版本无效")
        # 旧本地摘要仅作为输入兼容字段忽略；原生进度及地图资产绑定保持原样。
        return ExplorationCheckpoint.model_validate_json(envelope["payload"])

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
        with self._files.write_guard(), self.maps.map_activation_guard():
            self.validate_asset(value)
            self.root.mkdir(parents=True, exist_ok=True)
            path = self._path(value.checkpoint_id)
            if path.exists():
                previous = self.load(value.checkpoint_id, validate_asset=False)
                if previous != value:
                    raise ValueError("探索检查点已存在且内容不同")
                return
            payload = value.model_dump_json()
            envelope = {"schema_version": 2, "payload": payload}
            self._files.prepare_parent(path)
            _atomic_write_text(path, json.dumps(envelope, ensure_ascii=False))

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
        paths = heapq.nlargest(max(1, min(limit, 100)), self._files.iter_paths(), key=lambda path: (path.stat().st_mtime_ns, str(path)))
        for path in paths:
            value = self._read(path)
            if path != self._path(value.checkpoint_id):
                raise ValueError("探索检查点文件名与身份不符")
            available, reason = True, None
            try:
                self.validate_asset(value)
            except ValueError as error:
                available, reason = False, str(error)
            records.append({"checkpoint_id": value.checkpoint_id, "request_id": value.request.request_id, "map_name": value.request.map_name,
                "created_at": value.created_at.isoformat(), "pipeline": value.pipeline, "available": available, "unavailable_reason": reason,
                "asset_manifest_sha256": value.asset_manifest_sha256})
        return records
