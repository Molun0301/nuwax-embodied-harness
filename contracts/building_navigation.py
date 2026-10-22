"""建筑楼层、受管设施连接及跨层任务的统一契约。"""
from __future__ import annotations

from datetime import datetime
from typing import Dict, Literal, Optional, Tuple

from pydantic import ConfigDict, Field, field_validator, model_validator
from contracts.base import ContractModel
from contracts.map_identity import AssetIdentityV2


class FloorPose(ContractModel):
    """已登记层图中的平面机体位姿，单位米和弧度。"""

    model_config = ConfigDict(frozen=True, allow_inf_nan=False)
    x: float
    y: float
    yaw_rad: float = 0.0


class FloorRecord(ContractModel):
    """建筑内唯一楼层；资产身份为空表示尚未完成该层建图。"""

    model_config = ConfigDict(frozen=True)
    floor_id: str = Field(min_length=1, max_length=128)
    display_name: str = Field(min_length=1, max_length=128)
    map_name: str = Field(min_length=1, max_length=255)
    asset_identity: Optional[AssetIdentityV2] = None
    enabled: bool = True

    @field_validator("floor_id", "display_name", "map_name")
    @classmethod
    def _canonical_label(cls, value: str) -> str:
        if value.strip() != value or any(ord(c) < 32 for c in value):
            raise ValueError("楼层标识和名称不得有首尾空白或控制字符。")
        return value


class FloorConnection(ContractModel):
    """单个有向设施连接，两端位姿分别属于各自楼层地图。"""

    model_config = ConfigDict(frozen=True, allow_inf_nan=False)
    connection_id: str = Field(min_length=1, max_length=128)
    from_floor_id: str = Field(min_length=1, max_length=128)
    to_floor_id: str = Field(min_length=1, max_length=128)
    kind: Literal["elevator", "stairs"] = "elevator"
    provider_id: str = Field(min_length=1, max_length=128)
    facility_id: str = Field(min_length=1, max_length=128)
    waiting_pose: FloorPose
    boarding_pose: FloorPose
    arrival_pose: FloorPose
    exit_pose: FloorPose
    docking_calibration_ref: str = Field(min_length=1, max_length=256)
    expected_transfer_sec: float = Field(gt=0.0, le=3600.0)
    expected_localization_sec: float = Field(default=20.0, gt=0.0, le=600.0)
    enabled: bool = True
    allow_docking_without_localization: bool = False
    docking_max_distance_m: float = Field(default=0.0, ge=0.0, le=5.0)

    @model_validator(mode="after")
    def _valid_connection(self) -> "FloorConnection":
        if self.from_floor_id == self.to_floor_id:
            raise ValueError("跨层连接的起止楼层必须不同。")
        if self.allow_docking_without_localization and self.docking_max_distance_m <= 0.0:
            raise ValueError("无定位对接必须声明经标定的有限移动范围。")
        return self


class BuildingTopology(ContractModel):
    """楼层与连接的一个完整目录修订。"""

    model_config = ConfigDict(frozen=True)
    schema_version: Literal[2] = 2
    building_id: str = Field(min_length=1, max_length=128)
    display_name: str = Field(min_length=1, max_length=128)
    revision: int = Field(ge=1)
    floors: Tuple[FloorRecord, ...] = Field(min_length=1, max_length=256)
    connections: Tuple[FloorConnection, ...] = Field(default=(), max_length=2048)

    @model_validator(mode="after")
    def _references(self) -> "BuildingTopology":
        floors = {floor.floor_id for floor in self.floors}
        if len(floors) != len(self.floors) or len({f.map_name for f in self.floors}) != len(self.floors):
            raise ValueError("楼层标识和地图绑定在同一建筑中必须唯一。")
        if len({c.connection_id for c in self.connections}) != len(self.connections):
            raise ValueError("设施连接标识重复。")
        if any(c.from_floor_id not in floors or c.to_floor_id not in floors for c in self.connections):
            raise ValueError("设施连接引用未登记楼层。")
        return self


class FacilitySnapshot(ContractModel):
    """设施实际反馈，必须与本次预约会话和当前单调序列关联。"""

    model_config = ConfigDict(frozen=True, allow_inf_nan=False)
    provider_id: str
    facility_id: str
    session_id: Optional[str] = None
    sequence: int = Field(ge=1)
    observed_at: datetime
    available: bool
    floor_id: Optional[str] = None
    door: Literal["open", "closed", "unknown"] = "unknown"
    moving: bool = False
    occupied_by_other: bool = False
    docking_calibration_ref: Optional[str] = None
    docking_verified: bool = False
    fault: Optional[str] = None

    @field_validator("observed_at")
    @classmethod
    def _timezone_required(cls, value: datetime) -> datetime:
        if value.tzinfo is None or value.utcoffset() is None:
            raise ValueError("设施观察时间必须带时区。")
        return value


class FacilityOperation(ContractModel):
    """具幂等标识的设施操作及可查询终态。"""

    model_config = ConfigDict(frozen=True)
    operation_id: str
    session_id: str
    action: Literal["reserve", "call", "hold_door", "transport", "release"]
    target_floor_id: Optional[str] = None
    state: Literal["accepted", "running", "succeeded", "failed", "not_found", "unknown"]
    message: str = ""


class FloorNavigationRequest(ContractModel):
    """受理时已冻结的目标，执行中不再用“当前楼层”重新解释。"""

    model_config = ConfigDict(frozen=True, allow_inf_nan=False)
    request_id: str = Field(min_length=1, max_length=256)
    building_id: str
    topology_revision: int = Field(ge=1)
    source_floor_id: str
    target_floor_id: str
    target_map_name: str
    target_asset_identity: Optional[AssetIdentityV2] = None
    create_target_map: bool = False
    target_pose: FloorPose
    route: Tuple[str, ...] = ()
    timeout_sec: float = Field(default=600.0, gt=0.0, le=7200.0)

    @model_validator(mode="after")
    def _target_binding(self) -> "FloorNavigationRequest":
        if self.create_target_map != (self.target_asset_identity is None):
            raise ValueError("普通楼层导航必须绑定完整图；新层探索转移必须明确创建新图。")
        return self


class FloorExecutionState(ContractModel):
    """跨层流程的持久状态；不确定操作必须经查询收敛。"""

    schema_version: Literal[2] = 2
    request: FloorNavigationRequest
    stage: str = "accepted"
    owner_task_id: Optional[str] = None
    last_pose: Optional[FloorPose] = None
    last_pose_floor_id: Optional[str] = None
    last_pose_asset_identity: Optional[AssetIdentityV2] = None
    current_floor_id: Optional[str]
    attempt: int = Field(default=0, ge=0)
    created_map_asset_identity: Optional[AssetIdentityV2] = None
    last_zero_proof: dict = Field(default_factory=dict)
    facility_state: Optional[FacilitySnapshot] = None
    connection_index: int = Field(default=0, ge=0)
    facility_session_id: Optional[str] = None
    pending_operation: Optional[FacilityOperation] = None
    last_facility_sequence: int = Field(default=0, ge=0)
    navigation_goal_id: Optional[str] = None
    last_message: str = ""
    terminal: bool = False
    needs_reconciliation: bool = False


class FacilityProviderConfig(ContractModel):
    """标准设施桥接连接配置；认证内容不写入普通配置文件。"""

    type: Literal["http"] = "http"
    base_url: str
    authorization_env: Optional[str] = Field(default=None, pattern=r"^[A-Za-z_][A-Za-z0-9_]*$")

    @field_validator("base_url")
    @classmethod
    def _endpoint(cls, value: str) -> str:
        from urllib.parse import urlsplit
        parsed = urlsplit(value)
        if parsed.scheme not in ("http", "https") or not parsed.hostname or parsed.username or parsed.password or parsed.query or parsed.fragment:
            raise ValueError("设施地址必须是无内嵌凭据的HTTP或HTTPS地址。")
        return value.rstrip("/")


def normalize_facility_provider_configuration(value: str) -> str:
    """只校验配置，不创建连接或读取认证环境变量。"""
    import json
    payload = json.loads(value or "{}")
    if not isinstance(payload, dict) or len(payload) > 64:
        raise ValueError("设施提供器配置必须是至多六十四项的对象。")
    result = {}
    for provider_id, config in payload.items():
        if not isinstance(provider_id, str) or not provider_id or provider_id.strip() != provider_id or len(provider_id) > 128:
            raise ValueError("设施提供器标识无效。")
        result[provider_id] = FacilityProviderConfig.model_validate(config).model_dump(exclude_none=True)
    return json.dumps(result, ensure_ascii=False, separators=(",", ":"))


class BuildingExplorationRequest(ContractModel):
    """受理时冻结的逐层探索顺序与地图名称。"""

    model_config = ConfigDict(frozen=True, allow_inf_nan=False)
    request_id: str
    building_id: str
    topology_revision: int = Field(ge=1)
    floors: Tuple[FloorRecord, ...] = Field(min_length=1, max_length=256)
    stop_conditions: dict = Field(default_factory=dict)
    per_floor_timeout_sec: float = Field(default=300.0, gt=0.0, le=3600.0)
    timeout_sec: float = Field(default=7200.0, gt=0.0, le=43200.0)

    @model_validator(mode="after")
    def _unique_floors(self) -> "BuildingExplorationRequest":
        if len({floor.floor_id for floor in self.floors}) != len(self.floors):
            raise ValueError("逐层探索不能重复登记同一楼层。")
        return self


class BuildingExplorationState(ContractModel):
    """只持久化编排进度，恢复时重新取得当前运动准入。"""

    schema_version: Literal[2] = 2
    request: BuildingExplorationRequest
    topology_revision: int
    next_floor_index: int = 0
    completed_floors: Dict[str, dict] = Field(default_factory=dict)
    stage: str = "accepted"
    attempt: int = 0
    owner_task_id: Optional[str] = None
    child_task_id: Optional[str] = None
    child_request_id: Optional[str] = None
    child_kind: Optional[str] = None
    child_succeeded: bool = False
    terminal: bool = False
    needs_reconciliation: bool = False
    message: str = ""
