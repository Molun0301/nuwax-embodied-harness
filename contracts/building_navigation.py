"""楼栋目录与设施反馈；任务进度只由任务执行器保存。"""
from __future__ import annotations

from dataclasses import dataclass
import json
from typing import Literal, Optional

from pydantic import BaseModel, ConfigDict, field_validator


@dataclass(frozen=True)
class FloorPose:
    """所在楼层地图中的机体位姿，单位米与弧度。"""
    x: float
    y: float
    yaw_rad: float = 0.0


@dataclass(frozen=True)
class FloorRecord:
    floor_id: str
    display_name: str
    map_name: str
    enabled: bool = True


@dataclass(frozen=True)
class FloorConnection:
    """一个有向设施连接；两端位姿分别属于对应楼层地图。"""
    connection_id: str
    from_floor_id: str
    to_floor_id: str
    provider_id: str
    facility_id: str
    waiting_pose: FloorPose
    boarding_pose: FloorPose
    arrival_pose: FloorPose
    exit_pose: FloorPose
    docking_calibration_ref: str
    expected_transfer_sec: float
    kind: str = "elevator"
    expected_localization_sec: float = 20.0
    enabled: bool = True


@dataclass(frozen=True)
class BuildingTopology:
    building_id: str
    display_name: str
    floors: tuple[FloorRecord, ...]
    connections: tuple[FloorConnection, ...] = ()


def topology_from_dict(value: dict) -> BuildingTopology:
    """参数表或文件读取处已校验结构，此处只转换数据类型。"""
    connections = []
    for item in value.get("connections", ()):
        fields = dict(item)
        for name in ("waiting_pose", "boarding_pose", "arrival_pose", "exit_pose"):
            fields[name] = FloorPose(**fields[name])
        connections.append(FloorConnection(**fields))
    return BuildingTopology(value["building_id"], value["display_name"],
                            tuple(FloorRecord(**item) for item in value["floors"]), tuple(connections))


def _object(properties, required=()):
    return {"type": "object", "properties": properties, "required": list(required), "additionalProperties": False}


_LABEL = {"type": "string", "minLength": 1, "maxLength": 128}
_POSE_SCHEMA = _object({"x": {"type": "number"}, "y": {"type": "number"},
                        "yaw_rad": {"type": "number"}}, ("x", "y"))
_FLOOR_SCHEMA = _object({"floor_id": _LABEL, "display_name": _LABEL,
    "map_name": {"type": "string", "minLength": 1, "maxLength": 64,
                 "pattern": r"^(?!\.)(?!.*[/\x00-\x1f\x7f-\x9f]).+$"}, "enabled": {"type": "boolean"}},
    ("floor_id", "display_name", "map_name"))
_CONNECTION_SCHEMA = _object({
    "connection_id": _LABEL, "from_floor_id": _LABEL, "to_floor_id": _LABEL,
    "provider_id": _LABEL, "facility_id": _LABEL,
    "kind": {"type": "string", "enum": ["elevator", "stairs"]},
    "waiting_pose": _POSE_SCHEMA, "boarding_pose": _POSE_SCHEMA,
    "arrival_pose": _POSE_SCHEMA, "exit_pose": _POSE_SCHEMA,
    "docking_calibration_ref": {"type": "string", "minLength": 1, "maxLength": 256},
    "expected_transfer_sec": {"type": "number", "exclusiveMinimum": 0, "maximum": 3600},
    "expected_localization_sec": {"type": "number", "exclusiveMinimum": 0, "maximum": 600},
    "enabled": {"type": "boolean"}},
    ("connection_id", "from_floor_id", "to_floor_id", "provider_id", "facility_id", "waiting_pose",
     "boarding_pose", "arrival_pose", "exit_pose", "docking_calibration_ref", "expected_transfer_sec"))
TOPOLOGY_SCHEMA = _object({"building_id": _LABEL, "display_name": _LABEL,
    "floors": {"type": "array", "items": _FLOOR_SCHEMA, "minItems": 1, "maxItems": 256},
    "connections": {"type": "array", "items": _CONNECTION_SCHEMA, "maxItems": 2048}},
    ("building_id", "display_name", "floors"))


@dataclass(frozen=True)
class FacilitySnapshot:
    provider_id: str
    facility_id: str
    available: bool
    floor_id: Optional[str] = None
    door: str = "unknown"
    moving: bool = False
    occupied_by_other: bool = False
    docking_calibration_ref: Optional[str] = None
    docking_verified: bool = False
    fault: Optional[str] = None


class FacilityProviderConfig(BaseModel):
    """设施桥连接配置；authorization 保存请求所用的认证值。"""
    model_config = ConfigDict(extra="forbid")
    type: Literal["http"] = "http"
    base_url: str
    authorization: Optional[str] = None

    @field_validator("base_url")
    @classmethod
    def _endpoint(cls, value: str) -> str:
        from urllib.parse import urlsplit
        parsed = urlsplit(value)
        if parsed.scheme not in ("http", "https") or not parsed.hostname or parsed.username or parsed.password or parsed.query or parsed.fragment:
            raise ValueError("设施地址必须是无内嵌凭据的HTTP或HTTPS地址。")
        return value.rstrip("/")


def normalize_facility_provider_configuration(value: str) -> str:
    """配置在读取处校验一次，不连接设施。"""
    payload = json.loads(value or "{}")
    if not isinstance(payload, dict) or len(payload) > 64:
        raise ValueError("设施提供器配置必须是至多六十四项的对象。")
    result = {}
    for provider_id, config in payload.items():
        if not isinstance(provider_id, str) or not provider_id or provider_id.strip() != provider_id or len(provider_id) > 128:
            raise ValueError("设施提供器标识无效。")
        result[provider_id] = FacilityProviderConfig.model_validate(config).model_dump(exclude_none=True)
    return json.dumps(result, ensure_ascii=False, separators=(",", ":"))
