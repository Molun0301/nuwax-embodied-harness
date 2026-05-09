from __future__ import annotations

from typing import Dict, List, Optional, Any

from adapters.base import AdapterBase, AdapterCategory, AdapterConfig
from adapters.helpers import build_pose, resolve_timestamp
from contracts.maps import OccupancyGrid


class SlamOccupancyAdapter(AdapterBase[Dict[str, Any], OccupancyGrid]):
    """SLAM 占据栅格适配器模板。"""

    @classmethod
    def build_default_config(
        cls,
        name: str = "external_slam_nav2_occupancy",
        *,
        source_ref: Optional[str] = "/map",
    ) -> AdapterConfig:
        """构造默认配置。"""

        return AdapterConfig(
            name=name,
            category=AdapterCategory.MAPPING,
            source_kind="external_slam_nav2",
            contract_type="OccupancyGrid",
            source_ref=source_ref,
            settings={"default_frame_id": "map", "unknown_value": -1, "clamp_values": True},
            metadata={"geometry_stack": "external_slam_nav2", "topic": source_ref},
        )

    def convert_payload(self, payload: Dict[str, Any]) -> OccupancyGrid:
        """把外部 SLAM 数据转换成占据栅格契约。"""

        timestamp = resolve_timestamp(payload)
        frame_id = str(payload.get("frame_id") or self.config.settings.get("default_frame_id") or "map")
        data = self._normalize_data(payload.get("data", []))
        return OccupancyGrid(
            timestamp=timestamp,
            map_id=str(payload.get("map_id", self.adapter_name)),
            frame_id=frame_id,
            width=int(payload["width"]),
            height=int(payload["height"]),
            resolution_m=float(payload["resolution_m"]),
            origin=build_pose(payload.get("origin") or {}, frame_id=frame_id, timestamp=timestamp),
            data=data,
        )

    def _normalize_data(self, raw_data: List[Any]) -> List[int]:
        unknown_value = int(self.config.settings.get("unknown_value", -1))
        clamp_values = bool(self.config.settings.get("clamp_values", True))
        normalized: List[int] = []

        for value in raw_data:
            if value is None:
                normalized.append(unknown_value)
                continue

            parsed = int(value)
            if clamp_values:
                if parsed < -1:
                    parsed = -1
                if parsed > 100:
                    parsed = 100
            normalized.append(parsed)

        return normalized
