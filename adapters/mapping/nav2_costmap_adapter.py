from __future__ import annotations

from typing import Dict, Optional, Any

from adapters.base import AdapterBase, AdapterCategory, AdapterConfig
from adapters.helpers import build_pose, resolve_timestamp
from contracts.maps import CostMap


class Nav2CostMapAdapter(AdapterBase[Dict[str, Any], CostMap]):
    """Nav2 代价地图适配器模板。"""

    @classmethod
    def build_default_config(
        cls,
        name: str = "external_slam_nav2_costmap",
        *,
        source_ref: Optional[str] = "/global_costmap/costmap",
    ) -> AdapterConfig:
        """构造默认配置。"""

        return AdapterConfig(
            name=name,
            category=AdapterCategory.MAPPING,
            source_kind="external_slam_nav2",
            contract_type="CostMap",
            source_ref=source_ref,
            settings={"default_frame_id": "map", "cost_scale": 1.0},
            metadata={"geometry_stack": "external_slam_nav2", "topic": source_ref},
        )

    def convert_payload(self, payload: Dict[str, Any]) -> CostMap:
        """把外部 Nav2 数据转换成代价地图契约。"""

        timestamp = resolve_timestamp(payload)
        frame_id = str(payload.get("frame_id") or self.config.settings.get("default_frame_id") or "map")
        width = int(payload["width"])
        height = int(payload["height"])
        scale = float(payload.get("cost_scale", self.config.settings.get("cost_scale", 1.0)))
        raw_data = payload.get("data", [])
        data = [float(value) * scale for value in raw_data]
        origin_payload = payload.get("origin") or {}
        return CostMap(
            timestamp=timestamp,
            map_id=str(payload.get("map_id", self.adapter_name)),
            frame_id=frame_id,
            width=width,
            height=height,
            resolution_m=float(payload["resolution_m"]),
            origin=build_pose(origin_payload, frame_id=frame_id, timestamp=timestamp),
            data=data,
        )
