"""地名查找与已保存地理参考的坐标换算。"""
from __future__ import annotations

import math

import httpx

from core.config import value
from core.errors import GatewayError


class PlaceLookup:
    def __init__(self):
        self._url = value('map_cloud.nominatim_url')
        self._client = httpx.Client(headers={"User-Agent": "NuwaxRobotBridge/0.9"})

    def close(self):
        self._client.close()

    def search(self, query_text: str, *, timeout_sec=None) -> dict:
        try:
            response = self._client.get(self._url, params={"q": query_text, "format": "jsonv2", "limit": 1},
                                        timeout=timeout_sec)
            response.raise_for_status()
            values = response.json()
        except (httpx.HTTPError, ValueError) as error:
            raise GatewayError(str(error), error_code="place_lookup_failed", http_status=503) from error
        if not values:
            raise GatewayError(f"未找到地名：{query_text}", error_code="place_not_found", http_status=404)
        item = values[0]
        return {**item, "lat": float(item["lat"]), "lon": float(item["lon"])}


def map_coordinates(latitude: float, longitude: float, reference: dict) -> tuple[float, float, float]:
    """沿地图合同的 EPSG:3857（网络墨卡托坐标系）锚点换算。"""
    easting = 6_378_137.0 * math.radians(longitude)
    northing = 6_378_137.0 * math.log(math.tan(math.pi / 4 + math.radians(latitude) / 2))
    dx, dy = easting - reference["anchor_easting_m"], northing - reference["anchor_northing_m"]
    yaw = reference["map_from_projected_yaw_rad"]
    c, s = math.cos(yaw), math.sin(yaw)
    return (reference["map_origin_x_m"] + c * dx - s * dy,
            reference["map_origin_y_m"] + s * dx + c * dy, reference["map_plane_z_m"])
