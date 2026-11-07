"""设施桥同步HTTP动作；失败直接返回，不保存或查询操作回执。"""
from __future__ import annotations

import json
from typing import Optional
from urllib.parse import quote

import httpx

from contracts.building_navigation import FacilitySnapshot, normalize_facility_provider_configuration
from providers.facilities import FacilityUnavailable


class HttpFacilityProvider:
    provider_version = "2.0"
    supported_connection_kinds = ("elevator",)

    def __init__(self, provider_id: str, base_url: str, *, authorization: Optional[str] = None,
                 client: Optional[httpx.Client] = None) -> None:
        self.provider_name = provider_id
        self.base_url = base_url.rstrip("/")
        self._client = client or httpx.Client(follow_redirects=False, trust_env=False)
        self._owns_client = client is None
        self._closed = False
        self._headers = {"Accept": "application/json"}
        if authorization:
            self._headers["Authorization"] = authorization

    def is_available(self) -> bool:
        return not self._closed

    def close(self) -> None:
        self._closed = True
        if self._owns_client:
            self._client.close()

    def _request(self, method, path, timeout_sec, payload=None):
        try:
            response = self._client.request(method, self.base_url + path, headers=self._headers,
                                            json=payload, timeout=timeout_sec)
            response.raise_for_status()
            return response.json()
        except (httpx.HTTPError, ValueError) as error:
            raise FacilityUnavailable(f"设施请求失败：{error}") from error

    @staticmethod
    def _path(facility_id: str) -> str:
        return "/v1/facilities/" + quote(facility_id, safe="")

    def snapshot(self, facility_id: str, *, timeout_sec: Optional[float] = None) -> FacilitySnapshot:
        return FacilitySnapshot(**self._request("GET", self._path(facility_id) + "/state", timeout_sec))

    def operate(self, facility_id: str, action: str, target_floor_id: Optional[str] = None,
                timeout_sec: Optional[float] = None) -> dict:
        result = self._request("POST", self._path(facility_id) + "/actions", timeout_sec,
                               {"action": action, "target_floor_id": target_floor_id})
        if not result["ok"]:
            raise FacilityUnavailable(result.get("message") or "设施动作失败。")
        return result


def load_facility_providers(configuration: str) -> dict[str, HttpFacilityProvider]:
    """在配置读取处解析一次；连接只在首次动作或状态请求时建立。"""
    configs = json.loads(normalize_facility_provider_configuration(configuration))
    return {name: HttpFacilityProvider(name, config["base_url"],
            authorization=config.get("authorization"))
            for name, config in configs.items()}
