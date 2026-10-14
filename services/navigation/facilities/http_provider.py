"""平台设施协议第一版的HTTP接入；厂商桥接服务提供同一幂等查询语义。"""
from __future__ import annotations

import json
import os
from urllib.parse import quote, urlsplit
from typing import Optional

import httpx
from contracts.building_navigation import FacilityOperation, FacilitySnapshot, normalize_facility_provider_configuration
from providers.facilities import FacilityOutcomeUnknown, FacilityUnavailable


class HttpFacilityProvider:
    """同步有界请求，调用方负责任务截止和操作意图持久化。"""

    provider_version = "1.0"
    supported_connection_kinds = ("elevator",)

    def __init__(self, provider_id: str, base_url: str, *, authorization: Optional[str] = None, client: Optional[httpx.Client] = None) -> None:
        parsed = urlsplit(base_url)
        if parsed.scheme not in ("http", "https") or not parsed.hostname or parsed.username or parsed.password or parsed.query or parsed.fragment:
            raise ValueError("设施地址必须是无内嵌凭据的HTTP或HTTPS地址。")
        if not provider_id.strip() or provider_id != provider_id.strip():
            raise ValueError("设施提供器标识无效。")
        if authorization and ("\n" in authorization or "\r" in authorization):
            raise ValueError("设施认证头含无效字符。")
        self.provider_name = provider_id
        self.base_url = base_url.rstrip("/")
        self._client = client or httpx.Client(follow_redirects=False, trust_env=False)
        self._owns_client = client is None
        self._closed = False
        self._headers = {"Accept": "application/json"}
        if authorization:
            self._headers["Authorization"] = authorization

    def is_available(self) -> bool:
        """表示适配器已配置；设施实时可用性必须读取snapshot。"""
        return not self._closed

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self._owns_client:
            self._client.close()

    def _request(self, method: str, path: str, *, timeout_sec: float, payload=None, allow_not_found=False):
        if self._closed or not 0.0 < timeout_sec <= 30.0:
            raise FacilityUnavailable("设施连接已关闭或请求期限无效。")
        try:
            with self._client.stream(method, self.base_url + path, headers=self._headers, json=payload, timeout=timeout_sec) as response:
                if not (allow_not_found and response.status_code == 404):
                    response.raise_for_status()
                data = bytearray()
                for chunk in response.iter_bytes(chunk_size=16_384):
                    if len(data) + len(chunk) > 65_536:
                        raise ValueError("设施响应超过六十四千字节预算。")
                    data.extend(chunk)
                return json.loads(data)
        except (httpx.HTTPError, ValueError) as error:
            if method != "GET":
                raise FacilityOutcomeUnknown("设施操作回执未确认，须查询原操作标识。") from error
            raise FacilityUnavailable("设施状态查询失败。") from error

    @staticmethod
    def _facility_path(facility_id: str) -> str:
        return "/v1/facilities/" + quote(facility_id, safe="")

    def snapshot(self, facility_id: str, *, timeout_sec: float = 3.0) -> FacilitySnapshot:
        try:
            value = FacilitySnapshot.model_validate(self._request("GET", self._facility_path(facility_id) + "/state", timeout_sec=timeout_sec))
        except ValueError as error:
            raise FacilityUnavailable("设施状态格式无效。") from error
        if value.provider_id != self.provider_name or value.facility_id != facility_id:
            raise FacilityUnavailable("设施状态来源身份不符。")
        return value

    def _operation_path(self, facility_id: str, operation: FacilityOperation) -> str:
        return self._facility_path(facility_id) + "/sessions/" + quote(operation.session_id, safe="") + "/operations/" + quote(operation.operation_id, safe="")

    @staticmethod
    def _validate_receipt(value, expected: FacilityOperation) -> FacilityOperation:
        try:
            receipt = FacilityOperation.model_validate(value)
        except ValueError as error:
            raise FacilityOutcomeUnknown("设施操作回执格式无效。") from error
        identity = ("operation_id", "session_id", "action", "target_floor_id")
        if any(getattr(receipt, name) != getattr(expected, name) for name in identity):
            raise FacilityOutcomeUnknown("设施操作回执未对应当前会话和目标。")
        return receipt

    def submit_operation(self, facility_id: str, operation: FacilityOperation, *, timeout_sec: float = 3.0) -> FacilityOperation:
        value = self._request("PUT", self._operation_path(facility_id, operation), timeout_sec=timeout_sec,
                              payload=operation.model_dump(mode="json", exclude={"state", "message"}))
        return self._validate_receipt(value, operation)

    def query_operation(self, facility_id: str, operation: FacilityOperation, *, timeout_sec: float = 3.0) -> FacilityOperation:
        value = self._request("GET", self._operation_path(facility_id, operation), timeout_sec=timeout_sec, allow_not_found=True)
        return self._validate_receipt(value, operation)


def load_facility_providers(configuration: str) -> dict[str, HttpFacilityProvider]:
    """读取配置并创建受管连接；认证值只从明确的环境变量取得。"""
    payload = json.loads(normalize_facility_provider_configuration(configuration))
    if not isinstance(payload, dict) or len(payload) > 64:
        raise ValueError("设施提供器配置必须是至多六十四项的对象。")
    providers = {}
    try:
        for provider_id, config in payload.items():
            if not isinstance(config, dict) or set(config) - {"type", "base_url", "authorization_env"} or config.get("type", "http") != "http":
                raise ValueError("设施提供器配置字段或类型无效。")
            authorization_name = config.get("authorization_env")
            authorization = os.environ.get(authorization_name) if isinstance(authorization_name, str) and authorization_name else None
            if authorization_name and not authorization:
                raise ValueError("设施认证环境变量未配置。")
            providers[provider_id] = HttpFacilityProvider(provider_id, config["base_url"], authorization=authorization)
        return providers
    except Exception:
        for provider in providers.values():
            provider.close()
        raise
