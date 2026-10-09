"""Public relocalization IDs: explicit new business identity or retained recovery ID."""

from core.errors import GatewayError


def relocalization_request_identity(payload):
    """Return the unchanged ID and whether this is a recovery-only request.

    Native request_id is already the durable operation key; there is no second
    generated ID or business-to-operation mapping when a business ID is supplied.
    """
    business = payload.get("business_request_id")
    retained = payload.get("request_id")
    if business is not None and retained is not None:
        raise GatewayError("business_request_id 与旧 request_id 不能同时提供。",
                           error_code="invalid_params", http_status=422, jsonrpc_code=-32602)
    value = business if business is not None else retained
    if value is not None and (not isinstance(value, str) or not 1 <= len(value) <= 128
            or any(not (c.isascii() and (c.isalnum() or c in "._:-")) for c in value)):
        raise GatewayError("重定位编号必须为 1–128 个 ASCII 字母、数字或 ._:-，原值不会被转换。",
                           error_code="invalid_params", http_status=422, jsonrpc_code=-32602)
    return value, retained is not None
