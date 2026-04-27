from __future__ import annotations

from dataclasses import dataclass
import string
import uuid
from typing import Optional


REQUEST_ID_HEADER = "X-Request-ID"
_MAX_REQUEST_ID_LENGTH = 128
_ALLOWED_REQUEST_ID_CHARS = frozenset(string.ascii_letters + string.digits + "._-:")


@dataclass(frozen=True)
class RequestIdContext:
    request_id: str
    request_id_generated: bool
    request_id_replaced: bool
    request_id_invalid_reason: Optional[str] = None


def normalize_request_id(raw_value: Optional[str]) -> RequestIdContext:
    if raw_value is None:
        return RequestIdContext(
            request_id=uuid.uuid4().hex,
            request_id_generated=True,
            request_id_replaced=False,
        )

    value = raw_value.strip()
    if not value:
        return RequestIdContext(
            request_id=uuid.uuid4().hex,
            request_id_generated=True,
            request_id_replaced=True,
            request_id_invalid_reason="empty",
        )
    if len(value) > _MAX_REQUEST_ID_LENGTH:
        return RequestIdContext(
            request_id=uuid.uuid4().hex,
            request_id_generated=True,
            request_id_replaced=True,
            request_id_invalid_reason="too_long",
        )
    if any(char not in _ALLOWED_REQUEST_ID_CHARS for char in value):
        return RequestIdContext(
            request_id=uuid.uuid4().hex,
            request_id_generated=True,
            request_id_replaced=True,
            request_id_invalid_reason="invalid_char",
        )
    return RequestIdContext(
        request_id=value,
        request_id_generated=False,
        request_id_replaced=False,
    )


def categorize_gateway_error(error_code: str, http_status: int) -> str:
    normalized = str(error_code or "").lower()
    if normalized in {"authentication_required", "unauthorized", "invalid_token"} or http_status == 401:
        return "auth"
    if normalized in {"access_denied", "forbidden", "capability_not_allowed"} or http_status == 403:
        return "authorization"
    if normalized in {"invalid_params", "validation_error", "not_found", "artifact_not_found"} or http_status == 422:
        return "validation"
    if normalized in {"capability_not_registered", "capability_not_supported", "resource_conflict", "execution_failed"}:
        return "capability"
    if normalized == "timeout":
        return "timeout"
    if 400 <= http_status < 500:
        return "validation"
    if http_status >= 500:
        return "internal"
    return "capability"


def error_category_from_status(status_code: int) -> str:
    if status_code < 400:
        return "-"
    if status_code == 401:
        return "auth"
    if status_code == 403:
        return "authorization"
    if status_code in {400, 404, 405, 422}:
        return "validation"
    return "internal"
