"""Adapt relocalization identity validation to gateway error semantics."""

from contracts.relocalization_request_id import (
    relocalization_request_identity as _contract_identity,
)
from core.errors import GatewayError


def relocalization_request_identity(payload):
    """Preserve the contract identity and expose its public protocol error."""
    try:
        return _contract_identity(payload)
    except ValueError as error:
        raise GatewayError(
            str(error), error_code="invalid_params", http_status=422, jsonrpc_code=-32602
        ) from error
