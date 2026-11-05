"""能力参数表和执行错误转换。"""
from functools import wraps

from core.errors import GatewayError
from services.geometry.client import GeometryError
from services.navigation.motion_client import MotionError


def schema(properties=None, required=()):
    return {"type": "object", "properties": properties or {}, "required": list(required),
            "additionalProperties": False}


def rpc_errors(handler):
    @wraps(handler)
    def execute(*args, **kwargs):
        try:
            return handler(*args, **kwargs)
        except GeometryError as error:
            status = {"NOT_FOUND": 404, "INVALID_ARGUMENT": 400, "UNAVAILABLE": 503,
                      "DEADLINE_EXCEEDED": 503, "INTERNAL": 500}.get(error.code, 409)
            raise GatewayError(error.message, error_code="geometry_" + error.code.lower(),
                               http_status=status) from error
        except MotionError as error:
            from gateways.invoke import motion_error
            raise motion_error(error) from error
    return execute
