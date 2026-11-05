"""跟随能力：统一感知给目标，原生跟随进程给速度，守卫执行。"""
from core.capability import Capability
from core.errors import GatewayError
from gateways.capabilities.common import rpc_errors, schema
from gateways.person_follow import PersonFollowBridgeError, PersonFollowBridgeUnavailable
from skills.follow_target import follow_target as run_follow_target


@rpc_errors
def follow_target(platform, ctx, args):
    try:
        return run_follow_target(platform, ctx, args)
    except PersonFollowBridgeUnavailable as error:
        raise GatewayError(str(error), error_code="person_follow_unavailable", http_status=503) from error
    except PersonFollowBridgeError as error:
        raise GatewayError(str(error), error_code="person_follow_failed", http_status=409) from error


CAPABILITIES = [
    Capability("follow_target", "在指定时段消费统一感知结果跟随目标；目标丢失超时结束。",
        schema({"query": {"type": "string", "minLength": 1}, "camera_id": {"type": "string"},
                "detector_backend": {"type": "string"},
                "duration_sec": {"type": "number", "exclusiveMinimum": 0, "default": 2.0},
                "interval_sec": {"type": "number", "exclusiveMinimum": 0, "maximum": 0.2, "default": 0.1},
                "lost_timeout_sec": {"type": "number", "exclusiveMinimum": 0, "default": 0.8},
                "timeout_sec": {"type": "number", "exclusiveMinimum": 0}}, ("query",)),
        "task", follow_target, needs=("geometry", "person_follow"), motion=True, timeout_sec=900.0),
]
