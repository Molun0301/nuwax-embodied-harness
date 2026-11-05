"""设备操作使用公共任务执行器，不占用底盘运动代次。"""
from core.capability import Capability
from core.errors import GatewayError
from services.manipulation.http_backend import DeviceError


def execute_manipulation_task(platform, ctx, args):
    try:
        service = platform.manipulation
        binding = service.bind()
        return service.run(ctx, binding, {"instruction": args["instruction"].strip(),
            "timeout_sec": args.get("timeout_sec", 120), "max_steps": args.get("max_steps", 1440)})
    except DeviceError as error:
        raise GatewayError(str(error), error_code=error.code,
            http_status=503 if error.code == "manipulation_unavailable" else 409) from error


CAPABILITIES = [Capability("execute_manipulation_task", "把操作指令交给配置的设备并跟踪真实物理结果；取消转发设备取消，不等待停稳证明。",
    {"type": "object", "properties": {
        "instruction": {"type": "string", "minLength": 1, "maxLength": 2000, "pattern": r"\S"},
        "timeout_sec": {"type": "number", "minimum": 1, "maximum": 1800, "default": 120},
        "max_steps": {"type": "integer", "minimum": 1, "maximum": 10000, "default": 1440}},
     "required": ["instruction"], "additionalProperties": False}, "task", execute_manipulation_task,
     motion=False, timeout_sec=1800)]
