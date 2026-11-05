"""MCP、HTTP 与控制台共用的唯一调用分发。"""
from dataclasses import asdict
from typing import Any, Mapping

from jsonschema import Draft202012Validator
from jsonschema.exceptions import ValidationError

from core import components
from core.errors import GatewayError
from services.geometry.maps import MapNameError, validate_map_name
from services.navigation.motion_client import MotionError


def motion_error(error: MotionError) -> GatewayError:
    return GatewayError(error.message, error_code="motion_" + error.code.value,
                        http_status=503 if error.code.value in {"unavailable", "not_ready"} else 409)


def invoke(platform, name: str, args: Mapping[str, Any], caller: str) -> dict:
    cap = platform.capabilities.get(name)
    if cap is None:
        raise GatewayError(f"未知能力：{name}", error_code="unknown_capability", http_status=404)
    if cap.robots is not None and platform.robot.manifest.robot_name not in cap.robots:
        raise GatewayError("当前机器人不支持该能力。", error_code="unsupported", http_status=409)
    try:
        Draft202012Validator(cap.input_schema).validate(args)
        arguments = dict(args)
        for key in ("map_name", "new_map_name"):
            if key in arguments and key in cap.input_schema.get("properties", {}):
                arguments[key] = validate_map_name(arguments[key])
    except (ValidationError, MapNameError) as error:
        raise GatewayError(str(error), error_code="invalid_params", http_status=400) from error
    if cap.kind == "task":
        return {"task": asdict(platform.tasks.submit(cap, arguments, caller))}
    if cap.motion:
        platform.tasks.take_over_motion()
    platform.ensure(cap.needs, cap.timeout_sec)
    if cap.motion:
        arguments["motion_generation"] = platform.tasks._begin_motion()
    return cap.handler(platform, arguments)


def stop_all_motion(platform, reason: str) -> dict:
    result = {}
    try:
        platform.motion.stop(reason)
        result["guard"] = "ok"
    except MotionError as error:
        result["guard"] = "守卫未运行" if error.code.value == "unavailable" else error.message
    except Exception as error:
        result["guard"] = str(error)
    try:
        platform.tasks.take_over_motion(reason)
        result["task"] = "ok"
    except Exception as error:
        result["task"] = str(error)
    failures = []
    try:
        if components.status().get("geometry", {}).get("state") == "running":
            for operation in (platform.geometry.cancel_goal, platform.geometry.stop_exploration):
                try:
                    operation()
                except Exception as error:
                    failures.append(str(error))
    except Exception as error:
        failures.append(str(error))
    result["geometry"] = "；".join(failures) if failures else "ok"
    return result
