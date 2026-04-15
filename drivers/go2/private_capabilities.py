from __future__ import annotations

from contracts.capabilities import CapabilityRiskLevel
from drivers.common.private_capabilities import PrivateCapabilitySpec
from drivers.go2.capabilities import build_go2_sport_command_description, build_go2_sport_command_input_schema


def _schema_object(
    properties: dict[str, dict[str, object]] | None = None,
    *,
    required: tuple[str, ...] = (),
) -> dict[str, object]:
    return {
        "type": "object",
        "properties": dict(properties or {}),
        "required": list(required),
        "additionalProperties": False,
    }


ROBOT_PRIVATE_CAPABILITY_SPECS: tuple[PrivateCapabilitySpec, ...] = (
    PrivateCapabilitySpec(
        name="execute_sport_command",
        display_name="执行机身动作",
        description=build_go2_sport_command_description(),
        risk_level=CapabilityRiskLevel.MEDIUM,
        input_schema=build_go2_sport_command_input_schema(),
        output_schema=_schema_object({"code": {"type": "integer"}, "result": {"type": "object"}}),
        tags=("robot_private", "go2"),
    ),
    PrivateCapabilitySpec(
        name="set_body_pose",
        display_name="设置机身姿态",
        description="通过机身欧拉角控制机器人姿态。",
        risk_level=CapabilityRiskLevel.MEDIUM,
        input_schema=_schema_object(
            {
                "roll": {"type": "number", "default": 0.0},
                "pitch": {"type": "number", "default": 0.0},
                "yaw": {"type": "number", "default": 0.0},
            }
        ),
        output_schema=_schema_object({"code": {"type": "integer"}, "result": {"type": "object"}}),
        tags=("robot_private", "go2"),
    ),
    PrivateCapabilitySpec(
        name="set_speed_level",
        display_name="设置速度档位",
        description="设置机器人运动速度档位。",
        risk_level=CapabilityRiskLevel.MEDIUM,
        input_schema=_schema_object({"level": {"type": "integer", "minimum": 1, "maximum": 3}}, required=("level",)),
        output_schema=_schema_object({"code": {"type": "integer"}, "result": {"type": "object"}}),
        tags=("robot_private", "go2"),
    ),
    PrivateCapabilitySpec(
        name="get_volume",
        display_name="获取音量",
        description="读取当前机器人音量与软件播报音量状态。",
        risk_level=CapabilityRiskLevel.LOW,
        output_schema=_schema_object({"volume_state": {"type": "object"}}),
        tags=("robot_private", "go2"),
    ),
    PrivateCapabilitySpec(
        name="set_volume",
        display_name="设置音量",
        description="设置机器人音量与软件播报音量。",
        risk_level=CapabilityRiskLevel.MEDIUM,
        input_schema=_schema_object({"volume": {"type": "number", "minimum": 0.0, "maximum": 1.0}}, required=("volume",)),
        output_schema=_schema_object({"volume_state": {"type": "object"}}),
        tags=("robot_private", "go2"),
    ),
    PrivateCapabilitySpec(
        name="switch_control_mode",
        display_name="切换控制模式",
        description="切换机器人高层/低层控制模式。",
        risk_level=CapabilityRiskLevel.ADMIN,
        input_schema=_schema_object({"mode": {"type": "string", "enum": ["high", "low"]}}, required=("mode",)),
        output_schema=_schema_object({"code": {"type": "integer"}, "result": {"type": "object"}}),
        tags=("robot_private", "go2"),
    ),
    PrivateCapabilitySpec(
        name="set_obstacle_avoidance",
        display_name="设置实时避障",
        description="开启或关闭 Go2 官方实时避障服务。",
        risk_level=CapabilityRiskLevel.LOW,
        input_schema=_schema_object({"enabled": {"type": "boolean"}}, required=("enabled",)),
        output_schema=_schema_object(
            {
                "obstacle_avoidance_enabled": {"type": "boolean"},
                "backend_ready": {"type": "boolean"},
                "switch_enabled": {"type": "boolean"},
                "remote_command_enabled": {"type": "boolean"},
                "control_path": {"type": "string"},
            }
        ),
        tags=("robot_private", "go2"),
    ),
)
