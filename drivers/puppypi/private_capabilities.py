from __future__ import annotations

from contracts.capabilities import CapabilityRiskLevel
from drivers.common.private_capabilities import PrivateCapabilitySpec
from drivers.puppypi.capabilities import (
    PUPPYPI_ACTION_SPECS,
    build_run_action_group_input_schema,
)


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


def _action_result_schema() -> dict[str, object]:
    return _schema_object(
        {
            "code": {"type": "integer"},
            "result": {"type": "object"},
            "action": {"type": "string"},
        }
    )


def _wait_param_schema() -> dict[str, object]:
    return {
        "wait": {
            "type": "boolean",
            "default": False,
            "description": "是否等待动作组执行完成。",
        }
    }


def _build_posture_specs() -> tuple[PrivateCapabilitySpec, ...]:
    return tuple(
        PrivateCapabilitySpec(
            name=spec.name,
            display_name=spec.display_name,
            description=spec.description,
            risk_level=CapabilityRiskLevel.MEDIUM,
            input_schema=_schema_object(_wait_param_schema()),
            output_schema=_action_result_schema(),
            tags=("robot_private", "puppypi", "posture"),
        )
        for spec in PUPPYPI_ACTION_SPECS
    )


ROBOT_PRIVATE_CAPABILITY_SPECS: tuple[PrivateCapabilitySpec, ...] = (
    *_build_posture_specs(),
    PrivateCapabilitySpec(
        name="go_home",
        display_name="恢复初始姿态",
        description="调用 /puppy_control/go_home，让 PuppyPi 回到初始姿态。",
        risk_level=CapabilityRiskLevel.MEDIUM,
        input_schema=_schema_object(),
        output_schema=_action_result_schema(),
        tags=("robot_private", "puppypi", "posture"),
    ),
    PrivateCapabilitySpec(
        name="set_running",
        display_name="启停控制",
        description="调用 /puppy_control/set_running；true 启用，false 停机。",
        risk_level=CapabilityRiskLevel.MEDIUM,
        input_schema=_schema_object(
            {
                "enabled": {
                    "type": "boolean",
                    "description": "true 启用运行，false 停机。",
                }
            },
            required=("enabled",),
        ),
        output_schema=_action_result_schema(),
        tags=("robot_private", "puppypi", "control"),
    ),
    PrivateCapabilitySpec(
        name="set_self_balancing",
        display_name="自平衡",
        description="调用 /puppy_control/set_self_balancing 开启或关闭自平衡。",
        risk_level=CapabilityRiskLevel.MEDIUM,
        input_schema=_schema_object(
            {
                "enabled": {
                    "type": "boolean",
                    "description": "true 开启自平衡，false 关闭。",
                }
            },
            required=("enabled",),
        ),
        output_schema=_action_result_schema(),
        tags=("robot_private", "puppypi", "control"),
    ),
    PrivateCapabilitySpec(
        name="set_mark_time",
        display_name="原地踏步",
        description="调用 /puppy_control/set_mark_time；true 原地踏步，false 正常行走。",
        risk_level=CapabilityRiskLevel.LOW,
        input_schema=_schema_object(
            {
                "enabled": {
                    "type": "boolean",
                    "description": "true 原地踏步，false 正常行走。",
                }
            },
            required=("enabled",),
        ),
        output_schema=_action_result_schema(),
        tags=("robot_private", "puppypi", "control"),
    ),
    PrivateCapabilitySpec(
        name="run_action_group",
        display_name="运行动作组",
        description="调用 /puppy_control/runActionGroup 执行任意已安装动作组。",
        risk_level=CapabilityRiskLevel.MEDIUM,
        input_schema=build_run_action_group_input_schema(),
        output_schema=_action_result_schema(),
        tags=("robot_private", "puppypi", "posture"),
        action_name="run_action_group",
    ),
)
