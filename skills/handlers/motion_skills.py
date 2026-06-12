"""运动域 handler（P2-4 批 R1）。

从 gateways/runtime.py 下放的运动控制技能。handler 为模块级纯函数，首参 ctx:
SkillContext，函数体内原 self.xxx 重写为 ctx.xxx，原 self._require_*/_float/_int 等
校验助手改用 skills.common.params 的纯函数。行为与抛错条件与迁移前完全一致。

本批下放（分析文档 §3.A，5 个全 A 类）：
    handle_execute_sport_command、handle_set_body_pose、handle_set_speed_level、
    handle_switch_control_mode、handle_set_obstacle_avoidance。
均同步非任务化，签名 (ctx, payload, *, requested_by)，仅依赖 ctx.robot
（execute_action/switch_mode/data_plane）+ 参数校验，无网关可变状态写入、无私有方法。

留网关（C 类，导航编排）：navigate_*/find_target/explore_*/inspect/follow（任务化生命周期）。
"""

from __future__ import annotations

from typing import Any, Dict, Optional, Tuple

from core.errors import GatewayError
from providers.data_plane import ObstacleAvoidanceControlDataPlane
from skills.common import params
from skills.common.context import SkillContext


def handle_execute_sport_command(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    """执行已公开的高层机身动作命令。

    迁移自 GatewayRuntime._handle_execute_sport_command；self.robot→ctx.robot，
    self._require_str→params.require_str。支持动作集从机器人能力矩阵 metadata 泛化读取。
    """

    del requested_by
    if not hasattr(ctx.robot, "execute_action"):
        raise GatewayError("当前机器人入口未实现 execute_action。")
    action = params.require_str(payload, "action")
    # 平台侧零 import driver：从机器人能力矩阵 metadata 泛化读取私有动作集，
    # 新机器人在自己的 capability_matrix.metadata 声明 sport_command_names 即可。
    metadata = getattr(ctx.robot.manifest.capability_matrix, "metadata", None) or {}
    supported_actions: Tuple[str, ...] = tuple(metadata.get("sport_command_names", ()) or ())
    if action not in supported_actions:
        raise GatewayError(
            "execute_sport_command 当前只支持已公开的 Go2 高层机身动作命令。",
            error_code="invalid_action",
            http_status=422,
            jsonrpc_code=-32602,
            details={
                "action": action,
                "supported_actions": list(supported_actions),
                "hint": "姿态、速度档位、控制模式和低层关节控制请使用专用工具。",
            },
        )
    command_params = payload.get("params") or {}
    if not isinstance(command_params, dict):
        raise GatewayError(
            "params 必须为对象。",
            error_code="invalid_params",
            http_status=422,
            jsonrpc_code=-32602,
        )
    code, result = ctx.robot.execute_action(action, command_params)
    if code is None:
        raise GatewayError(str(result or f"动作 {action} 执行失败。"))
    return {"code": code, "result": result or {}, "action": action}


def handle_set_body_pose(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    """设置机身姿态（roll/pitch/yaw）。

    迁移自 GatewayRuntime._handle_set_body_pose；self.robot→ctx.robot，
    self._float→params.optional_float。
    """

    del requested_by
    if not hasattr(ctx.robot, "execute_action"):
        raise GatewayError("当前机器人入口未实现 execute_action。")
    code, result = ctx.robot.execute_action(
        # "euler" 为 go2 SDK 的机身姿态动作名，此处为网关侧覆盖实现。
        # 长期应迁回驱动侧（go2 private_capabilities 声明 action_name），
        # 由通用私有能力分发器处理，网关不再保留机型专属动作名。
        "euler",
        {
            "roll": params.optional_float(payload, "roll", 0.0),
            "pitch": params.optional_float(payload, "pitch", 0.0),
            "yaw": params.optional_float(payload, "yaw", 0.0),
        },
    )
    if code is None:
        raise GatewayError(str(result or "机身姿态设置失败。"))
    return {"code": code, "result": result or {}}


def handle_set_speed_level(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    """设置速度档位。

    迁移自 GatewayRuntime._handle_set_speed_level；self.robot→ctx.robot，
    self._int→params.optional_int。
    """

    del requested_by
    if not hasattr(ctx.robot, "execute_action"):
        raise GatewayError("当前机器人入口未实现 execute_action。")
    level = params.optional_int(payload, "level", 1)
    code, result = ctx.robot.execute_action("speed_level", {"level": level})
    if code is None:
        raise GatewayError(str(result or "速度档位设置失败。"))
    return {"code": code, "result": result or {}, "level": level}


def handle_switch_control_mode(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    """切换控制模式（high/low）。

    迁移自 GatewayRuntime._handle_switch_control_mode；self.robot→ctx.robot，
    self._require_str→params.require_str。
    """

    del requested_by
    if not hasattr(ctx.robot, "switch_mode"):
        raise GatewayError("当前机器人入口未实现 switch_mode。")
    mode = params.require_str(payload, "mode")
    if mode not in {"high", "low"}:
        raise GatewayError(
            "mode 仅支持 high 或 low。",
            error_code="invalid_params",
            http_status=422,
            jsonrpc_code=-32602,
        )
    code, result = ctx.robot.switch_mode(mode)
    return {"code": code, "result": result or {}, "mode": mode}


def handle_set_obstacle_avoidance(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    """开关实时避障。

    迁移自 GatewayRuntime._handle_set_obstacle_avoidance；self.robot→ctx.robot。
    ObstacleAvoidanceControlDataPlane Protocol 取自 providers.data_plane（其定义源头，
    drivers.common 仅再导出），守 R4b：skills 不依赖 gateways。
    """

    del requested_by
    enabled = bool(payload.get("enabled", True))
    data_plane = getattr(ctx.robot, "data_plane", None)
    if data_plane is None or not isinstance(data_plane, ObstacleAvoidanceControlDataPlane):
        raise GatewayError("当前机器人入口不支持避障开关控制。")
    if not data_plane.is_obstacle_avoidance_control_available():
        raise GatewayError("Go2 官方 obstacles_avoid 服务未就绪，当前不能切换实时避障。")
    try:
        result = data_plane.set_obstacle_avoidance_enabled(enabled)
    except Exception as exc:
        raise GatewayError(f"设置实时避障失败：{exc}") from exc
    if isinstance(result, dict):
        return result
    return {"obstacle_avoidance_enabled": enabled}
