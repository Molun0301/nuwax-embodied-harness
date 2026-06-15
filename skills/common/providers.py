"""provider 获取公共助手（P2-4 批 R2 接缝增量）。

从 gateways/runtime.py 下沉的 _get_provider 纯函数式辅助：仅读 robot.providers，
做类型与可用性校验，两道失败均抛 GatewayError。供 skills 层 handler（感知域
get_joint_state/get_imu_state，以及后续 follow/relative_move 若下放）复用。

仅依赖 core.errors.GatewayError（core 为 skills 合法下游，不构成 skills→gateways
反向依赖）。行为与抛错条件与迁移前的 GatewayRuntime._get_provider 完全一致。
网关侧 _get_provider 保留（仍被 C 类 follow/relative_move/inspect 使用）。
"""

from __future__ import annotations

from typing import Any

from core.errors import GatewayError

__all__ = ["get_provider"]


def get_provider(robot: Any, provider_type: type, provider_label: str) -> Any:
    """从 robot.providers 取指定类型的 provider，缺失/不可用抛 GatewayError。

    迁移自 GatewayRuntime._get_provider；self.robot→robot 首参。两道校验：
    1. providers 缺失或非 provider_type 实例 → "当前机器人入口未提供{label}。"
    2. providers.is_available() 为假 → "当前{label}暂不可用。"
    """

    providers = getattr(robot, "providers", None)
    if providers is None or not isinstance(providers, provider_type):
        raise GatewayError(f"当前机器人入口未提供{provider_label}。")
    if not providers.is_available():
        raise GatewayError(f"当前{provider_label}暂不可用。")
    return providers
