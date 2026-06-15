"""感知域 handler（P2-4 批 R2）。

从 gateways/runtime.py 下放的感知与状态读取技能。handler 为模块级纯函数，首参
ctx: SkillContext，函数体内原 self.xxx 重写为 ctx.xxx，原 self._get_provider 改用
skills.common.get_provider（纯助手）。行为与抛错条件与迁移前完全一致。

本批下放（分析文档 §2 感知域，8 个 A / A- 类）：
    handle_capture_image、handle_get_joint_state、handle_get_imu_state、
    handle_get_latest_observation、handle_perceive_current_scene、
    handle_describe_current_scene、handle_get_latest_perception、
    handle_get_perception_runtime_status。
签名统一 (ctx, payload, *, requested_by)，无 request_context。

留网关（C 类）：start_perception_runtime / stop_perception_runtime（回写网关可变
属性 _perception_exploration_coupled，与探索 attach/detach 共享运行时状态）。

StateProvider 取自 providers（其定义源头），守 R4b：skills 不依赖 gateways。
"""

from __future__ import annotations

from typing import Any, Dict, Optional

from contracts.events import RuntimeEventCategory
from core.errors import GatewayError
from providers import StateProvider
from skills.common import get_provider
from skills.common.context import SkillContext


def handle_capture_image(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    """采集一帧图像并写入宿主机，发布 artifact.created 事件。

    迁移自 GatewayRuntime._handle_capture_image；self.observation_service→
    ctx.observation_service，self.event_bus→ctx.event_bus（保持不判空，与原一致）。
    """

    camera_id = payload.get("camera_id")
    context = ctx.observation_service.capture_observation(
        camera_id=camera_id,
        requested_by=requested_by,
    )
    artifact = context.image_artifact
    if artifact is None:
        raise GatewayError("观察上下文未生成图像制品。")
    ctx.event_bus.publish(
        ctx.event_bus.build_event(
            "artifact.created",
            category=RuntimeEventCategory.PERCEPTION,
            source="gateway_runtime",
            subject_id=artifact.artifact_id,
            message="图像制品已写入宿主机。",
            payload={"artifact_id": artifact.artifact_id, "camera_id": context.camera_id},
        )
    )
    return {
        "artifact": artifact,
        "observation_context": context,
        "camera_id": context.camera_id,
        "frame_id": context.observation.frame_id,
        "encoding": context.observation.metadata.get("encoding"),
    }


def handle_get_joint_state(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    """读取关节状态；指定 joint_name 时额外返回单关节。

    迁移自 GatewayRuntime._handle_get_joint_state；self._get_provider→
    skills.common.get_provider(ctx.robot, ...)。
    """

    del requested_by
    state_provider = get_provider(ctx.robot, StateProvider, "状态提供器")
    joints = list(state_provider.get_joint_states())
    joint_name = str(payload.get("joint_name", "")).strip()
    if not joint_name:
        return {"joints": joints, "joint": None}
    joint = next((item for item in joints if item.name == joint_name), None)
    return {"joints": joints, "joint": joint}


def handle_get_imu_state(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    """读取 IMU 状态。

    迁移自 GatewayRuntime._handle_get_imu_state；self._get_provider→
    skills.common.get_provider(ctx.robot, ...)。
    """

    del payload, requested_by
    state_provider = get_provider(ctx.robot, StateProvider, "状态提供器")
    return {"imu_state": state_provider.get_imu_state()}


def handle_get_latest_observation(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    """返回最近一次观察缓存，无缓存抛 GatewayError。

    迁移自 GatewayRuntime._handle_get_latest_observation；
    self.observation_service→ctx.observation_service。
    """

    del requested_by
    camera_id = payload.get("camera_id")
    context = ctx.observation_service.get_latest_observation(camera_id)
    if context is None:
        raise GatewayError("当前还没有可用的观察缓存。")
    return {"observation_context": context}


def handle_perceive_current_scene(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    """对当前场景执行一次感知（检测）。

    迁移自 GatewayRuntime._handle_perceive_current_scene；self.perception_service→
    ctx.perception_service（批 R2 接缝增量新增字段）。
    """

    camera_id = payload.get("camera_id")
    detector_backend = payload.get("detector_backend")
    context = ctx.perception_service.perceive_current_scene(
        camera_id=camera_id,
        requested_by=requested_by,
        detector_backend_name=detector_backend,
    )
    return {"perception_context": context}


def handle_describe_current_scene(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    """生成当前场景的自然语言描述。

    迁移自 GatewayRuntime._handle_describe_current_scene；self.perception_service→
    ctx.perception_service。返回 summary=scene_summary.headline + 原对象。
    """

    camera_id = payload.get("camera_id")
    detector_backend = payload.get("detector_backend")
    refresh = bool(payload.get("refresh", True))
    context = ctx.perception_service.describe_current_scene(
        camera_id=camera_id,
        refresh=refresh,
        requested_by=requested_by,
        detector_backend_name=detector_backend,
    )
    return {
        "summary": context.scene_summary.headline,
        "scene_summary": context.scene_summary,
        "perception_context": context,
    }


def handle_get_latest_perception(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    """返回最近一次感知缓存，无缓存抛 GatewayError。

    迁移自 GatewayRuntime._handle_get_latest_perception；self.perception_service→
    ctx.perception_service。
    """

    del requested_by
    camera_id = payload.get("camera_id")
    context = ctx.perception_service.get_latest_perception(camera_id)
    if context is None:
        raise GatewayError("当前还没有可用的感知缓存。")
    return {"perception_context": context}


def handle_get_perception_runtime_status(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    """返回持续感知运行时状态。

    迁移自 GatewayRuntime._handle_get_perception_runtime_status；
    self.perception_video_runtime→ctx.perception_video_runtime。
    """

    del payload, requested_by
    return {"perception_runtime": ctx.perception_video_runtime.get_status()}
