"""SkillContext 依赖容器（P2-4 批0 脚手架）。

封装 handler 下放后所需的全部句柄，由网关组合根（gateways/runtime.py）构造并注入。
本模块仅引用 core 的类型；其余业务 service/runtime 句柄以 Any 占位，避免在 skills 层
反向感知 gateways 内部结构（守 R4b：skills 不得依赖 gateways 包）。

序列化助手以 Callable 形式注入（设计文档 §3.1 选项 A）：_serialize 依赖
gateways.serialization.to_jsonable，留在网关，以可调用对象下传，skills 调用
ctx.serialize(obj) 即可，无需反向依赖 gateways。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Optional

from core import CapabilityRegistry, EventBus, StateStore

__all__ = ["SkillContext"]


@dataclass(frozen=True)
class SkillContext:
    """handler 下放后的依赖容器。

    字段类型说明：capability_registry/state_store/event_bus 为 core 明确类型；
    robot 及各 service/runtime 句柄类型分布在 contracts/services/skills 侧，
    此处以 Any 占位以避免过度耦合具体实现类。
    """

    capability_registry: CapabilityRegistry
    robot: Any
    robot_state_service: Any
    observation_service: Any
    perception_service: Any
    mapping_service: Any
    localization_service: Any
    memory_service: Any
    navigation_service: Any
    map_workspace_service: Any
    perception_video_runtime: Any
    audio_service: Any
    safety_guard: Any
    task_manager: Any
    state_store: StateStore
    event_bus: Optional[EventBus]
    serialize: Callable[[Any], Any]
    serialize_lightweight: Callable[[Any], Any]
    # P2-4 批3：编排型 handler（relocalize/activate/delete/load_map_version）依赖网关
    # 运行时内部状态的私有方法（几何运行时启动锁/事件、流帧发布等），无法下沉到 skills，
    # 按设计文档 §2.5「作为 ctx 上的可调用句柄注入」下传（与 serialize 同范式，守 R4b）。
    ensure_geometry_runtime_for_capability: Callable[[str], None]
    ensure_geometry_runtime_started: Callable[[str], None]
    refresh_workspace_snapshots: Callable[..., None]
    publish_pointcloud_map_stream_frame: Callable[..., None]
    publish_runtime_governance_stream_frame: Callable[[], None]
    sync_perception_runtime_for_memory_state: Callable[..., None]
    map_delete_also_delete_library: Callable[[], bool]
    # RTAB merge：activate_map / get_active_map 在 RTAB 侧返回 geometry_stack 摘要
    # （+ next_step_hint），_geometry_stack_summary 仍留在网关运行时（start_mapping 等复用），
    # 以可调用句柄注入（与上方 ensure_geometry_runtime_* 同范式，守 R4b）。
    geometry_stack_summary: Callable[[], Any]
