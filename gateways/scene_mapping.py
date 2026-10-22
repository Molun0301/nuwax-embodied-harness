"""语义地图共用能力注册；网页与智能体进入相同服务和任务入口。"""
from __future__ import annotations

from functools import partial
from typing import Any, Dict

from contracts.capabilities import CapabilityDescriptor, CapabilityExecutionMode, CapabilityRiskLevel
from contracts.skills import SkillCategory, SkillDescriptor
from core.errors import GatewayError


def _schema(properties=None, required=()):
    return {"type": "object", "properties": properties or {}, "required": list(required), "additionalProperties": False}


SPECS = (
    ("get_scene_mapping_status", "语义建图状态", "读取语义后台及最近处理结果，查询不触发模型加载。", "status", _schema()),
    ("start_scene_mapping", "启动语义建图", "启动受管后台，基于当前地图和已记录观察自动产生区域与标签候选。", "start", _schema()),
    ("stop_scene_mapping", "停止语义建图", "请求后台停止，迟到结果拒收；stop_pending表示仍在等待在途调用退出。", "stop", _schema()),
    ("refresh_semantic_map", "刷新语义地图", "异步提取几何区域并用配置模型评估；不会启动机器人运动。", "refresh", _schema({
        "apply_labels": {"type": "boolean"}, "timeout_sec": {"type": "number", "minimum": 1, "maximum": 1800}})),
    ("get_semantic_map", "读取语义地图", "读取当前地图的区域边界、标签、候选和版本；过期几何明确标注。", "map", _schema()),
    ("get_scene_graph", "查询空间关系图", "检索区域、连接口、物品和观察节点；按节点展开邻域，返回图版本和明确截断信息。", "graph", _schema({
        "query": {"type": "string", "maxLength": 200}, "node_id": {"type": "string"},
        "hops": {"type": "integer", "minimum": 0, "maximum": 4},
        "kinds": {"type": "array", "items": {"type": "string", "enum": ["region", "passage", "object", "observation"]}, "maxItems": 4},
        "limit": {"type": "integer", "minimum": 1, "maximum": 500}, "expected_graph_revision": {"type": "string"}})),
    ("get_scene_graph_node", "读取空间节点详情", "读取节点坐标、精确范围、邻接关系和可用图像引用；图像需通过制品接口实际读取。", "graph_node", _schema({
        "node_id": {"type": "string"}, "expected_graph_revision": {"type": "string"}}, ("node_id",))),
    ("plan_scene_graph_route", "查询区域候选路径", "按最少连接口数量查询区域链及出入口位置；基于二维几何连接，实际机器人通行未检查，不执行运动。", "graph_route", _schema({
        "from_node_id": {"type": "string"}, "to_node_id": {"type": "string"},
        "expected_graph_revision": {"type": "string", "minLength": 1}},
        ("from_node_id", "to_node_id", "expected_graph_revision"))),
    ("render_scene_graph_image", "导出语义地图图像", "将同一图版本渲染为带区域编号和关系的地图图片，返回制品和编号对照；可指定节点取得局部图。", "graph_image", _schema({
        "node_id": {"type": "string"}, "expected_graph_revision": {"type": "string"},
        "width": {"type": "integer", "minimum": 512, "maximum": 2048}})),
    ("locate_current_scene_space", "查询当前空间归属", "按已有定位和精确区域边界判断归属，未知和歧义分别返回。", "locate", _schema()),
    ("update_scene_space_label", "修改区域名称", "按预期修订设置区域名称并保护该显式修改，不伪造人工审核或导航就绪。", "label", _schema({
        "scene_space_id": {"type": "string"}, "map_name": {"type": "string"},
        "expected_revision": {"type": "integer", "minimum": 1},
        "label": {"type": "string", "minLength": 1, "maxLength": 120},
        "request_id": {"type": "string", "minLength": 1, "maxLength": 160}},
        ("scene_space_id", "map_name", "expected_revision", "label", "request_id"))),
)


def _handle(runtime, operation, arguments: Dict[str, Any], *, requested_by=None, request_context=None):
    del request_context
    service = runtime.scene_mapping_service
    if operation == "status":
        return service.status()
    if operation == "start":
        return service.start()
    if operation == "stop":
        return service.stop()
    if operation == "map":
        return service.get_map()
    if operation == "graph":
        return service.graph.query(**arguments)
    if operation == "graph_node":
        return service.graph.node(**arguments)
    if operation == "graph_route":
        return service.graph.route(**arguments)
    if operation == "graph_image":
        return service.graph.image(**arguments)
    if operation == "locate":
        return service.locate()
    if operation == "label":
        space = service.update_label(
            scene_space_id=arguments["scene_space_id"], map_name=arguments["map_name"],
            expected_revision=arguments["expected_revision"], label=arguments["label"],
            request_id=arguments["request_id"], requested_by=requested_by or "unknown")
        return {"scene_space": space}
    if operation == "refresh":
        if not service.config.enabled:
            raise GatewayError("请先在设置中启用语义建图。", error_code="scene_mapping_disabled", http_status=409)
        # 受理时捕获范围，排队期间切图不能把原请求执行到另一个地图。
        capture = runtime.memory_service.capture_scene_mapping_state()

        def runner(context):
            context.ensure_active()
            context.update(stage="semantic_mapping", message="正在提取区域及核对观察证据。")
            result = service.run_once(apply_labels=arguments.get("apply_labels"), cancel_check=context.ensure_active, expected_capture=capture)
            context.ensure_active()
            return result

        return runtime.task_manager.submit("refresh_semantic_map", runner,
                    input_payload=arguments, requested_by=requested_by,
                    timeout_sec=float(arguments.get("timeout_sec", 300)),
                    metadata={"map_name": capture["library_name"], "source_epoch": capture["binding"]["source_epoch"]})
    raise GatewayError("未知语义地图操作。", error_code="invalid_params", http_status=422)


def register_scene_mapping_capabilities(runtime):
    names = []
    for name, title, description, operation, schema in SPECS:
        asynchronous = operation == "refresh"
        descriptor = CapabilityDescriptor(name=name, display_name=title, description=description,
            execution_mode=CapabilityExecutionMode.ASYNC_TASK if asynchronous else CapabilityExecutionMode.SYNC,
            risk_level=CapabilityRiskLevel.LOW, input_schema=schema,
            output_schema={"type": "object"}, exposed_to_agent=True,
            cancel_supported=asynchronous, timeout_sec=300 if asynchronous else None,
            tags=["memory", "semantic_map"])
        runtime.capability_registry.register(descriptor, partial(_handle, runtime, operation),
                                            owner="gateway_runtime", metadata={"capability_layer": "platform"}, overwrite=True)
        names.append(name)
    return tuple(names)


def register_scene_mapping_skills(runtime):
    for name, title, description, _operation, _schema_value in SPECS:
        runtime.skill_registry.register(SkillDescriptor(name=name, display_name=title,
            description=description, category=SkillCategory.MEMORY, capability_name=name,
            exposed_to_agent=True, tags=["semantic_map"]), overwrite=True)
