"""协议共用的完整结果与错误编码。"""
from __future__ import annotations

from base64 import b64encode
from io import BytesIO
from typing import Any

from PIL import Image

from core.errors import as_gateway_error
from gateways.serialization import to_jsonable, to_json_text


def build_public_error_payload(error, **_context) -> dict:
    """错误语义由业务层拥有，协议层保留原始详情。"""
    error = as_gateway_error(error)
    return {"code": error.error_code, "message": error.message,
            "details": to_jsonable(error.details)}


def build_task_summary(status, **_unused) -> dict:
    """返回任务执行器拥有的完整快照。"""
    return to_jsonable(status)


def build_public_runtime_event(event: Any) -> dict:
    return to_jsonable(event)


def _mcp_images(content, platform, tool_name):
    if tool_name == "render_scene_graph_image":
        return [content["artifact"]]
    if tool_name == "get_scene_space":
        space = content["scene_space"]
        ids = dict.fromkeys(identity for sample in space["observation_samples"] for identity in sample["artifact_ids"])
        return [to_jsonable(platform.memory.photo_ref(identity, library_name=space["map_name"]))
                for identity in list(ids)[:4]]
    task = content.get("task", content) if tool_name in {"get_task_status", "wait_task"} else {}
    if task.get("capability") == "find_target" and task.get("result"):
        found = task["result"]
        writeback = found.get("observation_writeback", {})
        if writeback.get("memory_id"):
            memory = platform.memory.get_record("memory", writeback["memory_id"],
                library_name=found["navigation_candidate"]["map_name"])
            return memory.metadata["photo_refs"]
    return content.get("images", [])


def _mcp_image_block(platform, artifact):
    identity = artifact["artifact_id"]
    data = (platform.memory.read_photo(identity, library_name=artifact["metadata"]["map_name"])
            if artifact["uri"].startswith("/api/memory/photos/") else platform.artifacts.read_bytes(identity))
    edge = platform.config.mcp.image_max_edge_px
    encoded = BytesIO()
    with Image.open(BytesIO(data)) as original:
        image = original.convert("RGB")
        image.thumbnail((edge, edge))
        image.save(encoded, "JPEG")
    return {"type": "image", "mimeType": "image/jpeg", "data": b64encode(encoded.getvalue()).decode("ascii")}


def build_mcp_tool_result(result: Any, *, platform=None, tool_name=None) -> dict:
    """MCP（模型上下文协议）的文本与结构化通道承载同一完整结果。"""
    content = to_jsonable(result)
    blocks = []
    if platform is not None and isinstance(content, dict):
        images = _mcp_images(content, platform, tool_name)
        if images:
            content = {**content, "images": images}
        artifact = content.get("artifact")
        references = {item["artifact_id"]: item for item in ([artifact] if artifact else []) + images
                      if item["kind"] == "image"}
        blocks = [_mcp_image_block(platform, item) for item in references.values()]
    return {"isError": isinstance(content, dict) and bool(content.get("error")),
            "content": [{"type": "text", "text": to_json_text(content)}, *blocks],
            "structuredContent": content}
