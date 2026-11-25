"""组合能力的参数表，处理器在同一个任务线程内顺序执行各步骤。"""
from core.capability import Capability
from gateways.capabilities.common import schema
from gateways.capabilities.exploration import EXPLORE_SCHEMA
from gateways.capabilities.navigation import explore_and_find_target, find_target, inspect_target

TEXT = {"type": "string"}
TIMEOUT = {"type": "number", "exclusiveMinimum": 0}
THRESHOLD = {"type": "number", "minimum": 0, "maximum": 1}
QUERY = {"query": {**TEXT, "description": "目标物品类别，用英文名词（如 apple、nightstand）。用作记忆标签，未给外观描述时也用作检测词。"},
         "appearance_description": {**TEXT, "description": "可选的英文外观描述（如 white two drawer cabinet）。用作检测词，并作为可查询的实例属性保存。"},
         "camera_id": TEXT, "timeout_sec": TIMEOUT}

CAPABILITIES = [
    Capability("find_target", "识别目标、查询地图证据，前往低歧义候选并核对。可限定区域。",
        schema({**QUERY, "map_name": TEXT, "verify_similarity_threshold": THRESHOLD,
                "region": {"type": ["string", "integer"],
                           "description": "只在指定区域内找物。可填区域名（如厨房）、场景空间 ID 或地图编号（如 R2、2）。"}},
               ("query",)),
        "task", find_target, needs=("nav_stack", "geometry"), motion=True, timeout_sec=900,
        normalize_names=("map_name",)),
    Capability("explore_and_find_target", "探索并检查新观察，找到候选后停止探索、导航并核对。",
        schema({**EXPLORE_SCHEMA["properties"], **QUERY,
                "search_refresh_interval_sec": TIMEOUT, "verify_similarity_threshold": THRESHOLD}, ("query",)),
        "task", explore_and_find_target, needs=("nav_stack", "geometry"), motion=True, timeout_sec=900,
        normalize_names=("map_name",)),
    Capability("inspect_target", "停稳后连拍检查目标并记录各帧，可先前往已知命名地点。",
        schema({**QUERY, "detector_backend": TEXT, "auto_navigate": {"type": "boolean"}}, ("query",)),
        "task", inspect_target, needs=("nav_stack", "geometry"), motion=True, timeout_sec=900),
]
