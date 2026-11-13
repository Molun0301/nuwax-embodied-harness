"""组合能力的参数表，处理器在同一个任务线程内顺序执行各步骤。"""
from core.capability import Capability
from gateways.capabilities.common import schema
from gateways.capabilities.exploration import EXPLORE_SCHEMA
from gateways.capabilities.navigation import explore_and_find_target, find_target, inspect_target

TEXT = {"type": "string"}
TIMEOUT = {"type": "number", "exclusiveMinimum": 0}
THRESHOLD = {"type": "number", "minimum": 0, "maximum": 1}
QUERY = {"query": TEXT, "camera_id": TEXT, "timeout_sec": TIMEOUT}

CAPABILITIES = [
    Capability("find_target", "查找目标", "识别目标、查询地图证据，前往低歧义候选并核对。",
        schema({**QUERY, "map_name": TEXT, "verify_similarity_threshold": THRESHOLD}, ("query",)),
        "task", find_target, needs=("nav_stack", "geometry"), motion=True, timeout_sec=900,
        normalize_names=("map_name",)),
    Capability("explore_and_find_target", "探索并查找目标", "探索并检查新观察，找到候选后停止探索、导航并核对。",
        schema({**EXPLORE_SCHEMA["properties"], **QUERY,
                "search_refresh_interval_sec": TIMEOUT, "verify_similarity_threshold": THRESHOLD}, ("query",)),
        "task", explore_and_find_target, needs=("nav_stack", "geometry"), motion=True, timeout_sec=900,
        normalize_names=("map_name",)),
    Capability("inspect_target", "检查目标", "检查当前目标，可先前往已知命名地点后再检查。",
        schema({**QUERY, "detector_backend": TEXT, "auto_navigate": {"type": "boolean"}}, ("query",)),
        "task", inspect_target, needs=("nav_stack", "geometry"), motion=True, timeout_sec=900),
]
