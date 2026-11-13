"""地图清理能力；参数只在统一调用入口校验。"""
from core.capability import Capability
from gateways.capabilities.common import schema


def preview(platform, args):
    return platform.map_cleanup.preview(args["map_name"], args.get("strength_percent", 50))


def status(platform, args):
    return platform.map_cleanup.status(args["preview_id"])


def cancel(platform, args):
    return platform.map_cleanup.cancel(args["preview_id"])


def apply(platform, ctx, args):
    return platform.map_cleanup.apply(args["preview_id"], args["new_map_name"], ctx)


TEXT = {"type": "string"}
CAPABILITIES = [
    Capability("preview_map_cleanup", "预览完成地图清理", "对已保存地图按力度识别浮空点，生成真实清理预览。",
               schema({"map_name": TEXT, "strength_percent": {"type": "integer", "minimum": 0, "maximum": 100, "default": 50}},
                      ("map_name",)), "action", preview, agent_tool=False, normalize_names=("map_name",)),
    Capability("get_map_cleanup", "读取清理预览", "读取清理作业状态与工具统计摘要。",
               schema({"preview_id": TEXT}, ("preview_id",)), "query", status, agent_tool=False),
    Capability("cancel_map_cleanup", "取消清理预览", "结束清理预览进程并删除预览工作目录。",
               schema({"preview_id": TEXT}, ("preview_id",)), "action", cancel, agent_tool=False),
    Capability("apply_map_cleanup", "另存清理后新图", "将确认后的清理候选另存为新地图，保留源地图。",
               schema({"preview_id": TEXT, "new_map_name": TEXT, "confirmed": {"type": "boolean", "const": True}},
                      ("preview_id", "new_map_name", "confirmed")), "task", apply, agent_tool=False,
               normalize_names=("new_map_name",)),
]
