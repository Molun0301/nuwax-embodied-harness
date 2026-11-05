"""楼栋目录、跨层导航与整栋探索能力。"""
from dataclasses import asdict

from contracts.building_navigation import TOPOLOGY_SCHEMA, topology_from_dict
from core.capability import Capability
from gateways.capabilities.common import rpc_errors, schema
from gateways.capabilities.exploration import CONDITIONS
from services.navigation.building_execution import navigate_to_floor as _navigate_to_floor
from services.navigation.building_exploration import explore_building as _explore_building


def list_buildings(platform, args):
    snapshot = platform.geometry_state.snapshot()
    return {"buildings": [asdict(value) for value in platform.buildings.list()],
            "active_map_name": snapshot.map_name if snapshot is not None else None,
            "facility_providers": list(platform.buildings.providers)}


def register_building(platform, args):
    return {"building": asdict(platform.buildings.save(topology_from_dict(args["topology"])))}


@rpc_errors
def navigate_to_floor(platform, ctx, args):
    return _navigate_to_floor(platform, ctx, args)


@rpc_errors
def explore_building(platform, ctx, args):
    return _explore_building(platform, ctx, args)


TEXT = {"type": "string", "minLength": 1}
TIMEOUT = {"type": "number", "exclusiveMinimum": 0}
CAPABILITIES = [
    Capability("list_buildings", "列出楼栋楼层和设施连接。", schema(), "query", list_buildings),
    Capability("register_building", "登记楼栋楼层及设施连接。",
               schema({"topology": TOPOLOGY_SCHEMA}, ("topology",)), "action", register_building),
    Capability("navigate_to_floor", "经登记的设施到目标楼层，加载地图并重定位后导航到目标位姿。",
               schema({"building_id": TEXT, "floor_id": TEXT, "x": {"type": "number"},
                       "y": {"type": "number"}, "yaw_rad": {"type": "number"}, "timeout_sec": TIMEOUT},
                      ("building_id", "x", "y")),
               "task", navigate_to_floor, needs=("nav_stack", "geometry"), motion=True, timeout_sec=600),
    Capability("explore_building", "按楼层顺序跨层、探索并保存地图。",
               schema({"building_id": TEXT, "floor_ids": {"type": "array", "items": TEXT},
                       "stop_conditions": CONDITIONS,
                       "per_floor_timeout_sec": {"type": "number", "exclusiveMinimum": 0, "maximum": 3600},
                       "timeout_sec": {"type": "number", "exclusiveMinimum": 0, "maximum": 43200}},
                      ("building_id",)),
               "task", explore_building, needs=("nav_stack", "geometry"), motion=True, timeout_sec=7200),
]
