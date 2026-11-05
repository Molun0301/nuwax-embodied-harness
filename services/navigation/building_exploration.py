"""整栋探索在当前任务线程里顺序跨层、探索和保存地图。"""
from __future__ import annotations

from services.navigation.building_execution import building, floor, load_floor, transfer


def explore_building(platform, ctx, args):
    from gateways.capabilities.exploration import explore_area

    topology = building(platform, args["building_id"])
    floor_ids = args.get("floor_ids") or [value.floor_id for value in topology.floors if value.enabled]
    completed = []
    for index, floor_id in enumerate(floor_ids):
        ctx.check()
        target = floor(topology, floor_id)
        saved = platform.maps.get(target.map_name)
        continuing = saved is not None and saved.saved
        # 目录只选择产品操作；加载失败原样结束，不把缺图错误改写为新建。
        route = transfer(platform, ctx, topology, target, args["motion_generation"], create_target=not continuing)
        if not route and continuing:
            load_floor(platform, ctx, target)
        ctx.progress(index / len(floor_ids), "exploring_floor", f"正在探索 {target.display_name}。",
                     floor_id=floor_id, map_name=target.map_name)
        conditions = dict(args.get("stop_conditions", {}))
        per_floor_ms = round(args.get("per_floor_timeout_sec", 300.0) * 1000)
        conditions["max_duration_ms"] = min(conditions.get("max_duration_ms", per_floor_ms), per_floor_ms)
        result = explore_area(platform, ctx, {"map_name": target.map_name,
            "continue_saved": continuing, "motion_generation": args["motion_generation"],
            "stop_conditions": conditions, "save_map": True})
        completed.append({"floor_id": floor_id, "map_name": target.map_name, "exploration": result})
        ctx.progress((index + 1) / len(floor_ids), "floor_saved", f"{target.display_name} 已探索并保存。",
                     completed_floors=[value["floor_id"] for value in completed])
    return {"building_id": topology.building_id, "completed_floors": completed}
