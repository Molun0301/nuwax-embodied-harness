"""健康信息直接读取各拥有方。"""
from dataclasses import asdict

from core import components
from services.navigation.motion_client import MotionError


def summary(platform) -> dict:
    processes = components.status()
    try:
        guard = (asdict(platform.motion.status()) if processes.get("motion_guard", {}).get("state") == "running"
                 else {"running": False})
    except MotionError as error:
        guard = {"running": True, "error": error.message}
    perception = platform.perception.video_runtime.get_status().model_dump(mode="json")
    perception["self_body_filter"] = platform.perception.self_body_filter_status()
    return {"processes": processes, "robot": asdict(platform.robot.get_status()),
            "motion_guard": guard, "geometry": {"connected": platform.geometry_state.connected,
                "state": platform.geometry_state.snapshot()}, "tasks": {"active": platform.tasks.active()},
            "input_bridge": platform.input_bridge.status(),
            "perception": perception,
            "memory": platform.memory.get_summary(), "artifacts": platform.artifacts.get_cached_summary(),
            "audio": platform.audio.get_cached_volume_state(), "follow": {"started": platform.follow.started},
            "streams": platform.streams.stream_bus.status(), "events": platform.events.status()}
