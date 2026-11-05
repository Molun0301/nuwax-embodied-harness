"""楼栋任务、假Geometry状态与同步HTTP设施桥的本地行为诊断；不验证物理闭环。"""
from __future__ import annotations

import argparse
from contextlib import ExitStack
from dataclasses import asdict
import json
from pathlib import Path
import queue
import sys
import time
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from core.capability import CapabilityTable
from core.event_bus import EventBus
from core.tasks import TaskRunner, TaskState
from gateways.capabilities import building, exploration, maps, navigation, system
from gateways.invoke import invoke
from gateways.serialization import to_jsonable
from scripts.diagnostics.building_geometry import GeometrySimulator
from scripts.diagnostics.facility_bridge import FacilityBridge, FacilityWorld
from services.geometry.client import GeometryClient
from services.geometry.maps import MapStore
from services.geometry.proto_gen import geometry_runtime_pb2 as pb
from services.geometry.state import GeometryStateView, SessionPhase
from services.navigation.building_catalog import BuildingCatalog
from services.navigation.facilities.http_provider import HttpFacilityProvider

SCENARIOS = ("complete", "wrong-floor", "explore-unmapped", "cancel", "explore-cancel")
TERMINAL = {TaskState.SUCCEEDED, TaskState.FAILED, TaskState.CANCELLED, TaskState.TIMED_OUT}


class LocalMotion:
    """只提供任务运动代次；此诊断不连接运动守卫或执行器。"""
    def __init__(self):
        self.generation = 0

    def begin(self):
        self.generation += 1
        return self.generation


class LocalMemory:
    """只记录地图能力选用的库名，不启动模型或语义记忆服务。"""
    def __init__(self):
        self.active = None

    def activate_memory_library(self, *, library_name, load_history=True, reset_library=False):
        self.active = library_name


def topology():
    return {"building_id": "模拟楼栋", "display_name": "模拟楼栋", "floors": [
        {"floor_id": "F1", "display_name": "一层", "map_name": "模拟一层"},
        {"floor_id": "F2", "display_name": "二层", "map_name": "模拟二层"}], "connections": [{
            "connection_id": "一到二", "from_floor_id": "F1", "to_floor_id": "F2",
            "provider_id": "sim_facility", "facility_id": "lift",
            "waiting_pose": {"x": -0.8, "y": 0}, "boarding_pose": {"x": 0, "y": 0},
            "arrival_pose": {"x": 0, "y": 0}, "exit_pose": {"x": 0.8, "y": 0},
            "docking_calibration_ref": "synthetic-dock-v1", "expected_transfer_sec": 2.0}]}


def main(output: Path, timeout: float, scenario="complete") -> dict:
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    report = {"scenario": "building-" + scenario, "passed": False, "shutdown_passed": False,
        "validation_scope": "假Geometry状态行为验证", "physical_closed_loop": False,
        "simulation": "Geometry目标到达、定位、探索完成和地图目录均由本地假服务产生；无扫描、物理求解、运动守卫或真实机器人。",
        "components": ["真实GeometryClient与GeometryStateView", "真实TaskRunner与Invoke",
                       "真实HttpFacilityProvider与本机同步HTTP设施桥", "同proto进程内假Geometry"],
        "before_transport": [], "expected_terminal": "cancelled" if scenario in {"wrong-floor", "cancel", "explore-cancel"} else "succeeded"}
    deadline = time.monotonic() + timeout
    events = EventBus()
    pending = queue.Queue()
    subscription = events.subscribe(pending.put, event_types=("task.updated",))
    store = MapStore(str(output / "maps"))
    backend = GeometrySimulator(store, hold_navigation=scenario == "cancel", hold_exploration=scenario == "explore-cancel")
    world = FacilityWorld(scenario)
    runner = client = view = catalog = bridge = None
    log = (output / "snapshots.jsonl").open("w", encoding="utf-8")

    def remaining():
        value = deadline - time.monotonic()
        if value <= 0:
            raise TimeoutError("楼栋状态诊断超过总期限")
        return value

    def before_transport():
        state = backend.snapshot()
        assert state.phase == pb.SESSION_PHASE_IDLE, "运载前须结束旧Geometry会话"
        report["before_transport"].append({"geometry_phase": "idle", "meaning": "只证明假Geometry会话已结束，不证明物理停车。"})

    world.before_transport = before_transport
    try:
        with ExitStack() as resources:
            resources.enter_context(backend)
            bridge = resources.enter_context(FacilityBridge(world))
            client = GeometryClient(backend.target)
            resources.callback(client.close)
            view = GeometryStateView(client, events)
            resources.callback(view.stop)
            view.start()
            # 等待假服务的初始订阅建立，业务任务随后使用正式状态流。
            startup = time.monotonic() + remaining()
            while view.snapshot() is None:
                if time.monotonic() >= startup:
                    raise TimeoutError("Geometry初始状态没有到达")
                time.sleep(.01)
            backend.prepare_map("模拟一层")
            if scenario != "explore-unmapped":
                backend.prepare_map("模拟二层")
            catalog = BuildingCatalog(store, providers={"sim_facility": HttpFacilityProvider("sim_facility", bridge.base_url)})
            resources.callback(catalog.close)
            required = set()
            platform = SimpleNamespace(events=events, capabilities=CapabilityTable(), motion=LocalMotion(),
                maps=store, geometry=client, geometry_state=view, buildings=catalog, memory=LocalMemory(),
                exploration_control=exploration.ExplorationControl(),
                robot=SimpleNamespace(manifest=SimpleNamespace(robot_name="local_diagnostic")),
                ensure=lambda names, _timeout: required.update(names))
            runner = platform.tasks = TaskRunner(platform)
            resources.callback(runner.shutdown)
            for module in (building, exploration, maps, navigation, system):
                for capability in module.CAPABILITIES:
                    platform.capabilities.register(capability)

            def call(name, args=None):
                return invoke(platform, name, args or {}, "diagnostic")

            def wait_task(accepted, *, monitor=False):
                task_id = accepted["task"]["task_id"]
                cancelled = False
                while True:
                    try:
                        event = pending.get(timeout=remaining())
                    except queue.Empty as error:
                        raise TimeoutError("任务没有在诊断期限内结束") from error
                    snapshot = to_jsonable(event.payload)
                    log.write(json.dumps({"time": time.time(), "task": snapshot}, ensure_ascii=False) + "\n")
                    log.flush()
                    if event.task_id != task_id:
                        continue
                    observed = runner.status(task_id)
                    if monitor and not cancelled and observed.state not in TERMINAL:
                        reason = None
                        if scenario == "cancel" and snapshot["stage"] == "executing":
                            reason = "本地诊断取消楼层导航"
                        elif scenario == "explore-cancel" and snapshot["stage"] == "exploring":
                            reason = "本地诊断取消整栋探索"
                        elif (scenario == "wrong-floor" and snapshot["stage"] == "waiting_facility"
                                and snapshot["data"].get("floor_id") == "F2"):
                            assert world.floor == "F1", "错层场景必须保留设施实际反馈"
                            assert "模拟二层" not in backend.loaded_maps, "错层不能加载目标层地图"
                            reason = "错层反馈后取消原任务"
                        if reason is not None:
                            report["cancellation"] = call("cancel_task", {"task_id": task_id, "message": reason})
                            cancelled = True
                    if observed.state in TERMINAL:
                        return asdict(observed)

            loaded = wait_task(call("load_map", {"map_name": "模拟一层", "timeout_sec": remaining()}))
            assert loaded["state"] == TaskState.SUCCEEDED, loaded
            view.wait_for(lambda state: state.map_name == "模拟一层" and state.phase is SessionPhase.LOCALIZED,
                          SimpleNamespace(check=remaining, remaining=remaining))
            call("register_building", {"topology": topology()})
            if scenario in {"explore-unmapped", "explore-cancel"}:
                accepted = call("explore_building", {"building_id": "模拟楼栋", "floor_ids": ["F1", "F2"],
                    "per_floor_timeout_sec": remaining(), "timeout_sec": remaining()})
            else:
                accepted = call("navigate_to_floor", {"building_id": "模拟楼栋", "floor_id": "F2",
                    "x": 1.2, "y": 1.0, "yaw_rad": 0.0, "timeout_sec": remaining()})
            terminal = report["terminal"] = wait_task(accepted, monitor=True)
            assert terminal["state"].value == report["expected_terminal"], terminal
            assert not world.errors, world.errors
            assert not world.reserved, "任务结束后设施预约尚未释放"
            if scenario == "complete":
                assert terminal["result"]["floor_id"] == "F2" and world.floor == "F2"
                state = client.get_state(timeout_sec=remaining())
                assert state.map_name == "模拟二层" and state.pose.x == 1.2 and state.pose.y == 1.0
            elif scenario == "explore-unmapped":
                assert [item["floor_id"] for item in terminal["result"]["completed_floors"]] == ["F1", "F2"]
                assert "模拟二层" in backend.created_maps and store.get("模拟二层").saved
                assert world.floor == "F2"
            elif scenario == "wrong-floor":
                assert "模拟二层" not in backend.loaded_maps and world.floor == "F1"
            elif scenario == "cancel":
                assert backend.cancelled_goals, "取消任务没有传到假Geometry目标"
            elif scenario == "explore-cancel":
                assert backend.stopped_explorations, "取消任务没有传到假Geometry探索"
            generations = {item["motion_generation"] for item in backend.goals + backend.explorations}
            assert generations == {platform.motion.generation}, "子步骤没有使用同一个任务运动代次"
            report.update(passed=True, geometry=asdict(client.get_state(timeout_sec=remaining())),
                facility=world.state(), facility_commands=world.commands, required_components=sorted(required),
                motion_generations=sorted(generations), created_maps=backend.created_maps,
                loaded_maps=backend.loaded_maps, map_save_results=backend.map_save_results)
    except Exception as error:
        report["passed"] = False
        report["error"] = f"{type(error).__name__}: {error}"
    finally:
        events.unsubscribe(subscription)
        closed_events = events.shutdown()
        shutdown = {"active_tasks": [] if runner is None else [value.task_id for value in runner.active()],
                    "geometry_server_closed": backend.closed,
                    "geometry_subscription_threads_alive": [] if view is None else [value.name for value in view._threads if value.is_alive()],
                    "facility_http_thread_alive": False if bridge is None else bridge.worker.is_alive(),
                    "events_closed": closed_events}
        report["shutdown"] = shutdown
        report["shutdown_passed"] = (closed_events and backend.closed and not shutdown["active_tasks"]
            and not shutdown["geometry_subscription_threads_alive"] and not shutdown["facility_http_thread_alive"])
        log.close()
        (output / "report.json").write_text(json.dumps(to_jsonable(report), ensure_ascii=False, indent=2), encoding="utf-8")
    return to_jsonable(report)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=30)
    parser.add_argument("--scenario", choices=SCENARIOS, default="complete")
    args = parser.parse_args()
    result = main(args.output_dir, args.timeout, args.scenario)
    print(json.dumps({key: result.get(key) for key in ("passed", "scenario", "physical_closed_loop", "error", "shutdown_passed")}, ensure_ascii=False))
    raise SystemExit(0 if result["passed"] and result["shutdown_passed"] else 1)
