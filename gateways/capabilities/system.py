"""系统与任务能力。"""
from dataclasses import asdict

from core import components
from core.capability import Capability
from core.errors import GatewayError
from gateways.capabilities.common import rpc_errors, schema
from gateways.invoke import stop_all_motion


@rpc_errors
def robot_status(platform, args):
    running = components.status()
    guard = (asdict(platform.motion.status()) if running.get("motion_guard", {}).get("state") == "running"
             else {"running": False})
    return {"robot_name": platform.robot.manifest.robot_name,
            "robot_model": platform.robot.manifest.robot_model,
            "assembly": asdict(platform.robot.get_status()),
            "robot_state": platform.robot_state.get_latest_snapshot(), "motion_guard": guard,
            "perception_runtime": platform.perception.video_runtime.get_status()}


def active_tasks(platform, args):
    return {"active": [asdict(item) for item in platform.tasks.active()],
            "history": [asdict(item) for item in platform.tasks.history(args.get("history_limit", 50))]}


def list_capabilities(platform, args):
    robot = platform.robot.manifest.robot_name
    return {"capabilities": [{"name": cap.name, "description": cap.description,
                              "input_schema": cap.input_schema, "kind": cap.kind,
                              "motion": cap.motion, "agent_tool": cap.agent_tool}
                             for cap in platform.capabilities.for_robot(robot)]}


def task_status(platform, args):
    status = platform.tasks.status(args["task_id"])
    if status is None:
        raise GatewayError("任务不存在。", error_code="task_not_found", http_status=404)
    return asdict(status)


def cancel_task(platform, args):
    cancelled = platform.tasks.cancel(args["task_id"], args.get("message", "用户取消"))
    status = platform.tasks.status(args["task_id"])
    return {"cancelled": cancelled, "task": None if status is None else asdict(status)}


def joint_state(platform, args):
    joints = list(platform.robot.providers.get_joint_states())
    return {"joints": joints, "joint": next((j for j in joints if j.name == args.get("joint_name")), None)}


CAPABILITIES = [
    Capability("get_robot_status", "读取机器人、机体与守卫状态。", schema(), "query", robot_status),
    Capability("get_active_tasks", "读取活动任务与最近终态任务。",
               schema({"history_limit": {"type": "integer", "minimum": 0}}), "query", active_tasks),
    Capability("list_capabilities", "列出当前机器人支持的能力。",
               schema({"include_unsupported": {"type": "boolean"}}), "query", list_capabilities),
    Capability("get_task_status", "读取任务快照。", schema({"task_id": {"type": "string"}}, ("task_id",)),
               "query", task_status),
    Capability("cancel_task", "请求取消任务，处理器退出后结束。",
               schema({"task_id": {"type": "string"}, "message": {"type": "string"}}, ("task_id",)),
               "action", cancel_task),
    Capability("stop_all_motion", "急停、取消运动任务并停止导航与探索。",
               schema({"reason": {"type": "string"}}), "action",
               lambda p, a: stop_all_motion(p, a.get("reason", "用户急停"))),
    Capability("get_joint_state", "读取全部或指定关节状态。",
               schema({"joint_name": {"type": "string"}}), "query", joint_state, agent_tool=False),
    Capability("get_imu_state", "读取惯性测量单元状态。", schema(), "query",
               lambda p, a: {"imu_state": p.robot.providers.get_imu_state()}, agent_tool=False),
    Capability("get_volume", "读取设备与软件音量。", schema(), "query",
               lambda p, a: {"volume_state": p.audio.get_volume_state()}, robots=frozenset({"go2"})),
    Capability("set_volume", "设置音量。",
               schema({"volume": {"type": "number", "minimum": 0, "maximum": 1}}, ("volume",)), "action",
               lambda p, a: {"volume_state": p.audio.set_volume(a["volume"])}, robots=frozenset({"go2"})),
]
