"""运动能力只通过运动守卫输出命令。"""
import time

from core.capability import Capability
from drivers.go2.capabilities import build_go2_sport_command_input_schema
from gateways.capabilities.common import rpc_errors, schema
from services.navigation.motion_client import MotionSource


@rpc_errors
def relative_move(platform, ctx, args):
    vx, vy, wz = args.get("vx", 0.0), args.get("vy", 0.0), args.get("vyaw", 0.0)
    duration = args["duration_sec"]
    command_limits = platform.motion.status().command_limits
    limited = False
    start = time.monotonic()
    with platform.motion.lease(MotionSource.GATEWAY_RELATIVE, args["motion_generation"]) as lease:
        while True:
            ctx.check()
            elapsed = time.monotonic() - start
            if elapsed >= duration:
                break
            limited_now, _ = lease.velocity(vx, vy, wz)
            limited = limited or limited_now
            ctx.progress(elapsed / duration, "moving", "正在相对移动。", elapsed_sec=elapsed)
            ctx.wait(min(args.get("interval_sec", 0.05), duration - elapsed))
    return {"vx": vx, "vy": vy, "vyaw": wz, "duration_sec": duration,
            "limited": limited, "command_limits": command_limits}


@rpc_errors
def sport(platform, args):
    generation = platform.motion.sport(action=args["action"], enabled=args.get("params", {}).get("flag", True))
    return {"action": args["action"], "generation": generation}


@rpc_errors
def body_pose(platform, args):
    return {"generation": platform.motion.sport(body_pose=(args.get("roll", 0), args.get("pitch", 0), args.get("yaw", 0)))}


@rpc_errors
def control_mode(platform, args):
    generation = platform.motion.sport(control_mode=args["mode"])
    return {"mode": args["mode"], "generation": generation}


@rpc_errors
def speed_level(platform, args):
    platform.motion.setting(speed_level=args["level"])
    return {"level": args["level"]}


@rpc_errors
def avoidance(platform, args):
    platform.motion.setting(obstacle_avoidance=args["enabled"])
    return {"enabled": args["enabled"]}


GO2 = frozenset({"go2"})
CAPABILITIES = [
    Capability("relative_move", "以给定速度和时长执行相对移动；速度由守卫按设备限幅执行。",
               schema({"vx": {"type": "number"}, "vy": {"type": "number"}, "vyaw": {"type": "number"},
                       "duration_sec": {"type": "number", "exclusiveMinimum": 0},
                       "interval_sec": {"type": "number", "exclusiveMinimum": 0},
                       "timeout_sec": {"type": "number", "exclusiveMinimum": 0}}, ("vx", "duration_sec")),
               "task", relative_move, needs=("motion_guard",), motion=True),
    Capability("execute_sport_command", "执行机体动作。", build_go2_sport_command_input_schema(),
               "action", sport, needs=("motion_guard",), motion=True, timeout_sec=15, robots=GO2),
    Capability("set_body_pose", "设置机身欧拉角。",
               schema({k: {"type": "number"} for k in ("roll", "pitch", "yaw")}), "action", body_pose,
               needs=("motion_guard",), motion=True, robots=GO2),
    Capability("switch_control_mode", "切换高层或低层控制。",
               schema({"mode": {"type": "string", "enum": ["high", "low"]}}, ("mode",)),
               "action", control_mode, needs=("motion_guard",), motion=True, robots=GO2),
    Capability("set_speed_level", "设置速度档位。",
               schema({"level": {"type": "integer", "minimum": 1, "maximum": 3}}, ("level",)),
               "action", speed_level, needs=("motion_guard",), robots=GO2),
    Capability("set_obstacle_avoidance", "设置实时避障。",
               schema({"enabled": {"type": "boolean"}}, ("enabled",)),
               "action", avoidance, needs=("motion_guard",), robots=GO2),
]
