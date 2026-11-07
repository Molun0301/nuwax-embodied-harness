from __future__ import annotations

import os
import logging
from core.config import motion_guard_socket, value, binary_path, install_home
from pathlib import Path
import signal
from typing import Optional
import threading

from .core import MotionGuardCore
from .go2_actuator import ActuatorError, Go2Actuator
from .motion_control_actuator import MotionControlActuator
from .server import MotionGuardUnixServer


def run() -> None:
    socket_path = Path(motion_guard_socket()).expanduser().resolve()
    stop_event = threading.Event()

    def request_stop(_signum: int, _frame: object) -> None:
        stop_event.set()

    previous_handlers = {
        signum: signal.signal(signum, request_stop)
        for signum in (signal.SIGINT, signal.SIGTERM)
    }
    server: Optional[MotionGuardUnixServer] = None
    actuator = None
    try:
        if value('motion_guard.backend').strip() == "motion_control":
            actuator = MotionControlActuator.from_environment()
        else:
            actuator = Go2Actuator.from_environment()
        core = MotionGuardCore(actuator=actuator)
        if stop_event.is_set():
            return
        socket_gid = os.getegid()
        server = MotionGuardUnixServer(
            socket_path=socket_path,
            core=core,
            socket_gid=socket_gid,
        )
        server.start()
        while not stop_event.wait(timeout=1.0):
            pass
    finally:
        try:
            if server is not None:
                server.stop()
        finally:
            try:
                closer = getattr(actuator, "close", None)
                if callable(closer):
                    closer()
            finally:
                for signum, handler in previous_handlers.items():
                    signal.signal(signum, handler)


def main() -> int:
    from core.settings import configure_component_logging
    configure_component_logging("nuwax-motion-guard")
    logger = logging.getLogger("nuwax_robot_bridge.motion_guard")
    try:
        run()
    except ActuatorError as exc:
        logger.exception("运动控制启动失败: %s", exc.message, extra={"error_code": exc.code})
        return 1
    except Exception as exc:
        logger.exception("运动控制启动失败: %s", exc)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
