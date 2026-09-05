from __future__ import annotations

import os
import logging
from core.config_values import runtime_value
from pathlib import Path
import signal
from typing import Optional
import threading

from .actor import GuardActor
from .constants import (
    DEFAULT_EPOCH_PATH,
    DEFAULT_LOCK_PATH,
    DEFAULT_PROVISION_PATH,
    DEFAULT_SOCKET_PATH,
)
from .core import MotionGuardCore
from .epoch import EpochStore, SingleInstanceLock
from .go2_actuator import ActuatorError, Go2Actuator
from .motion_control_actuator import MotionControlActuator
from .server import MotionGuardUnixServer


def _path_from_env(name: str, default: str) -> Path:
    value = runtime_value(name, default) if name == "NUWAX_MOTION_GUARD_SOCKET" else os.environ.get(name, default)
    return Path(value).expanduser().resolve()


def run() -> None:
    socket_path = _path_from_env("NUWAX_MOTION_GUARD_SOCKET", DEFAULT_SOCKET_PATH)
    lock_path = _path_from_env("NUWAX_MOTION_GUARD_LOCK", DEFAULT_LOCK_PATH)
    epoch_path = _path_from_env("NUWAX_MOTION_GUARD_EPOCH", DEFAULT_EPOCH_PATH)
    provision_path = _path_from_env(
        "NUWAX_MOTION_GUARD_PROVISIONED",
        DEFAULT_PROVISION_PATH,
    )
    stop_event = threading.Event()

    def request_stop(_signum: int, _frame: object) -> None:
        stop_event.set()

    previous_handlers = {
        signum: signal.signal(signum, request_stop)
        for signum in (signal.SIGINT, signal.SIGTERM)
    }
    lock = SingleInstanceLock(lock_path)
    server: Optional[MotionGuardUnixServer] = None
    try:
        lock.acquire()
        epoch_store = EpochStore(
            provision_path=provision_path,
            epoch_path=epoch_path,
        )
        standard_control = runtime_value("NUWAX_MOTION_GUARD_BACKEND", "").strip() == "motion_control"
        if standard_control:
            epoch_store.initialize_standard_control()
        epoch = epoch_store.advance_boot()
        if standard_control:
            actuator = MotionControlActuator.from_environment(
                actuator_symbols_sha256=epoch.actuator_symbols_sha256,
            )
        else:
            actuator = Go2Actuator.from_environment()
        core = MotionGuardCore(
            epoch=epoch,
            epoch_store=epoch_store,
            actuator=actuator,
        )
        if not core.startup_configured:
            raise RuntimeError(core.startup_reason)
        core.initialize_zero_fence()
        socket_gid = os.getegid()
        server = MotionGuardUnixServer(
            socket_path=socket_path,
            actor=GuardActor(core),
            socket_gid=socket_gid,
        )
        server.start()
        while not stop_event.wait(timeout=1.0):
            pass
    finally:
        if server is not None:
            server.stop()
        lock.release()
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
