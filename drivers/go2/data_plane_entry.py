from __future__ import annotations

import logging
import signal
import threading
import time

from drivers.go2.data_plane import Go2DataPlaneRuntime
from drivers.go2.settings import load_go2_data_plane_config
from logging_utils import build_default_formatter


LOGGER = logging.getLogger("nuwax_robot_bridge.go2.data_plane_entry")


def main() -> int:
    """以独立进程启动 Go2 设备侧数据面。"""

    handler = logging.StreamHandler()
    handler.setFormatter(build_default_formatter())
    logging.basicConfig(
        level=logging.INFO,
        handlers=[handler],
        force=True,
    )
    runtime = Go2DataPlaneRuntime(load_go2_data_plane_config())
    stop_event = threading.Event()

    def _handle_signal(signum, frame) -> None:
        del signum, frame
        stop_event.set()

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    runtime.start()
    LOGGER.info("Go2 设备侧数据面已启动。")
    try:
        while not stop_event.is_set():
            time.sleep(0.5)
    finally:
        runtime.stop()
        LOGGER.info("Go2 设备侧数据面已停止。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
