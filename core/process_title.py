"""Best-effort Linux process title helpers.

`exec -a` controls argv[0] when processes are launched by our shell scripts.
This helper complements it for Python entrypoints started directly, setting the
short Linux `comm` name via prctl when available. Linux limits `comm` to 15
bytes plus NUL, so telemetry display_name remains the Web-facing authority.
"""

from __future__ import annotations

import ctypes
import logging
import sys

LOGGER = logging.getLogger("nuwax_robot_bridge.process_title")
_PR_SET_NAME = 15
_LINUX_COMM_MAX_BYTES = 15


def set_process_title(title: str) -> bool:
    """Set Linux process `comm` name best-effort; return whether it succeeded."""

    if not title or not sys.platform.startswith("linux"):
        return False
    try:
        libc = ctypes.CDLL(None)
        raw = str(title).encode("utf-8")[:_LINUX_COMM_MAX_BYTES]
        buf = ctypes.create_string_buffer(raw)
        result = libc.prctl(_PR_SET_NAME, ctypes.byref(buf), 0, 0, 0)
        return result == 0
    except Exception:
        LOGGER.debug("设置进程短名称失败 title=%s", title, exc_info=True)
        return False
