"""Telemetry 调度器(R-N14 §3.6)。
6h 周期调用 collector._write_legacy_alias_counters(),落地 legacy_alias.json。
启动入口:gateways/app.py 装配阶段启动 daemon thread。
"""
from __future__ import annotations

import logging
import threading
from typing import Optional

LOGGER = logging.getLogger("nuwax_robot_bridge.telemetry.dispatcher")

DEFAULT_INTERVAL_SEC = 6 * 60 * 60  # 6h
DEFAULT_OUTPUT_PATH = "runtime_data/.legacy_alias_counters.json"


class LegacyAliasCounterDispatcher:
    """每 6h 调用一次 collector._write_legacy_alias_counters()。"""

    def __init__(
        self,
        *,
        interval_sec: int = DEFAULT_INTERVAL_SEC,
        output_path: Optional[str] = None,
    ) -> None:
        self._interval_sec = max(60, int(interval_sec))
        self._output_path = output_path or DEFAULT_OUTPUT_PATH
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            LOGGER.debug("LegacyAliasCounterDispatcher 已启动,跳过重复 start。")
            return
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._loop,
            name="legacy-alias-counter-dispatcher",
            daemon=True,
        )
        self._thread.start()
        LOGGER.info("LegacyAliasCounterDispatcher 已启动 interval_sec=%d", self._interval_sec)

    def stop(self, timeout_sec: float = 2.0) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=float(timeout_sec))
            self._thread = None
        LOGGER.info("LegacyAliasCounterDispatcher 已停止。")

    def _loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                self._tick()
            except Exception:
                LOGGER.exception("LegacyAliasCounterDispatcher._tick 异常,继续下一周期。")
            # wait_for 返回 True 表示事件 set,False 表示超时
            if self._stop_event.wait(timeout=float(self._interval_sec)):
                return

    def _tick(self) -> None:
        try:
            from services.telemetry.collector import SystemMetricsCollector  # type: ignore
        except Exception as exc:
            LOGGER.warning("导入 SystemMetricsCollector 失败,跳过本周期 tick: %s", exc)
            return
        try:
            collector = SystemMetricsCollector()
            target = collector._write_legacy_alias_counters(output_path=self._output_path)
            LOGGER.debug("dispatcher._tick 写完成 path=%s", target)
        except Exception:
            LOGGER.exception("dispatcher._tick 失败。")


__all__ = ["LegacyAliasCounterDispatcher", "DEFAULT_INTERVAL_SEC", "DEFAULT_OUTPUT_PATH"]