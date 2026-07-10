"""闭环检测状态（LoopClosureState）。

用途：
- 持久化 mapping（nuwax-mapping 板块，RTAB-Map 算法 Rust 移植）闭环检测信号，
  供 `_metadata_indicates_exploration_done` 第 8 条判据使用；
- 主路径 LCM `/explore/loop_closure` channel（由 nuwax-geometry-runtime 推送）；
  HIL fallback 路径走 SQLite DB 查询（SQLite RTAB-Map 兼容格式）；
- 严格遵循 services 层不订阅 ROS topic、不引入 services 层硬依赖 ROS2 Python 客户端库
  的边界（参照 memory `nuwax-layering-governance-boundaries.md`）。

约定：
- `confidence = min(loop_closure_count / threshold, 1.0)`，单调上升但不放大；
- DB SQLite 查询使用只读连接 + 超时保护，避免阻塞 GO2 实时建图；
- Py3.8.10 兼容：无 PEP 604 语法、无 `dict[str, ...]` 类型注解。
"""

from __future__ import annotations

import logging
import sqlite3
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional


LOGGER = logging.getLogger("nuwax_robot_bridge.geometry.loop_closure_state")

# RTAB-Map Link 表 type 字段语义（参见 rtabmap/core/Link.h）：
#   0 = Neighbor / 1 = Global scan-matching closure / 2 = User loop closure / 3 = Virtual / 4 = Recovery
_LOOP_CLOSURE_LINK_TYPE = 2

# 默认 DB 查询窗口：最近 100 条 Link 记录中 type=2 的数量。
# 与 design.md §决策 D2 / spec exploration-completion-criteria 对齐。
DEFAULT_LOOKBACK_LIMIT = 100

# SQLite 只读连接超时（秒），避免主线程被 DB lock 阻塞。
_DEFAULT_SQLITE_TIMEOUT_SEC = 0.5


@dataclass
class LoopClosureState:
    """闭环检测状态快照。

    字段：
    - `loop_closure_count`: 已观测到的累计闭环次数（单调递增）。
    - `confidence`: 置信度 `min(count / threshold, 1.0)`，范围 [0, 1]。
    - `last_check_at`: 最后一次状态更新时间戳（UTC ISO 8601）。
    - `source`: 信号来源，区分主路径（topic）与 fallback（db_sqlite）。
    - `threshold`: 触发阈值（默认 3，与 NAV_LOOP_CLOSURE_THRESHOLD 默认一致）。
    """

    loop_closure_count: int = 0
    confidence: float = 0.0
    last_check_at: Optional[str] = None
    source: str = "none"
    threshold: int = 3

    def to_dict(self) -> dict:
        return {
            "loop_closure_count": int(self.loop_closure_count),
            "confidence": float(self.confidence),
            "last_check_at": self.last_check_at,
            "source": str(self.source or "none"),
            "threshold": int(self.threshold),
        }

    def is_triggered(self) -> bool:
        """是否达到闭环阈值。"""

        return int(self.loop_closure_count) >= int(self.threshold)


def make_initial_loop_closure_state(*, threshold: int = 3) -> LoopClosureState:
    """构造初始状态（count=0, confidence=0, source=none）。"""

    return LoopClosureState(
        loop_closure_count=0,
        confidence=0.0,
        last_check_at=None,
        source="none",
        threshold=int(threshold) if int(threshold) > 0 else 3,
    )


def update_loop_closure_state_from_count(
    state: LoopClosureState,
    *,
    new_count: int,
    source: str = "topic",
    threshold: Optional[int] = None,
) -> LoopClosureState:
    """根据新观测闭环次数构造更新后的 LoopClosureState。

    `confidence = min(new_count / threshold, 1.0)`，保留单调性但不放大（即使
    `new_count` 远大于 `threshold`，confidence 不会 >1）。
    """

    final_threshold = int(threshold) if threshold is not None else int(state.threshold)
    if final_threshold <= 0:
        final_threshold = 3
    normalized_count = max(0, int(new_count))
    confidence = float(normalized_count) / float(final_threshold)
    if confidence > 1.0:
        confidence = 1.0
    elif confidence < 0.0:
        confidence = 0.0
    return LoopClosureState(
        loop_closure_count=normalized_count,
        confidence=confidence,
        last_check_at=_utc_now_iso(),
        source=str(source or state.source or "none"),
        threshold=final_threshold,
    )


def read_loop_closure_from_db(
    db_path: Path,
    *,
    threshold: int = 3,
    lookback_limit: int = DEFAULT_LOOKBACK_LIMIT,
    timeout_sec: float = _DEFAULT_SQLITE_TIMEOUT_SEC,
) -> int:
    """查询 RTAB-Map SQLite DB 中最近 N 条 Link 记录里 type=2（loop closure）的数量。

    参数：
    - `db_path`: RTAB-Map 数据库路径
      `runtime_data/maps/<map_name>/versions/mapver_NNNNNN/rtabmap.db`。
    - `threshold`: 触发阈值（>=该值即视为闭环，仅用于语义记录，不影响返回值）。
    - `lookback_limit`: 最近 N 条 Link 记录中扫描的窗口大小，默认 100。
    - `timeout_sec`: SQLite 连接超时秒数，默认 0.5s 避免阻塞主循环。

    返回：
    - `int` >= 0；DB 不存在 / 不可读 / 表缺失 / 查询异常时一律返回 0。

    行为约定：
    - 仅读取，不修改 DB；
    - 使用 immutable=1 标志打开只读连接（URI 模式）；
    - 对 SQLite Error / OperationalError 兜底返回 0 + warning 日志。
    """

    del threshold  # 保留参数签名以便上层语义化调用，但本函数不参与阈值判定
    try:
        resolved = Path(db_path).expanduser()
    except Exception:
        LOGGER.debug("loop_closure DB 路径解析失败：%s", db_path, exc_info=True)
        return 0
    if not resolved.is_file():
        LOGGER.debug("loop_closure DB 文件不存在：%s", resolved)
        return 0
    db_uri = "file:%s?mode=ro" % resolved.as_posix()
    try:
        conn = sqlite3.connect(db_uri, uri=True, timeout=float(timeout_sec))
    except Exception:
        LOGGER.warning(
            "loop_closure_state DB 只读连接失败 path=%s，闭环判据降级",
            resolved,
            exc_info=True,
        )
        return 0
    try:
        cursor = conn.cursor()
        cursor.execute(
            "SELECT COUNT(*) FROM (SELECT type FROM Link ORDER BY id DESC LIMIT ?) WHERE type = ?",
            (int(lookback_limit), _LOOP_CLOSURE_LINK_TYPE),
        )
        row = cursor.fetchone()
        if row is None or row[0] is None:
            return 0
        return int(row[0])
    except sqlite3.OperationalError as exc:
        LOGGER.warning(
            "loop_closure_state DB 查询 OperationalError path=%s err=%s",
            resolved,
            exc,
        )
        return 0
    except sqlite3.DatabaseError as exc:
        LOGGER.warning(
            "loop_closure_state DB 查询 DatabaseError path=%s err=%s",
            resolved,
            exc,
        )
        return 0
    except Exception:
        LOGGER.warning(
            "loop_closure_state DB 查询未知异常 path=%s", resolved, exc_info=True
        )
        return 0
    finally:
        try:
            conn.close()
        except Exception:
            LOGGER.debug("loop_closure DB 连接关闭失败 path=%s", resolved, exc_info=True)


def _utc_now_iso() -> str:
    """UTC ISO 8601 字符串（带 Z 后缀），用于 last_check_at 字段。"""

    return datetime.now(tz=timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")


class LoopClosureStateBuffer:
    """线程安全的 LoopClosureState 持有者，供 observer / runtime 之间共享。

    设计动机：
    - `_compute_loop_closure` 在 observer 线程更新；
    - `_metadata_indicates_exploration_done` 在 runtime 主循环读取；
    - 两者并发访问，需原子快照。
    """

    def __init__(self, *, threshold: int = 3) -> None:
        self._lock = threading.RLock()
        self._state = make_initial_loop_closure_state(threshold=threshold)

    def get(self) -> LoopClosureState:
        with self._lock:
            # 返回浅拷贝，避免外部修改污染内部状态
            return LoopClosureState(
                loop_closure_count=self._state.loop_closure_count,
                confidence=self._state.confidence,
                last_check_at=self._state.last_check_at,
                source=self._state.source,
                threshold=self._state.threshold,
            )

    def update(self, state: LoopClosureState) -> None:
        with self._lock:
            self._state = LoopClosureState(
                loop_closure_count=int(state.loop_closure_count),
                confidence=float(state.confidence),
                last_check_at=state.last_check_at,
                source=str(state.source or "none"),
                threshold=int(state.threshold) if int(state.threshold) > 0 else 3,
            )

    def reset(self, *, threshold: Optional[int] = None) -> None:
        """重置状态（探索会话开始时调用，避免缓存串味）。"""

        with self._lock:
            final_threshold = (
                int(threshold)
                if threshold is not None and int(threshold) > 0
                else int(self._state.threshold)
            )
            self._state = make_initial_loop_closure_state(threshold=final_threshold)


def now_monotonic() -> float:
    """薄包装：暴露给上层做节流，便于测试 patch。"""

    return time.monotonic()