"""标定质量在线监控：重投影误差采样 + 漂移检测 + 连续超阈触发重标定。

【本包未接线、本期冻结】CalibrationQualityMonitor.add_sample 无调用方（已 grep 核实）。
生效链路见 CalibrationProvider（drivers/go2/providers.py + services/geometry/runtime.py
静态外参边）。MI 在线标定【严禁直接接线】，理由见 05-P2-8/P2-9（calibrate 从不计算
reprojection_error_px，本监控体系实际无数据可用）。详见 config.py 头部冻结说明。
"""

from __future__ import annotations

import logging
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, List

import numpy as np

from services.calibration.config import get_calibration_reprojection_threshold


LOGGER = logging.getLogger(__name__)


@dataclass
class ReprojectionSample:
    """一次重投影误差采样。"""
    timestamp: float
    mean_error_px: float
    median_error_px: float
    max_error_px: float
    point_count: int


class CalibrationQualityMonitor:
    """标定质量在线监控。"""

    def __init__(
        self,
        *,
        threshold_px: float = 0.0,
        drift_window_size: int = 10,
        recalibration_trigger_count: int = 10,
        history_limit: int = 100,
    ) -> None:
        self._threshold_px = threshold_px or get_calibration_reprojection_threshold()
        self._drift_window_size = drift_window_size
        self._recalibration_trigger_count = recalibration_trigger_count
        self._history: Deque[ReprojectionSample] = deque(maxlen=max(1, history_limit))
        self._consecutive_failures: int = 0
        self._recalibration_requested: bool = False
        self._total_samples: int = 0
        self._last_quality_state: str = "unknown"

    @property
    def is_recalibration_requested(self) -> bool:
        """是否需要触发重标定。"""
        return self._recalibration_requested

    @property
    def last_quality_state(self) -> str:
        """最近的质量状态：good / degraded / failed / unknown。"""
        return self._last_quality_state

    @property
    def consecutive_failures(self) -> int:
        return self._consecutive_failures

    @property
    def total_samples(self) -> int:
        return self._total_samples

    def add_sample(
        self,
        *,
        mean_error_px: float,
        median_error_px: float,
        max_error_px: float,
        point_count: int,
    ) -> str:
        """添加一次重投影误差采样，返回质量状态。"""
        sample = ReprojectionSample(
            timestamp=time.time(),
            mean_error_px=mean_error_px,
            median_error_px=median_error_px,
            max_error_px=max_error_px,
            point_count=point_count,
        )
        self._history.append(sample)
        self._total_samples += 1

        # 判断质量
        if median_error_px <= self._threshold_px:
            self._consecutive_failures = 0
            self._last_quality_state = "good"
        elif median_error_px <= self._threshold_px * 2:
            self._consecutive_failures += 1
            self._last_quality_state = "degraded"
        else:
            self._consecutive_failures += 1
            self._last_quality_state = "failed"

        # 触发重标定
        if self._consecutive_failures >= self._recalibration_trigger_count:
            if not self._recalibration_requested:
                LOGGER.warning(
                    "标定质量连续 %d 次不合格（阈值 %.1f px），触发重标定请求。",
                    self._consecutive_failures,
                    self._threshold_px,
                )
                self._recalibration_requested = True

        return self._last_quality_state

    def acknowledge_recalibration(self) -> None:
        """确认重标定请求已被处理。"""
        self._recalibration_requested = False
        self._consecutive_failures = 0

    def get_statistics(self) -> Dict[str, object]:
        """获取监控统计信息。"""
        if not self._history:
            return {
                "total_samples": 0,
                "quality_state": "unknown",
                "consecutive_failures": 0,
                "recalibration_requested": False,
            }
        recent = list(self._history)[-self._drift_window_size:]
        recent_median_errors = [s.median_error_px for s in recent]
        return {
            "total_samples": self._total_samples,
            "quality_state": self._last_quality_state,
            "consecutive_failures": self._consecutive_failures,
            "recalibration_requested": self._recalibration_requested,
            "threshold_px": self._threshold_px,
            "recent_window_size": len(recent),
            "recent_mean_error_px": float(np.mean(recent_median_errors)) if recent_median_errors else 0.0,
            "recent_max_error_px": float(np.max(recent_median_errors)) if recent_median_errors else 0.0,
            "recent_min_error_px": float(np.min(recent_median_errors)) if recent_median_errors else 0.0,
            "trend": self._compute_trend(recent_median_errors),
        }

    def _compute_trend(self, values: List[float]) -> str:
        """计算误差趋势。"""
        if len(values) < 3:
            return "insufficient_data"
        first_half = np.mean(values[: len(values) // 2])
        second_half = np.mean(values[len(values) // 2 :])
        diff = second_half - first_half
        if diff > self._threshold_px * 0.2:
            return "increasing"
        elif diff < -self._threshold_px * 0.2:
            return "decreasing"
        return "stable"

    def reset(self) -> None:
        """重置监控状态。"""
        self._history.clear()
        self._consecutive_failures = 0
        self._recalibration_requested = False
        self._total_samples = 0
        self._last_quality_state = "unknown"
