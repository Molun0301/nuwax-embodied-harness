"""平台侧相机-激光雷达自动标定服务。

【本包未接线、本期冻结】CalibrationService 全仓零实例化（已 grep 核实），feed_frame_pair /
update_motion_state 无调用方。实际生效的标定链路在 CalibrationProvider 主线
（drivers/go2/providers.py + services/geometry/runtime.py 的 _inject_static_extrinsics
静态外参边），不在本包。MI 在线标定【严禁直接接线】，理由见 05-P2-8（NMI 门控恒通过）/
05-P2-9（外参坐标系错位）。详见 config.py 头部冻结说明。
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Optional

from contracts.calibration import (
    CalibrationState,
    CalibrationStatus,
    ExtrinsicCalibration,
    SensorTopology,
)
from services.calibration.config import (
    get_calibration_persist_dir,
    is_calibration_enabled,
)
from services.calibration.data_collector import CalibrationDataCollector
from services.calibration.quality_monitor import CalibrationQualityMonitor
from services.calibration.transform_injector import (
    inject_calibration_to_frame_tree,
    load_persisted_calibration,
    persist_calibration,
)


LOGGER = logging.getLogger(__name__)


class CalibrationService:
    """平台侧相机-激光雷达自动标定服务。"""

    def __init__(
        self,
        *,
        localization_provider: Any = None,
        topology: Optional[SensorTopology] = None,
    ) -> None:
        self._enabled = is_calibration_enabled()
        # 治理边界（01-P1-3 / 08-P1-3）：物理参数（内参/外参初值）不再从 env 直读。
        # 未来接线时由 CalibrationProvider.get_sensor_topology() 注入 topology；
        # 未注入时为空拓扑，is_enabled 恒 False，与本包未接线现状一致。
        self._topology = topology if topology is not None else SensorTopology(
            robot_id="unknown", pairs=[]
        )
        self._localization_provider = localization_provider
        self._persist_dir = get_calibration_persist_dir()
        self._state = CalibrationState.UNCALIBRATED
        self._active_calibration: Optional[ExtrinsicCalibration] = None
        self._data_collector: Optional[CalibrationDataCollector] = None
        self._quality_monitor = CalibrationQualityMonitor()
        self._calibration_count = 0
        self._lock = threading.Lock()
        self._background_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

    @property
    def is_enabled(self) -> bool:
        return self._enabled and len(self._topology.pairs) > 0

    @property
    def state(self) -> CalibrationState:
        return self._state

    @property
    def active_calibration(self) -> Optional[ExtrinsicCalibration]:
        return self._active_calibration

    def get_status(self) -> CalibrationStatus:
        """获取当前标定状态。"""
        return CalibrationStatus(
            state=self._state,
            active_calibration=self._active_calibration,
            last_reprojection_error_px=(
                self._active_calibration.reprojection_error_px
                if self._active_calibration else None
            ),
            collected_frame_pairs=(
                self._data_collector.collected_count
                if self._data_collector else 0
            ),
            required_frame_pairs=(
                self._data_collector.required_count
                if self._data_collector else 0
            ),
            calibration_count=self._calibration_count,
            message=self._build_status_message(),
        )

    def start(self) -> None:
        """启动标定服务。"""
        if not self.is_enabled:
            LOGGER.info("标定服务未启用（NUWAX_CALIBRATION_AUTO_ENABLED=false 或无传感器对配置）。")
            return

        # 1. 尝试加载持久化标定
        persisted = load_persisted_calibration(self._persist_dir)
        if persisted is not None:
            self._active_calibration = persisted
            self._state = CalibrationState.CALIBRATED
            self._inject_current_calibration()
            LOGGER.info("已加载持久化标定: source=%s, error=%.2f px",
                       persisted.source, persisted.reprojection_error_px or -1)
        else:
            # 2. 使用注入 topology 的 initial_guess 作为临时外参
            self._inject_initial_guess()
            LOGGER.info("无持久化标定，使用注入 topology 的 initial_guess 作为临时外参。")

        # 3. 启动后台标定线程
        self._data_collector = CalibrationDataCollector()
        self._state = CalibrationState.COLLECTING
        self._stop_event.clear()
        self._background_thread = threading.Thread(
            target=self._background_loop,
            name="calibration-service",
            daemon=True,
        )
        self._background_thread.start()
        LOGGER.info("标定服务已启动，后台数据采集中。")

    def stop(self) -> None:
        """停止标定服务。"""
        self._stop_event.set()
        if self._background_thread is not None:
            self._background_thread.join(timeout=5.0)
            self._background_thread = None
        LOGGER.info("标定服务已停止。")

    def update_motion_state(self, linear_velocity_mps: float, angular_velocity_rps: float) -> None:
        """更新机器人运动状态（由外部定位服务调用）。"""
        if self._data_collector is not None:
            self._data_collector.update_motion_state(linear_velocity_mps, angular_velocity_rps)

    def feed_frame_pair(
        self,
        *,
        image_bytes: bytes,
        image_width_px: int,
        image_height_px: int,
        point_cloud_xyz: Any,  # np.ndarray
        point_cloud_frame_id: str,
        timestamp_ns: int,
    ) -> bool:
        """喂入一对帧数据（由外部数据管道调用）。"""
        if self._data_collector is None or not self.is_enabled:
            return False
        return self._data_collector.try_collect(
            image_bytes=image_bytes,
            image_width_px=image_width_px,
            image_height_px=image_height_px,
            point_cloud_xyz=point_cloud_xyz,
            point_cloud_frame_id=point_cloud_frame_id,
            timestamp_ns=timestamp_ns,
        )

    def _background_loop(self) -> None:
        """后台标定循环。"""
        while not self._stop_event.is_set():
            try:
                if self._data_collector is not None and self._data_collector.is_ready:
                    self._run_calibration()
                time.sleep(1.0)
            except Exception:
                LOGGER.exception("标定后台循环异常。")
                time.sleep(5.0)

    def _run_calibration(self) -> None:
        """执行一次标定。"""
        if not self._topology.pairs:
            return
        sensor_pair = self._topology.pairs[0]
        frame_pairs = self._data_collector.get_collected_pairs()
        if not frame_pairs:
            return

        with self._lock:
            self._state = CalibrationState.CALIBRATING

        LOGGER.info("开始 MI 标定，帧对数: %d", len(frame_pairs))
        try:
            from services.calibration.mi_calibrator import calibrate
            result = calibrate(frame_pairs, sensor_pair)
        except Exception:
            LOGGER.exception("MI 标定执行失败。")
            result = None

        with self._lock:
            if result is not None:
                if self._should_accept_calibration(result):
                    self._active_calibration = result
                    self._state = CalibrationState.CALIBRATED
                    self._calibration_count += 1
                    persist_calibration(result, self._persist_dir)
                    self._inject_current_calibration()
                    self._quality_monitor.acknowledge_recalibration()
                    LOGGER.info("标定成功: NMI=%.4f, reprojection=%.2f px",
                               result.nmi_score or 0, result.reprojection_error_px or 0)
                else:
                    LOGGER.warning("标定结果未通过安全门控，保留当前外参。")
                    self._state = (
                        CalibrationState.CALIBRATED if self._active_calibration
                        else CalibrationState.FAILED
                    )
            else:
                LOGGER.warning("标定失败，保留当前外参。")
                self._state = (
                    CalibrationState.FAILED if not self._active_calibration
                    else CalibrationState.CALIBRATED
                )

            # 重置采集器准备下一轮
            self._data_collector.reset()
            if self._state != CalibrationState.CALIBRATED:
                self._state = CalibrationState.COLLECTING

    def _should_accept_calibration(self, result: ExtrinsicCalibration) -> bool:
        """安全门控：检查标定结果是否应被接受。"""
        from services.calibration.config import get_calibration_nmi_threshold
        nmi_threshold = get_calibration_nmi_threshold()

        if result.nmi_score is not None and result.nmi_score < nmi_threshold:
            LOGGER.debug("NMI %.4f < 阈值 %.4f", result.nmi_score, nmi_threshold)
            return False

        if (
            self._active_calibration is not None
            and self._active_calibration.reprojection_error_px is not None
            and result.reprojection_error_px is not None
        ):
            if result.reprojection_error_px >= self._active_calibration.reprojection_error_px:
                LOGGER.debug(
                    "新标定误差 %.2f >= 当前 %.2f，拒绝。",
                    result.reprojection_error_px,
                    self._active_calibration.reprojection_error_px,
                )
                return False

        return True

    def _inject_current_calibration(self) -> None:
        """注入当前标定到 frame_tree。"""
        if self._active_calibration is None or self._localization_provider is None:
            return
        success = inject_calibration_to_frame_tree(self._active_calibration, self._localization_provider)
        if success:
            LOGGER.debug("标定已注入 frame_tree。")
        else:
            LOGGER.warning("标定注入 frame_tree 失败。")

    def _inject_initial_guess(self) -> None:
        """注入 topology 中的 initial_guess 作为临时外参。"""
        if not self._topology.pairs or self._localization_provider is None:
            return
        pair = self._topology.pairs[0]
        initial = ExtrinsicCalibration(
            parent_frame_id=pair.parent_frame_id,
            child_frame_id=pair.child_frame_id,
            translation=pair.initial_translation,
            rotation=pair.initial_rotation,
            source="initial_guess",
        )
        success = inject_calibration_to_frame_tree(initial, self._localization_provider)
        if success:
            self._active_calibration = initial
            LOGGER.debug("initial_guess 已注入 frame_tree。")

    def _build_status_message(self) -> str:
        """构建状态描述。"""
        if self._state == CalibrationState.UNCALIBRATED:
            return "等待标定数据。"
        elif self._state == CalibrationState.COLLECTING:
            count = self._data_collector.collected_count if self._data_collector else 0
            required = self._data_collector.required_count if self._data_collector else 0
            return f"采集中: {count}/{required} 帧对。"
        elif self._state == CalibrationState.CALIBRATING:
            return "正在执行 MI 标定..."
        elif self._state == CalibrationState.CALIBRATED:
            error = self._active_calibration.reprojection_error_px if self._active_calibration else None
            return f"标定完成，重投影误差: {error:.2f} px。" if error else "标定完成。"
        elif self._state == CalibrationState.FAILED:
            return "标定失败，使用回退外参。"
        return ""
