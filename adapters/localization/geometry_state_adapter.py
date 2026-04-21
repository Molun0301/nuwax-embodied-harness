"""几何后端定位状态适配器。

将 RTAB-Map、Nav2 或 Go2 SDK 的定位状态映射到平台 LocalizationSessionStatus。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional

from pydantic import Field

from contracts.base import ContractModel, MetadataDict
from contracts.runtime_views import LocalizationSessionStatus


class BackendLocalizationState(ContractModel):
    """后端定位状态原始数据。"""

    backend_name: str = ""
    backend_type: str = "unknown"  # rtabmap, nav2, go2_sdk, robot_localization
    pose_available: bool = False
    tracking_state: str = "unknown"  # tracking, lost, relocalizing, initialization
    map_loaded: bool = False
    frame_id_match: bool = True
    quality_score: float = 0.0
    loop_closure_detected: bool = False
    relocalization_in_progress: bool = False
    last_error: Optional[str] = None
    metadata: MetadataDict = Field(default_factory=dict)


@dataclass
class GeometryLocalizationStateMapping:
    """几何后端状态到平台状态的映射规则。"""

    backend_type: str

    # 状态映射表
    state_mappings: Dict[str, LocalizationSessionStatus] = field(default_factory=dict)

    # 跟踪状态到平台状态的映射
    tracking_state_mappings: Dict[str, LocalizationSessionStatus] = field(default_factory=dict)

    def map_to_platform_status(
        self,
        backend_state: BackendLocalizationState,
    ) -> LocalizationSessionStatus:
        """将后端状态映射到平台 LocalizationSessionStatus。"""

        # 直接检查是否有明确的状态映射
        if backend_state.tracking_state in self.tracking_state_mappings:
            return self.tracking_state_mappings[backend_state.tracking_state]

        # 根据后端类型进行具体判断
        if self.backend_type == "rtabmap":
            return self._map_rtabmap_state(backend_state)
        elif self.backend_type == "go2_sdk":
            return self._map_go2_sdk_state(backend_state)
        elif self.backend_type in ("nav2", "robot_localization"):
            return self._map_generic_odom_state(backend_state)
        else:
            return self._map_generic_state(backend_state)

    def _map_rtabmap_state(
        self,
        backend_state: BackendLocalizationState,
    ) -> LocalizationSessionStatus:
        """映射 RTAB-Map 定位状态。

        RTAB-Map 状态映射：
        - initialization -> LOCALIZING
        - tracking -> READY (如果 pose_available)
        - lost -> STALE
        - relocalizing -> LOCALIZING
        - map_not_loaded -> FAILED
        """

        tracking = backend_state.tracking_state.strip().lower()

        # RTAB-Map 特定状态
        if tracking == "initialization":
            return LocalizationSessionStatus.LOCALIZING

        if tracking == "relocalizing":
            return LocalizationSessionStatus.LOCALIZING

        if tracking == "map_not_loaded":
            return LocalizationSessionStatus.FAILED

        if tracking == "lost":
            return LocalizationSessionStatus.STALE

        if tracking == "tracking" and backend_state.pose_available:
            return LocalizationSessionStatus.READY

        if tracking == "tracking" and not backend_state.pose_available:
            return LocalizationSessionStatus.LOCALIZING

        # 基于通用条件判断
        return self._map_generic_state(backend_state)

    def _map_go2_sdk_state(
        self,
        backend_state: BackendLocalizationState,
    ) -> LocalizationSessionStatus:
        """映射 Go2 SDK 定位状态。

        Go2 SDK 状态映射：
        - SDK 通常直接提供里程计，状态相对简单
        - pose_available -> READY
        - 无 pose -> LOCALIZING
        """

        if backend_state.pose_available and backend_state.map_loaded:
            return LocalizationSessionStatus.READY

        if backend_state.pose_available:
            # 有位姿但地图未加载，状态为 STALE
            return LocalizationSessionStatus.STALE

        return LocalizationSessionStatus.LOCALIZING

    def _map_generic_odom_state(
        self,
        backend_state: BackendLocalizationState,
    ) -> LocalizationSessionStatus:
        """映射通用里程计状态（Nav2/robot_localization）。"""

        if backend_state.pose_available:
            return LocalizationSessionStatus.READY

        return LocalizationSessionStatus.LOCALIZING

    def _map_generic_state(
        self,
        backend_state: BackendLocalizationState,
    ) -> LocalizationSessionStatus:
        """通用状态映射逻辑。"""

        if not backend_state.map_loaded:
            return LocalizationSessionStatus.LOCALIZING

        if not backend_state.pose_available:
            return LocalizationSessionStatus.LOCALIZING

        if not backend_state.frame_id_match:
            # 坐标系不匹配，进入 FAILED 状态
            return LocalizationSessionStatus.FAILED

        if backend_state.tracking_state == "lost":
            return LocalizationSessionStatus.STALE

        if backend_state.last_error:
            return LocalizationSessionStatus.STALE

        return LocalizationSessionStatus.READY


def create_rtabmap_state_mapping() -> GeometryLocalizationStateMapping:
    """创建 RTAB-Map 状态映射规则。"""

    return GeometryLocalizationStateMapping(
        backend_type="rtabmap",
        tracking_state_mappings={
            "initialization": LocalizationSessionStatus.LOCALIZING,
            "relocalizing": LocalizationSessionStatus.LOCALIZING,
            "tracking": LocalizationSessionStatus.READY,
            "lost": LocalizationSessionStatus.STALE,
            "map_not_loaded": LocalizationSessionStatus.FAILED,
        },
    )


def create_go2_sdk_state_mapping() -> GeometryLocalizationStateMapping:
    """创建 Go2 SDK 状态映射规则。"""

    return GeometryLocalizationStateMapping(
        backend_type="go2_sdk",
        tracking_state_mappings={
            "odom_available": LocalizationSessionStatus.READY,
            "odom_lost": LocalizationSessionStatus.STALE,
            "initializing": LocalizationSessionStatus.LOCALIZING,
        },
    )


def create_default_state_mapping() -> GeometryLocalizationStateMapping:
    """创建默认状态映射规则。"""

    return GeometryLocalizationStateMapping(
        backend_type="unknown",
        tracking_state_mappings={
            "ready": LocalizationSessionStatus.READY,
            "localizing": LocalizationSessionStatus.LOCALIZING,
            "stale": LocalizationSessionStatus.STALE,
            "failed": LocalizationSessionStatus.FAILED,
        },
    )


def map_backend_state_to_platform_status(
    backend_state: BackendLocalizationState,
) -> LocalizationSessionStatus:
    """将任意后端定位状态映射到平台状态。"""

    # 根据后端类型选择映射规则
    if backend_state.backend_type == "rtabmap":
        mapping = create_rtabmap_state_mapping()
    elif backend_state.backend_type == "go2_sdk":
        mapping = create_go2_sdk_state_mapping()
    else:
        mapping = create_default_state_mapping()

    return mapping.map_to_platform_status(backend_state)


def compute_tracking_quality(
    backend_state: BackendLocalizationState,
) -> float:
    """计算跟踪质量分数（0.0 - 1.0）。"""

    quality = 0.0

    # 基础位姿可用性
    if backend_state.pose_available:
        quality += 0.5

    # 地图加载状态
    if backend_state.map_loaded:
        quality += 0.2

    # 坐标系匹配
    if backend_state.frame_id_match:
        quality += 0.1

    # 回环检测（RTAB-Map）
    if backend_state.loop_closure_detected:
        quality += 0.1

    # 质量分数（如果有）
    quality += min(0.1, backend_state.quality_score)

    return min(1.0, quality)
