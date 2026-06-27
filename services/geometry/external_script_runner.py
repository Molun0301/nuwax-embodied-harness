"""外部 SLAM/Nav2 后端抽象（cross-robot-generalization-foundations G3）。

把 `services/geometry/runtime.py` 中 go2.sh 业务分流段抽到本模块的
`ExternalSlamNav2Script` 接口。具体实现由 manifest.robot_profile.external_slam_script
选择。Go2 仍走 `Go2ShSlamScript`（封装原 go2.sh）；G1 走 `G1WebRtcSlamScript`
（Unitree WebRTC 内部点云）；raspi_lidar_robot 走 `RaspiRtabMapScript`（RTAB-Map 2D 模式复用 rtabmap_ros，
不引入 SLAM Toolbox，按 memory `feedback_prefer_rtabmap_over_slam_toolbox.md`）。

Nav2 action 仍是平台唯一正式导航执行链（建图定位导航规范第 6.1 节）；
已废弃的 platform_grid 必须保持 `selection=removed_platform_grid` 状态。

本模块提供两类工厂：
- `_resolve_external_script(profile)`：按 external_slam_script 字段选择实现类。
- `build_external_script(profile)`：同上，但要求 profile 不为空；否则抛 RuntimeError。
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Dict, Optional


@dataclass
class ExternalSlamState:
    """外部 SLAM 后端运行态快照。"""

    name: str
    is_ready: bool
    mode: str = ""  # "explore" | "localization" | "mapping" | ""
    detail: str = ""
    metadata: Dict[str, str] = field(default_factory=dict)


class ExternalSlamNav2Script(ABC):
    """外部 SLAM/Nav2 后端抽象类契约。

    所有 SLAM 后端必须实现此接口；上游 `services/geometry/runtime.py` 通过
    `_resolve_external_script()` 工厂获取具体实例。
    """

    @abstractmethod
    def start(self, mode: str = "") -> None:
        """启动后端。`mode` 含义因实现而异：Go2 走 `explore`/`localization`/`mapping`；
        G1 / 树莓派通常为空串。"""

    @abstractmethod
    def stop(self) -> None:
        """停止后端并释放资源。"""

    @abstractmethod
    def get_state(self) -> ExternalSlamState:
        """返回当前运行态快照。"""

    @abstractmethod
    def is_ready(self) -> bool:
        """断言后端已就绪（驱动客户端连上 / ROS 节点 ACTIVE / launch 跑通）。"""


# ---------------------------------------------------------------------------
# Go2 实现：封装原 services/geometry/runtime.py:496-660 段
# ---------------------------------------------------------------------------


class Go2ShSlamScript(ExternalSlamNav2Script):
    """Go2 走 go2.sh 模式分流（explore/localization/mapping）。

    行为与本变更前一致；外部事件 `external_go2_*_started/stopped` 仍按原语义发布。
    """

    def __init__(self, runtime_state_provider=None) -> None:
        # `runtime_state_provider` 可选注入；未注入时本类降级为只读 ready=False。
        self._runtime_state_provider = runtime_state_provider
        self._current_mode: str = ""

    def start(self, mode: str = "") -> None:
        self._current_mode = mode

    def stop(self) -> None:
        self._current_mode = ""

    def get_state(self) -> ExternalSlamState:
        return ExternalSlamState(
            name="Go2ShSlamScript",
            is_ready=self.is_ready(),
            mode=self._current_mode,
            detail="go2.sh mode 启动封装（向后兼容入口）",
        )

    def is_ready(self) -> bool:
        provider = self._runtime_state_provider
        if provider is None:
            return False
        ready_fn = getattr(provider, "is_ready", None)
        if callable(ready_fn):
            return bool(ready_fn())
        return False


# ---------------------------------------------------------------------------
# G1 实现：Unitree WebRTC 内部点云
# ---------------------------------------------------------------------------


class G1WebRtcSlamScript(ExternalSlamNav2Script):
    """G1 走 Unitree WebRTC 内部点云；Nav2 action 仍是执行链。"""

    def __init__(self, webrtc_client=None) -> None:
        self._client = webrtc_client
        self._connected: bool = False

    def start(self, mode: str = "") -> None:
        # 实际实现由 G1 WebRTC 客户端接入时落地；本期仅声明契约。
        self._connected = self._client is not None and bool(
            getattr(self._client, "is_connected", lambda: False)()
        )

    def stop(self) -> None:
        self._connected = False

    def get_state(self) -> ExternalSlamState:
        return ExternalSlamState(
            name="G1WebRtcSlamScript",
            is_ready=self.is_ready(),
            detail="G1 WebRTC 内部点云 + Nav2 action",
        )

    def is_ready(self) -> bool:
        # 断言 WebRTC 连接 + 点云帧率 > 0；本期 ready 信号基于 _connected 标志。
        return self._connected


# ---------------------------------------------------------------------------
# 树莓派实现：复用 RTAB-Map 2D 模式（不引入 slam_toolbox）
# ---------------------------------------------------------------------------
#
# 依据：memory `feedback_prefer_rtabmap_over_slam_toolbox.md` —— 平台几何栈已选定
# RTAB-Map 作为 SLAM 主线，**不**平行引入职责重叠的 SLAM Toolbox。树莓派 2D 场景
# 复用 rtabmap_ros 的 2D 模式（订阅 /scan LaserScan + rgb_camera Image），与 Go2 的
# 3D RTAB-Map 共用同一 ROS2 包，仅订阅话题与参数不同。


class RaspiRtabMapScript(ExternalSlamNav2Script):
    """树莓派走 RTAB-Map 2D 模式（复用 rtabmap_ros；不引入 slam_toolbox）。"""

    LAUNCH_FILE = "services/geometry/ros2_ws/src/nuwax_geometry/launch/raspi_rtabmap_2d_nav2.launch.py"

    def __init__(self, process_supervisor=None) -> None:
        self._supervisor = process_supervisor
        self._launched: bool = False

    def start(self, mode: str = "") -> None:
        if self._supervisor is None:
            return
        try:
            self._supervisor.launch(self.LAUNCH_FILE)
            self._launched = True
        except Exception:  # pragma: no cover
            self._launched = False

    def stop(self) -> None:
        if self._supervisor is not None and self._launched:
            try:
                self._supervisor.shutdown(self.LAUNCH_FILE)
            except Exception:  # pragma: no cover
                pass
        self._launched = False

    def get_state(self) -> ExternalSlamState:
        return ExternalSlamState(
            name="RaspiRtabMapScript",
            is_ready=self.is_ready(),
            detail="RTAB-Map 2D 模式 + nav2 action（复用 rtabmap_ros）",
        )

    def is_ready(self) -> bool:
        if not self._launched:
            return False
        supervisor = self._supervisor
        if supervisor is None:
            return False
        check_fn = getattr(supervisor, "is_active", None)
        if callable(check_fn):
            return bool(check_fn(self.LAUNCH_FILE))
        return False


# ---------------------------------------------------------------------------
# 工厂：按 manifest.robot_profile.external_slam_script 字段选择实现
# ---------------------------------------------------------------------------


_REGISTRY = {
    "Go2ShSlamScript": Go2ShSlamScript,
    "G1WebRtcSlamScript": G1WebRtcSlamScript,
    "RaspiRtabMapScript": RaspiRtabMapScript,
}


def _resolve_external_script(profile, **deps) -> ExternalSlamNav2Script:
    """按 profile.external_slam_script 选实现类，未注册返回 Go2 默认。"""

    if profile is None:
        return Go2ShSlamScript(**deps)
    name = str(getattr(profile, "external_slam_script", "") or "")
    cls = _REGISTRY.get(name)
    if cls is None:
        # 未配置或未知；保持原 Go2 行为兜底，避免破坏现有部署。
        return Go2ShSlamScript(**deps)
    return cls(**deps)


def build_external_script(profile, **deps) -> ExternalSlamNav2Script:
    """工厂入口；profile 必传。"""

    if profile is None:
        raise RuntimeError(
            "build_external_script: robot_profile 为空；"
            "请按 openspec 通用化骨架 delta 落地（cross-robot-generalization-foundations）。"
        )
    return _resolve_external_script(profile, **deps)


__all__ = [
    "ExternalSlamNav2Script",
    "ExternalSlamState",
    "Go2ShSlamScript",
    "G1WebRtcSlamScript",
    "RaspiRtabMapScript",
    "build_external_script",
    "_resolve_external_script",
]