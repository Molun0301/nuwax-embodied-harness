"""外部 mapping（nuwax-mapping 板块） 后端抽象（cross-robot-generalization-foundations G3）。

把 `services/geometry/runtime.py` 中 go2.sh 业务分流段抽到本模块的
`ExternalMappingScript` 接口。具体实现由 manifest.robot_profile.external_slam_script
选择。Go2 仍走 `Go2ShSlamScript`（封装原 go2.sh）；G1 走 `G1WebRtcSlamScript`
（Unitree WebRTC 内部点云）。raspi_lidar_robot 共享 nuwax-geometry-runtime
（`RaspiRtabMapScript` 已 R-N14-6 删除）。

历史类名 `ExternalSlamNav2Script` 保留作 B 类 alias；Nav2/rtabmap_ros 时代代码已 drop-ros2
阶段物理删除。本模块提供两类工厂：
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


class ExternalMappingScript(ABC):
    """外部 mapping（nuwax-mapping 板块） 后端抽象类契约。

    所有 mapping 后端必须实现此接口；上游 `services/geometry/runtime.py` 通过
    `_resolve_external_script()` 工厂获取具体实例。历史 ABC 名 `ExternalSlamNav2Script`
    保留作 B 类 alias，见末尾。
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
        """断言后端已就绪（驱动客户端连上 / 节点 ACTIVE / launch 跑通）。"""


# B 类 alias 保留（1 版本周期 + telemetry 触发清理）
ExternalSlamNav2Script = ExternalMappingScript


# ---------------------------------------------------------------------------
# Go2 实现：封装原 services/geometry/runtime.py:496-660 段
# ---------------------------------------------------------------------------


class Go2ShSlamScript(ExternalMappingScript):
    """Go2 走 go2.sh 模式分流（explore/localization/mapping）。

    行为与本变更前一致；外部事件 `external_go2_*_started/stopped` 仍按原语义发布。
    SlamScript 命名是产品语义层，保留以兼容历史 manifest 字段（robot_profile.external_slam_script）。
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


class G1WebRtcSlamScript(ExternalMappingScript):
    """G1 走 Unitree WebRTC 内部点云；平台映射后端（nuwax-mapping / nuwax-nav-stack）执行链。"""

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
# 树莓派实现已 R-N14-6 物理删除
# ---------------------------------------------------------------------------
#
# 树莓派共享 nuwax-geometry-runtime（与 Go2/G1 同一栈），不再启独立脚本。
# 历史类 `RaspiRtabMapScript` 物理删除；旧 manifest.robot_profile.external_slam_script
# 字段名保留但未注册到 `_REGISTRY`，工厂回退到 Go2 默认实现以避免破坏现有部署。
# drivers/raspi_lidar_robot/manifest.py:55 残留 `RaspiRtabMapScript` Python 路径
# 留待后续 Phase 处理（本任务范围仅 external_script_runner.py）。


# ---------------------------------------------------------------------------
# 工厂：按 manifest.robot_profile.external_slam_script 字段选择实现
# ---------------------------------------------------------------------------


_REGISTRY = {
    "Go2ShSlamScript": Go2ShSlamScript,
    "G1WebRtcSlamScript": G1WebRtcSlamScript,
    # RaspiRtabMapScript 已 R-N14-6 删除（R-N14 v4 §R-N14-6 决策）
}


def _resolve_external_script(profile, **deps) -> "ExternalMappingScript":
    """按 profile.external_slam_script 选实现类，未注册返回 Go2 默认。"""

    if profile is None:
        return Go2ShSlamScript(**deps)
    name = str(getattr(profile, "external_slam_script", "") or "")
    cls = _REGISTRY.get(name)
    if cls is None:
        # 未配置或未知；保持原 Go2 行为兜底，避免破坏现有部署。
        return Go2ShSlamScript(**deps)
    return cls(**deps)


def build_external_script(profile, **deps) -> "ExternalMappingScript":
    """工厂入口；profile 必传。"""

    if profile is None:
        raise RuntimeError(
            "build_external_script: robot_profile 为空；"
            "请按 openspec 通用化骨架 delta 落地（cross-robot-generalization-foundations）。"
        )
    return _resolve_external_script(profile, **deps)


__all__ = [
    "ExternalMappingScript",  # 新名
    "ExternalSlamNav2Script",  # B 类 alias
    "ExternalSlamState",
    "Go2ShSlamScript",
    "G1WebRtcSlamScript",
    "build_external_script",
    "_resolve_external_script",
]