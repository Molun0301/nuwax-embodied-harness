"""Native point cloud source skeleton（Phase 5 fallback path）。

2026-07-07 Phase 5 新增：仅在 Unitree SDK PointCloud2 不可用时启用此路径。

硬约束
======

- 本文件**目前是占位骨架**，不是完成态；当前 Unitree SDK
  （``unitree_sdk2py``）已导出 ``sensor_msgs.msg.dds_.PointCloud2_``，
  因此 ``Go2DirectDdsConfig.point_cloud_enabled=True`` 时应当优先
  走 SDK 直订阅路径（``drivers/go2/data_plane.py``）。
- 若未来 SDK 移除了该 IDL（或 Go2 改用非 DDS 协议推送点云），
  应在本模块接入本仓 native source（例如订阅 webrtc/video frame 后
  做单目深度估计、写 ``PointCloudFrame`` / SHM ref），

  并把 ``Go2DirectDdsConfig.point_cloud_topic`` 改指 ``/go2/native/cloud`` 等。

NOT RUN 标注
============

实机/开发环境未对本模块执行任何 ``start()``；当前不提供
``runtime_unitree_sdk_missing`` smoke。

下一步
======

1. 在 ``RuntimeInputBridge`` 内新增 ``use_native_pointcloud_source: bool``
   配置；为 True 时 ``start_native_point_cloud_source()`` 取代 SDK 订阅。
2. 实现 ``subscribe_point_cloud(callback)`` 与
   ``get_latest_point_cloud()``，并把 frame 与 SHM 落点维护在
   ``runtime_data/go2/native_cloud/`` 目录。
3. ``native_pointcloud_source.get_status()`` 暴露
   ``latest_point_cloud_points`` / ``point_cloud_bridge_last_error``，
   与 SDK 路径对齐（Phase 5 §13.4 硬要求）。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional


@dataclass
class NativePointCloudSource:
    """Native point cloud source 骨架。NOT RUN，Unitree SDK 仍是 Phase 5 主路径。"""

    enabled: bool = False
    source_topic: str = ""
    last_error: Optional[str] = None
    last_error_at_monotonic: Optional[float] = None
    last_received_at_monotonic: Optional[float] = None
    latest_point_count: int = 0
    _callbacks: List[Callable[[Any], None]] = field(default_factory=list)

    def start(self) -> bool:
        """启动 native source。

        Returns
        -------
        bool
            ``True`` 表示启动成功。当前实现固定返回 ``False``，因为 native path
            尚未对接（Phase 5 仍以 SDK PointCloud2 为主路径）。
        """
        if not self.enabled:
            self.last_error = "native point cloud source disabled by config"
            return False
        self.last_error = (
            "native point cloud source skeleton only — "
            "implementation pending Phase 5 future lane; use Unitree SDK PathCloud2 path"
        )
        return False

    def stop(self) -> None:
        """停止 native source。"""
        self._callbacks.clear()

    def is_running(self) -> bool:
        return False

    def subscribe_point_cloud(self, callback: Callable[[Any], None]) -> Optional[int]:
        """订阅 native point cloud frames。NOT IMPLEMENTED。"""
        return None

    def get_latest_point_cloud(self) -> Optional[Any]:
        """获取最新点云。NOT IMPLEMENTED。"""
        return None

    def get_status(self) -> Dict[str, Any]:
        """暴露给 ``Go2DirectDdsBridge.get_status()`` 桥接用的字段。"""
        return {
            "native_pointcloud_enabled": self.enabled,
            "native_pointcloud_started": False,
            "native_pointcloud_topic": self.source_topic,
            "native_pointcloud_last_error": self.last_error,
            "native_pointcloud_latest_point_count": self.latest_point_count,
        }
