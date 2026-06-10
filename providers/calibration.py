from __future__ import annotations

from typing import Optional, Protocol, runtime_checkable

from contracts.calibration import SensorTopology
from providers.base import BaseProvider


@runtime_checkable
class CalibrationProvider(BaseProvider, Protocol):
    """传感器标定提供器接口。

    机器人装配通过该接口暴露 camera↔lidar 等传感器对的**静态外参初值**与对应
    相机内参（来自端侧 driver 的物理参数，经 contracts 契约暴露）。平台侧只依赖
    本接口与 contracts，不 import 任何具体 driver，保证换机器人时平台侧零改动。

    外参为静态值（机器人物理装配属性），由平台几何运行时在装配期取一次、缓存于
    RosBridgeProvider 实例，每帧 frame_tree 重建后合并为静态边。本接口不承担运行时
    高频更新；后续在线 MI 精修经同一接口提供更新值（接口不变，实现增量）。
    """

    def get_sensor_topology(self) -> Optional[SensorTopology]:
        """返回该机器人的传感器标定拓扑（含各 SensorPair 的外参初值与相机内参）。

        无可用标定拓扑时返回 None（平台侧据此报错，不兜底）。
        """
