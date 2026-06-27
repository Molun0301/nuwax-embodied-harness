"""services/calibration —— 相机-激光雷达自动标定（MI/NMI）算法包。

================================ 本包冻结说明 ================================
【本包未接线、本期冻结】CalibrationService 全仓零实例化，feed_frame_pair /
update_motion_state / quality_monitor.add_sample 均无调用方（已 grep 核实）。

实际生效的标定链路【不在本包】，而是 CalibrationProvider 主线：
  assembly.manifest.sensor_topology_ref（cross-robot-generalization-foundations G4 新增）
    → drivers/<robot>/defaults.py:<ROBOT>_DEFAULT_SENSOR_TOPOLOGY（Go2 为 GO2_DEFAULT_SENSOR_TOPOLOGY）
    → drivers/<robot>/providers.py（get_camera_info / get_sensor_topology）
  → 内参注入 SHM 帧 metadata（gateways/runtime.py）
  → 外参经 services/geometry/runtime.py 的 _inject_static_extrinsics 注入 frame_tree 静态边。

治理边界（01-P1-3 / 08-P1-3）：services 禁止从 env 直读相机物理参数；本包已去 env 化
（config.py 不再读 NUWAX_CAMERA_* / NUWAX_CALIBRATION_INITIAL_* 等物理参数），改由
CalibrationService.__init__ 的 topology 构造参数注入（来源 CalibrationProvider，
provider 通过 assembly.manifest.sensor_topology_ref 解析）。

MI 在线标定【严禁直接接线】，已知缺陷见 05-P2-8（NMI 门控恒通过）/ 05-P2-9（外参坐标系
错位、重投影误差全链未计算）/ 05-P2-10 / 05-P2-11。接线前必须先修上述缺陷。
============================================================================
"""

from __future__ import annotations

import importlib
from typing import Any, Optional


def _load_topology(assembly: Any) -> Optional[Any]:
    """按 assembly.manifest.sensor_topology_ref 解析路径获取 SensorTopology。

    路径格式：`<dotted.module.path>:<ATTR_NAME>`，例如
    `drivers.go2.defaults:GO2_DEFAULT_SENSOR_TOPOLOGY`（⚠️ 历史示例；实际通过 `_load_topology(assembly)` 从 `manifest.sensor_topology_ref` 解析，跨机器人变更记录：cross-robot-generalization-foundations G4）。
    空字符串或 None 表示该机器人不需要标定（如 raspi_lidar_robot 2D 单线激光），
    函数返回 None。

    跨机器人变更记录：cross-robot-generalization-foundations G4。
    """

    manifest = getattr(assembly, "manifest", None)
    if manifest is None:
        return None
    ref = str(getattr(manifest, "sensor_topology_ref", "") or "").strip()
    if not ref:
        return None
    if ":" not in ref:
        raise ValueError(
            f"sensor_topology_ref 格式非法: {ref!r}；应为 '<module>:<ATTR>'"
        )
    module_path, attr_name = ref.split(":", 1)
    module_path = module_path.strip()
    attr_name = attr_name.strip()
    if not module_path or not attr_name:
        raise ValueError(f"sensor_topology_ref 格式非法: {ref!r}")
    module = importlib.import_module(module_path)
    return getattr(module, attr_name)


__all__ = ["_load_topology"]
