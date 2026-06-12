"""services/calibration —— 相机-激光雷达自动标定（MI/NMI）算法包。

================================ 本包冻结说明 ================================
【本包未接线、本期冻结】CalibrationService 全仓零实例化，feed_frame_pair /
update_motion_state / quality_monitor.add_sample 均无调用方（已 grep 核实）。

实际生效的标定链路【不在本包】，而是 CalibrationProvider 主线：
  drivers/go2/defaults.py（内参 GO2_DEFAULT_CAMERA_INFO + 外参 GO2_DEFAULT_SENSOR_TOPOLOGY）
  → drivers/go2/providers.py（get_camera_info / get_sensor_topology）
  → 内参注入 SHM 帧 metadata（gateways/runtime.py）
  → 外参经 services/geometry/runtime.py 的 _inject_static_extrinsics 注入 frame_tree 静态边。

治理边界（01-P1-3 / 08-P1-3）：services 禁止从 env 直读相机物理参数；本包已去 env 化
（config.py 不再读 NUWAX_CAMERA_* / NUWAX_CALIBRATION_INITIAL_* 等物理参数），改由
CalibrationService.__init__ 的 topology 构造参数注入（未来来源 CalibrationProvider）。

MI 在线标定【严禁直接接线】，已知缺陷见 05-P2-8（NMI 门控恒通过）/ 05-P2-9（外参坐标系
错位、重投影误差全链未计算）/ 05-P2-10 / 05-P2-11。接线前必须先修上述缺陷。
============================================================================
"""
