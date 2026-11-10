"""安装时把设备声明的输入标定与已配置外形写入普通方案。"""
from __future__ import annotations
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from .device import capabilities

IDENTITY = {'translation_m': [0.0, 0.0, 0.0], 'rotation_row_major': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]}


def configure(values, root, home):
    model = values['unitree_sim.model']
    backend = capabilities(values['unitree_sim.base_url'], model)
    sensors, limits = backend['sensors'], backend['command_limits']
    calibration = sensors['calibration']
    reference = sensors['registration_frame_id']
    # 源已在采样时把点/惯性转换到同一 IMU 参考体；不得再次应用原始安装变换。
    lidar = dict(IDENTITY, ray_origin_in_input_m=calibration['lidar']['input_from_lidar']['translation_m'])
    values.update({'robot.base_frame': reference, 'robot.lidar_frame': reference,
                   'geometry.lio_body_from_sensor': json.dumps(lidar, separators=(',', ':')),
                   'geometry.lio_body_from_imu': json.dumps(IDENTITY, separators=(',', ':')),
                   'motion_guard.backend': 'motion_control',
                   'motion_guard.control_factory': 'drivers.unitree_sim.device:create_motion_control',
                   'robot.actuator_max_linear_mps': math.hypot(limits['vx_mps'], limits['vy_mps']),
                   'robot.actuator_max_yaw_rps': float(limits['wz_rps']),
                   'robot.actuator_motion_model': 'omnidirectional',
                   'perception.stream_camera_id': sensors['image']['camera_ids'][0]})
    if not values['geometry.pointlio_options']:
        values['geometry.pointlio_options'] = json.dumps(
            {'ivox_grid_resolution': 0.2, 'ivox_nearby_type': 'nearby18', 'match_s': 900.0},
            separators=(',', ':'))
    if model == 'go2':
        values['robot.model_base_link'] = 'imu'
        origin = calibration['imu']['translation_m']
        values['robot.actuator_max_linear_mps'] += math.hypot(origin[0], origin[1]) * limits['wz_rps']
    elif not values['robot.model_path']:
        raise ValueError('unitree_sim 的 G1 外形须由 robot.model_path 指定。')
    else:
        model_path = Path(values['robot.model_path'])
        if not model_path.is_absolute():
            model_path = Path(home) / model_path
        values['robot.model_path'] = str(model_path)
        tree = ET.parse(model_path)
        joint = tree.find("joint[@name='%s']" % calibration['imu']['source_joint'])
        values['robot.model_base_link'] = joint.find('child').attrib['link']
    for name in ('length_m', 'width_m', 'height_m', 'center_x_m', 'center_y_m', 'center_z_m'):
        values['robot.' + name] = ''
    return values
