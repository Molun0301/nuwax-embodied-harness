"""把公开设备载荷转成 NEH 现有输入类型，保持原始样本内容。"""
from __future__ import annotations
from datetime import datetime, timezone

import numpy as np

from contracts.calibration import SensorPair, SensorTopology
from contracts.geometry import Quaternion, Twist, Vector3
from contracts.image import CameraInfo, ImageFrame
from contracts.pointcloud import PointCloudFrame
from contracts.robot_state import JointState, RobotState, SafetyState
from contracts.sensors import CanonicalImuSample, CanonicalLidarFrame
from .clock import source_timestamp


def utc(stamp):
    return datetime.fromtimestamp(stamp / 1_000_000_000, timezone.utc)


class EventDecoder:
    def __init__(self, providers, device):
        self.providers, self.device = providers, device

    def accept(self, event):
        kind, payload = event['type'], event['payload']
        if kind in ('imu', 'lidar'):
            registration = payload['registration']
            common = dict(sensor_id=event['frame_id'] if kind == 'imu' else payload['sensor_id'],
                          source_sequence=event['sequence'], clock_domain='unix_ns',
                          clock_id=payload['clock_id'], frame_id=registration['frame_id'],
                          registration_state=registration['state'], registration_frame_id=registration['frame_id'],
                          calibration_ref=payload['calibration_ref'])
            if kind == 'imu':
                stamp = source_timestamp(payload, event['sim_time_ns'])
                sample = CanonicalImuSample(**common, timestamp_ns=stamp,
                                           accel_mps2=tuple(payload['accel_mps2']), gyro_rps=tuple(payload['gyro_rad_s']))
            else:
                sample = CanonicalLidarFrame(**common,
                    timestamp_ns=source_timestamp(payload, payload['scan_start_ns']),
                    xyz_f32_le=payload['xyz_f32_le'], point_count=payload['point_count'],
                    time_offset_ns_le=payload['time_offset_ns_u64_le'], scan_duration_ns=payload['scan_duration_ns'],
                    transform_provenance=payload['coordinates'])
            self.providers.publish_sensor_sample(sample)
        elif kind == 'image':
            self.image(event)
        elif kind == 'depth':
            self.depth(event)
        elif kind == 'status':
            feedback = self.device.get_motion_execution_feedback()
            twist = None if feedback is None else Twist(frame_id=self.device.frame_id,
                linear=Vector3(x=feedback.linear_x_mps, y=feedback.linear_y_mps, z=feedback.linear_z_mps),
                angular=Vector3(x=feedback.angular_x_rps, y=feedback.angular_y_rps, z=feedback.angular_z_rps))
            self.providers.publish_robot_state(RobotState(
                robot_id='unitree_sim', frame_id=self.device.frame_id,
                timestamp=utc(source_timestamp(payload, event['sim_time_ns'])), twist=twist,
                joints=[JointState(name=name, position_rad=position, velocity_rad_s=velocity)
                        for name, position, velocity in zip(payload['joint_names'], payload['joint_positions'],
                                                            payload['joint_velocities'])],
                safety=SafetyState(can_move=self.device.is_available() and not payload['fallen']),
                metadata={'source_timestamp_ns': source_timestamp(payload, event['sim_time_ns']), 'session': event['session'],
                          'epoch': event['epoch'], 'state': payload['state']}))

    def depth(self, event):
        payload = event['payload']
        height, width = payload['height'], payload['width']
        # 按原仿真桥的像素间距反投影每帧。无效像素不产生测距点。
        depth = np.frombuffer(payload['data'], dtype='<u2').reshape(height, width)[::4, ::4]
        rows, cols = np.nonzero(depth != payload['invalid_value'])
        distance = depth[rows, cols].astype(np.float32) * payload['depth_scale_m']
        intrinsic = payload['intrinsics']
        optical = np.column_stack(((4 * cols - intrinsic[0][2]) * distance / intrinsic[0][0],
                                   (4 * rows - intrinsic[1][2]) * distance / intrinsic[1][1], distance))
        calibration = payload['calibration']['imu_from_camera']
        w, x, y, z = calibration['rotation_wxyz']
        rotation = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ])
        tx, ty, tz = calibration['translation_m']
        points = np.asarray(optical @ rotation.T + [tx, ty, tz], dtype=np.float32)
        stamp = source_timestamp(payload, event['sim_time_ns'])
        self.providers.publish_point_cloud(PointCloudFrame(
            frame_id=self.device.frame_id, timestamp=utc(stamp), mapping_only=True,
            sensor_origin=Vector3(x=tx, y=ty, z=tz), point_count=len(points), points_xyz_array=points,
            metadata={'sensor_id': payload['sensor_id'], 'source_sequence': event['sequence'],
                      'source_timestamp_ns': stamp, 'clock_id': payload['clock_id'],
                      'calibration_ref': payload['calibration_ref']}))

    def image(self, event):
        payload = event['payload']
        identity = payload['camera_id']
        intrinsic = payload['intrinsics']
        stamp = source_timestamp(payload, event['sim_time_ns'])
        when = utc(stamp)
        frame = ImageFrame(camera_id=identity, frame_id=event['frame_id'], timestamp=when,
                           width_px=payload['width'], height_px=payload['height'],
                           encoding=payload['encoding'], data=payload['data'],
                           metadata={'source_timestamp_ns': stamp,
                                     'clock_id': payload['clock_id']})
        info = CameraInfo(camera_id=identity, frame_id=event['frame_id'], timestamp=when,
                          width_px=payload['width'], height_px=payload['height'],
                          fx=intrinsic[0][0], fy=intrinsic[1][1], cx=intrinsic[0][2], cy=intrinsic[1][2])
        calibration = payload['calibration']['imu_from_camera']
        w, x, y, z = calibration['rotation_wxyz']
        tx, ty, tz = calibration['translation_m']
        pair = SensorPair(parent_frame_id=self.device.frame_id, child_frame_id=event['frame_id'],
                          point_cloud_frame_id=self.device.frame_id,
                          initial_translation=Vector3(x=tx, y=ty, z=tz), initial_rotation=Quaternion(x=x, y=y, z=z, w=w),
                          camera_fx=info.fx, camera_fy=info.fy, camera_cx=info.cx, camera_cy=info.cy,
                          camera_width_px=info.width_px, camera_height_px=info.height_px)
        self.providers.set_camera_pair(pair)
        self.providers.publish_image(frame, info)
