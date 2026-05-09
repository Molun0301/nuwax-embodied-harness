#!/usr/bin/env python3
# -*- coding:utf-8 -*-

"""
Go2 低级别控制器。
使用 LowCmd/LowState 进行单关节控制。
"""

import logging
import sys
import time

from settings import APP_CONFIG

from drivers.go2.control.joint_config import (
    DEFAULT_KD,
    DEFAULT_KP,
    DEFAULT_MAX_VELOCITY,
    JOINT_NAMES,
    LegID,
    check_safety_limit,
    clamp_position,
)

logger = logging.getLogger(__name__)

SDK_PATH = APP_CONFIG.dds.sdk_path
if SDK_PATH not in sys.path:
    sys.path.insert(0, SDK_PATH)

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_, unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread


class LowLevelController:
    """低级别控制器。"""

    def __init__(self, iface=None):
        self.iface = iface
        self.is_initialized = False
        self.is_running = False
        self.mode = "high"
        self.low_cmd = None
        self.low_state = None
        self.cmd_thread = None
        self.crc = CRC()
        self.default_kp = DEFAULT_KP
        self.default_kd = DEFAULT_KD

    def init(self):
        """初始化低级别控制。"""

        if self.is_initialized:
            logger.warning("低级别控制器已初始化")
            return True

        logger.info("初始化DDS通道 (低级别)...")
        if self.iface:
            ChannelFactoryInitialize(0, self.iface)
        else:
            ChannelFactoryInitialize(0)

        self._init_low_cmd()
        self.cmd_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.cmd_publisher.Init()
        self.state_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.state_subscriber.Init(self._on_low_state, 10)

        logger.info("等待LowState数据...")
        timeout = 5.0
        start = time.time()
        while self.low_state is None and (time.time() - start) < timeout:
            time.sleep(0.1)

        if self.low_state is None:
            logger.error("未收到LowState数据")
            return False

        self.is_initialized = True
        logger.info("低级别控制器初始化成功")
        return True

    def _init_low_cmd(self):
        """初始化LowCmd结构。"""

        self.low_cmd = unitree_go_msg_dds__LowCmd_()
        self.low_cmd.head[0] = 0xFE
        self.low_cmd.head[1] = 0xEF
        self.low_cmd.level_flag = 0xFF
        self.low_cmd.gpio = 0

        for index in range(20):
            self.low_cmd.motor_cmd[index].mode = 0x01
            self.low_cmd.motor_cmd[index].q = 0
            self.low_cmd.motor_cmd[index].kp = 0
            self.low_cmd.motor_cmd[index].dq = 0
            self.low_cmd.motor_cmd[index].kd = 0
            self.low_cmd.motor_cmd[index].tau = 0

    def _on_low_state(self, msg: LowState_):
        """LowState回调。"""

        self.low_state = msg

    def start(self):
        """启动控制循环。"""

        if not self.is_initialized:
            logger.error("控制器未初始化")
            return False
        if self.is_running:
            logger.warning("控制循环已在运行")
            return True

        self.is_running = True
        self.cmd_thread = RecurrentThread(
            interval=APP_CONFIG.low_level.control_interval_sec,
            target=self._control_loop,
            name="lowcmd_writer",
        )
        self.cmd_thread.Start()
        logger.info("低级别控制循环已启动")
        return True

    def stop(self):
        """停止控制循环。"""

        self.is_running = False
        if self.cmd_thread:
            self.cmd_thread.Stop()
        logger.info("低级别控制循环已停止")

    def stop_motion(self):
        """清零低层命令并停止发布循环。"""

        if self.low_cmd is not None:
            for index in range(20):
                self.low_cmd.motor_cmd[index].q = 0
                self.low_cmd.motor_cmd[index].kp = 0
                self.low_cmd.motor_cmd[index].dq = 0
                self.low_cmd.motor_cmd[index].kd = 0
                self.low_cmd.motor_cmd[index].tau = 0
            if self.is_initialized:
                self._control_loop()
        self.stop()

    def _control_loop(self):
        """控制循环。"""

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.cmd_publisher.Write(self.low_cmd)

    def set_joint_position(self, joint_name: str, position: float, kp: float = None, kd: float = None) -> bool:
        """设置单关节位置。"""

        if not self.is_initialized:
            logger.error("控制器未初始化")
            return False

        try:
            joint_id = LegID[joint_name]
        except KeyError:
            logger.error("未知关节: %s", joint_name)
            return False

        if not check_safety_limit(joint_name, position):
            position = clamp_position(joint_name, position)
            logger.warning("位置超出限制，已限制到: %s", position)

        self.low_cmd.motor_cmd[joint_id].q = position
        self.low_cmd.motor_cmd[joint_id].dq = 0
        self.low_cmd.motor_cmd[joint_id].kp = kp if kp is not None else self.default_kp
        self.low_cmd.motor_cmd[joint_id].kd = kd if kd is not None else self.default_kd
        self.low_cmd.motor_cmd[joint_id].tau = 0
        return True

    def set_joints_position(self, joint_positions: dict, kp: float = None, kd: float = None) -> bool:
        """设置多关节位置。"""

        for joint_name, position in joint_positions.items():
            if not self.set_joint_position(joint_name, position, kp, kd):
                return False
        return True

    def set_joint_velocity(self, joint_name: str, velocity: float, kd: float = None) -> bool:
        """设置单关节速度。"""

        if not self.is_initialized:
            logger.error("控制器未初始化")
            return False

        try:
            joint_id = LegID[joint_name]
        except KeyError:
            logger.error("未知关节: %s", joint_name)
            return False

        if abs(velocity) > DEFAULT_MAX_VELOCITY:
            velocity = DEFAULT_MAX_VELOCITY if velocity > 0 else -DEFAULT_MAX_VELOCITY
            logger.warning("速度超出限制，已限制到: %s", velocity)

        self.low_cmd.motor_cmd[joint_id].q = 0
        self.low_cmd.motor_cmd[joint_id].dq = velocity
        self.low_cmd.motor_cmd[joint_id].kp = 0
        self.low_cmd.motor_cmd[joint_id].kd = kd if kd is not None else self.default_kd
        self.low_cmd.motor_cmd[joint_id].tau = 0
        return True

    def set_joint_torque(self, joint_name: str, torque: float) -> bool:
        """设置单关节力矩。"""

        if not self.is_initialized:
            logger.error("控制器未初始化")
            return False

        try:
            joint_id = LegID[joint_name]
        except KeyError:
            logger.error("未知关节: %s", joint_name)
            return False

        self.low_cmd.motor_cmd[joint_id].q = 0
        self.low_cmd.motor_cmd[joint_id].dq = 0
        self.low_cmd.motor_cmd[joint_id].kp = 0
        self.low_cmd.motor_cmd[joint_id].kd = 0
        self.low_cmd.motor_cmd[joint_id].tau = torque
        return True

    def get_joint_state(self, joint_name: str = None) -> dict:
        """获取关节状态。"""

        if self.low_state is None:
            return {}

        if joint_name:
            try:
                joint_id = LegID[joint_name]
            except KeyError:
                return {}
            motor = self.low_state.motor_state[joint_id]
            return {
                "name": joint_name,
                "position": motor.q,
                "velocity": motor.dq,
                "torque": motor.tau,
            }

        states = []
        for index, name in enumerate(JOINT_NAMES):
            motor = self.low_state.motor_state[index]
            states.append(
                {
                    "name": name,
                    "position": motor.q,
                    "velocity": motor.dq,
                    "torque": motor.tau,
                }
            )
        return {"joints": states}

    def get_imu_state(self) -> dict:
        """获取低层 IMU 状态。"""

        if self.low_state is None:
            return {}

        imu_state = getattr(self.low_state, "imu_state", None)
        if imu_state is None:
            return {}

        quaternion = getattr(imu_state, "quaternion", None)
        gyroscope = getattr(imu_state, "gyroscope", None)
        accelerometer = getattr(imu_state, "accelerometer", None)
        raw_quaternion = list(quaternion) if quaternion is not None else [1.0, 0.0, 0.0, 0.0]
        while len(raw_quaternion) < 4:
            raw_quaternion.append(0.0)
        w, x, y, z = raw_quaternion[:4]
        return {
            "quaternion": [x, y, z, w],
            "gyroscope": list(gyroscope) if gyroscope is not None else [0.0, 0.0, 0.0],
            "accelerometer": list(accelerometer) if accelerometer is not None else [0.0, 0.0, 0.0],
        }
