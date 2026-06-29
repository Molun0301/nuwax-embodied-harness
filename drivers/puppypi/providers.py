from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional

from contracts.geometry import Twist, Vector3
from contracts.robot_state import RobotControlMode, RobotState, SafetyState
from providers.motion import MotionControl

if TYPE_CHECKING:
    from drivers.puppypi.assembly import MotionProxyClient, PuppyPiAssembly


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


@dataclass
class PuppyPiProviderBundle(MotionControl):
    """薄驱动：仅转发运动指令到容器 motion_proxy。"""

    assembly: "PuppyPiAssembly"
    provider_name: str = "puppypi_motion_proxy"
    provider_version: str = "0.1.0"

    @property
    def _motion_client(self) -> "MotionProxyClient":
        return self.assembly.motion_proxy_client

    def is_available(self) -> bool:
        if self._motion_client.last_health_ok:
            return True
        return self._motion_client.health_check().ok

    def send_twist(self, twist: Twist) -> None:
        x_mps = _clamp(float(twist.linear.x), -0.08, 0.08)
        yaw_rate = _clamp(float(twist.angular.z), -0.4, 0.4)
        self._motion_client.post_twist(x_mps=x_mps, yaw_rate=yaw_rate)
        self.assembly.record_twist(x_mps=x_mps, yaw_rate=yaw_rate)

    def stop_motion(self) -> None:
        self._motion_client.post_twist(x_mps=0.0, yaw_rate=0.0)
        self.assembly.record_twist(x_mps=0.0, yaw_rate=0.0)

    def get_robot_state(self) -> RobotState:
        return RobotState(
            robot_id=str(self.assembly.defaults.parameters.get("robot_id", "puppypi")),
            frame_id=self.assembly.defaults.frame_ids.get("base", "world/puppypi/base"),
            mode=RobotControlMode.HIGH_LEVEL if self.is_available() else RobotControlMode.UNKNOWN,
            twist=self.assembly.last_twist,
            safety=SafetyState(can_move=self.is_available()),
            metadata={
                "motion_proxy_url": self._motion_client.base_url,
                "motion_proxy_ok": self._motion_client.last_health_ok,
                "last_twist": self.assembly.last_twist_metadata(),
            },
        )

    def capture_image(self, camera_id: Optional[str] = None):
        raise NotImplementedError("PuppyPi Phase 1 不直接抓图；视频由容器 video_proxy 主动推流。")
