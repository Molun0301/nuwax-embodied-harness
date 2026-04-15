from __future__ import annotations

from dataclasses import dataclass


@dataclass
class CustomRobotProviderBundle:
    """自定义机器人入口骨架占位。"""

    provider_name: str = "customrobot_placeholder"
    provider_version: str = "0.1.0"
