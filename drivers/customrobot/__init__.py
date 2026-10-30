"""通用标准输入机器人装配导出。"""

from drivers.customrobot.assembly import CustomRobotAssembly, create_customrobot_assembly
from drivers.customrobot.manifest import CUSTOMROBOT_MANIFEST
from drivers.customrobot.latest_frame import LatestFrameProcessor

__all__ = [
    "CUSTOMROBOT_MANIFEST",
    "CustomRobotAssembly",
    "LatestFrameProcessor",
    "create_customrobot_assembly",
]
