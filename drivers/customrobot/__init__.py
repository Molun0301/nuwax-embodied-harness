"""自定义机器人入口骨架导出。"""

from drivers.customrobot.assembly import create_customrobot_assembly
from drivers.customrobot.manifest import CUSTOMROBOT_MANIFEST

__all__ = [
    "CUSTOMROBOT_MANIFEST",
    "create_customrobot_assembly",
]
