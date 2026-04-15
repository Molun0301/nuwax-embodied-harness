"""G1 机器人入口骨架导出。"""

from drivers.g1.assembly import create_g1_assembly
from drivers.g1.manifest import G1_MANIFEST

__all__ = [
    "G1_MANIFEST",
    "create_g1_assembly",
]
