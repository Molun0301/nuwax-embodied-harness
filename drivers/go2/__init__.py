"""Go2 机器人入口导出。"""

from drivers.go2.assembly import Go2RobotAssembly, create_go2_assembly
from drivers.go2.capabilities import GO2_CAPABILITY_DESCRIPTORS, GO2_CAPABILITY_MATRIX
from drivers.go2.manifest import GO2_MANIFEST, build_go2_manifest

__all__ = [
    "GO2_CAPABILITY_DESCRIPTORS",
    "GO2_CAPABILITY_MATRIX",
    "GO2_MANIFEST",
    "Go2RobotAssembly",
    "build_go2_manifest",
    "create_go2_assembly",
]
