from __future__ import annotations

from contracts.capabilities import CapabilityMatrix

G1_CAPABILITY_MATRIX = CapabilityMatrix(
    robot_model="unitree_g1",
    capabilities=[],
    metadata={
        "entrypoint": "drivers/g1",
        "implemented": False,
    },
)
