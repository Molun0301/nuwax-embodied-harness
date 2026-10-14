from __future__ import annotations

from contracts.capabilities import CapabilityMatrix


CUSTOMROBOT_CAPABILITY_MATRIX = CapabilityMatrix(
    robot_model="customrobot",
    capabilities=[],
    metadata={
        "entrypoint": "drivers/customrobot",
        "implemented": True,
        "input_mode": "standard_frames",
    },
)
