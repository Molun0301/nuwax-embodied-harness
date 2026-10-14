"""验证 Python facade 与 Rust nuwax-codec 共用的 NWS1 v1 黄金字节。

执行：
    PYTHONDONTWRITEBYTECODE=1 python3 \
        scripts/diagnostics/nws1_cross_language_golden.py
"""

from __future__ import annotations

import importlib.util
import struct
import sys
from pathlib import Path
from typing import Any, Dict

_ROOT = Path(__file__).resolve().parents[2]
_WIRE_PATH = _ROOT / "services" / "geometry" / "sensor_wire.py"
_FIXTURE_PATH = (
    _ROOT
    / "services"
    / "nuwax-workspace"
    / "crates"
    / "nuwax-codec"
    / "reference"
    / "nws1_v1_cross_language.hex"
)


def _load_sensor_wire() -> Any:
    spec = importlib.util.spec_from_file_location("nws1_sensor_wire", str(_WIRE_PATH))
    if spec is None or spec.loader is None:
        raise RuntimeError("无法加载 NWS1 Python facade")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


wire = _load_sensor_wire()


def _identity(
    sensor_id: str,
    source_sequence: int,
    timestamp_ns: int,
    ownership: Any,
) -> Any:
    return wire.SensorIdentity(
        schema_version=1,
        sensor_id=sensor_id,
        source_sequence=source_sequence,
        timestamp_ns=timestamp_ns,
        clock_domain="unix_ns",
        frame_id="go2_base_link",
        registration_state="body",
        registration_frame_id="go2_base_link",
        payload_ownership=ownership,
        calibration_ref="go2-urdf-base-v1",
    )


def build_frames() -> Dict[str, Any]:
    inline_cloud = wire.CloudFrame(
        identity=_identity(
            "go2_l2",
            42,
            1787030400123456789,
            wire.PayloadOwnership.INLINE,
        ),
        point_count=3,
        scan_duration_ns=120000,
        payload=wire.InlineCloudPayload(
            xyz_f32_le=struct.pack(
                "<9f",
                1.0,
                -2.0,
                0.5,
                0.0,
                3.25,
                -4.5,
                10.0,
                0.125,
                -0.25,
            ),
            intensity_f32_le=struct.pack("<3f", 0.0, 42.5, 255.0),
            ring_u16_le=struct.pack("<3H", 0, 17, 65535),
            time_offset_ns_le=struct.pack("<3Q", 0, 60000, 120000),
        ),
    )
    shm_cloud = wire.CloudFrame(
        identity=_identity(
            "go2_l2",
            43,
            1787030400123456790,
            wire.PayloadOwnership.SHM,
        ),
        point_count=2,
        scan_duration_ns=80000,
        payload=wire.ExternalCloudPayload(
            ownership=wire.PayloadOwnership.SHM,
            payload_ref="shm://nws1-golden-cloud",
            payload_layout=(
                "xyz_f32_le",
                "intensity_f32_le",
                "ring_u16_le",
                "time_offset_ns_le",
            ),
            payload_size_bytes=52,
            lease_epoch=7,
            expiry_ns=1787030401000000000,
        ),
    )
    imu_before = wire.ImuFrame(
        identity=_identity(
            "go2_imu",
            9001,
            1787030400123446789,
            wire.PayloadOwnership.INLINE,
        ),
        accel_mps2=(0.0, -9.80665, 0.125),
        gyro_rps=(0.01, -0.02, 1.5),
    )
    imu_after = wire.ImuFrame(
        identity=_identity(
            "go2_imu",
            9002,
            1787030400123576789,
            wire.PayloadOwnership.INLINE,
        ),
        accel_mps2=(0.25, -9.75, 0.0),
        gyro_rps=(-0.03, 0.04, 1.25),
    )
    return {
        "inline_cloud": inline_cloud,
        "shm_cloud": shm_cloud,
        "imu_before": imu_before,
        "imu_after": imu_after,
    }


def load_goldens() -> Dict[str, bytes]:
    goldens: Dict[str, bytes] = {}
    for raw_line in _FIXTURE_PATH.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        name, encoded = line.split()
        goldens[name] = bytes.fromhex(encoded)
    return goldens


def run() -> Dict[str, int]:
    frames = build_frames()
    goldens = load_goldens()
    if set(frames) != set(goldens):
        raise AssertionError("NWS1 黄金样本名称集合不一致")
    lengths: Dict[str, int] = {}
    for name, frame in frames.items():
        encoded = wire.encode_sensor_frame(frame)
        if encoded != goldens[name]:
            raise AssertionError("%s 的 Python 编码字节发生漂移" % name)
        if wire.decode_sensor_frame(goldens[name]) != frame:
            raise AssertionError("%s 的 Python 解码结果发生漂移" % name)
        lengths[name] = len(encoded)
    return lengths


if __name__ == "__main__":
    try:
        result = run()
    except Exception as exc:
        print("[FAIL] nws1_cross_language_golden:", exc)
        raise
    print("[PASS] nws1_cross_language_golden", result)
