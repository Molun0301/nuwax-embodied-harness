"""规范二维极坐标转等角输入；无返回与坏测量不会被合并成障碍。"""
from __future__ import annotations
from dataclasses import dataclass
import math
from typing import Optional
import numpy as np
from contracts.sensors import CanonicalLaserScan


@dataclass(frozen=True)
class UniformPlanarScan:
    source: CanonicalLaserScan
    angle_min_rad: float
    angle_increment_rad: float
    ranges_f64: np.ndarray
    beam_states_u8: np.ndarray
    time_offsets_ns: Optional[np.ndarray]
    input_beams: int
    retained_hits: int


def normalize_planar_scan(scan: CanonicalLaserScan, *, angle_min_rad: float,
                          angle_increment_rad: float, beam_count: int) -> UniformPlanarScan:
    """按实际角度分格，同格保留最近回波与其真实时间。"""
    if not 1 <= beam_count <= 32768 or not math.isfinite(angle_min_rad) or not math.isfinite(angle_increment_rad) or angle_increment_rad <= 0:
        raise ValueError("目标等角扫描参数非法")
    if angle_increment_rad * (beam_count - 1) > 2 * math.pi + 1e-5:
        raise ValueError("目标角度跨度超过一圈")
    ranges = np.frombuffer(scan.ranges_f32_le, dtype="<f4")
    states = np.frombuffer(scan.beam_states_u8, dtype=np.uint8)
    if scan.angular_layout == "uniform":
        angles = scan.angle_min_rad + np.arange(scan.beam_count) * scan.angle_increment_rad
    else:
        angles = np.frombuffer(scan.angles_f32_le, dtype="<f4").astype(np.float64)
    relative = np.remainder(angles - angle_min_rad, 2 * math.pi)
    # 浮点误差下的负零附近角度归入起始束；部分视场不能把其它方向强塞进边缘。
    relative[np.isclose(relative, 2 * math.pi, atol=1e-6, rtol=0)] = 0.0
    indices = np.floor(relative / angle_increment_rad + 0.5).astype(np.int64)
    full_circle = math.isclose(angle_increment_rad * beam_count, 2 * math.pi, abs_tol=1e-5)
    if full_circle:
        indices %= beam_count
    output = np.full(beam_count, np.nan, dtype=np.float64)
    output_states = np.zeros(beam_count, dtype=np.uint8)
    offsets = None if scan.time_offsets_ns_le is None else np.frombuffer(scan.time_offsets_ns_le, dtype="<u8")
    output_offsets = None if offsets is None else np.zeros(beam_count, dtype=np.uint64)
    for i, target in enumerate(indices):
        if target < 0 or target >= beam_count or states[i] == 0:
            continue
        # 同方向存在有效近回波时，无返回不能擦除该回波。
        if states[i] == 1:
            if output_states[target] == 1 and output[target] <= ranges[i]:
                continue
            output[target] = ranges[i]
            output_states[target] = 1
        elif output_states[target] != 1:
            output[target] = np.inf
            output_states[target] = 2
        else:
            continue
        if output_offsets is not None:
            output_offsets[target] = offsets[i]
    for array in (output, output_states, output_offsets):
        if array is not None:
            array.setflags(write=False)
    return UniformPlanarScan(scan, angle_min_rad, angle_increment_rad, output,
                             output_states, output_offsets, scan.beam_count,
                             int(np.count_nonzero(output_states == 1)))
