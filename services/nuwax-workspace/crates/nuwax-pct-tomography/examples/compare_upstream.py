#!/usr/bin/env python3
"""执行 PCT-SCAN 原始 NumPy/SciPy 路径，保存上游层数与几何数值对照。"""
from __future__ import annotations

import argparse
import gc
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import scipy
from scipy import ndimage


GEOMETRY_DTYPE = np.dtype([("x", "<i4"), ("y", "<i4"), ("ground", "<f8"), ("ceiling", "<f8")])


def read_pcd(path):
    """按 PCD 头读取字段及 POINTS 数量，保留源坐标 f32。"""
    header = {}
    with path.open("rb") as source:
        while True:
            line = source.readline()
            if not line:
                raise ValueError("PCD 头不完整")
            words = line.decode("ascii").strip().split()
            if words and not words[0].startswith("#"):
                header[words[0]] = words[1:]
            if words and words[0] == "DATA":
                break
        if header["DATA"] != ["binary"]:
            raise ValueError("示例仅支持上游 Building/Plaza 的 binary PCD")
        fields = []
        for name, size, kind, count in zip(
            header["FIELDS"], header["SIZE"], header["TYPE"], header["COUNT"]
        ):
            dtype = {"F": "f", "U": "u", "I": "i"}[kind] + size
            fields.append((name, "<" + dtype, (int(count),)))
        points = np.frombuffer(source.read(), dtype=np.dtype(fields), count=int(header["POINTS"][0]))
        return np.column_stack([points[axis][:, 0] for axis in ("x", "y", "z")]).astype(np.float32)


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def run_original_cpu(module, points, center, dims, count, cfg, bridge_grid_xy=None):
    """调用未修改的函数；仅在返回事件取出原始切片及被选层号作为证据。"""
    captured = {}
    function = module.cpu_point2map

    def observe(frame, event, _arg):
        if frame.f_code is function.__code__:
            frame.f_trace_lines = False
            if event == "return":
                for key in ("layers_g", "layers_c", "idx_simp"):
                    captured[key] = frame.f_locals[key]
            return observe
        return None

    previous_trace = sys.gettrace()
    previous_round = module._round_like_c
    try:
        if bridge_grid_xy is not None:
            # 仅适配格索引，源点不变，后续NumPy/SciPy仍执行上游原函数。
            module._round_like_c = lambda _relative, _np: bridge_grid_xy
        sys.settrace(observe)
        result = function(points, center, *dims, count, 0.5, cfg, np, ndimage)
    finally:
        sys.settrace(previous_trace)
        module._round_like_c = previous_round
    captured["timings_ms"] = result[-1]
    return captured


def summarize_reference(reference):
    ground = reference["layers_g"]
    ceiling = reference["layers_c"]
    distinct = [0]
    for index in range(1, len(ground)):
        previous = distinct[-1]
        if not (np.array_equal(ground[previous], ground[index]) and np.array_equal(ceiling[previous], ceiling[index])):
            distinct.append(index)
    return {
        "initial_slices": len(ground),
        "geometry_distinct_global_slices": len(distinct),
        "geometry_distinct_slice_indices": distinct,
        "upstream_cost_simplified_layers": len(reference["idx_simp"]),
        "upstream_cost_selected_indices": reference["idx_simp"],
        "cpu_timings_ms": reference["timings_ms"],
    }


def sparse_geometry(reference, dims):
    rows = []
    for ground, ceiling in zip(reference["layers_g"], reference["layers_c"]):
        x, y = np.nonzero(ground > -1e6)
        cells = np.empty(len(x), dtype=GEOMETRY_DTYPE)
        cells["x"] = x - dims[0] // 2
        cells["y"] = y - dims[1] // 2
        cells["ground"] = ground[x, y]
        cells["ceiling"] = np.where(ceiling[x, y] < 1e6, ceiling[x, y], np.inf)
        rows.append(cells)
    return np.unique(np.concatenate(rows))


def scalar_height(value, ground):
    if (ground and value <= -1e6) or (not ground and value >= 1e6) or not np.isfinite(value):
        return None
    return float(value)


def compare_raw_slices(reference, actual, dims, affected_columns):
    affected = np.zeros(tuple(dims), dtype=bool)
    affected[affected_columns[:, 0] + dims[0] // 2, affected_columns[:, 1] + dims[1] // 2] = True
    ground = reference["layers_g"]
    ceiling = reference["layers_c"]
    total_ground = same_ground = stable_ground = stable_same_ground = 0
    total_ceiling = same_ceiling = stable_ceiling = stable_same_ceiling = 0
    max_ground_error = max_ceiling_error = 0.0
    missing_native = missing_reference = 0
    differences = []
    for index in range(len(ground)):
        native_ground = np.full(tuple(dims), -1e6, dtype=np.float32)
        native_ceiling = np.full(tuple(dims), 1e6, dtype=np.float32)
        height = np.float32(0.5 + index * 0.5)
        chosen = (actual["ground"] <= height) & (height < actual["ceiling"])
        rows = actual[chosen]
        x, y = rows["x"] + dims[0] // 2, rows["y"] + dims[1] // 2
        native_ground[x, y] = rows["ground"]
        native_ceiling[x, y] = np.minimum(rows["ceiling"], 1e6)
        valid_reference = ground[index] > -1e6
        valid_native = native_ground > -1e6
        valid = valid_reference | valid_native
        both = valid_reference & valid_native
        equal_ground = ground[index] == native_ground
        equal_ceiling = ceiling[index] == native_ceiling
        total_ground += int(valid.sum())
        same_ground += int((valid & equal_ground).sum())
        stable_ground += int((valid & ~affected).sum())
        stable_same_ground += int((valid & ~affected & equal_ground).sum())
        total_ceiling += int(both.sum())
        same_ceiling += int((both & equal_ceiling).sum())
        stable_ceiling += int((both & ~affected).sum())
        stable_same_ceiling += int((both & ~affected & equal_ceiling).sum())
        missing_native += int((valid_reference & ~valid_native).sum())
        missing_reference += int((~valid_reference & valid_native).sum())
        if np.any(both):
            max_ground_error = max(max_ground_error, float(np.max(np.abs(ground[index][both] - native_ground[both]))))
        finite_ceiling = both & (ceiling[index] < 1e6) & (native_ceiling < 1e6)
        if np.any(finite_ceiling):
            max_ceiling_error = max(max_ceiling_error, float(np.max(np.abs(ceiling[index][finite_ceiling] - native_ceiling[finite_ceiling]))))
        different = valid & (~equal_ground | (both & ~equal_ceiling))
        for x, y in np.column_stack(np.nonzero(different))[:max(0, 12 - len(differences))]:
            differences.append({"slice_index": index, "x": int(x - dims[0] // 2), "y": int(y - dims[1] // 2),
                                "reference_ground": scalar_height(ground[index, x, y], True),
                                "native_ground": scalar_height(native_ground[x, y], True),
                                "reference_ceiling": scalar_height(ceiling[index, x, y], False),
                                "native_ceiling": scalar_height(native_ceiling[x, y], False)})
    if stable_ground != stable_same_ground or stable_ceiling != stable_same_ceiling:
        raise AssertionError("边界分配以外的原始PCD稳定格存在高度差异")
    return {"ground_comparisons": total_ground, "ground_exact_matches": same_ground,
            "ground_exact_ratio": same_ground / total_ground,
            "ceiling_comparisons_both_have_ground": total_ceiling, "ceiling_exact_matches": same_ceiling,
            "ceiling_exact_ratio": same_ceiling / total_ceiling,
            "stable_ground_comparisons": stable_ground, "stable_ground_exact_matches": stable_same_ground,
            "stable_ceiling_comparisons": stable_ceiling, "stable_ceiling_exact_matches": stable_same_ceiling,
            "stable_ground_and_ceiling_exact_ratio": 1.0,
            "finite_ground_max_abs_error_m_at_boundary": max_ground_error,
            "finite_ceiling_max_abs_error_m_at_boundary": max_ceiling_error,
            "reference_only_ground_layer_cells": missing_native, "native_only_ground_layer_cells": missing_reference,
            "boundary_difference_samples": differences}


def compare_scene(args, module, scene, filename, slope):
    pcd = args.pct_scan_root / "assets" / scene / "pcd" / filename
    points = read_pcd(pcd)
    points = points[np.isfinite(points).all(axis=1)]
    binary = args.scratch / (scene + ".xyz.bin")
    points.astype("<f4").tofile(binary)
    from types import SimpleNamespace
    cfg = SimpleNamespace(
        map=SimpleNamespace(resolution=0.1, slice_dh=0.5),
        trav=SimpleNamespace(kernel_size=7, interval_min=0.50, interval_free=0.65,
                             slope_max=slope, step_max=0.17, standable_ratio=0.2,
                             cost_barrier=50.0, safe_margin=0.4, inflation=0.2),
    )
    minimum, maximum = points.min(axis=0), points.max(axis=0)
    original_center = (minimum[:2] + maximum[:2]) / 2
    original_dims = np.ceil((maximum[:2] - minimum[:2]) / 0.1).astype(int) + 4
    count = int(np.ceil(float(maximum[2]) / 0.5))
    reference = run_original_cpu(module, points, original_center, original_dims, count, cfg)
    original_summary = summarize_reference(reference)
    del reference
    gc.collect()

    # 原生桥为零点floor，上游为相对center的round。原点移到半格但源点不变；
    # 边界舍入分配差异必须单列，不能移动点让结果相同。
    world_xy = np.floor(points[:, :2].astype(np.float64) / 0.1).astype(np.int32)
    aligned_center = np.asarray([0.05, 0.05], dtype=np.float32)
    aligned_dims = 2 * np.max(np.abs(world_xy), axis=0) + 6
    direct_xy = module._round_like_c((points[:, :2] - aligned_center) / 0.1, np).astype(np.int32)
    direct_mismatch = np.any(direct_xy != world_xy, axis=1)
    affected_columns = np.unique(np.concatenate([world_xy[direct_mismatch], direct_xy[direct_mismatch]]), axis=0)
    boundary_indices = np.nonzero(direct_mismatch)[0]
    boundary_rows = [{"point_index": int(index), "xyz": points[index].tolist(),
                      "bridge_floor_xy": world_xy[index].tolist(), "upstream_round_xy": direct_xy[index].tolist()}
                     for index in boundary_indices]
    (args.scratch / (scene + ".boundary_assignments.json")).write_text(json.dumps(boundary_rows, ensure_ascii=False, indent=2) + "\n")

    geometry_path = args.scratch / (scene + ".geometry.bin")
    benchmarks = []
    for threads in args.threads:
        command = [str(args.benchmark), str(binary), "0.1", "0.5", "0.5", "0.05",
                   str(threads), str(args.repeats), str(args.warmups), str(geometry_path)]
        finished = subprocess.run(command, check=True, text=True, capture_output=True)
        metric = json.loads(finished.stdout)
        metric["command"] = command
        metric["geometry_sha256"] = hashlib.sha256(geometry_path.read_bytes()).hexdigest()
        benchmarks.append(metric)
    if len({metric["geometry_sha256"] for metric in benchmarks}) != 1:
        raise AssertionError("单线程与多线程的真实场景几何输出不同")
    with geometry_path.open("rb") as source:
        if source.read(8) != b"PCTG4V01":
            raise ValueError("原生几何文件标识错误")
        n_cells, n_bands = np.frombuffer(source.read(16), dtype="<u8")
        actual = np.frombuffer(source.read(int(n_cells) * GEOMETRY_DTYPE.itemsize), dtype=GEOMETRY_DTYPE)
    raw_reference = run_original_cpu(module, points, aligned_center, aligned_dims, count, cfg)
    raw_summary = summarize_reference(raw_reference)
    raw_comparison = compare_raw_slices(raw_reference, actual, aligned_dims, affected_columns)
    del raw_reference
    gc.collect()
    bridge_reference = run_original_cpu(module, points, aligned_center, aligned_dims, count, cfg, world_xy)
    bridge_summary = summarize_reference(bridge_reference)
    expected = sparse_geometry(bridge_reference, aligned_dims)
    del bridge_reference
    gc.collect()
    if len(expected) != len(actual) or not np.array_equal(expected, actual):
        raise AssertionError(f"{scene} 桥网格适配参考不同：参考{len(expected)}，实际{len(actual)}")
    unique_xy, counts = np.unique(np.column_stack([actual["x"], actual["y"]]), axis=0, return_counts=True)
    indices = sorted(set([0, len(actual) // 4, len(actual) // 2, 3 * len(actual) // 4, len(actual) - 1]))
    selected = []
    for index in indices:
        row = actual[index]
        selected.append({"x": int(row["x"]), "y": int(row["y"]), "ground": float(row["ground"]),
                         "ceiling": float(row["ceiling"]) if np.isfinite(row["ceiling"]) else None})
    return {
        "scene": scene, "pcd": str(pcd), "pcd_sha256": hashlib.sha256(pcd.read_bytes()).hexdigest(),
        "points": len(points), "config": {"resolution": 0.1, "slice_h0": 0.5, "slice_dh": 0.5, "point_resolution": 0.05},
        "original_upstream_frame": {"center": original_center.tolist(), "dims": original_dims.tolist(), **original_summary},
        "raw_points_half_cell_origin_upstream_round": {"center": aligned_center.tolist(), "dims": aligned_dims.tolist(), **raw_summary,
                                 "source_points_modified": False,
                                 "round_floor_boundary_points": int(direct_mismatch.sum()),
                                 "affected_columns": len(affected_columns),
                                 "maximum_assignment_shift_cells_linf": int(np.max(np.abs(world_xy - direct_xy))),
                                 "maximum_assignment_shift_m_l2": float(np.max(np.linalg.norm((world_xy - direct_xy) * 0.1, axis=1))),
                                 "boundary_assignment_samples": boundary_rows[:12], "comparison": raw_comparison},
        "raw_points_bridge_floor_index_adapter": {"source_points_modified": False, "only_grid_index_callback_replaced": True,
                                 **bridge_summary},
        "bridge_floor_comparison": {"geometry_pairs": len(actual), "occupied_columns": len(unique_xy), "bands": int(n_bands),
                       "maximum_geometry_pairs_per_column": int(counts.max()),
                       "ground_max_abs_error_m": float(np.max(np.abs(expected["ground"] - actual["ground"]))),
                       "ceiling_max_abs_error_m": 0.0,
                       "all_geometry_pairs_exact_equal": True, "sample_cells": selected},
        "rust_benchmarks": benchmarks,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pct-scan-root", type=Path, required=True)
    parser.add_argument("--benchmark", type=Path, required=True)
    parser.add_argument("--scratch", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--threads", type=int, nargs="+", default=[1, 8])
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--warmups", type=int, default=1)
    args = parser.parse_args()
    args.scratch.mkdir(parents=True, exist_ok=True)
    source = args.pct_scan_root / "scripts/generate_pct_tomogram.py"
    module = load_module("g4_upstream_cpu", source)
    started = time.monotonic()
    result = {
        "pct_scan_commit": subprocess.check_output(["git", "-C", str(args.pct_scan_root), "rev-parse", "HEAD"], text=True).strip(),
        "cpu_source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "numpy": np.__version__, "scipy": scipy.__version__,
        "scenes": [compare_scene(args, module, *scene) for scene in
                   [("building", "building2_9.pcd", 0.40), ("plaza", "plaza3_10.pcd", 0.36)]],
    }
    result["wall_seconds"] = time.monotonic() - started
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
