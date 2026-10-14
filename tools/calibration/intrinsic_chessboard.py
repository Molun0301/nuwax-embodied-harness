#!/usr/bin/env python3
"""相机内参离线棋盘格标定（Phase-A）。

按平台分层治理规范：内参是机器人特定物理参数，归端侧 driver 唯一真相源。本脚本
离线运行（图像在实机采集后拷出），产出真实内参 + 畸变系数，**用证据裁定畸变模型**
（plumb_bob vs fisheye-KB，按重投影残差比较，不预设鱼眼 = 解 Phase0 门2），
最后打印可写回 drivers/go2/defaults.py 的代码片段。

用法：
    uv run --with opencv-python-headless --with numpy \\
        python tools/calibration/intrinsic_chessboard.py \\
        --images '/path/to/chessboard/*.jpg' \\
        --rows 6 --cols 9 --square-size-m 0.025

要点：
- --rows/--cols 是棋盘**内部角点**数（不是方格数）。常见 A4 打印 9x6 角点。
- 至少 10~20 张不同角度/距离的清晰棋盘图，覆盖画面边角（边角畸变信息最关键）。
- 图像分辨率必须与实机推流一致（实测 1920×1080）；脚本会校验并打印实际分辨率。
- 平台侧/服务层不依赖本脚本；产物经 driver 静态常量进入 CalibrationProvider。
"""
from __future__ import annotations

import argparse
import glob
import sys
from typing import List, Tuple

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover
    print("需要 opencv：uv run --with opencv-python-headless --with numpy python ...", file=sys.stderr)
    raise


def _find_corners(image_paths: List[str], rows: int, cols: int) -> Tuple[List[np.ndarray], Tuple[int, int], int]:
    """在每张图找棋盘内部角点，返回 (角点列表, (w,h), 成功数)。"""
    pattern = (cols, rows)  # cv2 用 (内部角点列数, 行数)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    image_points: List[np.ndarray] = []
    image_size = None
    used = 0
    for path in image_paths:
        img = cv2.imread(path)
        if img is None:
            print(f"  跳过无法读取: {path}")
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if image_size is None:
            image_size = (gray.shape[1], gray.shape[0])
        elif (gray.shape[1], gray.shape[0]) != image_size:
            print(f"  ⚠ 分辨率不一致 {gray.shape[1]}x{gray.shape[0]} != {image_size}，跳过 {path}")
            continue
        found, corners = cv2.findChessboardCorners(
            gray, pattern,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
        )
        if not found:
            print(f"  未检出棋盘: {path}")
            continue
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        image_points.append(corners)
        used += 1
        print(f"  ✓ 角点检出: {path}")
    return image_points, image_size, used


def _object_points(rows: int, cols: int, square: float, n: int) -> List[np.ndarray]:
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= square
    return [objp.copy() for _ in range(n)]


def _calibrate_pinhole(obj, img, size):
    rms, K, D, _, _ = cv2.calibrateCamera(obj, img, size, None, None)
    return rms, K, D.flatten().tolist()


def _calibrate_fisheye(obj, img, size):
    obj_fe = [o.reshape(-1, 1, 3) for o in obj]
    K = np.zeros((3, 3))
    D = np.zeros((4, 1))
    flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_FIX_SKEW
    try:
        rms, K, D, _, _ = cv2.fisheye.calibrate(
            obj_fe, img, size, K, D, flags=flags,
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6),
        )
        return rms, K, D.flatten().tolist()
    except cv2.error as exc:
        print(f"  fisheye 标定失败（可能不是鱼眼或角点不足）: {exc}")
        return float("inf"), None, None


def main() -> int:
    ap = argparse.ArgumentParser(description="相机内参离线棋盘格标定（Phase-A）")
    ap.add_argument("--images", required=True, help="棋盘图 glob，如 '/data/chess/*.jpg'")
    ap.add_argument("--rows", type=int, required=True, help="棋盘内部角点行数")
    ap.add_argument("--cols", type=int, required=True, help="棋盘内部角点列数")
    ap.add_argument("--square-size-m", type=float, required=True, help="方格边长（米）")
    args = ap.parse_args()

    paths = sorted(glob.glob(args.images))
    if not paths:
        print(f"未匹配到图像: {args.images}", file=sys.stderr)
        return 2
    print(f"匹配到 {len(paths)} 张图，开始检测角点...")
    img_pts, size, used = _find_corners(paths, args.rows, args.cols)
    if used < 5:
        print(f"有效棋盘图仅 {used} 张，至少需 5（建议 10~20）。", file=sys.stderr)
        return 2
    print(f"\n有效图 {used} 张，分辨率 {size[0]}x{size[1]}")
    if (size[0], size[1]) != (1920, 1080):
        print(f"⚠ 注意：分辨率 {size[0]}x{size[1]} 与实机推流实测 1920x1080 不一致，确认采集源一致。")

    obj_pts = _object_points(args.rows, args.cols, args.square_size_m, used)

    print("\n=== 模型对比（证据裁定 Phase0 门2，不预设鱼眼）===")
    rms_p, K_p, D_p = _calibrate_pinhole(obj_pts, img_pts, size)
    print(f"plumb_bob  重投影 RMS = {rms_p:.4f} px")
    rms_f, K_f, D_f = _calibrate_fisheye(obj_pts, img_pts, size)
    print(f"fisheye-KB 重投影 RMS = {rms_f:.4f} px")

    # 裁定：残差更小者胜；接近时优先 plumb_bob（更简单、下游通用）。
    if rms_f < rms_p * 0.7:
        model, K, D = "fisheye", K_f, D_f
        verdict = f"fisheye 显著更优（{rms_f:.3f} < 0.7×{rms_p:.3f}）"
    else:
        model, K, D = "plumb_bob", K_p, D_p
        verdict = f"plumb_bob（fisheye 未显著更优）"
    print(f"\n裁定模型: {model} — {verdict}")

    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    print("\n=== 写回 drivers/go2/defaults.py 的 GO2_DEFAULT_CAMERA_INFO ===")
    print(f"""GO2_DEFAULT_CAMERA_INFO = CameraInfo(
    camera_id="front_camera",
    frame_id=GO2_DEFAULT_FRAME_IDS["camera_front"],
    width_px={size[0]},
    height_px={size[1]},
    fx={fx:.6f},
    fy={fy:.6f},
    cx={cx:.6f},
    cy={cy:.6f},
    distortion_model="{model}",
    distortion_coefficients={[round(float(c), 8) for c in D]},
)""")
    print("\n注意：写回后 SensorPair 的 camera_* 字段会自动同步（引用同一 CameraInfo）。")
    print("外参 initial_translation/rotation 不受影响（来自 URDF 建模值，由 MI 后续精修）。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
