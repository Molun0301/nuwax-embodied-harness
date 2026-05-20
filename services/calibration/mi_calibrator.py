from __future__ import annotations

import logging
import math
import time
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np

from contracts.calibration import ExtrinsicCalibration, SensorPair
from contracts.geometry import Quaternion, Vector3
from services.calibration.config import (
    get_calibration_nmi_threshold,
    get_calibration_reprojection_threshold,
)
from services.calibration.data_collector import FramePair
from services.calibration.transform_injector import euler_to_quaternion


LOGGER = logging.getLogger(__name__)

# Sobel 卷积核（3x3）
_SOBEL_X = np.array([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]], dtype=np.float32)
_SOBEL_Y = np.array([[-1, -2, -1], [0, 0, 0], [1, 2, 1]], dtype=np.float32)


def _sobel_magnitude(gray: np.ndarray) -> np.ndarray:
    """对灰度图做 Sobel 边缘检测，返回梯度幅值图（float32，未归一化）。"""
    h, w = gray.shape
    img = gray.astype(np.float32)

    # 手动 3x3 卷积（避免依赖 scipy/cv2）
    gx = np.zeros_like(img)
    gy = np.zeros_like(img)

    # 对内部像素做卷积（边缘填 0）
    for dy in range(3):
        for dx in range(3):
            kx = _SOBEL_X[dy, dx]
            ky = _SOBEL_Y[dy, dx]
            if kx == 0 and ky == 0:
                continue
            patch = img[dy : h - 2 + dy, dx : w - 2 + dx]
            gx[1 : h - 1, 1 : w - 1] += kx * patch
            gy[1 : h - 1, 1 : w - 1] += ky * patch

    return np.sqrt(gx * gx + gy * gy)


def _build_rotation_matrix(rx: float, ry: float, rz: float) -> np.ndarray:
    """欧拉角（ZYX 顺序，弧度）转 3x3 旋转矩阵。"""
    cr, sr = math.cos(rx), math.sin(rx)
    cp, sp = math.cos(ry), math.sin(ry)
    cy, sy = math.cos(rz), math.sin(rz)

    # R = Rz * Ry * Rx
    r = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr               ],
    ], dtype=np.float64)
    return r


def project_points_to_image(
    points_xyz: np.ndarray,
    extrinsic_params: np.ndarray,
    camera_params: Dict[str, float],
) -> Tuple[np.ndarray, np.ndarray]:
    """将 3D 点云投影到图像平面。

    Args:
        points_xyz: 形状 (N, 3) 的点云，lidar 坐标系。
        extrinsic_params: 形状 (6,) 的外参向量 [tx, ty, tz, rx, ry, rz]。
        camera_params: 包含 fx, fy, cx, cy, width, height 的字典。

    Returns:
        pixel_coords: 形状 (M, 2) 的像素坐标 [u, v]。
        depths: 形状 (M,) 的深度值（相机坐标系 Z 轴）。
    """
    if points_xyz.shape[0] == 0:
        return np.zeros((0, 2), dtype=np.float64), np.zeros((0,), dtype=np.float64)

    tx, ty, tz, rx, ry, rz = extrinsic_params
    R = _build_rotation_matrix(rx, ry, rz)

    # 变换到相机坐标系
    pts_cam = (R @ points_xyz.T).T + np.array([tx, ty, tz], dtype=np.float64)

    # 过滤 Z <= 0 的点
    valid = pts_cam[:, 2] > 0.0
    pts_cam = pts_cam[valid]
    if pts_cam.shape[0] == 0:
        return np.zeros((0, 2), dtype=np.float64), np.zeros((0,), dtype=np.float64)

    fx = camera_params["fx"]
    fy = camera_params["fy"]
    cx = camera_params["cx"]
    cy = camera_params["cy"]
    w = int(camera_params["width"])
    h = int(camera_params["height"])

    Z = pts_cam[:, 2]
    u = fx * pts_cam[:, 0] / Z + cx
    v = fy * pts_cam[:, 1] / Z + cy

    # 过滤图像范围外的点
    in_frame = (u >= 0) & (u < w) & (v >= 0) & (v < h)
    u = u[in_frame]
    v = v[in_frame]
    depths = Z[in_frame]

    pixel_coords = np.stack([u, v], axis=1)
    return pixel_coords, depths


def render_depth_edges(
    points_xyz: np.ndarray,
    extrinsic_params: np.ndarray,
    camera_params: Dict[str, float],
    image_shape: Tuple[int, int],
) -> np.ndarray:
    """渲染点云深度不连续边缘图。

    Args:
        points_xyz: 形状 (N, 3) 的点云。
        extrinsic_params: 形状 (6,) 的外参向量 [tx, ty, tz, rx, ry, rz]。
        camera_params: 包含 fx, fy, cx, cy, width, height 的字典。
        image_shape: (height, width) 元组。

    Returns:
        边缘强度图，形状 (H, W)，float32。
    """
    h, w = image_shape
    depth_map = np.zeros((h, w), dtype=np.float32)

    pixel_coords, depths = project_points_to_image(points_xyz, extrinsic_params, camera_params)
    if pixel_coords.shape[0] == 0:
        return depth_map

    us = pixel_coords[:, 0].astype(np.int32)
    vs = pixel_coords[:, 1].astype(np.int32)

    # 稀疏深度图：同一像素取最近深度
    for i in range(len(us)):
        u_i, v_i = us[i], vs[i]
        d = depths[i]
        if depth_map[v_i, u_i] == 0.0 or d < depth_map[v_i, u_i]:
            depth_map[v_i, u_i] = d

    # 对深度图做 Sobel 边缘检测
    edges = _sobel_magnitude(depth_map)
    return edges


def extract_image_edges(
    image_bytes: bytes,
    width: int,
    height: int,
) -> np.ndarray:
    """从 JPEG 图像提取边缘（Sobel）。

    Args:
        image_bytes: JPEG 编码的图像字节。
        width: 图像宽度（像素）。
        height: 图像高度（像素）。

    Returns:
        边缘强度图，形状 (H, W)，float32。
    """
    try:
        import cv2  # opencv-python 在 requirements.txt 中已声明
        buf = np.frombuffer(image_bytes, dtype=np.uint8)
        bgr = cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)
        if bgr is None:
            raise ValueError("cv2.imdecode 返回 None，图像数据可能损坏")
        gray = bgr.astype(np.float32)
    except ImportError:
        # 降级：假设 image_bytes 已是 uint8 灰度 numpy 数组的原始字节
        LOGGER.warning("cv2 不可用，尝试将 image_bytes 直接解析为灰度数组")
        arr = np.frombuffer(image_bytes, dtype=np.uint8)
        if arr.size == width * height:
            gray = arr.reshape(height, width).astype(np.float32)
        else:
            LOGGER.error(
                "image_bytes 大小 %d 与期望 %d 不符，返回空边缘图",
                arr.size, width * height,
            )
            return np.zeros((height, width), dtype=np.float32)

    return _sobel_magnitude(gray)


def compute_nmi(
    image_a: np.ndarray,
    image_b: np.ndarray,
    bins: int = 32,
) -> float:
    """计算两幅图像的归一化互信息（NMI）。

    NMI = (H(A) + H(B)) / H(A, B)

    Args:
        image_a: 第一幅图像，任意形状，float。
        image_b: 第二幅图像，与 image_a 形状相同。
        bins: 直方图分箱数。

    Returns:
        NMI 值（浮点数，≥ 1.0，值越大对齐越好）。
    """
    a = image_a.ravel().astype(np.float64)
    b = image_b.ravel().astype(np.float64)

    # 归一化到 [0, 1]
    a_min, a_max = a.min(), a.max()
    b_min, b_max = b.min(), b.max()
    if a_max - a_min < 1e-9 or b_max - b_min < 1e-9:
        return 1.0  # 退化情况：返回最小有效值

    a = (a - a_min) / (a_max - a_min)
    b = (b - b_min) / (b_max - b_min)

    # 联合直方图
    joint_hist, _, _ = np.histogram2d(a, b, bins=bins, range=[[0, 1], [0, 1]])
    joint_hist = joint_hist / (joint_hist.sum() + 1e-12)

    # 边缘分布
    p_a = joint_hist.sum(axis=1)
    p_b = joint_hist.sum(axis=0)

    def _entropy(p: np.ndarray) -> float:
        mask = p > 0
        return float(-np.sum(p[mask] * np.log(p[mask] + 1e-12)))

    h_a = _entropy(p_a)
    h_b = _entropy(p_b)
    h_ab = _entropy(joint_hist.ravel())

    if h_ab < 1e-12:
        return 1.0

    return (h_a + h_b) / h_ab


def nelder_mead_optimize(
    objective_fn: Callable[[np.ndarray], float],
    x0: np.ndarray,
    bounds: np.ndarray,
    max_iter: int = 300,
    tol: float = 1e-6,
) -> Tuple[np.ndarray, float, bool]:
    """纯 numpy 实现的 Nelder-Mead 单纯形优化器。

    Args:
        objective_fn: 目标函数，接受形状 (n,) 的参数向量，返回标量。
        x0: 初始参数，形状 (n,)。
        bounds: 参数边界，形状 (n, 2)，每行 [lower, upper]。
        max_iter: 最大迭代次数。
        tol: 收敛容差（单纯形尺寸）。

    Returns:
        (best_params, best_value, converged)
    """
    n = len(x0)
    alpha = 1.0   # 反射系数
    gamma = 2.0   # 扩张系数
    rho = 0.5     # 收缩系数
    sigma = 0.5   # 压缩系数

    def _clip(x: np.ndarray) -> np.ndarray:
        return np.clip(x, bounds[:, 0], bounds[:, 1])

    # 初始化单纯形（n+1 个顶点）
    simplex = np.zeros((n + 1, n), dtype=np.float64)
    simplex[0] = _clip(x0.astype(np.float64))
    for i in range(n):
        xi = simplex[0].copy()
        step = (bounds[i, 1] - bounds[i, 0]) * 0.05
        xi[i] += step if xi[i] + step <= bounds[i, 1] else -step
        simplex[i + 1] = _clip(xi)

    # 计算各顶点目标值
    scores = np.array([objective_fn(simplex[i]) for i in range(n + 1)])

    converged = False
    for _ in range(max_iter):
        # 按目标值排序
        order = np.argsort(scores)
        simplex = simplex[order]
        scores = scores[order]

        # 收敛检查：单纯形尺寸
        size = np.max(np.linalg.norm(simplex[1:] - simplex[0], axis=1))
        if size < tol:
            converged = True
            break

        # 重心（排除最差顶点）
        centroid = simplex[:-1].mean(axis=0)

        # 反射
        x_r = _clip(centroid + alpha * (centroid - simplex[-1]))
        f_r = objective_fn(x_r)

        if f_r < scores[0]:
            # 扩张
            x_e = _clip(centroid + gamma * (x_r - centroid))
            f_e = objective_fn(x_e)
            if f_e < f_r:
                simplex[-1] = x_e
                scores[-1] = f_e
            else:
                simplex[-1] = x_r
                scores[-1] = f_r
        elif f_r < scores[-2]:
            simplex[-1] = x_r
            scores[-1] = f_r
        else:
            # 收缩
            if f_r < scores[-1]:
                x_c = _clip(centroid + rho * (x_r - centroid))
                f_c = objective_fn(x_c)
                if f_c <= f_r:
                    simplex[-1] = x_c
                    scores[-1] = f_c
                else:
                    # 压缩整体单纯形
                    for i in range(1, n + 1):
                        simplex[i] = _clip(simplex[0] + sigma * (simplex[i] - simplex[0]))
                        scores[i] = objective_fn(simplex[i])
            else:
                x_c = _clip(centroid + rho * (simplex[-1] - centroid))
                f_c = objective_fn(x_c)
                if f_c < scores[-1]:
                    simplex[-1] = x_c
                    scores[-1] = f_c
                else:
                    for i in range(1, n + 1):
                        simplex[i] = _clip(simplex[0] + sigma * (simplex[i] - simplex[0]))
                        scores[i] = objective_fn(simplex[i])

    return simplex[0], scores[0], converged


def calibrate(
    frame_pairs: List[FramePair],
    sensor_pair: SensorPair,
) -> Optional[ExtrinsicCalibration]:
    """基于 NMI 最大化的相机-激光雷达外参标定主入口。

    Args:
        frame_pairs: 时间同步的点云-图像帧对列表。
        sensor_pair: 传感器对配置（含初始猜测和搜索范围）。

    Returns:
        标定结果 ExtrinsicCalibration，或 None（标定失败）。
    """
    if not frame_pairs:
        LOGGER.warning("标定失败：帧对列表为空")
        return None

    LOGGER.info("开始 NMI 标定，帧对数量: %d", len(frame_pairs))
    t_start = time.monotonic()

    # 从 sensor_pair 提取初始猜测
    q = sensor_pair.initial_rotation
    # 四元数转欧拉角（ZYX 顺序）
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    rx0 = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    ry0 = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    rz0 = math.atan2(siny_cosp, cosy_cosp)

    x0 = np.array([
        sensor_pair.initial_translation.x,
        sensor_pair.initial_translation.y,
        sensor_pair.initial_translation.z,
        rx0, ry0, rz0,
    ], dtype=np.float64)

    bt = sensor_pair.bounds.translation_m
    br = sensor_pair.bounds.rotation_rad
    bounds = np.array([
        [x0[0] - bt, x0[0] + bt],
        [x0[1] - bt, x0[1] + bt],
        [x0[2] - bt, x0[2] + bt],
        [x0[3] - br, x0[3] + br],
        [x0[4] - br, x0[4] + br],
        [x0[5] - br, x0[5] + br],
    ], dtype=np.float64)

    camera_params: Dict[str, float] = {
        "fx": sensor_pair.camera_fx,
        "fy": sensor_pair.camera_fy,
        "cx": sensor_pair.camera_cx,
        "cy": sensor_pair.camera_cy,
        "width": float(sensor_pair.camera_width_px),
        "height": float(sensor_pair.camera_height_px),
    }

    def objective(params: np.ndarray) -> float:
        """目标函数：对所有帧对计算平均 NMI 的负值（最小化 = 最大化 NMI）。"""
        total_nmi = 0.0
        valid_count = 0
        for fp in frame_pairs:
            h = fp.image_height_px
            w = fp.image_width_px
            try:
                img_edges = extract_image_edges(fp.image_bytes, w, h)
                depth_edges = render_depth_edges(
                    fp.point_cloud_xyz, params, camera_params, (h, w)
                )
                nmi = compute_nmi(img_edges, depth_edges)
                total_nmi += nmi
                valid_count += 1
            except Exception as exc:  # noqa: BLE001
                LOGGER.debug("帧对 NMI 计算异常（跳过）: %s", exc)
        if valid_count == 0:
            return 0.0  # 返回 0 使优化器远离此区域（NMI 取负后为 0 最差）
        return -(total_nmi / valid_count)

    best_params, best_neg_nmi, converged = nelder_mead_optimize(
        objective, x0, bounds, max_iter=300, tol=1e-6
    )
    best_nmi = -best_neg_nmi
    elapsed = time.monotonic() - t_start

    LOGGER.info(
        "NMI 优化完成: nmi=%.4f, converged=%s, elapsed=%.1fs",
        best_nmi, converged, elapsed,
    )

    # 安全门控：NMI 必须超过阈值
    nmi_threshold = get_calibration_nmi_threshold()
    if best_nmi < nmi_threshold:
        LOGGER.warning(
            "标定 NMI %.4f 低于阈值 %.4f，拒绝结果", best_nmi, nmi_threshold
        )
        return None

    # 安全门控：最优参数必须在 bounds 内
    if np.any(best_params < bounds[:, 0]) or np.any(best_params > bounds[:, 1]):
        LOGGER.warning("最优参数超出搜索范围，拒绝结果: %s", best_params)
        return None

    tx, ty, tz, rx, ry, rz = best_params
    rotation = euler_to_quaternion(rx, ry, rz)

    calibration = ExtrinsicCalibration(
        parent_frame_id=sensor_pair.parent_frame_id,
        child_frame_id=sensor_pair.child_frame_id,
        translation=Vector3(x=float(tx), y=float(ty), z=float(tz)),
        rotation=rotation,
        nmi_score=float(best_nmi),
        frame_pair_count=len(frame_pairs),
        version="nmi_v1",
        source="auto",
        metadata={
            "converged": converged,
            "elapsed_sec": round(elapsed, 2),
            "nmi_threshold": nmi_threshold,
        },
    )

    LOGGER.info(
        "标定成功: parent=%s child=%s tx=%.4f ty=%.4f tz=%.4f nmi=%.4f",
        calibration.parent_frame_id,
        calibration.child_frame_id,
        tx, ty, tz, best_nmi,
    )
    return calibration
