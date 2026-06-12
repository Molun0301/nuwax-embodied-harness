"""标定结果的欧拉/四元数转换、持久化（active.json）与 frame_tree 注入。

【本包未接线、本期冻结】仅被 calibration 包内引用。生效链路见 CalibrationProvider
（drivers/go2/providers.py + services/geometry/runtime.py 的 _inject_static_extrinsics
静态外参边）；注意本文件 inject_calibration_to_frame_tree 瞄准的 ingest_transform_snapshot
注入面与现行 provider.set_static_extrinsics 并非同一个（05-P1-6）。MI 在线标定
【严禁直接接线】，理由见 05-P2-8/P2-9。详见 config.py 头部冻结说明。

已知缺陷（接线前必修，05-P2-11）：load_persisted_calibration 无 robot_id/frame 对一致性
校验，换机或改 frame 命名后旧 active.json 仍会被加载注入。
"""

from __future__ import annotations

import json
import logging
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

from contracts.calibration import ExtrinsicCalibration
from contracts.geometry import Quaternion


LOGGER = logging.getLogger(__name__)


def euler_to_quaternion(rx: float, ry: float, rz: float) -> Quaternion:
    """欧拉角（弧度）转四元数，ZYX 旋转顺序（即 roll-pitch-yaw）。"""
    cr, sr = math.cos(rx / 2), math.sin(rx / 2)
    cp, sp = math.cos(ry / 2), math.sin(ry / 2)
    cy, sy = math.cos(rz / 2), math.sin(rz / 2)
    return Quaternion(
        w=cr * cp * cy + sr * sp * sy,
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
    )


def quaternion_to_euler(q: Quaternion) -> tuple[float, float, float]:
    """四元数转欧拉角（弧度），返回 (rx, ry, rz) 即 (roll, pitch, yaw)，ZYX 顺序。"""
    # roll (rx)
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    rx = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (ry)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1.0:
        ry = math.copysign(math.pi / 2, sinp)
    else:
        ry = math.asin(sinp)

    # yaw (rz)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    rz = math.atan2(siny_cosp, cosy_cosp)

    return rx, ry, rz


def build_transform_from_calibration(calibration: ExtrinsicCalibration) -> Dict[str, Any]:
    """从标定结果构建 frame_tree 兼容的 transform 字典。"""
    return {
        "parent_frame_id": calibration.parent_frame_id,
        "child_frame_id": calibration.child_frame_id,
        "translation": {
            "x": calibration.translation.x,
            "y": calibration.translation.y,
            "z": calibration.translation.z,
        },
        "rotation": {
            "x": calibration.rotation.x,
            "y": calibration.rotation.y,
            "z": calibration.rotation.z,
            "w": calibration.rotation.w,
        },
    }


def persist_calibration(calibration: ExtrinsicCalibration, persist_dir: str) -> None:
    """持久化标定结果到本地文件。

    - 保存到 {persist_dir}/history/{timestamp}.json
    - 更新 {persist_dir}/active.json
    """
    base = Path(persist_dir)
    history_dir = base / "history"
    history_dir.mkdir(parents=True, exist_ok=True)

    data = calibration.model_dump(mode="json")

    # 按时间戳命名历史文件（冒号替换为连字符，兼容文件系统）
    ts = datetime.now(tz=timezone.utc).strftime("%Y-%m-%dT%H-%M-%S")
    history_path = history_dir / f"{ts}.json"
    history_path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    LOGGER.info("标定历史已保存: %s", history_path)

    active_path = base / "active.json"
    active_path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    LOGGER.info(
        "active.json 已更新: parent=%s child=%s",
        calibration.parent_frame_id,
        calibration.child_frame_id,
    )


def load_persisted_calibration(persist_dir: str) -> Optional[ExtrinsicCalibration]:
    """从持久化目录加载最新有效标定结果。

    读取 {persist_dir}/active.json，文件不存在或解析失败返回 None。
    """
    active_path = Path(persist_dir) / "active.json"
    if not active_path.exists():
        LOGGER.debug("未找到持久化标定文件: %s", active_path)
        return None
    try:
        data = json.loads(active_path.read_text(encoding="utf-8"))
        calibration = ExtrinsicCalibration.model_validate(data)
        LOGGER.info(
            "已加载持久化标定: parent=%s child=%s",
            calibration.parent_frame_id,
            calibration.child_frame_id,
        )
        return calibration
    except Exception as exc:  # noqa: BLE001
        LOGGER.warning("解析持久化标定文件失败 (%s): %s", active_path, exc)
        return None


def inject_calibration_to_frame_tree(
    calibration: ExtrinsicCalibration,
    localization_provider: Any,
) -> bool:
    """将标定结果注入 frame_tree。

    优先调用 localization_provider.ingest_transform_snapshot()；
    若该方法不存在则尝试其他注入方式；均不可用时返回 False。
    """
    transform_data = build_transform_from_calibration(calibration)
    transforms: List[Dict[str, Any]] = [transform_data]

    # 优先路径：ingest_transform_snapshot
    ingest_fn = getattr(localization_provider, "ingest_transform_snapshot", None)
    if callable(ingest_fn):
        try:
            ingest_fn(transforms, source_topic="auto_calibration", static=True)
            LOGGER.info(
                "标定已通过 ingest_transform_snapshot 注入 frame_tree: %s -> %s",
                calibration.parent_frame_id,
                calibration.child_frame_id,
            )
            return True
        except Exception as exc:  # noqa: BLE001
            LOGGER.error("ingest_transform_snapshot 调用失败: %s", exc)
            return False

    # 备用路径：update_transform / set_transform
    for method_name in ("update_transform", "set_transform"):
        update_fn = getattr(localization_provider, method_name, None)
        if callable(update_fn):
            try:
                update_fn(transform_data)
                LOGGER.info(
                    "标定已通过 %s 注入 frame_tree: %s -> %s",
                    method_name,
                    calibration.parent_frame_id,
                    calibration.child_frame_id,
                )
                return True
            except Exception as exc:  # noqa: BLE001
                LOGGER.error("%s 调用失败: %s", method_name, exc)
                return False

    LOGGER.warning(
        "localization_provider 不支持任何已知注入方法，标定注入跳过: %s",
        type(localization_provider).__name__,
    )
    return False
