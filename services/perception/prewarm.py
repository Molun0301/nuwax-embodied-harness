"""YOLO-World 启动期预热。

vis-three-fixes 修复 1（启动慢）：感知模型 `_get_model()` 是懒加载，只在首次
`run_once` 触发 `detect` 时才加载 + 触发 GPU kernel cache 构建，造成探索 MCP 调用后
5-10s 检测空窗。本模块在网关装配期主动喂一张 dummy 图像跑一次推理，把"模型加载 +
首帧 warmup"挪到启动期。

设计要点：
- 解析 `config.perception.yolo_world.model_name`，复用后端自身的文件存在校验。
- 通过 `DetectorPipeline.get_backend()` 拿 YoloWorld 后端实例，调 `detect()` 跑一次
  完整链路（含 `_get_model` + `set_classes`（.pt）/ engine warmup + 推理）。
- 失败一律静默降级（logger.warning + 返回 deferred），**绝不 raise**，避免拖垮网关启动。
"""
from __future__ import annotations

import logging
from time import perf_counter
from typing import Any, Dict, Optional

import numpy as np

PERCEPTION_PREWARM_LOGGER = logging.getLogger("nuwax_robot_bridge.perception.prewarm")


def _build_dummy_image_frame(imgsz: int, camera_id: str):
    """构造一张 HWC BGR uint8 dummy ImageFrame（BGR8 编码，dense 字节）。"""
    from contracts.image import ImageEncoding, ImageFrame

    # 任务 2.2：HWC BGR uint8，随机噪声，imgsz×imgsz。
    dummy = np.random.randint(0, 255, (imgsz, imgsz, 3), dtype=np.uint8)
    return ImageFrame(
        camera_id=camera_id,
        frame_id="prewarm_dummy",
        width_px=imgsz,
        height_px=imgsz,
        encoding=ImageEncoding.BGR8,
        data=dummy.tobytes(),
        metadata={"source_kind": "prewarm_dummy"},
    )


def prewarm_yolo_world(detector_pipeline, config, *, camera_id: str = "front_camera") -> Dict[str, Any]:
    """启动期预热 YOLO-World 检测后端。

    Args:
        detector_pipeline: PerceptionService 的 DetectorPipeline（暴露 get_backend）。
        config: NuwaxRobotBridgeConfig（读取 perception.yolo_world 配置）。
        camera_id: dummy 帧 camera_id。

    Returns:
        dict：成功 ``{"status": "ready", "first_inference_ms": float, "backend": str}``；
        失败 ``{"status": "deferred", "reason": str}``。绝不抛异常。
    """
    try:
        yolo_cfg = config.perception.yolo_world
    except AttributeError as exc:
        PERCEPTION_PREWARM_LOGGER.warning("prewarm_yolo_world 无法读取 yolo_world 配置：%s", exc)
        return {"status": "deferred", "reason": "config_missing:%s" % exc}

    if not getattr(yolo_cfg, "enabled", False):
        return {"status": "deferred", "reason": "yolo_world_disabled"}

    getter = getattr(detector_pipeline, "get_backend", None)
    if not callable(getter):
        return {"status": "deferred", "reason": "detector_pipeline_no_get_backend"}

    backend = getter(getattr(yolo_cfg, "backend_name", None))
    if backend is None:
        return {"status": "deferred", "reason": "yolo_world_backend_not_registered"}

    imgsz = int(getattr(yolo_cfg, "image_size", 640) or 640)

    try:
        image_frame = _build_dummy_image_frame(imgsz, camera_id)
        started_at = perf_counter()
        # detect() 内部触发 _get_model + （.pt 时 set_classes）+ 推理，与生产链路一致。
        backend.detect(image_frame)
        first_inference_ms = round((perf_counter() - started_at) * 1000.0, 3)
    except Exception as exc:  # noqa: BLE001 — 预热失败必须静默降级，绝不 raise。
        PERCEPTION_PREWARM_LOGGER.warning(
            "prewarm_yolo_world 预热失败，降级为懒加载 backend=%s reason=%s",
            getattr(yolo_cfg, "backend_name", "yolo_world"),
            exc,
        )
        return {"status": "deferred", "reason": str(exc)}

    result = {
        "status": "ready",
        "first_inference_ms": first_inference_ms,
        "backend": getattr(yolo_cfg, "backend_name", "yolo_world"),
        "model_name": getattr(yolo_cfg, "model_name", ""),
        "image_size": imgsz,
    }
    PERCEPTION_PREWARM_LOGGER.info(
        "prewarm_yolo_world 完成 backend=%s model=%s first_inference_ms=%.1f",
        result["backend"],
        result["model_name"],
        first_inference_ms,
    )
    return result
