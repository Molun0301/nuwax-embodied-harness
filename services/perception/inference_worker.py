"""YOLO 推理 Worker 进程 — 隔离 GPU 推理以释放主线程 GIL。"""

from __future__ import annotations

import logging
import multiprocessing as mp
from multiprocessing import shared_memory
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)


def _worker_loop(
    request_queue: mp.Queue,
    response_queue: mp.Queue,
    model_source: str,
    predict_kwargs: Dict[str, Any],
) -> None:
    """Worker 进程主循环：加载模型，等待推理请求，返回结果。"""
    import os

    os.environ.setdefault("CUDA_VISIBLE_DEVICES", "0")

    try:
        from ultralytics import YOLO
    except ImportError:
        response_queue.put(("error", "ultralytics 未安装"))
        return

    model = YOLO(model_source)
    response_queue.put(("ready", None))  # 通知主进程模型已加载

    while True:
        try:
            msg = request_queue.get(timeout=30.0)
        except Exception:
            continue

        if msg is None:  # 关闭信号
            break

        shm_name, shape, dtype_str, req_id = msg
        shm: Optional[shared_memory.SharedMemory] = None
        try:
            shm = shared_memory.SharedMemory(name=shm_name)
            image_bgr = np.ndarray(shape, dtype=np.dtype(dtype_str), buffer=shm.buf)

            results = list(model.predict(image_bgr, **predict_kwargs))

            # 提取原始检测结果为可序列化格式
            raw_detections: List[Any] = []
            xyxy_np: Optional[np.ndarray] = None
            conf_np: Optional[np.ndarray] = None
            cls_np: Optional[np.ndarray] = None
            names: Optional[Dict[int, str]] = None
            masks_data: Optional[List[np.ndarray]] = None

            if results:
                result = results[0]
                boxes = getattr(result, "boxes", None)
                if boxes is not None:
                    xyxy = getattr(boxes, "xyxy", None)
                    conf = getattr(boxes, "conf", None)
                    cls = getattr(boxes, "cls", None)
                    if xyxy is not None:
                        xyxy_np = xyxy.cpu().numpy() if hasattr(xyxy, "cpu") else np.asarray(xyxy)
                        conf_np = (
                            conf.cpu().numpy()
                            if hasattr(conf, "cpu")
                            else np.asarray(conf)
                            if conf is not None
                            else np.zeros(len(xyxy_np))
                        )
                        cls_np = (
                            cls.cpu().numpy()
                            if hasattr(cls, "cpu")
                            else np.asarray(cls)
                            if cls is not None
                            else np.zeros(len(xyxy_np))
                        )

                names = getattr(result, "names", None)

                masks_obj = getattr(result, "masks", None)
                if masks_obj is not None:
                    xy = getattr(masks_obj, "xy", None)
                    if xy is not None:
                        masks_data = [np.asarray(m) for m in xy]

            raw_detections = [xyxy_np, conf_np, cls_np, names, masks_data]
            shm.close()
            response_queue.put(("result", req_id, raw_detections))
        except Exception as exc:
            if shm is not None:
                try:
                    shm.close()
                except Exception:
                    pass
            response_queue.put(("error", req_id, str(exc)))


class YoloInferenceWorker:
    """管理 YOLO 推理 Worker 进程的生命周期。"""

    def __init__(
        self,
        model_source: str,
        *,
        confidence_threshold: float = 0.25,
        iou_threshold: float = 0.45,
        image_size: int = 640,
        max_detections: int = 100,
        device: str = "",
    ) -> None:
        self._model_source = model_source
        self._predict_kwargs: Dict[str, Any] = {
            "conf": confidence_threshold,
            "iou": iou_threshold,
            "imgsz": image_size,
            "max_det": max_detections,
            "verbose": False,
        }
        if device:
            self._predict_kwargs["device"] = device
        self._process: Optional[mp.Process] = None
        self._request_queue: Optional[mp.Queue] = None
        self._response_queue: Optional[mp.Queue] = None
        self._shm: Optional[shared_memory.SharedMemory] = None
        self._shm_size: int = 0
        self._req_counter: int = 0
        self._ready: bool = False

    def start(self) -> bool:
        """启动 worker 进程，阻塞等待模型加载完成。"""
        self._request_queue = mp.Queue(maxsize=2)
        self._response_queue = mp.Queue(maxsize=2)
        self._process = mp.Process(
            target=_worker_loop,
            args=(
                self._request_queue,
                self._response_queue,
                self._model_source,
                self._predict_kwargs,
            ),
            daemon=True,
        )
        self._process.start()

        # 等待模型加载（最多 120 秒）
        try:
            msg = self._response_queue.get(timeout=120.0)
            if msg[0] == "ready":
                self._ready = True
                logger.info("YOLO 推理 worker 进程已就绪 pid=%d", self._process.pid)
                return True
            logger.error("YOLO 推理 worker 启动失败: %s", msg)
            return False
        except Exception as exc:
            logger.error("等待 YOLO worker 就绪超时: %s", exc)
            return False

    def predict(
        self,
        image_bgr: np.ndarray,
    ) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray, Any, Any]]:
        """发送图像到 worker 进程进行推理，阻塞等待结果（但释放 GIL）。

        Returns:
            (xyxy, conf, cls, names, masks_data) 或 None（推理失败时）
        """
        if not self._ready or self._process is None or not self._process.is_alive():
            return None

        # 确保共享内存足够大
        needed = image_bgr.nbytes
        if self._shm is None or self._shm_size < needed:
            if self._shm is not None:
                try:
                    self._shm.close()
                    self._shm.unlink()
                except Exception:
                    pass
            self._shm = shared_memory.SharedMemory(create=True, size=needed)
            self._shm_size = needed

        # 拷贝图像到共享内存
        shm_array = np.ndarray(image_bgr.shape, dtype=image_bgr.dtype, buffer=self._shm.buf)
        np.copyto(shm_array, image_bgr)

        self._req_counter += 1
        req_id = self._req_counter
        self._request_queue.put(
            (self._shm.name, image_bgr.shape, str(image_bgr.dtype), req_id)
        )

        # 等待结果（Queue.get 阻塞时释放 GIL）
        try:
            msg = self._response_queue.get(timeout=10.0)
            if msg[0] == "result" and msg[1] == req_id:
                raw = msg[2]
                if raw and len(raw) >= 3:
                    return (
                        raw[0],
                        raw[1],
                        raw[2],
                        raw[3] if len(raw) > 3 else None,
                        raw[4] if len(raw) > 4 else None,
                    )
                return None
            if msg[0] == "error":
                logger.warning(
                    "YOLO worker 推理错误: %s",
                    msg[2] if len(msg) > 2 else msg,
                )
                return None
        except Exception as exc:
            logger.warning("等待 YOLO worker 响应超时: %s", exc)
        return None

    def stop(self) -> None:
        """停止 worker 进程并释放共享内存。"""
        if self._request_queue is not None:
            try:
                self._request_queue.put(None, timeout=2.0)
            except Exception:
                pass
        if self._process is not None:
            self._process.join(timeout=5.0)
            if self._process.is_alive():
                self._process.terminate()
            self._process = None
        if self._shm is not None:
            try:
                self._shm.close()
                self._shm.unlink()
            except Exception:
                pass
            self._shm = None
        self._ready = False

    @property
    def is_ready(self) -> bool:
        """worker 进程是否就绪且存活。"""
        return self._ready and self._process is not None and self._process.is_alive()
