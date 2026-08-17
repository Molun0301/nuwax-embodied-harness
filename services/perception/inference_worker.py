"""YOLO 推理 Worker 进程 — 隔离 GPU 推理以释放主线程 GIL。"""

from __future__ import annotations

import atexit
import logging
import multiprocessing as mp
import os
import queue
import sys
import threading
import time
from multiprocessing import resource_tracker, shared_memory
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)


def _configure_spawn_executable() -> None:
    """确保修改进程标题后仍能启动 multiprocessing 子进程。"""
    executable = (sys.executable or "").strip()
    if not executable and sys.platform.startswith("linux"):
        try:
            executable = os.path.realpath("/proc/self/exe")
        except OSError:
            executable = ""
    if not executable or not os.path.isfile(executable):
        raise RuntimeError("无法确定 YOLO worker 使用的 Python 解释器")
    mp.set_executable(executable)


def _worker_loop(
    request_queue: mp.Queue,
    response_queue: mp.Queue,
    model_source: str,
    predict_kwargs: Dict[str, Any],
) -> None:
    """Worker 进程主循环：加载模型，等待推理请求，返回结果。"""
    try:
        import os

        os.environ.setdefault("CUDA_VISIBLE_DEVICES", "0")
        from ultralytics import YOLO

        # spawn 子进程不继承主进程的 torchvision NMS 回退。
        try:
            from services.perception.detectors import _ensure_torchvision_nms_fallback

            _ensure_torchvision_nms_fallback()
        except Exception as exc:  # noqa: BLE001 — 回退安装失败不致命。
            logger.warning("YOLO worker torchvision NMS 回退安装失败，忽略：%s", exc)

        model = YOLO(model_source)
        try:
            warmup_imgsz = int(predict_kwargs.get("imgsz", 640) or 640)
            warmup = getattr(model, "warmup", None)
            if callable(warmup):
                warmup(imgsz=(1, 3, warmup_imgsz, warmup_imgsz))
        except Exception as exc:  # noqa: BLE001 — warmup 失败不影响推理。
            logger.warning("YOLO worker warmup 失败，忽略：%s", exc)
    except Exception as exc:  # noqa: BLE001 — 启动失败必须立即回传父进程。
        response_queue.put(("startup_error", "bootstrap", type(exc).__name__, str(exc)))
        return

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
        self._start_lock = threading.RLock()
        self._stop_lock = threading.Lock()
        self._predict_lock = threading.Lock()
        self._readiness_condition = threading.Condition()
        self._readiness_state = "new"
        self._readiness_owner = False
        self._readiness_failure: Optional[str] = None
        # vis-three-fixes 任务 3.8：最近一次 worker 推理往返耗时（毫秒），供观测透传。
        self._last_inference_ms: Optional[float] = None
        self._atexit_registered: bool = False

    def start(self) -> bool:
        """启动 worker 进程并等待模型加载完成。"""
        if not self.start_nonblocking():
            return False
        return self.wait_until_ready()

    def start_nonblocking(self) -> bool:
        """只创建 worker 进程，不等待模型加载。必须由主线程调用。"""
        with self._start_lock:
            with self._readiness_condition:
                if self._readiness_state in ("starting", "ready"):
                    return True
            if self._process is not None:
                try:
                    if self._process.is_alive():
                        return True
                except Exception:
                    pass
            if any(
                resource is not None
                for resource in (
                    self._process,
                    self._request_queue,
                    self._response_queue,
                    self._shm,
                )
            ):
                self.stop()
                if any(
                    resource is not None
                    for resource in (
                        self._process,
                        self._request_queue,
                        self._response_queue,
                        self._shm,
                    )
                ):
                    logger.error("YOLO worker 旧资源仍被占用，拒绝启动新进程")
                    return False
            try:
                # resource_tracker 和 spawn 必须在 Gateway 主线程创建；模型加载与
                # 推理仍全部留在 worker 进程，调用方可异步等待 ready。exec -a 会让
                # Python 3.8 的 sys.executable 变为空，先从 /proc 恢复真实解释器。
                _configure_spawn_executable()
                resource_tracker.ensure_running()
                context = mp.get_context("spawn")
                self._request_queue = context.Queue(maxsize=2)
                self._response_queue = context.Queue(maxsize=2)
                self._process = context.Process(
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
            except Exception as exc:
                logger.error("启动 YOLO worker 进程失败: %s", exc)
                with self._readiness_condition:
                    self._readiness_state = "failed"
                    self._readiness_failure = str(exc)
                    self._readiness_owner = False
                    self._readiness_condition.notify_all()
                self.stop()
                return False
            with self._readiness_condition:
                self._ready = False
                self._readiness_state = "starting"
                self._readiness_failure = None
                self._readiness_owner = False
                self._readiness_condition.notify_all()
            self._register_atexit_cleanup()
            return True

    def wait_until_ready(self, timeout_sec: float = 120.0) -> bool:
        """等待模型加载；并发调用时只有一个 owner 消费就绪队列。"""
        with self._readiness_condition:
            if self._readiness_state == "ready":
                return True
            if self._readiness_state != "starting":
                return False
            if self._readiness_owner:
                while self._readiness_state == "starting":
                    self._readiness_condition.wait()
                return self._readiness_state == "ready"
            self._readiness_owner = True

        deadline = time.monotonic() + max(0.0, timeout_sec)
        failure_reason = ""
        while time.monotonic() < deadline:
            process = self._process
            response_queue = self._response_queue
            if process is None or response_queue is None:
                failure_reason = "worker_resources_missing"
                break
            try:
                msg = response_queue.get(
                    timeout=min(0.25, max(0.01, deadline - time.monotonic()))
                )
            except queue.Empty:
                try:
                    if process.is_alive():
                        continue
                    failure_reason = "worker_exited:%s" % process.exitcode
                except Exception as exc:
                    failure_reason = "worker_state_error:%s" % exc
                break
            except Exception as exc:
                failure_reason = "ready_queue_error:%s" % exc
                break

            if msg[0] == "ready":
                with self._readiness_condition:
                    self._ready = True
                    self._readiness_state = "ready"
                    self._readiness_owner = False
                    self._readiness_condition.notify_all()
                logger.info("YOLO 推理 worker 进程已就绪 pid=%d", process.pid)
                return True
            if msg[0] == "startup_error" and len(msg) >= 4:
                failure_reason = "%s:%s:%s" % (msg[1], msg[2], msg[3])
                logger.error(
                    "YOLO 推理 worker 启动失败 stage=%s error=%s: %s",
                    msg[1],
                    msg[2],
                    msg[3],
                )
            else:
                failure_reason = "unexpected_startup_response:%r" % (msg,)
                logger.error("YOLO 推理 worker 启动失败: %s", msg)
            break

        if not failure_reason:
            failure_reason = "startup_timeout:%.1fs" % timeout_sec
            logger.error("等待 YOLO worker 就绪超时: %.1fs", timeout_sec)
        elif failure_reason.startswith("worker_exited:"):
            logger.error("YOLO worker 在就绪前退出 %s", failure_reason)
        with self._readiness_condition:
            if self._readiness_state == "starting":
                self._ready = False
                self._readiness_state = "failed"
                self._readiness_failure = failure_reason
            self._readiness_owner = False
            self._readiness_condition.notify_all()
        self.stop()
        return False

    def predict(
        self,
        image_bgr: np.ndarray,
    ) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray, Any, Any]]:
        """发送图像到 worker 进程进行推理，阻塞等待结果（但释放 GIL）。

        Returns:
            (xyxy, conf, cls, names, masks_data) 或 None（推理失败时）
        """
        with self._predict_lock:
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
            inference_started_at = time.perf_counter()
            self._request_queue.put(
                (self._shm.name, image_bgr.shape, str(image_bgr.dtype), req_id)
            )

            # 等待结果（Queue.get 阻塞时释放 GIL）
            try:
                msg = self._response_queue.get(timeout=10.0)
                if msg[0] == "result" and msg[1] == req_id:
                    # 任务 3.8：记录 worker 推理往返耗时（毫秒）。
                    self._last_inference_ms = round((time.perf_counter() - inference_started_at) * 1000.0, 3)
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
        """停止 worker 进程并释放共享内存。并发调用只执行一次清理。"""
        with self._start_lock:
            with self._stop_lock:
                with self._predict_lock:
                    self._stop_locked()

    def _stop_locked(self) -> None:
        request_queue = self._request_queue
        response_queue = self._response_queue
        process = self._process
        shm = self._shm
        with self._readiness_condition:
            self._ready = False
            if self._readiness_state != "failed":
                self._readiness_state = "stopped"
            self._readiness_owner = False
            self._readiness_condition.notify_all()

        if request_queue is not None:
            try:
                request_queue.put(None, timeout=2.0)
            except Exception as exc:
                logger.warning("发送 YOLO worker 关闭信号失败: %s", exc)
        if process is not None:
            try:
                process.join(timeout=5.0)
            except Exception as exc:
                logger.warning("等待 YOLO worker 进程退出失败: %s", exc)
            try:
                process_alive = process.is_alive()
            except Exception as exc:
                logger.warning("确认 YOLO worker 进程状态失败: %s", exc)
                process_alive = True
            if process_alive:
                try:
                    process.terminate()
                except Exception as exc:
                    logger.warning("终止 YOLO worker 进程失败: %s", exc)
                try:
                    process.join(timeout=1.0)
                except Exception as exc:
                    logger.warning("等待已终止 YOLO worker 进程失败: %s", exc)
            try:
                process_alive = process.is_alive()
            except Exception as exc:
                logger.warning("确认 YOLO worker 进程状态失败: %s", exc)
                process_alive = True
            if process_alive:
                try:
                    process.kill()
                except Exception as exc:
                    logger.warning("强制终止 YOLO worker 进程失败: %s", exc)
                try:
                    process.join(timeout=1.0)
                except Exception as exc:
                    logger.warning("等待已强制终止 YOLO worker 进程失败: %s", exc)
            try:
                process_exited = not process.is_alive()
            except Exception as exc:
                logger.warning("确认 YOLO worker 进程状态失败: %s", exc)
                process_exited = False
            if not process_exited:
                logger.error("YOLO worker 进程拒绝退出，保留资源以便后续重试")
                return
            try:
                process.close()
            except Exception as exc:
                logger.warning("关闭 YOLO worker 进程句柄失败: %s", exc)
        self._process = None

        if shm is not None:
            try:
                shm.close()
            except Exception as exc:
                logger.warning("关闭 YOLO worker 共享内存失败 name=%s: %s", shm.name, exc)
            try:
                shm.unlink()
            except FileNotFoundError:
                pass
            except Exception as exc:
                logger.warning("删除 YOLO worker 共享内存失败 name=%s: %s", shm.name, exc)
        self._shm = None
        self._shm_size = 0

        for queue_name, queue in (
            ("request", request_queue),
            ("response", response_queue),
        ):
            if queue is None:
                continue
            try:
                queue.close()
                queue.join_thread()
            except Exception as exc:
                logger.warning("关闭 YOLO worker %s queue 失败: %s", queue_name, exc)
        self._request_queue = None
        self._response_queue = None
        self._unregister_atexit_cleanup()

    def _register_atexit_cleanup(self) -> None:
        if self._atexit_registered:
            return
        atexit.register(self.stop)
        self._atexit_registered = True

    def _unregister_atexit_cleanup(self) -> None:
        if not self._atexit_registered:
            return
        atexit.unregister(self.stop)
        self._atexit_registered = False

    def __del__(self) -> None:
        try:
            self.stop()
        except Exception:
            # 析构和解释器退出阶段只能做 best-effort 清理。
            pass

    @property
    def is_ready(self) -> bool:
        """worker 进程是否就绪且存活。"""
        with self._readiness_condition:
            ready = self._readiness_state == "ready" and self._ready
            process = self._process
        return ready and process is not None and process.is_alive()

    @property
    def last_inference_ms(self) -> Optional[float]:
        """最近一次 worker 推理往返耗时（毫秒），无则 None。vis-three-fixes 任务 3.8。"""
        return self._last_inference_ms
