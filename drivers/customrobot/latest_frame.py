"""把可跳帧的重计算移到独立进程，设备接收只替换一张待处理帧。"""

from __future__ import annotations

import multiprocessing
import threading
import time


def _process_frames(connection, process_frame):
    """子进程只运行帧算法；提供器与订阅者始终由父进程持有。"""
    try:
        while True:
            args, kwargs = connection.recv()
            started = time.monotonic()
            try:
                result = process_frame(*args, **kwargs)
            except Exception as exc:
                connection.send((False, exc, time.monotonic() - started))
            else:
                connection.send((True, result, time.monotonic() - started))
    except EOFError:
        return
    finally:
        connection.close()


class LatestFrameProcessor:
    """一张运行中、一张待处理；算法和结果分发均不占设备消费线程。

    process_frame 必须可由 spawn（新解释器进程）导入；参数中的帧发布后不得
    再修改。调用方在 submit 时捕获同帧所需参考，工作进程不读取最新提供器。
    on_result 和 on_error 在独立分发线程执行，close 会取消未完成的计算。
    """

    def __init__(self, process_frame, on_result, on_error, *, name="device-frame"):
        self._process_frame = process_frame
        self._on_result = on_result
        self._on_error = on_error
        self._name = name
        self._condition = threading.Condition()
        self._pending = None
        self._closed = False
        self._thread = None
        self._process = None
        self._active = False
        self._stats = dict(received_count=0, processed_count=0, replaced_count=0,
                           cancelled_count=0, error_count=0,
                           last_processing_seconds=0.0, max_processing_seconds=0.0)

    def submit(self, *args, **kwargs) -> bool:
        with self._condition:
            if self._closed:
                return False
            self._stats["received_count"] += 1
            if self._pending is not None:
                self._stats["replaced_count"] += 1
            self._pending = (args, kwargs)
            if self._thread is None:
                self._thread = threading.Thread(target=self._run, name=self._name, daemon=True)
                self._thread.start()
            self._condition.notify_all()
            return True

    def snapshot(self) -> dict:
        with self._condition:
            return {**self._stats, "pending_count": int(self._pending is not None),
                    "processing": self._active, "closed": self._closed}

    def close(self) -> None:
        with self._condition:
            self._closed = True
            if self._pending is not None:
                self._stats["cancelled_count"] += 1
                self._pending = None
            process, thread = self._process, self._thread
            self._condition.notify_all()
        # 原生算法可能持有子进程解释器锁；终止它以取消计算，不等待慢帧完成。
        if process is not None and process.is_alive():
            process.terminate()
        if thread is not None and thread is not threading.current_thread():
            thread.join()

    def _run(self) -> None:
        context = multiprocessing.get_context("spawn")
        connection, child_connection = context.Pipe()
        process = context.Process(target=_process_frames,
                                  args=(child_connection, self._process_frame),
                                  name=self._name, daemon=True)
        try:
            process.start()
            child_connection.close()
            with self._condition:
                self._process = process
            while True:
                with self._condition:
                    while not self._closed and self._pending is None:
                        self._condition.wait()
                    if self._closed:
                        return
                    item, self._pending = self._pending, None
                    self._active = True
                connection.send(item)
                success, value, elapsed = connection.recv()
                with self._condition:
                    if self._closed:
                        return
                    self._stats["last_processing_seconds"] = elapsed
                    self._stats["max_processing_seconds"] = max(
                        self._stats["max_processing_seconds"], elapsed)
                if success:
                    self._on_result(value)
                else:
                    self._on_error(value)
                with self._condition:
                    self._stats["processed_count" if success else "error_count"] += 1
                    self._active = False
        except Exception as exc:
            with self._condition:
                closed = self._closed
            if not closed:
                self._on_error(exc)
                with self._condition:
                    self._stats["error_count"] += 1
                    self._active = False
        finally:
            with self._condition:
                self._closed = True
                if self._active:
                    self._stats["cancelled_count"] += 1
                    self._active = False
                if self._pending is not None:
                    self._stats["cancelled_count"] += 1
                    self._pending = None
            if process.pid is not None:
                if process.is_alive():
                    process.terminate()
                process.join()
            connection.close()
            child_connection.close()
