"""Go2采集进程：完整标准输入与可替代显示分路，网页负载不占用DDS解释器。"""
from __future__ import annotations

from collections import deque
import multiprocessing as mp
import os
import sys
import threading
import time


_MAX_QUEUE_BYTES = 8 * 1024 * 1024
_MAX_QUEUE_SAMPLES = 512
_MAX_QUEUE_WAIT_SEC = 1.0


def _sample_size(sample):
    return 1024 + sum(len(getattr(sample, key, None) or b"") for key in (
        "xyz_f32_le", "intensity_f32_le", "ring_u16_le", "time_offset_ns_le",
    ))


def _default_source(config):
    from drivers.go2.data_plane import Go2DirectDdsBridge
    return Go2DirectDdsBridge(config)


def _sensor_process_main(config, data, status, commands, stop, source_factory, forward_samples):
    """此进程只拥有传感器，不创建运动客户端或网关业务。"""
    source = None
    pending = deque()
    condition = threading.Condition()
    queued_bytes = 0
    failure = None
    sender = None
    delivery = None
    local_delivery_selected = False
    delivery_sequence = 0
    delivery_lock = threading.RLock()
    recording_dropped = 0
    snapshot_condition = threading.Condition()
    latest_snapshot = None
    latest_pose_metadata = {}
    status_send_lock = threading.Lock()

    def send_status(kind, payload):
        with status_send_lock:
            status.send((kind, payload))

    def observe_pose(pose, frame_tree, metadata):
        nonlocal latest_pose_metadata
        latest_pose_metadata = {key: value for key, value in metadata.items() if key != "canonical_imu_sample"}

    def record(kind, value):
        nonlocal queued_bytes, recording_dropped
        size = _sample_size(value) if kind == "sensor" else 4096
        with condition:
            if queued_bytes + size > _MAX_QUEUE_BYTES or len(pending) >= _MAX_QUEUE_SAMPLES:
                recording_dropped += 1
                return False
            pending.append(((kind, value), time.monotonic(), size))
            queued_bytes += size
            condition.notify()
            return True

    def send_snapshots():
        nonlocal latest_snapshot
        try:
            while not stop.is_set():
                with snapshot_condition:
                    snapshot_condition.wait_for(lambda: latest_snapshot is not None or stop.is_set(), timeout=.1)
                    snapshot, latest_snapshot = latest_snapshot, None
                if snapshot is not None:
                    send_status("snapshot", snapshot)
        except (EOFError, OSError):
            pass

    def fail(message, timestamp_ns=0):
        nonlocal failure
        with condition:
            if failure is None:
                failure = (message, timestamp_ns)
            condition.notify_all()

    def receive(sample):
        nonlocal queued_bytes
        with delivery_lock:
            if delivery is not None:
                delivery.receive(sample)
                return
            if local_delivery_selected or not forward_samples:
                return
        size = _sample_size(sample)
        with condition:
            if failure is not None or stop.is_set():
                return
            if len(pending) >= _MAX_QUEUE_SAMPLES or queued_bytes + size > _MAX_QUEUE_BYTES:
                fail("采集进程标准输入队列已满，停止交付以保留真实缺口", sample.timestamp_ns)
                return
            pending.append((sample, time.monotonic(), size))
            queued_bytes += size
            condition.notify()

    def send_samples():
        nonlocal queued_bytes
        try:
            while not stop.is_set():
                with condition:
                    condition.wait_for(lambda: pending or failure is not None or stop.is_set(), timeout=.1)
                    if failure is not None or stop.is_set():
                        return
                    batch = []
                    while pending and len(batch) < 64:
                        batch.append(pending.popleft())
                if not batch:
                    continue
                if not local_delivery_selected and time.monotonic() - batch[0][1] > _MAX_QUEUE_WAIT_SEC:
                    fail("采集进程标准输入交付积压超过1秒", batch[0][0].timestamp_ns)
                    return
                # 先排队再序列化，DDS回调不等待套接字和网页消费者。
                data.send([(sample, at) for sample, at, _ in batch])
                with condition:
                    queued_bytes -= sum(size for _, _, size in batch)
        except (EOFError, OSError) as error:
            fail("采集进程标准输入通道失败：%s" % error)

    try:
        source = source_factory(config)
        source.subscribe_sensor_sample(receive)
        subscribe_pose = getattr(source, "subscribe_localization", None)
        if callable(subscribe_pose):
            subscribe_pose(observe_pose)
        sender = threading.Thread(target=send_samples, name="go2-input-send", daemon=True)
        sender.start()
        source.start()
        send_status("ready", os.getpid())
        threading.Thread(target=send_snapshots, name="go2-snapshot-send", daemon=True).start()
        previous_pose = previous_cloud = None
        while not stop.wait(.05):
            if commands.poll():
                command_id, operation, arguments = commands.recv()
                try:
                    if operation == "start":
                        factory, spec = arguments
                        with delivery_lock:
                            if delivery is not None:
                                raise RuntimeError("输入交付已启动")
                        # 连接期间尚未订阅输入，不能持锁阻塞DDS读取并积累旧样本。
                        candidate = factory(spec, record)
                        with delivery_lock:
                            delivery = candidate
                            local_delivery_selected = True
                        result = delivery.get_status()
                        delivery_sequence = command_id
                    elif operation == "stop":
                        with delivery_lock:
                            if delivery is not None:
                                delivery.stop()
                                delivery = None
                        result = {"started": False, "ready": False, "recording_transport_dropped": recording_dropped}
                        delivery_sequence = command_id
                        # 排空标记不能因记录满队列而丢失；不占用传感器交付锁。
                        with condition:
                            pending.append((("drained", command_id), time.monotonic(), 0))
                            condition.notify()
                    else:
                        raise ValueError("未知采集进程命令")
                    commands.send((command_id, True, result))
                except Exception as error:
                    commands.send((command_id, False, str(error)))
            with condition:
                problem = failure
                depth, size = len(pending), queued_bytes
            if problem is not None:
                send_status("error", problem)
                return
            pose, imu = source.get_current_pose(), source.get_imu_state()
            cloud = source.get_latest_local_point_cloud()
            pose_stamp = pose.timestamp if pose is not None else None
            cloud_stamp = cloud.timestamp if cloud is not None else None
            # 仅显示状态可合并；300Hz惯性和每帧雷达始终走上面的完整通道。
            snapshot = {
                "pose": pose if pose_stamp != previous_pose else None,
                "imu": imu if pose_stamp != previous_pose else None,
                "cloud": cloud if cloud_stamp != previous_cloud else None,
                "status": source.get_status(), "pending_samples": depth, "pending_bytes": size,
                "pose_metadata": latest_pose_metadata,
                "input_delivery": delivery.get_status() if delivery is not None else None,
                "delivery_sequence": delivery_sequence,
                "recording_dropped": recording_dropped,
            }
            with snapshot_condition:
                previous = latest_snapshot or {}
                for key in ("pose", "imu", "cloud"):
                    if snapshot[key] is None:
                        snapshot[key] = previous.get(key)
                latest_snapshot = snapshot
                snapshot_condition.notify()
            previous_pose, previous_cloud = pose_stamp, cloud_stamp
    except BaseException as error:
        try:
            send_status("error", ("采集进程失败：%s" % error, 0))
        except (EOFError, OSError):
            pass
    finally:
        stop.set()
        with condition:
            condition.notify_all()
        if source is not None:
            source.stop()
        if delivery is not None:
            delivery.stop()
        if sender is not None:
            sender.join(timeout=.5)
        data.close()
        status.close()
        commands.close()


class Go2SensorProcess:
    """父进程拥有子进程及交付线程，停止时撤销代际并回收真实进程。"""

    def __init__(self, config, *, on_sample, on_snapshot, on_error, source_factory=_default_source, forward_samples=True):
        self.config = config
        self.on_sample, self.on_snapshot, self.on_error = on_sample, on_snapshot, on_error
        self.source_factory = source_factory
        self.forward_samples = forward_samples
        self._context = mp.get_context("spawn")
        self._stop = self._context.Event()
        self._closing = threading.Event()
        self._process = None
        self._threads = []
        self._data = self._status = None
        self._commands = None
        self._command_lock = threading.Lock()
        self._command_sequence = 0
        self._drain_condition = threading.Condition()
        self._drained_sequence = 0
        self._delivery_status = None
        self._delivery_sequence = 0
        self._recording_sink = None
        self.recording_dropped = 0
        self._snapshot_condition = threading.Condition()
        self._latest_snapshot = None
        self._failure_lock = threading.Lock()
        self._failed = False
        self._recording_loss_lock = threading.Lock()
        self.pid = None

    def start(self):
        executable = sys.executable or os.path.realpath("/proc/self/exe")
        mp.set_executable(executable)
        self._data, child_data = self._context.Pipe(duplex=False)
        self._status, child_status = self._context.Pipe(duplex=False)
        self._commands, child_commands = self._context.Pipe()
        self._process = self._context.Process(target=_sensor_process_main,
            args=(self.config, child_data, child_status, child_commands, self._stop, self.source_factory, self.forward_samples), name="go2-sensor-input", daemon=True)
        try:
            self._process.start()
            child_data.close(); child_status.close()
            child_commands.close()
            if not self._status.poll(10):
                raise RuntimeError("Go2采集进程启动超时")
            kind, payload = self._status.recv()
            if kind != "ready":
                raise RuntimeError(payload[0] if kind == "error" else "Go2采集进程启动回执非法")
            self.pid = payload
            self._threads = [
                threading.Thread(target=self._receive_data, name="go2-standard-input", daemon=True),
                threading.Thread(target=self._receive_status, name="go2-input-status", daemon=True),
                threading.Thread(target=self._deliver_snapshot, name="go2-display-input", daemon=True),
            ]
            for thread in self._threads:
                thread.start()
        except BaseException:
            child_data.close(); child_status.close()
            child_commands.close()
            self.close()
            raise

    def _fail(self, message, timestamp_ns=0):
        with self._failure_lock:
            if self._failed or self._closing.is_set():
                return
            self._failed = True
        self._stop.set()
        self.on_error(message, timestamp_ns)

    def _receive_data(self):
        try:
            while not self._closing.is_set():
                batch = self._data.recv()
                for sample, received_at in batch:
                    if self._closing.is_set() or self._failed:
                        return
                    if isinstance(sample, tuple):
                        kind, value = sample
                        if kind == "drained":
                            with self._drain_condition:
                                self._drained_sequence = value
                                self._drain_condition.notify_all()
                            continue
                        sink = self._recording_sink
                        if sink is not None:
                            try:
                                if kind == "sensor":
                                    sink.record_sensor(value)
                                else:
                                    event, metadata, stamp = value
                                    sink.record_event(event, metadata, source_ns=stamp)
                            except Exception:
                                import logging
                                logging.getLogger(__name__).exception("独立输入记录交付失败")
                        continue
                    if self._delivery_status is not None:
                        continue
                    if time.monotonic() - received_at > _MAX_QUEUE_WAIT_SEC:
                        self._fail("标准输入跨进程交付积压超过1秒", sample.timestamp_ns)
                        return
                    self.on_sample(sample)
        except (EOFError, OSError):
            self._fail("Go2采集进程标准输入通道已关闭")
        except Exception as error:
            self._fail("标准输入消费者异常：%s" % error)

    def _receive_status(self):
        try:
            while not self._closing.is_set():
                kind, payload = self._status.recv()
                if self._closing.is_set():
                    return
                if kind == "error":
                    self._fail(*payload)
                    return
                if kind == "snapshot":
                    if payload.get("input_delivery") is not None and payload.get("delivery_sequence") == self._delivery_sequence:
                        self._delivery_status = payload["input_delivery"]
                    self._report_recording_dropped(payload.get("recording_dropped", 0))
                    with self._snapshot_condition:
                        # 同一通道的快照已是完整的最近状态，慢显示只保留最近一份。
                        previous = self._latest_snapshot or {}
                        for key in ("pose", "imu", "cloud"):
                            if payload[key] is None:
                                payload[key] = previous.get(key)
                        self._latest_snapshot = payload
                        self._snapshot_condition.notify()
        except (EOFError, OSError):
            self._fail("Go2采集进程状态通道已关闭")
        except Exception as error:
            self._fail("采集显示消费者异常：%s" % error)

    def _deliver_snapshot(self):
        while not self._closing.is_set():
            with self._snapshot_condition:
                self._snapshot_condition.wait_for(lambda: self._latest_snapshot is not None or self._closing.is_set())
                snapshot, self._latest_snapshot = self._latest_snapshot, None
            if snapshot is not None and not self._closing.is_set():
                try:
                    self.on_snapshot(snapshot)
                except Exception:
                    # 显示失败不影响完整输入，下一份快照仍可恢复。
                    import logging
                    logging.getLogger(__name__).exception("Go2显示快照交付失败")

    def start_input_delivery(self, factory, spec, recording_sink=None):
        self._recording_sink = recording_sink
        self._delivery_status = {"ready": False, "started": False}
        self._delivery_status = self._command("start", (factory, spec))
        self._delivery_sequence = self._command_sequence
        return self._delivery_status

    def _report_recording_dropped(self, dropped):
        with self._recording_loss_lock:
            report = getattr(self._recording_sink, "record_transport_loss", None)
            if dropped > self.recording_dropped:
                if callable(report):
                    try:
                        report(dropped - self.recording_dropped)
                    except Exception:
                        import logging
                        logging.getLogger(__name__).exception("独立输入记录缺口报告失败")
                self.recording_dropped = dropped

    def stop_input_delivery(self):
        if self._process is not None and not self._process.is_alive():
            self._process.join()
            self._delivery_status = {"started": False, "ready": False}
            return
        self._delivery_status = self._command("stop", None)
        self._delivery_sequence = self._command_sequence
        self._report_recording_dropped(self._delivery_status.get("recording_transport_dropped", 0))
        with self._drain_condition:
            if not self._drain_condition.wait_for(lambda: self._drained_sequence >= self._command_sequence, timeout=10):
                raise RuntimeError("独立输入记录尚未排空")

    def input_delivery_status(self):
        result = dict(self._delivery_status or {})
        result["sensor_process_pid"] = self.pid
        result["recording_transport_dropped"] = self.recording_dropped
        if self._failed or self._process is None or not self._process.is_alive():
            result.update(ready=False, runtime_unavailable=True)
        return result

    def _command(self, operation, arguments):
        with self._command_lock:
            if self._closing.is_set() or self._failed:
                raise RuntimeError("采集进程已关闭或失败")
            self._command_sequence += 1
            sequence = self._command_sequence
            self._commands.send((sequence, operation, arguments))
            deadline = time.monotonic() + 15
            while True:
                if not self._commands.poll(max(0, deadline - time.monotonic())):
                    raise RuntimeError("采集进程命令超时：%s" % operation)
                response_id, success, result = self._commands.recv()
                if response_id != sequence:
                    continue
                if not success:
                    raise RuntimeError(result)
                return result

    def close(self):
        self._closing.set()
        with self._snapshot_condition:
            self._latest_snapshot = None
            self._snapshot_condition.notify_all()
        self._stop.set()
        process = self._process
        if process is not None and process.pid is not None:
            process.join(timeout=3)
            if process.is_alive():
                process.terminate()
                process.join(timeout=2)
            if process.is_alive():
                process.kill()
                process.join(timeout=2)
            if process.is_alive():
                raise RuntimeError("Go2采集进程未能回收")
        for thread in self._threads:
            if thread is not threading.current_thread():
                thread.join(timeout=2)
        for connection in (self._data, self._status, self._commands):
            if connection is not None:
                connection.close()
        pending = [thread.name for thread in self._threads
                   if thread is not threading.current_thread() and thread.is_alive()]
        if pending:
            raise RuntimeError("采集进程交付线程尚未退出：%s" % ", ".join(pending))
