"""在采集进程中配组和交付；网关仅拥有生命周期代理及观察快照。"""
from types import SimpleNamespace


class _RecordingRelay:
    def __init__(self, emit):
        self.emit = emit

    def record_sensor(self, sample):
        return self.emit("sensor", sample)

    def record_event(self, kind, metadata, *, source_ns=0):
        return self.emit("event", (kind, metadata, source_ns))


class _InputDelivery:
    def __init__(self, spec, emit):
        from services.geometry.geometry_runtime_provider import GeometryRuntimeProvider
        from services.geometry.runtime_input_bridge import RuntimeInputBridge

        self.callback = None
        # 此连接只拥有远程接口，不创建或关闭几何运行时进程。
        endpoint = SimpleNamespace(is_running=True, grpc_target=spec["target"], disconnect=lambda: None)
        self.provider = GeometryRuntimeProvider(endpoint)
        self.bridge = RuntimeInputBridge(runtime_provider=self.provider, sensor_sample_provider=self,
            recording_sink=_RecordingRelay(emit) if spec["recording"] else None)
        try:
            if not self.provider.connect() or not self.bridge.start() or not self.bridge.is_ready():
                raise RuntimeError("独立标准输入交付启动失败")
        except BaseException:
            self.stop()
            raise

    def subscribe_sensor_sample(self, callback):
        self.callback = callback
        return 1

    def unsubscribe_sensor_sample(self, subscription_id):
        self.callback = None

    def receive(self, sample):
        callback = self.callback
        if callback is not None:
            callback(sample)

    def get_status(self):
        return self.bridge.get_status()

    def stop(self):
        self.bridge.stop()
        self.provider.disconnect()


class ProcessInputBridge:
    """与普通输入桥保持相同拥有方接口，关闭必须等待子进程真实排空。"""

    def __init__(self, runtime_provider, executor, recording_sink=None):
        self.runtime_provider = runtime_provider
        self.executor = executor
        self.recording_sink = recording_sink
        self._started = False

    def start(self):
        if self._started:
            return self.is_ready()
        if not self.runtime_provider.is_connected:
            return False
        spec = {"target": self.runtime_provider._bridge.grpc_target,
                "recording": self.recording_sink is not None}
        # 调用失败也保留关闭拥有权，远端可能已经开始交付。
        self._started = True
        status = self.executor.start_input_delivery(_InputDelivery, spec, self.recording_sink)
        return bool(status.get("ready"))

    def stop(self):
        if self._started:
            self.executor.stop_input_delivery()
            self._started = False

    def is_ready(self):
        return self._started and bool(self.executor.input_delivery_status().get("ready"))

    def get_status(self):
        return self.executor.input_delivery_status()
