"""real LCM `/cmd_vel` 订阅者 + 200ms watchdog zero-stop 桥。

设计目标(00_最终执行方案.md §5 Phase 6 + §12.5 + §13.3)
1. 独立 daemon thread `cmd_receiver_thread` + `watchdog_thread`,
   不依赖 Rust runtime event loop 或 provider 存活。
2. atomic 状态:`last_cmd_ts_ns` / `last_zero_sent_ts_ns` / `cmd_count` /
   `dropped_count`;`last_error` 用 Mutex 保护。
3. 收到 `/cmd_vel` 后:
   decode 32-byte Twist payload -> clamp 到 Go2 motion 安全上下限 ->
   `last_cmd_ts_ns = now_ns` -> `twist_sink(twist)`。
4. watchdog 每 50ms tick;距离 `last_cmd_ts_ns` 超过 200ms 时下发 zero twist。
   首次超时 zero 必须在 200ms-250ms 窗口内发生;重复 zero 限流。
5. shutdown 顺序:zero-stop -> unsubscribe LCM -> 停 watchdog -> 停 receiver。

模块位置:`services/navigation/`(与 §5 Phase 6 任务 1 一致)。
禁止嵌入 `geometry provider` —— provider 跟 gRPC server 一起可能挂掉。

real LCM Python binding 不可用时:fallback 到 `MockLcmBackend`
(单元测试与 CI dry-run 使用),并在 `last_error` 写入
"NUWAX_LCM_PYTHON_AVAILABLE=false"。
"""
from __future__ import annotations

import logging
import os
import queue
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

logger = logging.getLogger(__name__)

# ============================================================
# 32-byte typed payload 布局(镜像 typed_codec.rs encode_cmd_vel_payload)
# linear.x/y/z (3*f32 LE) + angular.x/y/z (3*f32 LE) + stamp_ns (u64 LE)
# ============================================================
CMD_VEL_PAYLOAD_BYTES = 32
_CMD_VEL_STRUCT = struct.Struct("<6fQ")  # 6 floats + 1 u64 = 32 bytes

# 默认上下限(方案默认值,Go2 config 缺省时使用)
DEFAULT_MAX_LINEAR_M_S = 0.6
DEFAULT_MAX_ANGULAR_RAD_S = 1.0

# watchdog 时序
WATCHDOG_TIMEOUT_S = 0.200  # 200ms 无新 cmd 必须 zero
WATCHDOG_TICK_S = 0.050     # 50ms 检查一次
WATCHDOG_ZERO_GAP_S = 0.020 # 同一超时窗口内零位重复限流(默认 20ms,留出 200-250ms 首次窗口)


# ============================================================
# Twist payload codec(与 Rust typed_codec.rs encode_cmd_vel_payload 对称)
# ============================================================
def decode_cmd_vel_payload(payload: bytes) -> Optional[tuple]:
    """解码 32-byte typed payload 为 (linear[3], angular[3], stamp_ns)。

    返回 None 表示 payload 长度不符或字段非法。frame_id 不在 payload 中,
    与 typed_codec 一致(caller 注入默认 'base_link')。
    """
    if len(payload) != CMD_VEL_PAYLOAD_BYTES:
        return None
    try:
        linear_x, linear_y, linear_z, ang_x, ang_y, ang_z, stamp_ns = _CMD_VEL_STRUCT.unpack(payload)
    except struct.error:
        return None
    return ([linear_x, linear_y, linear_z], [ang_x, ang_y, ang_z], stamp_ns)


def encode_cmd_vel_payload(linear, angular, stamp_ns: int) -> bytes:
    """对称的 encode 端,仅用于单测 / mock publisher。"""
    return _CMD_VEL_STRUCT.pack(
        float(linear[0]), float(linear[1]), float(linear[2]),
        float(angular[0]), float(angular[1]), float(angular[2]),
        int(stamp_ns),
    )


# ============================================================
# 上下限 clamp(默认 / 注入 / env 三层 fallback)
# ============================================================
@dataclass(frozen=True)
class MotionClampConfig:
    """Go2 motion 上下限配置。

    上下限来自 drivers/go2 配置(MAX_LINEAR_VEL 等)。当前阶段方案默认
    0.6 m/s + 1.0 rad/s,后续可在 caller 构造时注入实测值。
    """
    max_linear_m_s: float = DEFAULT_MAX_LINEAR_M_S
    max_angular_rad_s: float = DEFAULT_MAX_ANGULAR_RAD_S


def clamp_twist(linear, angular, clamp: MotionClampConfig):
    """对 (linear[3], angular[3]) 做 ±clamp,返回新 list。"""
    cap_l = clamp.max_linear_m_s
    cap_a = clamp.max_angular_rad_s
    return (
        [max(-cap_l, min(cap_l, float(linear[0]))),
         max(-cap_l, min(cap_l, float(linear[1]))),
         max(-cap_l, min(cap_l, float(linear[2])))],
        [max(-cap_a, min(cap_a, float(angular[0]))),
         max(-cap_a, min(cap_a, float(angular[1]))),
         max(-cap_a, min(cap_a, float(angular[2])))],
    )


# ============================================================
# mock backend —— 不依赖 lcm Python 包,用于单测 + CI dry-run
# ============================================================
class MockLcmBackend:
    """进程内的 mock LCM:publish 后 subscriber 立刻从 in-memory queue 收到。"""

    def __init__(self, url: str = "mock://"):
        self.url = url
        self._lock = threading.Lock()
        self._next_handle = 1
        self._subs: dict[int, Callable[[bytes], None]] = {}

    def publish(self, channel: str, payload: bytes) -> None:
        with self._lock:
            subs = list(self._subs.items())
        for _handle, cb in subs:
            try:
                cb(payload)
            except Exception as exc:  # noqa: BLE001
                logger.warning("MockLcmBackend 回调异常 channel=%s err=%s", channel, exc)

    def subscribe(self, channel: str, callback: Callable[[bytes], None]) -> int:
        with self._lock:
            handle = self._next_handle
            self._next_handle += 1
            self._subs[handle] = callback
        return handle

    def unsubscribe(self, handle: int) -> None:
        with self._lock:
            self._subs.pop(handle, None)


def _resolve_backend(url: str):
    """根据 url 选择 backend。'mock://' 强制 mock;其它 url 尝试 import lcm。

    真实 LCM 后端要求目标机安装 `python3-lcm` 或等价本仓 binding。
    Phase 6 阶段本仓只提供 mock backend;后续 Phase 8 installer 加
    `python3-lcm` 后,这里返回真实 LCM binding。
    """
    if url.startswith("mock://"):
        return MockLcmBackend(url), True
    try:
        import lcm as _lcm_mod  # type: ignore
    except ImportError as exc:
        logger.warning(
            "real LCM binding (python 'lcm' module) 不可用: %s;回退 mock backend。"
            "目标机安装 python3-lcm 或运行 scripts/installer/phases/system.py 后将切换。",
            exc,
        )
        return (
            _UnavailableBackend(
                url=url,
                reason=(
                    "NUWAX_LCM_PYTHON_AVAILABLE=false:"
                    f"python3 -c 'import lcm' 失败 ({exc});目标机需安装 python3-lcm"
                ),
            ),
            False,
        )
    return _RealLcmBackend(url, _lcm_mod), True


class _UnavailableBackend:
    """LcmBinding 缺失占位:publish/subscribe 均返回空操作并设置 last_error。"""

    def __init__(self, url: str, reason: str):
        self.url = url
        self.reason = reason
        self.subscribers = 0

    def publish(self, channel, payload):  # noqa: ARG002
        return

    def subscribe(self, channel, callback):  # noqa: ARG002
        self.subscribers += 1
        return -1

    def unsubscribe(self, handle):  # noqa: ARG002
        return


class _RealLcmBackend:
    """真实 python-lcm 后端占位 —— Phase 8 installer 落地后启用。"""

    def __init__(self, url: str, lcm_mod):
        self.url = url
        self.lcm_mod = lcm_mod
        self._lock = threading.Lock()
        self._next_handle = 1
        self._subs: dict[int, object] = {}

    def publish(self, channel, payload):
        self.lcm_mod.LCM(self.url).publish(channel, payload)

    def subscribe(self, channel, callback):
        with self._lock:
            handle = self._next_handle
            self._next_handle += 1
        sub = self.lcm_mod.LCM(self.url, wait_for_data_in_read=False)
        sub.subscribe(channel, callback)
        self._subs[handle] = sub
        # 真正的 lcm 包由 caller 在单独 thread 调用 sub.handle()
        return handle

    def unsubscribe(self, handle):
        with self._lock:
            sub = self._subs.pop(handle, None)
        if sub is not None:
            try:
                sub.unsubscribe(channel=None)  # type: ignore[arg-type]
            except Exception as exc:  # noqa: BLE001
                logger.debug("LCM unsubscribe 异常 handle=%s err=%s", handle, exc)


# ============================================================
# 状态结构(暴露给 readiness / diagnostics)
# ============================================================
@dataclass
class BridgeStats:
    """bridge 运行统计,atomic-ish(由外部锁保护:caller 在 start/stop 之间只读)。"""
    cmd_count: int = 0
    dropped_count: int = 0
    zero_count: int = 0
    last_cmd_ts_ns: int = 0
    last_zero_sent_ts_ns: int = 0
    last_error: Optional[str] = None


# ============================================================
# 主类:LcmMotionCommandBridge
# ============================================================
class LcmMotionCommandBridge:
    """订阅 `/cmd_vel` -> 解码 -> clamp -> 调 Go2 twist sink;watchdog 200ms zero-stop。

    构造参数:
      - lcm_url: real LCM URL(如 'udpm://239.255.76.67:7665?ttl=1') 或 'mock://'
      - channel: typed channel 名,默认 `NUWAX_LCM_CHANNEL_CMD_VEL` env 或 '/cmd_vel'
      - twist_sink: 接收 (Twist, source: 'cmd'|'zero') 的 callback,
                    在 drivers/go2 侧注入 `MotionControlProvider.send_twist` 包装。
      - clamp: 上下限(默认 0.6 m/s + 1.0 rad/s)。
      - timeout_s / tick_s: watchdog 时序(默认 200ms / 50ms)。
      - frame_id: Twist frame_id(默认 'base_link',与 Go2 motion chain 一致)。

    内部线程模型:
      - `_receiver_thread` (daemon): 后端 subscribe,bytes -> cmd_queue
      - `_watchdog_thread` (daemon): 50ms tick,cmd_queue / zero / shutdown 处理
      - 调用方必须调用 `start()` 启动,`stop(timeout=1.0)` 关闭。

    安全策略(13.3 硬设计):
      - watchdog 超时 200ms 内首次 zero-stop,必须 200-250ms 窗口发生。
      - `CancelNavigation` 通过 `request_immediate_zero()` 触发独立 zero,
        测试可观测 sink 在 200ms 内收到 zero。
      - motion sink 异常 → last_error 写入,继续运行不阻塞 watchdog。
    """

    DEFAULT_CHANNEL = "/cmd_vel"

    def __init__(
        self,
        lcm_url: Optional[str] = None,
        channel: Optional[str] = None,
        twist_sink: Optional[Callable[[object, str], None]] = None,
        clamp: Optional[MotionClampConfig] = None,
        timeout_s: float = WATCHDOG_TIMEOUT_S,
        tick_s: float = WATCHDOG_TICK_S,
        frame_id: str = "base_link",
        on_sink_state_change: Optional[Callable[[bool], None]] = None,
    ) -> None:
        self.lcm_url = lcm_url or os.environ.get("NUWAX_LCM_URL", "mock://")
        self.channel = (
            channel
            or os.environ.get("NUWAX_LCM_CHANNEL_CMD_VEL")
            or self.DEFAULT_CHANNEL
        )
        self.twist_sink = twist_sink
        self.clamp = clamp or MotionClampConfig()
        self.timeout_s = float(timeout_s)
        self.tick_s = float(tick_s)
        self.frame_id = frame_id
        # Rework-9:gateway 注入的 sink 状态回写 callback。
        # 调用语义:
        #   - `True`  —— bridge.start() 成功后,cmd_vel sink 已就绪
        #                (backend 订阅成功,watchdog 线程已 spawn)
        #   - `False` —— bridge 启动失败 / subscribe 抛异常 /
        #                providers 不实现 MotionControl /
        #                后端不可用(状态不可恢复,需要 caller 决策)
        # 注入实现细节:gateway 把这个 callback 接到
        # `gateways/runtime.py:_start_motion_command_bridge`,在每条
        # return / 失败路径前同步调用,确保 Python `_motion_bridge_errors`
        # 与诊断 metadata 与该 callback 一致,避免出现"bridge 已挂但
        # readiness 看不到"的不一致状态。
        self._on_sink_state_change: Optional[Callable[[bool], None]] = (
            on_sink_state_change
        )

        # atomic state(整数计数器使用 Lock-free 单变量赋值,Python GIL 保障可见性)
        self._cmd_count = 0
        self._dropped_count = 0
        self._zero_count = 0
        self._last_cmd_ts_ns = 0
        self._last_zero_sent_ts_ns = 0
        self._last_error: Optional[str] = None
        self._state_lock = threading.Lock()

        self._cmd_queue: "queue.Queue[bytes]" = queue.Queue(maxsize=256)
        self._stop_event = threading.Event()
        self._receiver_thread: Optional[threading.Thread] = None
        self._watchdog_thread: Optional[threading.Thread] = None
        self._lcm_sub_handle: Optional[int] = None
        self._backend, self._backend_real = _resolve_backend(self.lcm_url)

    # --------------------------------------------------------
    # 公共 API
    # --------------------------------------------------------
    def start(self) -> None:
        """启动 receiver + watchdog 线程;订阅 LCM。

        Rework-9:启动成功路径同步触发 `on_sink_state_change(True)`;
        subscribe / 启动失败路径同步触发 `on_sink_state_change(False)`。
        callback 由 gateway 在构造时注入,用于把 sink ready 状态写回
        Rust 侧(handle.cmd_vel_sink_ready)以及 diagnostics metadata。
        """
        if self._receiver_thread is not None or self._watchdog_thread is not None:
            return
        self._stop_event.clear()
        try:
            self._lcm_sub_handle = self._backend.subscribe(self.channel, self._on_lcm_bytes)
        except Exception as exc:  # noqa: BLE001
            self._record_error(f"subscribe 失败: {exc}")
            logger.exception("LcmMotionCommandBridge.subscribe 失败: %s", exc)
            self._notify_sink_state(False)
            raise
        self._receiver_thread = threading.Thread(
            target=self._receiver_loop, name="lcm-cmd-receiver", daemon=True
        )
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop, name="lcm-cmd-watchdog", daemon=True
        )
        try:
            self._receiver_thread.start()
            self._watchdog_thread.start()
        except Exception as exc:  # noqa: BLE001
            self._record_error(f"thread spawn 失败: {exc}")
            logger.exception("LcmMotionCommandBridge thread spawn 失败: %s", exc)
            self._notify_sink_state(False)
            raise
        self._notify_sink_state(True)
        logger.info(
            "LcmMotionCommandBridge started channel=%s url=%s timeout=%.0fms tick=%.0fms backend_real=%s",
            self.channel, self.lcm_url, self.timeout_s * 1000, self.tick_s * 1000, self._backend_real,
        )

    def stop(self, timeout: float = 1.0) -> None:
        """顺序关闭:zero-stop -> unsubscribe LCM -> 停 watchdog -> 停 receiver。

        Rework-9:关闭成功后通知 `on_sink_state_change(False)`,
        防止 gateway 已经把 bridge 拆掉但 Rust handle 还停留在 True 的
        不一致状态(会被 `compute_readiness` 当成"ready"继续放行 goal)。
        """
        self._stop_event.set()
        if self._lcm_sub_handle is not None:
            try:
                self._backend.unsubscribe(self._lcm_sub_handle)
            finally:
                self._lcm_sub_handle = None
        # 先发一个 immediate zero-stop,等一次 sink
        self.request_immediate_zero(source="shutdown")
        # 再 join 线程
        for t in (self._watchdog_thread, self._receiver_thread):
            if t is not None and t.is_alive():
                t.join(timeout=timeout)
        self._watchdog_thread = None
        self._receiver_thread = None
        self._notify_sink_state(False)

    def set_sink_state_change_callback(
        self, callback: Optional[Callable[[bool], None]]
    ) -> None:
        """运行时注入/替换 sink state change callback。

        gateway 在构造 bridge 之后调本方法,把真实写回 Rust handle 的
        函数注入进来(允许延迟绑定,避免构造参数顺序问题)。
        """
        self._on_sink_state_change = callback

    def _notify_sink_state(self, ready: bool) -> None:
        """触发 sink state change callback(若已注入)。

        callback 异常被吞掉 + 记 log:这是 best-effort 通知,不能反过来
        影响 bridge 自身的启停流程;callback 失败会在 gateway 侧拿到
        `_motion_bridge_errors`,但 bridge 实例本身仍能正常工作。
        """
        cb = self._on_sink_state_change
        if cb is None:
            return
        try:
            cb(bool(ready))
        except Exception as exc:  # noqa: BLE001
            logger.warning(
                "LcmMotionCommandBridge on_sink_state_change callback 异常 ready=%s err=%s",
                ready, exc,
            )

    def request_immediate_zero(self, source: str = "cancel") -> None:
        """CancelNavigation / shutdown / 测试触发的同步 zero-stop。

        调用方在发送 `/nav/cancel` 后调用本方法,sink 必须在 200ms 内收到
        zero twist(13.3 第 6 条)。`source` 透传给 sink 用于诊断区分:
          - 'cancel'    CancelNavigation / 测试
          - 'watchdog'  watchdog timeout
          - 'shutdown'  bridge.stop()
        """
        now_ns = time.monotonic_ns()
        if self._twist_sink_available():
            try:
                self._invoke_sink_zero(now_ns, source=source)
            except Exception as exc:  # noqa: BLE001
                self._record_error(f"immediate_zero({source}) 失败: {exc}")
        with self._state_lock:
            self._last_zero_sent_ts_ns = now_ns
            self._zero_count += 1

    def stats(self) -> BridgeStats:
        with self._state_lock:
            return BridgeStats(
                cmd_count=self._cmd_count,
                dropped_count=self._dropped_count,
                zero_count=self._zero_count,
                last_cmd_ts_ns=self._last_cmd_ts_ns,
                last_zero_sent_ts_ns=self._last_zero_sent_ts_ns,
                last_error=self._last_error,
            )

    def is_sink_ready(self) -> bool:
        """cmd_vel_sink_ready = (sink 已注入) AND (backend real 或 mock 显式) AND (last_error 非致命)。"""
        return self._twist_sink_available() and (
            self._backend_real or isinstance(self._backend, MockLcmBackend)
        )

    # --------------------------------------------------------
    # 内部:receiver + watchdog loop
    # --------------------------------------------------------
    def _on_lcm_bytes(self, payload: bytes) -> None:
        """LcmBackend subscribe 回调,把 bytes 推到 cmd_queue(非阻塞,满则丢)。"""
        try:
            self._cmd_queue.put_nowait(payload)
        except queue.Full:
            with self._state_lock:
                self._dropped_count += 1
                self._last_error = "cmd_queue 满,丢帧"
            logger.warning("cmd_queue 满,丢帧 dropped_count=%d", self._dropped_count)

    def _receiver_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                payload = self._cmd_queue.get(timeout=self.tick_s)
            except queue.Empty:
                continue
            self._handle_payload(payload)

    def _handle_payload(self, payload: bytes) -> None:
        """decode -> clamp -> sink;失败计数 + last_error。"""
        decoded = decode_cmd_vel_payload(payload)
        if decoded is None:
            with self._state_lock:
                self._dropped_count += 1
                self._last_error = f"payload 解码失败(len={len(payload)})"
            return
        linear, angular, _stamp_ns = decoded
        linear, angular = clamp_twist(linear, angular, self.clamp)
        twist = self._build_twist(linear, angular)
        now_ns = time.monotonic_ns()
        try:
            self._invoke_sink_cmd(twist)
        except Exception as exc:  # noqa: BLE001
            with self._state_lock:
                self._last_error = f"sink 异常: {exc}"
            return
        with self._state_lock:
            self._cmd_count += 1
            self._last_cmd_ts_ns = now_ns

    def _watchdog_loop(self) -> None:
        while not self._stop_event.is_set():
            time.sleep(self.tick_s)
            now_ns = time.monotonic_ns()
            with self._state_lock:
                last_cmd = self._last_cmd_ts_ns
                last_zero = self._last_zero_sent_ts_ns
            if last_cmd == 0:
                continue
            gap_ns = now_ns - last_cmd
            if gap_ns <= int(self.timeout_s * 1e9):
                continue
            # 首次超时 zero 必须在 200-250ms 窗口内
            if last_zero >= last_cmd:
                continue  # 已经发过 zero,本超时窗口不重复
            self.request_immediate_zero(source="watchdog")

    # --------------------------------------------------------
    # sink 抽象
    # --------------------------------------------------------
    def _twist_sink_available(self) -> bool:
        return callable(self.twist_sink)

    def _build_twist(self, linear, angular):
        """根据 frame_id 构造 contracts.geometry.Twist。

        延后 import 避免 services/navigation 包加载阶段对 pydantic 的硬依赖。
        pydantic 2.x 必须用 keyword args,因此 linear/angular 用 .model_validate。
        """
        from contracts.geometry import Twist, Vector3  # type: ignore

        return Twist(
            frame_id=self.frame_id,
            linear=Vector3.model_validate({"x": linear[0], "y": linear[1], "z": linear[2]}),
            angular=Vector3.model_validate({"x": angular[0], "y": angular[1], "z": angular[2]}),
        )

    def _invoke_sink_cmd(self, twist) -> None:
        if not self._twist_sink_available():
            with self._state_lock:
                self._last_error = "twist_sink 未注入,丢弃 cmd"
                self._dropped_count += 1
            return
        self.twist_sink(twist, "cmd")  # type: ignore[misc]

    def _invoke_sink_zero(self, _now_ns: int, source: str = "cancel") -> None:
        from contracts.geometry import Twist, Vector3  # type: ignore

        twist = Twist(
            frame_id=self.frame_id,
            linear=Vector3.model_validate({"x": 0.0, "y": 0.0, "z": 0.0}),
            angular=Vector3.model_validate({"x": 0.0, "y": 0.0, "z": 0.0}),
        )
        if not self._twist_sink_available():
            return
        self.twist_sink(twist, source)  # type: ignore[misc]

    def _record_error(self, msg: str) -> None:
        with self._state_lock:
            self._last_error = msg


# ============================================================
# Go2 集成 glue:封装 MotionControl.send_twist / stop_motion
# ============================================================
class Go2TwistSinkAdapter:
    """把 LcmMotionCommandBridge 的 sink 接到 Go2 MotionControlProvider。

    source = 'cmd'  -> send_twist(twist)
    source = 'cancel' / 'watchdog' / 'shutdown' -> stop_motion + send zero twist
        (保证任何 zero-stop 路径都走 Go2 实际停止接口,而非仅清状态)
    """

    def __init__(self, motion_provider, clamp: Optional[MotionClampConfig] = None):
        self.motion_provider = motion_provider
        self.clamp = clamp or MotionClampConfig()
        self.last_source: Optional[str] = None
        self.last_twist = None
        self.zero_invocations = 0
        self.cmd_invocations = 0
        self.errors: list[str] = []

    def __call__(self, twist, source: str) -> None:
        self.last_source = source
        self.last_twist = twist
        if source == "cmd":
            self._send_cmd(twist)
        else:
            self._send_zero(twist)

    def _send_cmd(self, twist) -> None:
        try:
            self.motion_provider.send_twist(twist)
            self.cmd_invocations += 1
        except Exception as exc:  # noqa: BLE001
            self.errors.append(f"send_twist 失败: {exc}")
            raise

    def _send_zero(self, _twist) -> None:
        try:
            self.motion_provider.stop_motion()
        except Exception as exc:  # noqa: BLE001
            self.errors.append(f"stop_motion 失败: {exc}")
        self.zero_invocations += 1


__all__ = [
    "CMD_VEL_PAYLOAD_BYTES",
    "BridgeStats",
    "DEFAULT_MAX_ANGULAR_RAD_S",
    "DEFAULT_MAX_LINEAR_M_S",
    "Go2TwistSinkAdapter",
    "LcmMotionCommandBridge",
    "MockLcmBackend",
    "MotionClampConfig",
    "WATCHDOG_TIMEOUT_S",
    "WATCHDOG_TICK_S",
    "WATCHDOG_ZERO_GAP_S",
    "clamp_twist",
    "decode_cmd_vel_payload",
    "encode_cmd_vel_payload",
]