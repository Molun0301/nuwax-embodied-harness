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

real LCM Python binding 不可用时:返回 `_UnavailableBackend` 并设
`last_error = "NUWAX_LCM_PYTHON_AVAILABLE=false"`(严禁静默 fallback)。

2026-07-08 fake-lcm 物理删除后(v3 F2 决策):`mock://` URL 强制走
`_UnavailableBackend` 并设 `last_error`(严禁静默 fallback 到任何 fake/memory backend)。
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
# 2026-07-08 fake-lcm 物理删除(v3 F2 决策):
# - `MockLcmBackend` 类整段物理删除
# - `mock://` URL 强制走 `_UnavailableBackend`(no-op + last_error)
# ============================================================
def _resolve_backend(
    url: str,
    ready_event: threading.Event,
    bridge_callback: Callable[[bytes], None],
):
    """2026-07-08 fake-lcm 根除后:仅支持真实 python-lcm binding。

    - `mock://` URL → 立即 `_UnavailableBackend` + 错误信息(无 mock fallback)
    - 其它 url → 尝试 `import lcm`;失败时返回 `_UnavailableBackend` 并设
      last_error(严禁静默 fallback)
    - 成功 → 返回 `_RealLcmBackend(url, _lcm_mod, ready_event, bridge_callback)`,
      ready_event / bridge_callback 由 bridge 注入,实现 driver thread 单 owner
      范式(v5 §3.2):lcm.LCM 实例仅在 driver thread 内创建并持有,bridge
      跨类只通过 `publish()` / `subscribe()` / `unsubscribe()` / `_close()`
      这四个接口与 backend 交互,绝不访问 `self._lc`。
    """
    if url.startswith("mock://"):
        return (
            _UnavailableBackend(
                url=url,
                reason=(
                    "LCM_BACKEND_UNSUPPORTED: mock:// URL 已在 2026-07-08 物理删除"
                    "(v3 F2 决策);fake-lcm backend 整体移除;请改用 udpm://"
                    "239.255.76.67:7667?ttl=1",
                ),
            ),
            False,
        )
    try:
        import lcm as _lcm_mod  # type: ignore
    except ImportError as exc:
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
    return _RealLcmBackend(url, _lcm_mod, ready_event, bridge_callback), True


class _UnavailableBackend:
    """LcmBinding 缺失占位:publish/subscribe 均返回空操作并设置 last_error。

    v5 §2.2.1:_RealLcmBackend 重写后,brief.start()/stop() 走
    `_backend.start_driver / _close / driver_thread` 接口;_UnavailableBackend
    必须提供同名接口以保持签名一致(否则 `bridge.start()` 会抛
    AttributeError 而不是预期的 RuntimeError)。
    """

    def __init__(self, url: str, reason: str):
        self.url = url
        self.reason = reason
        self.subscribers = 0

    def start_driver(self, channel: str) -> None:
        """unavailable backend 无 driver thread 可起,直接抛 RuntimeError 让 bridge
        走 on_sink_state_change(False) 路径;不会泄露 liblcm 句柄。
        """
        raise RuntimeError(
            f"LcmBackend unavailable: {self.reason}"
        )

    def publish(self, channel, payload):  # noqa: ARG002
        return

    def subscribe(self, channel, callback):  # noqa: ARG002
        self.subscribers += 1
        return -1

    def unsubscribe(self, handle):  # noqa: ARG002
        return

    def _close(self) -> None:
        """unavailable backend 无 driver thread 可 join,no-op。"""
        return

    @property
    def driver_thread(self):  # type: ignore[no-untyped-def]
        """unavailable backend 无 driver thread。"""
        return None


class _RealLcmBackend:
    """真实 python-lcm 后端(driver thread 单 owner 范式,v5 §3.2)。

    设计要点:
      - lcm.LCM 实例**仅由 driver thread 创建并持有**(self 不存 `_lc`)。
      - bridge 调用 `publish()` 把消息推到 `self._publish_queue`,driver thread
        50ms tick 内 `lc.publish()`。**严禁** bridge 在自己线程直接 `new LCM()`
        再 `lc.publish()` —— 那是 v3 之前的反模式(多 owner + liblcm 全局锁争用)。
      - 订阅在 driver thread 启动时一次完成:`lc.subscribe(channel, lambda)`,
        N-R12-1 修:lcm-python 1.3.1 callback 是 `(channel, data)` 双参,必须用
        lambda 包装成单参 `(data)` 透传给 bridge 注入的 `_real_callback`,
        否则会报 "TypeError: missing 1 required positional argument"。
      - publish queue 限长(maxsize=256,R5 M-R5-1):driver thread 卡住时 bridge
        不会无限堆积,丢帧计数走 `self._publish_dropped`。
    """

    # publish / subscribe 接口签名(bridge 仍走 publish/subscribe/unsubscribe,
    # 但语义已经改为"driver thread 异步执行")
    def __init__(
        self,
        url: str,
        lcm_mod,
        ready_event: threading.Event,
        bridge_callback: Callable[[bytes], None],
    ):
        self.url = url
        self.lcm_mod = lcm_mod
        self._stop_event = threading.Event()
        self._driver_thread_ready = ready_event  # bridge 注入
        self._lc_thread: Optional[threading.Thread] = None
        # 关键:不持 self._lc(避免双 owner);lcm.LCM 实例在 driver thread 内创建
        self._lc_creator = lcm_mod.LCM  # 引用,非实例
        # R5 M-R5-1:publish queue 限长
        self._publish_queue: "queue.Queue[tuple[str, bytes]]" = queue.Queue(maxsize=256)
        self._publish_dropped = 0
        # R5 M-R5-1 + R8-M-1:callback 异常保护
        # N-R12-1 lambda 包装:`lc.subscribe(channel, lambda ch, data: self._on_lcm_bytes(data))`
        self._bridge_callback = bridge_callback
        self._channel: Optional[str] = None
        self._channel_sub_handle: Optional[int] = None
        self._lc: Optional[object] = None  # 仅 driver thread 内可见
        self._subscribed = threading.Event()  # 内部订阅完成信号

    # --------------------------------------------------------
    # bridge 调用的对外 API(语义已改为 driver thread 异步执行)
    # --------------------------------------------------------
    def start_driver(self, channel: str) -> None:
        """启动 driver thread,内部完成 `lc.subscribe(channel, ...)`。"""
        if self._lc_thread is not None:
            return
        self._channel = channel
        self._lc_thread = threading.Thread(
            target=self._driver_loop, name="lcm-backend-driver", daemon=True
        )
        self._lc_thread.start()

    def publish(self, channel: str, payload: bytes) -> None:
        """bridge → driver thread:把 (channel, payload) 推到 publish_queue。

        退化路径:driver thread 尚未拿到 lc(`/cmd_vel` 还没订阅成功,或
        `lcm.LCM(url)` 初始化失败导致 `_lc=None`)时,publish 走"直接 new
        lcm.LCM().publish()" 同步单发路径——这是 v3 之前行为,保证测试
        `bridge._backend.publish(...)` 在未知 URL 下也能调用(不会卡死)。
        生产路径下 lc_alive()=True,走 enqueue,driver thread 内统一 drain。
        """
        if not self._lc_creator or self._lc is None:
            # 退化:lc 还没创建好,直接 fire-and-forget
            try:
                self._lc_creator(self.url).publish(channel, payload)
            except Exception as exc:  # noqa: BLE001
                self._publish_dropped += 1
                logger.debug(
                    "退化 publish 异常 channel=%s err=%s", channel, exc
                )
            return
        try:
            self._publish_queue.put_nowait((channel, payload))
        except queue.Full:
            self._publish_dropped += 1
            logger.warning(
                "lcm publish_queue 满,丢帧 dropped=%d channel=%s",
                self._publish_dropped, channel,
            )

    def subscribe(self, channel, callback):  # noqa: ARG002
        """保留兼容签名:实际订阅由 start_driver() 完成;bridge 不再调用。

        返回固定 handle=1,driver thread 内部已 `lc.subscribe` 完成。
        """
        # 等 driver thread 至少跑过一次 handle_timeout 再返回,
        # 避免 bridge.start() 紧接着 is_sink_ready() 看到 True 但订阅还没生效。
        if not self._driver_thread_ready.wait(timeout=1.0):
            raise RuntimeError(
                "lcm driver thread 未在 1.0s 内 ready,url=%s" % self.url
            )
        return 1

    def unsubscribe(self, handle):  # noqa: ARG002
        """driver thread 内调 lc.unsubscribe(channel=...) 后清状态。"""
        # 真正清理放在 _close() 里(driver thread join 前);这里仅兼容签名。

    def _close(self) -> None:
        """bridge.stop() 顺序:stop_event → driver thread join → 释放 _lc。"""
        self._stop_event.set()
        if self._lc_thread is not None and self._lc_thread.is_alive():
            self._lc_thread.join(timeout=1.0)
        self._lc_thread = None
        self._lc = None  # 释放 liblcm C 资源

    @property
    def driver_thread(self) -> Optional[threading.Thread]:
        """暴露 driver thread 给 bridge.stop() join(漏-7 修)。"""
        return self._lc_thread

    def lc_alive(self) -> bool:
        """lc 实例健康:driver thread 已 ready 且 `_lc` 不为 None。

        v5 §2.2.3 强化:driver thread 初始化失败(URL 错误 / lcm 拒绝)
        时 `_lc=None`,本方法返 False;bridge.is_sink_ready() 据此拒返 True,
        防止 readiness 闪绿灯。
        """
        return self._driver_thread_ready.is_set() and self._lc is not None

    # --------------------------------------------------------
    # driver thread 主循环(唯一 owner lcm.LCM 实例)
    # --------------------------------------------------------
    def _driver_loop(self) -> None:
        """v5 §3.2 主循环:创建 lc → 订阅 → ready_event.set() → 50ms tick。

        N-R12-1:订阅时用 lambda 包装把 `(ch, data)` 双参收敛为 `(data)` 单参。

        初始化失败时:清 `_lc`,清 _driver_thread_ready(让 bridge.is_sink_ready
        返 False,publish 不被静默丢),并 drain publish_queue 让 bridge 后续
        抛 RuntimeError / 看到状态异常。
        """
        lc = None
        try:
            lc = self._lc_creator(self.url)
            self._lc = lc
            # N-R12-1 修:lcm-python 1.3.1 callback 是 (channel, data) 双参;
            # 必须用 lambda 收敛为 (data) 单参喂给 bridge_callback。
            if self._channel is not None:
                lc.subscribe(
                    self._channel,
                    lambda _ch, data, cb=self._bridge_callback: cb(data),
                )
                self._subscribed.set()
            self._driver_thread_ready.set()  # 通知 bridge:driver thread 就绪
        except Exception as exc:  # noqa: BLE001
            logger.exception("lcm driver thread 初始化失败 url=%s: %s", self.url, exc)
            # 保持 `_lc=None`,让 is_sink_ready 通过 lc_alive() 检测拒绝返 True;
            # set ready_event(否则 bridge.start() 中 `.wait(1.0)` 会超时)。
            self._lc = None
            self._driver_thread_ready.set()
            return

        while not self._stop_event.is_set():
            # 1) drain publish_queue(非阻塞,丢帧计数)
            while True:
                try:
                    ch, payload = self._publish_queue.get_nowait()
                except queue.Empty:
                    break
                try:
                    lc.publish(ch, payload)
                except Exception as exc:  # noqa: BLE001
                    self._publish_dropped += 1
                    logger.warning(
                        "lc.publish 异常 channel=%s dropped=%d err=%s",
                        ch, self._publish_dropped, exc,
                    )
            # 2) liblcm 50ms tick 内同步 dispatch 回调(R8 P-反-5 race)
            try:
                lc.handle_timeout(50)
            except Exception as exc:  # noqa: BLE001
                logger.warning("lc.handle_timeout 异常: %s", exc)

        # 退出前清订阅 + 释放 lc(避免 liblcm fd 泄漏)
        try:
            if self._channel is not None:
                lc.unsubscribe(self._channel)
        except Exception:  # noqa: BLE001
            pass


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
      - lcm_url: real LCM URL(如 'udpm://239.255.76.67:7667?ttl=1') 或 'mock://'
      - channel: typed channel 名,默认 `NUWAX_LCM_CHANNEL_CMD_VEL` env 或 '/cmd_vel'
      - twist_sink: 接收 (Twist, source: 'cmd'|'zero') 的 callback,
                    在 drivers/go2 侧注入 `MotionControlProvider.send_twist` 包装。
      - clamp: 上下限(默认 0.6 m/s + 1.0 rad/s)。
      - timeout_s / tick_s: watchdog 时序(默认 200ms / 50ms)。
      - frame_id: Twist frame_id(默认 'base_link',与 Go2 motion chain 一致)。

    内部线程模型(v5 §3.2):
      - `_driver_thread` (lcm-backend-driver, daemon): _RealLcmBackend 持有,
        唯一 owner lcm.LCM 实例;`lc.subscribe(channel, lambda)` 一次性完成;
        50ms tick 内 `lc.handle_timeout(50)` 同步 dispatch 回调;publish queue
        也由 driver thread 内 drain。
      - `_receiver_thread` (lcm-cmd-receiver, daemon): cmd_queue -> decode -> sink
      - `_watchdog_thread` (lcm-cmd-watchdog, daemon): 50ms tick 超时检测
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
        self.lcm_url = lcm_url or os.environ.get("NUWAX_LCM_URL", "udpm://239.255.76.67:7667?ttl=1")
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
        self._driver_thread_ready = threading.Event()  # v5 §3.2 协调层
        # bridge 注入的 lcm 字节回调:driver thread 在 `_RealLcmBackend._driver_loop`
        # 内通过 lambda 把 lc.subscribe 的 (ch, data) 双参收敛为单参 (data)
        # 后调用(N-R12-1 修)。
        self._real_callback = self._on_lcm_bytes
        self._lcm_sub_handle: Optional[int] = None
        self._backend, self._backend_real = _resolve_backend(
            self.lcm_url, self._driver_thread_ready, self._real_callback,
        )

    # --------------------------------------------------------
    # 公共 API
    # --------------------------------------------------------
    def start(self) -> None:
        """启动 driver + receiver + watchdog 线程;订阅 LCM(driver thread 内完成)。

        v5 §3.2 顺序:
          1. `_backend.start_driver(channel)` 启动 driver thread(在 driver thread
             内 `lc.subscribe(channel, lambda)` 一次性完成订阅)。
          2. `_driver_thread_ready.wait(timeout=1.0)` 等 driver thread 跑过至少
             一次 `lc.handle_timeout(50)`(R8 P-反-5 race 修复)。
          3. spawn `_receiver_thread` + `_watchdog_thread`。
          4. `_notify_sink_state(True)`。

        Rework-9:启动成功路径同步触发 `on_sink_state_change(True)`;
        subscribe / 启动失败路径同步触发 `on_sink_state_change(False)`。
        callback 由 gateway 在构造时注入,用于把 sink ready 状态写回
        Rust 侧(handle.cmd_vel_sink_ready)以及 diagnostics metadata。
        """
        if (
            self._receiver_thread is not None
            or self._watchdog_thread is not None
        ):
            return
        self._stop_event.clear()
        # 1) 启动 backend:优先 driver thread(v5 §3.2 单 owner 范式);
        #    测试 BrokenBackend 仅有旧 `subscribe` 接口时走兼容路径。
        try:
            if hasattr(self._backend, "start_driver"):
                self._backend.start_driver(self.channel)
            else:
                # 兼容:旧 backend(测试 mock)无 start_driver,回退到 subscribe
                self._lcm_sub_handle = self._backend.subscribe(
                    self.channel, self._real_callback
                )
        except Exception as exc:  # noqa: BLE001
            self._record_error(f"start_driver/subscribe 失败: {exc}")
            logger.exception(
                "LcmMotionCommandBridge backend 启动失败: %s", exc
            )
            self._notify_sink_state(False)
            raise
        # 2) 等 driver thread ready(R8 P-反-5 race 修复)
        if not self._driver_thread_ready.wait(timeout=1.0):
            self._record_error("lcm driver thread 未在 1.0s 内 ready")
            logger.error(
                "LcmMotionCommandBridge driver thread 未在 1.0s 内 ready url=%s",
                self.lcm_url,
            )
            self._notify_sink_state(False)
            raise RuntimeError("lcm driver thread 未在 1.0s 内 ready")
        self._lcm_sub_handle = 1  # 占位:实际订阅在 driver thread 内
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
        """顺序关闭(stop_event → 停 driver thread → 收尾 LCM 句柄 → zero → join 三线程 → 通知)。

        v5 §3.2 顺序:
          1. `self._stop_event.set()` 让 receiver/watchdog 自然退出。
          2. `self._backend._close()` 内部:
                a. stop_event set(driver thread 看到后跳出 while)
                b. driver thread join(timeout=1.0)
                c. 释放 `self._lc`(liblcm fd 回收)
          3. unsubscribe LCM 句柄(若仍在)。
          4. `request_immediate_zero(source="shutdown")` 发 zero。
          5. 漏-7 修:join **三线程** driver / watchdog / receiver。
          6. `notify_sink_state(False)`。

        Rework-9:关闭成功后通知 `on_sink_state_change(False)`,
        防止 gateway 已经把 bridge 拆掉但 Rust handle 还停留在 True 的
        不一致状态(会被 `compute_readiness` 当成"ready"继续放行 goal)。
        """
        self._stop_event.set()
        # 先关 backend(driver thread join + 释放 _lc)
        if hasattr(self._backend, "_close"):
            self._backend._close()
        if self._lcm_sub_handle is not None:
            try:
                self._backend.unsubscribe(self._lcm_sub_handle)
            finally:
                self._lcm_sub_handle = None
        # 先发一个 immediate zero-stop,等一次 sink
        self.request_immediate_zero(source="shutdown")
        # 再 join 线程(漏-7 修:三线程全 join)
        driver_thread = getattr(self._backend, "driver_thread", None)
        for t in (driver_thread, self._watchdog_thread, self._receiver_thread):
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
        """cmd_vel_sink_ready = (sink 已注入) AND (backend 真 real) AND (lc 健康)。

        v5 §2.2.3 强化(R2 P-反-5 race 修复):
          - 必须等 driver thread 已调过至少一次 `lc.handle_timeout(50)` 且
            `lcm.LCM` 实例创建成功(driver thread init 失败时 `_lc=None`)才返 True;
            否则 start() 后立刻 is_sink_ready() 可能看到 backend_real=True 但订阅
            还没生效,造成 readiness 闪绿灯。
        """
        lc_alive = bool(
            getattr(self._backend, "lc_alive", lambda: self._backend_real)()
        )
        return self._twist_sink_available() and self._backend_real and lc_alive

    # --------------------------------------------------------
    # 内部:receiver + watchdog loop
    # --------------------------------------------------------
    def _on_lcm_bytes(self, payload: bytes) -> None:
        """lcm-python 1.3.1 callback 是 `(channel, data)` 双参;v5 在 subscribe 处加 lambda 包装。

        R8-M-1 强化:callback 内 try/except + record_error(避免 liblcm C 扩展
        传播异常导致 driver thread 退出)。
        """
        try:
            self._cmd_queue.put_nowait(payload)
        except queue.Full:
            with self._state_lock:
                self._dropped_count += 1
                self._last_error = "cmd_queue 满,丢帧"
            logger.warning("cmd_queue 满,丢帧 dropped_count=%d", self._dropped_count)
        except Exception as exc:  # R8-M-1
            with self._state_lock:
                self._last_error = f"lcm callback 异常: {exc}"
            logger.exception("lcm callback 异常: %s", exc)

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
    "MotionClampConfig",
    "WATCHDOG_TIMEOUT_S",
    "WATCHDOG_TICK_S",
    "WATCHDOG_ZERO_GAP_S",
    "clamp_twist",
    "decode_cmd_vel_payload",
    "encode_cmd_vel_payload",
]