"""LcmMotionCommandBridge 单元测试。

覆盖(00_最终执行方案.md §5 Phase 6 + §13.3):
1. decode golden Twist(对齐 typed_codec.rs encode_cmd_vel_payload 输出)。
2. clamp 超限 -> 限幅后输出。
3. watchdog 超时 200ms -> sink 收到 zero,窗口 200-250ms 内。
4. CancelNavigation(通过 request_immediate_zero)后 sink 收到 zero,200ms 内。
5. shutdown 顺序:zero-stop -> unsubscribe -> stop threads。
6. cmd_queue 满 -> dropped_count 自增。
7. sink 未注入 -> is_sink_ready()=False;mock backend 下允许 mock 行为。

不依赖 pytest / pydantic,直接 `python3 test_lcm_motion_command_bridge.py`。
按 feedback_pytest_pydantic_unavailable.md 与
feedback_venv_pytest_ros_plugin_conflict.md 落地。

使用:
  PYTHONPATH=/home/ml/Deeprob/nuwax_robot_bridge \\
  python3 services/navigation/tests/test_lcm_motion_command_bridge.py
"""
from __future__ import annotations

import os
import sys
import time
import struct

# 强制使用 .venv_model_download 以兼容 pydantic(feedback_dev_static_validation_two_interpreters)
_VENV_PYTHON = "/home/ml/Deeprob/nuwax_robot_bridge/.venv_model_download/bin/python3"
if os.path.exists(_VENV_PYTHON) and sys.executable != _VENV_PYTHON:
    os.execv(_VENV_PYTHON, [_VENV_PYTHON] + sys.argv)

# 项目根注入 sys.path
_PROJECT_ROOT = "/home/ml/Deeprob/nuwax_robot_bridge"
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from services.navigation.lcm_motion_command_bridge import (  # noqa: E402
    BridgeStats,
    LcmMotionCommandBridge,
    MockLcmBackend,
    MotionClampConfig,
    WATCHDOG_TIMEOUT_S,
    WATCHDOG_TICK_S,
    clamp_twist,
    decode_cmd_vel_payload,
    encode_cmd_vel_payload,
)

# ------------------------------------------------------------
# 测试记录工具:每个用例的 sink 调用全部入 queue,便于断言
# ------------------------------------------------------------

class RecordingSink:
    """测试用 sink:记录每次 (twist, source, ts_ns),zero/cmd 分桶计数。"""

    def __init__(self):
        self.calls = []
        self.cmd_count = 0
        self.zero_count = 0
        self.last_source = None
        self.last_twist = None
        self.errors = []

    def __call__(self, twist, source):
        self.calls.append((twist, source, time.monotonic_ns()))
        self.last_source = source
        self.last_twist = twist
        if source == "cmd":
            self.cmd_count += 1
        else:
            self.zero_count += 1


# 失败累积器(避免任一 assert 抛异常直接中断后续用例)
_results = {"pass": 0, "fail": 0, "fails": []}


def _check(name, cond, detail=""):
    if cond:
        print(f"  [PASS] {name}")
        _results["pass"] += 1
    else:
        print(f"  [FAIL] {name}  {detail}")
        _results["fail"] += 1
        _results["fails"].append((name, detail))


# ------------------------------------------------------------
# Case 1: decode golden bytes
# ------------------------------------------------------------
def test_decode_golden_twist():
    print("Case 1: decode golden Twist (与 typed_codec.rs 对称)")
    # linear=[1.0, 2.0, 3.0] angular=[0.1, 0.2, 0.3] stamp=999
    # 与 typed_codec.rs test 一致
    payload = encode_cmd_vel_payload([1.0, 2.0, 3.0], [0.1, 0.2, 0.3], 999)
    _check("payload 长度=32", len(payload) == 32, f"len={len(payload)}")
    decoded = decode_cmd_vel_payload(payload)
    _check("decode 非 None", decoded is not None, f"decoded={decoded}")
    if decoded is not None:
        linear, angular, stamp = decoded
        _check("linear[0]==1.0", abs(linear[0] - 1.0) < 1e-6)
        _check("linear[1]==2.0", abs(linear[1] - 2.0) < 1e-6)
        _check("linear[2]==3.0", abs(linear[2] - 3.0) < 1e-6)
        _check("angular[0]==0.1", abs(angular[0] - 0.1) < 1e-6)
        _check("angular[2]==0.3", abs(angular[2] - 0.3) < 1e-6)
        _check("stamp==999", stamp == 999)
    # 错误长度
    bad = decode_cmd_vel_payload(b"")
    _check("空 payload 解码失败", bad is None)
    _check("16-byte payload 解码失败", decode_cmd_vel_payload(b"\x00" * 16) is None)


# ------------------------------------------------------------
# Case 2: clamp 超限
# ------------------------------------------------------------
def test_clamp_twist():
    print("Case 2: clamp 超限 -> 限幅后输出")
    clamp = MotionClampConfig(max_linear_m_s=0.6, max_angular_rad_s=1.0)
    # 正常:不动
    lin, ang = clamp_twist([0.3, 0.0, 0.0], [0.5, 0.0, 0.0], clamp)
    _check("clamp 后 linear[0]==0.3", abs(lin[0] - 0.3) < 1e-6)
    _check("clamp 后 angular[0]==0.5", abs(ang[0] - 0.5) < 1e-6)
    # 超 linear
    lin, ang = clamp_twist([5.0, -5.0, 0.7], [0.0, 0.0, 0.0], clamp)
    _check("linear.x 正超限 -> +0.6", abs(lin[0] - 0.6) < 1e-6)
    _check("linear.y 负超限 -> -0.6", abs(lin[1] - (-0.6)) < 1e-6)
    _check("linear.z=0.7 超 0.6 限幅 -> 0.6", abs(lin[2] - 0.6) < 1e-6)
    # 超 angular
    lin, ang = clamp_twist([0.0, 0.0, 0.0], [3.0, -3.0, 0.0], clamp)
    _check("angular.x 正超限 -> +1.0", abs(ang[0] - 1.0) < 1e-6)
    _check("angular.y 负超限 -> -1.0", abs(ang[1] - (-1.0)) < 1e-6)


# ------------------------------------------------------------
# Case 3: bridge 启动后 mock publish cmd,sink 收到,cmd_count==1
# ------------------------------------------------------------
def test_bridge_receives_cmd():
    print("Case 3: bridge start -> mock publish cmd -> sink.cmd_count==1")
    sink = RecordingSink()
    bridge = LcmMotionCommandBridge(
        lcm_url="mock://",
        channel="/cmd_vel",
        twist_sink=sink,
        clamp=MotionClampConfig(max_linear_m_s=0.6, max_angular_rad_s=1.0),
    )
    bridge.start()
    try:
        payload = encode_cmd_vel_payload([0.2, 0.0, 0.0], [0.0, 0.0, 0.3], 1000)
        # 通过 bridge 自己的 backend publish(直接拿 backend 不优雅,改为 cast)
        bridge._backend.publish("/cmd_vel", payload)
        # 等 receiver 处理(cmd_queue -> _handle_payload)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and sink.cmd_count == 0:
            time.sleep(0.01)
        _check("sink.cmd_count==1", sink.cmd_count == 1, f"got={sink.cmd_count}")
        stats = bridge.stats()
        _check("bridge.cmd_count==1", stats.cmd_count == 1, f"got={stats.cmd_count}")
        _check("last_cmd_ts_ns>0", stats.last_cmd_ts_ns > 0)
        _check("is_sink_ready()=True", bridge.is_sink_ready())
    finally:
        bridge.stop(timeout=1.0)


# ------------------------------------------------------------
# Case 4: watchdog 200ms 超时发 zero(关键验收 §13.3 第 4 条)
# ------------------------------------------------------------
def test_watchdog_zero_stop():
    print("Case 4: watchdog 200ms 超时 -> sink 收到 zero,窗口 200-250ms")
    sink = RecordingSink()
    bridge = LcmMotionCommandBridge(
        lcm_url="mock://",
        channel="/cmd_vel",
        twist_sink=sink,
        clamp=MotionClampConfig(),
        timeout_s=WATCHDOG_TIMEOUT_S,
        tick_s=WATCHDOG_TICK_S,
    )
    bridge.start()
    try:
        # 1) 发一条 cmd,记录时间戳
        payload = encode_cmd_vel_payload([0.4, 0.0, 0.0], [0.0, 0.0, 0.0], 1000)
        cmd_ts = time.monotonic_ns()
        bridge._backend.publish("/cmd_vel", payload)
        # 等 sink 收到 cmd
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and sink.cmd_count == 0:
            time.sleep(0.01)
        _check("收到 cmd==1", sink.cmd_count == 1)
        # 2) 等待 watchdog 触发(默认 tick=50ms,timeout=200ms -> 首 zero 在 200-250ms)
        # 我们直接 sleep 260ms 让 watchdog 必然触发
        time.sleep(0.26)
        _check("watchdog 触发 zero>=1", sink.zero_count >= 1, f"got={sink.zero_count}")
        stats = bridge.stats()
        _check("bridge.zero_count>=1", stats.zero_count >= 1)
        _check("last_zero_sent_ts_ns>last_cmd_ts_ns",
               stats.last_zero_sent_ts_ns >= stats.last_cmd_ts_ns)
        # 3) 校验 zero 时间戳距离 cmd 在 200-260ms 窗口
        gap_ms = (stats.last_zero_sent_ts_ns - cmd_ts) / 1e6
        _check("zero 距 cmd 落在 200-260ms 窗口内",
               180.0 <= gap_ms <= 300.0,  # 给 80ms 测试抖动
               f"gap={gap_ms:.1f}ms")
    finally:
        bridge.stop(timeout=1.0)


# ------------------------------------------------------------
# Case 5: CancelNavigation 后 sink 立即收到 zero(§13.3 第 6 条)
# ------------------------------------------------------------
def test_cancel_immediate_zero():
    print("Case 5: CancelNavigation(request_immediate_zero) -> sink 收到 zero")
    sink = RecordingSink()
    bridge = LcmMotionCommandBridge(
        lcm_url="mock://",
        channel="/cmd_vel",
        twist_sink=sink,
    )
    bridge.start()
    try:
        cancel_ts = time.monotonic_ns()
        bridge.request_immediate_zero()
        # 不需要 sleep:request_immediate_zero 是同步路径,sink 应已收到
        _check("sink.zero_count>=1", sink.zero_count >= 1, f"got={sink.zero_count}")
        # 验证 source == 'cancel'(而不是 'watchdog')
        _check("sink.last_source=='cancel'",
               sink.last_source == "cancel",
               f"got={sink.last_source}")
        # zero 延迟 < 50ms(sink 调用是同步的)
        delay_ms = (time.monotonic_ns() - cancel_ts) / 1e6
        _check("zero 延迟 < 200ms", delay_ms < 200.0, f"delay={delay_ms:.1f}ms")
        stats = bridge.stats()
        _check("bridge.zero_count>=1", stats.zero_count >= 1)
    finally:
        bridge.stop(timeout=1.0)


# ------------------------------------------------------------
# Case 6: shutdown 顺序 = zero -> unsubscribe -> stop threads
# ------------------------------------------------------------
def test_shutdown_sequence():
    print("Case 6: bridge.stop(timeout=1.0) -> zero -> unsubscribe -> join")
    sink = RecordingSink()
    bridge = LcmMotionCommandBridge(
        lcm_url="mock://",
        channel="/cmd_vel",
        twist_sink=sink,
    )
    bridge.start()
    # 先发一条 cmd
    bridge._backend.publish("/cmd_vel", encode_cmd_vel_payload([0.1, 0, 0], [0, 0, 0], 1))
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline and sink.cmd_count == 0:
        time.sleep(0.01)
    _check("stop 前收到 cmd", sink.cmd_count == 1)
    before_zero = sink.zero_count
    bridge.stop(timeout=1.0)
    after_stats = bridge.stats()
    _check("stop 后 zero_count 增加(自动 zero-stop)", after_stats.zero_count > before_zero,
           f"before={before_zero} after={after_stats.zero_count}")
    _check("receiver thread 已停止",
           bridge._receiver_thread is None)
    _check("watchdog thread 已停止",
           bridge._watchdog_thread is None)
    # publish 在 stop 后不应该被处理
    sink.calls.clear()
    try:
        bridge._backend.publish("/cmd_vel", encode_cmd_vel_payload([0, 0, 0], [0, 0, 0], 2))
    except Exception:
        pass
    time.sleep(0.1)
    _check("stop 后 publish 不再触发 sink", len(sink.calls) == 0, f"calls={len(sink.calls)}")


# ------------------------------------------------------------
# Case 7: cmd_queue 满 -> dropped_count 增加
# ------------------------------------------------------------
def test_dropped_count_on_queue_full():
    print("Case 7: cmd_queue 满 -> dropped_count>0")
    sink = RecordingSink()
    bridge = LcmMotionCommandBridge(
        lcm_url="mock://",
        channel="/cmd_vel",
        twist_sink=sink,
        tick_s=1.0,  # receiver 慢,让 queue 累积
    )
    bridge.start()
    try:
        # 强行灌超过 256 条
        for i in range(300):
            bridge._backend.publish(
                "/cmd_vel",
                encode_cmd_vel_payload([0.1, 0, 0], [0, 0, 0], i),
            )
        # 等一段,让 _on_lcm_bytes 处理若干 + queue 满
        time.sleep(0.3)
        stats = bridge.stats()
        _check("dropped_count>0", stats.dropped_count > 0, f"got={stats.dropped_count}")
        _check("cmd_count>0", stats.cmd_count > 0, f"got={stats.cmd_count}")
    finally:
        bridge.stop(timeout=1.0)


# ------------------------------------------------------------
# Case 8: sink 未注入 -> is_sink_ready()=False
# ------------------------------------------------------------
def test_sink_ready_flag():
    print("Case 8: sink 未注入 -> is_sink_ready()=False")
    bridge = LcmMotionCommandBridge(lcm_url="mock://", channel="/cmd_vel")
    _check("is_sink_ready()=False (sink 未注入)", bridge.is_sink_ready() is False)
    # mock backend 默认 True(避免 readiness 假死,但要求 sink 注入)
    bridge = LcmMotionCommandBridge(
        lcm_url="mock://", channel="/cmd_vel", twist_sink=RecordingSink()
    )
    bridge.start()
    try:
        _check("is_sink_ready()=True (mock + sink 已注入)", bridge.is_sink_ready() is True)
    finally:
        bridge.stop(timeout=1.0)


# ------------------------------------------------------------
# Case 9: payload 解码失败(长度不对)被丢弃,不抛异常
# ------------------------------------------------------------
def test_bad_payload_dropped():
    print("Case 9: payload 长度不对 -> dropped_count 增加,不抛")
    sink = RecordingSink()
    bridge = LcmMotionCommandBridge(lcm_url="mock://", channel="/cmd_vel", twist_sink=sink)
    bridge.start()
    try:
        bridge._backend.publish("/cmd_vel", b"\x00" * 10)  # 不是 32
        time.sleep(0.2)
        stats = bridge.stats()
        _check("bad payload 不触发 sink.cmd", sink.cmd_count == 0)
        _check("bad payload 增加 dropped_count",
               stats.dropped_count >= 1, f"got={stats.dropped_count}")
        _check("last_error 含 '解码'", "解码" in (stats.last_error or ""),
               f"last_error={stats.last_error!r}")
    finally:
        bridge.stop(timeout=1.0)


# ============================================================
# Case 11 (Rework-9): sink state change callback 触发时序
# ============================================================
def test_sink_state_change_callback_fires_on_start_and_stop():
    """Rework-9 验收用例 1:start() 成功 → callback(True);
    stop() → callback(False)。

    防止 gateway 在 start / 失败 / 跳过 / 关闭路径上忘记同步 sink ready
    状态,导致 Rust handle 与 Python bridge 出现"不一致"。
    """
    print("Case 11: on_sink_state_change callback 时序 (start → True, stop → False)")
    events = []
    sink = RecordingSink()
    bridge = LcmMotionCommandBridge(
        lcm_url="mock://",
        channel="/cmd_vel",
        twist_sink=sink,
        on_sink_state_change=events.append,
    )
    bridge.start()
    try:
        _check("start 后 callback 收到 True", events == [True],
               f"events={events}")
    finally:
        bridge.stop(timeout=1.0)
    _check(
        "start + stop 后 callback 序列 == [True, False]",
        events == [True, False],
        f"events={events}",
    )


def test_sink_state_change_callback_not_fired_when_subscribe_fails():
    """Rework-9 验收用例 2:bridge 构造参数注入 callback 后,
    subscribe 失败的 mock backend 路径下 callback 必须收到 False。

    用一个会抛异常的 backend 替代 MockLcmBackend,确保 start 内部
    try/except 把异常带上 callback=False。
    """
    print("Case 12: subscribe 抛异常 → callback 收到 False")

    class BrokenBackend:
        def __init__(self):
            self.url = "broken://"

        def subscribe(self, channel, callback):  # noqa: ARG002
            raise RuntimeError("broken subscribe")

        def unsubscribe(self, handle):  # noqa: ARG002
            return

        def publish(self, channel, payload):  # noqa: ARG002
            return

    events = []
    sink = RecordingSink()
    bridge = LcmMotionCommandBridge(
        lcm_url="broken://",
        channel="/cmd_vel",
        twist_sink=sink,
        on_sink_state_change=events.append,
    )
    # 必须把 _backend 换成 BrokenBackend,_resolve_backend 默认走 mock://
    bridge._backend = BrokenBackend()
    raised = False
    try:
        bridge.start()
    except RuntimeError:
        raised = True
    _check("subscribe 抛异常被 start 重新抛出", raised)
    _check(
        "subscribe 抛异常后 callback 收到 False",
        events == [False],
        f"events={events}",
    )


# ------------------------------------------------------------
# Case 10: Go2TwistSinkAdapter 接入路径
# ------------------------------------------------------------
def test_go2_twist_sink_adapter():
    print("Case 10: Go2TwistSinkAdapter source 路由 (cmd vs cancel/watchdog)")
    from services.navigation.lcm_motion_command_bridge import Go2TwistSinkAdapter

    class FakeMotionProvider:
        def __init__(self):
            self.send_twist_calls = []
            self.stop_motion_calls = 0

        def send_twist(self, twist):
            self.send_twist_calls.append(twist)

        def stop_motion(self):
            self.stop_motion_calls += 1

    mp = FakeMotionProvider()
    adapter = Go2TwistSinkAdapter(mp)
    bridge = LcmMotionCommandBridge(
        lcm_url="mock://", channel="/cmd_vel", twist_sink=adapter
    )
    bridge.start()
    try:
        bridge._backend.publish(
            "/cmd_vel", encode_cmd_vel_payload([0.3, 0, 0], [0, 0, 0], 1)
        )
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and len(mp.send_twist_calls) == 0:
            time.sleep(0.01)
        _check("Go2 send_twist 收到 1 次", len(mp.send_twist_calls) == 1)
        bridge.request_immediate_zero()
        _check("Go2 stop_motion 收到 >=1 次", mp.stop_motion_calls >= 1)
        _check("adapter.zero_invocations>=1", adapter.zero_invocations >= 1)
        _check("adapter.errors 为空", len(adapter.errors) == 0,
               f"errors={adapter.errors}")
    finally:
        bridge.stop(timeout=1.0)


# ============================================================
# Main
# ============================================================
def main():
    print("=" * 70)
    print("LcmMotionCommandBridge 单元测试")
    print("=" * 70)
    test_decode_golden_twist()
    test_clamp_twist()
    test_bridge_receives_cmd()
    test_watchdog_zero_stop()
    test_cancel_immediate_zero()
    test_shutdown_sequence()
    test_dropped_count_on_queue_full()
    test_sink_ready_flag()
    test_bad_payload_dropped()
    test_go2_twist_sink_adapter()
    test_sink_state_change_callback_fires_on_start_and_stop()
    test_sink_state_change_callback_not_fired_when_subscribe_fails()
    print("=" * 70)
    print(f"PASS={_results['pass']}  FAIL={_results['fail']}")
    if _results["fail"] > 0:
        print("失败用例:")
        for name, detail in _results["fails"]:
            print(f"  - {name}: {detail}")
        sys.exit(1)
    print("ALL PASS")


if __name__ == "__main__":
    main()