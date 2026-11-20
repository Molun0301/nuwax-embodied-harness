"""验证系统时钟、共享内存和设备绑定的符号链接。"""

import asyncio
import os
import struct
import threading
import time

import httpx
import pytest

from core import clock
from drivers.unitree_sim import device


@pytest.fixture
def device_clock(tmp_path, monkeypatch):
    path = tmp_path / "clock"
    path.write_bytes(struct.pack("<8sQQdQ", b"USIMCLK1", 2, 1_000_000_000, 1.0, 0))
    monkeypatch.setenv("NUWAX_DEVICE_CLOCK", str(path))
    monkeypatch.setattr(clock, "_mapping", None)
    return path


def publish(path, sequence, ns, rate=1.0, paused=False):
    with path.open("r+b", buffering=0) as writer:
        writer.seek(8)
        writer.write(struct.pack("<Q", sequence - 1))
        writer.write(struct.pack("<QdQ", ns, rate, paused))
        writer.seek(8)
        writer.write(struct.pack("<Q", sequence))


def test_system_clock(monkeypatch):
    monkeypatch.delenv("NUWAX_DEVICE_CLOCK", raising=False)
    monkeypatch.delenv(clock.CLOCK_OFFSET_ENV, raising=False)
    assert time.time_ns() <= clock.source_now_ns() <= time.time_ns()
    start = time.monotonic()
    clock.sleep(0.004)
    assert clock.monotonic() - start >= 0.004
    ready = threading.Event()
    ready.set()
    assert clock.wait_until(clock.monotonic() + 1, ready.wait)


def test_go2_reader_uses_recent_largest_offset_and_keeps_ticking(tmp_path, monkeypatch):
    from types import SimpleNamespace

    from drivers.go2.data_plane import Go2DirectDdsBridge
    from drivers.go2.settings import Go2DataPlaneConfig

    path = tmp_path / "clock-offset"
    monkeypatch.setenv(clock.CLOCK_OFFSET_ENV, str(path))
    monkeypatch.delenv("NUWAX_DEVICE_CLOCK", raising=False)
    clock.initialize_source_clock_offset(path)
    now = [1_700_000_000_000_000_000, 10.0]
    monkeypatch.setattr(clock.time, "time_ns", lambda: now[0])
    monkeypatch.setattr(clock.time, "monotonic", lambda: now[1])
    assert clock.source_now_ns() == now[0]
    assert not clock.uses_device_clock()
    bridge = Go2DirectDdsBridge(Go2DataPlaneConfig())
    bridge._started = bridge._direct_dds_started = True
    bridge._active_generation = 1
    bridge._direct_dds_stop_event = threading.Event()
    offset = 71_844_000_000_000

    def receive(kind, elapsed_ns, estimate, wall_jump=0):
        now[0] = 1_700_000_000_000_000_000 + elapsed_ns + wall_jump
        now[1] = 10 + elapsed_ns / 1e9
        source = now[0] + estimate
        sec, nanosec = divmod(source, 1_000_000_000)
        bridge._record_dds_input_timing(
            kind, SimpleNamespace(sec=sec, nanosec=nanosec), 1, now[1], now[1], now[0],
        )
        return source

    receive("imu", 0, offset)
    lidar = receive("pointcloud", 200_000_000, offset - 30_000_000)
    assert clock.source_now_ns() - lidar == 30_000_000
    assert bridge.get_status()["source_clock_offset_ns"] == offset
    assert bridge.get_status()["input_timing"]["pointcloud"]["source_age_ms_latest"] == 30
    latest = receive("pointcloud", 2_100_000_000, offset - 40_000_000)
    assert clock.source_now_ns() - latest == 10_000_000
    assert clock.source_clock_offset_ns() == offset - 30_000_000
    corrected = receive("imu", 2_200_000_000, -5_000_000, wall_jump=offset)
    assert clock.source_clock_offset_ns() == -5_000_000
    assert clock.source_now_ns() == corrected
    # 另一读取线程稍后完成的旧墙钟样本仍处于时间窗口内。
    old_local = now[0] - offset - 10_000_000
    sec, nanosec = divmod(old_local + offset, 1_000_000_000)
    bridge._record_dds_input_timing(
        "pointcloud", SimpleNamespace(sec=sec, nanosec=nanosec), 1,
        now[1] - 0.01, now[1] - 0.01, old_local,
    )
    assert clock.source_clock_offset_ns() == -5_000_000
    deadline = clock.monotonic() + 1
    now[0] += 4_000_000_000
    now[1] += 4
    assert clock.source_now_ns() - corrected == 4_000_000_000
    assert clock.next_wait(deadline) == 0


def test_supervisor_chooses_source_offset_only_for_go2(tmp_path, monkeypatch):
    from pathlib import Path

    from core.config import ConfigStore
    from scripts.supervisor import Supervisor, load_environment

    monkeypatch.setenv("NUWAX_INSTALL_HOME", str(tmp_path))
    root = Path(__file__).resolve().parents[1]
    store = ConfigStore(tmp_path)
    store.create("开发")
    store.activate("开发")
    store.save({"robot.driver": "go2"})
    env = load_environment(root)
    assert "NUWAX_DEVICE_CLOCK" not in env
    path = tmp_path / "run/device_clock_offset"
    assert env[clock.CLOCK_OFFSET_ENV] == str(path)

    async def run():
        owner = Supervisor(root, env, components={})
        owner.stopping.set()
        await owner.run()

    asyncio.run(run())
    assert struct.unpack("@q", path.read_bytes()) == (0,)
    store.save({"robot.driver": "unitree_sim"})
    env = load_environment(root)
    assert env["NUWAX_DEVICE_CLOCK"] == str(tmp_path / "run/device_clock")
    assert clock.CLOCK_OFFSET_ENV not in env


@pytest.mark.parametrize(("offset", "late"), [
    (2_000_000_000, False), (-2_000_000_000, False), (1_999_999_999, False),
    (None, False), (2_000_000_000, True), (0, False),
])
def test_manager_sets_time_once_using_this_startup_offset(tmp_path, monkeypatch, offset, late):
    from types import SimpleNamespace
    from unittest.mock import Mock

    from core.config import atomic_write
    from scripts.device_manager import runtime

    (tmp_path / "logs").mkdir()
    (tmp_path / "config").mkdir()
    path = tmp_path / "run/device_clock_offset"
    atomic_write(path, struct.pack("@q", 71_844_000_000_000))
    helper = Mock()
    now = 1_700_000_000_000_000_000
    monkeypatch.setattr(runtime, "system_helper", helper)
    monkeypatch.setattr(runtime, "probe", lambda *_: {"transport.real_lcm_required": False})
    monkeypatch.setattr(runtime, "clean_environment", lambda *args, **kwargs: {})
    elapsed = 0.0
    waiting = threading.Event()
    published = threading.Event()
    threads = []

    def sleep(seconds):
        nonlocal elapsed
        if late:
            waiting.set()
            assert published.wait(2)
        elapsed += seconds

    def create_thread(*args, **kwargs):
        thread = threading.Thread(*args, **kwargs)
        threads.append(thread)
        return thread

    monkeypatch.setattr(runtime, "time", SimpleNamespace(
        monotonic=lambda: elapsed, time_ns=lambda: now, sleep=sleep,
    ))
    monkeypatch.setattr(runtime, "threading", SimpleNamespace(Thread=create_thread))

    def launch(*args, **kwargs):
        if offset is not None:
            atomic_write(path, struct.pack("@q", 0 if late else offset))
        return SimpleNamespace(poll=lambda: None)

    monkeypatch.setattr(runtime.subprocess, "Popen", launch)
    owner = runtime.Runtime(tmp_path)
    monkeypatch.setattr(owner, "status", Mock(return_value={"running": False}))
    monkeypatch.setattr(owner, "gateway_healthy", Mock(return_value=True))
    try:
        owner.start(tmp_path)
        if late:
            assert waiting.wait(2)
            helper.assert_not_called()
            clock.publish_source_clock_offset_ns(str(path), offset)
    finally:
        published.set()
        for thread in threads:
            thread.join(2)
            assert not thread.is_alive()
    assert len(threads) == 1
    assert not threads[0].daemon
    if offset == 0:
        assert elapsed == pytest.approx(15)
    if offset is not None and abs(offset) == 2_000_000_000:
        helper.assert_called_once_with("set-system-time", {"unix_ns": now + offset})
    else:
        helper.assert_not_called()


@pytest.mark.parametrize("rtc_failure", [None, "硬件时钟不可用"])
def test_fixed_helper_sets_unix_nanoseconds_even_if_rtc_write_fails(monkeypatch, capsys, rtc_failure):
    from types import SimpleNamespace
    from unittest.mock import Mock

    from scripts import neh_system_helper as helper

    clock_set = Mock()
    write_rtc = Mock(return_value=SimpleNamespace(returncode=int(rtc_failure is not None), stdout=rtc_failure or ""))
    monkeypatch.setattr(helper.time, "clock_settime_ns", clock_set)
    monkeypatch.setattr(helper, "run", write_rtc)
    timestamp = 1_795_147_740_123_456_789
    assert helper.dispatch("set-system-time", {"unix_ns": timestamp}) == {"system_time_set": True}
    clock_set.assert_called_once_with(time.CLOCK_REALTIME, timestamp)
    write_rtc.assert_called_once_with(["hwclock", "--systohc"], check=False, timeout=30)
    assert ("写入硬件时钟失败" in capsys.readouterr().err) == (rtc_failure is not None)


def test_device_rate_and_pause(device_clock):
    deadline = clock.monotonic() + 0.004
    for sequence, rate, seconds in [(4, 0.5, 0.008), (6, 1, 0.004), (8, 2.5, 0.0016), (10, 0, 0.001)]:
        publish(device_clock, sequence, 1_000_000_000, rate)
        assert clock.next_wait(deadline) == pytest.approx(seconds)
    publish(device_clock, 12, 1_000_000_000, 0.5)
    assert clock.next_wait(deadline + 1) == 0.010
    publish(device_clock, 14, 1_000_000_000, 2.5, paused=True)
    assert clock.next_wait(deadline) == 0.010

    async def exercise():
        wait = asyncio.create_task(clock.async_sleep(0.004))
        await asyncio.sleep(0.035)
        assert not wait.done()
        publish(device_clock, 16, 1_004_000_000, 2.5)
        await asyncio.wait_for(wait, 1)

    asyncio.run(exercise())
    assert clock.next_wait(deadline) == 0
    assert clock.monotonic_ns() == clock.source_now_ns() == 1_004_000_000


def test_clock_waits_for_missing_file_and_even_sequence(tmp_path, monkeypatch):
    path = tmp_path / "late-clock"
    monkeypatch.setenv("NUWAX_DEVICE_CLOCK", str(path))
    monkeypatch.setattr(clock, "_mapping", None)
    result = []
    reader = threading.Thread(target=lambda: result.append(clock.source_now_ns()), daemon=True)
    reader.start()
    reader.join(0.015)
    assert reader.is_alive()
    writer = tmp_path / "writer"
    writer.write_bytes(struct.pack("<8sQQdQ", b"USIMCLK1", 1, 7, 1.0, 0))
    os.replace(writer, path)
    reader.join(0.015)
    assert reader.is_alive()
    publish(path, 2, 9)
    reader.join(1)
    assert result == [9]


def test_driver_links_clock_before_feedback(tmp_path, monkeypatch):
    platform_clock = tmp_path / "platform-clock"
    platform_clock.write_bytes(struct.pack("<8sQQdQ", b"USIMCLK1", 2, 41, 1.37, 0))
    home = tmp_path / "neh"
    link = home / "run/device_clock"
    link.parent.mkdir(parents=True)
    link.symlink_to(tmp_path / "old-clock")
    monkeypatch.setenv("NUWAX_INSTALL_HOME", str(home))
    monkeypatch.setenv("NUWAX_DEVICE_CLOCK", str(link))
    monkeypatch.setattr(clock, "_mapping", None)

    def respond(request):
        if request.url.path.endswith("/capabilities"):
            return httpx.Response(200, json={
                "protocol": "unitree-sim/1",
                "backend": {"robot": "go2"},
                "clock": {"shared_memory_path": str(platform_clock)},
            })
        return httpx.Response(200, json={
            "mode": "physics", "session": "test", "epoch": 1, "robot_status": {}, "sim_time_ns": 0,
        })

    client = httpx.Client
    monkeypatch.setattr(device.httpx, "Client", lambda **kw: client(transport=httpx.MockTransport(respond), **kw))
    simulation = device.Device("http://test", "go2", "base")
    monkeypatch.setattr(simulation, "subscribe", lambda *args: None)
    observed = []
    monkeypatch.setattr(simulation, "_status", lambda *args: observed.append(clock.monotonic_ns()))
    try:
        simulation.start()
        assert link.is_symlink()
        assert os.readlink(link) == str(platform_clock)
        assert observed == [41]
    finally:
        simulation.close()


def test_bind_running_status_builds_motion_feedback(device_clock, tmp_path, monkeypatch):
    home = tmp_path / "neh"
    monkeypatch.setenv("NUWAX_INSTALL_HOME", str(home))
    monkeypatch.setenv("NUWAX_DEVICE_CLOCK", str(home / "run/device_clock"))

    def respond(request):
        if request.url.path.endswith("/capabilities"):
            return httpx.Response(200, json={
                "protocol": "unitree-sim/1",
                "backend": {
                    "robot": "g1",
                    "command_limits": {"vx_mps": 1, "vy_mps": 1, "wz_rps": 1, "source": "test"},
                },
                "clock": {"shared_memory_path": str(device_clock)},
            })
        return httpx.Response(200, json={
            "mode": "physics",
            "session": "test",
            "epoch": 1,
            "sim_time_ns": 100,
            "robot_status": {
                "state": "running",
                "source_clock_mapping": {
                    "clock_id": "device-clock",
                    "unix_anchor_ns": 999_999_950,
                    "sim_anchor_ns": 50,
                },
                "velocity": {
                    "reference_frame_id": "base",
                    "reference_linear_mps": [0.25, 0.0, 0.0],
                    "reference_angular_rps": [0.0, 0.0, 0.5],
                    "reference_orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
                    "command_from_reference_rotation_matrix": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                    "command_frame_id": "base",
                },
            },
        })

    client = httpx.Client
    monkeypatch.setattr(device.httpx, "Client", lambda **kw: client(transport=httpx.MockTransport(respond), **kw))
    simulation = device.Device("http://test", "g1", "base")
    try:
        simulation.bind()
        feedback = simulation.get_motion_execution_feedback()
        assert feedback is not None
        assert feedback.source_id == "unitree-sim/test/1/device-clock"
        assert feedback.source_timestamp_ns == feedback.received_monotonic_ns == 1_000_000_000
        assert (feedback.linear_x_mps, feedback.angular_z_rps) == (0.25, 0.5)
    finally:
        simulation.close()
