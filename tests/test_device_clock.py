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
    assert time.time_ns() <= clock.source_now_ns() <= time.time_ns()
    start = time.monotonic()
    clock.sleep(0.004)
    assert clock.monotonic() - start >= 0.004
    ready = threading.Event()
    ready.set()
    assert clock.wait_until(clock.monotonic() + 1, ready.wait)


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
        assert link.readlink() == platform_clock
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
