from __future__ import annotations

import os
import json
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import types

import pytest

from services.geometry.geometry_runtime_client import GeometryRuntimeSubprocess


REPO_ROOT = Path(__file__).resolve().parents[2]
LAUNCHER = REPO_ROOT / "scripts/systemd/start_geometry_runtime.sh"
WAITER = REPO_ROOT / "scripts/systemd/wait_for_geometry_runtime_ready.sh"
CLIENT_SOURCE = REPO_ROOT / "services/geometry/geometry_runtime_client.py"


def test_client_has_no_health_check_bypass() -> None:
    source = CLIENT_SOURCE.read_text(encoding="utf-8")

    assert "NUWAX_GEOMETRY_RUNTIME_SKIP_HEALTH_CHECK" not in source
    assert "_SKIP_ATTACH_HEALTH_CHECK" not in source


def _write_executable(path: Path, body: str) -> None:
    path.write_text("#!/usr/bin/env bash\nset -euo pipefail\n" + body, encoding="utf-8")
    path.chmod(0o755)


def _check_ready(ready: Path, port: str = "50051") -> int:
    return subprocess.run(
        ["bash", str(WAITER), "check"],
        env={
            **os.environ,
            "NUWAX_GEOMETRY_RUNTIME_READY_FILE": str(ready),
            "NUWAX_GEOMETRY_RUNTIME_PORT": port,
        },
        check=False,
    ).returncode












def test_client_minimal_ready_then_grpc_status_gate(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    ready = tmp_path / "geometry.ready"
    ready.write_text("pid=%d\nport=50123\n" % os.getpid(), encoding="utf-8")
    monkeypatch.setenv("NUWAX_GEOMETRY_RUNTIME_READY_FILE", str(ready))
    client = GeometryRuntimeSubprocess(port=50123)
    grpc_checks = []
    monkeypatch.setattr(client, "_is_port_open", lambda: True)
    monkeypatch.setattr(
        client,
        "_verify_attached_runtime_locked",
        lambda: grpc_checks.append(True) or True,
    )
    monkeypatch.setattr(client, "_warn_if_video_relay_missing", lambda: None)

    assert client._verify_ready_locked() is True
    assert client.start() is True
    assert grpc_checks == [True]

    ready.write_text(
        "pid=%d\nport=50123\nrelease_generation=old\n" % os.getpid(),
        encoding="utf-8",
    )
    assert client.is_running is False


def test_client_detects_changed_live_ready_pid_before_cached_health(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    ready = tmp_path / "geometry.ready"
    ready.write_text(f"pid={os.getpid()}\nport=50123\n", encoding="utf-8")
    monkeypatch.setenv("NUWAX_GEOMETRY_RUNTIME_READY_FILE", str(ready))
    client = GeometryRuntimeSubprocess(port=50123)
    health_calls = []
    monkeypatch.setattr(client, "_is_port_open", lambda: True)
    monkeypatch.setattr(
        client, "_verify_attached_runtime_locked", lambda: health_calls.append(True) or True
    )
    monkeypatch.setattr(client, "_warn_if_video_relay_missing", lambda: None)
    assert client.start() is True
    assert client.is_running is True
    assert len(health_calls) == 1

    replacement = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"])
    try:
        # 新旧进程同时存在且端口仍健康，旧缓存仍不得跨进程复用。
        ready.write_text(f"pid={replacement.pid}\nport=50123\n", encoding="utf-8")
        assert client.is_running is False
        assert len(health_calls) == 1
        assert client.start() is True
        assert client.is_running is True
        assert len(health_calls) == 2
        client.disconnect()
        assert replacement.poll() is None
        assert client.start() is True
        assert len(health_calls) == 3
    finally:
        replacement.terminate()
        replacement.wait(timeout=3)


@pytest.mark.parametrize("already_attached", [False, True])
def test_client_rejects_ready_process_change_during_health_check(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, already_attached: bool
) -> None:
    ready = tmp_path / "geometry.ready"
    ready.write_text(f"pid={os.getpid()}\nport=50123\n", encoding="utf-8")
    monkeypatch.setenv("NUWAX_GEOMETRY_RUNTIME_READY_FILE", str(ready))
    client = GeometryRuntimeSubprocess(port=50123)
    monkeypatch.setattr(client, "_is_port_open", lambda: True)
    monkeypatch.setattr(client, "_verify_attached_runtime_locked", lambda: True)
    monkeypatch.setattr(client, "_warn_if_video_relay_missing", lambda: None)
    if already_attached:
        assert client.start() is True
        client._attached_health_check_at = 0.0

    replacement = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"])
    try:
        def replace_during_health_check() -> bool:
            ready.write_text(f"pid={replacement.pid}\nport=50123\n", encoding="utf-8")
            return True

        monkeypatch.setattr(client, "_verify_attached_runtime_locked", replace_during_health_check)
        assert (client.is_running if already_attached else client.start()) is False
        assert client.is_running is False
        assert client.start() is True
        assert client.is_running is True
    finally:
        replacement.terminate()
        replacement.wait(timeout=3)


@pytest.mark.parametrize("grpc_fails", [False, True])
def test_client_always_calls_real_get_status_gate(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    grpc_fails: bool,
) -> None:
    from services.geometry import proto_gen

    ready = tmp_path / "geometry.ready"
    ready.write_text("pid=%d\nport=50123\n" % os.getpid(), encoding="utf-8")
    monkeypatch.setenv("NUWAX_GEOMETRY_RUNTIME_READY_FILE", str(ready))
    monkeypatch.setenv("NUWAX_GEOMETRY_RUNTIME_SKIP_HEALTH_CHECK", "1")
    calls = []

    class FakeChannel:
        def close(self) -> None:
            calls.append("close")

    class FakeStub:
        def GetStatus(self, request, timeout):
            calls.append((request, timeout))
            if grpc_fails:
                raise RuntimeError("GetStatus failed")
            return object()

    grpc_module = types.ModuleType("grpc")
    grpc_module.insecure_channel = lambda target: calls.append(target) or FakeChannel()
    pb2_module = types.ModuleType("geometry_runtime_pb2")
    pb2_module.Empty = lambda: "empty"
    pb2_grpc_module = types.ModuleType("geometry_runtime_pb2_grpc")
    pb2_grpc_module.GeometryRuntimeStub = lambda channel: FakeStub()
    monkeypatch.setitem(sys.modules, "grpc", grpc_module)
    monkeypatch.setitem(
        sys.modules,
        "services.geometry.proto_gen.geometry_runtime_pb2",
        pb2_module,
    )
    monkeypatch.setitem(
        sys.modules,
        "services.geometry.proto_gen.geometry_runtime_pb2_grpc",
        pb2_grpc_module,
    )
    monkeypatch.setattr(proto_gen, "geometry_runtime_pb2", pb2_module, raising=False)
    monkeypatch.setattr(
        proto_gen,
        "geometry_runtime_pb2_grpc",
        pb2_grpc_module,
        raising=False,
    )

    client = GeometryRuntimeSubprocess(port=50123)
    monkeypatch.setattr(client, "_is_port_open", lambda: True)
    monkeypatch.setattr(client, "_warn_if_video_relay_missing", lambda: None)

    assert client.start() is (not grpc_fails)
    assert "127.0.0.1:50123" in calls
    assert any(isinstance(item, tuple) and item[0] == "empty" for item in calls)
    assert "close" in calls
