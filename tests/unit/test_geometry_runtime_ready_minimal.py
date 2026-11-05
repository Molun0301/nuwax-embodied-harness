from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys
import types

import pytest

from services.geometry.geometry_runtime_client import GeometryRuntimeSubprocess


REPO_ROOT = Path(__file__).resolve().parents[2]
CLIENT_SOURCE = REPO_ROOT / "services/geometry/geometry_runtime_client.py"




def _write_executable(path: Path, body: str) -> None:
    path.write_text("#!/usr/bin/env bash\nset -euo pipefail\n" + body, encoding="utf-8")
    path.chmod(0o755)

















@pytest.mark.parametrize("grpc_fails", [True])
def test_client_always_calls_real_get_status_gate(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    grpc_fails: bool,
) -> None:
    from services.geometry import proto_gen

    monkeypatch.setenv("NUWAX_GEOMETRY_RUNTIME_SKIP_HEALTH_CHECK", "1")
    calls = []

    class FakeChannel:
        def close(self) -> None:
            calls.append("close")

    class FakeStub:
        def HealthCheck(self, request, timeout):
            return types.SimpleNamespace(runtime_generation=1)

        def GetStatus(self, request, timeout):
            calls.append((request, timeout))
            if grpc_fails:
                raise RuntimeError("GetStatus failed")
            return object()

    grpc_module = types.ModuleType("grpc")
    grpc_module.insecure_channel = lambda target: calls.append(target) or FakeChannel()
    pb2_module = types.ModuleType("geometry_runtime_pb2")
    pb2_module.Empty = lambda: "empty"
    pb2_module.HealthCheckRequest = lambda: "health"
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

    assert client.start() is (not grpc_fails)
    assert "127.0.0.1:50123" in calls
    assert any(isinstance(item, tuple) and item[0] == "empty" for item in calls)
    assert "close" in calls
