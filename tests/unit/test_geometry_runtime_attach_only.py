from __future__ import annotations

from pathlib import Path

from services.geometry.geometry_runtime_client import GeometryRuntimeSubprocess


REPO_ROOT = Path(__file__).resolve().parents[2]
CLIENT_SOURCE = REPO_ROOT / "services" / "geometry" / "geometry_runtime_client.py"
RUNTIME_SOURCE = REPO_ROOT / "services" / "geometry" / "runtime.py"
CANONICAL_LAUNCHER = REPO_ROOT / "scripts/supervisor.py"
ORPHAN_LAUNCHER = (
    REPO_ROOT
    / "services"
    / "geometry"
    / "scripts"
    / "start_nuwax_geometry_runtime.sh"
)




def test_python_geometry_client_fails_closed_when_canonical_owner_is_absent(
    monkeypatch,
) -> None:
    client = GeometryRuntimeSubprocess(port=50123)
    monkeypatch.setattr(client, "_is_port_open", lambda: False)

    assert client.start() is False
    assert client.is_running is False
    assert "python -m scripts.supervisor" in str(client.last_error)


def test_python_geometry_client_attaches_only_to_healthy_runtime(monkeypatch) -> None:
    client = GeometryRuntimeSubprocess(port=50123)
    monkeypatch.setattr(client, "_is_port_open", lambda: True)
    monkeypatch.setattr(client, "_verify_attached_runtime_locked", lambda: True)

    assert client.start() is True
    assert client.is_running is True
    client.stop()
    assert client.is_running is False
