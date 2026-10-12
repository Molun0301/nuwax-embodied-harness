from __future__ import annotations

from pathlib import Path

from services.geometry.geometry_runtime_client import GeometryRuntimeSubprocess


REPO_ROOT = Path(__file__).resolve().parents[2]
CLIENT_SOURCE = REPO_ROOT / "services" / "geometry" / "geometry_runtime_client.py"
RUNTIME_SOURCE = REPO_ROOT / "services" / "geometry" / "runtime.py"
CANONICAL_LAUNCHER = REPO_ROOT / "scripts" / "systemd" / "start_geometry_runtime.sh"
ORPHAN_LAUNCHER = (
    REPO_ROOT
    / "services"
    / "geometry"
    / "scripts"
    / "start_nuwax_geometry_runtime.sh"
)


def test_python_geometry_client_contains_no_direct_process_spawn_path() -> None:
    source = CLIENT_SOURCE.read_text(encoding="utf-8")

    assert "subprocess.Popen" not in source
    assert "def _spawn_process" not in source
    assert "def _monitor_loop" not in source


def test_python_geometry_client_fails_closed_when_canonical_owner_is_absent(
    monkeypatch,
) -> None:
    client = GeometryRuntimeSubprocess(port=50123)
    monkeypatch.setattr(client, "_is_port_open", lambda: False)

    assert client.start() is False
    assert client.is_running is False
    assert "scripts/systemd/start_geometry_runtime.sh" in str(client.last_error)


def test_python_geometry_client_attaches_only_to_healthy_runtime(monkeypatch) -> None:
    client = GeometryRuntimeSubprocess(port=50123)
    monkeypatch.setattr(client, "_is_port_open", lambda: True)
    monkeypatch.setattr(client, "_verify_ready_locked", lambda: True)
    monkeypatch.setattr(client, "_verify_attached_runtime_locked", lambda: True)
    monkeypatch.setattr(client, "_warn_if_video_relay_missing", lambda: None)

    assert client.start() is True
    assert client.is_running is True
    client.stop()
    assert client.is_running is False


def test_only_canonical_geometry_launcher_remains_and_runtime_reports_it() -> None:
    runtime_source = RUNTIME_SOURCE.read_text(encoding="utf-8")

    assert CANONICAL_LAUNCHER.is_file()
    assert not ORPHAN_LAUNCHER.exists()
    assert "scripts/systemd/start_geometry_runtime.sh" in runtime_source
    assert '_geometry_script("start_nuwax_geometry_runtime.sh")' not in runtime_source
