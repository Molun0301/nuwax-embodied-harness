"""Persisted, adapter-neutral installation ownership and lifecycle operations.

No service is installed or started while rendering an installation. User services
are created only by an explicit start; privileged system services are untouched.
"""
from __future__ import annotations

import os
import re
import signal
import subprocess
import time
from pathlib import Path
from typing import Mapping, Optional

from scripts.device_manager.common import atomic_write
from scripts.device_manager.runtime import base_environment

from .runtime import effective_settings

MODES = ("foreground", "systemd-user")


def _service_name(name, mode):
    if mode not in MODES:
        raise ValueError("Unsupported lifecycle mode: " + str(mode))
    if name is None and mode == "foreground":
        return None
    if (not isinstance(name, str) or len(name) > 255 or ".." in name
            or not re.fullmatch(r"neh-[A-Za-z0-9][A-Za-z0-9_.:@-]*\.service", name)
            or name.endswith("@.service") or name.count("@") > 1):
        raise ValueError("Invalid lifecycle service name")
    return name


def service_identifier(home: Path) -> Optional[str]:
    """Read the completed installation's opaque registration without allocating."""
    return inspect_installation(home)["lifecycle"]["service_name"]


def deployment_descriptor(root: Path, home: Path, prefix: Path, modules, robot: str,
                          mode: str = "foreground", environment=None, *, service_name=None) -> dict:
    service_name = _service_name(service_name, mode)
    root, home, prefix = root.resolve(), home.resolve(), prefix.resolve()
    values, settings_path = effective_settings(root, prefix, environment=environment)
    host = values.get("NUWAX_GATEWAY_HOST", "127.0.0.1") or "127.0.0.1"
    port = int(values.get("NUWAX_GATEWAY_PORT", "8766") or "8766")
    if not 1 <= port <= 65535:
        raise ValueError("Invalid gateway listener port")
    return {"version": 1, "source_root": str(root), "install_home": str(home),
            "robot_adapter": robot, "modules": list(modules), "prefix": str(prefix),
            "lifecycle": {"mode": mode, "service_name": service_name},
            "listener": {"host": host, "port": port, "settings_path": str(settings_path)},
            "pid_file": str(root / "runtime_data/gateway/gateway.pid")}


def inspect_installation(home: Path, environment=None) -> dict:
    from .manager import read_current
    home = Path(home).resolve()
    current = read_current(home)
    if not current:
        raise ValueError("No completed managed installation")
    saved = current["deployment"]
    if not isinstance(saved, Mapping) or type(saved.get("version")) is not int or saved["version"] != 1:
        raise ValueError("Unsupported deployment descriptor version")
    registration = saved.get("lifecycle")
    if not isinstance(registration, Mapping) or "service_name" not in registration:
        raise ValueError("Invalid deployment lifecycle registration")
    mode = registration.get("mode")
    name = _service_name(registration["service_name"], mode)
    descriptor = deployment_descriptor(Path(current["source_root"]), home, Path(current["prefix"]),
                                       current["modules"], current["robot"], mode, environment,
                                       service_name=name)
    for key in ("source_root", "install_home", "robot_adapter", "modules", "prefix"):
        if saved.get(key) != descriptor[key]:
            raise ValueError("Deployment metadata mismatch: " + key)
    return descriptor


def _check(descriptor):
    if type(descriptor.get("version")) is not int or descriptor["version"] != 1:
        raise ValueError("Unsupported deployment descriptor version")
    mode = descriptor["lifecycle"]["mode"]
    name = _service_name(descriptor["lifecycle"]["service_name"], mode)
    current = inspect_installation(Path(descriptor["install_home"]))
    if any(descriptor.get(key) != current[key] for key in ("install_home", "source_root")):
        raise ValueError("Invalid lifecycle installation owner")
    registered = current["lifecycle"]
    if mode != registered["mode"] or name != registered["service_name"]:
        raise ValueError("Invalid lifecycle identity")
    return mode, name


def _user_only():
    if os.geteuid() == 0:
        raise RuntimeError("Lifecycle operations require an unprivileged installation owner")


def _unit_quote(value, *, executable=False):
    # systemd performs percent specifier expansion even inside quotes.
    value = str(value).replace("$", "$$") if executable else str(value)
    return '"' + value.replace('\\', '\\\\').replace('"', '\\"').replace('%', '%%').replace('\n', '\\n') + '"'


def render_user_service(descriptor: Mapping) -> str:
    _, name = _check(descriptor)
    if name is None:
        raise ValueError("No registered lifecycle service name")
    home = Path(descriptor["install_home"])
    return ("[Unit]\nDescription=NEH installed gateway\n\n[Service]\nType=simple\n"
            + "WorkingDirectory=" + _unit_quote(descriptor["source_root"]) + "\n"
            + "ExecStart=/bin/bash " + _unit_quote(home / "current/start.sh", executable=True) + "\n"
            + "Environment=" + _unit_quote("NUWAX_GATEWAY_HOST=" + descriptor["listener"]["host"]) + "\n"
            + "Environment=" + _unit_quote("NUWAX_GATEWAY_PORT=" + str(descriptor["listener"]["port"])) + "\n"
            + "KillMode=control-group\nTimeoutStopSec=60\nRestart=on-failure\nRestartSec=3\n"
            + "\n[Install]\nWantedBy=default.target\n")


def _exited(pid):
    try:
        return (Path("/proc") / str(pid) / "stat").read_text().rsplit(")", 1)[1].split()[0] == "Z"
    except FileNotFoundError:
        return True


def stop_current(descriptor: Mapping) -> int:
    _user_only()
    mode, name = _check(descriptor)
    if mode == "systemd-user":
        subprocess.run(["systemctl", "--user", "stop", name], check=True)
        return 0
    try:
        pid = int(Path(descriptor["pid_file"]).read_text().strip())
        if pid <= 1:
            raise ValueError("Invalid gateway PID")
        if _exited(pid):
            return 0
        identity = ("NUWAX_INSTALL_HOME=" + descriptor["install_home"]).encode()
        try:
            owned = identity in (Path("/proc") / str(pid) / "environ").read_bytes().split(b"\0")
        except OSError:
            owned = False
        if not owned:
            Path(descriptor["pid_file"]).unlink(missing_ok=True)
            return 0
        os.kill(pid, signal.SIGTERM)
        for _ in range(300):
            if _exited(pid):
                return 0
            time.sleep(0.2)
        raise RuntimeError("Gateway shutdown is still pending; installation retained")
    except (FileNotFoundError, ProcessLookupError):
        return 0


def start_current(descriptor: Mapping, *, foreground=False, log=None):
    _user_only()
    mode, name = _check(descriptor)
    # Resolve at invocation time, so rollback starts the restored current link.
    home = Path(descriptor["install_home"])
    current = inspect_installation(home)
    if current["lifecycle"] != {"mode": mode, "service_name": name}:
        raise ValueError("Lifecycle registration changed before start")
    if mode == "systemd-user":
        unit = Path(os.environ.get("XDG_CONFIG_HOME") or Path.home() / ".config") / "systemd/user" / name
        unit.parent.mkdir(parents=True, exist_ok=True)
        atomic_write(unit, (render_user_service(current)).encode("utf-8"), mode=0o600)
        subprocess.run(["systemctl", "--user", "daemon-reload"], check=True)
        subprocess.run(["systemctl", "--user", "start", name], check=True)
        return None
    argv = ["/bin/bash", str(home / "current/start.sh")]
    environment = base_environment()
    environment["NUWAX_INSTALL_HOME"] = str(home)
    environment["NUWAX_GATEWAY_HOST"] = current["listener"]["host"]
    environment["NUWAX_GATEWAY_PORT"] = str(current["listener"]["port"])
    if foreground:
        os.execve(argv[0], argv, environment)
    return subprocess.Popen(argv, cwd=current["source_root"], env=environment, stdin=subprocess.DEVNULL,
                            stdout=log, stderr=subprocess.STDOUT, start_new_session=True, close_fds=True)
