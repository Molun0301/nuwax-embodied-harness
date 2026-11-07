"""Persisted, adapter-neutral installation ownership and lifecycle operations.

No service is installed or started while rendering an installation. User services
are created only by an explicit start; privileged system services are untouched.
"""
from __future__ import annotations

import os
import re
import subprocess
from pathlib import Path
from typing import Mapping, Optional

from scripts.device_manager.common import atomic_json, atomic_write
from scripts.device_manager.runtime import Runtime, base_environment

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
    host = values["gateway.host"]
    port = int(values["gateway.port"])
    if not 1 <= port <= 65535:
        raise ValueError("Invalid gateway listener port")
    return {"version": 1, "source_root": str(root), "install_home": str(home),
            "robot_adapter": robot, "modules": list(modules), "prefix": str(prefix),
            "lifecycle": {"mode": mode, "service_name": service_name},
            "listener": {"host": host, "port": port, "settings_path": str(settings_path)}}


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
            + "ExecStart=" + _unit_quote(Path(descriptor["prefix"]) / "venv/bin/python", executable=True) + " -m scripts.supervisor\n"
            + "Environment=" + _unit_quote("NUWAX_INSTALL_HOME=" + str(home)) + "\n"
            + "Environment=" + _unit_quote("LD_LIBRARY_PATH=" + str(Path(descriptor["prefix"]) / "lib") + ":" + str(Path(descriptor["prefix"]) / "lib64")) + "\n"
            + "KillMode=process\nTimeoutStopSec=60\nRestart=on-failure\nRestartSec=3\n"
            + "\n[Install]\nWantedBy=default.target\n")


def stop_current(descriptor: Mapping) -> int:
    _user_only()
    mode, name = _check(descriptor)
    if mode == "systemd-user":
        subprocess.run(["systemctl", "--user", "stop", name], check=True)
        return 0
    Runtime(Path(descriptor["install_home"])).stop(Path(descriptor["prefix"]))
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
    state = home / "config/runtime.json"
    if foreground:
        atomic_json(state, {"pid": os.getpid(), "prefix": current["prefix"]})
        os.execve(argv[0], argv, environment)
    process = subprocess.Popen(argv, cwd=current["source_root"], env=environment, stdin=subprocess.DEVNULL,
                               stdout=log, stderr=subprocess.STDOUT, start_new_session=True, close_fds=True)
    atomic_json(state, {"pid": process.pid, "prefix": current["prefix"]})
    return process
