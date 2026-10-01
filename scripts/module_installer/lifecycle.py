"""Persisted, adapter-neutral installation ownership and lifecycle operations.

No service is installed or started while rendering an installation. User services
are created only by an explicit start; privileged system services are untouched.
"""
from __future__ import annotations

import hashlib
import os
import signal
import subprocess
import time
import uuid
from pathlib import Path
from typing import Mapping

from scripts.deploy_installer.config import atomic_write

from .runtime import effective_settings

MODES = ("foreground", "systemd-user")


def service_identifier(home: Path) -> str:
    return "neh-" + hashlib.sha256(str(home.resolve()).encode()).hexdigest()[:16] + ".service"


def deployment_descriptor(root: Path, home: Path, prefix: Path, modules, robot: str,
                          mode: str = "foreground", environment=None) -> dict:
    if mode not in MODES:
        raise ValueError("Unsupported lifecycle mode: " + str(mode))
    root, home, prefix = root.resolve(), home.resolve(), prefix.resolve()
    values, settings_path = effective_settings(root, prefix, environment=environment)
    host = values.get("NUWAX_GATEWAY_HOST", "127.0.0.1") or "127.0.0.1"
    port = int(values.get("NUWAX_GATEWAY_PORT", "8766") or "8766")
    if not 1 <= port <= 65535:
        raise ValueError("Invalid gateway listener port")
    return {"version": 1, "source_root": str(root), "install_home": str(home),
            "robot_adapter": robot, "modules": list(modules), "prefix": str(prefix),
            "lifecycle": {"mode": mode, "service_name": service_identifier(home)},
            "listener": {"host": host, "port": port, "settings_path": str(settings_path)},
            "pid_file": str(root / "runtime_data/gateway/gateway.pid")}


def inspect_installation(home: Path, environment=None) -> dict:
    from .manager import read_current
    home = Path(home).resolve()
    current = read_current(home)
    if not current:
        raise ValueError("No completed managed installation")
    saved = current.get("deployment", {})
    if saved and saved.get("version") != 1:
        raise ValueError("Unsupported deployment descriptor version")
    mode = saved.get("lifecycle", {}).get("mode", "foreground")
    descriptor = deployment_descriptor(Path(current["source_root"]), home, Path(current["prefix"]),
                                       current["modules"], current["robot"], mode, environment)
    if saved:
        for key in ("source_root", "install_home", "robot_adapter", "modules"):
            if saved.get(key) != descriptor[key]:
                raise ValueError("Deployment metadata mismatch: " + key)
        if saved.get("lifecycle", {}).get("service_name") != descriptor["lifecycle"]["service_name"]:
            raise ValueError("Deployment service identity mismatch")
    return descriptor


def _check(descriptor):
    mode = descriptor["lifecycle"]["mode"]
    name = descriptor["lifecycle"]["service_name"]
    if mode not in MODES or name != service_identifier(Path(descriptor["install_home"])):
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
    _check(descriptor)
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
        prefix = Path(descriptor["prefix"])
        identity = ("NUWAX_BOOTSTRAP_ENV_FILE=" + str(prefix / "settings.env")).encode()
        environment = (Path("/proc") / str(pid) / "environ").read_bytes().split(b"\0")
        if identity not in environment:
            raise RuntimeError("Gateway PID belongs to another installation; refusing to signal")
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
    # Preserve the effective endpoint that accepted the update, including any
    # explicit process overrides; generation-specific paths still come from current.
    current["listener"] = dict(descriptor["listener"])
    if mode == "systemd-user":
        unit = Path(os.environ.get("XDG_CONFIG_HOME") or Path.home() / ".config") / "systemd/user" / name
        unit.parent.mkdir(parents=True, exist_ok=True)
        atomic_write(unit, render_user_service(current), mode=0o600)
        subprocess.run(["systemctl", "--user", "daemon-reload"], check=True)
        subprocess.run(["systemctl", "--user", "start", name], check=True)
        return None
    argv = ["/bin/bash", str(home / "current/start.sh")]
    environment = dict(os.environ)
    for key in environment.get("_NUWAX_CONFIG_FILE_KEYS", "").split(":"):
        if key:
            environment.pop(key, None)
    for key in ("PYTHONHOME", "PYTHONPATH", "VIRTUAL_ENV", "_NUWAX_CONFIG_FILE_KEYS",
                "NUWAX_GATEWAY_SUPERVISOR_TITLED", "NUWAX_CONFIG_FROM_SYSTEMD_FILES",
                "NUWAX_BOOTSTRAP_ENV_FILE", "NUWAX_SETTINGS_LAUNCHER",
                "GO2_DATA_PLANE_ENV_FILE", "NUWAX_GEOMETRY_WEB_CONFIG_ENV_FILE",
                "NUWAX_GATEWAY_WEB_CONFIG_ENV_FILE", "NUWAX_USER_CONFIG_ENV_FILE"):
        environment.pop(key, None)
    environment["NUWAX_INSTALL_HOME"] = str(home)
    environment["NUWAX_GATEWAY_HOST"] = current["listener"]["host"]
    environment["NUWAX_GATEWAY_PORT"] = str(current["listener"]["port"])
    if foreground:
        os.execve(argv[0], argv, environment)
    return subprocess.Popen(argv, cwd=current["source_root"], env=environment, stdin=subprocess.DEVNULL,
                            stdout=log, stderr=subprocess.STDOUT, start_new_session=True, close_fds=True)


def spawn_updater(descriptor: Mapping, argv, *, environment=None, env=None, log=None):
    """Launch trusted updater argv outside the gateway's owner cgroup.

    User-systemd workers use the deployment account's persistent Git credentials.
    Gateway-only SSH agents, custom PATH and credential environment variables are
    intentionally not copied into transient unit arguments or properties.
    """
    _user_only()
    mode, name = _check(descriptor)
    argv = [str(item) for item in argv]
    if not argv or not Path(argv[0]).is_absolute():
        raise ValueError("Updater must use an absolute interpreter path")
    if env is not None and environment is not None:
        raise ValueError("Pass environment or env, not both")
    environment = environment if environment is not None else env
    env = dict(os.environ if environment is None else environment)
    env.pop("PYTHONHOME", None)
    if mode == "systemd-user":
        unit = name[:-8] + "-update-" + uuid.uuid4().hex
        command = ["systemd-run", "--user", "--collect", "--quiet", "--unit=" + unit,
                   "--property=Type=exec", "--property=KillMode=control-group",
                   "--working-directory=" + descriptor["source_root"]]
        # Only these non-secret, updater-specific values need transmission to
        # the user manager; do not copy the gateway's complete credential env.
        for key in ("PYTHONPATH", "PYTHONUNBUFFERED", "NUWAX_UPDATE_INSTALL_DIR", "NUWAX_INSTALL_HOME"):
            if key in env:
                command.append("--setenv=" + key + "=" + env[key])
        subprocess.run(command + ["--"] + argv, env=env, check=True,
                       stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT)
        return unit + ".service"
    return subprocess.Popen(argv, cwd=descriptor["source_root"], env=env,
                            stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True, close_fds=True)
