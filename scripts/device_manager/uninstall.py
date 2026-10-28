"""只卸载当前实例拥有的程序；默认保留配置与业务数据。"""
from __future__ import annotations

import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

from .common import ManagerError, file_lock, process_alive, read_json


def _stop_service(engine, service):
    if service not in ("system", "user"):
        return
    name = "neh-manager-{}.service".format(os.getuid()) if service == "system" else "neh-manager.service"
    command = ["systemctl"] + (["--user"] if service == "user" else [])
    state = subprocess.run(command + ["show", name, "--property=ExecStart", "--value"], capture_output=True, text=True, check=True)
    if str(engine.manager_home / "manager/current") not in state.stdout:
        raise ManagerError("service_owner", "管理服务不属于本安装，拒绝卸载。", 409)
    prefix = ["sudo"] if service == "system" else []
    subprocess.run(prefix + command + ["disable", "--now", name], check=True)
    unit = Path("/etc/systemd/system") / name if service == "system" else Path.home() / ".config/systemd/user" / name
    if unit.is_file() and str(engine.manager_home / "manager/current") in unit.read_text():
        subprocess.run(["sudo", "rm", "--", str(unit)], check=True) if service == "system" else unit.unlink()
    subprocess.run(prefix + command + ["daemon-reload"], check=True)


def uninstall(engine, *, purge=False):
    with file_lock(engine.lock_path):
        engine._assert_idle()
        current = engine.current()
        if current and engine.runtime.has_owner():
            readiness = engine.runtime.update_readiness()
            if not readiness["ready"]:
                raise ManagerError("not_idle", readiness.get("reason", "设备尚未空闲"), 409)
            engine.runtime.stop(Path(current["prefix"]))
        engine.configuration.close()
        _stop_service(engine, read_json(engine.manager_home / "config/bootstrap.json", {}).get("service", "none"))
        server = read_json(engine.manager_home / "config/server.json", {}).get("process")
        if process_alive(server):
            cwd = Path("/proc/{}/cwd".format(server["pid"])).resolve()
            if engine.manager_home / "manager/versions" not in cwd.parents:
                raise ManagerError("server_owner", "管理进程不属于本安装，拒绝停止。", 409)
            os.kill(server["pid"], signal.SIGTERM)
            deadline = time.monotonic() + 15
            while process_alive(server) and time.monotonic() < deadline:
                time.sleep(0.1)
            if process_alive(server):
                raise ManagerError("server_stop", "管理进程尚未退出，卸载已推迟。", 409)
        links = read_json(engine.manager_home / "config/command-links.json", {}).get("links", [])
        for value in links:
            path = Path(value)
            if path.is_symlink() and path.resolve() == engine.manager_home / "bin/neh":
                path.unlink()
        for root in {engine.home, engine.manager_home}:
            if purge:
                shutil.rmtree(root)
            else:
                for name in ("current", "releases", "python-environments", "manager", "bin"):
                    path = root / name
                    if path.is_symlink():
                        path.unlink()
                    elif path.is_dir():
                        shutil.rmtree(path)
        return {"status": "uninstalled", "purged": purge, "data_retained": not purge, "install_dir": str(engine.home)}
