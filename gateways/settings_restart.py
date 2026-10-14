"""把重启交回现有启动拥有方；请求不包含可执行命令或任意服务名。"""
from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid

from core.config_io import _atomic_write, _target_lock


def restart_status(root):
    path = root / "runtime_data/settings/restart.json"
    try:
        status = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        status = {"state": "idle"}
    except (OSError, ValueError):
        status = {"state": "failed", "message": "无法读取重启回执"}
    if status.get("state") in {"starting", "restarting"} and status.get("previous_process_id") != os.getpid():
        status = dict(status, state="restarted", message="新网关已启动；相关功能就绪状态以控制台为准")
    if status.get("state") in {"queued", "restarting", "starting"} and time.time() - status.get("created_at", 0) > 180:
        status = dict(status, state="failed", message="重启未在期限内回报，请检查启动日志")
    managed = os.environ.get("NUWAX_GEOMETRY_RUNTIME_MANAGED_BY_SYSTEMD", "").lower() in {"1", "true"}
    launcher = os.environ.get("NUWAX_SETTINGS_LAUNCHER", "")
    available = bool(launcher and Path(launcher).is_file())
    reason = ""
    if managed:
        try:
            available = subprocess.run(["systemctl", "is-active", "--quiet", "nuwax-settings-apply.path"],
                                       timeout=2, check=False).returncode == 0
        except (OSError, subprocess.TimeoutExpired):
            available = False
        if not available:
            reason = "请更新部署服务以启用设置应用入口；当前可以保存并稍后重启"
    elif not available:
        reason = "当前未由统一启动脚本管理；保存后请使用原启动方式重新启动"
    return {"restart": status, "restart_available": available, "restart_unavailable_reason": reason,
            "restart_scope": "网关及其依赖的运行组件" if managed else "网关及其管理的子进程"}


def queue_restart(root):
    with _target_lock(root / "runtime_data/settings/restart"):
        status = restart_status(root)
        if not status["restart_available"]:
            raise ValueError(status["restart_unavailable_reason"])
        if status["restart"].get("state") in {"queued", "restarting", "starting"}:
            raise ValueError("已有重启请求正在处理")
        request = {"request_id": uuid.uuid4().hex, "created_at": time.time(), "state": "queued",
                   "scope": "stack", "previous_process_id": os.getpid(), "message": "已安排重启，等待原启动拥有方执行"}
        _atomic_write(root / "runtime_data/settings/restart.json", json.dumps(request, ensure_ascii=False) + "\n")
        return request


def dispatch_restart(root, request):
    """响应发送后调用；本地辅助进程脱离网关，系统服务由专用路径单元接管。"""
    managed = os.environ.get("NUWAX_GEOMETRY_RUNTIME_MANAGED_BY_SYSTEMD", "").lower() in {"1", "true"}
    if managed:
        _atomic_write(root / "runtime_data/settings/restart.request", request["request_id"] + "\n")
        return
    environment = dict(os.environ)
    for key in environment.get("_NUWAX_CONFIG_FILE_KEYS", "").split(":"):
        environment.pop(key, None)
    for key in ("_NUWAX_CONFIG_FILE_KEYS", "NUWAX_GATEWAY_SUPERVISOR_TITLED"):
        environment.pop(key, None)
    path = root / "runtime_data/settings/restart.log"
    try:
        with path.open("ab") as output:
            subprocess.Popen([sys.executable, str(root / "scripts/apply_settings.py"), "--local",
                              os.environ["NUWAX_SETTINGS_LAUNCHER"]], cwd=str(root), env=environment,
                             stdin=subprocess.DEVNULL, stdout=output, stderr=subprocess.STDOUT,
                             start_new_session=True, close_fds=True)
    except Exception as error:
        _atomic_write(root / "runtime_data/settings/restart.json",
                      json.dumps(dict(request, state="failed", message=str(error)), ensure_ascii=False) + "\n")
