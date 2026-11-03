#!/usr/bin/env python3
"""设置重启辅助入口：固定调用既有拥有方，不执行请求文件中的命令。"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
STATE = ROOT / "runtime_data/settings/restart.json"


def write_state(value):
    old = STATE.stat()
    descriptor, temporary = tempfile.mkstemp(dir=str(STATE.parent), prefix=".restart-")
    try:
        os.fchmod(descriptor, 0o600)
        os.fchown(descriptor, old.st_uid, old.st_gid)
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(value, handle, ensure_ascii=False)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, STATE)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--local", metavar="LAUNCHER", required=True)
    args = parser.parse_args(argv)
    request = json.loads(STATE.read_text(encoding="utf-8"))
    if request.get("state") != "queued" or request.get("scope") != "stack" or time.time() - request.get("created_at", 0) > 180:
        raise ValueError("重启请求不存在、失效或已处理")
    request.update(state="restarting", message="正在沿原生命周期入口停止并重新启动相关进程")
    write_state(request)
    try:
        # restart 会先关闭旧拥有方，随后成为新的长期运行拥有方。
        process = subprocess.Popen(["bash", args.local, "restart"], cwd=str(ROOT),
                                   stdin=subprocess.DEVNULL, start_new_session=True, close_fds=True)
        request.update(state="starting", launcher_pid=process.pid,
                       message="已启动重启入口，等待新网关回报")
        write_state(request)
    except Exception as error:
        write_state(dict(request, state="failed", message=str(error)))
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
