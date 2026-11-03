"""安装命令停止真实进程的隔离检查。"""

from __future__ import annotations

import fcntl
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scripts.module_installer.cli import _stop


def wait_until(predicate, timeout=5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("Timed out waiting for the isolated test process")


def process_state(pid):
    try:
        return (Path("/proc") / str(pid) / "stat").read_text().rsplit(")", 1)[1].split()[0]
    except FileNotFoundError:
        return None


@unittest.skipUnless(sys.platform == "linux", "进程拥有方检查使用 Linux 的 /proc")
class ProcessTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="neh-update-processes-")
        self.root = Path(self.temporary.name)
        self.prefix = self.root / "installation/generations/test"
        self.prefix.mkdir(parents=True)
        self.pid_file = self.root / "runtime_data/gateway/gateway.pid"
        self.lock_file = self.pid_file.parent / "start_gateway.lock"
        self.pid_file.parent.mkdir(parents=True)
        (self.prefix / "settings.env").write_text("NUWAX_GATEWAY_PID_FILE=" + str(self.pid_file) + "\n")
        (self.prefix / "manifest.json").write_text(json.dumps({
            "schema": 1, "complete": True, "modules": ["core"], "robot": "customrobot",
            "source_root": str(self.root),
        }))
        (self.prefix.parent.parent / "current").symlink_to("generations/test")
        self.processes = []

    def tearDown(self):
        for process in self.processes:
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            process.wait(timeout=5)
        self.temporary.cleanup()

    def supervisor(self, *, identity=True):
        code = f"""import fcntl, os, pathlib, time
lock = open({str(self.lock_file)!r}, 'a')
fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
pathlib.Path({str(self.pid_file)!r}).write_text(str(os.getpid()))
time.sleep(60)
"""
        environment = dict(os.environ)
        environment["NUWAX_BOOTSTRAP_ENV_FILE"] = str(
            self.prefix / ("settings.env" if identity else "unrelated.env")
        )
        process = subprocess.Popen(
            [sys.executable, "-c", code], env=environment, start_new_session=True,
            stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        self.processes.append(process)
        wait_until(self.pid_file.exists)
        return process

    def test_cli_stop_accepts_child_that_becomes_zombie(self):
        process = self.supervisor()
        self.assertEqual(_stop({"prefix": str(self.prefix)}, self.root), 0)
        # 先不回收子进程，确认停止操作能接受已退出、尚未回收的子进程。
        self.assertEqual(process_state(process.pid), "Z")
        process.wait(timeout=5)


if __name__ == "__main__":
    unittest.main(verbosity=2)
