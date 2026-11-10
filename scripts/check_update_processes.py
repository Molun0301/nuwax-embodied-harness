"""安装命令停止真实进程的隔离检查。"""

from __future__ import annotations

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

from scripts.module_installer.lifecycle import deployment_descriptor, inspect_installation, stop_current
from core.config import ConfigStore


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
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.prefix = self.root / "installation/generations/test"
        self.prefix.mkdir(parents=True)
        self.pid_file = self.prefix.parent.parent / "config/runtime.json"
        self.lock_file = self.prefix.parent.parent / "run/business.lock"
        self.pid_file.parent.mkdir(parents=True)
        self.lock_file.parent.mkdir(parents=True)
        (self.prefix / "source").symlink_to(ROOT, target_is_directory=True)
        (self.prefix / "venv").symlink_to(sys.prefix, target_is_directory=True)
        store = ConfigStore(self.prefix.parent.parent)
        store.install("测试", {})
        (self.prefix / "manifest.json").write_text(json.dumps({
            "deployment": deployment_descriptor(self.root, self.prefix.parent.parent, self.prefix, ["core"], "customrobot", service_name="neh-fixture.service"),
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
        code = f"""import fcntl, json, os, pathlib, socket
lock = open({str(self.lock_file)!r}, 'a')
fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
# 通过本安装的监督者套接字声明拥有方，状态文件仅作为夹具就绪标志。
server = socket.socket(socket.AF_UNIX)
server.bind({str(self.lock_file.parent / 'supervisor.sock')!r})
server.listen()
pathlib.Path({str(self.pid_file)!r}).write_text(json.dumps({{"pid":os.getpid()}}))
while True:
    connection, _ = server.accept()
    with connection, connection.makefile('rb') as request:
        assert json.loads(request.readline()) == {{"op": "status"}}
        connection.sendall(json.dumps({{"pid": os.getpid()}}).encode() + b'\\n')
"""
        environment = dict(os.environ)
        environment["NUWAX_INSTALL_HOME"] = str(self.prefix.parent.parent if identity else self.root / "unrelated")
        process = subprocess.Popen(
            [sys.executable, "-c", code], env=environment, start_new_session=True,
            stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        self.processes.append(process)
        wait_until(self.pid_file.exists)
        return process

    def test_cli_stop_accepts_child_that_becomes_zombie(self):
        process = self.supervisor()
        self.assertEqual(stop_current(inspect_installation(self.prefix.parent.parent)), 0)
        # 先不回收子进程，确认停止操作能接受已退出、尚未回收的子进程。
        self.assertEqual(process_state(process.pid), "Z")
        process.wait(timeout=5)


if __name__ == "__main__":
    unittest.main(verbosity=2)
