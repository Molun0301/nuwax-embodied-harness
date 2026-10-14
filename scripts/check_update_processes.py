"""Real-process updater regressions; isolated temporary state, no robot or deployment."""

from __future__ import annotations

import ast
import asyncio
import fcntl
import inspect
import json
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from gateways.updates import UpdateMaintenanceMiddleware, UpdateService
from scripts import self_update as updater
from scripts.module_installer import lifecycle
from scripts.module_installer.cli import _stop
from scripts.module_installer.manager import InstallError


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


@unittest.skipUnless(sys.platform == "linux", "Process ownership checks use Linux /proc")
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

    def test_stop_live_owner_holding_gateway_lock_and_reap(self):
        process = self.supervisor()
        with self.lock_file.open("a") as stream:
            with self.assertRaises(BlockingIOError):
                fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
            updater.stop(self.root, self.prefix, process)
            self.assertIsNotNone(process.returncode)
            self.assertIsNone(process_state(process.pid))
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)

    def test_cli_stop_accepts_child_that_becomes_zombie(self):
        process = self.supervisor()
        self.assertEqual(_stop({"prefix": str(self.prefix)}, self.root), 0)
        # Do not poll/wait first: this verifies the stop operation itself accepts
        # an exited child that its caller has not yet had an opportunity to reap.
        self.assertEqual(process_state(process.pid), "Z")
        process.wait(timeout=5)

    def test_stop_reaps_already_exited_owned_child(self):
        process = self.supervisor()
        os.kill(process.pid, signal.SIGTERM)
        wait_until(lambda: process_state(process.pid) == "Z")
        updater.stop(self.root, self.prefix, process)
        self.assertIsNotNone(process.returncode)
        self.assertIsNone(process_state(process.pid))

    def test_stop_rejects_unrelated_live_process(self):
        process = self.supervisor(identity=False)
        with self.assertRaises(InstallError):
            updater.stop(self.root, self.prefix, process)
        self.assertIsNone(process.poll())

    def descriptor(self):
        return lifecycle.deployment_descriptor(
            self.root, self.prefix.parent.parent, self.prefix, ["core"], "customrobot", environment={}
        )

    def test_descriptor_stops_real_locked_foreground_owner(self):
        process = self.supervisor()
        with patch.object(lifecycle.os, "geteuid", return_value=1000):
            updater.stop(self.root, self.prefix, process, descriptor=self.descriptor())
        self.assertIsNone(process_state(process.pid))
        with self.lock_file.open("a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)

    def test_descriptor_reaps_already_exited_foreground_child(self):
        process = self.supervisor()
        os.kill(process.pid, signal.SIGTERM)
        wait_until(lambda: process_state(process.pid) == "Z")
        with patch.object(lifecycle.os, "geteuid", return_value=1000):
            updater.stop(self.root, self.prefix, process, descriptor=self.descriptor())
        self.assertIsNotNone(process.returncode)
        self.assertIsNone(process_state(process.pid))

    def test_descriptor_rejects_another_foreground_owner(self):
        process = self.supervisor(identity=False)
        with patch.object(lifecycle.os, "geteuid", return_value=1000), self.assertRaisesRegex(
            RuntimeError, "another installation"
        ):
            updater.stop(self.root, self.prefix, process, descriptor=self.descriptor())
        self.assertIsNone(process.poll())

    def test_timeout_cleans_actual_installer_detached_build_process(self):
        # Execute the actual inline installer program, replacing only the build
        # payload and hardware discovery. Its signal handler and the real
        # CommandRunner process-group cleanup are both exercised, not mocked.
        programs = [
            node.value for node in ast.walk(ast.parse(inspect.getsource(updater.execute)))
            if isinstance(node, ast.Constant) and isinstance(node.value, str)
            and "from scripts.module_installer.manager import" in node.value
            and "apply(request,hardware)" in node.value
        ]
        self.assertEqual(len(programs), 1, "Locate the production inline installer program")
        child_pid = self.root / "build.pid"
        child_code = f"""import os, pathlib, signal, time
signal.signal(signal.SIGTERM, signal.SIG_IGN)
pathlib.Path({str(child_pid)!r}).write_text(str(os.getpid()))
time.sleep(60)
"""
        wrapper = """import sys
from pathlib import Path
from types import SimpleNamespace
from scripts.module_installer import manager, hardware
manager.read_current = lambda home: {{'modules': ['core'], 'robot': 'customrobot', 'python': sys.executable}}
hardware.detect_hardware = lambda root: SimpleNamespace(build_jobs=1)
manager.apply = lambda request, detected: manager.CommandRunner(Path({log!r}), Path({root!r}))([sys.executable, '-c', {child!r}])
exec({program!r})
""".format(log=str(self.root / "build.log"), root=str(ROOT), child=child_code, program=programs[0])
        try:
            with patch.dict(
                os.environ, {"NEH_UPDATE_ROOT": str(ROOT), "NUWAX_UPDATE_INSTALL_DIR": str(self.root)}
            ), self.assertRaises(subprocess.TimeoutExpired):
                updater.command([sys.executable, "-c", wrapper], ROOT, self.root / "worker.log", timeout=2)
            self.assertTrue(child_pid.is_file(), (self.root / "worker.log").read_text())
            self.assertIsNone(process_state(int(child_pid.read_text())), "Detached build child survived timeout")
        finally:
            if child_pid.is_file():
                pid = int(child_pid.read_text())
                if process_state(pid) not in (None, "Z"):
                    try:
                        os.killpg(pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass


class AdmissionTests(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="neh-update-admission-")
        self.root = Path(self.temporary.name)
        self.runtime = SimpleNamespace()
        self.service = UpdateService(self.runtime, self.root)

    def tearDown(self):
        self.temporary.cleanup()

    async def call(self, app, path="/api/test/mutate", method="POST"):
        messages = []

        async def receive():
            return {"type": "http.request", "body": b"{}", "more_body": False}

        async def send(message):
            messages.append(message)

        await app({"type": "http", "method": method, "path": path}, receive, send)
        return messages

    @staticmethod
    async def response(scope, receive, send):
        await send({"type": "http.response.start", "status": 200, "headers": []})
        await send({"type": "http.response.body", "body": b"{}"})

    def queue(self, root, expected, *args, **kwargs):
        status = {"request_id": "test", "state": "queued", "target_revision": expected, "created_at": time.time()}
        updater.write_json(self.root / "runtime_data/updates/state.json", status)
        return status

    async def test_freeze_drains_existing_request_before_safety_check(self):
        entered, release = asyncio.Event(), asyncio.Event()
        order = []

        async def endpoint(scope, receive, send):
            if scope["path"] == "/api/test/mutate":
                entered.set()
                await release.wait()
                order.append("mutation-finished")
            await self.response(scope, receive, send)

        middleware = UpdateMaintenanceMiddleware(endpoint, self.service)
        active = asyncio.create_task(self.call(middleware))
        await asyncio.wait_for(entered.wait(), timeout=3)
        install = None
        try:
            with patch.object(self.service, "require_idle", side_effect=lambda: order.append("safety-check")), patch.object(
                updater, "queue", side_effect=self.queue
            ), patch.object(updater, "dispatch"):
                install = asyncio.get_running_loop().run_in_executor(None, self.service.install, "b" * 40)
                for _ in range(300):
                    if self.service.freezing:
                        break
                    await asyncio.sleep(0.01)
                self.assertTrue(self.service.freezing)
                self.assertEqual(order, [])
                blocked = await self.call(middleware, "/api/test/late")
                self.assertEqual(blocked[0]["status"], 503)
                emergency = await self.call(middleware, "/api/navigation/cancel")
                self.assertEqual(emergency[0]["status"], 200)
                release.set()
                await asyncio.wait_for(active, timeout=3)
                await asyncio.wait_for(install, timeout=3)
                self.assertEqual(order, ["mutation-finished", "safety-check"])
                self.assertEqual(self.service.mutators, 0)
        finally:
            release.set()
            await asyncio.gather(active, return_exceptions=True)
            if install is not None:
                await asyncio.gather(install, return_exceptions=True)

    async def test_recovery_required_blocks_mutation_but_allows_cancel(self):
        updater.write_json(self.root / "runtime_data/updates/state.json", {"state": "recovery_required"})
        middleware = UpdateMaintenanceMiddleware(self.response, self.service)
        self.assertEqual((await self.call(middleware))[0]["status"], 503)
        self.assertEqual((await self.call(middleware, "/api/tasks/example/cancel"))[0]["status"], 200)
        self.assertEqual((await self.call(middleware, "/api/health", "GET"))[0]["status"], 200)

    async def test_failed_before_stop_releases_transient_freeze(self):
        with patch.object(self.service, "require_idle"), patch.object(updater, "queue", side_effect=self.queue), patch.object(
            updater, "dispatch"
        ):
            self.service.install("b" * 40)
        updater.write_json(self.root / "runtime_data/updates/state.json", {"state": "failed"})
        middleware = UpdateMaintenanceMiddleware(self.response, self.service)
        self.assertEqual((await self.call(middleware))[0]["status"], 200)

    async def test_handler_exception_releases_inflight_mutator(self):
        async def failing(scope, receive, send):
            raise RuntimeError("simulated handler failure")

        middleware = UpdateMaintenanceMiddleware(failing, self.service)
        with self.assertRaisesRegex(RuntimeError, "simulated"):
            await self.call(middleware)
        self.assertEqual(self.service.mutators, 0)

    async def test_contended_admission_does_not_block_event_loop(self):
        locked, heartbeat = threading.Event(), threading.Event()
        observations = []

        def hold_check_lock():
            with self.service.mutex:
                locked.set()
                observations.append(heartbeat.wait(timeout=1.5))

        holder = threading.Thread(target=hold_check_lock)
        holder.start()
        self.assertTrue(locked.wait(timeout=3))
        middleware = UpdateMaintenanceMiddleware(self.response, self.service)
        mutation = asyncio.create_task(self.call(middleware))
        try:
            await asyncio.sleep(0.05)
            heartbeat.set()
            await asyncio.wait_for(mutation, timeout=3)
        finally:
            heartbeat.set()
            holder.join(timeout=3)
        self.assertEqual(observations, [True], "A contended update mutex blocked the event loop")


if __name__ == "__main__":
    unittest.main(verbosity=2)
