"""业务组件的唯一进程监督者：按需启动、状态查询和依序关闭。"""
import asyncio
import ctypes
from dataclasses import dataclass, field
import fcntl
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import urllib.request


STOP_ORDER = ("gateway", "video", "person_follow", "geometry", "nav_stack", "motion_guard")
READY_TIMEOUT = 60
STOP_TIMEOUT = 15
RESTART_DELAY = 3
RESTART_MAX = 30
STABLE_SECONDS = 60
LIBC = ctypes.CDLL(None)


def parent_death_signal():
    LIBC.prctl(1, signal.SIGTERM)


def load_environment(root):
    """启动时合成完整配置，子进程仅继承系统和诊断环境。"""
    from core.config import ConfigStore, install_home
    home = install_home()
    config = ConfigStore(home, root).compose()
    system_names = {"HOME", "USER", "LOGNAME", "SHELL", "PATH", "LANG", "TZ", "TMPDIR", "TMP", "TEMP",
                    "LD_LIBRARY_PATH", "PYTHONPATH", "PYTHONNOUSERSITE", "PYTHONDONTWRITEBYTECODE", "VIRTUAL_ENV",
                    "SSL_CERT_FILE", "SSL_CERT_DIR", "LOGS_DIRECTORY", "INVOCATION_ID", "JOURNAL_STREAM",
                    "RUST_LOG", "NUWAX_SOURCE_TIME_FILE", "NUWAX_PGO_FULL_WINDOW_FILTER",
                    "NEH_PGO_PHASE_DIAGNOSTIC", "NEH_RAY_PHASE_DIAGNOSTIC", "NEH_SCAN_A_TRACE"}
    env = {key: value for key, value in os.environ.items()
           if key in system_names or key.startswith(("LC_", "CUDA_", "NVIDIA_", "OMP_", "OPENBLAS_", "MKL_",
                                                     "NUWAX_SURFACE_CAPTURE_", "NUWAX_PGO_WINDOW_", "NUWAX_TEST_"))}
    env["NUWAX_INSTALL_HOME"] = str(home)
    if config["robot"]["driver"] == "unitree_sim":
        from drivers.unitree_sim.clock import source_time_path
        env["NUWAX_SOURCE_TIME_FILE"] = str(source_time_path(home))
    env["PYTHONUNBUFFERED"] = "1"
    return env


def socket_ready(path):
    try:
        with socket.socket(socket.AF_UNIX) as connection:
            connection.settimeout(0.1)
            connection.connect(str(path))
        return True
    except OSError:
        return False


def gateway_ready(port):
    try:
        with urllib.request.build_opener(urllib.request.ProxyHandler({})).open(
                "http://127.0.0.1:%s/api/health" % port, timeout=0.2) as response:
            return response.status == 200
    except OSError:
        return False


@dataclass
class Component:
    command: list
    environment: dict = field(default_factory=dict)
    dependencies: tuple = ()
    ready: object = None
    automatic: bool = False
    enabled: bool = True
    process: object = None
    task: object = None
    lock: asyncio.Lock = field(default_factory=asyncio.Lock)
    state: str = "stopped"
    exit_code: object = None
    error: object = None
    started_at: float = 0
    log_file: object = None


def component_table(root, config):
    from core.config import binary_path, install_home, motion_guard_socket
    home = install_home()
    nav_ready = home / "runtime_data/navigation/nuwax_nav_stack.ready.runtime"
    geometry_ready = home / "runtime_data/geometry/nuwax_geometry_runtime.ready.runtime"
    follow_ready = home / "runtime_data/person-follow/nuwax_person_follow.ready.runtime"
    return {
        "motion_guard": Component([sys.executable, "-m", "services.navigation.motion_safety_guard"],
            ready=lambda: socket_ready(motion_guard_socket(home)), enabled=bool(config["motion_guard"]["backend"])),
        "nav_stack": Component([binary_path("nuwax-nav-stack"), "--profile=" + config["derived"]["nav_profile"]],
            dependencies=("motion_guard",), ready=nav_ready, enabled=config["navigation"]["autostart"]),
        "geometry": Component([binary_path("nuwax-geometry-runtime")], ready=geometry_ready,
            enabled=config["geometry"]["external_enabled"]),
        "person_follow": Component([binary_path("nuwax-person-follow")], dependencies=("nav_stack", "geometry"),
            ready=follow_ready, enabled=config["person_follow"]["autostart"]),
        "gateway": Component([sys.executable, "-m", "uvicorn", "gateways.app:create_gateway_app_from_env", "--factory",
            "--host", config["gateway"]["host"], "--port", str(config["gateway"]["port"]),
            "--ws-per-message-deflate", "false", "--timeout-graceful-shutdown", "5"],
            ready=lambda: gateway_ready(config["gateway"]["port"]), automatic=True),
        "video": Component([sys.executable, "-m", "drivers.go2.video_streamer"], automatic=True,
            enabled=config["robot"]["driver"] == "go2" and config["go2"]["video_stream_enabled"]),
    }


class Supervisor:
    def __init__(self, root, environment, components=None):
        self.root, self.environment = Path(root), environment
        self.home = Path(environment.get("NUWAX_INSTALL_HOME", root))
        self.socket_path = self.home / "run/supervisor.sock"
        from core.config import read_config
        self.components = component_table(self.root, read_config(self.home)) if components is None else components
        self.stopping = asyncio.Event()
        self.background = set()

    def background_task(self, coroutine):
        task = asyncio.create_task(coroutine)
        self.background.add(task)
        task.add_done_callback(self.background.discard)
        return task

    async def ensure(self, names):
        for name in names:
            item = self.components[name]
            if self.stopping.is_set() or not item.enabled:
                return {"ok": False, "component": name, "error": "业务正在停止" if self.stopping.is_set() else "组件未启用"}
            result = await self.ensure(item.dependencies)
            if not result["ok"]:
                return result
            async with item.lock:
                if item.task is None or item.task.done():
                    item.task = self.background_task(self.start_component(name, item))
                task = item.task
            result = await asyncio.shield(task)
            if not result["ok"]:
                return result
        return {"ok": True}

    async def start_component(self, name, item):
        try:
            if item.process is None or item.process.returncode is not None:
                item.state, item.exit_code, item.error = "starting", None, None
                if isinstance(item.ready, Path):
                    item.ready.parent.mkdir(parents=True, exist_ok=True)
                    item.ready.unlink(missing_ok=True)
                from core.config import value
                directory = Path(value("logging.dir"))
                directory.mkdir(parents=True, exist_ok=True)
                item.log_file = directory / (name + ".log")
                with item.log_file.open("ab") as log:
                    item.process = await asyncio.create_subprocess_exec(*item.command, cwd=self.root,
                        env=dict(self.environment, **item.environment), stdin=subprocess.DEVNULL,
                        stdout=log, stderr=log, start_new_session=True, preexec_fn=parent_death_signal)
                item.started_at = time.monotonic()
                self.background_task(self.watch(item, item.process))
            deadline = time.monotonic() + READY_TIMEOUT
            while item.process.returncode is None and not self.stopping.is_set():
                if item.ready is None or (item.ready.exists() if isinstance(item.ready, Path) else item.ready()):
                    item.state, item.error = "running", None
                    return {"ok": True}
                if time.monotonic() >= deadline:
                    raise TimeoutError("等待就绪超时（%s秒）" % READY_TIMEOUT)
                await asyncio.sleep(0.05)
            raise RuntimeError("业务正在停止" if self.stopping.is_set() else "进程退出，退出码 %s" % item.process.returncode)
        except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
            item.error = str(error)
            if item.process is None or item.process.returncode is not None:
                item.state = "exited"
            return {"ok": False, "component": name, "error": item.error}

    async def watch(self, item, process):
        code = await process.wait()
        if item.process is process:
            item.state, item.exit_code = "exited", code

    def status(self):
        return {"pid": os.getpid(), "components": {
            name: {"state": item.state, "pid": item.process.pid if item.process else None,
                   "exit_code": item.exit_code, "error": item.error,
                   "log_file": str(item.log_file) if item.log_file is not None else None}
            for name, item in self.components.items()}}

    async def business_component(self, name):
        delay = RESTART_DELAY
        item = self.components[name]
        while not self.stopping.is_set():
            await self.ensure([name])
            if item.process is not None:
                await item.process.wait()
                if time.monotonic() - item.started_at >= STABLE_SECONDS:
                    delay = RESTART_DELAY
            try:
                await asyncio.wait_for(self.stopping.wait(), delay)
            except asyncio.TimeoutError:
                pass
            delay = min(delay * 2, RESTART_MAX)

    async def handle(self, reader, writer):
        try:
            request = json.loads(await reader.readline())
            result = self.status() if request["op"] == "status" else await self.ensure(request["components"])
            writer.write((json.dumps(result, ensure_ascii=False) + "\n").encode())
            await writer.drain()
        except ConnectionError:
            pass
        finally:
            writer.close()
            try:
                await writer.wait_closed()
            except ConnectionError:
                pass

    async def stop(self):
        self.stopping.set()
        # 先结束在途创建，保证关闭顺序开始后不会再出现新子进程。
        await asyncio.gather(*(item.task for item in self.components.values() if item.task), return_exceptions=True)
        for name in STOP_ORDER:
            item = self.components.get(name)
            if item is None or item.process is None or item.process.returncode is not None:
                continue
            try:
                item.process.terminate()
                await asyncio.wait_for(item.process.wait(), STOP_TIMEOUT)
            except asyncio.TimeoutError:
                item.process.kill()
                await item.process.wait()
            except ProcessLookupError:
                pass
        await asyncio.gather(*self.background, return_exceptions=True)

    async def run(self):
        self.socket_path.parent.mkdir(parents=True, exist_ok=True)
        with (self.home / "run/business.lock").open("a") as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                print("业务已在运行", flush=True)
                return 1
            self.socket_path.unlink(missing_ok=True)
            server = await asyncio.start_unix_server(self.handle, path=self.socket_path)
            loop = asyncio.get_running_loop()
            for sig in (signal.SIGINT, signal.SIGTERM):
                loop.add_signal_handler(sig, self.stopping.set)
            try:
                for name, item in self.components.items():
                    if item.automatic and item.enabled:
                        self.background_task(self.business_component(name))
                await self.stopping.wait()
            finally:
                server.close()
                await server.wait_closed()
                await self.stop()
                self.socket_path.unlink(missing_ok=True)
        return 0


async def main():
    root = Path(__file__).resolve().parents[1]
    return await Supervisor(root, load_environment(root)).run()


if __name__ == "__main__":
    if not __package__:
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    raise SystemExit(asyncio.run(main()))
