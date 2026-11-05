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
    """沿共同解析器读取一次配置，再物化原启动脚本的参数。"""
    raw = subprocess.check_output(
        ["bash", "-ec", 'source scripts/lib/config_env.sh; load_platform_config; env -0'], cwd=root,
        env=dict(os.environ, PATH=str(Path(sys.executable).parent) + ":" + os.environ["PATH"]))
    env = dict(item.decode().split("=", 1) for item in raw.split(b"\0") if item)
    defaults = {
        "NUWAX_INSTALL_HOME": env.get("NUWAX_MANAGED_HOME", str(root)),
        "PYTHONUNBUFFERED": "1", "NUWAX_LOG_DIR": str(root / "runtime_data/logs"), "NUWAX_LCM_URL": "udpm://239.255.76.67:7667?ttl=0",
        "NUWAX_GEOMETRY_DYNAMIC_BACKEND": "dufomap", "NUWAX_NAV_PCT_TERRAIN_SOURCE": "tomography",
        "NUWAX_GEOMETRY_FINE_CLOUD_CACHE_MIB": "auto", "NUWAX_GEOMETRY_FINE_CLOUD_CACHE_AUTO_MAX_MIB": "256",
        "NUWAX_GEOMETRY_RUNTIME_PORT": "50051", "NUWAX_GEOMETRY_PIPELINE": "lio_3d",
        "NUWAX_GEOMETRY_LIO_BACKEND": "device", "NUWAX_GEOMETRY_MAPPING_BACKEND": "voxel",
        "NUWAX_GEOMETRY_CORRECTION_BACKEND": "none",
        "NUWAX_GEOMETRY_SEQUENCE_STATE_PATH": str(root / "runtime_data/geometry/sequence_state.json"),
        "NUWAX_GEOMETRY_EXPLORATION_RECONCILIATION_JOURNAL_PATH": str(root / "runtime_data/geometry/exploration_ownership_journal.json"),
        "NUWAX_NAV_STACK_READY_FILE": str(root / "runtime_data/navigation/nuwax_nav_stack.ready"),
        "NUWAX_GEOMETRY_RUNTIME_READY_FILE": str(root / "runtime_data/geometry/nuwax_geometry_runtime.ready"),
        "NUWAX_PERSON_FOLLOW_READY_FILE": str(root / "runtime_data/person-follow/nuwax_person_follow.ready"),
        "NUWAX_PERSON_FOLLOW_TARGET_CHANNEL": "/person_follow_target",
        "NUWAX_PERSON_FOLLOW_POINTCLOUD_CHANNEL": "/global_map",
        "NUWAX_PERSON_FOLLOW_TF_CHANNEL": "/tf",
        "NUWAX_PERSON_FOLLOW_COMMAND_CHANNEL": "/gateway/person_follow_cmd_vel",
    }
    for key, value in defaults.items():
        if not env.get(key, "").strip():
            env[key] = value
    output = subprocess.check_output([sys.executable, str(root / "scripts/resolve_navigation_pipeline.py"), "--runtime-env"], env=env, text=True)
    env.update(dict(line.split("=", 1) for line in output.splitlines()))
    backend = env["NUWAX_GEOMETRY_LIO_BACKEND"]
    binary = env.get("NUWAX_GEOMETRY_RUNTIME_BINARY", str(root / "services/geometry/nuwax_geometry_runtime/target/release/nuwax-geometry-runtime"))
    if env.get("NUWAX_INSTALL_HOME") and env["NUWAX_GEOMETRY_PIPELINE"] == "lio_3d" and backend != "device" and not env.get("NUWAX_GEOMETRY_NATIVE_LIO_EXECUTABLE"):
        env["NUWAX_GEOMETRY_NATIVE_LIO_EXECUTABLE"] = str(Path(binary).parent / (backend + "_native"))
    env.setdefault("NUWAX_GEOMETRY_SLAM2D_BINARY", str(root / "services/nuwax-workspace/target/release/nuwax-slam-2d-worker"))
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


def component_table(root, env):
    python = env.get("NUWAX_PYTHON_BIN", sys.executable)
    native = root / "services/nuwax-workspace/target/release"
    truthy = lambda key, default="true": env.get(key, default).lower() in {"1", "true", "yes", "on"}
    nav_ready = Path(env["NUWAX_NAV_STACK_READY_FILE"] + ".runtime")
    geometry_ready = Path(env["NUWAX_GEOMETRY_RUNTIME_READY_FILE"] + ".runtime")
    follow_ready = Path(env["NUWAX_PERSON_FOLLOW_READY_FILE"] + ".runtime")
    return {
        "motion_guard": Component([python, "-m", "services.navigation.motion_safety_guard"],
            ready=lambda: socket_ready(env.get("NUWAX_MOTION_GUARD_SOCKET", "/run/nuwax/motion-guard.sock")),
            enabled=bool(env.get("NUWAX_MOTION_GUARD_BACKEND"))),
        "nav_stack": Component([env.get("NUWAX_NAV_STACK_BINARY", str(native / "nuwax-nav-stack")), "--profile=" + env["NUWAX_NAV_PROFILE"]],
            {"NUWAX_NAV_STACK_INTERNAL_READY_FILE": str(nav_ready)}, ("motion_guard",), nav_ready,
            enabled=truthy("NUWAX_NAV_AUTOSTART")),
        "geometry": Component([env.get("NUWAX_GEOMETRY_RUNTIME_BINARY", str(root / "services/geometry/nuwax_geometry_runtime/target/release/nuwax-geometry-runtime"))],
            {"NUWAX_GEOMETRY_RUNTIME_READY_FILE": str(geometry_ready)}, ready=geometry_ready,
            enabled=truthy("NUWAX_GEOMETRY_EXTERNAL_ENABLED")),
        "person_follow": Component([env.get("NUWAX_PERSON_FOLLOW_BINARY", str(native / "nuwax-person-follow"))],
            {"NUWAX_PERSON_FOLLOW_INTERNAL_READY_FILE": str(follow_ready)}, ("nav_stack", "geometry"), follow_ready,
            enabled=truthy("NUWAX_PERSON_FOLLOW_AUTOSTART")),
        "gateway": Component([python, "-m", "uvicorn", "gateways.app:create_gateway_app_from_env", "--factory",
            "--host", env.get("NUWAX_GATEWAY_HOST", "0.0.0.0"), "--port", env.get("NUWAX_GATEWAY_PORT", "8766"),
            "--ws-per-message-deflate", "false", "--timeout-graceful-shutdown", "5"],
            ready=lambda: gateway_ready(env.get("NUWAX_GATEWAY_PORT", "8766")), automatic=True),
        "video": Component([python, "-m", "drivers.go2.video_streamer"], automatic=True,
            enabled=env.get("NUWAX_ACTIVE_ROBOT", "").lower() == "go2" and truthy("GO2_VIDEO_STREAM_ENABLED")),
    }


class Supervisor:
    def __init__(self, root, environment, components=None):
        self.root, self.environment = Path(root), environment
        self.home = Path(environment.get("NUWAX_INSTALL_HOME", root))
        self.socket_path = self.home / "run/supervisor.sock"
        self.components = component_table(self.root, environment) if components is None else components
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
                if name in {"nav_stack", "geometry"} and not self.environment.get("NUWAX_ROBOT_GEOMETRY_JSON"):
                    self.environment["NUWAX_ROBOT_GEOMETRY_JSON"] = subprocess.check_output(
                        [sys.executable, str(self.root / "scripts/resolve_robot_geometry.py")],
                        env=self.environment, text=True).strip()
                directory = Path(self.environment.get("NUWAX_LOG_DIR", str(self.home / "logs")))
                directory.mkdir(parents=True, exist_ok=True)
                with (directory / (name + ".log")).open("ab") as log:
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
        return {name: {"state": item.state, "pid": item.process.pid if item.process else None,
            "exit_code": item.exit_code, "error": item.error} for name, item in self.components.items()}

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
    raise SystemExit(asyncio.run(main()))
