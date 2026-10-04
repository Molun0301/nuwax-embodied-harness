"""在开发机运行可视化导航模拟器，算法由本仓库原生规划和跟随实现提供。"""
from __future__ import annotations

import argparse
from collections import OrderedDict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import importlib.util
import json
import mimetypes
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from urllib.parse import parse_qs, unquote, urlparse
import uuid

ROOT = Path(__file__).resolve().parents[2]
STATIC = ROOT / "gateways/http/static"
DEFAULT_CLOUD = ROOT / "docs/开发文档/任务记录/2026-09-16_Point对照Fast实录修复与三维展示/三维对照/0918-1512-full.xyzt"
DEFAULT_POSES = ROOT / "docs/开发文档/任务记录/2026-09-18_本次实录纯Point与完整三维链路对照/实机原参数完整链路/轨迹.jsonl"


def worker_environment() -> dict[str, str]:
    """只解析模型，既不创建Go2驱动也不连接真实设备。"""
    name = "navigation_simulator_geometry"
    spec = importlib.util.spec_from_file_location(name, ROOT / "core/robot_geometry.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    geometry = module.resolve_robot_geometry(module.RobotGeometryConfig(), robot_name="go2", base_dir=ROOT)
    return {
        **os.environ,
        "NUWAX_ROBOT_GEOMETRY_JSON": geometry.to_runtime_json(),
        "NUWAX_NAV_BODY_TO_SUPPORT_Z_M": "0.4",
        "NUWAX_NAV_CAPABILITIES_JSON": "",
        "NUWAX_GEOMETRY_MAP_FRAME": "map",
        "_NUWAX_NAV_ACTUATOR_MOTION_MODEL": "omnidirectional",
        "_NUWAX_NAV_ACTUATOR_MAX_LINEAR_MPS": "1.0",
        "_NUWAX_NAV_ACTUATOR_MAX_YAW_RPS": "0.8",
    }


def binary_provenance(binary: Path, cloud: Path, poses: Path) -> dict:
    """只记录文件元数据，不读取二进制内容，也不为取得版本启动原生程序。"""
    observed = binary.stat()
    return {
        "binary": str(binary), "source_cloud": str(cloud), "source_poses": str(poses),
        "binary_stat": {"size_bytes": observed.st_size, "mtime_ns": observed.st_mtime_ns,
                        "ctime_ns": observed.st_ctime_ns, "dev": observed.st_dev, "inode": observed.st_ino},
        "binary_version": None,
        "binary_version_status": "not_reported",
        "provenance_scope": "Filesystem metadata observation only; not binary content verification. No native version probe was run",
    }


class NativeWorker:
    def __init__(self, binary: Path, data: Path, cloud: Path, poses: Path):
        self.data = data
        self.provenance = binary_provenance(binary, cloud, poses)
        self.lock = threading.Lock()
        self.error_log = (data / "原生模拟.log").open("a", encoding="utf-8")
        args = [str(binary), str(data)]
        if cloud.exists() and poses.exists():
            args += [str(cloud), str(poses)]
        self.process = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        stderr=self.error_log, text=True, encoding="utf-8",
                                        bufsize=1, env=worker_environment(), cwd=ROOT)
        line = self.process.stdout.readline()
        if not line:
            raise RuntimeError("原生模拟器未完成地图加载，请查看原生模拟.log")
        ready = json.loads(line)
        self.catalog = ready["catalog"]

    def exchange(self, request: dict) -> dict:
        with self.lock:
            if self.process.poll() is not None:
                raise RuntimeError("原生模拟器已退出")
            self.process.stdin.write(json.dumps(request, ensure_ascii=False, separators=(",", ":")) + "\n")
            self.process.stdin.flush()
            line = self.process.stdout.readline()
            if not line:
                raise RuntimeError("原生模拟器连接已关闭")
            value = json.loads(line)
            if "error" in value:
                raise ValueError(value["error"])
            return value

    def interrupt(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGUSR1)

    def cpu_seconds(self) -> float | None:
        try:
            fields = Path(f"/proc/{self.process.pid}/stat").read_text().split(") ", 1)[1].split()
            return (int(fields[11]) + int(fields[12])) / os.sysconf("SC_CLK_TCK")
        except (OSError, ValueError, IndexError):
            return None

    def close(self):
        self.interrupt()
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        self.error_log.close()


class Run:
    def __init__(self, request: dict, cpu_reader=None):
        self.id = uuid.uuid4().hex
        self.request = request
        self.created = time.monotonic()
        self.finished: float | None = None
        self.cpu_reader = cpu_reader
        self.cpu_start = cpu_reader() if cpu_reader else None
        self.cpu_final = None
        self.lock = threading.Lock()
        self.wake = threading.Event()
        self.cancelled = False
        self.paused = False
        self.frames: list[dict] = []
        self.path_history: dict = {}
        self.obstacles = list(request.get("obstacles") or [])
        self.obstacles_changed = False
        self.thread: threading.Thread | None = None
        self.state = {"id": self.id, "status": "planning", "sim_time_sec": 0,
                      "pose": request["start"], "goal": request["goal"], "command": [0, 0, 0],
                      "global_path": [], "local_path": [], "trajectory": [],
                      "metrics": {}, "message": "正在调用原生规划器", "path_revision": 0}

    def accept(self, state: dict):
        with self.lock:
            terminal = self.state["status"] in {"succeeded", "failed", "cancelled"}
            previous_status, previous_message = self.state["status"], self.state["message"]
            self.frames.extend(state.pop("frames", []))
            self.path_history.update(state.pop("path_history", {}))
            self.state.update(state)
            if terminal:
                self.state.update(status=previous_status, message=previous_message)
            elif self.cancelled:
                self.state.update(status="cancelled", command=[0, 0, 0], message="模拟已取消")
            elif self.paused and self.state["status"] not in {"succeeded", "failed"}:
                self.state["status"] = "paused"
            if self.state["status"] in {"succeeded", "failed", "cancelled"}:
                self.finished = self.finished or time.monotonic()
                if self.cpu_reader and self.cpu_final is None:
                    self.cpu_final = self.cpu_reader()

    def snapshot(self, after: int | None = None) -> dict:
        with self.lock:
            offset = max(0, min(len(self.frames), after or 0))
            elapsed = (self.finished or time.monotonic()) - self.created
            metrics = {**self.state.get("metrics", {}), "elapsed_wall_sec": elapsed}
            # 规划尚未返回或模拟已暂停时也读真实进程用量；终态冻结，后续任务不会改写历史。
            if self.cpu_reader:
                current_cpu = self.cpu_final if self.finished else self.cpu_reader()
                used_cpu = (max(0.0, current_cpu - self.cpu_start)
                            if current_cpu is not None and self.cpu_start is not None else None)
                metrics.update(native_cpu_seconds=used_cpu,
                               native_cpu_percent=(100.0 * used_cpu / max(0.001, elapsed)
                                                   if used_cpu is not None else None))
            return {**self.state, "id": self.id, "scene_id": self.request["scene_id"],
                    "metrics": metrics, "frame_offset": offset, "frames": self.frames[offset:],
                    "total_frames": len(self.frames), "path_history": dict(self.path_history),
                    "obstacles": list(self.obstacles),
                    "trajectory": [] if after is not None else [frame["pose"][:3] for frame in self.frames]}


class Simulator:
    def __init__(self, worker: NativeWorker, data: Path):
        self.worker = worker
        self.data = data
        self.runs: OrderedDict[str, Run] = OrderedDict()
        self.current: Run | None = None
        self.lock = threading.Lock()
        self.start_lock = threading.Lock()
        (data / "results").mkdir(exist_ok=True)

    def start(self, request: dict) -> Run:
        # 一个原生工作进程只归一轮模拟驱动；切换期间状态查询仍可独立读取。
        with self.start_lock:
            return self._start(request)

    def _start(self, request: dict) -> Run:
        scene_ids = {scene["id"] for scene in self.worker.catalog["scenes"]}
        if request.get("scene_id") not in scene_ids:
            raise ValueError("场景不存在")
        for key in ("start", "goal"):
            if not isinstance(request.get(key), list) or len(request[key]) != 4:
                raise ValueError("起点和目标需要机体XYZ以及方向角")
        with self.lock:
            previous = self.current
            if previous:
                previous.cancelled = True
                previous.wake.set()
                self.worker.interrupt()
        if previous and previous.thread and previous.thread.is_alive():
            previous.thread.join()
        run = Run(request, self.worker.cpu_seconds)
        with self.lock:
            self.current = run
            self.runs[run.id] = run
            while len(self.runs) > 8:
                self.runs.popitem(last=False)
        run.thread = threading.Thread(target=self._drive, args=(run,), name="导航模拟", daemon=True)
        run.thread.start()
        return run

    def _drive(self, run: Run):
        try:
            run.accept(self.worker.exchange({**run.request, "op": "start"}))
            while not run.cancelled:
                if run.obstacles_changed:
                    with run.lock:
                        obstacles = list(run.obstacles)
                        run.obstacles_changed = False
                    run.accept(self.worker.exchange({"op": "obstacles", "obstacles": obstacles}))
                if run.paused:
                    run.wake.wait(0.2)
                    run.wake.clear()
                    continue
                began = time.monotonic()
                result = self.worker.exchange({"op": "step", "count": 1})
                run.accept(result)
                if run.state["status"] in {"succeeded", "failed", "cancelled"}:
                    break
                run.wake.wait(max(0, 0.05 - (time.monotonic() - began)))
                run.wake.clear()
        except Exception as exc:
            run.accept({"status": "failed", "message": str(exc), "command": [0, 0, 0]})
        finally:
            if run.cancelled:
                run.accept({"status": "cancelled", "message": "模拟已取消", "command": [0, 0, 0]})
            result = {**run.snapshot(), "request": run.request,
                      "simulation": "本仓库生产三维规划/跟随；点云地面运动学与加减速响应；不连接实机", "algorithm": self.worker.provenance}
            (self.data / "results" / (run.id + ".json")).write_text(
                json.dumps(result, ensure_ascii=False, separators=(",", ":")), encoding="utf-8")

    def get(self, identifier: str) -> Run:
        with self.lock:
            if identifier not in self.runs:
                raise ValueError("模拟不存在或已从内存历史移出，可读取已导出的结果")
            return self.runs[identifier]

    def control(self, run: Run, payload: dict) -> dict:
        action = payload.get("action")
        with run.lock:
            if action == "pause":
                run.paused = True
                if run.state["status"] in {"planning", "running"}:
                    run.state["status"] = "paused"
            elif action == "resume":
                run.paused = False
                if run.state["status"] == "paused":
                    run.state["status"] = "running"
            elif action in {"cancel", "reset"}:
                run.cancelled = True
                if run.state["status"] not in {"succeeded", "failed", "cancelled"}:
                    run.state.update(status="cancelled", command=[0, 0, 0], message="模拟已重置" if action == "reset" else "模拟已取消")
                    run.finished = time.monotonic()
                with self.lock:
                    if self.current is run:
                        self.worker.interrupt()
            elif action in {"add_obstacle", "clear_obstacles"}:
                if action == "add_obstacle":
                    obstacle = payload.get("obstacle") or {}
                    for name in ("x", "y"):
                        float(obstacle[name])
                    run.obstacles.append(obstacle)
                else:
                    run.obstacles.clear()
                run.obstacles_changed = True
            else:
                raise ValueError("未知模拟操作")
        run.wake.set()
        return run.snapshot()

    def close(self):
        run = self.current
        if run:
            run.cancelled = True
            run.wake.set()
        self.worker.close()
        if run and run.thread:
            run.thread.join(timeout=3)


def serve(simulator: Simulator, host: str, port: int):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            if args and str(args[1:2]) != "('200',)":
                super().log_message(fmt, *args)

        def reply(self, body, status=200, content_type="application/json; charset=utf-8"):
            if not isinstance(body, bytes):
                body = json.dumps(body, ensure_ascii=False, separators=(",", ":"), allow_nan=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            parsed = urlparse(self.path)
            parts = unquote(parsed.path).strip("/").split("/")
            try:
                if parsed.path == "/":
                    self.send_response(302)
                    self.send_header("Location", "/static/nav-simulator/")
                    self.end_headers()
                elif parts == ["api", "scenes"]:
                    self.reply(simulator.worker.catalog)
                elif len(parts) == 4 and parts[:2] == ["api", "scenes"]:
                    identifier, kind = parts[2:]
                    if identifier not in {s["id"] for s in simulator.worker.catalog["scenes"]}:
                        raise ValueError("场景不存在")
                    if kind == "cloud":
                        self.reply((simulator.data / (identifier + ".bin")).read_bytes(), content_type="application/octet-stream")
                    elif kind == "cases":
                        self.reply(json.loads((simulator.data / (identifier + "-cases.json")).read_text()))
                    else:
                        self.reply({"error": "接口不存在"}, 404)
                elif len(parts) in {3, 4} and parts[:2] == ["api", "simulations"]:
                    if len(parts) == 4 and parts[3] == "result":
                        saved = simulator.data / "results" / (parts[2] + ".json")
                        if saved.exists():
                            self.reply(saved.read_bytes())
                        else:
                            run = simulator.get(parts[2])
                            self.reply({**run.snapshot(), "request": run.request})
                    else:
                        after = parse_qs(parsed.query).get("after_frame")
                        self.reply(simulator.get(parts[2]).snapshot(int(after[0]) if after else None))
                elif parts[0] == "static":
                    file = (STATIC / "/".join(parts[1:])).resolve()
                    if STATIC.resolve() not in file.parents:
                        raise ValueError("静态路径无效")
                    if file.is_dir():
                        file /= "index.html"
                    self.reply(file.read_bytes(), content_type=mimetypes.guess_type(file)[0] or "application/octet-stream")
                else:
                    self.reply({"error": "页面不存在"}, 404)
            except (ValueError, KeyError, FileNotFoundError) as exc:
                self.reply({"error": str(exc)}, 400)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_POST(self):
            parts = urlparse(self.path).path.strip("/").split("/")
            try:
                payload = json.loads(self.rfile.read(int(self.headers.get("Content-Length", "0"))))
                if parts == ["api", "simulations"]:
                    run = simulator.start(payload)
                    self.reply({"id": run.id, "state": run.snapshot()})
                elif len(parts) == 4 and parts[:2] == ["api", "simulations"] and parts[3] == "control":
                    self.reply(simulator.control(simulator.get(parts[2]), payload))
                elif len(parts) == 4 and parts[:2] == ["api", "scenes"] and parts[3] == "pose":
                    self.reply(simulator.worker.exchange({**payload, "op": "pose", "scene_id": parts[2]}))
                else:
                    self.reply({"error": "接口不存在"}, 404)
            except (ValueError, KeyError, RuntimeError) as exc:
                self.reply({"error": str(exc)}, 400)
            except (BrokenPipeError, ConnectionResetError):
                pass

    server = ThreadingHTTPServer((host, port), Handler)
    server.daemon_threads = True
    print(f"导航模拟器：http://{host}:{port}/", flush=True)
    try:
        server.serve_forever(poll_interval=0.2)
    finally:
        simulator.close()
        server.server_close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=18898, type=int)
    parser.add_argument("--binary", type=Path, default=ROOT / ".nuwax-test-env/target/release/nuwax-navigation-simulator")
    parser.add_argument("--data-dir", type=Path, default=ROOT / "runtime_data/navigation_simulator")
    parser.add_argument("--cloud", type=Path, default=DEFAULT_CLOUD)
    parser.add_argument("--poses", type=Path, default=DEFAULT_POSES)
    args = parser.parse_args()
    args.data_dir.mkdir(parents=True, exist_ok=True)
    worker = NativeWorker(args.binary.resolve(), args.data_dir.resolve(), args.cloud.resolve(), args.poses.resolve())
    try:
        serve(Simulator(worker, args.data_dir.resolve()), args.host, args.port)
    except KeyboardInterrupt:
        pass
    finally:
        worker.close()


if __name__ == "__main__":
    main()
