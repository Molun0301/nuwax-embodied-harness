"""复用单个原生进程运行导航预设矩阵，保留每例请求、轨迹与失败证据。"""
from __future__ import annotations

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import queue
import re
import sys
import threading
import time

from navigation_simulator import NativeWorker, Run, default_binary, select_recording


def write_json(path: Path, value) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")


def exchange_until(worker: NativeWorker, request: dict, deadline: float) -> dict:
    """墙钟预算包含原生规划；取消后由调用方关闭进程，避免后台继续占用。"""
    channel: queue.Queue = queue.Queue(maxsize=1)

    def invoke():
        try:
            channel.put((True, worker.exchange(request)))
        except Exception as exc:
            channel.put((False, exc))

    thread = threading.Thread(target=invoke, daemon=True)
    thread.start()
    try:
        okay, value = channel.get(timeout=max(0.001, deadline - time.monotonic()))
    except queue.Empty:
        worker.interrupt()
        raise TimeoutError("单例墙钟预算耗尽") from None
    if not okay:
        raise value
    return value


def frame_metrics(frames: list[dict], start: list[float]) -> dict:
    longest = current = stationary = 0
    max_displacement = 0.0
    for frame in frames:
        velocity = frame.get("velocity", [0, 0, 0])
        stopped = math.hypot(*velocity[:2]) < 1e-4 and abs(velocity[2]) < 1e-4
        current = current + 1 if stopped else 0
        stationary += int(stopped)
        longest = max(longest, current)
        max_displacement = max(max_displacement, math.dist(start[:3], frame["pose"][:3]))
    return {"stationary_frames": stationary, "longest_stationary_sim_sec": longest * 0.05,
            "max_displacement_m": max_displacement}


def arrival_checks(result: dict, request: dict) -> dict:
    """从原请求、实际末姿态和末速独立核对到达，终态文字不能代替证据。"""
    try:
        pose, goal, velocity = result["pose"], request["goal"], result["velocity"]
        options, stopping = result["goal_options"], result["stopping_criteria"]
        values = [*pose, *goal, *velocity, options["position_tolerance_m"],
                  options["yaw_tolerance_rad"], stopping["linear_speed_below_mps"],
                  stopping["angular_speed_below_rps"]]
        if len(pose) != 4 or len(goal) != 4 or len(velocity) != 3 or not all(
                isinstance(value, (int, float)) and not isinstance(value, bool)
                and math.isfinite(value) for value in values):
            raise ValueError("末姿态、目标、速度或门槛无效")
        if options["position_tolerance_m"] <= 0 or options["yaw_tolerance_rad"] <= 0:
            raise ValueError("到达门槛必须为正")
        if stopping["linear_speed_below_mps"] <= 0 or stopping["angular_speed_below_rps"] <= 0:
            raise ValueError("停止门槛必须为正")
        if stopping["comparison"] != "strictly_less":
            raise ValueError("未知停止门槛比较方式")
        requires_orientation = bool(request.get("requires_orientation", False))
        if options["requires_orientation"] != requires_orientation:
            raise ValueError("请求与实际朝向要求不一致")
        position_error = math.dist(pose[:3], goal[:3])
        yaw_error = abs(math.atan2(math.sin(pose[3] - goal[3]), math.cos(pose[3] - goal[3])))
        linear_speed, angular_speed = math.hypot(*velocity[:2]), abs(velocity[2])
        return {"available": True, "position_error_m": position_error, "yaw_error_rad": yaw_error,
                "position_ok": position_error <= options["position_tolerance_m"],
                "orientation_ok": not requires_orientation or yaw_error <= options["yaw_tolerance_rad"],
                "stopped": linear_speed < stopping["linear_speed_below_mps"]
                and angular_speed < stopping["angular_speed_below_rps"],
                "linear_speed_mps": linear_speed, "angular_speed_rps": angular_speed}
    except (KeyError, TypeError, ValueError) as exc:
        return {"available": False, "reason": str(exc)}


def run_one(worker: NativeWorker, request: dict, directory: Path, timeout: float,
            sim_seconds: float) -> dict:
    directory.mkdir(parents=True, exist_ok=True)
    write_json(directory / "请求.json", request)
    log_path = worker.data / "原生模拟.log"
    log_start = log_path.stat().st_size
    run = Run(request, worker.cpu_seconds)
    deadline = time.monotonic() + timeout
    runner_failure = None
    next_progress = time.monotonic()
    try:
        run.accept(exchange_until(worker, {**request, "op": "start"}, deadline))
        while run.state["status"] in {"running", "planning"}:
            if run.state["sim_time_sec"] >= sim_seconds:
                runner_failure = "simulation_budget_exceeded"
                run.accept({"status": "failed", "message": "模拟时间预算耗尽", "command": [0, 0, 0]})
                break
            # 生产局部规划在后台线程执行；必须保持现有网页相同节拍，
            # 不能在规划尚未完成时凭空耗尽数百秒模拟时间。
            tick_began = time.monotonic()
            run.accept(exchange_until(worker, {"op": "step", "count": 1}, deadline))
            if time.monotonic() >= next_progress:
                write_json(directory / "最新状态.json", run.snapshot(after=max(0, len(run.frames) - 20)))
                next_progress = time.monotonic() + 5
            if run.state["status"] in {"running", "planning"}:
                time.sleep(max(0, min(0.05 - (time.monotonic() - tick_began), deadline - time.monotonic())))
    except TimeoutError as exc:
        runner_failure = "wall_budget_exceeded"
        worker.close()
        run.accept({"status": "failed", "message": str(exc), "command": [0, 0, 0]})
    except Exception as exc:
        runner_failure = "runner_error"
        run.accept({"status": "failed", "message": str(exc), "command": [0, 0, 0]})
    result = run.snapshot()
    result["metrics"].update(frame_metrics(result["frames"], request["start"]))
    expected = request.get("expected_reachable", True)
    checks = arrival_checks(result, request)
    reached = (result["status"] == "succeeded" and checks.get("available", False)
               and all(checks[key] for key in ("position_ok", "orientation_ok", "stopped")))
    safe_rejection = (result["status"] == "failed" and result["metrics"]["max_displacement_m"] <= 1e-9
                      and runner_failure is None and bool(result.get("message")))
    passed = ((reached if expected else safe_rejection) and runner_failure is None
              and result["metrics"].get("motion_validation_failures", 0) == 0)
    result.update(request=request, expected_reachable=expected, passed=passed, runner_failure=runner_failure,
                  arrival_checks=checks,
                  algorithm=worker.provenance,
                  clock_model="single_50ms_tick_with_realtime_pacing_for_async_local_planning",
                  simulation="生产三维导航与跟随；独立点云地面及速度响应；无实机连接")
    write_json(directory / "结果.json", result)
    write_json(directory / "最新状态.json", {key: value for key, value in result.items()
                                                if key not in {"frames", "path_history", "trajectory"}})
    with log_path.open("rb") as source, (directory / "原生诊断.log").open("wb") as target:
        source.seek(log_start)
        while chunk := source.read(1024 * 1024):
            target.write(chunk)
    summary = {key: result[key] for key in ("status", "sim_time_sec", "message", "expected_reachable", "passed", "runner_failure", "arrival_checks", "metrics")}
    summary.update(scene_id=request["scene_id"], case_id=request["case_id"], result_file=str(directory / "结果.json"))
    return summary


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, default=default_binary())
    parser.add_argument("--data-dir", type=Path,
                        default=Path("runtime_data/complex_ground_navigation") / datetime.now().strftime("%Y%m%d-%H%M%S"))
    parser.add_argument("--cloud", type=Path)
    parser.add_argument("--poses", type=Path)
    parser.add_argument("--synthetic-only", action="store_true")
    parser.add_argument("--include", default=".*", help="按场景/用例标识的正则表达式筛选")
    parser.add_argument("--request", type=Path, help="仅复跑保存的单个请求")
    parser.add_argument("--timeout", type=float, default=180, help="每例墙钟预算秒数")
    parser.add_argument("--sim-seconds", type=float, default=180, help="每例模拟时间预算秒数")
    parser.add_argument("--startup-timeout", type=float, default=180)
    parser.add_argument("--speed", type=float, default=1.0)
    args = parser.parse_args(argv)
    if not all(math.isfinite(value) and value > 0 for value in
               (args.timeout, args.sim_seconds, args.startup_timeout, args.speed)):
        parser.error("时间和速度必须为正有限数值")
    pattern = re.compile(args.include)
    data = args.data_dir.resolve()
    data.mkdir(parents=True, exist_ok=True)
    if (data / "矩阵请求.json").exists() or (data / "矩阵结果.json").exists():
        parser.error("输出目录已有矩阵证据，请选择新目录以保留上次请求与结果")
    cloud, poses, recording_name = select_recording(args.cloud, args.poses, args.synthetic_only)
    worker = None
    results = []
    began = time.monotonic()
    try:
        worker = NativeWorker(args.binary.resolve(), data, cloud, poses, args.startup_timeout)
        requests = []
        if args.request:
            request = json.loads(args.request.read_text(encoding="utf-8"))
            request.setdefault("case_id", "custom")
            requests.append(request)
        else:
            for scene in worker.catalog["scenes"]:
                cases = json.loads((data / (scene["id"] + "-cases.json")).read_text())["cases"]
                for case in cases:
                    if pattern.search(scene["id"] + "/" + case["id"]):
                        requests.append({**case, "case_id": case["id"], "scene_id": scene["id"], "speed_mps": args.speed})
        if not requests:
            raise ValueError("筛选后没有导航用例")
        write_json(data / "矩阵请求.json", requests)
        for index, request in enumerate(requests, 1):
            if worker.process.poll() is not None:
                worker = NativeWorker(args.binary.resolve(), data, cloud, poses, args.startup_timeout)
            directory = data / "cases" / request["scene_id"] / request["case_id"]
            summary = run_one(worker, request, directory, args.timeout, args.sim_seconds)
            results.append(summary)
            write_json(data / "矩阵结果.json", {"recording_name": recording_name,
                "elapsed_wall_sec": time.monotonic() - began, "cases": results,
                "passed": sum(item["passed"] for item in results), "total": len(requests)})
            print(json.dumps({"index": index, "total": len(requests), **summary}, ensure_ascii=False), flush=True)
        return 0 if all(item["passed"] for item in results) else 1
    except (OSError, ValueError, RuntimeError) as exc:
        print(str(exc), file=sys.stderr)
        return 1
    finally:
        if worker is not None and worker.process.poll() is None:
            worker.close()


if __name__ == "__main__":
    raise SystemExit(main())
