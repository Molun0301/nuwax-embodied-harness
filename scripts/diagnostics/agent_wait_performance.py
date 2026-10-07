#!/usr/bin/env python3
"""智能体长轮询占满共享线程池的离线隔离测量。

使用生产智能体路由、AgentToolService.wait_task/wait_task_async/cancel_task 及快照和回执
投影。仅任务状态、权限检查、事件总线和调用存储为合成边界，不启动机器人、
SQLite、网络监听或外部接口，不代表实机性能、部署吞吐或安全取消时延。

在仓库根目录使用已有测试环境运行::

    .nuwax-test-env/bin/python scripts/diagnostics/agent_wait_performance.py \
        --samples 7 --wait-seconds 0.4 --output /tmp/neh-agent-wait-performance.json

屏障读取真实线程栈或协程等待链，不替换生产方法。同步路径确认四个生产
wait_task 均已进入 threading.Event.wait，并占满四个共享令牌；异步路径确认
四个生产 wait_task_async 均已订阅并在各自 asyncio.Event 上等待，且共享令牌
占用为零。两种实现均在确定进入等待状态后才发起同一组探针。
"""
from __future__ import annotations

import argparse
import asyncio
import importlib.metadata
import json
import os
import platform
import statistics
import subprocess
import sys
import threading
import time
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import anyio.to_thread
import httpx
from fastapi import FastAPI

WORKERS = 4
FIXTURE_ID = "agent-wait-four-long-polls-v1"
SOURCE_PATHS = ("gateways/agent/router.py", "gateways/agent/service.py",
                "scripts/diagnostics/agent_wait_performance.py")


def capture_source_observation(root, evidence_directory, phase):
    """保留本次逐文件读取的原始字节；不推断已加载模块或传递依赖的身份。"""
    sources = {path: (root / path).read_bytes() for path in SOURCE_PATHS}
    captured_utc = datetime.now(timezone.utc).isoformat()
    files = {}
    for relative, content in sources.items():
        evidence = evidence_directory / phase / relative
        evidence.parent.mkdir(parents=True, exist_ok=True)
        evidence.write_bytes(content)
        files[relative] = {"evidence_file": str(evidence.resolve()), "size_bytes": len(content)}
    return sources, {"captured_utc": captured_utc, "files": files}


class SyntheticBus:
    """线程安全的最小内存订阅边界，不启动消费者线程。"""
    def __init__(self):
        self.lock = threading.RLock()
        self.callbacks = {}
        self.next_id = 0

    def subscribe(self, callback, **_kwargs):
        with self.lock:
            self.next_id += 1
            self.callbacks[self.next_id] = callback
            return self.next_id

    def unsubscribe(self, subscription):
        with self.lock:
            self.callbacks.pop(subscription)

    def publish(self, task_id):
        with self.lock:
            callbacks = tuple(self.callbacks.values())
        event = SimpleNamespace(task_id=task_id)
        for callback in callbacks:
            callback(event)

    def count(self):
        with self.lock:
            return len(self.callbacks)


class SyntheticStore:
    def __init__(self):
        self.lock = threading.Lock()
        self.terminals = {}

    def find_task_call_identity(self, task_id):
        return None

    def save_terminal(self, task_id, snapshot):
        with self.lock:
            self.terminals[task_id] = deepcopy(snapshot)


class SyntheticRuntime:
    """实现生产快照及取消代码所用的运行时边界。"""
    def __init__(self):
        self._lock = threading.RLock()
        self.event_bus = SyntheticBus()
        self.task_manager = self
        self.capability_registry = SimpleNamespace(get_descriptor=lambda name: name)
        self.states = {}
        self.cancel_entered_ns = None

    def reset(self):
        with self._lock:
            self.states = {name: "running" for name in ["probe", *wait_ids()]}
            self.cancel_entered_ns = None

    def get_task_spec(self, task_id):
        self.states[task_id]
        return SimpleNamespace(capability_name="diagnostic.synthetic")

    def get_task_status(self, task_id):
        return {"task_id": task_id, "state": self.states[task_id], "stage": "working"}

    def get_task_events(self, task_id):
        return []

    def get_task_result(self, task_id):
        return None

    def get_task_error(self, task_id):
        return None

    def get_task_error_payload(self, task_id):
        return None

    def get_task_snapshot(self, task_id):
        return {"task_id": task_id,
                "task": {"capability_name": "diagnostic.synthetic", "metadata": {}},
                "status": self.get_task_status(task_id), "events": [],
                "result": None, "error": None, "error_payload": None}

    def _handle_cancel_task(self, arguments, *, requested_by):
        self.cancel_entered_ns = time.perf_counter_ns()
        self.finish(arguments["task_id"])

    def finish(self, task_id):
        with self._lock:
            self.states[task_id] = "cancelled"
        self.event_bus.publish(task_id)


def wait_ids():
    return [f"wait-{index}" for index in range(WORKERS)]


def fixture():
    # main 已在请求导入生产路由和服务前保存源文件；不强制重载已有模块。
    from gateways.agent.router import build_agent_router
    from gateways.agent.service import AgentToolService

    runtime = SyntheticRuntime()
    runtime.reset()
    # 跳过生产初始化的持久 SQLite 账本和长生命周期终态订阅；
    # 所测生产方法本身保持原样。
    service = AgentToolService.__new__(AgentToolService)
    service.runtime = runtime
    service.access_manager = SimpleNamespace(authorize_capability=lambda descriptor: None)
    service.store = SyntheticStore()
    service.gateway_instance_id = "offline-isolation-assay"
    service._subscription_id = None
    service._waiters_lock = threading.Lock()
    service._async_waiters = {}
    service._closed = False
    app = FastAPI()
    app.include_router(build_agent_router(service))

    @app.get("/diagnostic/sync")
    def synchronous_probe():
        return {"ok": True, "entered_ns": time.perf_counter_ns()}

    @app.get("/diagnostic/async")
    async def asynchronous_control():
        return {"ok": True}

    return app, service, runtime


def blocked_production_waits(service):
    """查找此服务 wait_task 栈下的真实 Event.wait 和 Condition.wait。"""
    found = {}
    for frame in sys._current_frames().values():
        saw_event = saw_condition = False
        while frame is not None:
            if frame.f_code is threading.Event.wait.__code__:
                saw_event = True
            if frame.f_code is threading.Condition.wait.__code__:
                saw_condition = True
            if frame.f_code is type(service).wait_task.__code__:
                if frame.f_locals.get("self") is service and saw_event and saw_condition:
                    found[frame.f_locals["task_id"]] = frame.f_locals["deadline"]
                break
            frame = frame.f_back
    return found


def coroutine_frames(awaitable):
    """只读遍历协程和生成器的当前 await 链，不驱动其执行。"""
    seen = set()
    while awaitable is not None and id(awaitable) not in seen:
        seen.add(id(awaitable))
        frame = getattr(awaitable, "cr_frame", None) or getattr(awaitable, "gi_frame", None)
        if frame is not None:
            yield frame
        awaitable = getattr(awaitable, "cr_await", None) or getattr(awaitable, "gi_yieldfrom", None)


def suspended_production_waits(service):
    """生产协程已挂起，且其独立 Event 的真实等待协程也已挂起才算就绪。"""
    method = getattr(type(service), "wait_task_async", None)
    if method is None:
        return {}
    event_waits = set()
    for task in asyncio.all_tasks():
        if not task.done():
            for frame in coroutine_frames(task.get_coro()):
                if frame.f_code is asyncio.Event.wait.__code__:
                    event_waits.add(id(frame.f_locals["self"]))
    found = {}
    with service.runtime.event_bus.lock:
        subscriptions = set(service.runtime.event_bus.callbacks)
    for task in asyncio.all_tasks():
        if task.done():
            continue
        for frame in coroutine_frames(task.get_coro()):
            if frame.f_code is not method.__code__ or frame.f_locals.get("self") is not service:
                continue
            changed = frame.f_locals.get("changed")
            changed = getattr(changed, "event", changed)
            if not isinstance(changed, asyncio.Event) or id(changed) not in event_waits:
                continue
            if changed.is_set() or not any(not future.done() for future in changed._waiters):
                continue
            if frame.f_locals.get("subscription") not in subscriptions:
                continue
            found[frame.f_locals["task_id"]] = frame.f_locals["deadline"]
    return found


async def saturation_barrier(service, limiter, tasks, wait_seconds, wait_mode):
    started = time.perf_counter()
    deadline = time.monotonic() + min(1.0, wait_seconds / 2)
    while time.monotonic() < deadline:
        if any(task.done() for task in tasks):
            raise RuntimeError("长轮询在建立饱和屏障前已结束")
        blocked = (blocked_production_waits(service) if wait_mode == "sync"
                   else suspended_production_waits(service))
        if set(blocked) == set(wait_ids()):
            remaining = min(blocked.values()) - time.monotonic()
            expected_tokens = WORKERS if wait_mode == "sync" else 0
            if limiter.borrowed_tokens != expected_tokens:
                raise RuntimeError(f"等待已建立，但共享线程池令牌占用不等于预期 {expected_tokens}")
            if service.runtime.event_bus.count() != WORKERS:
                raise RuntimeError("合成事件总线订阅数异常")
            if remaining < wait_seconds / 2:
                raise RuntimeError("等待剩余时间不足，无法取得有效样本")
            readiness_key = ("blocked_in_production_event_wait" if wait_mode == "sync"
                             else "suspended_in_production_async_event_wait")
            return {"wait_mode": wait_mode, readiness_key: len(blocked),
                    "shared_pool_borrowed_tokens": limiter.borrowed_tokens,
                    "subscriptions": service.runtime.event_bus.count(),
                    "barrier_ms": (time.perf_counter() - started) * 1000,
                    "minimum_remaining_wait_ms": remaining * 1000}
        await asyncio.sleep(0.0005)
    raise RuntimeError(f"建立四个真实生产 {wait_mode} 等待的屏障超时")


async def request_timed(client, method, path):
    started = time.perf_counter_ns()
    response = await asyncio.wait_for(client.request(method, path), timeout=2.0)
    elapsed_ms = (time.perf_counter_ns() - started) / 1e6
    response.raise_for_status()
    body = response.json()
    return elapsed_ms, started, body


async def sample(client, service, runtime, limiter, probe, loaded, wait_seconds, wait_mode):
    runtime.reset()
    initial_tasks = asyncio.all_tasks()
    pending = []
    barrier = None
    load_started_ns = None
    try:
        if loaded:
            load_started_ns = time.perf_counter_ns()
            pending = [asyncio.create_task(client.get(
                f"/api/agent/tasks/{task_id}/wait", params={"timeout_sec": wait_seconds}
            )) for task_id in wait_ids()]
            barrier = await saturation_barrier(service, limiter, pending, wait_seconds, wait_mode)
        control_ms, _, _ = await request_timed(client, "GET", "/diagnostic/async")
        if probe == "normal_sync":
            method, path = "GET", "/diagnostic/sync"
        elif probe == "agent_read":
            method, path = "GET", "/api/agent/tasks/probe"
        else:
            method, path = "POST", "/api/agent/tasks/wait-0/cancel"
        elapsed_ms, started_ns, body = await request_timed(client, method, path)
        record = {"probe_latency_ms": elapsed_ms, "async_control_latency_ms": control_ms,
                  "barrier": barrier}
        if probe == "normal_sync":
            record["handler_entry_delay_ms"] = (body["entered_ns"] - started_ns) / 1e6
        elif probe == "cancel":
            assert body["status"]["state"] == "cancelled", body
            assert runtime.cancel_entered_ns is not None
            record["handler_entry_delay_ms"] = (runtime.cancel_entered_ns - started_ns) / 1e6
        if pending:
            responses = await asyncio.wait_for(asyncio.gather(*pending), timeout=2.0)
            for response in responses:
                response.raise_for_status()
            wait_bodies = [response.json() for response in responses]
            timed_out = [body["wait_timed_out"] for body in wait_bodies]
            if probe != "cancel":
                assert all(timed_out), timed_out
            elif wait_mode == "async":
                assert not timed_out[0], "异步取消未及时唤醒目标等待"
                assert wait_bodies[0]["status"]["state"] == "cancelled", wait_bodies[0]
                assert all(timed_out[1:]), "取消不应提前结束其他任务的等待"
            record["wait_timed_out_count"] = sum(timed_out)
            record["wait_states"] = [body["status"]["state"] for body in wait_bodies]
            record["long_polls_completed_after_ms"] = (time.perf_counter_ns() - load_started_ns) / 1e6
            assert record["long_polls_completed_after_ms"] >= wait_seconds * 950, "长轮询未保持配置的等待时长"
        assert runtime.event_bus.count() == 0, "等待订阅未释放"
        assert limiter.borrowed_tokens == 0, "共享线程池令牌未释放"
        assert not service._async_waiters, "异步等待注册表未清理"
        await asyncio.sleep(0)
        remaining_tasks = [task for task in asyncio.all_tasks() - initial_tasks if not task.done()]
        assert not remaining_tasks, f"请求结束后仍有异步任务存活: {remaining_tasks}"
        record["remaining_subscriptions"] = runtime.event_bus.count()
        record["remaining_borrowed_tokens"] = limiter.borrowed_tokens
        record["remaining_async_waiters"] = len(service._async_waiters)
        record["remaining_request_tasks"] = len(remaining_tasks)
        return record
    finally:
        # 屏障、请求或断言失败时也唤醒真实等待；其自身不超过 0.6 秒
        # 的截止时间另行限制工作线程的阻塞时长。
        if any(not task.done() for task in pending):
            for task_id in wait_ids():
                runtime.finish(task_id)
            try:
                await asyncio.wait_for(asyncio.gather(*pending, return_exceptions=True), timeout=2)
            except asyncio.TimeoutError:
                for task in pending:
                    task.cancel()
                raise RuntimeError("长轮询清理超过限定截止时间")


def percentile95(values):
    ordered = sorted(values)
    position = (len(ordered) - 1) * 0.95
    low = int(position)
    high = min(low + 1, len(ordered) - 1)
    return ordered[low] + (ordered[high] - ordered[low]) * (position - low)


def summarize(records):
    summary = {"sample_count": len(records)}
    for field in ("probe_latency_ms", "async_control_latency_ms", "handler_entry_delay_ms"):
        values = [record[field] for record in records if field in record]
        if values:
            summary[field] = {"median": statistics.median(values), "p95": percentile95(values),
                              "min": min(values), "max": max(values)}
    return summary


async def run(args):
    app, service, runtime = fixture()
    limiter = anyio.to_thread.current_default_thread_limiter()
    original_tokens = limiter.total_tokens
    limiter.total_tokens = WORKERS
    wait_mode = args.wait_mode
    if wait_mode == "auto":
        wait_mode = "async" if hasattr(type(service), "wait_task_async") else "sync"
    results = {}
    try:
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app),
                                    base_url="http://offline-assay.invalid") as client:
            # 预热导入、AnyIO 工作线程和生产回执投影；
            # 不计入样本的负载探针先验证屏障可用。
            await sample(client, service, runtime, limiter, "normal_sync", True, args.wait_seconds, wait_mode)
            await sample(client, service, runtime, limiter, "agent_read", False, args.wait_seconds, wait_mode)
            await sample(client, service, runtime, limiter, "cancel", False, args.wait_seconds, wait_mode)
            for probe in ("normal_sync", "agent_read", "cancel"):
                for loaded in (False, True):
                    name = f"{probe}_{'saturated' if loaded else 'no_load'}"
                    records = [await sample(client, service, runtime, limiter, probe, loaded, args.wait_seconds, wait_mode)
                               for _ in range(args.samples)]
                    results[name] = {"summary": summarize(records), "samples": records}
    finally:
        limiter.total_tokens = original_tokens
    return results, original_tokens, wait_mode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=7)
    parser.add_argument("--wait-seconds", type=float, default=0.4)
    parser.add_argument("--wait-mode", choices=("auto", "sync", "async"), default="auto")
    parser.add_argument("--output", type=Path, default=Path("/tmp/neh-agent-wait-performance.json"))
    args = parser.parse_args()
    if not 5 <= args.samples <= 100:
        parser.error("--samples 必须为 5 至 100")
    if not 0.3 <= args.wait_seconds <= 0.6:
        parser.error("--wait-seconds 必须为 0.3 至 0.6")
    run_id = "agent-wait-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ") + f"-pid{os.getpid()}"
    evidence_directory = args.output.with_name(args.output.name + ".sources") / run_id
    evidence_directory.mkdir(parents=True, exist_ok=False)
    sources_before, observation_before = capture_source_observation(ROOT, evidence_directory, "before")
    git_commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    git_status = subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT, text=True)
    results, original_tokens, wait_mode = asyncio.run(run(args))
    sources_after, observation_after = capture_source_observation(ROOT, evidence_directory, "after")
    if sources_before != sources_after:
        raise RuntimeError(f"测量前后源文件字节不同，请重新测量；两次观察的文件副本保存在 {evidence_directory}")
    report = {
        "assay": "PRODUCTION_CODE_OFFLINE_THREADPOOL_ISOLATION",
        "fixture_id": FIXTURE_ID,
        "run_id": run_id,
        "real_robot_performance": "NOT_EVALUATED",
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "git_commit": git_commit,
        "git_observation_scope": "Commit and working-tree status observed before the run; version labels only",
        "git_dirty": bool(git_status),
        "git_status_porcelain": git_status,
        "source_observations": {"before": observation_before, "after": observation_after},
        "source_bytes_equal_at_observation_points": True,
        "source_observation_scope": "Direct byte equality for the listed files captured sequentially before and after the run only; does not establish continuous stability, loaded module identity or transitive dependency identity",
        "environment": {"python": platform.python_version(), "platform": platform.platform(),
                        "cpu_count": os.cpu_count(),
                        "cpu_affinity": sorted(os.sched_getaffinity(0)) if hasattr(os, "sched_getaffinity") else None,
                        **{name: importlib.metadata.version(name) for name in ("anyio", "fastapi", "starlette", "httpx")}},
        "configuration": {"samples_per_case": args.samples, "wait_seconds": args.wait_seconds,
                          "wait_mode": wait_mode,
                          "long_poll_concurrency": WORKERS, "shared_pool_tokens": WORKERS,
                          "original_shared_pool_tokens": original_tokens, "restored_original_limiter": True,
                          "percentile_method": "linear interpolation", "network": "in-process ASGI only"},
        "production_path": (["build_agent_router", "task_wait", "_reply", "run_in_threadpool",
                             "AgentToolService.wait_task", "threading.Event.wait"] if wait_mode == "sync"
                            else ["build_agent_router", "task_wait", "AgentToolService.wait_task_async",
                                  "asyncio.Event.wait"]),
        "synthetic_boundaries": ["task manager and snapshots", "capability authorization",
                                 "event bus", "call/terminal store", "cancel handler"],
        "barrier": ("Four worker stacks include production wait_task, threading.Event.wait and Condition.wait; shared limiter borrowed_tokens=4"
                    if wait_mode == "sync" else "Four production wait_task_async coroutine frames each have a registered subscription and a real pending asyncio.Event.wait; shared limiter borrowed_tokens=0"),
        "case_naming": "The saturated suffix means four active long-polls against a four-token pool; the async variant must leave the shared pool unsaturated",
        "limitations": ["Small-sample p95 is descriptive, not a production SLO estimate",
                        "Reduced four-token shared pool deliberately reproduces saturation; default pool has more tokens",
                        "No hardware, network, middleware, auth, SQLite or real task cancellation measured",
                        "Synthetic cancel changes task state immediately when its production route/service obtains a pool token"],
        "results": results,
    }
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({"output": str(args.output), "assay": report["assay"],
                      "summaries": {name: result["summary"] for name, result in results.items()}}, indent=2))


if __name__ == "__main__":
    main()
