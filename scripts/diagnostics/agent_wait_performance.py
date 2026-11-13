#!/usr/bin/env python3
"""当前智能体异步长轮询与任务读取、取消的离线延迟测量。

使用真实TaskRunner、AgentToolService和ASGI路由。仅临时内存任务和进程内HTTP，
不代表网络、实机吞吐或物理取消时延；每轮等待四个真实任务。
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


def fixture():
    from gateways.agent.router import build_agent_router
    from gateways.agent.service import AgentToolService
    from core.capability import Capability
    from gateways.capabilities.system import CAPABILITIES
    from scripts.diagnostics.current_fixtures import task_platform
    owner=task_platform();runtime=owner.__enter__()
    runtime.close=lambda:owner.__exit__(None,None,None)
    runtime.capabilities.register(Capability("wait_example","等待样例", "等待样例",{"type":"object"},"task",lambda p,c,a:c.wait(10)))
    for cap in CAPABILITIES:runtime.capabilities.register(cap)
    service=AgentToolService(runtime);app=FastAPI();app.include_router(build_agent_router(service))
    @app.get("/diagnostic/sync")
    def synchronous_probe():return {"ok":True,"entered_ns":time.perf_counter_ns()}
    return app,service,runtime


async def sample(client,service,runtime,limiter,probe,loaded,wait_seconds,wait_mode):
    ids=[service.invoke("wait_example",{})["task"]["task_id"] for _ in range(WORKERS)]
    waits=[]
    try:
        if loaded:
            waits=[asyncio.create_task(client.get(f"/api/agent/tasks/{identity}/wait",params={"timeout_sec":wait_seconds})) for identity in ids]
            deadline=time.monotonic()+wait_seconds+2
            while len(service._waiters)<WORKERS:
                if time.monotonic()>=deadline:raise RuntimeError("长轮询没有进入真实等待")
                await asyncio.sleep(.001)
        method,path=("GET","/diagnostic/sync") if probe=="normal_sync" else (("GET",f"/api/agent/tasks/{ids[0]}") if probe=="agent_read" else ("POST",f"/api/agent/tasks/{ids[0]}/cancel"))
        started=time.perf_counter_ns();response=await client.request(method,path);response.raise_for_status()
        return {"latency_ms":(time.perf_counter_ns()-started)/1e6,"borrowed_tokens":limiter.borrowed_tokens,"loaded":loaded,"response":response.json()}
    finally:
        for task in waits:task.cancel()
        await asyncio.gather(*waits,return_exceptions=True)
        for identity in ids:runtime.tasks.cancel(identity)
        from scripts.diagnostics.current_fixtures import finished
        for identity in ids:await asyncio.get_running_loop().run_in_executor(None,finished,runtime,identity)


def summarize(records):
    values=sorted(record["latency_ms"] for record in records)
    return {"sample_count":len(values),"median_ms":statistics.median(values),"maximum_ms":max(values)}


async def run(args):
    app,service,runtime=fixture();limiter=anyio.to_thread.current_default_thread_limiter();original_tokens=limiter.total_tokens;limiter.total_tokens=WORKERS
    results={}
    try:
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app),base_url="http://offline-assay.invalid") as client:
            for probe in ("normal_sync","agent_read","cancel"):
                for loaded in (False,True):
                    records=[await sample(client,service,runtime,limiter,probe,loaded,args.wait_seconds,"async") for _ in range(args.samples)]
                    results[f"{probe}_{'waiting' if loaded else 'no_load'}"]={"summary":summarize(records),"samples":records}
    finally:
        service.close();runtime.close();limiter.total_tokens=original_tokens
    return results,original_tokens,"async"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=7)
    parser.add_argument("--wait-seconds", type=float, default=0.4)
    parser.add_argument("--wait-mode", choices=("auto", "async"), default="auto")
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
                          "network": "in-process ASGI only"},
        "production_path":["build_agent_router","task_wait","AgentToolService.wait_task_async","TaskRunner","asyncio.Event.wait"],
        "synthetic_boundaries":["合成任务处理器","进程内ASGI传输"],
        "barrier":"四个长轮询已在当前AgentToolService登记；等待事件来自真实TaskRunner更新",
        "case_naming":"waiting表示四个真实任务正在长轮询，no_load表示没有长轮询负载",
        "limitations":["小样本仅描述本轮延迟，不代表生产服务指标","共享线程池固定四个令牌，结束后恢复原值",
            "没有硬件、实际网络、中间件、鉴权或物理停车测量","取消沿正式任务执行器完成收尾，任务处理器是可取消的合成等待"],
        "results": results,
    }
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({"output": str(args.output), "assay": report["assay"],
                      "summaries": {name: result["summary"] for name, result in results.items()}}, indent=2))


if __name__ == "__main__":
    main()
