"""一步更新的持久衔接和切换门禁；业务停启仍由原事务拥有。"""
from __future__ import annotations

import os
import sys
import time

from . import MANAGER_REVISION
from .common import ManagerError, atomic_json
from .runtime import clean_environment


def begin_update(engine, *, local_build=False, retry=False, automatic=False):
    plan = engine.plan({"action": "update", "one_step": True, "local_build": local_build,
                        "retry": retry, "automatic": automatic}, local_request=True)
    if plan["blockers"]:
        raise ManagerError("plan_blocked", "更新计划存在阻塞条件。", 409, plan["blockers"])
    return engine.submit(plan["id"])


def adopt_manager(engine, job):
    source = job["_plan"].get("_source")
    if not source or source["revision"] == MANAGER_REVISION:
        job["_manager_adopted"] = True
        return
    from .self_update import apply_manager
    result = apply_manager(engine, job)
    target = (engine.manager_home / "manager/current").resolve()
    job["_manager_adopted"] = True
    job["_manager_result"] = result
    # 管理程序已独立提交；后续业务失败只恢复业务，不撤销管理程序。
    job["_journal"] = {}
    atomic_json(engine._job_path(job["id"]), job)
    atomic_json(engine.manager_home / "config/restart-manager.json",
                {"instance_id": engine.instance_id, "job_id": job["id"], "version": result["manager_version"], "adopted": True})
    environment = clean_environment()
    environment["PYTHONPATH"] = str(target)
    os.chdir(str(target))
    # 重新执行同一持久任务，让新管理程序真正拥有业务候选的构建和切换。
    os.execve(sys.executable, [sys.executable, "-m", "scripts.device_manager", "--home", str(engine.manager_home), "_worker", job["id"]], environment)


def wait_until_idle(engine, job):
    while True:
        engine._cancelled(job["id"])
        readiness = engine.runtime.update_readiness()
        window_open = True
        if job["_plan"].get("_request", {}).get("automatic"):
            from .updates import in_window
            window_open = in_window(engine.settings().get("maintenance_window", ""))
        if readiness["ready"] and window_open:
            return
        message = readiness.get("reason", "等待维护时间窗") if window_open else "等待维护时间窗"
        engine._progress(job, "waiting_idle", 74, "更新已准备，" + message)
        time.sleep(5)


def wait_prebuilt(engine, source, job):
    request = job["_plan"].get("_request", {})
    if request.get("local_build") or not source["repository"].startswith("https://github.com/"):
        return
    from scripts.neh_prebuilt import PrebuiltPending, PrebuiltUnavailable, fetch_index, platform_key, program_names
    if not platform_key() or not program_names(job["_plan"]["modules"]):
        return
    while True:
        engine._cancelled(job["id"])
        try:
            fetch_index(source["repository"], source["revision"])
            return
        except PrebuiltPending:
            engine._progress(job, "prebuilt_pending", 18, "云端编译中；默认等待，每15分钟重查。可取消任务后选择本机编译。")
        except PrebuiltUnavailable:
            # 网络或索引校验故障由共同构建器按程序回退本机编译。
            return
        deadline = time.monotonic() + 900
        while time.monotonic() < deadline:
            engine._cancelled(job["id"])
            time.sleep(1)
