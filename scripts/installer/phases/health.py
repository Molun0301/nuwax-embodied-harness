"""health phase：探测 gateway ``/api/health``。

设计要点（openspec/changes/installer-redesign/specs/installer/spec.md
Requirement "health phase 区分 gateway 来源" + RK13/RK19）：

1. **禁止订阅 ROS 主题**：memory ``feedback_ros_bad_alloc_lightweight_sampling.md``
   记录实机 ROS2 大节点查询会触发 ``bad_alloc``；health 只走 ``curl -sS -m 5``
   HTTP 探测（5s timeout），不碰任何 ROS topic。
2. **PID 来源区分**：
   * 持有者 PID ∈ ``state.json`` 启动列表 → installer 自己起的 → 走 curl
   * 持有者 PID ∉ 启动列表 → 外部启动 → 跳过 curl，打印
     ``[health] gateway 已由外部启动（PID=<pid>），跳过健康检查``
3. **state.json 缺失**：视为 installer 未启动过 gateway → 直接走"外部启动"路径。

文件契约：

* ``runtime_data/.gateway.lock`` 由 ``scripts.installer._lock`` 写入持有者 PID
* ``runtime_data/installer_state.json`` 由 ``scripts.installer.state`` 维护
  ``phases[health].started_pids: list[int]`` 字段（写时记，启动时清）

实现形式：作为 ``Phase`` 子类暴露 ``run(planner, ctx)`` / ``dry_run(planner, ctx)``，
保留原 module-level ``run()`` 供向后兼容 / 单测调用。
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
from pathlib import Path
from typing import Any, List, Tuple

from scripts.installer._lock import LOCK_FILE, current_holder_pid
from scripts.installer.phases.base import Phase

STATE_FILE = Path("runtime_data/installer_state.json")
HEALTH_URL = "http://127.0.0.1:8766/api/health"
CURL_TIMEOUT_SEC = 5


def _read_state_pids() -> list[int]:
    """读 ``state.json`` 中 installer 启动过的 gateway PID 列表。

    缺失文件或解析失败 → 空列表（视作 installer 未启动过）。
    字段名兼容 ``health.started_pids`` 与旧版 ``health.launched_pid``。
    """
    if not STATE_FILE.exists():
        return []
    try:
        with open(STATE_FILE, "r", encoding="utf-8") as fp:
            data: dict[str, Any] = json.load(fp)
    except (OSError, json.JSONDecodeError):
        return []

    pids: list[int] = []
    health = data.get("phases", {}).get("health", {}) or {}
    for key in ("started_pids", "launched_pids"):
        for raw in health.get(key, []) or []:
            try:
                pids.append(int(raw))
            except (TypeError, ValueError):
                continue
    if "launched_pid" in health:
        try:
            pids.append(int(health["launched_pid"]))
        except (TypeError, ValueError):
            pass
    return pids


def _is_pid_alive(pid: int) -> bool:
    """PID 是否存活（不走 ps，直接 ``os.kill(pid, 0)``）。"""
    if pid <= 0:
        return False
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def _curl_health() -> tuple[int, str]:
    """执行 ``curl -sS -m 5`` 探测 health endpoint。

    返回 ``(exit_code, stdout_stderr)``；``exit_code==0`` 即 HTTP 200 响应。
    """
    if shutil.which("curl") is None:
        return 127, "curl not found in PATH"
    try:
        proc = subprocess.run(
            ["curl", "-sS", "-m", str(CURL_TIMEOUT_SEC), HEALTH_URL],
            check=False,
            capture_output=True,
            text=True,
            timeout=CURL_TIMEOUT_SEC + 1,
        )
    except subprocess.TimeoutExpired:
        return 124, f"curl timeout after {CURL_TIMEOUT_SEC}s"
    return proc.returncode, (proc.stdout or "") + (proc.stderr or "")


def run() -> dict[str, Any]:
    """health phase 主入口（module-level 兼容旧调用）。

    返回 ``{"status": "success"|"skipped"|"failed",
             "holder_pid": int|None,
             "message": str,
             "curl_exit": int|None}``
    供 ``_legacy_bridge`` / state writer 落盘。
    """
    holder_pid = current_holder_pid()
    installer_pids = _read_state_pids()

    # 锁文件无 PID 或读不出来 → 视为外部启动
    if holder_pid is None or not _is_pid_alive(holder_pid):
        return {
            "status": "skipped",
            "holder_pid": holder_pid,
            "message": (
                f"[health] gateway 已由外部启动（PID={holder_pid}），跳过健康检查"
                if holder_pid is not None
                else "[health] gateway 锁文件无持有者 PID，跳过健康检查"
            ),
            "curl_exit": None,
        }

    # holder 存活但不在 installer 启动列表 → 外部启动
    if holder_pid not in installer_pids:
        return {
            "status": "skipped",
            "holder_pid": holder_pid,
            "message": (
                f"[health] gateway 已由外部启动（PID={holder_pid}），"
                "跳过健康检查"
            ),
            "curl_exit": None,
        }

    # installer 自己起的 → 走 curl
    exit_code, output = _curl_health()
    if exit_code == 0:
        return {
            "status": "success",
            "holder_pid": holder_pid,
            "message": f"[health] /api/health HTTP 200: {output.strip()[:120]}",
            "curl_exit": exit_code,
        }
    return {
        "status": "failed",
        "holder_pid": holder_pid,
        "message": f"[health] curl exit {exit_code}: {output.strip()[:200]}",
        "curl_exit": exit_code,
    }


class HealthPhase(Phase):
    """阶段 11（最后一个）— gateway /api/health 探测。"""

    name = "health"
    order = 10
    requires: tuple = ()
    retryable = True
    dry_run_capable = True

    def run(self, planner, ctx) -> int:
        if not bool(getattr(planner, "healthcheck", False)):
            ctx.info(f"[phase: {self.name}] planner.healthcheck=False，跳过")
            return 0
        result = run()
        if result["status"] == "success":
            ctx.success(result["message"])
            return 0
        if result["status"] == "skipped":
            # 外部启动：不算失败
            ctx.info(result["message"])
            return 0
        # failed
        ctx.error(result["message"])
        return 1

    def dry_run(self, planner, ctx) -> List[str]:
        if not bool(getattr(planner, "healthcheck", False)):
            return ["[SKIP] planner.healthcheck=False"]
        return [
            f"+ curl -sS -m {CURL_TIMEOUT_SEC} {HEALTH_URL}（5s timeout，禁止订阅 ROS 主题）",
            "+ if PID ∈ state.json started_pids → installer 自启 → 走 curl",
            "+ else → 外部启动 → 跳过 + 打印 'gateway 已由外部启动'",
        ]


from scripts.installer.phases import default_registry  # noqa: E402

default_registry.register(name="health", cls=HealthPhase)


if __name__ == "__main__":  # pragma: no cover —— 仅供本地手动调试
    import sys

    result = run()
    print(json.dumps(result, ensure_ascii=False, indent=2))
    sys.exit(0 if result["status"] in ("success", "skipped") else 1)