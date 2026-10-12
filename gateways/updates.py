"""网页与后台共用的软件更新状态；默认只检查，自动安装需显式开启。"""

from __future__ import annotations

import asyncio
import ipaddress
import logging
import math
import os
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from urllib.parse import urlsplit

from fastapi import APIRouter, Request
from fastapi.concurrency import run_in_threadpool
from fastapi.responses import JSONResponse

from scripts import self_update as updater

LOG = logging.getLogger("nuwax_robot_bridge.updates")

# 与既有 Web 停止入口一致；stop_mapping 会保存地图，不属于停车豁免。
_MCP_MAINTENANCE_STOPS = frozenset({
    "cancel_navigation", "cancel_task", "stop_all_motion", "stop_exploration",
})
# 已核对的任务/导航只读合同；不按 get_ 前缀或风险等级猜测新工具语义。
_MCP_MAINTENANCE_READS = frozenset({
    "get_active_tasks", "get_task_status", "get_navigation_state",
})
_MCP_UPDATE_OWNER = "neh.update_maintenance_owner"


def run_mcp_operation(request, operation, *, tool_name=None):
    """在实际工作线程内成对准入/退出，HTTP 取消不能提前释放仍执行的操作。"""
    service = request.scope.get(_MCP_UPDATE_OWNER)
    if (service is None or tool_name in _MCP_MAINTENANCE_STOPS
            or tool_name in _MCP_MAINTENANCE_READS):
        return operation()
    if not service.begin_mutation():
        from gateways.errors import GatewayError
        raise GatewayError(
            "软件更新维护中，暂不接受新的操作；停止与取消入口仍可用",
            error_code="update_maintenance", http_status=503, jsonrpc_code=-32000,
        )
    try:
        return operation()
    finally:
        service.end_mutation()



class UpdateService:
    def __init__(self, runtime, root=None):
        self.runtime = runtime
        self.root = (Path(root) if root else Path(__file__).resolve().parents[1]).resolve()
        self.state_root = self.root
        self.check_mutex = threading.Lock()
        self.mutex = threading.RLock()
        self.drain = threading.Condition(self.mutex)
        self.mutators = 0
        self.freezing = False
        self.task = None
        try:
            self.running_revision = updater.git(self.root, "rev-parse", "HEAD")
        except RuntimeError:
            self.running_revision = "unknown"

    def transaction_status(self):
        return updater.state(self.root)

    def preferences(self):
        return updater.read_json(
            self.state_root / "runtime_data/updates/preferences.json",
            {"auto_check": True, "auto_install": False, "interval_minutes": 60},
        )

    def status(self):
        try:
            latest = updater.read_json(self.state_root / "runtime_data/updates/check.json")
            transaction = self.transaction_status()
            settings = self.preferences()
            check_error = updater.read_json(self.state_root / "runtime_data/updates/error.json")
        except (OSError, ValueError):
            return {
                "running_revision": self.running_revision,
                "available": False,
                "state": "error",
                "message": "更新状态文件无法读取，请检查文件权限或恢复备份",
            }
        capability = {}
        try:
            _, installed = updater.install_home(self.root)
            descriptor = installed.get("deployment", {})
            capability = {"mode": descriptor.get("lifecycle", {}).get("mode", "foreground"),
                          "robot_adapter": descriptor.get("robot_adapter"),
                          "modules": installed.get("modules", []),
                          "capabilities": {"arbitrary_latest_git_build": True}}
            reason = ""
        except (RuntimeError, OSError, ValueError) as error:
            reason = str(error)
        # 运行进程身份优先于已被更新器切换的磁盘HEAD。
        available = bool(latest.get("latest_revision") and latest["latest_revision"] != self.running_revision)
        return {
            **latest,
            "available": available,
            "running_revision": self.running_revision,
            "settings": settings,
            "check_error": check_error,
            "transaction": transaction,
            "deployment": capability or {"mode": "module", "capabilities": {"arbitrary_latest_git_build": True}},
            "install_available": not reason,
            "install_unavailable_reason": reason,
        }

    def check(self):
        with self.check_mutex:
            if self.transaction_status().get("state") in updater.BUSY:
                return self.status()
            try:
                updater.check(self.root)
            except (OSError, RuntimeError, ValueError) as error:
                updater.write_json(
                    self.state_root / "runtime_data/updates/error.json", {"message": str(error), "at": time.time()}
                )
                raise
            updater.write_json(self.state_root / "runtime_data/updates/error.json", {})
            return self.status()

    def configure(self, settings):
        if not isinstance(settings, dict) or set(settings) - {"auto_check", "auto_install", "interval_minutes"}:
            raise ValueError("无效的更新设置")
        with self.mutex:
            current = self.preferences()
            for key in ("auto_check", "auto_install"):
                if key in settings and type(settings[key]) is not bool:
                    raise ValueError("更新开关必须为布尔值")
            interval = settings.get("interval_minutes", current["interval_minutes"])
            if type(interval) is not int or not 15 <= interval <= 1440:
                raise ValueError("检查间隔必须为15—1440分钟")
            current.update(settings)
            if current.get("auto_install"):
                current["auto_check"] = True
            updater.write_json(self.state_root / "runtime_data/updates/preferences.json", current)
            return self.status()

    def require_idle(self):
        if self.runtime.task_manager.list_active_tasks():
            raise RuntimeError("有任务正在运行，请先完成或取消任务")
        health = self.runtime.get_health_summary()
        robot = health.get("latest_robot_snapshot")
        if hasattr(robot, "model_dump"):
            robot = robot.model_dump(mode="json")
        # 无真实设备的基础网关允许更新；已接入设备必须确认电机停用。
        if not robot:
            assembly = getattr(self.runtime, "robot", None)
            actuator_check = getattr(assembly, "has_motion_actuator", None)
            if callable(actuator_check) and actuator_check() is False:
                return
            raise RuntimeError("缺少机器人状态，无法确认安全停机")
        if not isinstance(robot, dict):
            raise RuntimeError("无法核对机器人安全状态")  # noqa: TRY004 - 公开安全准入错误
        timestamp = robot.get("timestamp")
        try:
            age = (
                time.time()
                - datetime.fromisoformat(str(timestamp).replace("Z", "+00:00")).astimezone(timezone.utc).timestamp()
            )
        except (ValueError, TypeError):
            raise RuntimeError("机器人状态缺少有效时间戳")
        if not math.isfinite(age) or not -1 <= age <= 5:
            raise RuntimeError("机器人状态已过期，拒绝更新")
        if (robot.get("safety") or {}).get("motors_enabled") is not False:
            raise RuntimeError("请先停稳并停用机器人电机，再执行软件更新")

    def begin_mutation(self):
        with self.mutex:
            try:
                blocked = self.transaction_status().get("state") in updater.BUSY | {"recovery_required"}
            except (OSError, ValueError):
                blocked = True
            if self.freezing or blocked:
                return False
            self.mutators += 1
            return True

    def end_mutation(self):
        with self.drain:
            self.mutators -= 1
            self.drain.notify_all()

    def install(self, expected):
        with self.drain:
            if self.freezing:
                raise RuntimeError("更新维护已开始")
            self.freezing = True
            try:
                if not self.drain.wait_for(lambda: self.mutators == 0, timeout=30):
                    raise RuntimeError("仍有请求执行中，请稍后重试")
                self.require_idle()
                gateway = getattr(getattr(self.runtime, "config", None), "gateway", None)
                probe_url = updater.listener_url(
                    getattr(gateway, "host", "127.0.0.1"), getattr(gateway, "port", 8766)
                )
                status = updater.queue(self.root, expected, probe_url=probe_url)
                updater.dispatch(self.root, status)
                self.freezing = False  # 排队完成后由持久事务状态决定维护模式
            except BaseException:
                self.freezing = False
                raise
            LOG.info(
                "软件更新已受理",
                extra={"event": "system.update.queued", "update_id": status["request_id"], "target_revision": expected},
            )
            return self.status()

    def _read_schedule(self):
        return self.preferences(), updater.read_json(self.state_root / "runtime_data/updates/check.json")

    async def run(self):
        next_attempt = 0.0
        while True:
            try:
                preferences, last = await run_in_threadpool(self._read_schedule)
                if (
                    preferences.get("auto_check")
                    and time.time() >= next_attempt
                    and time.time() - last.get("checked_at", 0) >= preferences.get("interval_minutes", 60) * 60
                ):
                    next_attempt = time.time() + preferences.get("interval_minutes", 60) * 60
                    snapshot = await run_in_threadpool(self.check)
                    tx = snapshot.get("transaction", {})
                    target = snapshot.get("latest_revision")
                    if (
                        preferences.get("auto_install")
                        and snapshot.get("available")
                        and tx.get("target_revision") != target
                    ):
                        await run_in_threadpool(self.install, target)
            except asyncio.CancelledError:
                raise
            except Exception:  # noqa: BLE001 - 后台检查失败不能结束周期调度
                LOG.warning("自动更新检查或安装暂未完成；可在设置页手动重试", extra={"event": "system.update.deferred"})
            await asyncio.sleep(30)

    def start(self):
        self.task = asyncio.create_task(self.run())

    async def close(self):
        if self.task:
            self.task.cancel()
            try:
                await self.task
            except asyncio.CancelledError:
                pass


def require_same_origin(request):
    # 无额外账号系统；防止其它网站用浏览器跨站触发代码部署。
    if request.headers.get("sec-fetch-site") == "cross-site":
        raise ValueError("拒绝跨站更新请求")
    host = request.url.hostname or ""
    allowed = {"localhost", *filter(None, os.environ.get("NUWAX_UPDATE_ALLOWED_HOSTS", "").split(","))}
    try:
        ipaddress.ip_address(host)
    except ValueError:
        if host not in allowed:
            raise ValueError("更新入口请使用网关IP地址；可信域名需由管理员配置 NUWAX_UPDATE_ALLOWED_HOSTS")
    origin = request.headers.get("origin")
    if origin and (urlsplit(origin).scheme, urlsplit(origin).netloc) != (
        request.url.scheme,
        request.headers.get("host"),
    ):
        raise ValueError("更新请求来源与网关地址不一致")
    if not request.headers.get("content-type", "").lower().startswith("application/json"):
        raise ValueError("更新操作需要 application/json")


def build_update_router(service):
    router = APIRouter()

    async def action(request, function, *args):
        try:
            require_same_origin(request)
            return await run_in_threadpool(function, *args)
        except (RuntimeError, ValueError, OSError) as error:
            return JSONResponse({"error": {"message": str(error)}}, status_code=409)

    @router.get("/api/updates")
    async def get_updates():
        return await run_in_threadpool(service.status)

    @router.get("/api/updates/readiness")
    async def update_readiness():
        try:
            await run_in_threadpool(service.require_idle)
            return {"ready": True}
        except (RuntimeError, ValueError, OSError) as error:
            return {"ready": False, "reason": str(error)}

    @router.post("/api/updates/check")
    async def check_updates(request: Request):
        return await action(request, service.check)

    @router.post("/api/updates/settings")
    async def update_settings(request: Request):
        try:
            payload = await request.json()
        except ValueError:
            return JSONResponse({"error": {"message": "无效JSON"}}, status_code=400)
        return await action(request, service.configure, payload)

    @router.post("/api/updates/install")
    async def install_update(request: Request):
        try:
            payload = await request.json()
            if not isinstance(payload, dict) or set(payload) != {"revision"}:
                raise ValueError()
        except ValueError:
            return JSONResponse({"error": {"message": "请指定已检查的revision"}}, status_code=400)
        return await action(request, service.install, payload["revision"])

    return router


class UpdateMaintenanceMiddleware:
    """部署进入事务后冻结其它写操作，防止停机期间新任务或设置写入。"""

    def __init__(self, app, service):
        self.app, self.service = app, service

    async def __call__(self, scope, receive, send):
        path = scope.get("path", "")
        emergency = (
            path == "/api/navigation/cancel"
            or (path.startswith("/api/tasks/") and path.endswith("/cancel"))
            or path
            in {
                "/api/capabilities/stop_all_motion/invoke",
                "/api/capabilities/cancel_task/invoke",
                "/api/capabilities/stop_exploration/invoke",
            }
        )
        # MCP 的 HTTP POST 同时承载查询和停车。由已解析、已校验的工具入口
        # 使用本中间件的唯一维护状态拥有方准入，不在这里重复读取/解析请求体。
        mcp_post = scope["type"] == "http" and path == "/mcp" and scope.get("method") == "POST"
        if mcp_post:
            scope[_MCP_UPDATE_OWNER] = self.service
        mutation = (
            scope["type"] == "http"
            and not mcp_post
            and scope.get("method") not in {"GET", "HEAD", "OPTIONS"}
            and not path.startswith("/api/updates")
            and not emergency
        )
        if mutation and not await run_in_threadpool(self.service.begin_mutation):
            response = JSONResponse(
                {"error": {"message": "软件更新维护中，暂不接受新的操作；停止与取消入口仍可用"}}, status_code=503
            )
            await response(scope, receive, send)
            return
        try:
            await self.app(scope, receive, send)
        finally:
            if mutation:
                await run_in_threadpool(self.service.end_mutation)
