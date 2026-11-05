"""供设备管理程序查询业务是否空闲。"""
from fastapi import APIRouter
from fastapi.concurrency import run_in_threadpool
from core import components
from services.geometry.state import SessionPhase
from services.navigation.motion_client import MotionError


def require_idle(platform):
    if platform.tasks.active():
        raise RuntimeError("有任务正在运行，请先完成或取消任务")
    state = platform.geometry_state.snapshot()
    if state is not None and state.phase in {SessionPhase.MAPPING, SessionPhase.EXPLORING}:
        raise RuntimeError("正在建图，请先保存或结束地图会话")
    if components.status().get("motion_guard", {}).get("state") == "running":
        try:
            status = platform.motion.status()
        except MotionError as error:
            raise RuntimeError(error.message) from error
        if status.lease_id:
            raise RuntimeError("运动控制仍在进行，请先停止运动")


def build_update_router(platform):
    router = APIRouter()

    @router.get("/api/updates/readiness")
    async def update_readiness():
        try:
            await run_in_threadpool(require_idle, platform)
            return {"ready": True}
        except (RuntimeError, ValueError, OSError) as error:
            return {"ready": False, "reason": str(error)}

    return router
