"""供设备管理程序查询业务运行时是否空闲。"""

from __future__ import annotations

from fastapi import APIRouter
from fastapi.concurrency import run_in_threadpool

from gateways.motion_guard import GatewayMotionGuardError


def require_idle(runtime):
    if runtime.task_manager.has_active_tasks():
        raise RuntimeError("有任务正在运行，请先完成或取消任务")
    recorder = runtime.mapping_recorder
    if recorder is not None and recorder.status()["recording"]:
        raise RuntimeError("正在录制建图，请先保存或停止建图")
    # 更新只重启本系统软件，要确认的是此刻没有在控制运动，与电机是否上电无关。
    # 所有运动都须持有运动守卫租约，撤销租约要等到零速确认才完成；本次运行未接入守卫则未发出过运动。
    guard = runtime.motion_guard
    if not guard.has_connected:
        return
    try:
        state = guard.guard_state()
    except GatewayMotionGuardError:
        raise RuntimeError("无法读取运动守卫状态，稍后重查")
    if state.active_lease_id:
        raise RuntimeError("运动控制仍在进行，请先停止运动")


def build_update_router(runtime):
    router = APIRouter()

    @router.get("/api/updates/readiness")
    async def update_readiness():
        try:
            await run_in_threadpool(require_idle, runtime)
            return {"ready": True}
        except (RuntimeError, ValueError, OSError) as error:
            return {"ready": False, "reason": str(error)}

    return router
