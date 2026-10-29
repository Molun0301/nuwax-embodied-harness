"""控制台只读机器人外观资产与关节姿态；不采集硬件、不发送控制。"""

from __future__ import annotations

from datetime import datetime, timezone
import math
from pathlib import Path
import threading
import time
from typing import TYPE_CHECKING

from fastapi import APIRouter, HTTPException
from fastapi.concurrency import run_in_threadpool
from fastapi.responses import JSONResponse, Response

from core.robot_geometry import RobotGeometryError, resolve_robot_geometry
from core.robot_kinematics import RobotKinematicsError
from core.robot_visual import RobotVisualAssetStore, RobotVisualError
from core.source_time import acquired_times, SourceTimeError
from gateways.serialization import to_jsonable

if TYPE_CHECKING:
    from gateways.runtime import GatewayRuntime

_BASE_DIR = Path(__file__).resolve().parents[2]
_MAX_AGE_SEC = 0.5
_MAX_SKEW_SEC = 0.25


def _timestamp(value):
    if isinstance(value, str):
        try:
            value = datetime.fromisoformat(value.replace("Z", "+00:00"))
        except ValueError:
            return None
    if not isinstance(value, datetime) or value.tzinfo is None:
        return None
    return value.astimezone(timezone.utc)


def _matrices(transforms):
    return {name: matrix.reshape(16, order="F").tolist() for name, matrix in transforms.items()}


def _acquired(stamp, metadata=None):
    if stamp is None:
        return None, None
    source_ns = int(stamp.timestamp() * 1_000_000_000)
    metadata = metadata or {}
    if metadata.get("joint_timestamp_basis") == "dds_receive_time":
        monotonic = metadata.get("joint_received_monotonic")
        return source_ns, (int(monotonic * 1_000_000_000)
                           if isinstance(monotonic, (int, float)) and math.isfinite(monotonic) else None)
    return acquired_times(source_ns)


def _sample_age(sample, now_unix_ns, now_monotonic_ns):
    unix, monotonic = sample
    if monotonic is not None:
        return (now_monotonic_ns - monotonic) / 1_000_000_000
    return (now_unix_ns - unix) / 1_000_000_000 if unix is not None else None


class RobotModelView:
    """转换和内容校验在工作线程中完成，缓存每个资产对应的导航外包。"""

    def __init__(self, runtime, *, store=None, base_dir=_BASE_DIR):
        self.runtime = runtime
        self.base_dir = Path(base_dir)
        self.store = store or RobotVisualAssetStore(base_dir=self.base_dir)
        self._lock = threading.RLock()
        self._geometry_key = None
        self._geometry = None

    def _load(self):
        robot = self.runtime.config.robot
        config = robot.geometry
        try:
            asset = self.store.get_asset(config)
            problem = None
        except (RobotVisualError, OSError, ValueError) as exc:
            asset = None
            problem = {"reason": getattr(exc, "code", "robot_model_unavailable"), "message": str(exc)}
        if asset is not None:
            # 与外观共用已审计、已哈希的资源快照，避免路径二次读取。
            geometry = asset.navigation_geometry
            if geometry is None:
                geometry = {"error": asset.navigation_geometry_error["message"]}
            return asset, None, geometry
        if config.model_path:
            try:
                # 显示网格缺失时，仍可使用经相同路径审计的完整碰撞几何。
                # 内容检查和失败重试由资产缓存限频，不能在这里永久缓存缺失结果。
                geometry = self.store.get_navigation_geometry(config)
            except (RobotVisualError, RobotGeometryError, OSError, ValueError) as exc:
                geometry = {"error": str(exc)}
            return None, problem, geometry
        key = (repr(config), robot.active_robot, problem["message"])
        with self._lock:
            if key != self._geometry_key:
                try:
                    self._geometry = resolve_robot_geometry(
                        config, robot_name=robot.active_robot, base_dir=self.base_dir,
                    ).to_runtime_dict()
                except (RobotGeometryError, OSError) as exc:
                    self._geometry = {"error": str(exc)}
                self._geometry_key = key
            geometry = self._geometry
        return asset, problem, geometry

    def manifest(self):
        asset, problem, geometry = self._load()
        if asset is None:
            return {"available": False, "asset_id": None, "status": "unavailable", **problem, "geometry": geometry}
        return {**asset.manifest, "available": True, "status": "available", "reason": None,
                "mesh_url": "/api/robot-model/%s/mesh" % asset.asset_id, "geometry": geometry}

    def mesh(self, asset_id):
        asset, _problem, _geometry = self._load()
        # URL 只识别当前资产哈希，从不把请求路径拼到文件系统。
        if asset is None or asset.asset_id != asset_id:
            raise HTTPException(status_code=404, detail="机器人模型资产已更换或不可用。")
        return asset.mesh_bytes

    def state(self):
        asset, problem, _geometry = self._load()
        if asset is None:
            return {"available": False, "asset_id": None, "mode": "unavailable", **problem,
                    "link_transforms": {}, "valid_for_sec": 0.0}
        snapshot = self.runtime.robot_state_service.get_latest_snapshot()
        state = getattr(snapshot, "robot_state", None)
        metadata = getattr(state, "metadata", {}) or {}
        # 设备聚合时钟不能续命底层关节；驱动提供独立采样时间时必须优先使用。
        source_stamp = _timestamp(metadata.get("joint_source_timestamp", getattr(state, "timestamp", None)))
        localization = self.runtime.localization_service.get_latest_snapshot()
        pose = getattr(localization, "current_pose", None)
        pose_stamp = _timestamp(getattr(pose, "timestamp", None))
        joint_clock_error = pose_clock_error = None
        try:
            joint_acquired = _acquired(source_stamp, metadata)
        except SourceTimeError as exc:
            joint_acquired, joint_clock_error = (None, None), str(exc)
        try:
            pose_acquired = _acquired(pose_stamp)
        except SourceTimeError as exc:
            pose_acquired, pose_clock_error = (None, None), str(exc)
        now_unix_ns, now_monotonic_ns = time.time_ns(), time.monotonic_ns()
        age = _sample_age(joint_acquired, now_unix_ns, now_monotonic_ns)
        pose_age = _sample_age(pose_acquired, now_unix_ns, now_monotonic_ns)
        index = 1 if joint_acquired[1] is not None and pose_acquired[1] is not None else 0
        skew = (abs(joint_acquired[index] - pose_acquired[index]) / 1_000_000_000
                if joint_acquired[index] is not None and pose_acquired[index] is not None else None)
        source_skew = abs((pose_stamp - source_stamp).total_seconds()) if pose_stamp and source_stamp else None
        response = {
            "available": True, "asset_id": asset.asset_id, "base_link": asset.model.base_link,
            "mode": "reference", "reason": None, "message": "参考姿态",
            "link_transforms": _matrices(asset.reference_transforms),
            "joint_source_timestamp": source_stamp.isoformat() if source_stamp else None,
            "joint_timestamp_basis": metadata.get("joint_timestamp_basis", "robot_state_source"),
            "joint_source_sequence": metadata.get("joint_source_sequence"),
            "joint_acquired_unix_ns": joint_acquired[0],
            "joint_age_sec": age, "max_age_sec": _MAX_AGE_SEC, "valid_for_sec": 0.0,
            "pose": to_jsonable(pose), "pose_source": getattr(localization, "source_name", None),
            "pose_timestamp": pose_stamp.isoformat() if pose_stamp else None,
            "pose_frame_id": getattr(pose, "frame_id", None), "pose_age_sec": pose_age,
            "pose_acquired_unix_ns": pose_acquired[0], "pose_joint_source_skew_sec": source_skew,
            "pose_joint_skew_sec": skew, "max_skew_sec": _MAX_SKEW_SEC,
            "sync_basis": ("joint_receive_time" if metadata.get("joint_timestamp_basis") == "dds_receive_time"
                           else "source_acquisition_time"),
            "sync_status": "joint_reference",
        }
        reason, message = None, None
        joints = getattr(state, "joints", []) or []
        values = {}
        if not joints and asset.model.actuated_joint_names:
            reason, message = "joint_state_missing", "缺少实时关节状态，显示参考姿态"
        else:
            for joint in joints:
                name, value = joint.name, joint.position_rad
                if name in values or isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                    reason, message = "joint_state_invalid", "关节状态含重复名称或非法角度，显示参考姿态"
                    break
                values[name] = value
        if reason is None and joint_clock_error:
            reason, message = "source_time_unavailable", joint_clock_error + "，显示参考姿态"
        if reason is None and (age is None or age < -0.1):
            reason, message = "joint_timestamp_invalid", "关节来源时间不可用，显示参考姿态"
        if reason is None and age > _MAX_AGE_SEC:
            reason, message = "joint_state_stale", "实时关节状态已过期，显示参考姿态"
        transforms = None
        if reason is None:
            try:
                transforms = asset.model.link_transforms(values)
            except RobotKinematicsError as exc:
                reason, message = exc.code, str(exc) + "，显示参考姿态"
        if reason is None:
            if pose_clock_error:
                response["sync_status"] = "pose_stale"
                reason, message = "source_time_unavailable", pose_clock_error + "，显示参考姿态"
            elif pose_stamp is None:
                response["sync_status"] = "pose_missing"
                reason, message = "pose_missing", "缺少定位位姿，显示参考姿态"
            elif pose_age < -0.1 or pose_age > _MAX_AGE_SEC:
                response["sync_status"] = "pose_stale"
                reason, message = "pose_stale", "定位位姿已过期，显示参考姿态"
            elif skew > _MAX_SKEW_SEC:
                response["sync_status"] = "time_skew"
                reason, message = "pose_joint_time_skew", "定位与关节采样时间不匹配，显示参考姿态"
        if reason is None:
            response.update(mode="live", message="实时关节姿态", sync_status="matched",
                            link_transforms=_matrices(transforms),
                            valid_for_sec=max(0.0, min(_MAX_AGE_SEC - age, _MAX_AGE_SEC - pose_age)))
        else:
            response.update(reason=reason, message=message)
        return response


def build_robot_model_router(runtime: GatewayRuntime) -> APIRouter:
    router = APIRouter()
    view = RobotModelView(runtime)

    @router.get("/api/robot-model")
    async def robot_model_manifest():
        return JSONResponse(await run_in_threadpool(view.manifest), headers={"Cache-Control": "no-store"})

    @router.get("/api/robot-model/state")
    async def robot_model_state():
        return JSONResponse(await run_in_threadpool(view.state), headers={"Cache-Control": "no-store"})

    @router.get("/api/robot-model/{asset_id}/mesh")
    async def robot_model_mesh(asset_id: str):
        content = await run_in_threadpool(view.mesh, asset_id)
        return Response(content, media_type="application/octet-stream", headers={
            "Content-Encoding": "gzip", "Cache-Control": "public,max-age=31536000,immutable",
            "ETag": '"%s"' % asset_id,
        })

    return router
