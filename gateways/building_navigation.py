"""将楼层连接状态机接到现有平台导航、地图事务和唯一运动守卫。"""
from __future__ import annotations

import math
from datetime import datetime, timezone
import time
from typing import Any, Optional

from contracts.building_navigation import FloorPose
from contracts.geometry import Pose, Quaternion, Vector3
from contracts.geometry_runtime import DEFAULT_MAP_FRAME
from contracts.navigation import NavigationGoal, NavigationStatus
from contracts.runtime_views import LocalizationSessionStatus
from core.errors import GatewayError
from providers.facilities import BoundedDockingProvider


class GatewayBuildingPorts:
    """一个原平台任务持有的接线对象，不创建额外导航任务或速度写入端。"""

    def __init__(self, runtime: Any, context: Any, admission: Any, *, requested_by: Optional[str] = None) -> None:
        self.runtime = runtime
        self.context = context
        self.task_id = context.task_id
        self.admission = admission
        self.requested_by = requested_by
        self.last_goal_id = None
        self.last_zero = None
        self.state = None
        self._last_pose_save = 0.0

    def ensure_active(self) -> None:
        self.context.ensure_active()
        self.runtime._assert_motion_admission(self.admission, self.admission.capability_name)

    def remaining_sec(self) -> float:
        self.context.ensure_active()
        remaining = self.context.timeout_remaining_sec()
        return max(0.001, float(remaining if remaining is not None else 600.0))

    def active_map_name(self):
        return self.runtime.map_workspace_service.get_active_map_name()

    def progress(self, state) -> None:
        self.state = state
        self.context.update(stage="floor_" + state.stage, message=state.last_message or "正在执行楼层连接。",
                            payload={"building_navigation": state.model_dump(mode="json")})
        self.context.update_metadata({"building_request_id": state.request.request_id, "building_stage": state.stage})

    def _remember_pose(self, floor, pose=None):
        if self.state is None or time.monotonic()-self._last_pose_save<0.5:
            return
        if pose is None:
            snapshot=self.runtime.localization_service.get_latest_snapshot()
            pose=snapshot.current_pose if snapshot is not None else None
        if pose is None or pose.frame_id!="map" or (datetime.now(timezone.utc)-pose.timestamp).total_seconds()>0.5:
            return
        q=pose.orientation
        yaw=math.atan2(2.0*(q.w*q.z+q.x*q.y),1.0-2.0*(q.y*q.y+q.z*q.z))
        self.state.last_pose=FloorPose(x=pose.position.x,y=pose.position.y,yaw_rad=yaw)
        self.state.last_pose_floor_id=floor.floor_id
        self.state.last_pose_manifest_sha256=floor.asset_manifest_sha256
        self.runtime._get_building_navigation_service().journal.save(self.state)
        self._last_pose_save=time.monotonic()

    def navigate(self, floor, pose, goal_id, invariant=None):
        self.ensure_active()
        if invariant:
            invariant()
        descriptor=self.runtime.platform_geometry_runtime.describe_backend()
        if self.state is not None and self.state.attempt>0 and (descriptor.get("active_map_name")!=floor.map_name or descriptor.get("asset_manifest_sha256")!=floor.asset_manifest_sha256):
            if self.state.last_pose is None or self.state.last_pose_floor_id!=floor.floor_id or self.state.last_pose_manifest_sha256!=floor.asset_manifest_sha256:
                raise GatewayError("恢复层内移动前缺少与完整图绑定的最后位姿。",error_code="floor_resume_localization_required")
            if not self.activate_and_localize(floor,self.state.last_pose):
                raise GatewayError("恢复层内移动前尚未取得有效定位。",error_code="floor_resume_localization_required")
        goal = NavigationGoal(goal_id=goal_id, map_name=floor.map_name,
            target_pose=Pose(frame_id=DEFAULT_MAP_FRAME, position=Vector3(x=pose.x,y=pose.y,z=0.0), orientation=Quaternion(z=math.sin(pose.yaw_rad/2),w=math.cos(pose.yaw_rad/2))),
            tolerance_position_m=0.1,tolerance_yaw_rad=0.1,
            metadata={"requested_by":self.requested_by or "unknown", "parent_task_id":self.context.task_id,"floor_id":floor.floor_id})
        goal = self.runtime._goal_with_motion_admission(goal,self.admission)
        deadline = self.context.timeout_deadline_monotonic()
        provider=self.runtime.platform_geometry_runtime.get_geometry_runtime_provider()
        while True:
            self.ensure_active()
            if invariant:
                invariant()
            readiness=provider.get_navigation_readiness(deadline_monotonic=deadline) if provider is not None else {}
            if readiness.get("ready") is True and readiness.get("forbidden_zone_map_name") in {None,"",floor.map_name}:
                break
            self.context.update(stage="floor_"+self.state.stage,message="等待目标层导航输入就绪。",payload={"floor_id":floor.floor_id,"waiting_for":readiness.get("reason_code")})
            time.sleep(min(0.05,self.remaining_sec()))
        self._remember_pose(floor)
        self.last_goal_id=goal_id
        self.last_zero=None
        initial=self.runtime._run_with_motion_admission(self.admission,lambda:self.runtime.navigation_service.set_goal(goal,deadline_monotonic=deadline))
        generation=self.runtime._capture_task_generation_snapshot(navigation_context=initial,motion_admission=self.admission)
        self.runtime._record_task_generation_snapshot(self.context,generation)
        while True:
            self.ensure_active()
            if invariant:
                invariant()
            navigation=self.runtime.navigation_service.refresh_navigation(deadline_monotonic=deadline)
            self._remember_pose(floor,navigation.navigation_state.current_pose)
            current=self.runtime._capture_task_generation_snapshot(navigation_context=navigation,motion_admission=self.admission)
            changes=self.runtime._task_generation_rollovers(generation,current)
            if changes:
                self.runtime._handle_navigation_generation_rollover(self.context,goal_id=goal_id,expected=generation,current=current,rollovers=changes,deadline_monotonic=time.monotonic()+8.0)
            proof=self.runtime._navigation_terminal_proof(navigation,goal_id=goal_id,allowed_statuses={NavigationStatus.SUCCEEDED.value})
            if proof and navigation.goal_reached:
                self.last_zero={"zeroed":True,**proof}
                return {"reached":True,**self.last_zero}
            if navigation.navigation_state.status in {NavigationStatus.FAILED,NavigationStatus.CANCELLED}:
                raise GatewayError(navigation.navigation_state.message or "楼层连接导航失败。",error_code="floor_navigation_failed")
            self.context.update(stage="floor_"+self.state.stage,message=navigation.navigation_state.message or "层内导航执行中。",
                                payload={"goal_id":goal_id,"floor_id":floor.floor_id,"remaining_distance_m":navigation.navigation_state.remaining_distance_m})
            time.sleep(min(0.1,self.remaining_sec()))

    def stop_navigation(self,goal_id):
        exact=goal_id or self.last_goal_id
        if self.last_zero is not None and (not exact or self.last_zero.get("goal_id")==exact):
            return self.last_zero
        if exact:
            try:
                navigation=self.runtime.navigation_service.refresh_navigation(deadline_monotonic=time.monotonic()+8.0)
                proof=self.runtime._navigation_terminal_proof(navigation,goal_id=exact,allowed_statuses={"succeeded","failed","cancelled"})
                if not proof:
                    navigation=self.runtime._cancel_navigation_with_safety_barrier(self.context,goal_id=exact,poll_interval_sec=0.05,acknowledge_interrupt=False,parking_deadline_monotonic=time.monotonic()+8.0)
                    proof=self.runtime._navigation_terminal_proof(navigation,goal_id=exact,allowed_statuses={"succeeded","failed","cancelled"})
                if proof:
                    self.last_zero={"zeroed":True,**proof}
                    return self.last_zero
            except Exception:
                # 精确后端不能证明停车时，继续由同一运动守卫提供最终屏障。
                pass
        stopped=self.runtime.motion_guard.stop_all_motion("跨层任务停车对账",operation_id=self.runtime._build_runtime_request_id("floor_stop"))
        proof={"zeroed":bool(stopped.zero.zero_ack_id and stopped.zero.applied_monotonic_ns>0),
               "zero_ack_id":stopped.zero.zero_ack_id,"zero_applied_monotonic_ns":stopped.zero.applied_monotonic_ns,
               "motion_stop_generation":stopped.motion_stop_generation,"goal_id":exact}
        if not proof["zeroed"]:
            return proof
        self.last_zero=proof
        if not self.context.is_cancel_requested() and not self.context.is_timed_out():
            # 只接续本任务刚完成的原生停车代次；上层全局停车或另一原生停车仍终止旧请求。
            renewed=self.runtime._run_with_motion_admission(self.admission,lambda:self.runtime._admit_motion_request(self.admission.capability_name,request_id=self.admission.request_id,deadline_monotonic=self.context.timeout_deadline_monotonic()))
            if renewed.guard_motion_stop_generation!=stopped.motion_stop_generation or renewed.generation!=stopped.generation or renewed.boot_nonce!=bytes(stopped.boot_nonce):
                raise GatewayError("跨层停车后运动代次再次改变，不能继续旧任务。",error_code="motion_request_cancelled")
            self.admission=renewed
        return proof

    def stop_mapping(self):
        self.ensure_active()
        state=self.runtime.platform_geometry_runtime.get_mapping_session_context()
        if isinstance(state,dict) and state.get("active") is False and state.get("phase")=="STOPPED":
            return
        result=self.runtime.platform_geometry_runtime.stop_mapping(reason="设施运载前暂停旧层输入")
        if result.get("stopped") is not True or result.get("mapping_active") is not False:
            raise GatewayError("设施运载前未确认旧层建图停止。",error_code="floor_mapping_stop_failed")

    def activate_and_localize(self,floor,seed:FloorPose):
        self.ensure_active()
        self.runtime.map_workspace_service.activate_map(map_name=floor.map_name,create_if_missing=False,metadata={"source":"building_navigation","requested_by":self.requested_by or "unknown"})
        self.ensure_active()
        try:
            session,workspace=self.runtime.map_workspace_service.relocalize_active_map(map_name=floor.map_name,metadata={
                "requested_by":self.requested_by or "unknown", "business_request_id":self.runtime._build_runtime_request_id("floor_localize"),
                "logical_timeout_ms":int(min(30.0,self.remaining_sec())*1000),"planar_seed":[seed.x,seed.y,seed.yaw_rad]})
        except GatewayError as error:
            if error.error_code in {"relocalization_candidate_rejected","relocalization_failed","relocalization_deadline_exceeded"}:
                return False
            raise
        self.runtime._refresh_workspace_snapshots(refresh_localization=False)
        return bool(session.status==LocalizationSessionStatus.READY and workspace.active_map_name==floor.map_name
                    and workspace.map_asset is not None and workspace.map_asset.asset_manifest_sha256==floor.asset_manifest_sha256)

    def start_floor_mapping(self,floor,connection,request_id):
        self.ensure_active()
        catalog=self.runtime._get_building_catalog()
        with catalog.maps.map_activation_guard():
            existing=catalog.maps.get_map_asset(floor.map_name)
            if existing is not None and existing.metadata.get("floor_creation_request_id")!=request_id:
                raise GatewayError("新层地图名称已由其它任务使用，不能覆盖。",error_code="floor_new_map_collision")
            if existing is None:
                self.runtime.map_workspace_service.create_map(map_name=floor.map_name,display_name=floor.display_name,
                    metadata={"floor_creation_request_id":request_id,"floor_id":floor.floor_id})
            elif existing.finalized and existing.asset_manifest_sha256:
                restored=floor.model_copy(update={"asset_manifest_sha256":existing.asset_manifest_sha256})
                if not self.activate_and_localize(restored,connection.arrival_pose):
                    raise GatewayError("初始层图已有检查点，但尚未恢复定位。",error_code="floor_new_map_localization_required")
                return existing.asset_manifest_sha256
        self.runtime._handle_start_mapping({"map_name":floor.map_name},requested_by=self.requested_by)
        while True:
            self.ensure_active()
            snapshot=self.runtime.mapping_service.get_latest_snapshot()
            if snapshot is not None and snapshot.map_name==floor.map_name and snapshot.occupancy_grid is not None and any(v>=0 for v in snapshot.occupancy_grid.data):
                break
            time.sleep(min(0.1,self.remaining_sec()))
        bundle=self.runtime.map_workspace_service.finalize_active_map(map_name=floor.map_name,reason="跨层新图初始检查点",
            metadata={"floor_creation_request_id":request_id,"floor_id":floor.floor_id})
        if not bundle.manifest_sha256:
            raise GatewayError("新层初始地图没有保存成功。",error_code="floor_initial_checkpoint_failed")
        return bundle.manifest_sha256

    def dock_to_exit(self,connection,floor,invariant=None):
        provider=getattr(self.runtime.robot.providers,"bounded_docking_provider",None)
        if not isinstance(provider,BoundedDockingProvider) or not provider.is_available() or provider.calibration_ref!=connection.docking_calibration_ref:
            return False
        distance=math.hypot(connection.exit_pose.x-connection.arrival_pose.x,connection.exit_pose.y-connection.arrival_pose.y)
        if distance>connection.docking_max_distance_m or not connection.allow_docking_without_localization:
            return False
        self.ensure_active()
        session=self.runtime._run_with_motion_admission(self.admission,lambda:self.runtime.motion_guard.acquire_relative_session(self.admission.proof))
        try:
            result=provider.execute_docking(connection,guard_session=session,ensure_active=self.ensure_active,remaining_sec=self.remaining_sec,interlock=invariant)
        finally:
            zero=self.runtime.motion_guard.stop_and_fence(session)
            self.last_zero={"zeroed":bool(zero.zero_ack_id and zero.applied_monotonic_ns>0),"zero_ack_id":zero.zero_ack_id,"zero_applied_monotonic_ns":zero.applied_monotonic_ns}
        actual=float(result.get("actual_distance_m",math.inf))
        return bool(result.get("reached") is True and math.isfinite(actual) and 0.0<=actual<=connection.docking_max_distance_m and self.last_zero["zeroed"])

    def interrupt_barrier(self,proof,*,reconciled):
        if proof.get("zeroed") is True and (self.context.is_cancel_requested() or self.context.is_timed_out()):
            self.context.acknowledge_interrupt_barrier({**dict(proof),"building_request_id":self.state.request.request_id,"facility_reconciliation_required":not reconciled})
