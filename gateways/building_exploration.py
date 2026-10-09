"""逐层探索的网关子任务接线，保留父请求停车代次与精确子任务关联。"""
from __future__ import annotations

import time
import uuid
from types import SimpleNamespace

from contracts.building_navigation import FloorPose
from contracts.navigation import ExploreAreaRequest
from core.errors import GatewayError
from skills.handlers.motion_skills import MOTION_ADMISSION_ARGUMENT


class GatewayBuildingExplorationPorts:
    def __init__(self,runtime,context,admission,*,requested_by=None):
        self.runtime=runtime;self.context=context;self.admission=admission;self.requested_by=requested_by
        self.task_id=context.task_id;self.state=None

    def ensure_active(self):
        self.context.ensure_active();self.runtime._assert_motion_admission(self.admission,self.admission.capability_name)

    def progress(self,state):
        self.state=state
        self.context.update(stage="building_"+state.stage,progress=state.next_floor_index/len(state.request.floors),
            message=state.message or "按冻结顺序执行逐层探索。",payload={"building_exploration":state.model_dump(mode="json")})

    def active_map_name(self):return self.runtime.map_workspace_service.get_active_map_name()

    def _admit_child(self,capability,business_id):
        self.ensure_active()
        admission=self.runtime._admit_motion_request(capability,request_id=business_id,deadline_monotonic=self.context.timeout_deadline_monotonic())
        self.ensure_active()
        return admission

    def submit_transfer(self,floor,business_id,*,resume=False):
        service=self.runtime._get_building_navigation_service()
        previous=service.journal.load()
        restoring=bool(resume and previous is not None and not previous.terminal and previous.request.request_id==business_id)
        if restoring:
            request=previous.request
        else:
            if resume:business_id="building-transfer-"+uuid.uuid4().hex
            topology=service.catalog.get(self.state.request.building_id)
            from services.navigation.building_catalog import route_connections
            current=next((f for f in topology.floors if f.map_name==self.active_map_name()),None)
            if current is None:raise GatewayError("当前地图未绑定到逐层探索建筑。")
            route=route_connections(topology,current.floor_id,floor.floor_id,service.supported_connections(topology.building_id))
            pose=next((c.exit_pose for c in topology.connections if route and c.connection_id==route[-1]),FloorPose(x=0,y=0))
            request=service.catalog.freeze_navigation(request_id=business_id,building_id=topology.building_id,target_floor_id=floor.floor_id,target_pose=pose,
                supported_connections=service.supported_connections(topology.building_id),timeout_sec=min(600,self.context.timeout_remaining_sec() or 600),allow_unmapped_target=True)
        capability="resume_floor_navigation" if restoring else "navigate_to_floor"
        admission=self._admit_child(capability,business_id)
        task=self.runtime._submit_floor_navigation({MOTION_ADMISSION_ARGUMENT:admission},request,restoring,self.requested_by,{"parent_task_id":self.task_id})
        self.ensure_active()
        return task.task_id,request.request_id

    def prepare_floor(self,floor):
        self.ensure_active()
        if floor.asset_manifest_sha256 is not None:
            descriptor=self.runtime.platform_geometry_runtime.describe_backend()
            session=self.runtime.map_workspace_service.get_active_localization_session()
            if descriptor.get("active_map_name")!=floor.map_name or session is None or str(session.status.value)!="ready":
                self.runtime.map_workspace_service.activate_map(map_name=floor.map_name,create_if_missing=False,metadata={"source":"building_exploration"})
                self.runtime.map_workspace_service.relocalize_active_map(map_name=floor.map_name,metadata={"business_request_id":"building-localize-"+uuid.uuid4().hex,"logical_timeout_ms":30000})

    def submit_exploration(self,floor,business_id,request):
        admission=self._admit_child("explore_area_v2",business_id)
        exploration=ExploreAreaRequest(request_id=business_id,map_name=floor.map_name,strategy="wavefront",stop_conditions=dict(request.stop_conditions),
            max_duration_sec=request.per_floor_timeout_sec,metadata={"arbitration":"prefer_wavefront","max_navigation_retries":1,"blacklist_radius_m":0.75})
        task=self.runtime._submit_exploration_task(request=exploration,motion_admission=admission,poll_interval_sec=0.2,
            timeout_sec=min(request.per_floor_timeout_sec+30,self.context.timeout_remaining_sec() or request.per_floor_timeout_sec+30),requested_by=self.requested_by,
            request_context={"parent_task_id":self.task_id},task_metadata={"building_request_id":request.request_id,"request_id":business_id},capability_name="explore_area_v2")
        self.ensure_active();return task.task_id,business_id

    def _find_task(self,task_id):
        if task_id:
            try:return self.runtime.task_manager.get_task_spec(task_id)
            except KeyError:pass
        if self.state is not None and self.state.child_request_id:
            candidates=tuple(self.runtime.task_manager.list_active_tasks())+tuple(self.runtime.task_manager.list_history(limit=100))
            for candidate in candidates:
                spec=self.runtime.task_manager.get_task_spec(candidate.task_id)
                if spec.input_payload.get("request_id")==self.state.child_request_id:return spec
        return None

    def child(self,task_id):
        spec=self._find_task(task_id)
        if spec is not None:
            status=self.runtime.task_manager.get_task_status(spec.task_id)
            result=self.runtime.task_manager.get_task_result(spec.task_id) or {}
            transfer=result.get("building_navigation") if isinstance(result,dict) else None
            if transfer is not None:
                created=transfer.get("created_map_manifest_sha256") if isinstance(transfer,dict) else transfer.created_map_manifest_sha256
            else:created=None
            context=result.get("exploration_context") if isinstance(result,dict) else None
            stop=getattr(getattr(context,"exploration_state",None),"metadata",{}).get("last_stop_reason") if context is not None else None
            return {"state":status.state.value,"message":status.message,"created_manifest":created,"stop_reason":stop}
        if self.state is not None and self.state.child_kind=="transfer":
            transfer=self.runtime._get_building_navigation_service().journal.load()
            if transfer is not None and transfer.request.request_id==self.state.child_request_id and transfer.terminal and transfer.stage=="succeeded":
                return {"state":"succeeded","created_manifest":transfer.created_map_manifest_sha256}
        if self.state is not None and self.state.child_kind=="explore":
            floor=self.state.request.floors[self.state.next_floor_index]
            recovered=self._saved_receipt(self.state,floor)
            if recovered:return {"state":"succeeded","stop_reason":"旧请求的地图保存回执已恢复"}
        return {"state":"missing","message":"原子任务记录不在当前进程，需按持久状态恢复。"}

    def _saved_receipt(self,state,floor):
        bundle=self.runtime._get_building_catalog().assets.get_map_asset_bundle(floor.map_name,migrate_legacy=False)
        if bundle is not None and bundle.metadata.get("exploration_request_id")==state.child_request_id and bundle.manifest_sha256:
            return {"status":"succeeded","manifest_sha256":bundle.manifest_sha256}
        return None

    def asset_receipt(self,state,floor):
        while True:
            self.ensure_active()
            spec=self._find_task(state.child_task_id)
            receipt=spec.metadata.get("map_asset_finalize") if spec is not None else self._saved_receipt(state,floor)
            if receipt:
                if receipt.get("status")!="succeeded" or not receipt.get("manifest_sha256"):
                    raise GatewayError("该层探索的地图资产尚未成功保存。",details={"asset_receipt":receipt})
                return receipt
            time.sleep(0.1)

    def stop_child(self,state):
        spec=self._find_task(state.child_task_id)
        if spec is not None:
            self.runtime.task_manager.cancel_task(spec.task_id,"逐层探索父任务停止")
            deadline=time.monotonic()+15
            while time.monotonic()<deadline:
                status=self.runtime.task_manager.get_task_status(spec.task_id)
                if status.state.value in {"succeeded","failed","cancelled","timeout"}:break
                time.sleep(0.05)
            else:raise GatewayError("逐层探索子任务仍未完成停止。",error_code="building_child_stop_pending")
            if status.state.value=="succeeded":state.child_succeeded=True
            if any(item.task_id==spec.task_id for item in self.runtime.task_manager.list_quarantined_tasks()):
                raise GatewayError("逐层探索子任务仍保留停车隔离。",error_code="building_child_stop_pending")
            error=self.runtime.task_manager.get_task_error_payload(spec.task_id) or {}
            details=error.get("details") or {}
            if details.get("backend_not_started") is True and details.get("request_id")==state.child_request_id:
                return {"request_id":state.child_request_id,"backend_not_started":True}
            events=self.runtime.task_manager.get_task_events(spec.task_id)
            for event in reversed(events):
                proof=event.payload.get("interrupt_barrier_proof") or event.payload.get("completion_proof")
                if proof:return {"child_task_id":spec.task_id,**dict(proof)}
        if state.child_kind=="explore" and state.child_request_id:
            result=self.runtime._stop_exploration_with_safety_barrier(self.context,request_id=state.child_request_id,poll_interval_sec=0.05,acknowledge_interrupt=False)
            return {"request_id":state.child_request_id,"exploration_terminal":True}
        if state.child_kind=="transfer":
            from gateways.building_navigation import GatewayBuildingPorts
            ports=GatewayBuildingPorts(self.runtime,self.context,self.admission,requested_by=self.requested_by)
            journal=self.runtime._get_building_navigation_service().journal.load()
            if journal is not None and journal.request.request_id==state.child_request_id and journal.last_zero_proof.get("zeroed") is True:
                return dict(journal.last_zero_proof)
            return ports.stop_navigation(journal.navigation_goal_id if journal is not None else None)
        return {"backend_not_started":True}

    def checkpoint(self,floor,state):
        if state.child_succeeded:
            deadline=time.monotonic()+30
            while time.monotonic()<deadline:
                spec=self._find_task(state.child_task_id)
                receipt=spec.metadata.get("map_asset_finalize") if spec is not None else self._saved_receipt(state,floor)
                if receipt:
                    if receipt.get("status")=="succeeded" and receipt.get("manifest_sha256"):return receipt["manifest_sha256"]
                    raise GatewayError("已结束探索的存图回执未成功。",details={"receipt":receipt})
                time.sleep(0.05)
            raise GatewayError("已接纳存图仍在处理，不并行覆盖同一地图。")
        current=self.runtime.platform_geometry_runtime.get_mapping_session_context()
        if isinstance(current,dict) and current.get("active") is True and current.get("map_name")==floor.map_name:
            result=self.runtime._finalize_workspace_assets_after_task(map_name=floor.map_name,trigger="explore_building_checkpoint")
            if result.get("map_asset_finalize_error"):raise GatewayError("逐层探索部分地图保存失败。")
            bundle=result.get("finalized_map_asset")
            if bundle is not None:return bundle.manifest_sha256
        asset=self.runtime._get_building_catalog().maps.get_map_asset(floor.map_name)
        return asset.asset_manifest_sha256 if asset is not None and asset.finalized else None

    def interrupt_barrier(self,proof):
        if self.context.is_cancel_requested() or self.context.is_timed_out():self.context.acknowledge_interrupt_barrier(dict(proof))
