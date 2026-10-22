"""逐层探索编排；转移和探索均由现有具名子任务执行。"""
from __future__ import annotations

from contextlib import contextmanager
import fcntl
import json
import time
from typing import Protocol
import uuid

from contracts.building_navigation import BuildingExplorationRequest, BuildingExplorationState, FloorRecord
from contracts.map_identity import AssetIdentityV2
from services.mapping.catalog import _atomic_write_text
from services.navigation.building_catalog import BuildingCatalog


class BuildingExplorationPorts(Protocol):
    task_id: str
    def ensure_active(self): ...
    def active_map_name(self): ...
    def submit_transfer(self, floor, business_id, *, resume=False): ...
    def submit_exploration(self, floor, business_id, request): ...
    def child(self, task_id): ...
    def stop_child(self, state): ...
    def checkpoint(self, floor, state): ...
    def asset_receipt(self, state, floor): ...
    def prepare_floor(self, floor): ...
    def progress(self, state): ...
    def interrupt_barrier(self, proof): ...


class BuildingExplorationService:
    """单个受管工作流保留跨层顺序，不占用子任务的运动资源。"""

    def __init__(self, catalog: BuildingCatalog):
        self.catalog=catalog
        self.path=catalog.maps.root_dir/"building_exploration.json"
        self.lock_path=catalog.maps.root_dir/".building_exploration.lock"

    def load(self):
        if not self.path.exists():return None
        if self.path.stat().st_size>1024*1024:raise ValueError("逐层探索记录超出预算。")
        document=json.loads(self.path.read_text(encoding="utf-8"))
        if not isinstance(document,dict) or type(document.get("schema_version")) is not int or document["schema_version"]!=2:raise ValueError("旧逐层探索依赖已退役的摘要身份，不能自动恢复；请先对账原子任务。")
        return BuildingExplorationState.model_validate(document)

    def save(self,state,ports):
        with self.catalog.maps.map_activation_guard():
            _atomic_write_text(self.path,state.model_dump_json(indent=2))
        ports.progress(state)

    @contextmanager
    def guard(self):
        with self.lock_path.open("a+b")as file:
            try:fcntl.flock(file.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
            except BlockingIOError as error:raise RuntimeError("已有逐层探索工作流正在执行。")from error
            try:yield
            finally:fcntl.flock(file.fileno(),fcntl.LOCK_UN)

    def _topology(self,state):
        topology=self.catalog.get(state.request.building_id)
        if topology is None or topology.revision!=state.topology_revision:raise ValueError("逐层探索绑定的建筑目录修订已改变。")
        actual={floor.floor_id:floor for floor in topology.floors}
        for requested in state.request.floors:
            if requested.floor_id not in actual or actual[requested.floor_id].map_name!=requested.map_name or not actual[requested.floor_id].enabled:
                raise ValueError("逐层探索的楼层绑定已改变。")
        return topology,actual

    def _bind_asset(self,state,floor,identity):
        identity=AssetIdentityV2.model_validate(identity)
        topology,actual=self._topology(state)
        updated=floor.model_copy(update={"asset_identity":identity})
        self.catalog.validate_floor_asset(updated)
        if actual[floor.floor_id].asset_identity!=identity:
            next_topology=topology.model_copy(update={"revision":topology.revision+1,"floors":tuple(updated if f.floor_id==floor.floor_id else f for f in topology.floors)})
            self.catalog.save(next_topology,expected_revision=topology.revision)
            state.topology_revision=next_topology.revision
        return updated

    def _wait_child(self,state,ports):
        while True:
            ports.ensure_active()
            observed=ports.child(state.child_task_id)
            if observed["state"]=="succeeded":return observed
            if observed["state"] in {"failed","cancelled","timeout","missing"}:raise RuntimeError(observed.get("message") or "逐层探索子任务未成功。")
            time.sleep(0.1)

    def run(self,request:BuildingExplorationRequest,ports:BuildingExplorationPorts,*,resume=False):
        with self.guard():
            previous=self.load()
            if resume:
                if previous is None or previous.terminal or previous.request!=request:raise ValueError("没有匹配的逐层探索进度。")
                state=previous;state.attempt+=1;state.needs_reconciliation=False
                ports.progress(state)
                observed=ports.child(state.child_task_id) if state.child_kind else {"state":"missing"}
                if observed["state"]=="succeeded":state.child_succeeded=True
                # 旧任务还存在时先等待它真正退出；不存在时按原设施/地图状态恢复。
                if state.child_task_id and observed["state"] not in {"succeeded","failed","cancelled","timeout","missing"}:
                    ports.stop_child(state)
                if state.child_kind=="transfer" and not state.child_succeeded:
                    _,floors=self._topology(state);floor=floors[request.floors[state.next_floor_index].floor_id]
                    task=ports.submit_transfer(floor,state.child_request_id,resume=True)
                    state.child_task_id=task[0];state.child_request_id=task[1]
                elif state.child_kind=="explore" and not state.child_succeeded:
                    state.child_task_id=None;state.child_request_id=None;state.child_kind=None;state.stage="accepted"
            else:
                if previous is not None and not previous.terminal:raise ValueError("旧逐层探索需要先恢复。")
                state=BuildingExplorationState(request=request,topology_revision=request.topology_revision)
            state.owner_task_id=ports.task_id;self.save(state,ports)
            try:
                while state.next_floor_index<len(request.floors):
                    ports.ensure_active()
                    _,floors=self._topology(state);floor=floors[request.floors[state.next_floor_index].floor_id]
                    if state.child_kind=="transfer" and state.child_task_id:
                        result=self._wait_child(state,ports)
                        state.child_succeeded=True;self.save(state,ports)
                        identity=result.get("created_asset_identity")
                        if identity:floor=self._bind_asset(state,floor,identity)
                        state.child_task_id=None;state.child_request_id=None;state.child_kind=None;state.child_succeeded=False
                        state.stage="accepted";self.save(state,ports)
                    if ports.active_map_name()!=floor.map_name:
                        state.stage="transfer";state.child_kind="transfer";state.child_request_id="building-transfer-"+uuid.uuid4().hex
                        self.save(state,ports)
                        task=ports.submit_transfer(floor,state.child_request_id)
                        state.child_task_id=task[0];state.child_request_id=task[1];self.save(state,ports)
                        continue
                    if not state.child_task_id:
                        ports.prepare_floor(floor)
                        state.stage="exploring";state.child_kind="explore";state.child_request_id="building-explore-"+uuid.uuid4().hex
                        self.save(state,ports)
                        task=ports.submit_exploration(floor,state.child_request_id,request)
                        state.child_task_id=task[0];state.child_request_id=task[1];self.save(state,ports)
                    if not state.child_succeeded:
                        result=self._wait_child(state,ports)
                        state.child_succeeded=True;state.stage="finalizing";state.message=result.get("stop_reason") or "探索终态已确认，等待地图资产。"
                        self.save(state,ports)
                    receipt=ports.asset_receipt(state,floor)
                    floor=self._bind_asset(state,floor,receipt["asset_identity"])
                    state.completed_floors[floor.floor_id]={"map_name":floor.map_name,"asset_identity":floor.asset_identity.model_dump(),"stop_reason":state.message}
                    state.next_floor_index+=1;state.child_task_id=None;state.child_request_id=None;state.child_kind=None;state.child_succeeded=False;state.stage="accepted"
                    self.save(state,ports)
                state.stage="succeeded";state.terminal=True;state.message="设定的楼层探索与地图保存已完成。"
                self.save(state,ports);return state
            except Exception as error:
                proof=ports.stop_child(state)
                state.needs_reconciliation=True;state.message=str(error)
                if state.child_kind=="explore":
                    try:
                        _,floors=self._topology(state);floor=floors[request.floors[state.next_floor_index].floor_id]
                        identity=ports.checkpoint(floor,state)
                        if identity:self._bind_asset(state,floor,identity)
                    except Exception as save_error:state.message+="；地图检查点未完成："+str(save_error)
                self.save(state,ports);ports.interrupt_barrier(proof);raise
