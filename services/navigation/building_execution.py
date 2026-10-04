"""跨层连接状态机；只编排已有导航、地图与设施提供器。"""
from __future__ import annotations

from contextlib import contextmanager
from datetime import datetime, timezone
import fcntl
from pathlib import Path
import time
import uuid
from typing import Callable, Iterator, Mapping, Protocol

from contracts.building_navigation import (
    FacilityOperation, FacilitySnapshot, FloorConnection, FloorExecutionState,
    FloorNavigationRequest, FloorPose, FloorRecord,
)
from providers.facilities import FacilityOutcomeUnknown, FacilityProvider, FacilityUnavailable
from services.mapping.catalog import _atomic_write_text
from services.navigation.building_catalog import BuildingCatalog


class BuildingExecutionPorts(Protocol):
    """由平台组合根注入，实际运动始终经过现有导航和守卫。"""

    def ensure_active(self) -> None: ...
    def remaining_sec(self) -> float: ...
    def navigate(self, floor: FloorRecord, pose: FloorPose, goal_id: str, invariant: Callable[[], None] | None = None) -> Mapping[str, object]: ...
    def stop_navigation(self, goal_id: str | None) -> Mapping[str, object]: ...
    def stop_mapping(self) -> None: ...
    def activate_and_localize(self, floor: FloorRecord, seed: FloorPose) -> bool: ...
    def start_floor_mapping(self, floor: FloorRecord, connection: FloorConnection, request_id: str) -> str: ...
    def dock_to_exit(self, connection: FloorConnection, floor: FloorRecord, invariant: Callable[[], None] | None = None) -> bool: ...
    def active_map_name(self) -> str | None: ...
    def progress(self, state: FloorExecutionState) -> None: ...
    def interrupt_barrier(self, proof: Mapping[str, object], *, reconciled: bool) -> None: ...


class BuildingExecutionJournal:
    """一台机器人只保留一个未决跨层拥有方；意图先于设施副作用落盘。"""

    def __init__(self, catalog: BuildingCatalog) -> None:
        self.catalog = catalog
        self.path = catalog.maps.root_dir / "building_execution.json"
        self.lock_path = catalog.maps.root_dir / ".building_execution.lock"

    @contextmanager
    def execution_guard(self) -> Iterator[None]:
        with self.lock_path.open("a+b") as file:
            try:
                fcntl.flock(file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as error:
                raise RuntimeError("已有跨层任务正在执行或收尾。") from error
            try:
                yield
            finally:
                fcntl.flock(file.fileno(), fcntl.LOCK_UN)

    def load(self) -> FloorExecutionState | None:
        if not self.path.exists():
            return None
        if self.path.stat().st_size > 1024 * 1024:
            raise ValueError("跨层任务记录超过大小预算。")
        return FloorExecutionState.model_validate_json(self.path.read_text(encoding="utf-8"))

    def save(self, state: FloorExecutionState) -> None:
        with self.catalog.maps.map_activation_guard():
            previous = self.load()
            if previous is not None and not previous.terminal and previous.request.request_id != state.request.request_id:
                raise RuntimeError("另一个跨层任务尚未完成对账。")
            _atomic_write_text(self.path, state.model_dump_json(indent=2))


class BuildingNavigationService:
    """原任务线程驱动的可恢复状态机，不创建后台运动循环。"""

    def __init__(self, catalog: BuildingCatalog, providers: Mapping[str, FacilityProvider]) -> None:
        self.catalog = catalog
        self.providers = dict(providers)
        self.journal = BuildingExecutionJournal(catalog)

    def supported_connections(self, building_id: str) -> tuple[str, ...]:
        topology = self.catalog.get(building_id)
        if topology is None:
            return ()
        return tuple(c.connection_id for c in topology.connections if c.enabled
                     and (provider := self.providers.get(c.provider_id)) is not None
                     and provider.is_available() and c.kind in provider.supported_connection_kinds)

    def _save(self, state: FloorExecutionState, ports: BuildingExecutionPorts, stage: str | None = None, message: str = "") -> None:
        if stage is not None and stage != state.stage:
            if state.pending_operation is not None:
                if state.pending_operation.state != "succeeded":
                    raise FacilityOutcomeUnknown("设施操作尚未成功对账，不能改变执行阶段。")
                state.pending_operation = None
            state.stage = stage
        if message:
            state.last_message = message
        self.journal.save(state)
        ports.progress(state)

    @staticmethod
    def _wait(ports: BuildingExecutionPorts) -> None:
        ports.ensure_active()
        time.sleep(min(0.1, max(0.001, ports.remaining_sec())))
        ports.ensure_active()

    def _provider(self, connection: FloorConnection) -> FacilityProvider:
        provider = self.providers.get(connection.provider_id)
        if provider is None or not provider.is_available() or connection.kind not in provider.supported_connection_kinds:
            raise FacilityUnavailable("连接没有当前可用的设施提供器。")
        return provider

    def _snapshot(self, state: FloorExecutionState, connection: FloorConnection, ports: BuildingExecutionPorts) -> FacilitySnapshot:
        ports.ensure_active()
        snapshot = self._provider(connection).snapshot(connection.facility_id, timeout_sec=min(3.0, ports.remaining_sec()))
        age = (datetime.now(timezone.utc) - snapshot.observed_at).total_seconds()
        if age < -0.1 or age > 2.0 or snapshot.sequence < state.last_facility_sequence:
            raise FacilityUnavailable("设施状态过期、来自未来或序列倒退。")
        if snapshot.sequence == state.last_facility_sequence and state.facility_state is not None and snapshot != state.facility_state:
            raise FacilityUnavailable("设施状态在相同序列下改变了内容。")
        if state.facility_session_id and snapshot.session_id != state.facility_session_id:
            raise FacilityUnavailable("设施状态不属于当前预约会话。")
        if not snapshot.available or snapshot.fault or snapshot.occupied_by_other:
            raise FacilityUnavailable("设施故障、不可用或被其它使用者占用。")
        state.last_facility_sequence = snapshot.sequence
        state.facility_state = snapshot
        return snapshot

    def _await_docked(self, state: FloorExecutionState, connection: FloorConnection, floor_id: str, ports: BuildingExecutionPorts) -> None:
        next_feedback = 0.0
        while True:
            snapshot = self._snapshot(state, connection, ports)
            if not snapshot.moving and snapshot.docking_verified and snapshot.docking_calibration_ref == connection.docking_calibration_ref:
                state.current_floor_id = snapshot.floor_id
            if not snapshot.moving and snapshot.floor_id == floor_id and snapshot.door == "open" and snapshot.docking_verified and snapshot.docking_calibration_ref == connection.docking_calibration_ref:
                state.current_floor_id = floor_id
                self._save(state, ports, message="设施已确认目标停层、开门和当前停靠标定。")
                return
            if time.monotonic() >= next_feedback:
                self._save(state, ports, message="等待设施确认目标停层与开门条件。")
                next_feedback = time.monotonic() + 0.5
            self._wait(ports)

    def _operation(self, state: FloorExecutionState, connection: FloorConnection, action: str, target_floor_id: str | None, ports: BuildingExecutionPorts) -> None:
        session_id = state.facility_session_id
        if not session_id:
            raise RuntimeError("设施操作缺少已冻结预约会话。")
        operation = state.pending_operation
        if operation is not None and (operation.session_id != session_id or operation.action != action or operation.target_floor_id != target_floor_id):
            raise FacilityOutcomeUnknown("先前设施操作尚未对账，不能开始另一个动作。")
        provider = self._provider(connection)
        if operation is None:
            operation = FacilityOperation(operation_id="facility-operation-" + uuid.uuid4().hex, session_id=session_id, action=action, target_floor_id=target_floor_id, state="unknown")
            state.pending_operation = operation
            self._save(state, ports, message="设施操作意图已保存，准备提交。")
            ports.ensure_active()
            try:
                operation = provider.submit_operation(connection.facility_id, operation, timeout_sec=min(3.0, ports.remaining_sec()))
            except FacilityOutcomeUnknown:
                # 意图仍在，下一步只查询原操作，直到设施给出明确结果。
                operation = state.pending_operation
        while True:
            state.pending_operation = operation
            self._save(state, ports)
            if operation.state == "succeeded":
                return
            if operation.state == "failed":
                raise FacilityUnavailable(operation.message or "设施操作明确失败。")
            ports.ensure_active()
            queried = provider.query_operation(connection.facility_id, operation, timeout_sec=min(3.0, ports.remaining_sec()))
            if queried.state == "not_found":
                # 只有带原会话和操作身份的明确未发现回执才允许同一幂等键再提交。
                self._save(state, ports, message="设施确认原操作不存在，按同一幂等标识提交。")
                try:
                    queried = provider.submit_operation(connection.facility_id, operation, timeout_sec=min(3.0, ports.remaining_sec()))
                except FacilityOutcomeUnknown:
                    queried = operation.model_copy(update={"state": "unknown"})
            operation = queried
            if operation.state not in ("succeeded", "failed"):
                self._wait(ports)

    def _require_docked(self, state: FloorExecutionState, connection: FloorConnection, floor_id: str, ports: BuildingExecutionPorts) -> None:
        snapshot = self._snapshot(state, connection, ports)
        if snapshot.moving or snapshot.floor_id != floor_id or snapshot.door != "open" or not snapshot.docking_verified or snapshot.docking_calibration_ref != connection.docking_calibration_ref:
            raise FacilityUnavailable("设施不再满足当前进出连接所需的停靠和开门条件。")

    @staticmethod
    def _effective_floor(state: FloorExecutionState, floor: FloorRecord) -> FloorRecord:
        if state.request.create_target_map and floor.floor_id == state.request.target_floor_id and state.created_map_manifest_sha256:
            return floor.model_copy(update={"asset_manifest_sha256": state.created_map_manifest_sha256})
        return floor

    def _navigate(self, state: FloorExecutionState, floor: FloorRecord, pose: FloorPose, ports: BuildingExecutionPorts, invariant: Callable[[], None] | None = None) -> None:
        ports.ensure_active()
        floor=self._effective_floor(state,floor)
        self.catalog.validate_floor_asset(floor)
        if ports.active_map_name() != floor.map_name:
            raise RuntimeError("当前活动地图与此导航阶段绑定楼层不符。")
        state.navigation_goal_id = f"{state.request.request_id}.{state.attempt}.{state.connection_index}.{state.stage}"
        self._save(state, ports)
        if invariant:
            invariant()
        result = ports.navigate(floor, pose, state.navigation_goal_id, invariant)
        if result.get("reached") is not True or result.get("zeroed") is not True:
            raise RuntimeError("层内导航没有同时确认到达与停车。")
        state.last_zero_proof = dict(result)
        state.navigation_goal_id = None
        self._save(state, ports)

    def run(self, request: FloorNavigationRequest, ports: BuildingExecutionPorts, *, resume: bool = False) -> FloorExecutionState:
        with self.journal.execution_guard():
            previous = self.journal.load()
            if resume:
                if previous is None or previous.request != request or previous.terminal:
                    raise RuntimeError("没有与当前冻结请求一致的待恢复跨层任务。")
                state = previous
                proof = ports.stop_navigation(state.navigation_goal_id)
                if proof.get("zeroed") is not True:
                    raise RuntimeError("恢复前尚未确认机器人停车。")
                state.navigation_goal_id = None
                state.attempt += 1
                state.needs_reconciliation = False
                if state.pending_operation is not None and state.pending_operation.state == "failed":
                    state.pending_operation = None
            else:
                if previous is not None and not previous.terminal:
                    raise RuntimeError("旧跨层任务需要先恢复或对账。")
                state = FloorExecutionState(request=request, current_floor_id=request.source_floor_id)
            state.owner_task_id = getattr(ports, "task_id", None)
            self._save(state, ports)
            try:
                while True:
                    ports.ensure_active()
                    topology = self.catalog.validate_frozen(request)
                    floors = {floor.floor_id: floor for floor in topology.floors}
                    connections = {connection.connection_id: connection for connection in topology.connections}
                    if state.connection_index >= len(request.route):
                        self._save(state, ports, "navigating_final", "导航到冻结目标楼层中的最终位姿。")
                        self._navigate(state, floors[request.target_floor_id], request.target_pose, ports)
                        state.current_floor_id = request.target_floor_id
                        state.terminal = True
                        state.needs_reconciliation = False
                        self._save(state, ports, "succeeded", "跨层导航已到达并停车。")
                        return state
                    connection = connections.get(request.route[state.connection_index])
                    if connection is None or not connection.enabled or connection.connection_id not in self.supported_connections(request.building_id):
                        raise RuntimeError("冻结路线中的设施连接已不可用。")
                    source, target = floors[connection.from_floor_id], floors[connection.to_floor_id]
                    self._run_connection(state, connection, source, target, ports)
                    state.connection_index += 1
                    state.facility_session_id = None
                    state.last_facility_sequence = 0
                    state.facility_state = None
                    self._save(state, ports, "accepted", "当前连接已完成，准备下一段。")
            except Exception:
                proof = ports.stop_navigation(state.navigation_goal_id)
                state.last_zero_proof = dict(proof)
                safe_outside = state.stage in {"accepted", "navigating_waiting", "navigating_final"} and state.facility_session_id is None
                state.needs_reconciliation = not safe_outside or proof.get("zeroed") is not True
                state.terminal = not state.needs_reconciliation
                self._save(state, ports, message="执行已中断；机器人停车结果与设施未决操作已保留。")
                ports.interrupt_barrier(proof, reconciled=not state.needs_reconciliation)
                raise

    def _run_connection(self, state: FloorExecutionState, connection: FloorConnection, source: FloorRecord, target: FloorRecord, ports: BuildingExecutionPorts) -> None:
        stages = ("accepted", "navigating_waiting", "reserving", "calling", "waiting_entry", "boarding", "stopping_inside", "suspending_mapping", "transporting", "waiting_arrival", "activating_target", "exiting", "releasing")
        if state.stage not in stages:
            raise RuntimeError("跨层恢复记录包含未知执行阶段。")
        start = stages.index(state.stage)
        for stage in stages[start:]:
            ports.ensure_active()
            self.catalog.validate_frozen(state.request)
            self._save(state, ports, stage)
            if stage == "accepted":
                if state.current_floor_id != source.floor_id or ports.active_map_name() != source.map_name:
                    raise RuntimeError("连接起点楼层与当前工作区不符。")
            elif stage == "navigating_waiting":
                self._navigate(state, source, connection.waiting_pose, ports)
            elif stage == "reserving":
                if state.facility_session_id is None:
                    state.facility_session_id = "facility-session-" + uuid.uuid4().hex
                    self._save(state, ports)
                self._operation(state, connection, "reserve", None, ports)
            elif stage == "calling":
                self._operation(state, connection, "call", source.floor_id, ports)
            elif stage == "waiting_entry":
                self._await_docked(state, connection, source.floor_id, ports)
                self._operation(state, connection, "hold_door", source.floor_id, ports)
            elif stage == "boarding":
                self._await_docked(state, connection, source.floor_id, ports)
                self._navigate(state, source, connection.boarding_pose, ports,
                               lambda: self._require_docked(state, connection, source.floor_id, ports))
            elif stage == "stopping_inside":
                if ports.stop_navigation(state.navigation_goal_id).get("zeroed") is not True:
                    raise RuntimeError("进梯后尚未确认机器人停车。")
            elif stage == "suspending_mapping":
                ports.stop_mapping()
            elif stage == "transporting":
                state.current_floor_id = None
                self._operation(state, connection, "transport", target.floor_id, ports)
            elif stage == "waiting_arrival":
                self._await_docked(state, connection, target.floor_id, ports)
                self._operation(state, connection, "hold_door", target.floor_id, ports)
            elif stage == "activating_target":
                self._await_docked(state, connection, target.floor_id, ports)
                if state.request.create_target_map and target.asset_manifest_sha256 is None:
                    if state.created_map_manifest_sha256 is None:
                        state.created_map_manifest_sha256 = ports.start_floor_mapping(target, connection, state.request.request_id)
                        self._save(state, ports, message="新层已取得真实扫描并保存初始地图检查点。")
                    else:
                        restored=self._effective_floor(state,target)
                        self.catalog.validate_floor_asset(restored)
                        if not ports.activate_and_localize(restored,connection.arrival_pose):
                            raise RuntimeError("新层初始地图尚未恢复定位。")
                    continue
                self.catalog.validate_floor_asset(target)
                if not ports.activate_and_localize(target, connection.arrival_pose):
                    if not connection.allow_docking_without_localization or not ports.dock_to_exit(connection, target, lambda: self._require_docked(state, connection, target.floor_id, ports)):
                        raise RuntimeError("轿厢内定位不可用，且没有已准入的有界对接能力。")
                    if not ports.activate_and_localize(target, connection.exit_pose):
                        raise RuntimeError("对接后仍未取得目标层定位。")
            elif stage == "exiting":
                self._await_docked(state, connection, target.floor_id, ports)
                self._navigate(state, target, connection.exit_pose, ports,
                               lambda: self._require_docked(state, connection, target.floor_id, ports))
            elif stage == "releasing":
                self._operation(state, connection, "release", None, ports)
