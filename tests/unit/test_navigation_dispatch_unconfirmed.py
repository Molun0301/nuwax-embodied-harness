"""Typed dispatch ownership controls. Offline fixtures; no test executed for this proposal."""
from __future__ import annotations

import copy
from datetime import timedelta
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import pytest

from contracts.geometry import Pose
from contracts.navigation import NavigationGoal, NavigationStatus
from core import StateStore
from core.errors import GatewayError
from scripts.check_navigation_map_snapshot_identity import OfflineProvider
from services.geometry import geometry_runtime_provider as module
from services.geometry.geometry_runtime_provider import GeometryRuntimeProvider
from services.geometry.proto_gen import geometry_runtime_pb2 as pb2
from services.navigation.forbidden_zones import ForbiddenZoneStore
from services.navigation.service import NavigationService
from services.navigation.state_machine import NavigationStateMachine, map_nav2_status_to_platform

GOAL = "typed-public-goal"
PUBLIC = pb2.NAVIGATION_OWNER_KIND_PUBLIC_NAV
ORIGIN = "GOAL_PUBLISH_UNCONFIRMED"
PENDING = "GOAL_TERMINAL_PROOF_PENDING"
ADMISSION = {"generation": 1, "boot_nonce_hex": "01" * 16, "motion_stop_generation": 0}


def wire(status="UNCONFIRMED", *, goal_id=GOAL, reason=ORIGIN, timestamp=0,
         path=3, planner=8, proof=False, generation=2, producer=7):
    return pb2.NavigationState(
        status=status, goal_id=goal_id, owner_kind=PUBLIC, parent_request_id=goal_id,
        goal_generation=generation, producer_runtime_generation=producer,
        planner_runtime_generation=planner, path_revision=path, timestamp_ns=timestamp,
        reason_code=reason, message="fixture execution body", distance_remaining=2,
        cancel_phase="ZEROED" if proof else "", cancel_generation=9 if proof else 0,
        actuator_zeroed=proof, zero_ack_id="fixture-exact-zero" if proof else "",
        zero_applied_monotonic_ns=71 if proof else 0,
        runtime_generation=7, guard_generation=4, sink_ready=True,
        planner_map_frame="map", planner_pose_frame="map",
        planned_path=[pb2.Vec3(x=1, y=2, z=0)], local_path=[pb2.Vec3(x=2, y=3, z=0)],
    )


def goal(goal_id=GOAL):
    return NavigationGoal(goal_id=goal_id, target_pose=Pose(frame_id="map"), metadata={"motion_admission": ADMISSION})


class Stub:
    def __init__(self, response=None):
        self.response = wire() if response is None else response
        self.cancel_response = wire("pending", reason="CANCEL_REQUESTED", timestamp=0)
        self.before_set_return = None
        self.before_cancel_return = None
        self.set_calls = []
        self.cancel_calls = []
        self.cloud_calls = []
        self.cloud_response = None

    def SetNavigationGoal(self, request, **kwargs):
        self.set_calls.append(request)
        if self.before_set_return:
            self.before_set_return()
        return self.response

    def CancelNavigation(self, request, **kwargs):
        self.cancel_calls.append(request)
        if self.before_cancel_return:
            self.before_cancel_return()
        return self.cancel_response

    def SubmitMappingCloudGoal(self, request, **kwargs):
        self.cloud_calls.append(request)
        if self.before_set_return:
            self.before_set_return()
        return self.cloud_response


def provider(response=None):
    backend = GeometryRuntimeProvider(NS())
    backend._stub = Stub(response)
    backend._connected = True
    backend._nav_connection_epoch = 4
    backend.stream_starts = 0
    def ensure_stream():
        backend.stream_starts += 1
        if backend._nav_active_stream_token is None:
            backend._nav_stream_epoch += 1
            backend._nav_active_stream_token = (backend._nav_connection_epoch, backend._nav_stream_epoch, id(backend._stub))
            backend._nav_stream_started = True
    backend._ensure_nav_state_stream = ensure_stream
    return backend


def snapshot(backend):
    return backend._build_navigation_state_snapshot(include_readiness=False, include_exploration_overlay=False)


def stream(backend, update, *, token=None, stub=None):
    return backend._cache_navigation_update(update,
        stream_token=backend._nav_active_stream_token if token is None else token,
        stream_stub=backend._stub if stub is None else stub)


def submit(backend):
    accepted = backend.set_goal(goal())
    return accepted, backend.take_goal_submission_result(GOAL)


def precommit(backend, *updates):
    backend._ensure_nav_state_stream()
    backend._stub.before_set_return = lambda: [stream(backend, update) for update in updates]


class ServiceProvider(OfflineProvider):
    def __init__(self, backend):
        super().__init__()
        self.backend = backend

    def set_goal(self, value, **kwargs):
        self.goals.append(value)
        return self.backend.set_goal(value, **kwargs)

    def take_goal_submission_result(self, value):
        return self.backend.take_goal_submission_result(value)

    def get_navigation_state(self, **kwargs):
        return snapshot(self.backend)

    def get_navigation_control_state(self, **kwargs):
        return snapshot(self.backend)

    def cancel_goal(self, request_id=None, **kwargs):
        return self.backend.cancel_goal(request_id, **kwargs)


def service(backend, tmp_path):
    view = ServiceProvider(backend)
    navigation = NavigationService(provider_owner=NS(providers=view), state_store=StateStore(),
        mapping_service=NS(get_latest_snapshot=lambda: None),
        localization_service=NS(get_active_session=lambda: None),
        forbidden_zone_store=ForbiddenZoneStore(tmp_path / "zones.json"))
    return navigation, view


def cloud_submit(backend):
    authority = {"committed_occupancy_commit_id": {"boot_nonce": "01" * 16, "sequence": 1},
        "map_name": "fixture", "map_frame_id": "map", "runtime_generation": 7,
        "mapping_session_id": "mapping", "mapping_session_generation": 1,
        "committed_occupancy_revision": 1, "committed_occupancy_source_timestamp_ns": 1}
    # These tests isolate navigation command ownership, not map activation validation.
    with patch.object(module, "_mapping_cloud_authority_receipt", return_value=None), \
         patch.object(module, "_assigned_activation_receipt_proto", return_value=None), \
         patch.object(module.pb2, "MappingCloudGoalRequest", side_effect=lambda **kw: NS(**kw)):
        return backend.submit_mapping_cloud_goal(query_request_id=GOAL, query_text="fixture",
            authority=authority, georeference_json=b"{}",
            motion_admission={"guard_generation": 1, "guard_boot_nonce": bytes([1]) * 16, "motion_stop_generation": 0})


def cloud_reply(navigation=None, status="ADMISSION_UNCONFIRMED", query_request_id=GOAL):
    return NS(navigation=navigation, status=status, message="fixture", query_request_id=query_request_id,
              HasField=lambda field: field == "navigation" and navigation is not None)


def test_typed_unknown_false_keeps_exact_lease_goal_and_stream():
    backend = provider()
    accepted, result = submit(backend)
    assert accepted is False
    assert result["disposition"] == "owned_unconfirmed"
    assert result["goal_id"] == GOAL and result["navigation"]["goal_id"] == GOAL
    assert result["ownership_retained"] is True and result["authority_current"] is True
    assert snapshot(backend).status == NavigationStatus.UNCONFIRMED
    assert backend.stream_starts == 1
    assert backend._current_navigation_lease_locked() == (GOAL, PUBLIC, GOAL, 2, 7)


def test_compatibility_facade_retains_unconfirmed_goal_id():
    backend = provider()
    result = backend.set_navigation_goal(0, 0, 0, 0, 0, 0, 1, goal_id=GOAL, motion_admission=ADMISSION)
    assert result["accepted"] is False and result["goal_id"] == GOAL
    assert result["disposition"] == "owned_unconfirmed"


@pytest.mark.parametrize("status", ["planning", "running"])
def test_typed_progress_accepts_without_releasing_obligation(status):
    backend = provider(wire(status))
    accepted, result = submit(backend)
    assert accepted and result["disposition"] == "confirmed"
    state = snapshot(backend)
    assert state.status.value == status
    assert state.metadata["ownership_retained"] is True
    assert state.metadata["command_projection_without_source"] is True
    assert backend._last_nav_timestamp_ns == 0


def test_stronger_precommit_keeps_unary_origin_and_real_source():
    backend = provider()
    precommit(backend, wire("running", reason="REAL_RUNNING", timestamp=100))
    accepted, result = submit(backend)
    assert accepted
    assert result["reason_code"] == ORIGIN
    state = snapshot(backend)
    assert state.status == NavigationStatus.RUNNING
    assert state.metadata["reason_code"] == "REAL_RUNNING"
    assert state.metadata["dispatch_obligation"]["origin_reason_code"] == ORIGIN
    assert not state.metadata.get("command_projection_without_source")
    assert backend._last_nav_timestamp_ns == 100


def test_equal_running_precommit_replaces_source_less_command_body():
    backend = provider(wire("running"))
    precommit(backend, wire("running", reason="REAL_SOURCE", timestamp=100))
    accepted, result = submit(backend)
    assert accepted and result["navigation"]["timestamp_ns"] == 100
    assert result["navigation"]["reason_code"] == "REAL_SOURCE"
    assert result["ownership_retained"]


def test_stream_typed_reason_cannot_establish_dispatch_obligation():
    backend = provider(wire("accepted", reason="ACCEPTED", timestamp=50))
    submit(backend)
    assert stream(backend, wire("running", reason=ORIGIN, timestamp=100))
    assert not snapshot(backend).metadata.get("ownership_retained")
    assert "dispatch_obligation" not in snapshot(backend).metadata


@pytest.mark.parametrize("mutation", ["foreign_goal", "zero_generation", "wrong_owner", "message_only"])
def test_untrusted_or_noncanonical_unary_cannot_establish_ownership(mutation):
    update = wire()
    if mutation == "foreign_goal":
        update.goal_id = update.parent_request_id = "foreign"
    elif mutation == "zero_generation":
        update.goal_generation = 0
    elif mutation == "wrong_owner":
        update.owner_kind = pb2.NAVIGATION_OWNER_KIND_EXPLORATION_CHILD
    else:
        update.status = "FAILED"
        update.reason_code = ""
        update.message = ORIGIN
    backend = provider(update)
    accepted, result = submit(backend)
    assert not accepted and result["disposition"] == "rejected"
    assert not result["ownership_retained"]
    assert snapshot(backend).current_goal_id is None


@pytest.mark.parametrize("terminal", ["failed", "succeeded"])
def test_owned_unproved_terminal_projects_unknown_and_blocks_running(terminal):
    backend = provider()
    submit(backend)
    assert stream(backend, wire("running", reason="RUNNING", timestamp=90))
    assert stream(backend, wire(terminal, reason="REAL_TERMINAL", timestamp=100))
    state = snapshot(backend)
    assert state.status == NavigationStatus.UNCONFIRMED
    assert state.metadata["observed_status"] == terminal
    assert state.metadata["reason_code"] == PENDING
    assert state.metadata["dispatch_obligation"]["terminal_observed"]
    assert not stream(backend, wire("running", reason="LATE", timestamp=110))
    assert stream(backend, wire(terminal, reason="ORIGINAL_RECEIPT", timestamp=101, proof=True))
    state = snapshot(backend)
    assert state.status.value == terminal
    assert not state.metadata["ownership_retained"]
    assert state.metadata["reason_code"] == "ORIGINAL_RECEIPT"
    assert "dispatch_obligation" not in state.metadata


def test_terminal_gap_typed_marker_blocks_running_without_guessing_terminal_kind():
    backend = provider(wire(reason=PENDING))
    submit(backend)
    state = snapshot(backend)
    assert state.metadata["dispatch_obligation"]["terminal_observed"]
    assert "observed_status" not in state.metadata
    assert not stream(backend, wire("running", reason="RUNNING", timestamp=20))
    assert stream(backend, wire("failed", reason="RECEIPT", timestamp=25, proof=True))


@pytest.mark.parametrize("mutation", ["old_path", "wrong_planner", "old_source", "foreign_lease", "missing_zero"])
def test_proof_upgrade_requires_exact_path_planner_source_and_zero(mutation):
    backend = provider()
    submit(backend)
    assert stream(backend, wire("failed", reason="OBSERVED", timestamp=100))
    before = copy.deepcopy(backend._last_nav_metadata)
    receipt = wire("failed", reason="RECEIPT", timestamp=101, proof=True)
    if mutation == "old_path":
        receipt.path_revision = 2
    elif mutation == "wrong_planner":
        receipt.planner_runtime_generation = 9
    elif mutation == "old_source":
        receipt.timestamp_ns = 99
    elif mutation == "foreign_lease":
        receipt.goal_generation = 1
    else:
        receipt.zero_ack_id = ""
    assert not stream(backend, receipt)
    assert backend._last_nav_metadata == before
    assert snapshot(backend).metadata["ownership_retained"]


def test_precommit_unproved_terminal_then_exact_proof_resolves_before_unary():
    backend = provider()
    receipt = wire("failed", reason="COMPLETE_RECEIPT", timestamp=101, proof=True)
    precommit(backend, wire("failed", reason="OBSERVED", timestamp=100), receipt)
    accepted, result = submit(backend)
    assert not accepted and result["disposition"] == "resolved"
    for key, value in module._navigation_wire_document(receipt).items():
        assert result["navigation"][key] == value
    assert not result["ownership_retained"]


@pytest.mark.parametrize("change", ["path_revision", "planner_runtime_generation", "timestamp_ns"])
def test_precommit_bad_proof_upgrade_does_not_escape_terminal_observation(change):
    backend = provider()
    bad = wire("failed", reason="BAD", timestamp=101, proof=True)
    setattr(bad, change, {"path_revision": 2, "planner_runtime_generation": 9, "timestamp_ns": 99}[change])
    precommit(backend, wire("failed", reason="OBSERVED", timestamp=100), bad)
    accepted, result = submit(backend)
    assert not accepted and result["disposition"] == "owned_unconfirmed"
    assert result["navigation"]["observed_status"] == "failed"
    assert result["navigation"]["zero_ack_id"] == ""


def test_ordinary_failed_without_zero_keeps_terminal_contract():
    backend = provider(wire("accepted", reason="ACCEPTED", timestamp=50))
    submit(backend)
    assert stream(backend, wire("failed", reason="ORDINARY_FAILURE", timestamp=100))
    assert snapshot(backend).status == NavigationStatus.FAILED
    assert not snapshot(backend).metadata["ownership_retained"]
    assert not stream(backend, wire("failed", reason="LATE_PROOF", timestamp=101, proof=True))


def test_cancel_advances_command_but_retains_origin_until_exact_receipt():
    backend = provider()
    submit(backend)
    previous_generation = backend._nav_command_generation
    previous = snapshot(backend)
    assert backend.cancel_goal(GOAL)
    assert backend._nav_command_generation == previous_generation + 1
    state = snapshot(backend)
    assert state.metadata["dispatch_obligation"] == previous.metadata["dispatch_obligation"]
    assert state.metadata["ownership_retained"]
    request = backend._stub.cancel_calls[0]
    assert module._navigation_lease_from_message(request) == (GOAL, PUBLIC, GOAL, 2, 7)
    backend._stub.cancel_response = wire("cancelled", reason="ZERO_CONFIRMED", timestamp=100, proof=True)
    assert backend.cancel_goal(GOAL)
    assert snapshot(backend).status == NavigationStatus.CANCELLED
    assert not snapshot(backend).metadata["ownership_retained"]


def test_cancel_unproved_failure_retains_responsibility_and_terminal_observation():
    backend = provider()
    submit(backend)
    backend._stub.cancel_response = wire("failed", reason="CANCEL_TIMED_OUT", timestamp=100)
    assert not backend.cancel_goal(GOAL)
    state = snapshot(backend)
    assert state.status == NavigationStatus.UNCONFIRMED
    assert state.metadata["observed_status"] == "failed"
    assert state.metadata["ownership_retained"]
    assert not stream(backend, wire("running", timestamp=101))


def test_same_epoch_disconnect_preserves_source_path_origin_and_recovery():
    backend = provider()
    submit(backend)
    assert stream(backend, wire("running", reason="RUNNING", timestamp=100))
    before = snapshot(backend)
    assert backend._mark_navigation_stream_disconnected("fixture EOF",
        stream_token=backend._nav_active_stream_token, stream_stub=backend._stub)
    after = snapshot(backend)
    assert after.status == NavigationStatus.RUNNING
    assert after.updated_at == before.updated_at
    for key in ("reason_code", "path_revision", "dispatch_obligation", "ownership_retained"):
        assert after.metadata[key] == before.metadata[key]
    assert stream(backend, wire("failed", reason="TERMINAL", timestamp=101))
    assert snapshot(backend).status == NavigationStatus.UNCONFIRMED


def test_epoch_replacement_keeps_history_but_never_cancels_new_stub():
    backend = provider()
    submit(backend)
    token, old_stub = backend._nav_active_stream_token, backend._stub
    backend._transition_navigation_connection(connected=True, message="replacement")
    backend._stub = Stub()
    backend._ensure_nav_state_stream()
    state = snapshot(backend)
    assert state.status == NavigationStatus.UNCONFIRMED and state.current_goal_id == GOAL
    assert state.metadata["ownership_retained"] and not state.metadata["authority_current"]
    assert state.metadata["historical_navigation_identity"] == {
        "goal_id": GOAL, "owner_kind": PUBLIC, "parent_request_id": GOAL,
        "goal_generation": 2, "producer_runtime_generation": 7}
    assert state.metadata["goal_generation"] == 0
    assert not backend.cancel_goal(GOAL) and not backend._stub.cancel_calls
    assert not stream(backend, wire("cancelled", timestamp=100, proof=True), token=token, stub=old_stub)
    assert not stream(backend, wire("running", timestamp=100))
    assert not stream(backend, pb2.NavigationState(status="idle"))
    assert snapshot(backend).metadata["ownership_retained"]
    assert not backend.set_goal(goal("new-goal")) and not backend._stub.set_calls


def test_late_unary_after_epoch_replacement_cannot_adopt_lease():
    backend = provider()
    backend._stub.before_set_return = lambda: backend._transition_navigation_connection(connected=True, message="new epoch")
    accepted, result = submit(backend)
    assert not accepted and result["disposition"] == "rejected"
    assert not result["ownership_retained"]
    assert snapshot(backend).current_goal_id is None


def test_cloud_busy_does_not_query_register_token_or_block_old_cancel():
    backend = provider()
    submit(backend)
    command, lease = backend._nav_command_generation, backend._current_navigation_lease_locked()
    result = cloud_submit(backend)
    assert result["status"] == "BUSY"
    assert not backend._stub.cloud_calls
    assert backend._nav_command_generation == command
    assert backend._current_navigation_lease_locked() == lease
    assert backend._nav_inflight_command_token is None
    assert backend.cancel_goal(GOAL)
    assert len(backend._stub.cancel_calls) == 1


@pytest.mark.parametrize("status,expected,accepted", [("UNCONFIRMED", "ADMISSION_UNCONFIRMED", False), ("running", "SUCCEEDED", True)])
def test_cloud_adopts_same_cache_transaction_for_unknown_and_progress(status, expected, accepted):
    backend = provider()
    backend._stub.cloud_response = cloud_reply(wire(status))
    result = cloud_submit(backend)
    assert result["status"] == expected and result["accepted"] is accepted
    assert result["ownership_retained"] and result["authority_current"]
    assert result["navigation"]["goal_id"] == GOAL
    assert snapshot(backend).current_goal_id == GOAL
    assert backend._nav_command_generation == 1
    assert backend.stream_starts == 1


def test_cloud_missing_navigation_cannot_claim_installed_unknown():
    backend = provider()
    backend._stub.cloud_response = cloud_reply(None)
    result = cloud_submit(backend)
    assert result["status"] == "ADMISSION_REJECTED"
    assert not result["ownership_retained"] and result["navigation"] is None
    assert backend._nav_inflight_command_token is None


def test_cloud_mismatching_query_id_cannot_commit_navigation():
    backend = provider()
    backend._stub.cloud_response = cloud_reply(wire(), query_request_id="foreign")
    result = cloud_submit(backend)
    assert result["status"] == "ADMISSION_REJECTED"
    assert not result["ownership_retained"]
    assert snapshot(backend).current_goal_id is None


def test_cloud_complete_failed_receipt_is_execution_result_not_rejection():
    backend = provider()
    receipt = wire("failed", reason="REAL_FAILURE", timestamp=100, proof=True)
    backend._stub.cloud_response = cloud_reply(receipt, status="FAILED")
    result = cloud_submit(backend)
    assert result["status"] == "FAILED" and result["disposition"] == "resolved"
    assert result["navigation"]["zero_ack_id"] == "fixture-exact-zero"
    assert result["navigation"]["reason_code"] == "REAL_FAILURE"


def test_service_owned_unknown_preserves_goal_and_raises_specific_error(tmp_path):
    backend = provider()
    navigation, view = service(backend, tmp_path)
    requested = goal()
    with pytest.raises(GatewayError) as caught:
        navigation.set_goal(requested)
    assert caught.value.error_code == "navigation_dispatch_unconfirmed"
    assert caught.value.details["retryable"] is False
    assert caught.value.details["navigation"]["goal_id"] == GOAL
    assert navigation._current_goal is requested
    assert navigation._latest_navigation_context.navigation_state.status == NavigationStatus.UNCONFIRMED
    assert navigation._state_machine.is_terminal is False
    assert navigation._state_machine.is_running is False


def test_service_resolved_false_consumes_receipt_before_rejection_branch(tmp_path):
    backend = provider(wire("failed", reason="REAL_FAILED_RECEIPT", timestamp=100, proof=True))
    navigation, view = service(backend, tmp_path)
    requested = goal()
    context = navigation.set_goal(requested)
    assert context.current_goal is requested
    assert context.navigation_state.current_goal_id == GOAL
    assert context.navigation_state.status == NavigationStatus.FAILED
    assert context.navigation_state.metadata["zero_ack_id"] == "fixture-exact-zero"
    assert context.navigation_state.metadata["reason_code"] == "REAL_FAILED_RECEIPT"
    assert navigation._current_goal is None


def test_service_real_running_earlier_than_command_display_time_wins(tmp_path):
    backend = provider(wire("running"))
    navigation, view = service(backend, tmp_path)
    initial = navigation.set_goal(goal()).navigation_state
    assert initial.metadata["command_projection_without_source"]
    assert stream(backend, wire("running", reason="REAL_RUNNING", timestamp=1_000_000_000))
    context = navigation._ingest_navigation_state(view, snapshot(backend))
    assert context.navigation_state.updated_at < initial.updated_at
    assert context.navigation_state.status == NavigationStatus.RUNNING
    assert context.navigation_state.metadata["reason_code"] == "REAL_RUNNING"
    assert not context.navigation_state.metadata.get("command_projection_without_source")
    assert context.navigation_state.metadata["ownership_retained"]
    # Reverse direction is deliberately asymmetric, even with a later display timestamp.
    returned = navigation._ingest_navigation_state(view,
        initial.model_copy(update={"updated_at": initial.updated_at + timedelta(seconds=1)}))
    assert returned is context
    assert returned.navigation_state.metadata["source_timestamp_ns"] == 1_000_000_000


def test_service_source_less_command_cannot_erase_terminal_receipt(tmp_path):
    backend = provider(wire("running"))
    navigation, view = service(backend, tmp_path)
    initial = navigation.set_goal(goal()).navigation_state
    assert stream(backend, wire("failed", reason="FULL_RECEIPT", timestamp=1_000_000_000, proof=True))
    context = navigation._ingest_navigation_state(view, snapshot(backend))
    assert context.navigation_state.status == NavigationStatus.FAILED
    assert navigation._ingest_navigation_state(view, initial) is context
    assert context.navigation_state.metadata["zero_ack_id"] == "fixture-exact-zero"


def test_service_cross_epoch_cancel_is_explicitly_unavailable(tmp_path):
    backend = provider()
    navigation, view = service(backend, tmp_path)
    with pytest.raises(GatewayError):
        navigation.set_goal(goal())
    backend._transition_navigation_connection(connected=True, message="new epoch")
    backend._stub = Stub()
    with pytest.raises(GatewayError) as caught:
        navigation.cancel_goal(GOAL)
    assert caught.value.error_code == "navigation_control_authority_unavailable"
    assert caught.value.details["authority_current"] is False
    assert not backend._stub.cancel_calls
    assert navigation._current_goal is not None


def test_unknown_state_machine_is_nonterminal_and_not_running():
    machine = NavigationStateMachine()
    machine.update_from_backend_state("fixture", "UNCONFIRMED", goal_id=GOAL)
    assert machine.current_status == NavigationStatus.UNCONFIRMED
    assert not machine.is_running and not machine.is_terminal
    assert machine.current_goal_id == GOAL
    assert machine.get_state_summary()["end_time"] is None
    assert map_nav2_status_to_platform("UNCONFIRMED") == NavigationStatus.UNCONFIRMED


def test_terminal_gap_keeps_real_precommit_source_floor_without_resurrecting_running():
    backend = provider(wire(reason=PENDING))
    precommit(backend, wire("running", reason="REAL_RUNNING", timestamp=100))
    accepted, result = submit(backend)
    assert not accepted and result["disposition"] == "owned_unconfirmed"
    assert result["navigation"]["status"] == "unconfirmed"
    assert result["navigation"]["timestamp_ns"] == 0
    assert result["navigation"]["dispatch_obligation"]["source_timestamp_floor_ns"] == 100
    assert not stream(backend, wire("failed", reason="STALE_PROOF", timestamp=99, proof=True))
    assert stream(backend, wire("failed", reason="REAL_RECEIPT", timestamp=101, proof=True))


def test_terminal_gap_preserves_actual_unproved_precommit_terminal_kind():
    backend = provider(wire(reason=PENDING))
    precommit(backend, wire("failed", reason="REAL_FAILED", timestamp=100))
    accepted, result = submit(backend)
    assert not accepted
    assert result["navigation"]["observed_status"] == "failed"
    assert result["navigation"]["timestamp_ns"] == 100
    assert not result["navigation"]["command_projection_without_source"]


def test_service_new_typed_terminal_observation_can_advance_real_running(tmp_path):
    backend = provider(wire("running"))
    navigation, view = service(backend, tmp_path)
    navigation.set_goal(goal())
    assert stream(backend, wire("running", reason="REAL_RUNNING", timestamp=1_000_000_000))
    real = navigation._ingest_navigation_state(view, snapshot(backend))
    # A second isolated provider supplies the exact trusted unary projection body;
    # this is service ordering only, not a new runtime submission or a retry Goal.
    pending_backend = provider(wire(reason=PENDING))
    submit(pending_backend)
    pending = snapshot(pending_backend)
    context = navigation._ingest_navigation_state(view, pending)
    assert context is not real
    assert context.navigation_state.status == NavigationStatus.UNCONFIRMED
    assert context.navigation_state.metadata["dispatch_obligation"]["terminal_observed"]
    assert navigation._ingest_navigation_state(view, real.navigation_state) is context


def test_ordinary_zero_source_and_idle_do_not_get_command_projection_exception(tmp_path):
    backend = provider(wire("accepted", reason="ACCEPTED", timestamp=0))
    navigation, view = service(backend, tmp_path)
    initial = navigation.set_goal(goal()).navigation_state
    assert not initial.metadata.get("command_projection_without_source")
    old = initial.model_copy(update={"updated_at": initial.updated_at - timedelta(seconds=1)})
    previous = navigation._latest_navigation_context
    assert navigation._ingest_navigation_state(view, old) is previous
    idle_backend = provider()
    assert snapshot(idle_backend).status == NavigationStatus.IDLE
    assert not snapshot(idle_backend).metadata.get("command_projection_without_source")


def test_ordinary_accepted_then_precommit_failed_keeps_old_acceptance_contract():
    backend = provider(wire("accepted", reason="ACCEPTED", timestamp=50))
    precommit(backend, wire("failed", reason="ORDINARY_FAILED", timestamp=100))
    accepted, result = submit(backend)
    assert accepted and result["disposition"] == "confirmed"
    assert result["navigation"]["status"] == "failed"
    assert not result["ownership_retained"]
    assert snapshot(backend).status == NavigationStatus.FAILED


def test_cloud_historical_owner_busy_survives_new_epoch_idle():
    backend = provider()
    submit(backend)
    backend._transition_navigation_connection(connected=True, message="replacement")
    backend._stub = Stub()
    backend._ensure_nav_state_stream()
    assert not stream(backend, pb2.NavigationState(status="idle"))
    generation = backend._nav_command_generation
    result = cloud_submit(backend)
    assert result["status"] == "BUSY"
    assert backend._nav_command_generation == generation
    assert not backend._stub.cloud_calls
    assert not backend.cancel_goal(GOAL)
    assert not backend._stub.cancel_calls


@pytest.mark.parametrize("new_path,new_planner", [(4, 8), (4, 7), (4, 9)])
def test_stale_terminal_gap_projection_cannot_cover_newer_precommit_path(new_path, new_planner):
    backend = provider(wire(reason=PENDING, path=3, planner=8))
    precommit(backend, wire("running", reason="NEW_PATH_RUNNING", timestamp=100,
                           path=new_path, planner=new_planner))
    accepted, result = submit(backend)
    assert accepted and result["disposition"] == "confirmed"
    state = snapshot(backend)
    assert state.status == NavigationStatus.RUNNING
    assert state.metadata["path_revision"] == new_path
    assert state.metadata["planner_runtime_generation"] == new_planner
    assert not state.metadata["dispatch_obligation"]["terminal_observed"]
    assert state.metadata["ownership_retained"]
    assert stream(backend, wire("failed", reason="NEW_PATH_TERMINAL", timestamp=101,
                               path=new_path, planner=new_planner))
    assert snapshot(backend).status == NavigationStatus.UNCONFIRMED


@pytest.mark.parametrize("new_path,new_planner", [(4, 8), (3, 9)])
def test_service_old_path_terminal_gap_cannot_cover_real_running(tmp_path, new_path, new_planner):
    backend = provider(wire("running", path=new_path, planner=new_planner))
    navigation, view = service(backend, tmp_path)
    navigation.set_goal(goal())
    assert stream(backend, wire("running", reason="CURRENT_RUNNING", timestamp=100,
                               path=new_path, planner=new_planner))
    context = navigation._ingest_navigation_state(view, snapshot(backend))
    old_path_backend = provider(wire(reason=PENDING, path=3, planner=8))
    submit(old_path_backend)
    assert navigation._ingest_navigation_state(view, snapshot(old_path_backend)) is context
    assert context.navigation_state.status == NavigationStatus.RUNNING


def test_stale_same_epoch_stream_token_cannot_advance_owned_cache():
    backend = provider()
    submit(backend)
    old_token = backend._nav_active_stream_token
    backend._nav_stream_epoch += 1
    backend._nav_active_stream_token = (backend._nav_connection_epoch, backend._nav_stream_epoch, id(backend._stub))
    assert not stream(backend, wire("failed", timestamp=100, proof=True), token=old_token)
    assert snapshot(backend).metadata["ownership_retained"]
    assert stream(backend, wire("failed", timestamp=100, proof=True))


def test_epoch_transition_drains_started_callback_before_invalidating_lease():
    import threading
    backend = provider()
    submit(backend)
    entered, release, waiting, finished = (threading.Event() for _ in range(4))
    observed = []
    def callback(state):
        entered.set()
        assert release.wait(2), "fixture callback release missing"
        observed.append(state.current_goal_id)
    backend._navigation_update_subscribers[1] = callback
    ticket = backend._begin_navigation_notification(snapshot(backend))
    assert ticket is not None
    deliver = threading.Thread(target=backend._notify_navigation_subscribers, args=(ticket,))
    original_wait = backend._wait_for_navigation_notifications
    def wait_for_callbacks():
        waiting.set()
        return original_wait()
    backend._wait_for_navigation_notifications = wait_for_callbacks
    def replace_epoch():
        backend._transition_navigation_connection(connected=True, message="replacement")
        finished.set()
    transition = threading.Thread(target=replace_epoch)
    deliver.start()
    try:
        assert entered.wait(2)
        transition.start()
        assert waiting.wait(2)
        assert not finished.is_set()
        assert backend._nav_connection_epoch == 4
    finally:
        release.set()
        deliver.join(2)
        if transition.ident is not None:
            transition.join(2)
    assert not deliver.is_alive() and not transition.is_alive()
    assert finished.is_set() and observed == [GOAL]
    assert snapshot(backend).metadata["authority_current"] is False


def test_old_cancel_ack_after_connection_change_never_clears_history():
    backend = provider()
    submit(backend)
    old_stub = backend._stub
    old_stub.cancel_response = wire("cancelled", timestamp=100, proof=True)
    def replace():
        backend._transition_navigation_connection(connected=True, message="new epoch")
        backend._stub = Stub()
    old_stub.before_cancel_return = replace
    assert not backend.cancel_goal(GOAL)
    state = snapshot(backend)
    assert state.status == NavigationStatus.UNCONFIRMED
    assert state.metadata["ownership_retained"] and not state.metadata["authority_current"]
    assert not backend._stub.cancel_calls


def test_exact_stream_receipt_during_cancel_wins_over_late_pending_ack():
    backend = provider()
    submit(backend)
    backend._stub.before_cancel_return = lambda: stream(backend,
        wire("cancelled", reason="REAL_RECEIPT", timestamp=100, proof=True))
    assert backend.cancel_goal(GOAL)
    state = snapshot(backend)
    assert state.status == NavigationStatus.CANCELLED
    assert state.metadata["reason_code"] == "REAL_RECEIPT"
    assert not state.metadata["ownership_retained"]


def test_cancel_bad_path_proof_cannot_release_observed_terminal():
    backend = provider()
    submit(backend)
    assert stream(backend, wire("failed", reason="OBSERVED", timestamp=100))
    backend._stub.cancel_response = wire("cancelled", reason="OLD_PATH", timestamp=101, path=2, proof=True)
    assert not backend.cancel_goal(GOAL)
    state = snapshot(backend)
    assert state.status == NavigationStatus.UNCONFIRMED
    assert state.metadata["path_revision"] == 3
    assert state.metadata["ownership_retained"]


def test_cloud_late_response_after_epoch_change_cannot_commit_navigation():
    backend = provider()
    old_stub = backend._stub
    old_stub.cloud_response = cloud_reply(wire())
    def replace():
        backend._transition_navigation_connection(connected=True, message="replacement")
        backend._stub = Stub()
    old_stub.before_set_return = replace
    result = cloud_submit(backend)
    assert result["status"] == "ADMISSION_REJECTED"
    assert not result["ownership_retained"]
    assert snapshot(backend).current_goal_id is None


def test_owned_goal_blocks_new_direct_set_without_changing_cancel_command():
    backend = provider()
    submit(backend)
    token_generation = backend._nav_command_generation
    original_lease = backend._current_navigation_lease_locked()
    assert not backend.set_goal(goal("new-goal"))
    result = backend.take_goal_submission_result("new-goal")
    assert result["reason_code"] == "BUSY"
    assert backend._nav_command_generation == token_generation
    assert backend._current_navigation_lease_locked() == original_lease
    assert len(backend._stub.set_calls) == 1
    assert backend.cancel_goal(GOAL)


@pytest.mark.parametrize("path,planner", [(2, 8), (3, 7), (3, 9)])
def test_service_source_time_exception_does_not_relax_path_planner_fence(tmp_path, path, planner):
    backend = provider(wire("running", path=3, planner=8))
    navigation, view = service(backend, tmp_path)
    context = navigation.set_goal(goal())
    older = context.navigation_state.model_copy(deep=True)
    older.metadata.pop("command_projection_without_source")
    older.metadata.update(path_revision=path, planner_runtime_generation=planner, source_timestamp_ns=100)
    older.updated_at = context.navigation_state.updated_at - timedelta(seconds=1)
    assert navigation._ingest_navigation_state(view, older) is context


def test_service_ordinary_rejection_still_clears_new_goal(tmp_path):
    backend = provider(wire("failed", reason="ORDINARY_REJECTED", timestamp=100))
    navigation, view = service(backend, tmp_path)
    with pytest.raises(GatewayError) as caught:
        navigation.set_goal(goal())
    assert caught.value.error_code == "navigation_goal_rejected"
    assert navigation._current_goal is None
    assert navigation._state_machine.current_status == NavigationStatus.FAILED


def test_service_owned_unproved_terminal_does_not_clear_current_goal(tmp_path):
    backend = provider(wire("running"))
    navigation, view = service(backend, tmp_path)
    requested = goal()
    navigation.set_goal(requested)
    assert stream(backend, wire("failed", reason="OBSERVED", timestamp=100))
    pending = navigation._ingest_navigation_state(view, snapshot(backend))
    assert pending.navigation_state.status == NavigationStatus.UNCONFIRMED
    assert navigation._current_goal is requested
    assert stream(backend, wire("failed", reason="COMPLETE", timestamp=101, proof=True))
    resolved = navigation._ingest_navigation_state(view, snapshot(backend))
    assert resolved.navigation_state.status == NavigationStatus.FAILED
    assert resolved.current_goal is requested
    assert navigation._current_goal is None


@pytest.mark.parametrize("unary_status,reason", [("UNCONFIRMED", PENDING), ("running", ORIGIN)])
@pytest.mark.parametrize("bad_field,bad_value", [("path_revision", 1), ("planner_runtime_generation", 9), ("timestamp_ns", 99)])
def test_complete_precommit_receipt_cannot_bypass_current_unary_path_planner_source(unary_status, reason, bad_field, bad_value):
    backend = provider(wire(unary_status, reason=reason, path=2, planner=8, timestamp=100))
    receipt = wire("failed", reason="STALE_FULL_RECEIPT", path=2, planner=8, timestamp=101, proof=True)
    setattr(receipt, bad_field, bad_value)
    precommit(backend, receipt)
    accepted, result = submit(backend)
    assert result["disposition"] != "resolved"
    assert result["ownership_retained"]
    assert result["navigation"]["zero_ack_id"] == ""
    assert result["navigation"]["path_revision"] == 2
    assert result["navigation"]["planner_runtime_generation"] == 8
    assert accepted is (unary_status == "running")


@pytest.mark.parametrize("unary_status,reason", [("UNCONFIRMED", PENDING), ("running", ORIGIN)])
@pytest.mark.parametrize("receipt_path,receipt_planner", [(2, 8), (3, 8), (3, 7)])
def test_current_or_newer_path_complete_precommit_receipt_resolves(unary_status, reason, receipt_path, receipt_planner):
    backend = provider(wire(unary_status, reason=reason, path=2, planner=8, timestamp=100))
    receipt = wire("failed", reason="CURRENT_FULL_RECEIPT", path=receipt_path, planner=receipt_planner, timestamp=101, proof=True)
    precommit(backend, receipt)
    accepted, result = submit(backend)
    assert not accepted and result["disposition"] == "resolved"
    assert not result["ownership_retained"]
    for key, value in module._navigation_wire_document(receipt).items():
        assert result["navigation"][key] == value


@pytest.mark.parametrize("real_status", ["running", "succeeded", "failed", "cancelled"])
def test_service_real_callback_after_commit_beats_old_owned_unconfirmed_result(tmp_path, real_status):
    backend = provider()
    navigation, view = service(backend, tmp_path)
    take = view.take_goal_submission_result
    def deliver_before_service_consumes(goal_id):
        result = take(goal_id)
        assert result["disposition"] == "owned_unconfirmed"
        assert stream(backend, wire(real_status, reason="REAL_AFTER_COMMIT", timestamp=100,
                                    proof=real_status != "running"))
        navigation._ingest_navigation_state(view, snapshot(backend))
        return result
    view.take_goal_submission_result = deliver_before_service_consumes
    context = navigation.set_goal(goal())
    assert context.navigation_state.status.value == real_status
    assert context.navigation_state.metadata["reason_code"] == "REAL_AFTER_COMMIT"
    if real_status == "running":
        assert context.navigation_state.metadata["ownership_retained"]
        assert navigation._current_goal is not None
    else:
        assert context.navigation_state.metadata["zero_ack_id"] == "fixture-exact-zero"
        assert navigation._current_goal is None


def test_service_unproved_terminal_after_commit_reports_selected_proof_pending(tmp_path):
    backend = provider()
    navigation, view = service(backend, tmp_path)
    take = view.take_goal_submission_result
    def deliver_before_service_consumes(goal_id):
        result = take(goal_id)
        assert stream(backend, wire("failed", reason="REAL_FAILED", timestamp=100))
        navigation._ingest_navigation_state(view, snapshot(backend))
        return result
    view.take_goal_submission_result = deliver_before_service_consumes
    with pytest.raises(GatewayError) as caught:
        navigation.set_goal(goal())
    assert caught.value.error_code == "navigation_dispatch_unconfirmed"
    assert caught.value.details["reason_code"] == PENDING
    assert caught.value.details["navigation"]["observed_status"] == "failed"
    assert navigation._current_goal is not None


def test_service_connection_change_after_commit_cannot_restore_current_authority(tmp_path):
    backend = provider()
    navigation, view = service(backend, tmp_path)
    take = view.take_goal_submission_result
    def replace_before_service_consumes(goal_id):
        result = take(goal_id)
        backend._transition_navigation_connection(connected=True, message="replacement")
        backend._stub = Stub()
        navigation._ingest_navigation_state(view, snapshot(backend))
        return result
    view.take_goal_submission_result = replace_before_service_consumes
    with pytest.raises(GatewayError) as caught:
        navigation.set_goal(goal())
    assert caught.value.error_code == "navigation_dispatch_unconfirmed"
    assert caught.value.details["authority_current"] is False
    assert caught.value.details["navigation"]["historical_navigation_identity"]["goal_id"] == GOAL
    context = navigation._latest_navigation_context
    assert context.navigation_state.metadata["authority_current"] is False
    with pytest.raises(GatewayError) as cancel:
        navigation.cancel_goal(GOAL)
    assert cancel.value.error_code == "navigation_control_authority_unavailable"
    assert not backend._stub.cancel_calls


def test_foreign_exploration_overlay_and_idle_cannot_erase_owned_public_goal():
    backend = provider()
    submit(backend)
    foreign = wire("running", goal_id="exploration.nav.child", reason="FOREIGN", timestamp=100)
    foreign.owner_kind = pb2.NAVIGATION_OWNER_KIND_EXPLORATION_CHILD
    foreign.parent_request_id = "exploration"
    assert not stream(backend, foreign)
    assert backend._nav_exploration_overlay is None
    assert not stream(backend, pb2.NavigationState(status="idle"))
    assert snapshot(backend).current_goal_id == GOAL
    assert snapshot(backend).metadata["ownership_retained"]


def test_trusted_public_unary_clears_only_stale_display_overlay():
    backend = provider()
    stale = wire("failed", goal_id="exploration.nav.old", reason="OLD_OVERLAY", timestamp=50)
    stale.owner_kind = pb2.NAVIGATION_OWNER_KIND_EXPLORATION_CHILD
    stale.parent_request_id = "exploration"
    backend._nav_exploration_overlay = stale
    backend._nav_exploration_overlay_terminal = True
    submit(backend)
    state = backend._build_navigation_state_snapshot(include_readiness=False)
    assert state.current_goal_id == GOAL and state.status == NavigationStatus.UNCONFIRMED
    assert backend._nav_exploration_overlay is None


@pytest.mark.parametrize("staged_planner", [7, 9])
def test_same_path_different_planner_precommit_cannot_replace_terminal_gap(staged_planner):
    backend = provider(wire(reason=PENDING, path=3, planner=8))
    precommit(backend, wire("running", reason="OTHER_PLANNER", path=3,
                           planner=staged_planner, timestamp=100))
    accepted, result = submit(backend)
    assert not accepted and result["disposition"] == "owned_unconfirmed"
    assert result["navigation"]["path_revision"] == 3
    assert result["navigation"]["planner_runtime_generation"] == 8
    assert result["navigation"]["dispatch_obligation"]["terminal_observed"]


def test_service_newer_path_lower_numeric_planner_accepts_real_source(tmp_path):
    backend = provider(wire("running", path=3, planner=8))
    navigation, view = service(backend, tmp_path)
    initial = navigation.set_goal(goal()).navigation_state
    assert initial.metadata["command_projection_without_source"]
    assert stream(backend, wire("running", reason="NEW_PATH_NEW_PLANNER", path=4, planner=7, timestamp=100))
    context = navigation._ingest_navigation_state(view, snapshot(backend))
    assert context.navigation_state.updated_at < initial.updated_at
    assert context.navigation_state.status == NavigationStatus.RUNNING
    assert context.navigation_state.metadata["path_revision"] == 4
    assert context.navigation_state.metadata["planner_runtime_generation"] == 7
    assert context.navigation_state.metadata["ownership_retained"]


@pytest.mark.parametrize("lightweight", [False, True])
@pytest.mark.parametrize("replace_epoch", [False, True])
def test_resolved_command_returns_own_receipt_when_new_owner_wins_service_context(tmp_path, lightweight, replace_epoch):
    import time
    original_receipt = wire("failed", reason="ORIGINAL_COMMAND_RECEIPT", timestamp=100, proof=True)
    backend = provider(original_receipt)
    navigation, view = service(backend, tmp_path)
    original_goal = goal()
    newer_goal = goal("newer-goal")
    take = view.take_goal_submission_result
    selected = []
    def install_newer_before_old_command_consumed(goal_id):
        result = take(goal_id)
        assert result["disposition"] == "resolved"
        if replace_epoch:
            backend._transition_navigation_connection(connected=True, message="new runtime")
            backend._stub = Stub()
        backend._stub.response = wire("running", goal_id="newer-goal", reason="NEW_OWNER",
                                     timestamp=200, generation=3, producer=9 if replace_epoch else 7)
        assert backend.set_goal(newer_goal)
        backend.take_goal_submission_result("newer-goal")
        navigation._current_goal = newer_goal
        current = navigation._ingest_navigation_state(view, snapshot(backend), lightweight=lightweight)
        selected.append((current, len(navigation._navigation_history), navigation._state_machine.current_goal_id))
        return result
    view.take_goal_submission_result = install_newer_before_old_command_consumed
    returned = navigation.set_goal(original_goal,
        deadline_monotonic=time.monotonic() + 5 if lightweight else None)
    newer_context, history_length, machine_goal = selected[0]
    assert returned is not newer_context
    assert returned.current_goal is original_goal
    assert returned.navigation_state.current_goal_id == GOAL
    assert returned.navigation_state.status == NavigationStatus.FAILED
    assert returned.navigation_state.metadata["reason_code"] == "ORIGINAL_COMMAND_RECEIPT"
    assert returned.navigation_state.metadata["zero_ack_id"] == "fixture-exact-zero"
    assert returned.navigation_state.metadata["source_timestamp_ns"] == 100
    assert returned.navigation_state.metadata["authority_current"] is False
    assert returned.metadata["command_receipt_only"]
    assert returned.metadata["provider_version"] == view.provider_version
    if not lightweight:
        assert returned.metadata["diagnostics"]["goal_id"] == GOAL
        assert returned.metadata["navigation_state_machine"]["goal_id"] == GOAL
    assert navigation._latest_navigation_context is newer_context
    assert len(navigation._navigation_history) == history_length
    assert navigation._current_goal is newer_goal
    assert navigation._state_machine.current_goal_id == machine_goal == "newer-goal"
    assert backend._current_navigation_lease_locked()[0] == "newer-goal"
    assert not backend._stub.cancel_calls


# Performance-boundary controls: deterministic call counts and ownership isolation,
# not wall-time benchmarks. The pre-existing controls above remain byte-for-byte intact.

def _perf_submit(backend, transport):
    if transport == "cloud":
        backend._stub.cloud_response = cloud_reply(backend._stub.response)
        return cloud_submit(backend)
    return submit(backend)[1]


@pytest.mark.parametrize("transport", ["set", "cloud"])
@pytest.mark.parametrize("status,reason,proof,disposition", [
    ("accepted", "ACCEPTED", False, "confirmed"),
    ("running", "REAL_RUNNING", False, "confirmed"),
    ("UNCONFIRMED", ORIGIN, False, "owned_unconfirmed"),
    ("UNCONFIRMED", PENDING, False, "owned_unconfirmed"),
    ("succeeded", "ORIGINAL_RECEIPT", True, "resolved"),
    ("failed", "ORDINARY_REJECTION", False, "rejected"),
])
def test_submission_materializes_only_cache_and_returned_paths(transport, status, reason, proof, disposition):
    body = wire(status, reason=reason, proof=proof, timestamp=100 if proof else 0)
    backend = provider(body)
    with patch.object(module, "_navigation_state_typed_metadata", wraps=module._navigation_state_typed_metadata) as materialize, \
         patch.object(module, "_navigation_wire_document", wraps=module._navigation_wire_document) as wire_document:
        result = _perf_submit(backend, transport)
    print("PERF_NAV submission transport=%s disposition=%s typed_metadata=%d wire=%d" %
          (transport, disposition, materialize.call_count, wire_document.call_count))
    assert result["disposition"] == disposition
    assert wire_document.call_count == 1
    assert materialize.call_count == (1 if disposition == "rejected" else 2)
    assert result["navigation"]["planned_path"] == [{"x": 1.0, "y": 2.0, "z": 0.0}]
    assert result["navigation"]["local_path"] == [{"x": 2.0, "y": 3.0, "z": 0.0}]
    assert backend._nav_set_transaction_context.result is None
    if disposition == "resolved":
        assert result["navigation"]["reason_code"] == "ORIGINAL_RECEIPT"
        assert result["navigation"]["zero_ack_id"] == body.zero_ack_id
        assert result["navigation"]["timestamp_ns"] == 100


@pytest.mark.parametrize("transport", ["set", "cloud"])
def test_stale_submission_materializes_only_rejected_wire(transport):
    backend = provider()
    backend._stub.before_set_return = lambda: backend._transition_navigation_connection(
        connected=True, message="performance control epoch replacement")
    with patch.object(module, "_navigation_state_typed_metadata", wraps=module._navigation_state_typed_metadata) as materialize:
        result = _perf_submit(backend, transport)
    print("PERF_NAV stale transport=%s typed_metadata=%d" % (transport, materialize.call_count))
    assert result["disposition"] == "rejected"
    assert materialize.call_count == 1
    assert not result["ownership_retained"] and not result["authority_current"]
    assert backend._nav_set_transaction_context.result is None


@pytest.mark.parametrize("transport", ["set", "cloud"])
@pytest.mark.parametrize("resolved", [False, True])
def test_precommit_materializes_only_selected_cache_and_result(transport, resolved):
    backend = provider()
    selected = (wire("failed", reason="EXACT_RECEIPT", timestamp=101, proof=True)
                if resolved else wire("running", reason="REAL_RUNNING", timestamp=100))
    if resolved:
        precommit(backend, wire("failed", reason="TERMINAL_SEEN", timestamp=100), selected)
    else:
        precommit(backend, selected)
    with patch.object(module, "_navigation_state_typed_metadata", wraps=module._navigation_state_typed_metadata) as materialize:
        result = _perf_submit(backend, transport)
    print("PERF_NAV precommit transport=%s resolved=%s typed_metadata=%d" %
          (transport, resolved, materialize.call_count))
    assert materialize.call_count == 2
    assert all(call.args[0] == selected for call in materialize.call_args_list)
    assert result["disposition"] == ("resolved" if resolved else "confirmed")
    assert result["navigation"]["timestamp_ns"] == selected.timestamp_ns
    assert result["navigation"]["reason_code"] == selected.reason_code
    assert not backend._nav_set_precommit_candidates


@pytest.mark.parametrize("change,expected", [
    (None, True), ("path_revision", False), ("planner_runtime_generation", False),
    ("timestamp_ns", False), ("zero_ack_id", False),
])
def test_precommit_proof_predicate_never_materializes_paths(change, expected):
    observed = wire("failed", reason="TERMINAL_SEEN", timestamp=100)
    receipt = wire("failed", reason="EXACT_RECEIPT", timestamp=101, proof=True)
    if change is not None:
        setattr(receipt, change, {
            "path_revision": 2, "planner_runtime_generation": 9,
            "timestamp_ns": 99, "zero_ack_id": "",
        }[change])
    with patch.object(module, "_navigation_state_typed_metadata", wraps=module._navigation_state_typed_metadata) as materialize:
        result = module._navigation_precommit_proof_upgrade(observed, receipt)
    print("PERF_NAV proof_predicate change=%s typed_metadata=%d" % (change, materialize.call_count))
    assert result is expected
    assert materialize.call_count == 0


@pytest.mark.parametrize("transport", ["set", "cloud"])
@pytest.mark.parametrize("staged", [False, True])
@pytest.mark.parametrize("resolved", [False, True])
def test_submission_result_mutation_cannot_reach_cache_precommit_or_receipt(transport, staged, resolved):
    body = (wire("failed", reason="EXACT_RECEIPT", timestamp=101, proof=True)
            if resolved and not staged else wire())
    backend = provider(body)
    inputs = [body]
    held_precommit = []
    if staged:
        update = (wire("failed", reason="EXACT_RECEIPT", timestamp=101, proof=True)
                  if resolved else wire("running", reason="REAL_RUNNING", timestamp=100))
        inputs.append(update)
        backend._ensure_nav_state_stream()
        def stage_and_retain_fixture_reference():
            stream(backend, update)
            held_precommit.extend(backend._nav_set_precommit_candidates.values())
        backend._stub.before_set_return = stage_and_retain_fixture_reference
    # Exercise a nontrivial returned tree without turning this into a timing benchmark.
    for item in inputs:
        item.ClearField("planned_path")
        item.planned_path.extend(pb2.Vec3(x=index, y=index + 1, z=0) for index in range(2048))
        item.ClearField("local_path")
        item.local_path.extend(pb2.Vec3(x=index, y=index + 1, z=0) for index in range(256))
    original_inputs = copy.deepcopy(inputs)
    result = _perf_submit(backend, transport)
    cache_before = copy.deepcopy(backend._last_nav_metadata)
    precommit_before = copy.deepcopy(held_precommit)
    navigation = result["navigation"]
    assert navigation["planned_path"] is not backend._last_nav_metadata["planned_path"]
    assert navigation["planned_path"][0] is not backend._last_nav_metadata["planned_path"][0]
    assert navigation["local_path"] is not backend._last_nav_metadata["local_path"]
    navigation["planned_path"][0]["x"] = -901
    navigation["planned_path"].append({"x": -902, "y": -903, "z": -904})
    navigation["local_path"][0]["y"] = -905
    navigation["local_path"].clear()
    if not resolved:
        obligation = navigation["dispatch_obligation"]
        assert obligation is not backend._last_nav_metadata["dispatch_obligation"]
        assert obligation["identity"] is not backend._last_nav_metadata["dispatch_obligation"]["identity"]
        obligation["identity"]["goal_generation"] = 999
        obligation["terminal_observed"] = True
        obligation["origin_reason_code"] = "MUTATED_BY_CALLER"
    navigation["status"] = "caller_mutation"
    navigation["reason_code"] = "MUTATED_BY_CALLER"
    result["goal_id"] = "caller-mutated-goal"
    result["disposition"] = "caller_mutation"
    assert backend._last_nav_metadata == cache_before
    assert inputs == original_inputs
    assert held_precommit == precommit_before
    assert not backend._nav_set_precommit_candidates
    assert backend._nav_set_transaction_context.result is None
    assert backend.take_goal_submission_result(GOAL) is None


@pytest.mark.parametrize("status,proof", [("accepted", False), ("UNCONFIRMED", False), ("failed", True)])
def test_take_submission_result_transfers_detached_object_once(status, proof):
    backend = provider(wire(status, proof=proof))
    backend.set_goal(goal())
    original = backend._nav_set_transaction_context.result
    assert isinstance(original, dict)
    with patch.object(module.copy, "deepcopy", wraps=module.copy.deepcopy) as copies:
        result = backend.take_goal_submission_result(GOAL)
    root_copies = sum(call.args[0] is original for call in copies.call_args_list)
    print("PERF_NAV take status=%s proof=%s result_root_deepcopies=%d same_object=%s" %
          (status, proof, root_copies, result is original))
    assert root_copies == 0
    assert result is original
    assert backend._nav_set_transaction_context.result is None
    assert backend.take_goal_submission_result(GOAL) is None


def test_take_wrong_goal_discards_detached_submission_result():
    backend = provider()
    backend.set_goal(goal())
    assert backend.take_goal_submission_result("foreign-goal") is None
    assert backend._nav_set_transaction_context.result is None
    assert backend.take_goal_submission_result(GOAL) is None


@pytest.mark.parametrize("exit_kind", ["busy", "missing_body", "wrong_query", "exception"])
def test_cloud_direct_return_retires_submission_slot_on_every_exit(exit_kind):
    backend = provider()
    if exit_kind == "busy":
        assert not backend.set_goal(goal())
    elif exit_kind == "missing_body":
        backend._stub.cloud_response = cloud_reply(None)
    elif exit_kind == "wrong_query":
        backend._stub.cloud_response = cloud_reply(wire(), query_request_id="foreign-query")
    else:
        def fail_rpc():
            raise RuntimeError("fixture cloud failure")
        backend._stub.before_set_return = fail_rpc
    if exit_kind == "exception":
        with pytest.raises(RuntimeError, match="fixture cloud failure"):
            cloud_submit(backend)
    else:
        result = cloud_submit(backend)
        assert result["status"] == ("BUSY" if exit_kind == "busy" else "ADMISSION_REJECTED")
    assert backend._nav_set_transaction_context.result is None
    assert backend._nav_set_transaction_context.active is False
    assert backend.take_goal_submission_result(GOAL) is None


@pytest.mark.parametrize("lightweight", [False, True])
@pytest.mark.parametrize("status,reason,owned", [
    ("accepted", "ACCEPTED", False), ("running", "REAL_RUNNING", False),
    ("running", ORIGIN, True),
])
def test_service_confirmed_uses_one_refresh_ingest_and_two_path_materializations(tmp_path, lightweight, status, reason, owned):
    import time
    backend = provider(wire(status, reason=reason))
    navigation, view = service(backend, tmp_path)
    with patch.object(navigation, "_ingest_navigation_state", wraps=navigation._ingest_navigation_state) as ingest, \
         patch.object(backend, "_build_navigation_state_snapshot", wraps=backend._build_navigation_state_snapshot) as build_snapshot, \
         patch.object(module, "_navigation_state_typed_metadata", wraps=module._navigation_state_typed_metadata) as materialize:
        context = navigation.set_goal(goal(),
            deadline_monotonic=time.monotonic() + 5 if lightweight else None)
    print("PERF_NAV service status=%s owned=%s lightweight=%s ingest=%d snapshot=%d typed_metadata=%d" %
          (status, owned, lightweight, ingest.call_count, build_snapshot.call_count, materialize.call_count))
    assert ingest.call_count == 1
    assert build_snapshot.call_count == 1
    assert materialize.call_count == 2
    assert context.navigation_state.status.value == status
    assert context.navigation_state.current_goal_id == GOAL
    assert context.navigation_state.metadata["ownership_retained"] is owned
    assert context.navigation_state.metadata["authority_current"] is True
    assert context.navigation_state.metadata["planned_path"] == [{"x": 1.0, "y": 2.0, "z": 0.0}]
    assert context is navigation._latest_navigation_context
    if owned:
        assert context.navigation_state.metadata["command_projection_without_source"] is True
        assert context.navigation_state.metadata["dispatch_obligation"]["origin_reason_code"] == ORIGIN
        assert stream(backend, wire("failed", reason="TERMINAL_WITHOUT_ZERO", timestamp=100))
        pending = navigation._ingest_navigation_state(view, snapshot(backend), lightweight=lightweight)
        assert pending.navigation_state.status == NavigationStatus.UNCONFIRMED
        assert pending.navigation_state.metadata["ownership_retained"] is True
        assert pending.navigation_state.metadata["dispatch_obligation"]["terminal_observed"] is True
        assert navigation._current_goal.goal_id == GOAL


@pytest.mark.parametrize("lightweight", [False, True])
@pytest.mark.parametrize("resolved", [False, True])
def test_service_unknown_and_resolved_keep_command_ingest_without_refresh(tmp_path, lightweight, resolved):
    import time
    backend = provider(wire("failed", reason="EXACT_RECEIPT", timestamp=100, proof=True) if resolved else wire())
    navigation, view = service(backend, tmp_path)
    with patch.object(navigation, "_ingest_navigation_state", wraps=navigation._ingest_navigation_state) as ingest, \
         patch.object(navigation, "refresh_navigation", wraps=navigation.refresh_navigation) as refresh:
        if resolved:
            context = navigation.set_goal(goal(),
                deadline_monotonic=time.monotonic() + 5 if lightweight else None)
            assert context.navigation_state.status == NavigationStatus.FAILED
            assert context.navigation_state.metadata["zero_ack_id"] == "fixture-exact-zero"
        else:
            with pytest.raises(GatewayError) as caught:
                navigation.set_goal(goal(),
                    deadline_monotonic=time.monotonic() + 5 if lightweight else None)
            assert caught.value.error_code == "navigation_dispatch_unconfirmed"
            assert navigation._latest_navigation_context.navigation_state.metadata["ownership_retained"]
    print("PERF_NAV command_projection resolved=%s lightweight=%s ingest=%d refresh=%d" %
          (resolved, lightweight, ingest.call_count, refresh.call_count))
    assert ingest.call_count == 1
    assert refresh.call_count == 0


@pytest.mark.parametrize("outer_transport", ["set", "cloud"])
def test_nested_cloud_active_guard_does_not_clear_outer_submission_slot(outer_transport):
    backend = provider()
    seen_outer_slots = []
    def reject_nested_cloud_before_entering_cleanup():
        context = backend._nav_set_transaction_context
        outer_slot = context.result
        assert context.active is True
        with pytest.raises(RuntimeError, match="reentrant_set_goal"):
            cloud_submit(backend)
        assert context.result is outer_slot
        assert context.active is True
        seen_outer_slots.append(outer_slot)
    backend._stub.before_set_return = reject_nested_cloud_before_entering_cleanup
    result = _perf_submit(backend, outer_transport)
    assert len(seen_outer_slots) == 1
    assert result["disposition"] == "owned_unconfirmed"
    assert result["navigation"]["dispatch_obligation"]["origin_reason_code"] == ORIGIN
    assert len(backend._stub.cloud_calls) == (1 if outer_transport == "cloud" else 0)
    assert backend._nav_set_transaction_context.active is False
    assert backend._nav_set_transaction_context.result is None


@pytest.mark.parametrize("transport", ["set", "cloud"])
@pytest.mark.parametrize("stale_epoch", [False, True])
def test_rejected_wire_materializes_after_delivery_command_and_state_unlock(transport, stale_epoch):
    import threading
    body = wire() if stale_epoch else wire("failed", reason="ORDINARY_REJECTION")
    body.planned_path.extend(pb2.Vec3(x=index, y=index + 1, z=0) for index in range(2047))
    backend = provider(body)
    if stale_epoch:
        backend._stub.before_set_return = lambda: backend._transition_navigation_connection(
            connected=True, message="lock boundary control epoch replacement")
    observations = []
    original_wire_document = module._navigation_wire_document
    def observe_locks_before_materialization(update):
        observation = {"locks": {}, "errors": []}
        probe_done = threading.Event()
        def probe_from_another_thread():
            try:
                # The outer Set-only transaction lock intentionally remains held.
                for name, lock in (
                    ("delivery", backend._nav_delivery_lock),
                    ("command", backend._nav_command_lock),
                    ("state", backend._nav_state_lock),
                ):
                    acquired = lock.acquire(blocking=False)
                    try:
                        observation["locks"][name] = acquired
                    finally:
                        if acquired:
                            lock.release()
            except Exception as exc:
                observation["errors"].append(repr(exc))
            finally:
                probe_done.set()
        observer = threading.Thread(target=probe_from_another_thread)
        observer.start()
        try:
            # These bounds only prevent a stuck fixture; neither measures performance.
            observation["completed"] = probe_done.wait(timeout=5)
        finally:
            observer.join(timeout=5)
        observation["observer_alive"] = observer.is_alive()
        observations.append(observation)
        return original_wire_document(update)
    with patch.object(module, "_navigation_wire_document", side_effect=observe_locks_before_materialization):
        result = _perf_submit(backend, transport)
    print("PERF_NAV rejected_lock_boundary transport=%s stale_epoch=%s observations=%r" %
          (transport, stale_epoch, observations))
    assert result["disposition"] == "rejected"
    assert result["navigation"]["goal_id"] == body.goal_id
    assert result["navigation"]["reason_code"] == body.reason_code
    assert len(result["navigation"]["planned_path"]) == 2048
    assert result["navigation"]["planned_path"][0] == {"x": 1.0, "y": 2.0, "z": 0.0}
    assert len(observations) == 1
    observed = observations[0]
    assert observed["completed"] and not observed["observer_alive"] and not observed["errors"]
    assert observed["locks"] == {"delivery": True, "command": True, "state": True}
    assert not result["ownership_retained"] and not result["authority_current"]
    assert not backend._nav_set_precommit_candidates
    assert backend._nav_inflight_command_token is None
    assert backend._nav_set_transaction_context.result is None


@pytest.mark.parametrize("delivery", ["unary", "precommit", "stream"])
def test_unleased_guard_rejection_releases_obligation_without_zero_proof(delivery):
    rejection = wire("failed", reason="LEASE_NOT_GRANTED", timestamp=100, path=0, planner=0)
    rejection.message = "LEASE_NOT_GRANTED: STALE_STOP_GENERATION"
    backend = provider(rejection if delivery == "unary" else wire())
    if delivery == "precommit":
        precommit(backend, rejection)
    accepted, result = submit(backend)
    if delivery == "stream":
        assert not accepted and result["ownership_retained"]
        assert stream(backend, rejection)
    else:
        assert not accepted and result["disposition"] == "resolved"
        assert result["reason_code"] == "LEASE_NOT_GRANTED"
    state = snapshot(backend)
    assert state.status == NavigationStatus.FAILED
    assert state.metadata["reason_code"] == "LEASE_NOT_GRANTED"
    assert state.metadata["lease_not_granted"]
    assert not state.metadata["ownership_retained"]
    assert not state.metadata["actuator_zeroed"]
    assert not state.metadata["zero_ack_id"]
    assert state.metadata["zero_applied_monotonic_ns"] == 0
    assert "dispatch_obligation" not in state.metadata
    backend._stub.response = wire("accepted", goal_id="next-after-rejection", reason="ACCEPTED", path=0, planner=0, generation=3)
    assert backend.set_goal(goal("next-after-rejection"))
    assert len(backend._stub.set_calls) == 2


def test_unleased_rejection_retires_only_the_matching_goal():
    backend = provider()
    submit(backend)
    foreign = wire("failed", reason="LEASE_NOT_GRANTED", timestamp=100, path=0, planner=0,
                   goal_id="another-goal")
    assert not stream(backend, foreign)
    assert snapshot(backend).metadata["ownership_retained"]
    generic = wire("failed", reason="GUARD_ERROR", timestamp=100)
    generic.message = "LEASE_NOT_GRANTED: 文本不能代替原生具名结果"
    assert stream(backend, generic)
    assert snapshot(backend).metadata["ownership_retained"]
