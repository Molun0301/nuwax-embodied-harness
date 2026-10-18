"""Candidate-only controls for the typed dispatch responsibility public boundary.

These controls exercise real Gateway task/finalizer, candidate-loop, HTTP compact
projection, MCP/Agent wrappers, and SDK receipt handling with isolated endpoints.
They do not prove native publish, actuator behavior, or cross-process recovery.
"""
import json
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

from clients.robot_client import RobotClient
from contracts.geometry import Pose
from contracts.navigation import NavigationGoal, NavigationState, NavigationStatus
from contracts.runtime_views import NavigationContext
from core import StateStore
from core.errors import GatewayError
from core.event_bus import EventBus
from core.resource_lock import ResourceLockManager
from core.task_manager import TaskManager, TaskSafetyBarrierError
from gateways.agent.feedback import build_receipt, checked_json
from gateways.feedback import build_mcp_tool_result, build_public_error_payload, compact_public_data
from gateways.http.api import _navigation_state_payload
from gateways.runtime import GatewayRuntime
from scripts.check_navigation_map_snapshot_identity import OfflineProvider
from services.geometry.geometry_runtime_provider import GeometryRuntimeProvider
from services.geometry.proto_gen import geometry_runtime_pb2 as pb2
from services.navigation.service import NavigationService
from skills.map_workspace import MapWorkspaceService


IDENTITY = dict(goal_id="owned-goal", owner_kind=1, parent_request_id="owned-goal",
                goal_generation=7, producer_runtime_generation=11)


def wire(status="UNCONFIRMED", *, proof=False):
    return {**IDENTITY, "status": status, "message": "typed source result",
            "reason_code": "GOAL_PUBLISH_UNCONFIRMED", "timestamp_ns": 0,
            "path_revision": 3, "planner_runtime_generation": 13,
            "actuator_zeroed": proof, "zero_ack_id": "zero-exact" if proof else "",
            "zero_applied_monotonic_ns": 123 if proof else 0,
            "ownership_retained": not proof, "authority_current": True}


def navigation(status=NavigationStatus.UNCONFIRMED, *, proof=False, historical=False, goal_id="owned-goal"):
    identity = {**IDENTITY, "goal_id": goal_id, "parent_request_id": goal_id}
    metadata = {**identity, "observed_goal_id": goal_id,
                "reason_code": "GOAL_PUBLISH_UNCONFIRMED", "path_revision": 3,
                "planner_runtime_generation": 13, "ownership_retained": not proof,
                "authority_current": not historical, "actuator_zeroed": proof,
                "zero_ack_id": "zero-exact" if proof else "",
                "zero_applied_monotonic_ns": 123 if proof else 0}
    if not proof:
        metadata["dispatch_obligation"] = dict(origin_reason_code="GOAL_PUBLISH_UNCONFIRMED",
            connection_epoch=4, identity=identity, terminal_observed=False)
    if historical:
        metadata.update(owner_kind=0, parent_request_id="", goal_generation=0,
                        producer_runtime_generation=0, historical_navigation_identity=identity,
                        historical_navigation=wire())
    reached = status == NavigationStatus.SUCCEEDED
    return NavigationContext(backend_name="isolated", goal_reached=reached,
        current_goal=NavigationGoal(goal_id=goal_id, target_name="fixture"),
        navigation_state=NavigationState(current_goal_id=goal_id, status=status,
            goal_reached=reached, message="typed fixture", metadata=metadata))


def uncertain_error():
    return GatewayError("目标发送结果未确认", error_code="navigation_dispatch_unconfirmed",
        http_status=503, details={"goal_id": IDENTITY["goal_id"], "navigation": wire(),
            "reason_code": "GOAL_PUBLISH_UNCONFIRMED", "ownership_retained": True,
            "authority_current": True, "retryable": False})


class CancelControlProvider(OfflineProvider):
    """Service/control seam only; deliberately does not replace the production bundle."""

    def __init__(self, backend):
        super().__init__()
        self.is_available = Mock(return_value=True)
        self.get_navigation_state = Mock(side_effect=AssertionError("display read on control seam"))
        self.get_navigation_control_state = Mock(wraps=backend.get_navigation_control_state)
        self.cancel_goal = Mock(wraps=backend.cancel_goal)


class PublicUnconfirmedTests(unittest.TestCase):
    def gateway(self):
        runtime = GatewayRuntime.__new__(GatewayRuntime)
        runtime._run_with_motion_admission = lambda admission, operation: operation()
        runtime._require_motion_admission = lambda *args: NS(generation=2, boot_nonce=b"x" * 16,
                                                            guard_motion_stop_generation=4)
        return runtime

    def control_gateway(self):
        """Real service, provider control snapshot, epoch transition and exact Cancel RPC."""
        runtime = self.gateway()
        backend = GeometryRuntimeProvider(NS())
        response = pb2.NavigationState(**IDENTITY, status="UNCONFIRMED",
            reason_code="GOAL_PUBLISH_UNCONFIRMED", path_revision=3, planner_runtime_generation=13)
        backend._stub = NS(SetNavigationGoal=Mock(return_value=response),
            CancelNavigation=Mock(return_value=pb2.NavigationState(**IDENTITY, status="pending",
                reason_code="CANCEL_REQUESTED", path_revision=3, planner_runtime_generation=13)))
        backend._connected = True
        backend._nav_connection_epoch = 4
        def ensure_stream():
            if backend._nav_active_stream_token is None:
                backend._nav_stream_epoch += 1
                backend._nav_active_stream_token = (
                    backend._nav_connection_epoch, backend._nav_stream_epoch, id(backend._stub))
                backend._nav_stream_started = True
        backend._ensure_nav_state_stream = ensure_stream
        requested = NavigationGoal(goal_id=IDENTITY["goal_id"], target_pose=Pose(frame_id="map"),
            metadata={"motion_admission": {"generation": 1, "boot_nonce_hex": "01" * 16,
                                           "motion_stop_generation": 0}})
        self.assertFalse(backend.set_goal(requested))
        submission = backend.take_goal_submission_result(IDENTITY["goal_id"])
        self.assertEqual(submission["disposition"], "owned_unconfirmed")
        self.assertTrue(submission["authority_current"])
        view = CancelControlProvider(backend)
        service = NavigationService(provider_owner=NS(providers=view), state_store=StateStore(),
            mapping_service=NS(get_latest_snapshot=Mock(side_effect=AssertionError("map aggregation"))),
            localization_service=NS(get_active_session=Mock(side_effect=AssertionError("localization aggregation"))),
            forbidden_zone_store=NS())
        service._current_goal = requested
        service.refresh_navigation = Mock(side_effect=AssertionError("full refresh before exact cancel"))
        service._ingest_navigation_state = Mock(side_effect=AssertionError("display aggregation"))
        service.get_navigation_control_context = Mock(wraps=service.get_navigation_control_context)
        service.cancel_goal = Mock(wraps=service.cancel_goal)
        backend.get_navigation_readiness = Mock(side_effect=AssertionError("readiness on control seam"))
        backend.get_current_pose = Mock(side_effect=AssertionError("pose on control seam"))
        runtime.navigation_service = service
        return runtime, backend, view

    def assert_control_reads_only(self, runtime, backend, view):
        runtime.navigation_service.refresh_navigation.assert_not_called()
        runtime.navigation_service._ingest_navigation_state.assert_not_called()
        runtime.navigation_service._mapping_service.get_latest_snapshot.assert_not_called()
        runtime.navigation_service._localization_service.get_active_session.assert_not_called()
        view.get_navigation_state.assert_not_called()
        backend.get_navigation_readiness.assert_not_called()
        backend.get_current_pose.assert_not_called()

    def task_gateway(self, *, stopped=True, submitted=None):
        runtime = self.gateway()
        bus, locks = EventBus(), ResourceLockManager()
        manager = TaskManager(event_bus=bus, resource_lock_manager=locks)
        self.addCleanup(bus.shutdown, 2)
        self.addCleanup(manager.shutdown, wait=True)
        runtime.event_bus, runtime.task_manager = bus, manager
        runtime.capability_registry = NS(get_descriptor=lambda _: NS(required_resources=["navigation"],
                                                                     display_name="isolated"))
        runtime._reconcile_navigation_quarantine = lambda: {"pending": False}
        runtime._goal_with_motion_admission = lambda goal, admission: goal
        runtime._ensure_motion_runtime_for_backend_call = lambda *args, **kwargs: None
        runtime._capture_task_generation_snapshot = lambda **kwargs: {}
        runtime._record_task_generation_snapshot = lambda *args: None
        runtime._task_generation_rollovers = lambda *args: {}
        runtime._estimate_navigation_progress = lambda context: .1
        runtime._build_task_request_metadata = lambda **kwargs: {}
        runtime.memory_service = NS(resolve_navigation_region=lambda goal: None)
        unknown, terminal = navigation(), navigation(NavigationStatus.CANCELLED, proof=True)
        error = uncertain_error()
        service = NS(set_goal=Mock(side_effect=error if submitted is None else None,
                                  return_value=submitted),
                     refresh_navigation=Mock(return_value=unknown if submitted is None else submitted),
                     cancel_goal=Mock(return_value=terminal if stopped else unknown))
        runtime.navigation_service = service
        return runtime, manager, locks, error

    def submit(self, runtime):
        return runtime._submit_navigation_task(capability_name="navigate_to_pose",
            goal=NavigationGoal(goal_id=IDENTITY["goal_id"], target_name="fixture"),
            motion_admission=object(), poll_interval_sec=.02, timeout_sec=2, requested_by="test")

    def test_typed_unknown_preserves_dispatch_and_real_finally_exact_cancel(self):
        runtime, manager, locks, error = self.task_gateway()
        error.details["backend_code"] = "PLANNER_NOT_READY"  # Typed uncertainty must not enter readiness retries.
        task = self.submit(runtime)
        with self.assertRaises(GatewayError) as raised:
            manager.wait_for_task(task.task_id, timeout=2)
        self.assertEqual(raised.exception.error_code, error.error_code)
        runtime.navigation_service.set_goal.assert_called_once()
        runtime.navigation_service.cancel_goal.assert_called_once()
        self.assertEqual(runtime.navigation_service.cancel_goal.call_args.kwargs["request_id"], IDENTITY["goal_id"])
        self.assertEqual(locks.held_resources(task.task_id), ())
        self.assertFalse(manager._records[task.task_id].resource_quarantined)

    def test_typed_unknown_missing_zero_uses_original_resource_quarantine(self):
        runtime, manager, locks, _ = self.task_gateway(stopped=False)
        with patch("gateways.runtime._EXPLORATION_STOP_BARRIER_TIMEOUT_SEC", 0):
            task = self.submit(runtime)
            with self.assertRaises(TaskSafetyBarrierError):
                manager.wait_for_task(task.task_id, timeout=2)
        self.assertTrue(manager._records[task.task_id].resource_quarantined)
        self.assertEqual(locks.held_resources(task.task_id), ("navigation",))
        runtime.navigation_service.set_goal.assert_called_once()
        runtime.navigation_service.cancel_goal.assert_called_once()

    def test_definite_rejection_does_not_gain_uncertain_responsibility(self):
        runtime, manager, _, _ = self.task_gateway()
        runtime.navigation_service.set_goal.side_effect = GatewayError("definitely rejected",
            error_code="navigation_goal_rejected", details={"backend_code": "INVALID_GOAL"})
        task = self.submit(runtime)
        with self.assertRaises(GatewayError) as raised:
            manager.wait_for_task(task.task_id, timeout=2)
        self.assertEqual(raised.exception.error_code, "navigation_goal_rejected")
        runtime.navigation_service.cancel_goal.assert_not_called()
        runtime.navigation_service.set_goal.assert_called_once()

    def test_resolved_receipt_skips_extra_cancel_and_retains_execution_failure(self):
        runtime, manager, _, _ = self.task_gateway(submitted=navigation(NavigationStatus.FAILED, proof=True))
        task = self.submit(runtime)
        with self.assertRaises(GatewayError) as raised:
            manager.wait_for_task(task.task_id, timeout=2)
        self.assertEqual(raised.exception.error_code, "navigation_failed")
        runtime.navigation_service.cancel_goal.assert_not_called()

    def test_real_running_from_uncertain_source_continues_without_automatic_cancel(self):
        running = navigation(NavigationStatus.RUNNING)
        runtime, manager, _, _ = self.task_gateway(submitted=running)
        terminal = navigation(NavigationStatus.SUCCEEDED, proof=True)
        runtime.navigation_service.refresh_navigation.side_effect = [running, terminal]
        task = self.submit(runtime)
        result = manager.wait_for_task(task.task_id, timeout=2)
        self.assertEqual(result["navigation_context"].navigation_state.status, NavigationStatus.SUCCEEDED)
        runtime.navigation_service.set_goal.assert_called_once()
        runtime.navigation_service.cancel_goal.assert_not_called()

    def test_unknown_is_cancel_intermediate_until_exact_zero(self):
        runtime = self.gateway()
        pending, terminal = navigation(), navigation(NavigationStatus.CANCELLED, proof=True)
        runtime.navigation_service = NS(refresh_navigation=Mock(side_effect=[pending, terminal]),
                                         cancel_goal=Mock(return_value=pending))
        task_context = NS(update=Mock(), acknowledge_interrupt_barrier=Mock())
        with patch("gateways.runtime.time.sleep", return_value=None):
            result = runtime._cancel_navigation_with_safety_barrier(task_context,
                goal_id=IDENTITY["goal_id"], poll_interval_sec=.02)
        self.assertIs(result, terminal)
        task_context.acknowledge_interrupt_barrier.assert_called_once()
        self.assertEqual(task_context.acknowledge_interrupt_barrier.call_args.args[0]["zero_ack_id"], "zero-exact")

    def test_ordinary_failed_or_cancelled_without_zero_does_not_relax_barrier(self):
        for status in (NavigationStatus.FAILED, NavigationStatus.CANCELLED):
            with self.subTest(status=status):
                runtime = self.gateway()
                unproven = navigation(status)
                unproven.navigation_state.metadata.pop("dispatch_obligation")
                unproven.navigation_state.metadata["ownership_retained"] = False
                runtime.navigation_service = NS(refresh_navigation=Mock(return_value=unproven),
                                                 cancel_goal=Mock(return_value=unproven))
                with self.assertRaises(TaskSafetyBarrierError):
                    runtime._cancel_navigation_with_safety_barrier(NS(update=Mock(), acknowledge_interrupt_barrier=Mock()),
                        goal_id=IDENTITY["goal_id"], poll_interval_sec=.02)

    def test_unknown_public_cancel_reports_pending_not_completed(self):
        runtime = self.gateway()
        runtime.navigation_service = NS(refresh_navigation=Mock(return_value=navigation()),
                                         cancel_goal=Mock(return_value=navigation()))
        result = runtime._handle_cancel_navigation({})
        self.assertTrue(result["cancellation_requested"])
        self.assertFalse(result["completed"])
        runtime.navigation_service.cancel_goal.assert_called_once_with(request_id=IDENTITY["goal_id"])

    def test_historical_owner_rejects_both_cancel_forms_without_sending(self):
        for args in ({}, {"request_id": IDENTITY["goal_id"]}):
            with self.subTest(args=args):
                runtime = self.gateway()
                runtime.navigation_service = NS(refresh_navigation=Mock(return_value=navigation(historical=True)),
                    get_navigation_control_context=Mock(return_value=navigation(historical=True)), cancel_goal=Mock())
                with self.assertRaises(GatewayError) as raised:
                    runtime._handle_cancel_navigation(args)
                self.assertEqual(raised.exception.error_code, "navigation_control_authority_unavailable")
                self.assertFalse(raised.exception.details["authority_current"])
                self.assertEqual(raised.exception.details["historical_navigation_identity"], IDENTITY)
                self.assertFalse(build_public_error_payload(raised.exception)["retryable"])
                runtime.navigation_service.cancel_goal.assert_not_called()
                if args:
                    runtime.navigation_service.get_navigation_control_context.assert_called_once_with()
                    runtime.navigation_service.refresh_navigation.assert_not_called()
                else:
                    runtime.navigation_service.refresh_navigation.assert_called_once_with()
                    runtime.navigation_service.get_navigation_control_context.assert_not_called()

    def test_explicit_cancel_uses_real_lightweight_control_and_exact_backend_request(self):
        for completed in (False, True):
            with self.subTest(completed=completed):
                runtime, backend, view = self.control_gateway()
                if completed:
                    backend._stub.CancelNavigation.return_value = pb2.NavigationState(**IDENTITY,
                        status="cancelled", reason_code="CANCELLED", path_revision=3,
                        planner_runtime_generation=13, timestamp_ns=100, actuator_zeroed=True,
                        zero_ack_id="zero-exact", zero_applied_monotonic_ns=123)
                result = runtime._handle_cancel_navigation({"request_id": "  owned-goal  "})
                self.assertTrue(result["cancellation_requested"])
                self.assertEqual(result["completed"], completed)
                self.assertEqual(result["request_id"], IDENTITY["goal_id"])
                runtime.navigation_service.cancel_goal.assert_called_once_with(request_id=IDENTITY["goal_id"])
                self.assertEqual(view.get_navigation_control_state.call_count, 3)
                backend._stub.CancelNavigation.assert_called_once()
                request = backend._stub.CancelNavigation.call_args[0][0]
                for key, expected in IDENTITY.items():
                    self.assertEqual(getattr(request, key), expected)
                self.assert_control_reads_only(runtime, backend, view)

    def test_explicit_historical_control_keeps_409_before_unavailable_checks(self):
        for available in (False, True):
            with self.subTest(available=available):
                runtime, backend, view = self.control_gateway()
                old_stub = backend._stub
                backend._transition_navigation_connection(connected=True, message="replacement epoch")
                backend._stub = NS(CancelNavigation=Mock())
                view.is_available.return_value = available
                with self.assertRaises(GatewayError) as raised:
                    runtime._handle_cancel_navigation({"request_id": IDENTITY["goal_id"]})
                error = raised.exception
                self.assertEqual(error.error_code, "navigation_control_authority_unavailable")
                self.assertEqual(error.http_status, 409)
                self.assertEqual(error.details["historical_navigation_identity"], IDENTITY)
                self.assertFalse(error.details["authority_current"])
                # Preserve the Gateway's full state shape, not just the historical wire.
                self.assertEqual(error.details["navigation"]["current_goal_id"], IDENTITY["goal_id"])
                self.assertEqual(error.details["navigation"]["metadata"]["historical_navigation_identity"], IDENTITY)
                public = build_public_error_payload(error)
                mcp = build_mcp_tool_result({"result": {"ok": False, "error_payload": public}})
                self.assertTrue(mcp["isError"])
                for projected in (public, mcp["structuredContent"]["result"]["error_payload"]):
                    self.assertEqual(projected["code"], error.error_code)
                    self.assertFalse(projected["retryable"])
                    self.assertEqual(projected["error_category"], "validation")
                    self.assertEqual(projected["details"], compact_public_data(error.details))
                runtime.navigation_service.cancel_goal.assert_not_called()
                view.is_available.assert_not_called()
                view.cancel_goal.assert_not_called()
                old_stub.CancelNavigation.assert_not_called()
                backend._stub.CancelNavigation.assert_not_called()
                self.assertEqual(view.get_navigation_control_state.call_count, 1)
                self.assert_control_reads_only(runtime, backend, view)

    def test_explicit_mismatch_keeps_real_service_409_and_public_mcp_error(self):
        runtime, backend, view = self.control_gateway()
        with self.assertRaises(GatewayError) as raised:
            runtime._handle_cancel_navigation({"request_id": "foreign-goal"})
        error = raised.exception
        self.assertEqual(error.error_code, "navigation_goal_mismatch")
        self.assertEqual(error.http_status, 409)
        self.assertEqual(error.details["requested_goal_id"], "foreign-goal")
        self.assertEqual(error.details["active_goal_id"], IDENTITY["goal_id"])
        public = build_public_error_payload(error)
        mcp = build_mcp_tool_result({"result": {"ok": False, "error_payload": public}})
        self.assertTrue(mcp["isError"])
        self.assertEqual(public["error_category"], "validation")
        self.assertEqual(mcp["structuredContent"]["result"]["error_payload"], public)
        view.cancel_goal.assert_not_called()
        backend._stub.CancelNavigation.assert_not_called()
        self.assert_control_reads_only(runtime, backend, view)

    def test_explicit_control_read_error_does_not_fallback_to_full_refresh_or_cancel(self):
        runtime, backend, view = self.control_gateway()
        error = GatewayError("control unavailable", error_code="navigation_goal_identity_unavailable", http_status=503)
        runtime.navigation_service.get_navigation_control_context.side_effect = error
        with self.assertRaises(GatewayError) as raised:
            runtime._handle_cancel_navigation({"request_id": IDENTITY["goal_id"]})
        self.assertIs(raised.exception, error)
        runtime.navigation_service.cancel_goal.assert_not_called()
        backend._stub.CancelNavigation.assert_not_called()
        self.assert_control_reads_only(runtime, backend, view)

    def test_noarg_and_blank_cancel_keep_original_refresh_contract(self):
        for args in ({}, {"request_id": "   "}):
            for active in (False, True):
                with self.subTest(args=args, active=active):
                    runtime = self.gateway()
                    context = navigation() if active else NavigationContext(backend_name="isolated",
                        navigation_state=NavigationState(status=NavigationStatus.IDLE))
                    runtime.navigation_service = NS(refresh_navigation=Mock(return_value=context),
                        get_navigation_control_context=Mock(side_effect=AssertionError("noarg control shortcut")),
                        cancel_goal=Mock(return_value=context))
                    result = runtime._handle_cancel_navigation(args)
                    runtime.navigation_service.refresh_navigation.assert_called_once_with()
                    runtime.navigation_service.get_navigation_control_context.assert_not_called()
                    self.assertEqual(result["cancellation_requested"], active)
                    self.assertEqual(result["completed"], not active)
                    if active:
                        runtime.navigation_service.cancel_goal.assert_called_once_with(request_id=IDENTITY["goal_id"])
                    else:
                        runtime.navigation_service.cancel_goal.assert_not_called()

    def candidates(self, *, semantic=False, ordinary=False):
        runtime = self.gateway()
        runtime._ensure_workspace_memory_library_ready = lambda **kwargs: None
        runtime._ensure_localization_ready = lambda **kwargs: None
        runtime._ensure_motion_runtime_for_backend_call = lambda *args, **kwargs: None
        runtime._goal_with_motion_admission = lambda goal, admission: goal
        runtime._build_goal_from_navigation_candidate = lambda **kwargs: NavigationGoal(
            goal_id=IDENTITY["goal_id"] if kwargs["candidate"].record_id == "candidate-1" else "owned-goal-2",
            target_name=kwargs["candidate"].record_id)
        candidates = [NS(record_id="candidate-1", record_kind=NS(value="object")),
                      NS(record_id="candidate-2", record_kind=NS(value="object"))]
        error = GatewayError("ordinary reject", error_code="navigation_goal_rejected") if ordinary else uncertain_error()
        terminal = navigation(NavigationStatus.SUCCEEDED, proof=True, goal_id="owned-goal-2")
        runtime.navigation_service = NS(set_goal=Mock(side_effect=[error, terminal] if ordinary else error),
            refresh_navigation=Mock(side_effect=[navigation(NavigationStatus.FAILED, proof=True), terminal] if ordinary else None,
                                    return_value=navigation()),
            cancel_goal=Mock(return_value=navigation(NavigationStatus.CANCELLED, proof=True)))
        runtime._build_target_navigation_verification = Mock(return_value={"arrival_verification": NS(verified=True)})
        runtime._estimate_navigation_progress = lambda context: .2
        search = dict(memory_candidates=[] if semantic else candidates,
            semantic_goal=NavigationGoal(goal_id=IDENTITY["goal_id"], target_name="semantic") if semantic else None,
            current_map_name="map", current_localization_session_id="session", workspace=object(),
            candidate_diagnostics=[dict(record_id=c.record_id, allowed_for_navigation=True) for c in candidates])
        rejected, lifecycle = set(), []
        def invoke():
            return runtime._run_target_navigation_candidates(task_context=NS(update=Mock(), ensure_active=Mock()),
                capability_name="navigate_to_named_location", motion_admission=object(), map_name="map", query="fixture",
                camera_id=None, requested_by="test", poll_interval_sec=.02, verify_similarity_threshold=.5,
                search_state=search, rejected_record_ids=rejected, allow_not_found=True,
                navigation_state_callback=lifecycle.append)
        return runtime, invoke, error, rejected, lifecycle

    def test_memory_unknown_parks_then_rethrows_without_second_goal_or_unreachable_budget(self):
        runtime, invoke, error, rejected, lifecycle = self.candidates()
        with self.assertRaises(GatewayError) as raised:
            invoke()
        self.assertIs(raised.exception, error)
        self.assertEqual(rejected, set())
        runtime.navigation_service.set_goal.assert_called_once()
        runtime.navigation_service.cancel_goal.assert_called_once()
        self.assertEqual(lifecycle, [IDENTITY["goal_id"], None])
        runtime._build_target_navigation_verification.assert_not_called()

    def test_semantic_unknown_cannot_be_swallowed_by_allow_not_found(self):
        runtime, invoke, error, rejected, _ = self.candidates(semantic=True)
        with self.assertRaises(GatewayError) as raised:
            invoke()
        self.assertIs(raised.exception, error)
        self.assertEqual(rejected, set())
        runtime.navigation_service.set_goal.assert_called_once()
        runtime.navigation_service.cancel_goal.assert_called_once()

    def test_ordinary_candidate_failure_keeps_existing_next_candidate_budget(self):
        runtime, invoke, _, rejected, _ = self.candidates(ordinary=True)
        result = invoke()
        self.assertEqual(result["navigation_candidate"].record_id, "candidate-2")
        self.assertEqual(rejected, {"candidate-1"})
        self.assertEqual(runtime.navigation_service.set_goal.call_count, 2)

    def test_http_unknown_and_historical_projection_never_invents_current_identity_or_proof(self):
        live = _navigation_state_payload(navigation())
        self.assertEqual(live["state"], "UNCONFIRMED")
        self.assertEqual(live["reason_code"], "GOAL_PUBLISH_UNCONFIRMED")
        self.assertEqual(live["current_navigation_identity"], IDENTITY)
        self.assertTrue(live["ownership_retained"])
        self.assertFalse(live["cancel_zero_ack_observed"])
        self.assertFalse(live["actuator_zeroed"])
        history = _navigation_state_payload(navigation(historical=True))
        self.assertIsNone(history["current_navigation_identity"])
        self.assertEqual(history["historical_navigation_identity"], IDENTITY)
        self.assertEqual(history["historical_navigation"], wire())
        self.assertFalse(history["authority_current"])
        self.assertFalse(history["cancel_zero_ack_observed"])

    def test_http_complete_identity_cannot_grant_missing_or_false_authority(self):
        for authority in (None, False):
            with self.subTest(authority=authority):
                context = navigation()
                if authority is None:
                    context.navigation_state.metadata.pop("authority_current")
                else:
                    context.navigation_state.metadata["authority_current"] = authority
                result = _navigation_state_payload(context)
                self.assertEqual(result["observed_navigation_identity"], IDENTITY)
                self.assertIsNone(result["current_navigation_identity"])
                self.assertIs(result["authority_current"], authority)
                self.assertFalse(result["cancel_zero_ack_observed"])

    def test_http_observed_terminal_is_diagnostic_only(self):
        context = navigation()
        context.navigation_state.metadata["dispatch_obligation"].update(terminal_observed=True, observed_status="FAILED")
        context.navigation_state.metadata["reason_code"] = "GOAL_TERMINAL_PROOF_PENDING"
        payload = _navigation_state_payload(context)
        self.assertEqual(payload["status"], "unconfirmed")
        self.assertEqual(payload["observed_status"], "FAILED")
        self.assertTrue(payload["terminal_observed"])
        self.assertFalse(payload["cancel_zero_ack_observed"])
        # The ordinary typed-CANCELLED compact API contract is unchanged.
        self.assertEqual(_navigation_state_payload(navigation(NavigationStatus.CANCELLED, proof=True))["state"], "CANCELED")

    def place_result(self, status="ADMISSION_UNCONFIRMED", nav=None):
        runtime = self.gateway()
        nav = wire() if nav is None else nav
        raw = dict(status=status, message="typed query result", query_request_id="query-1", navigation=nav,
                   accepted=status == "SUCCEEDED", ownership_retained=status == "ADMISSION_UNCONFIRMED",
                   authority_current=True)
        runtime.map_workspace_service = NS(submit_mapping_cloud_goal=Mock(return_value=raw))
        result = runtime._handle_navigate_to_place({"query_text": "fixture", "query_request_id": "query-1"})
        return raw, result

    def test_mapping_unknown_mcp_agent_sdk_keep_error_and_navigation(self):
        raw, result = self.place_result()
        self.assertFalse(result["ok"])
        self.assertFalse(result["accepted"])
        self.assertEqual(result["error_payload"]["code"], "navigation_dispatch_unconfirmed")
        self.assertFalse(result["error_payload"]["retryable"])
        self.assertIs(result["navigation"], raw["navigation"])
        mcp = build_mcp_tool_result({"result": result})
        self.assertTrue(mcp["isError"])
        self.assertEqual(json.loads(mcp["content"][0]["text"]), mcp["structuredContent"])
        self.assertEqual(mcp["structuredContent"]["result"]["navigation"], raw["navigation"])
        receipt = build_receipt(request_id="request-1", gateway_instance_id="gateway-1", tool_name="navigate_to_place",
            capability_name="navigate_to_place", response={"result": result}, delivery_state="recorded",
            full_result_path="/api/agent/calls/request-1/result")
        self.assertEqual(receipt["error"]["code"], "navigation_dispatch_unconfirmed")
        self.assertFalse(receipt["error"]["retryable"])
        self.assertEqual(receipt["result"]["navigation"], raw["navigation"])
        self.assertIs(RobotClient._receipt(receipt), receipt)
        self.assertEqual(checked_json(result)["navigation"], raw["navigation"])

    def test_mapping_rejection_without_navigation_does_not_invent_installed_ownership(self):
        runtime = self.gateway()
        runtime.map_workspace_service = NS(submit_mapping_cloud_goal=Mock(return_value={
            "status": "ADMISSION_REJECTED", "query_request_id": "query-1", "navigation": None}))
        result = runtime._handle_navigate_to_place({"query_text": "fixture", "query_request_id": "query-1"})
        self.assertEqual(result["error_payload"]["code"], "navigation_goal_rejected")
        self.assertIsNone(result["error_payload"]["details"]["navigation"])
        self.assertFalse(result["error_payload"]["details"]["ownership_retained"])
        self.assertFalse(result["error_payload"]["details"]["authority_current"])
        self.assertTrue(build_mcp_tool_result({"result": result})["isError"])

    def test_mapping_busy_and_resolved_receipts_are_not_successful_admission_rejection(self):
        for outer, expected in (("BUSY", "navigation_busy"), ("FAILED", "navigation_failed"),
                                ("CANCELLED", "navigation_cancelled")):
            with self.subTest(status=outer):
                nav = wire(outer, proof=outer != "BUSY")
                _, result = self.place_result(outer, nav)
                self.assertEqual(result["error_payload"]["code"], expected)
                self.assertEqual(result["navigation"], nav)
                self.assertTrue(build_mcp_tool_result({"result": result})["isError"])
        raw, success = self.place_result("SUCCEEDED", wire("RUNNING"))
        self.assertIs(success, raw)
        self.assertNotIn("error_payload", success)
        self.assertEqual(success["navigation"]["status"], "RUNNING")

    def test_authority_unavailable_is_narrowly_nonretryable_across_public_wrappers(self):
        runtime = self.gateway()
        runtime.navigation_service = NS(refresh_navigation=Mock(return_value=navigation(historical=True)),
                                         cancel_goal=Mock())
        with self.assertRaises(GatewayError) as raised:
            runtime._handle_cancel_navigation({})
        error = raised.exception
        self.assertEqual(error.error_code, "navigation_control_authority_unavailable")
        self.assertEqual(error.http_status, 409)
        runtime.navigation_service.cancel_goal.assert_not_called()
        expected = {"goal_id": IDENTITY["goal_id"], "historical_navigation_identity": IDENTITY,
                    "ownership_retained": True, "authority_current": False, "retryable": False}
        for key, value in expected.items():
            self.assertEqual(error.details[key], value)
        public = build_public_error_payload(error)
        self.assertFalse(public["retryable"])
        self.assertEqual(public["error_category"], "validation")
        wrapped = {"error_payload": public, "ok": False}
        mcp = build_mcp_tool_result({"result": wrapped})
        self.assertTrue(mcp["isError"])
        mcp_error = mcp["structuredContent"]["result"]["error_payload"]
        receipt = build_receipt(request_id="history-request", gateway_instance_id="gateway-1",
            tool_name="cancel_navigation", capability_name="cancel_navigation", response={"result": wrapped},
            delivery_state="recorded", full_result_path="/api/agent/calls/history-request/result")
        # Real handler details include a larger navigation body subject to the
        # existing bounded projection. Identity and authority remain exact at every exit.
        for projected in (public, mcp_error, receipt["error"]):
            self.assertFalse(projected["retryable"])
            for key, value in expected.items():
                self.assertEqual(projected["details"][key], value)
        ordinary = build_public_error_payload(GatewayError("ordinary unavailable",
            error_code="runtime_unavailable", http_status=503,
            details={"private_diagnostic": "must remain redacted"}))
        self.assertTrue(ordinary["retryable"])
        self.assertEqual(ordinary["error_category"], "internal")
        self.assertNotIn("details", ordinary)

    def test_map_workspace_skill_passes_full_typed_navigation_unchanged(self):
        response = dict(status="ADMISSION_UNCONFIRMED", navigation=wire(), ownership_retained=True,
                        authority_current=True, reason_code="GOAL_PUBLISH_UNCONFIRMED", retryable=False)
        provider = NS(get_mapping_cloud_authority=Mock(), submit_mapping_cloud_goal=Mock(return_value=response))
        service = MapWorkspaceService.__new__(MapWorkspaceService)
        service._geometry_runtime = NS(get_geometry_runtime_provider=lambda: provider)
        service._asset_repository, service._repository = object(), object()
        resolved = NS(authority=NS(to_dict=lambda: {"fixture": "authority"}), georeference_bytes=b"{}")
        with patch("services.mapping.finalized_asset_authority.FinalizedAssetAuthorityResolver") as resolver:
            resolver.return_value.resolve.return_value = resolved
            result = service.submit_mapping_cloud_goal(query_request_id="query-1", query_text="fixture", motion_admission={})
        self.assertIs(result, response)
        provider.submit_mapping_cloud_goal.assert_called_once()


if __name__ == "__main__":
    unittest.main()
