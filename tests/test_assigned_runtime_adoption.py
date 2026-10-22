"""Assigned workspace runtime adoption: real store/provider, fake wire only."""
import hashlib
import tempfile
import threading
import unittest
from contextlib import ExitStack
from copy import deepcopy
from dataclasses import FrozenInstanceError
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from tests.conftest import LoaderStub, LostReply, predecessor
from tests.test_assigned_asset_storage import bundle_fixture, checkpoint_fixture
from tests.conftest import response_fixture

from contracts.map_identity import AssetIdentityV2
from contracts.geometry_runtime import PlatformGeometryConfig
from contracts.runtime_views import MapSnapshot
from services.geometry import geometry_runtime_provider as provider_module
from services.geometry import runtime as module
from services.geometry.mapping_backend_checkpoint_store import MappingBackendCheckpointStore


class ObservedLock:
    def __init__(self, lock, *, before=None):
        self.lock, self.before = lock, before

    def __enter__(self):
        if self.before is not None:
            self.before()
        self.lock.acquire()
        return self

    def __exit__(self, *args):
        self.lock.release()

    def __getattr__(self, name):
        return getattr(self.lock, name)


class AssignedRuntimeAdoptionTests(unittest.TestCase):
    def setUp(self):
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.root = Path(self.stack.enter_context(tempfile.TemporaryDirectory()))
        checkpoint, occupancy, cloud = checkpoint_fixture()
        self.stored = MappingBackendCheckpointStore(self.root).publish(
            checkpoint, occupancy_asset=occupancy, cloud_asset=cloud)
        self.bundle = bundle_fixture().model_copy(update={"asset_state": "finalized"})
        self.stub = LoaderStub(response_fixture())
        self.provider = provider_module.GeometryRuntimeProvider(None)
        self.provider._connected = True
        self.provider._stub = self.stub
        self.stub.owner = self.provider
        self.runtime = module.PlatformGeometryRuntime(config=PlatformGeometryConfig(), catalog_root=str(self.root),
            robot=SimpleNamespace(providers=SimpleNamespace()))
        self.runtime._provider_attach_epoch = 2
        self.runtime._remote_provider = self.provider
        self.runtime._provider_attach_stopping = False
        self.stub.during = self.assert_unlocked
        for name in hashlib.algorithms_guaranteed | {"new", "file_digest"}:
            if hasattr(hashlib, name):
                self.stack.enter_context(patch.object(hashlib, name, side_effect=AssertionError("hash called")))
        for name in ("_materialize_checkpoint_pcd_asset",
                     "_install_map_activation_snapshot_locked", "get_authoritative_mapping_session_state",
                     "stop_mapping"):
            self.stack.enter_context(patch.object(self.runtime, name, side_effect=AssertionError(name)))
        self.stack.enter_context(patch.object(MapSnapshot, "__deepcopy__", side_effect=AssertionError("map copy")))
        self.prepared = self.runtime.prepare_assigned_map_activation(self.bundle, bundle_root=self.root)
        self.request = {"activation_request_id": "mapload:exact", "session_id": "mapload:exact",
                        "map_name": self.bundle.map_name,
                        "asset_identity": self.bundle.asset_identity.model_dump(),
                        "checkpoint_id": self.prepared.checkpoint_id, "expected_runtime_generation": 7}

    def assert_unlocked(self, operation):
        self.assertFalse(self.runtime._runtime_state_lock._is_owned(), operation)
        self.assertFalse(self.runtime._mapping_lifecycle_lock._is_owned(), operation)
        self.assertFalse(self.runtime._provider_attach_lock._is_owned(), operation)

    def activate(self):
        return self.runtime.dispatch_prepared_assigned_map(self.prepared, self.request)

    def install(self, evidence=None, prepared=None):
        return self.runtime.install_prepared_assigned_map(
            prepared or self.prepared,
            activation_evidence=evidence if evidence is not None else self.activate())

    def test_preparation_is_opaque_immutable_and_store_loaded_once(self):
        with self.assertRaises(TypeError):
            module._PreparedAssignedMapActivation()
        with self.assertRaises(FrozenInstanceError):
            self.prepared._attach_epoch = 99
        cells = self.bundle.snapshot.occupancy_grid.data
        self.assertIs(self.prepared.bundle.snapshot.occupancy_grid.data, cells)
        with patch.object(MappingBackendCheckpointStore, "load", side_effect=AssertionError("second load")):
            self.runtime.capture_assigned_mapping_state(self.prepared)
            target, snapshot = self.install()
        self.assertIs(snapshot.occupancy_grid.data, cells)
        self.assertEqual(target.activation_receipt.to_dict()["request"], self.request)

    def test_reject_candidate_and_noncloud_before_storage(self):
        with patch.object(MappingBackendCheckpointStore, "load", side_effect=AssertionError("premature load")), \
                self.assertRaisesRegex(ValueError, "finalized"):
            self.runtime.prepare_assigned_map_activation(bundle_fixture(), bundle_root=self.root)

    def test_state_capture_detects_attach_replacement_without_lock_over_rpc(self):
        def during(operation):
            self.assert_unlocked(operation)
            self.runtime._provider_attach_epoch += 1
        self.stub.during = during
        with self.assertRaisesRegex(RuntimeError, "ownership changed"):
            self.runtime.capture_assigned_mapping_state(self.prepared)

    def test_state_capture_detects_connection_replacement(self):
        def during(operation):
            self.assert_unlocked(operation)
            self.provider._map_connection_epoch += 1
        self.stub.during = during
        with self.assertRaisesRegex(RuntimeError, "connection changed"):
            self.runtime.capture_assigned_mapping_state(self.prepared)

    def test_preparation_rechecks_origin_after_storage_io(self):
        original = MappingBackendCheckpointStore.load
        def load(store, *args, **kwargs):
            self.assert_unlocked("load")
            value = original(store, *args, **kwargs)
            self.runtime._provider_attach_epoch += 1
            return value
        with patch.object(MappingBackendCheckpointStore, "load", load), \
                self.assertRaisesRegex(RuntimeError, "ownership changed"):
            self.runtime.prepare_assigned_map_activation(self.bundle, bundle_root=self.root)

    def test_exact_predecessor_stop_preserves_stable_descriptor(self):
        self.stub.state = predecessor(active=True)
        state = self.runtime.capture_assigned_mapping_state(self.prepared)
        stopped = self.runtime.stop_assigned_mapping_predecessor(self.prepared, state)
        self.assertEqual(stopped["phase"], "STOPPED")
        self.assertFalse(stopped["active"])
        self.assertEqual(stopped["session_id"], "predecessor")
        self.assertEqual(self.stub.calls.count("StopMapping"), 1)

    def test_predecessor_change_never_dispatches_stop(self):
        self.stub.state = predecessor(active=True)
        state = self.runtime.capture_assigned_mapping_state(self.prepared)
        for name, changed in (("map_name", "other"), ("session_generation", 4),
                              ("runtime_generation", 8), ("mapping_backend_kind", "raytracing")):
            with self.subTest(name=name):
                self.stub.state = predecessor(active=True)
                setattr(self.stub.state, name, changed)
                with self.assertRaisesRegex(ValueError, "captured owner"):
                    self.runtime.stop_assigned_mapping_predecessor(self.prepared, state)
        self.assertNotIn("StopMapping", self.stub.calls)

    def test_stop_lost_reply_and_postdispatch_mismatch_are_unknown(self):
        for fault in ("lost", "generation", "attach", "connection"):
            with self.subTest(fault=fault):
                self.stub.state = predecessor(active=True)
                prepared = self.runtime.prepare_assigned_map_activation(self.bundle, bundle_root=self.root)
                state = self.runtime.capture_assigned_mapping_state(prepared)
                original = self.stub.StopMapping
                def stop(request, timeout, *, original=original, fault=fault):
                    result = original(request, timeout)
                    if fault == "lost":
                        raise LostReply()
                    if fault == "generation":
                        result.session_generation += 1
                    if fault == "attach":
                        self.runtime._provider_attach_epoch += 1
                    if fault == "connection":
                        self.provider._map_connection_epoch += 1
                    return result
                with patch.object(self.stub, "StopMapping", side_effect=stop), \
                        self.assertRaises(provider_module.GeometryRuntimeMapActivationOutcomeUnknown) as raised:
                    self.runtime.stop_assigned_mapping_predecessor(prepared, state)
                self.assertTrue(raised.exception.backend_activation_outcome_unknown)
                self.assertFalse(raised.exception.backend_stop_confirmed)

    def test_dispatch_preserves_six_fields_and_original_stored_owner(self):
        with patch.object(self.provider, "activate_assigned_platform_map",
                          wraps=self.provider.activate_assigned_platform_map) as dispatch:
            self.activate()
        self.assertEqual(dispatch.call_args.kwargs["activation_request"], self.request)
        self.assertIs(dispatch.call_args.kwargs["stored_checkpoint"], self.prepared.stored_checkpoint)
        self.assertEqual(self.stub.last_request.expected_runtime_generation, 7)
        self.assertEqual(self.stub.last_request.activation_request_id, "mapload:exact")

    def test_dispatch_origin_change_preserves_valid_ack_evidence(self):
        def during(operation):
            self.assert_unlocked(operation)
            if operation == "ActivateMap":
                self.runtime._provider_attach_epoch += 1
        self.stub.during = during
        with self.assertRaises(provider_module.GeometryRuntimeMapActivationAppliedError) as raised:
            self.activate()
        self.assertEqual(raised.exception.activation_evidence["activation_receipt_v2"].to_dict()["request"], self.request)
        self.assertTrue(raised.exception.backend_activation_applied)

    def test_dispatch_unknown_is_not_retried_or_adopted(self):
        self.stub.error = LostReply()
        with self.assertRaises(provider_module.GeometryRuntimeMapActivationOutcomeUnknown):
            self.activate()
        self.assertEqual(self.stub.calls.count("ActivateMap"), 1)
        self.assertIsNone(self.runtime._map_activation_snapshot_read().bundle)

    def test_unknown_current_sealed_read_settles_pending_then_next_activation(self):
        self.stub.error = LostReply()
        with self.assertRaises(provider_module.GeometryRuntimeMapActivationOutcomeUnknown):
            self.activate()
        pending = self.provider._assigned_activation_attempt
        self.assertIsNotNone(pending)
        # Even a complete caller dictionary cannot retire provider uncertainty.
        claimed = self.provider._mapping_state_payload(deepcopy(self.stub.state))
        self.runtime.validate_prepared_assigned_map_evidence(
            self.prepared, activation_evidence=claimed)
        self.assertIs(self.provider._assigned_activation_attempt, pending)
        actual = self.runtime.capture_assigned_mapping_state(self.prepared)
        self.assertIsNone(self.provider._assigned_activation_attempt)
        self.install(actual)
        self.runtime.stop_assigned_mapping_predecessor(self.prepared, actual)
        next_bundle = bundle_fixture(identity=AssetIdentityV2(asset_id="e" * 32)).model_copy(
            update={"asset_state": "finalized"})
        prepared = self.runtime.prepare_assigned_map_activation(next_bundle, bundle_root=self.root)
        next_request = {**self.request, "activation_request_id": "mapload:next", "session_id": "mapload:next",
                        "asset_identity": next_bundle.asset_identity.model_dump()}
        self.stub.error = None
        self.stub.installs = 0  # The fixture's replay counter is per exact request.
        result = self.runtime.dispatch_prepared_assigned_map(prepared, next_request)
        self.assertEqual(result["activation_receipt_v2"].to_dict()["request"], next_request)
        self.assertEqual(self.stub.calls.count("ActivateMap"), 2)

    def test_current_nonmatching_or_partial_state_never_retires_pending_attempt(self):
        self.stub.error = LostReply()
        with self.assertRaises(provider_module.GeometryRuntimeMapActivationOutcomeUnknown):
            self.activate()
        pending, original = self.provider._assigned_activation_attempt, deepcopy(self.stub.state)
        for change in ("request", "partial"):
            with self.subTest(change=change):
                self.stub.state = deepcopy(original)
                if change == "request":
                    self.stub.state.activation_request_id = "mapload:other"
                    receipt = self.stub.state.activation_receipt_v2
                    for request in (receipt.request, receipt.activation.request, receipt.target.activation.request):
                        request.activation_request_id = "mapload:other"
                else:
                    self.stub.state.activation_receipt_disposition = 1
                    self.stub.state.relocalization_target_available = False
                    self.stub.state.activation_receipt_v2.ClearField("target")
                    self.stub.state.activation_receipt_v2.ClearField("activation")
                self.runtime.capture_assigned_mapping_state(self.prepared)
                self.assertIs(self.provider._assigned_activation_attempt, pending)

    def test_pending_retirement_rechecks_transition_and_exact_attempt_cas(self):
        self.stub.error = LostReply()
        with self.assertRaises(provider_module.GeometryRuntimeMapActivationOutcomeUnknown):
            self.activate()
        pending = self.provider._assigned_activation_attempt
        def during(operation):
            self.assert_unlocked(operation)
            if operation == "GetMappingState":
                self.provider._mapping_transition_epoch += 1
        self.stub.during = during
        with self.assertRaisesRegex(RuntimeError, "ownership changed"):
            self.runtime.capture_assigned_mapping_state(self.prepared)
        self.assertIs(self.provider._assigned_activation_attempt, pending)
        replacement = provider_module._AssignedActivationAttempt(
            pending.request_key, pending.stored_checkpoint, pending.stub, pending.connection_epoch)
        def replace_during(operation):
            self.assert_unlocked(operation)
            if operation == "GetMappingState":
                self.provider._assigned_activation_attempt = replacement
        self.stub.during = replace_during
        with self.assertRaisesRegex(RuntimeError, "ownership changed"):
            self.runtime.capture_assigned_mapping_state(self.prepared)
        self.assertIs(self.provider._assigned_activation_attempt, replacement)

    def test_install_reconcile_preserves_token_generation_and_source_content(self):
        evidence = self.activate()
        target, snapshot = self.install(evidence)
        before = self.runtime._map_activation_snapshot_read()
        current = self.runtime.capture_assigned_mapping_state(self.prepared)
        second, second_snapshot = self.install(current)
        self.assertIs(second._snapshot, target._snapshot)
        self.assertIs(before, self.runtime._map_activation_snapshot_read())
        self.assertEqual(snapshot.metadata["map_activation_generation"], second_snapshot.metadata["map_activation_generation"])
        self.assertEqual(snapshot.metadata["source_occupancy_receipt"]["session_id"], "source")
        self.assertEqual(snapshot.metadata["activation_receipt_v2"]["target"]["occupancy_receipt"]["session_id"], self.request["session_id"])
        self.assertEqual(snapshot.update_sequence, 6)
        self.assertIs(snapshot.occupancy_grid.data, self.bundle.snapshot.occupancy_grid.data)
        self.assertIs(target.activation_receipt, evidence["activation_receipt_v2"])
        self.assertEqual(target.map_name, self.bundle.map_name)


    def test_validation_admits_exact_receipt_without_install_or_origin_mutation(self):
        evidence = self.activate()
        before = self.runtime._map_activation_snapshot_read()
        receipt = self.runtime.validate_prepared_assigned_map_evidence(
            self.prepared, activation_evidence=evidence)
        self.assertEqual(receipt.to_dict()["request"], self.request)
        self.assertIs(self.runtime._map_activation_snapshot_read(), before)
        self.assertIsNone(self.runtime._assigned_map_installation_origin)
        self.assertIs(receipt, evidence["activation_receipt_v2"])

    def test_install_cas_rejects_changed_installation(self):
        evidence = self.activate()
        with self.runtime._runtime_state_lock:
            self.runtime._map_activation_snapshot = module._MapActivationSnapshot(None, None, {}, 1)
        with self.assertRaisesRegex(RuntimeError, "changed before adoption"):
            self.install(evidence)

    def test_native_unavailable_is_not_promoted(self):
        evidence = self.activate()
        evidence["relocalization_target_available"] = False
        target, _ = self.install(evidence)
        with self.runtime.assigned_map_installation_guard(expected_target=target) as receipt:
            self.assertTrue(self.runtime._runtime_state_lock._is_owned())
            self.assertIs(receipt, target.activation_receipt)
        state = self.runtime._activation_result_from_snapshot(target._snapshot)
        self.assertIs(state["relocalization_target_available"], False)

    def test_changed_native_availability_updates_all_immutable_snapshot_consumers(self):
        evidence = self.activate()
        first, _ = self.install(evidence)
        prepared = self.runtime.prepare_assigned_map_activation(self.bundle, bundle_root=self.root)
        self.stub.state.relocalization_target_available = False
        evidence = self.runtime.capture_assigned_mapping_state(prepared)
        reconciled, _ = self.install(evidence, prepared=prepared)
        self.assertIsNot(reconciled._snapshot, first._snapshot)
        self.assertEqual(reconciled.installation_generation, first.installation_generation + 1)
        current = self.runtime._map_activation_snapshot_read()
        self.assertFalse(self.runtime._activation_result_from_snapshot(current)["relocalization_target_available"])
        self.assertIs(current.bundle.snapshot.occupancy_grid.data, first._snapshot.bundle.snapshot.occupancy_grid.data)
        with self.runtime._runtime_state_lock:
            with self.assertRaisesRegex(RuntimeError, "unavailable"):
                self.runtime._require_assigned_map_activation_target_locked(reconciled, require_available=True)
            with self.assertRaisesRegex(RuntimeError, "changed"):
                self.runtime._require_assigned_map_activation_target_locked(first)

    def test_availability_replacement_cas_rejects_older_prepared_evidence(self):
        original = self.activate()
        self.install(original)
        older = self.runtime.prepare_assigned_map_activation(self.bundle, bundle_root=self.root)
        newer = self.runtime.prepare_assigned_map_activation(self.bundle, bundle_root=self.root)
        self.stub.state.relocalization_target_available = False
        state = self.runtime.capture_assigned_mapping_state(newer)
        target, _ = self.install(state, prepared=newer)
        with self.assertRaisesRegex(RuntimeError, "changed before adoption"):
            self.install(original, prepared=older)
        self.assertIs(self.runtime._map_activation_snapshot_read(), target._snapshot)

    def test_exact_stop_and_adoption_replace_cached_mapping_owner(self):
        self.stub.state = predecessor(active=True)
        captured = self.runtime.capture_assigned_mapping_state(self.prepared)
        self.runtime._mapping_session_context = deepcopy(captured)
        self.runtime._mapping_session_provider_attach_epoch = self.runtime._provider_attach_epoch
        self.runtime.stop_assigned_mapping_predecessor(self.prepared, captured)
        stopped = self.runtime.get_mapping_session_context()
        self.assertEqual(stopped["session_id"], "predecessor")
        self.assertIs(stopped["active"], False)
        self.install()
        installed = self.runtime.get_mapping_session_context()
        self.assertEqual(installed["session_id"], self.request["session_id"])
        self.assertEqual(installed["map_name"], self.request["map_name"])
        self.assertIs(installed["active"], True)
        self.assertEqual(self.runtime._mapping_session_provider_attach_epoch, self.prepared._attach_epoch)
        self.runtime._runtime_started = True
        self.runtime._exploration_loop_lock = threading.RLock()
        self.runtime._external_geometry_session_mode = None
        self.runtime._external_exploration_database_path = None
        with patch.object(self.runtime, "_is_geometry_profile_enabled", return_value=True), \
                patch.object(self.provider, "get_mapping_diagnostics", return_value=None), \
                patch.object(self.provider, "start_mapping", return_value=installed) as start:
            result = self.runtime.start_mapping(map_name=self.request["map_name"])
        self.assertEqual(start.call_args.kwargs["session_id"], self.request["session_id"])
        self.assertEqual(result["mapping_session_id"], self.request["session_id"])
        with patch.object(self.provider, "get_exploration_state", return_value={"status": "IDLE"}), \
                patch.object(self.provider, "stop_mapping", return_value={**installed, "active": False, "phase": "STOPPED"}) as stop, \
                patch.object(self.runtime, "_record_runtime_diagnostic"):
            result = module.PlatformGeometryRuntime.stop_mapping(self.runtime)
        self.assertEqual(stop.call_args.kwargs["session_id"], self.request["session_id"])
        self.assertEqual(result["mapping_session_id"], self.request["session_id"])

    def test_installation_guard_fences_actual_provider_disconnect_metadata(self):
        target, _ = self.install()
        attempt, finished = threading.Event(), threading.Event()
        main_thread = threading.get_ident()
        self.provider._map_state_lock = ObservedLock(self.provider._map_state_lock,
            before=lambda: attempt.set() if threading.get_ident() != main_thread else None)
        errors = []
        def disconnect():
            try:
                self.provider.disconnect()
            except BaseException as exc:  # noqa: BLE001 - surface every worker failure in the parent assertion.
                errors.append(exc)
            finally:
                finished.set()
        worker = threading.Thread(target=disconnect)
        with self.runtime.assigned_map_installation_guard(expected_target=target):
            worker.start()
            self.assertTrue(attempt.wait(2))
            self.assertFalse(finished.is_set())
            self.assertIs(self.provider._stub, self.stub)
            self.assertFalse(self.provider._stopping)
            self.assertEqual(self.provider._map_connection_epoch, self.prepared._provider_origin[1])
        worker.join(2)
        self.assertTrue(finished.is_set())
        self.assertEqual(errors, [])
        self.assertIsNone(self.provider._stub)

    def test_runtime_attach_swap_waits_for_existing_installation_guard(self):
        self.install()
        runtime = self.runtime
        runtime._runtime_started = True
        runtime._navigation_update_subscribers = {}
        runtime._remote_navigation_subscription_ids = {}
        runtime._remote_exploration_subscription_id = None
        runtime._exploration_update_subscribers = {}
        runtime._pending_localization_callbacks = []
        runtime._pending_map_callbacks = []
        runtime._remote_localization_subscription_handles = {}
        runtime._remote_map_subscription_handles = {}
        at_commit, allow_commit, attempted_r, done = (threading.Event() for _ in range(4))
        main_thread = threading.get_ident()
        visits = []
        def before_exploration():
            if threading.get_ident() != main_thread:
                visits.append(1)
                if len(visits) == 3:
                    at_commit.set()
                    if not allow_commit.wait(2):
                        raise AssertionError("attach commit barrier timed out")
        runtime._exploration_loop_lock = ObservedLock(threading.RLock(), before=before_exploration)
        runtime._runtime_state_lock = ObservedLock(runtime._runtime_state_lock,
            before=lambda: attempted_r.set() if threading.get_ident() != main_thread else None)
        replacement = provider_module.GeometryRuntimeProvider(None)
        replacement._connected = True
        results = []
        def attach():
            try:
                results.append(runtime._attach_geometry_runtime_provider(SimpleNamespace()))
            finally:
                done.set()
        with patch.object(module, "GeometryRuntimeProvider", return_value=replacement), \
                patch.object(replacement, "connect", return_value=True), \
                patch.object(runtime, "_inject_static_extrinsics"), \
                patch.object(runtime, "_rollback_geometry_runtime_provider_attach"):
            worker = threading.Thread(target=attach)
            worker.start()
            try:
                self.assertTrue(at_commit.wait(2))
                prepared = runtime.prepare_assigned_map_activation(self.bundle, bundle_root=self.root)
                target, _ = self.install(runtime.capture_assigned_mapping_state(prepared), prepared=prepared)
                with runtime.assigned_map_installation_guard(expected_target=target):
                    attempted_r.clear()
                    allow_commit.set()
                    self.assertTrue(attempted_r.wait(2))
                    self.assertIs(runtime._remote_provider, self.provider)
                    self.assertFalse(done.is_set())
            finally:
                allow_commit.set()
                worker.join(2)
        self.assertEqual(results, [True])
        self.assertIs(runtime._remote_provider, replacement)

    def test_fence_uses_whole_request_and_existing_delivery_gate(self):
        drains = []
        def drain(tickets):
            self.assert_unlocked("drain")
            drains.append(tickets)
        with patch.object(self.runtime, "_wait_for_subscription_deliveries", side_effect=drain):
            self.assertTrue(self.runtime.arm_assigned_map_activation_fence(activation_request=self.request))
        self.assertEqual(len(drains), 1)
        self.assertTrue(self.runtime.is_map_asset_activation_recovery_pending())
        view = self.runtime._map_asset_activation_fence_snapshot()
        view["assigned_activation_request"]["asset_identity"]["asset_id"] = "e" * 32
        self.assertEqual(self.runtime._map_asset_activation_fence_snapshot()["assigned_activation_request"], self.request)
        for name, changed in (("session_id", "another"), ("expected_runtime_generation", 8),
                              ("checkpoint_id", "e" * 32), ("asset_identity", {"asset_id": "e" * 32})):
            with self.subTest(name=name):
                other = {**self.request, name: changed}
                self.assertFalse(self.runtime.arm_assigned_map_activation_fence(activation_request=other))
                self.assertFalse(self.runtime.complete_assigned_map_activation(activation_request=other))
        target, _ = self.install()
        self.assertTrue(self.runtime.complete_assigned_map_activation(
            activation_request=self.request, expected_target=target))

    def test_guard_and_fence_release_reject_old_installation_token(self):
        target, _ = self.install()
        self.runtime.arm_assigned_map_activation_fence(activation_request=self.request)
        with self.runtime._runtime_state_lock:
            self.runtime._map_activation_snapshot = module._MapActivationSnapshot(None, None, {}, 2)
        with self.assertRaisesRegex(RuntimeError, "changed"), \
                self.runtime.assigned_map_installation_guard(expected_target=target):
            self.fail("old target entered installation guard")
        with self.assertRaisesRegex(RuntimeError, "changed"):
            self.runtime.complete_assigned_map_activation(activation_request=self.request, expected_target=target)
        self.assertTrue(self.runtime.is_map_asset_activation_recovery_pending())

    def test_existing_assigned_fence_blocks_ordinary_start_and_stop_only(self):
        self.stub.state = predecessor(active=True)
        captured = self.runtime.capture_assigned_mapping_state(self.prepared)
        self.runtime._mapping_session_context = deepcopy(captured)
        self.runtime.arm_assigned_map_activation_fence(activation_request=self.request)
        with patch.object(self.provider, "get_exploration_state", side_effect=AssertionError("exploration queried")), \
                patch.object(self.provider, "stop_mapping", side_effect=AssertionError("ordinary stop dispatched")):
            with self.assertRaisesRegex(RuntimeError, "fenced"):
                module.PlatformGeometryRuntime.stop_mapping(self.runtime)
            with self.assertRaises(RuntimeError):
                self.runtime.start_mapping(map_name=self.request["map_name"])
        stopped = self.runtime.stop_assigned_mapping_predecessor(self.prepared, captured)
        self.assertEqual(stopped["phase"], "STOPPED")

    def test_installation_guard_rejects_lost_origin_and_current_state_reconciles_token(self):
        target, _ = self.install()
        self.assertEqual(target.installation_generation, 1)
        self.runtime._provider_attach_epoch += 1
        self.provider._map_connection_epoch += 1
        with self.assertRaisesRegex(RuntimeError, "ownership changed"), \
                self.runtime.assigned_map_installation_guard(expected_target=target):
            self.fail("stale provider entered guard")
        prepared = self.runtime.prepare_assigned_map_activation(self.bundle, bundle_root=self.root)
        state = self.runtime.capture_assigned_mapping_state(prepared)
        reconciled, _ = self.install(state, prepared=prepared)
        self.assertIs(reconciled._snapshot, target._snapshot)
        with self.runtime.assigned_map_installation_guard(expected_target=reconciled):
            self.assertTrue(self.runtime._runtime_state_lock._is_owned())


if __name__ == "__main__":
    unittest.main()
