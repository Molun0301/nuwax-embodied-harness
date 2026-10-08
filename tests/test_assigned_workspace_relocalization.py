"""Public assigned save/load/relocalize and navigation-pose qualification.

Real ordinary-save preparation, protobuf admission, runtime, stores and session
CAS are used. Only native RPC and peripheral endpoints are simulated. These
Python checks do not qualify the native sample's geometry or real RPC/LCM path.
"""

import json
import struct
import unittest
from contextlib import ExitStack
from copy import deepcopy
from dataclasses import FrozenInstanceError
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import test_assigned_workspace_save as save_fixtures
from test_assigned_activation_loader import LostReply
from test_assigned_asset_storage import MAP_NAME
from test_assigned_relocalization_contract import terminal_dict
from test_assigned_workspace_save import SaveNativeStub

from contracts.map_activation import AssignedActivationReceipt
from contracts.map_relocalization import (
    AssignedRelocalizationQuery,
    AssignedRelocalizationRequest,
    AssignedRelocalizationTerminal,
)
from contracts.runtime_views import LocalizationSessionStatus
from core import StateNamespace
from core.errors import GatewayError
from services.geometry import geometry_runtime_provider as provider_module
from services.localization.service import LocalizationService
from services.mapping import catalog as catalog_module
from services.mapping.catalog import MapCatalogRepository
from services.mapping.service import MappingService
from skills import assigned_map_relocalization as relocalization_module

pb = provider_module.pb2
REQUEST_ID = "relocalize:public-exact"


class RelocalizationCancelled(BaseException):
    """Task cancellation retains its type and its already-crossed request."""


def _candidate_body(owner, frame):
    """Independent V3 native field order, adapted to actual saved ownership."""
    def string(value):
        encoded = value.encode()
        return struct.pack(">i", len(encoded) + 1) + encoded + b"\0"

    occupancy = frame["occupancy_receipt"]
    geometry = frame["map_geometry"]
    body = struct.pack(">3i3q", 3, 3, 1, owner["runtime_generation"],
                       owner["correction_backend_generation"], 17)
    body += string(occupancy["session_id"]) + struct.pack(">q", occupancy["session_generation"])
    body += string(occupancy["backend_kind"]) + struct.pack(">2q",
        occupancy["backend_generation"], frame["map_frame_generation"])
    body += bytes.fromhex(occupancy["commit_id"]["boot_nonce"])
    body += struct.pack(">4q", occupancy["commit_id"]["sequence"],
        occupancy["occupancy_revision"], frame["cloud_revision"], occupancy["source_timestamp_ns"])
    body += struct.pack(">i6q", frame["lio_backend"], frame["runtime_generation"],
        frame["source_sequence"], frame["source_timestamp_ns"], frame["admitted_point_count"],
        frame["correlated_odom_source_sequence"], frame["correlated_odom_source_timestamp_ns"])
    body += struct.pack(">7d", 0, 0, 0, 0, 0, 0, 1)
    body += struct.pack(">2q3Qq", geometry["width"], geometry["height"],
        geometry["resolution_m_bits"], geometry["origin_x_m_bits"],
        geometry["origin_y_m_bits"], frame["source_timestamp_ns"])
    body += b"".join(string(frame[name]) for name in (
        "map_frame_id", "odom_frame_id", "body_frame_id", "points_frame_id"))
    body += bytes.fromhex(owner["correction_lease_boot_nonce"])
    body += struct.pack(">q7di", owner["correction_lease_generation"], 0, 0, 0, 0, 0, 0, 1, 0)
    return body


def terminal_for_request(request, *, phase=5):
    """Retain exact request, actual installed receipts and non-hash evidence."""
    document = terminal_dict(phase)
    document["request_id"] = request.request_id
    body = document["assigned_cloud_v2"]
    body["request"] = request.to_dict()
    if phase == 1:
        return AssignedRelocalizationTerminal.admit(document)
    target = request.to_dict()["assigned_cloud_v2"]["target"]
    evidence = body["evidence"]
    owner = evidence["owner"]
    owner["runtime_generation"] = target["activation"]["runtime_generation"]
    if phase >= 3:
        frame = evidence["source_frames"][0]
        frame.update(occupancy_receipt=deepcopy(target["occupancy_receipt"]),
            cloud_receipt=deepcopy(target["cloud_receipt"]),
            cloud_revision=target["cloud_receipt"]["cloud_revision"],
            runtime_generation=owner["runtime_generation"],
            admitted_point_count=target["cloud_receipt"]["point_count"])
        frame["source_timestamp_ns"] = max(frame["source_timestamp_ns"],
                                           target["occupancy_receipt"]["source_timestamp_ns"])
        frame["correlated_odom_source_timestamp_ns"] = frame["source_timestamp_ns"]
        if phase == 5:
            body["candidate_payload_v3"] = _candidate_body(owner, frame).hex()
    return AssignedRelocalizationTerminal.admit(document)


def sample_for_terminal(terminal):
    wire = terminal.to_proto(pb.RelocalizeResult)
    evidence = wire.assigned_cloud_v2.evidence
    sample = pb.AssignedLocalizationSampleV2(schema_version=2, terminal=wire,
        correction_owner=evidence.owner, correction_revision=terminal.correction_revision,
        map_frame_generation=evidence.source_frames[-1].map_frame_generation)
    sample.pose.frame_id = evidence.owner.map_frame_id
    sample.pose.timestamp_ns = 4_000_000_000
    sample.pose.position.x, sample.pose.position.y, sample.pose.position.z = 1.5, -2, .25
    sample.pose.orientation.w = 1
    sample.map_from_odom.parent_frame = evidence.owner.map_frame_id
    sample.map_from_odom.child_frame = evidence.owner.odom_frame_id
    sample.odom_from_body.parent_frame = evidence.owner.odom_frame_id
    sample.odom_from_body.child_frame = evidence.owner.body_frame_id
    for edge in (sample.map_from_odom, sample.odom_from_body):
        edge.timestamp_ns = sample.pose.timestamp_ns
        edge.translation.SetInParent()
        edge.rotation.w = 1
    sample.odom_from_body.translation.CopyFrom(sample.pose.position)
    return sample


class RelocalizationNativeStub(SaveNativeStub):
    """Python-only exact native-ledger/sample stand-in; never a native proof."""

    def __init__(self):
        super().__init__()
        self.relocalization_requests = []
        self.query_requests = []
        self.sample_requests = []
        self.terminal = None
        self.terminal_phase = 5
        self.lose_relocalization_reply = False
        self.query_status = 1
        self.sample_mutation = None
        self.query_mutation = None
        self.before_dispatch = None
        self.during_sample = None

    def Relocalize(self, request, timeout):
        self.visit("Relocalize")
        admitted = AssignedRelocalizationRequest.from_proto(request)
        if self.before_dispatch:
            self.before_dispatch(admitted)
        self.relocalization_requests.append(deepcopy(request))
        if self.terminal is None:
            self.terminal = terminal_for_request(admitted, phase=self.terminal_phase)
        else:
            assert self.terminal.request.exact_replay_of(admitted)
        if self.lose_relocalization_reply:
            raise LostReply()
        return self.terminal.to_proto(pb.RelocalizeResult)

    def GetRelocalizationTerminal(self, request, timeout):
        self.visit("GetRelocalizationTerminal")
        self.query_requests.append(deepcopy(request))
        query = AssignedRelocalizationQuery.from_proto(request)
        response = query.response(self.query_status, self.terminal if self.query_status == 1 else None)
        wire = pb.GetRelocalizationTerminalResponse(**query.response_to_provider_fields(response))
        if self.query_mutation:
            self.query_mutation(wire)
        return wire

    def GetAssignedLocalizationSample(self, request, timeout):
        self.visit("GetAssignedLocalizationSample")
        self.sample_requests.append(deepcopy(request))
        admitted = AssignedRelocalizationRequest.admit_retained(
            provider_module._project_assigned_proto(request, 'request'))
        assert self.terminal is not None and self.terminal.request.exact_replay_of(admitted)
        result = sample_for_terminal(self.terminal)
        if self.sample_mutation:
            self.sample_mutation(result)
        if self.during_sample:
            self.during_sample()
        return result


class AssignedWorkspaceRelocalizationTests(unittest.TestCase):
    # Reuse ordinary cloud-save setup without inheriting and recollecting its tests.
    new_runtime = save_fixtures.AssignedWorkspaceSaveTests.new_runtime
    new_service = save_fixtures.AssignedWorkspaceSaveTests.new_service
    save = save_fixtures.AssignedWorkspaceSaveTests.save
    assert_unlocked = save_fixtures.AssignedWorkspaceSaveTests.assert_unlocked
    assigned_only = save_fixtures.AssignedWorkspaceSaveTests.assigned_only

    def setUp(self):
        save_fixtures.AssignedWorkspaceSaveTests.setUp(self)
        self.stub = RelocalizationNativeStub()
        self.provider._stub = self.stub
        self.stub.owner = self.provider
        self.saved = self.save()
        self.service.activate_map(map_name=MAP_NAME, load_memory_history=False)
        self.stub.calls.clear()
        self.stub.during = self.assert_unlocked

    def relocalize(self, *, request_id=REQUEST_ID):
        return self.service.relocalize_active_map(map_name=MAP_NAME,
            metadata={"request_id": request_id, "logical_timeout_ms": 10_000})

    def forbid_unpaired_pose(self):
        guard = ExitStack()
        guard.enter_context(self.assigned_only())
        for owner, names in (
            (self.provider, ("get_current_pose", "get_frame_tree", "get_localization_state")),
            (self.localization, ("refresh",)),
            (self.runtime, ("get_relocalization_terminal", "get_validated_relocalization_receipt")),
        ):
            for name in names:
                if hasattr(owner, name):
                    guard.enter_context(patch.object(owner, name,
                        side_effect=AssertionError("unpaired or legacy path: " + name)))
        return guard

    def assert_not_ready(self):
        session = self.localization.get_active_session()
        if session is not None:
            self.assertNotEqual(session.status, LocalizationSessionStatus.READY)
            self.assertFalse(session.pose_available)
        self.assertIsNone(self.service.get_relocalized_pose(MAP_NAME))

    def assert_original_owner(self):
        self.assertEqual(len(self.stub.relocalization_requests), 1)
        request = AssignedRelocalizationRequest.from_proto(self.stub.relocalization_requests[0])
        self.assertEqual(request.request_id, REQUEST_ID)
        for wire in self.stub.query_requests:
            query = AssignedRelocalizationQuery.from_proto(wire)
            self.assertTrue(query.request.exact_replay_of(request))
        for wire in self.stub.sample_requests:
            self.assertTrue(AssignedRelocalizationRequest.from_proto(wire).exact_replay_of(request))
        return request

    def test_save_load_one_durable_exact_dispatch_then_native_sample_publishes_ready(self):
        def durable_before_dispatch(request):
            pending = MapCatalogRepository(str(self.root)).get_assigned_relocalization_intent()
            self.assertEqual(pending["phase"], "backend_dispatching")
            self.assertEqual(pending["assigned_request_v2"], request.to_dict())
            self.assertEqual(request.asset_identity, self.saved.asset_identity)

        self.stub.before_dispatch = durable_before_dispatch
        with self.forbid_unpaired_pose():
            session, workspace = self.relocalize()
            pose = self.service.get_relocalized_pose(MAP_NAME)
        self.assertEqual(session.status, LocalizationSessionStatus.READY)
        self.assertTrue(session.pose_available)
        self.assertTrue(workspace.localization_ready)
        self.assertEqual(pose.position.x, 1.5)
        self.assertEqual(pose.position.y, -2)
        self.assertEqual(session.asset_identity, self.saved.asset_identity)
        self.assertTrue(session.relocalization_terminal_v2.exact_replay_of(self.stub.terminal))
        self.assertIsNone(self.service._repository.get_relocalization_intent())
        self.assertEqual(self.stub.calls.count("Relocalize"), 1)
        self.assertGreaterEqual(self.stub.calls.count("GetAssignedLocalizationSample"), 1)
        self.assertNotIn("ActivateMap", self.stub.calls)
        self.assertNotIn("StopMapping", self.stub.calls)
        request = self.assert_original_owner()
        receipt = MapCatalogRepository(str(self.root)).get_last_successful_relocalization_receipt()
        self.assertEqual(receipt["assigned_terminal_v2"], self.stub.terminal.to_dict())
        self.assertEqual(request.activation_receipt, session.activation_receipt_v2)

    def test_lost_reply_exact_query_recovers_without_redispatch_or_request_rotation(self):
        self.stub.lose_relocalization_reply = True
        with self.assertRaises(GatewayError):
            self.relocalize()
        pending = self.service._repository.get_assigned_relocalization_intent()
        self.assertEqual(pending["phase"], "backend_outcome_unknown")
        self.assert_not_ready()
        with self.forbid_unpaired_pose():
            self.service.recover_relocalization_receipt()
            pose = self.service.get_relocalized_pose(MAP_NAME)
        self.assertIsNotNone(pose)
        self.assertEqual(self.localization.get_active_session().status, LocalizationSessionStatus.READY)
        self.assertTrue(self.stub.query_requests)
        self.assert_original_owner()
        self.assertIsNone(self.service._repository.get_relocalization_intent())

    def test_pending_not_found_and_identity_conflict_queries_keep_unknown_original_owner(self):
        self.stub.lose_relocalization_reply = True
        with self.assertRaises(GatewayError):
            self.relocalize()
        original = self.service._repository.get_assigned_relocalization_intent()
        for status in (2, 3, 4):
            with self.subTest(status=status):
                self.stub.query_status = status
                with self.assertRaises(GatewayError):
                    self.service.recover_relocalization_receipt()
                self.assertEqual(self.service._repository.get_assigned_relocalization_intent(), original)
                with self.assertRaises(GatewayError):
                    self.service.relocalize_active_map(map_name=MAP_NAME)
                self.assertEqual(self.service._repository.get_assigned_relocalization_intent(), original)
                before_conflict = list(self.stub.calls)
                with self.assertRaises(GatewayError):
                    self.relocalize(request_id="relocalize:must-not-rotate")
                self.assertEqual(self.stub.calls, before_conflict)
                self.assert_not_ready()
                self.assert_original_owner()
        self.assertFalse(self.stub.sample_requests)

    def test_stale_same_frame_pose_never_publishes_ready_or_navigation_pose(self):
        def stale(sample):
            for value in (sample.pose, sample.map_from_odom, sample.odom_from_body):
                value.timestamp_ns = 1
        self.stub.sample_mutation = stale
        with self.forbid_unpaired_pose(), self.assertRaises(GatewayError):
            self.relocalize()
        self.assert_not_ready()
        self.assert_original_owner()
        self.assertIsNotNone(self.service._repository.get_relocalization_intent())

    def test_query_same_id_with_changed_whole_request_never_releases_original_unknown(self):
        self.stub.lose_relocalization_reply = True
        with self.assertRaises(GatewayError):
            self.relocalize()
        original = self.service._repository.get_assigned_relocalization_intent()

        def replace_echo(wire):
            request = deepcopy(original["assigned_request_v2"])
            request["assigned_cloud_v2"]["logical_timeout_ms"] = 250
            changed = AssignedRelocalizationRequest.admit(request)
            wire.assigned_request_v2.CopyFrom(changed.to_proto(pb.RelocalizeRequest))
            wire.terminal.CopyFrom(terminal_for_request(changed).to_proto(pb.RelocalizeResult))

        self.stub.query_mutation = replace_echo
        with self.assertRaises(GatewayError):
            self.service.recover_relocalization_receipt()
        self.assertEqual(self.service._repository.get_assigned_relocalization_intent(), original)
        self.assertFalse(self.stub.sample_requests)
        self.assert_not_ready()
        self.assert_original_owner()

    def test_changed_exact_terminal_in_sample_cannot_upgrade_the_durable_receipt(self):
        def change_terminal(sample):
            sample.terminal.assigned_cloud_v2.correction_revision += 1
            sample.correction_revision += 1
        self.stub.sample_mutation = change_terminal
        with self.assertRaises(GatewayError):
            self.relocalize()
        self.assert_not_ready()
        receipt = self.service._repository.get_last_successful_relocalization_receipt()
        self.assertEqual(receipt["assigned_terminal_v2"], self.stub.terminal.to_dict())
        self.assert_original_owner()

    def test_changed_correction_owner_revision_or_frame_generation_cannot_publish_ready(self):
        changes = (
            lambda value: setattr(value.correction_owner, "correction_lease_generation", 999),
            lambda value: setattr(value, "correction_revision", 999),
            lambda value: setattr(value, "map_frame_generation", 999),
        )
        self.stub.sample_mutation = changes[0]
        with self.assertRaises(GatewayError):
            self.relocalize()
        for mutate in changes:
            with self.subTest(change=mutate):
                self.stub.sample_mutation = mutate
                with self.assertRaises(GatewayError):
                    self.service.recover_relocalization_receipt()
                self.assert_not_ready()
                self.assert_original_owner()

    def test_exact_terminal_request_source_and_ids_are_detached_and_immutable(self):
        session, _ = self.relocalize()
        terminal = session.relocalization_terminal_v2
        before = terminal.to_dict()
        document = terminal.to_dict()
        document["request_id"] = "changed"
        document["assigned_cloud_v2"]["request"]["request_id"] = "changed"
        frame = document["assigned_cloud_v2"]["evidence"]["source_frames"][0]
        frame["occupancy_receipt"]["session_id"] = "changed"
        frame["cloud_receipt"]["commit_id"]["boot_nonce"] = "00" * 16
        self.assertEqual(terminal.to_dict(), before)
        with self.assertRaises((FrozenInstanceError, TypeError)):
            terminal.request_id = "changed"
        self.assertEqual(self.service._repository.get_last_successful_relocalization_receipt()[
            "assigned_terminal_v2"], before)
        pose = self.service.get_relocalized_pose(MAP_NAME)
        pose.position.x = 999
        self.assertEqual(self.service.get_relocalized_pose(MAP_NAME).position.x, 1.5)

    def test_pending_and_completed_request_metadata_conflicts_fail_before_any_rpc(self):
        self.stub.lose_relocalization_reply = True
        with self.assertRaises(GatewayError):
            self.relocalize()
        for state in ("pending", "completed"):
            for changed_metadata in ({"logical_timeout_ms": 250}, {"planar_seed": [0, 0, 0]}):
                with self.subTest(state=state, metadata=changed_metadata):
                    before = list(self.stub.calls)
                    before_pending = self.service._repository.get_relocalization_intent()
                    with self.assertRaises(GatewayError):
                        self.service.relocalize_active_map(map_name=MAP_NAME,
                            metadata={"request_id": REQUEST_ID, **changed_metadata})
                    self.assertEqual(self.stub.calls, before)
                    self.assertEqual(self.service._repository.get_relocalization_intent(), before_pending)
            if state == "pending":
                self.service.recover_relocalization_receipt()
                self.assertIsNone(self.service._repository.get_relocalization_intent())
        self.assert_original_owner()

    def test_retained_policy_v2_pending_and_completed_same_timeout_recover_without_dispatch(self):
        target = self.runtime.capture_assigned_map_activation_target(
            map_name=MAP_NAME, asset_identity=self.saved.asset_identity)
        policy = json.loads((Path(__file__).parent / 'fixtures/relocalization_policy_algorithm2_0276.json').read_text())
        request = AssignedRelocalizationRequest.from_dict({
            'schema_version': 2, 'request_id': REQUEST_ID,
            'assigned_cloud_v2': {'target': target.activation_receipt.to_dict()['target'],
                'policy': policy, 'logical_timeout_ms': 10_000}})
        repository = self.service._repository
        arguments = dict(assigned_request=request, activation_receipt=target.activation_receipt,
            target_asset_identity=target.asset_identity)
        # 模拟旧程序已提交而丢失回复的持久化日记；新程序只查询和取样。
        repository.begin_assigned_relocalization_durable(**arguments)
        repository.mark_assigned_relocalization_backend_dispatching(**arguments)
        repository.mark_assigned_relocalization_outcome_unknown(**arguments)
        self.stub.terminal = terminal_for_request(request)
        for phase in ('pending', 'completed'):
            with self.subTest(phase=phase):
                before = list(self.stub.calls)
                with self.assertRaises(GatewayError):
                    self.service.relocalize_active_map(map_name=MAP_NAME,
                        metadata={'request_id': REQUEST_ID, 'logical_timeout_ms': 250})
                self.assertEqual(self.stub.calls, before)
                with self.forbid_unpaired_pose():
                    session, workspace = self.relocalize()
                self.assertTrue(workspace.localization_ready)
                self.assertEqual(session.status, LocalizationSessionStatus.READY)
                self.assertEqual(session.relocalization_terminal_v2.request.to_dict(), request.to_dict())
        self.assertEqual(self.stub.relocalization_requests, [])
        self.assertTrue(self.stub.query_requests)
        self.assertTrue(self.stub.sample_requests)
        self.assertIsNone(repository.get_assigned_relocalization_intent())

    def test_exact_completed_request_retry_uses_retained_terminal_without_redispatch(self):
        original_session, _ = self.relocalize()
        queries = len(self.stub.query_requests)
        samples = len(self.stub.sample_requests)
        with self.forbid_unpaired_pose():
            session, workspace = self.relocalize()
        self.assertTrue(workspace.localization_ready)
        self.assertEqual(session.status, LocalizationSessionStatus.READY)
        self.assertTrue(session.relocalization_terminal_v2.exact_replay_of(
            original_session.relocalization_terminal_v2))
        self.assertEqual(len(self.stub.query_requests), queries)
        self.assertGreater(len(self.stub.sample_requests), samples)
        self.assertIsNone(self.service._repository.get_relocalization_intent())
        self.assert_original_owner()

    def test_navigation_pose_getter_rechecks_native_sample_and_rejects_new_staleness(self):
        self.relocalize()
        before = len(self.stub.sample_requests)
        changes = (
            lambda value: setattr(value.pose, "timestamp_ns", 1),
            lambda value: setattr(value, "correction_revision", 999),
            lambda value: setattr(value.correction_owner, "correction_lease_generation", 999),
            lambda value: setattr(value, "map_frame_generation", 999),
        )
        with self.forbid_unpaired_pose():
            for mutate in changes:
                with self.subTest(change=mutate):
                    self.stub.sample_mutation = mutate
                    self.assertIsNone(self.service.get_relocalized_pose(MAP_NAME))
        self.assertEqual(len(self.stub.sample_requests), before + len(changes))
        self.assert_original_owner()

    def test_current_provider_mapping_owner_changed_during_sample_cannot_publish_ready(self):
        def replace_owner():
            with self.provider._map_state_lock:
                self.provider._mapping_identity = ("successor", MAP_NAME, 999)
        self.stub.during_sample = replace_owner
        with self.assertRaises(GatewayError):
            self.relocalize()
        self.assert_not_ready()
        self.assertIsNotNone(self.service._repository.get_relocalization_intent())
        self.assert_original_owner()

    def test_provider_replacement_during_sample_cannot_publish_ready(self):
        def replace_provider():
            with self.runtime._runtime_state_lock:
                self.runtime._remote_provider = provider_module.GeometryRuntimeProvider(None)
                self.runtime._provider_attach_epoch += 1
        self.stub.during_sample = replace_provider
        with self.assertRaises(GatewayError):
            self.relocalize()
        self.assert_not_ready()
        self.assertIsNotNone(self.service._repository.get_relocalization_intent())
        self.assert_original_owner()

    def test_session_token_captured_before_sample_prevents_late_ready_overwrite(self):
        published = []
        original = self.store.write

        def record(namespace, key, value, **kwargs):
            self.assert_unlocked("session CAS publication")
            if namespace == StateNamespace.LOCALIZATION and key == "active_session":
                published.append(value)
            return original(namespace, key, value, **kwargs)

        self.stub.during_sample = lambda: self.localization.clear_active_session(reason="successor during sample")
        with patch.object(self.store, "write", side_effect=record), self.assertRaises(GatewayError):
            self.relocalize()
        self.assertIsNone(self.localization.get_active_session_token()[0])
        self.assertFalse(any(item is not None and item.status == LocalizationSessionStatus.READY
                             for item in published))
        self.assertIsNotNone(self.service._repository.get_relocalization_intent())
        self.assertIsNone(self.service.get_relocalized_pose(MAP_NAME))

    def test_ready_publication_callback_supersession_prevents_stale_success_return(self):
        original = self.store.write
        superseded = []

        def supersede(namespace, key, value, **kwargs):
            self.assert_unlocked("READY callback")
            result = original(namespace, key, value, **kwargs)
            if (namespace == StateNamespace.LOCALIZATION and key == "active_session"
                    and value is not None and value.status == LocalizationSessionStatus.READY):
                superseded.append(value)
                self.localization.clear_active_session(reason="callback successor")
            return result

        with patch.object(self.store, "write", side_effect=supersede), self.assertRaises(GatewayError):
            self.relocalize()
        self.assertEqual(len(superseded), 1)
        self.assertIsNone(self.localization.get_active_session_token()[0])
        self.assertIsNone(self.service.get_relocalized_pose(MAP_NAME))

    def test_dispatch_phase_fsync_failure_never_dispatches_and_keeps_original_journal(self):
        original = catalog_module._fsync_directory

        def fail_dispatch(path):
            intent_path = self.service._repository._relocalization_intent_path
            if intent_path.exists() and json.loads(intent_path.read_text())["phase"] == "backend_dispatching":
                raise OSError("dispatch directory fsync failed")
            return original(path)

        with patch.object(catalog_module, "_fsync_directory", side_effect=fail_dispatch), self.assertRaises(GatewayError):
            self.relocalize()
        self.assertFalse(self.stub.relocalization_requests)
        pending = self.service._repository.get_assigned_relocalization_intent()
        self.assertEqual(pending["request_id"], REQUEST_ID)
        self.stub.query_status = 3
        with self.assertRaises(GatewayError):
            self.service.recover_relocalization_receipt()
        with self.assertRaises(GatewayError):
            self.relocalize()
        self.assertFalse(self.stub.relocalization_requests)
        self.assertEqual(self.service._repository.get_assigned_relocalization_intent()[
            "assigned_request_v2"], pending["assigned_request_v2"])
        self.assert_not_ready()

    def test_receipt_fsync_failure_keeps_journal_and_never_reports_unproved_success(self):
        original = catalog_module._fsync_directory

        def fail_receipt(path):
            if self.service._repository._relocalization_receipt_path.exists():
                raise OSError("receipt directory fsync failed")
            return original(path)

        with patch.object(catalog_module, "_fsync_directory", side_effect=fail_receipt):
            with self.assertRaises(GatewayError):
                self.relocalize()
            self.assertIsNotNone(self.service._repository.get_relocalization_intent())
            self.assertIsNone(self.service.get_relocalized_pose(MAP_NAME))
            with self.assertRaises(GatewayError):
                self.service.recover_relocalization_receipt()
            self.assertIsNotNone(self.service._repository.get_relocalization_intent())
            self.assertIsNone(self.service.get_relocalized_pose(MAP_NAME))
        self.service.recover_relocalization_receipt()
        self.assertIsNone(self.service._repository.get_relocalization_intent())
        self.assertIsNotNone(self.service.get_relocalized_pose(MAP_NAME))
        self.assert_original_owner()

    def test_cold_restore_adopts_actual_sealed_activation_readonly_before_native_sample(self):
        self.stub.lose_relocalization_reply = True
        with self.assertRaises(GatewayError):
            self.relocalize()
        pending = self.service._repository.get_assigned_relocalization_intent()
        self.runtime = self.new_runtime()
        self.mapping = MappingService(provider_owner=SimpleNamespace(), state_store=self.store)
        self.localization = LocalizationService(provider_owner=SimpleNamespace(), state_store=self.store)
        self.stub.calls.clear()
        self.stub.during = None

        def assert_installed_before_sample():
            target = self.runtime.capture_assigned_map_activation_target(
                map_name=MAP_NAME, asset_identity=self.saved.asset_identity)
            self.assertTrue(target.activation_receipt.exact_replay_of(self.stub.terminal.request.activation_receipt))
        self.stub.during_sample = assert_installed_before_sample
        self.service = self.new_service()
        self.stub.during = self.assert_unlocked
        with self.forbid_unpaired_pose():
            restored = self.service.restore_active_workspace(load_memory_history=False)
        self.assertTrue(restored.localization_ready)
        self.assertEqual(restored.active_localization_session.status, LocalizationSessionStatus.READY)
        self.assertEqual(restored.active_localization_session.relocalization_terminal_v2.request.to_dict(),
                         pending["assigned_request_v2"])
        self.assertIn("GetMappingState", self.stub.calls)
        self.assertIn("GetAssignedLocalizationSample", self.stub.calls)
        self.assertLess(self.stub.calls.index("GetMappingState"), self.stub.calls.index("GetAssignedLocalizationSample"))
        self.assertNotIn("ActivateMap", self.stub.calls)
        self.assertNotIn("StopMapping", self.stub.calls)
        self.assertNotIn("Relocalize", self.stub.calls)
        self.assert_original_owner()
        self.assertIsNone(self.service._repository.get_relocalization_intent())

    def test_shutdown_unknown_owner_queries_exactly_without_rotation_or_dispatch(self):
        self.stub.lose_relocalization_reply = True
        with self.assertRaises(GatewayError):
            self.relocalize()
        pending = self.service._repository.get_assigned_relocalization_intent()
        self.stub.query_status = 2
        outcome = self.service.shutdown_relocalization_owner()
        self.assertFalse(outcome["settled"])
        self.assertEqual(self.service._repository.get_assigned_relocalization_intent(), pending)
        self.assertTrue(self.stub.query_requests)
        self.assert_original_owner()

    def test_cold_start_not_found_keeps_journal_and_never_activates_or_returns_pose(self):
        self.stub.lose_relocalization_reply = True
        with self.assertRaises(GatewayError):
            self.relocalize()
        pending = self.service._repository.get_assigned_relocalization_intent()
        self.stub.query_status = 3
        self.runtime = self.new_runtime()
        self.mapping = MappingService(provider_owner=SimpleNamespace(), state_store=self.store)
        self.localization = LocalizationService(provider_owner=SimpleNamespace(), state_store=self.store)
        self.stub.calls.clear()
        self.service = self.new_service()
        self.assertEqual(self.service._repository.get_assigned_relocalization_intent(), pending)
        self.assert_not_ready()
        with self.assertRaises(GatewayError):
            self.service.restore_active_workspace(load_memory_history=False)
        self.assertEqual(self.service._repository.get_assigned_relocalization_intent(), pending)
        self.assert_not_ready()
        self.assertNotIn("ActivateMap", self.stub.calls)
        self.assertNotIn("StopMapping", self.stub.calls)
        self.assertNotIn("Relocalize", self.stub.calls)
        self.assertFalse(self.stub.sample_requests)
        self.assert_original_owner()

    def test_authoritative_negative_query_clears_only_exact_pending_without_ready(self):
        self.stub.terminal_phase = 1
        self.stub.lose_relocalization_reply = True
        with self.assertRaises(GatewayError):
            self.relocalize()
        activation = self.service._repository.get_last_successful_map_activation_receipt()
        with self.assertRaises(GatewayError) as failure:
            self.service.recover_relocalization_receipt()
        self.assertEqual(failure.exception.error_code, "relocalization_rejected")
        self.assertIsNone(self.service._repository.get_relocalization_intent())
        self.assertIsNone(self.service._repository.get_last_successful_relocalization_receipt())
        self.assertEqual(self.service._repository.get_last_successful_map_activation_receipt(), activation)
        self.assertFalse(self.stub.sample_requests)
        self.assert_not_ready()
        self.assert_original_owner()

    def test_applied_old_request_retires_against_actual_durable_sealed_replacement(self):
        self.stub.lose_relocalization_reply = True
        with self.assertRaises(GatewayError):
            self.relocalize()
        original_request = self.assert_original_owner()
        replacement_wire = deepcopy(self.stub.activation_requests[-1])
        replacement_request = original_request.activation_receipt.to_dict()["request"]
        replacement_request.update(activation_request_id="mapload:" + "e" * 32,
                                   session_id="mapload:" + "e" * 32)
        args = {"target_map_name": MAP_NAME, "target_asset_identity": self.saved.asset_identity,
                "target_checkpoint_id": replacement_wire.mapping_backend_checkpoint.checkpoint_id}
        replacement_journal = self.service._repository.begin_assigned_map_activation(
            previous_map_name=MAP_NAME, load_memory_history=False,
            activation_request=replacement_request, reuse_completed_request_id=False, **args)
        replacement_wire.activation_request_id = replacement_journal["activation_request_id"]
        replacement_wire.session_id = replacement_wire.activation_request_id
        owner_args = dict(**args, activation_request_id=replacement_wire.activation_request_id,
                          activation_request=replacement_request)
        self.service._repository.update_assigned_map_activation_phase("backend_dispatching",
            expected_phases=["prepared"], **owner_args)
        # Fixture a genuine distinct native ACK, then persist that exact proof.
        self.stub.StopMapping(SimpleNamespace(session_id=self.stub.state.session_id), timeout=1)
        replacement_state = self.stub.ActivateMap(replacement_wire, timeout=1)
        replacement_evidence = self.provider._mapping_state_payload(replacement_state)
        replacement = AssignedActivationReceipt.admit(replacement_evidence["activation_receipt_v2"])
        self.assertFalse(replacement.exact_replay_of(original_request.activation_receipt))
        self.service._repository.record_assigned_map_activation_evidence(**owner_args,
            activation_receipt=replacement, expected_phases=["backend_dispatching"],
            **{name: replacement_evidence[name] for name in (
                "relocalization_policy_v2", "relocalization_target_available",
                "activation_receipt_disposition", "activation_receipt_disposition_name")})
        self.service._repository.complete_assigned_map_activation_durable(**owner_args,
            activation_receipt=replacement, expected_phases=["backend_applied"],
            relocalization_target_available=True)
        # A restarted consumer must observe the current sealed replacement itself.
        self.runtime = self.new_runtime()
        self.mapping = MappingService(provider_owner=SimpleNamespace(), state_store=self.store)
        self.localization = LocalizationService(provider_owner=SimpleNamespace(), state_store=self.store)
        self.service._geometry_runtime = self.runtime
        self.service._mapping_service = self.mapping
        self.service._localization_service = self.localization
        self.stub.calls.clear()
        with self.forbid_unpaired_pose():
            result = self.service.recover_relocalization_receipt()
        self.assertIsNone(result)
        self.assertIsNone(self.service._repository.get_relocalization_intent())
        self.assertEqual(self.service._repository.get_last_successful_map_activation_receipt()[
            "activation_receipt_v2"], replacement.to_dict())
        self.assertIsNone(self.service._repository.get_last_successful_relocalization_receipt())
        self.assertIn("GetMappingState", self.stub.calls)
        self.assertIn("GetRelocalizationTerminal", self.stub.calls)
        self.assertNotIn("StopMapping", self.stub.calls)
        self.assertNotIn("ActivateMap", self.stub.calls)
        self.assertNotIn("Relocalize", self.stub.calls)
        self.assertFalse(self.stub.sample_requests)
        self.assert_not_ready()
        current = self.localization.get_active_session()
        if current is not None:
            self.assertTrue(current.activation_receipt_v2.exact_replay_of(replacement))
        self.assert_original_owner()

    def test_dispatch_cancellation_keeps_type_and_original_request_for_exact_query(self):
        native_dispatch = self.stub.Relocalize
        cancelled = RelocalizationCancelled("cancel after native applied")

        def cancelled_reply(request, timeout):
            native_dispatch(request, timeout)
            raise cancelled

        with patch.object(self.stub, "Relocalize", side_effect=cancelled_reply), \
                self.assertRaises(RelocalizationCancelled) as failure:
            self.relocalize()
        self.assertIs(failure.exception, cancelled)
        pending = self.service._repository.get_assigned_relocalization_intent()
        self.assertEqual(pending["request_id"], REQUEST_ID)
        self.assertEqual(pending["phase"], "backend_outcome_unknown")
        self.assert_not_ready()
        self.service.recover_relocalization_receipt()
        self.assertIsNone(self.service._repository.get_relocalization_intent())
        self.assertEqual(self.localization.get_active_session().status, LocalizationSessionStatus.READY)
        self.assert_original_owner()

    def test_settlement_rename_cancellation_restores_exact_journal_without_overwriting_successor(self):
        intent_path = self.service._repository._relocalization_intent_path
        replace = catalog_module.os.replace
        cancellation = RelocalizationCancelled("cancel after intent tombstone rename")
        observed = {}
        create_foreign_owner = False

        def cancel_after_rename(source, destination):
            if Path(source) == intent_path and Path(destination).name.startswith(".map_relocalization.cleared."):
                observed["original"] = intent_path.read_bytes()
                observed["tombstone"] = Path(destination)
                replace(source, destination)
                if create_foreign_owner:
                    foreign = json.loads(observed["original"])
                    foreign.update(request_id=REQUEST_ID + ":successor",
                                   phase="backend_dispatching", terminal_receipt=None)
                    foreign["assigned_request_v2"]["request_id"] = foreign["request_id"]
                    observed["foreign"] = json.dumps(foreign).encode()
                    intent_path.write_bytes(observed["foreign"])
                raise cancellation
            return replace(source, destination)

        with patch.object(catalog_module.os, "replace", side_effect=cancel_after_rename), \
                self.assertRaises(RelocalizationCancelled) as failure:
            self.relocalize()
        self.assertIs(failure.exception, cancellation)
        self.assertEqual(intent_path.read_bytes(), observed["original"])
        self.assertEqual(self.service._repository.get_assigned_relocalization_intent()["request_id"], REQUEST_ID)
        self.assertFalse(observed["tombstone"].exists())
        self.assertIsNone(self.service.get_relocalized_pose(MAP_NAME))
        self.assert_original_owner()

        samples = len(self.stub.sample_requests)
        with self.forbid_unpaired_pose():
            self.service.recover_relocalization_receipt()
        self.assertGreater(len(self.stub.sample_requests), samples)
        self.assertIsNone(self.service._repository.get_relocalization_intent())
        self.assertIsNotNone(self.service.get_relocalized_pose(MAP_NAME))
        self.assert_original_owner()

        # A concurrent successor's exact journal must never be replaced by rollback.
        create_foreign_owner = True
        with patch.object(catalog_module.os, "replace", side_effect=cancel_after_rename), \
                self.assertRaises(RelocalizationCancelled) as failure:
            self.relocalize()
        self.assertIs(failure.exception, cancellation)
        self.assertEqual(intent_path.read_bytes(), observed["foreign"])
        self.assertEqual(observed["tombstone"].read_bytes(), observed["original"])
        self.assertEqual(self.service._repository.get_assigned_relocalization_intent()["request_id"],
                         REQUEST_ID + ":successor")
        self.assertIsNone(self.service.get_relocalized_pose(MAP_NAME))
        self.assert_original_owner()

    def test_prepared_caller_cannot_demote_same_request_completed_before_pending_session(self):
        set_pending = relocalization_module._set_pending_session
        entered = False
        completed = []
        completed_token = []

        def complete_before_pending(owner, target, request):
            nonlocal entered
            self.assert_unlocked("pending helper entry interleaving")
            if not entered:
                entered = True
                pending = self.service._repository.get_assigned_relocalization_intent()
                self.assertEqual(pending["phase"], "prepared")
                self.assertEqual(pending["assigned_request_v2"], request.to_dict())
                completed.append(self.relocalize())
                completed_token.append(self.localization.get_active_session_token())
                self.assertIsNone(self.service._repository.get_relocalization_intent())
            return set_pending(owner, target, request)

        with patch.object(relocalization_module, "_set_pending_session", side_effect=complete_before_pending), \
                self.assertRaises(GatewayError):
            self.relocalize()
        self.assertEqual(len(completed), 1)
        session, workspace = completed[0]
        self.assertTrue(workspace.localization_ready)
        self.assertEqual(session.status, LocalizationSessionStatus.READY)
        current, epoch = self.localization.get_active_session_token()
        self.assertIs(current, completed_token[0][0])
        self.assertEqual(epoch, completed_token[0][1])
        self.assertEqual(current.status, LocalizationSessionStatus.READY)
        self.assertTrue(current.pose_available)
        self.assertIsNone(self.service._repository.get_relocalization_intent())
        self.assertEqual(self.stub.calls.count("Relocalize"), 1)
        self.assertIsNotNone(self.service.get_relocalized_pose(MAP_NAME))
        self.assert_original_owner()


if __name__ == "__main__":
    unittest.main()
