"""Public assigned save/load/relocalize and navigation-pose qualification.

Real ordinary-save preparation, protobuf admission, runtime, stores and session
CAS are used. Only native RPC and peripheral endpoints are simulated. These
Python checks do not qualify the native sample's geometry or real RPC/LCM path.
"""

import grpc
import struct
import unittest
from contextlib import ExitStack
from copy import deepcopy
from dataclasses import FrozenInstanceError
from unittest.mock import patch

import tests.test_assigned_workspace_save as save_fixtures
from tests.conftest import LostReply
from tests.test_assigned_asset_storage import MAP_NAME
from tests.test_assigned_relocalization_contract import terminal_dict
from tests.test_assigned_workspace_save import SaveNativeStub

from contracts.map_relocalization import (
    AssignedRelocalizationRequest,
    AssignedRelocalizationTerminal,
)
from contracts.runtime_views import LocalizationSessionStatus
from core import StateNamespace
from core.errors import GatewayError
from services.geometry import geometry_runtime_provider as provider_module

pb = provider_module.pb2
REQUEST_ID = "order-000123"


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
        self.query_status = None
        self.terminals = {}
        self.sample_mutation = None
        self.sample_error = None
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
        elif not self.terminal.request.exact_replay_of(admitted):
            self.terminal = terminal_for_request(admitted, phase=self.terminal_phase)
        self.terminals[admitted.request_id] = self.terminal
        if self.lose_relocalization_reply:
            raise LostReply()
        return self.terminal.to_proto(pb.RelocalizeResult)


    def GetAssignedLocalizationSample(self, request, timeout):
        self.visit("GetAssignedLocalizationSample")
        self.sample_requests.append(deepcopy(request))
        admitted = AssignedRelocalizationRequest.admit_retained(
            provider_module._project_assigned_proto(request, 'request'))
        assert self.terminal is not None and self.terminal.request.exact_replay_of(admitted)
        if self.sample_error is not None:
            raise self.sample_error
        result = sample_for_terminal(self.terminal)
        if self.sample_mutation:
            self.sample_mutation(result)
        if self.during_sample:
            self.during_sample()
        return result


class SampleRpcError(grpc.RpcError):
    def __init__(self, detail): self.detail = detail
    def code(self): return grpc.StatusCode.FAILED_PRECONDITION
    def details(self): return self.detail


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

    def relocalize(self):
        return self.service.relocalize_active_map(map_name=MAP_NAME, metadata={"logical_timeout_ms": 10_000})

    def forbid_unpaired_pose(self):
        guard = ExitStack()
        guard.enter_context(self.assigned_only())
        for owner, names in (
            (self.provider, ("get_current_pose", "get_frame_tree", "get_localization_state")),
            (self.localization, ("refresh",)),
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
        for wire in self.stub.sample_requests:
            self.assertTrue(AssignedRelocalizationRequest.from_proto(wire).exact_replay_of(request))
        return request











    def test_native_sample_publishes_ready_for_current_activation(self):
        def before_dispatch(request):
            self.assertTrue(self.service._repository._activation_executor_guard_state.mutex._is_owned())
            self.assertEqual(request.asset_identity, self.saved.asset_identity)
            self.assertFalse(any((self.root / name).exists() for name in (
                "map_relocalization.json", "map_relocalization_receipt.json")))
        self.stub.before_dispatch = before_dispatch
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
        self.assertEqual(self.stub.calls.count("Relocalize"), 1)
        self.assertGreaterEqual(self.stub.calls.count("GetAssignedLocalizationSample"), 1)
        self.assertNotIn("ActivateMap", self.stub.calls)
        self.assertNotIn("StopMapping", self.stub.calls)
        request = self.assert_original_owner()
        self.assertEqual(request.activation_receipt, session.activation_receipt_v2)



    def test_stale_same_frame_pose_never_publishes_ready_or_navigation_pose(self):
        self.stub.sample_error = SampleRpcError("ASSIGNED_SAMPLE_POSE_OR_TF_STALE")
        with self.forbid_unpaired_pose(), self.assertRaises(GatewayError):
            self.relocalize()
        self.assert_not_ready()
        self.assert_original_owner()


    def test_missing_exact_terminal_in_sample_cannot_publish_ready(self):
        self.stub.sample_error = SampleRpcError("ASSIGNED_SAMPLE_TERMINAL_UNAVAILABLE")
        with self.assertRaises(GatewayError):
            self.relocalize()
        self.assert_not_ready()
        self.assert_original_owner()

    def test_native_correction_replacement_cannot_publish_ready(self):
        self.stub.sample_error = SampleRpcError("ASSIGNED_SAMPLE_CORRECTION_REPLACED")
        with self.assertRaises(GatewayError):
            self.relocalize()
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
        pose = self.service.get_relocalized_pose(MAP_NAME)
        pose.position.x = 999
        self.assertEqual(self.service.get_relocalized_pose(MAP_NAME).position.x, 1.5)




    def test_navigation_pose_getter_propagates_current_native_rejection(self):
        self.relocalize()
        before = len(self.stub.sample_requests)
        reasons = ("ASSIGNED_SAMPLE_POSE_OR_TF_STALE", "ASSIGNED_SAMPLE_CORRECTION_REPLACED",
                   "ASSIGNED_SAMPLE_MAPPING_OWNER_REPLACED")
        with self.forbid_unpaired_pose():
            for reason in reasons:
                self.stub.sample_error = SampleRpcError(reason)
                self.assertIsNone(self.service.get_relocalized_pose(MAP_NAME))
        self.assertEqual(len(self.stub.sample_requests), before + len(reasons))
        self.assert_original_owner()

    def test_current_provider_mapping_owner_changed_during_sample_cannot_publish_ready(self):
        def replace_owner():
            with self.provider._map_state_lock:
                self.provider._mapping_identity = ("successor", MAP_NAME, 999)
        self.stub.during_sample = replace_owner
        with self.assertRaises(GatewayError):
            self.relocalize()
        self.assert_not_ready()
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














if __name__ == "__main__":
    unittest.main()
