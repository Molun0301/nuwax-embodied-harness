"""Public assigned exploration pause/save/resume with two actual save owners.

The gateway, workspace, protobuf admission, stores, runtime, localization sample
and READY publication are real. Only native RPCs and outer task/context,
memory/navigation endpoints are simulated. These are Python acceptance checks,
not native geometry, motion-barrier, hardware or RPC/LCM qualification.
"""

import json
import struct
import unittest
from contextlib import ExitStack
from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import patch

import tests.test_assigned_workspace_save as save_fixtures
from tests.test_assigned_asset_storage import MAP_NAME
from tests.test_assigned_workspace_relocalization import RelocalizationNativeStub
from tests.conftest import dense_bytes, fine_bytes

from contracts.map_relocalization import AssignedRelocalizationQuery, AssignedRelocalizationRequest
from contracts.map_workspace import AssetIdentityV2
from contracts.navigation import ExploreAreaRequest
from contracts.runtime_views import LocalizationSessionStatus
from core.errors import GatewayError
from gateways import exploration_checkpoint as gateway_checkpoint
from gateways.runtime import GatewayRuntime
from services.geometry import geometry_runtime_provider as provider_module
from services.mapping import catalog as catalog_module
from services.mapping.asset_store import MapAssetBundleRepository
from services.navigation import exploration_checkpoint as checkpoint_module
from services.navigation.exploration_checkpoint import ExplorationCheckpointStore

pb = provider_module.pb2


def progress_for(request_id, checkpoint, *, elapsed_ns=500_000_000):
    """A stopped native V2 payload retains source, never installed, identity."""
    return json.dumps({
        "schema_version": 2, "map_name": MAP_NAME, "request_id": request_id,
        "map_frame_id": "map", "origin_session_id": checkpoint.source_session_id,
        "origin_session_generation": checkpoint.source_session_generation,
        "origin_frame_generation": 44,  # Native map-frame generation is not the cloud member's zero field
        "configuration": {"profile_version": 1, "source": "wavefront", "strategy": "wavefront",
            "arbitration": "prefer_wavefront", "boundary": None,
            "return_policy": {"maximum_distance_m": None, "reserve_distance_m": 0.0,
                "return_on_completion": False},
            "stop_conditions": {"max_coverage_pct": None, "no_frontier_timeout_ms": None,
                "max_duration_ms": None, "max_goals": None},
            "max_navigation_retries": 1, "blacklist_radius_m": 3.0},
        "navigation_configuration": {"profile_version": 1, "capabilities": None,
            "geometry": None, "body_to_support_z_m": None, "base_frame": ""},
        "elapsed_ns": elapsed_ns, "reached_goals": 1, "mission": None,
        "return_reason": None, "sources": [{"name": "wavefront", "payload": "[1,{}]"}],
    }, separators=(",", ":"))


def _rebind_archive(payload, checkpoint, *, fine):
    """Native fixture bytes receive coherent source ownership before export."""
    offset = 16 if fine else 12
    length = struct.unpack("<Q" if fine else "<I", payload[8:offset])[0]
    document = json.loads(payload[offset:offset + length])
    if fine:
        document["domain"]["session"] = checkpoint.source_session_id
        document["domain"]["generation"] = checkpoint.source_session_generation
        document["revision"] = checkpoint.cloud_receipt.cloud_revision
    else:
        document["session_id"] = checkpoint.source_session_id
        document["session_generation"] = checkpoint.source_session_generation
        document["cloud_revision"] = checkpoint.cloud_receipt.cloud_revision
    document["source_timestamp_ns"] = checkpoint.cloud_receipt.source_timestamp_ns
    encoded = json.dumps(document, separators=(",", ":")).encode()
    return payload[:8] + struct.pack("<Q" if fine else "<I", len(encoded)) + encoded + payload[offset + length:]


class ExplorationNativeStub(RelocalizationNativeStub):
    def __init__(self):
        super().__init__()
        self.progress = None
        self.progress_requests = []
        self.exported_checkpoints = []

    def GetExplorationProgress(self, request, timeout):
        self.visit("GetExplorationProgress")
        self.progress_requests.append(request.request_id)
        assert json.loads(self.progress)["request_id"] == request.request_id
        return pb.ExplorationProgressResponse(checkpoint_json=self.progress)

    def ExportMappingBackendCheckpoint(self, request, timeout):
        self.exported_checkpoints.append(deepcopy(self.response))
        return super().ExportMappingBackendCheckpoint(request, timeout)

    def GetMappingReadiness(self, request, timeout):
        self.visit("GetMappingReadiness")
        return pb.MappingReadiness(ready=True, message=json.dumps({
            "mapping_session_id": self.state.session_id, "map_name": MAP_NAME,
            "session_generation": self.state.session_generation,
            "last_occupancy_revision": self.response.occupancy_receipt.occupancy_revision,
        }))

    def continue_same_session(self):
        """Advance only native map content; the installed session stays live."""
        response = save_fixtures.cloud_response()
        response.checkpoint_id = "d" * 32
        response.source_session_id = self.state.session_id
        response.source_session_generation = self.state.session_generation
        response.occupancy_receipt.CopyFrom(self.state.activation_receipt_v2.target.occupancy_receipt)
        response.cloud_receipt.CopyFrom(self.state.activation_receipt_v2.target.cloud_receipt)
        response.occupancy_receipt.commit_id.sequence += 1
        response.occupancy_receipt.occupancy_revision += 1
        response.occupancy_receipt.source_timestamp_ns += 1
        response.cloud_receipt.commit_id.CopyFrom(response.occupancy_receipt.commit_id)
        response.cloud_receipt.occupancy_revision = response.occupancy_receipt.occupancy_revision
        response.cloud_receipt.cloud_revision += 1
        response.cloud_receipt.source_timestamp_ns = response.occupancy_receipt.source_timestamp_ns
        self.state.revision = response.occupancy_receipt.occupancy_revision
        response.canonical_occupancy_checkpoint.meta.revision = self.state.revision
        response.canonical_occupancy_checkpoint.meta.timestamp_ns = response.occupancy_receipt.source_timestamp_ns
        response.canonical_cloud_checkpoint.source_revision = response.cloud_receipt.cloud_revision
        for member in (response.backend_identity, response.occupancy_identity,
                response.canonical_cloud_checkpoint.member_identity,
                response.fine_map_artifact.member_identity, response.dense_cloud_artifact.member_identity):
            member.checkpoint_id = response.checkpoint_id
            member.occupancy_receipt.CopyFrom(response.occupancy_receipt)
            member.cloud_receipt.CopyFrom(response.cloud_receipt)
        response.fine_map_artifact.id = "e" * 32
        response.dense_cloud_artifact.id = "f" * 32
        fine = _rebind_archive(fine_bytes(), response, fine=True)
        dense = _rebind_archive(dense_bytes(), response, fine=False)
        for reference, payload in ((response.fine_map_artifact, fine), (response.dense_cloud_artifact, dense)):
            reference.size = len(payload)
            self.entries[reference.id] = [deepcopy(reference), bytearray(payload), True]
        self.response = response
        self.terminal = None


class TaskEndpoint:
    """Cancellation requests leave the original task metadata available."""
    def __init__(self):
        self.specs = {}
        self.active = set()
        self.cancelled = []
        self.changes = []

    def add(self, task_id, request):
        self.specs[task_id] = SimpleNamespace(capability_name="explore_area_v2",
            input_payload={"request_id": request.request_id}, metadata={})
        self.active.add(task_id)

    def list_active_tasks(self):
        return [SimpleNamespace(task_id=task_id) for task_id in sorted(self.active)]

    def get_task_spec(self, task_id):
        return self.specs[task_id]

    def update_task_metadata(self, task_id, values):
        self.specs[task_id].metadata.update(deepcopy(values))
        self.changes.append((task_id, deepcopy(values)))

    def cancel_task(self, task_id, reason):
        self.cancelled.append((task_id, reason))
        return task_id in self.active


class TaskContext:
    def __init__(self, task_id, *, remaining=12.5):
        self.task_id = task_id
        self.remaining = remaining
        self.updates = []
        self.on_active = None

    def ensure_active(self):
        if self.on_active:
            self.on_active()

    def update(self, **values):
        self.updates.append(values)

    def timeout_remaining_sec(self):
        return self.remaining


class AssignedExplorationCycleTests(unittest.TestCase):
    new_runtime = save_fixtures.AssignedWorkspaceSaveTests.new_runtime
    new_service = save_fixtures.AssignedWorkspaceSaveTests.new_service
    assert_unlocked = save_fixtures.AssignedWorkspaceSaveTests.assert_unlocked
    assigned_only = save_fixtures.AssignedWorkspaceSaveTests.assigned_only

    def setUp(self):
        save_fixtures.AssignedWorkspaceSaveTests.setUp(self)
        self.stub = ExplorationNativeStub()
        self.provider._stub = self.stub
        self.stub.owner = self.provider
        self.stub.during = self.assert_unlocked
        self.tasks = TaskEndpoint()
        self.gateway = object.__new__(GatewayRuntime)
        self.gateway.config = SimpleNamespace(geometry=SimpleNamespace(pipeline="lio_3d"),
            runtime_data=SimpleNamespace(map_catalog_root=str(self.root)))
        self.gateway.platform_geometry_runtime = self.runtime
        self.gateway.map_workspace_service = self.service
        self.gateway.task_manager = self.tasks
        self.gateway._refresh_workspace_snapshots = lambda: None
        self.gateway.event_bus = SimpleNamespace(build_event=lambda *args, **kwargs: kwargs,
            publish=lambda value: None)
        self.request = ExploreAreaRequest(request_id="exploration:source", map_name=MAP_NAME,
            metadata={"motion_admission": {"old": True}, "purpose": "two-cycle acceptance"})
        self.context = self.add_task("task:first", self.request)
        self.stub.progress = progress_for(self.request.request_id, self.stub.response)

    def add_task(self, task_id, request):
        self.tasks.add(task_id, request)
        return TaskContext(task_id)

    def cloud_only(self):
        guard = ExitStack()
        guard.enter_context(self.assigned_only())
        guard.enter_context(patch.object(self.runtime.providers, "get_exploration_pose",
            side_effect=AssertionError("lio_3d requested an unrelated pose seed")))
        return guard

    def pause(self, context=None, request=None):
        context, request = context or self.context, request or self.request
        pending = gateway_checkpoint.request_pause(self.gateway, request.request_id)
        self.assertEqual(pending["task_id"], context.task_id)
        self.assertTrue(pending["checkpoint_pending"])
        with patch.object(provider_module, "_protobuf_payload_bytes",
                side_effect=AssertionError("save rebuilt the full cloud protobuf")):
            gateway_checkpoint.save_after_stop(self.gateway, context, request)
        saved = ExplorationCheckpointStore(self.root).load(pending["checkpoint_id"])
        self.assertEqual(self.tasks.get_task_spec(context.task_id).metadata["exploration_checkpoint"]["status"], "succeeded")
        return saved

    def resume(self, saved, *, context=None, request_id="exploration:resumed"):
        request = saved.request.model_copy(update={"request_id": request_id,
            "resume_progress_json": saved.geometry_progress_json}, deep=True)
        context = context or self.add_task("task:resume", request)
        gateway_checkpoint.prepare_resume(self.gateway, context, request, saved.checkpoint_id, "acceptance")
        return context, request

    def prepare_second_pause(self):
        first = self.pause()
        resume_context, resumed = self.resume(first)
        before = self.runtime.get_mapping_session_context()
        self.stub.continue_same_session()
        self.assertEqual(self.stub.state.session_id, before["session_id"])
        self.assertEqual(self.stub.state.session_generation, before["session_generation"])
        snapshot = self.mapping.get_latest_snapshot()
        self.mapping.ingest_external_snapshot(map_name=MAP_NAME, source_name="continued-native-stream",
            source_epoch="continued:" + self.stub.state.session_id, occupancy_grid=snapshot.occupancy_grid,
            metadata={"mapping_session_id": self.stub.state.session_id,
                "session_generation": self.stub.state.session_generation, "map_name": MAP_NAME})
        self.tasks.active.discard(self.context.task_id)
        self.stub.progress = progress_for(resumed.request_id, self.stub.response, elapsed_ns=1_000_000_000)
        return first, resume_context, resumed

    def test_public_pause_saves_exact_source_then_resume_requires_actual_assigned_ready(self):
        with self.cloud_only():
            saved = self.pause()
            source = json.loads(saved.geometry_progress_json)
            self.assertEqual(saved.schema_version, 2)
            self.assertIsNone(saved.asset_manifest_sha256)
            self.assertIsNone(saved.localization_seed)
            self.assertNotIn("motion_admission", saved.request.metadata)
            self.assertIsNone(saved.request.resume_progress_json)
            self.assertEqual(saved.mapping_checkpoint_id, self.stub.response.checkpoint_id)
            self.assertEqual(saved.geometry_progress_json, self.stub.progress)
            receipt = self.tasks.get_task_spec(self.context.task_id).metadata["map_asset_finalize"]
            self.assertEqual(receipt["status"], "succeeded")
            self.assertEqual(receipt["identity_schema_version"], 2)
            self.assertEqual(receipt["asset_identity"], saved.asset_identity.model_dump())
            self.assertEqual(receipt["mapping_checkpoint_id"], saved.mapping_checkpoint_id)
            self.assertNotIn("manifest_sha256", receipt)
            self.resume(saved)
        session = self.localization.get_active_session()
        self.assertEqual(session.status, LocalizationSessionStatus.READY)
        self.assertTrue(session.pose_available)
        self.assertEqual(session.asset_identity, saved.asset_identity)
        activation = session.activation_receipt_v2
        self.assertNotEqual(activation.session_id, source["origin_session_id"])
        member = activation.to_dict()["target"]["source_cloud_member"]
        self.assertEqual(member["occupancy_receipt"]["session_id"], source["origin_session_id"])
        self.assertEqual(member["occupancy_receipt"]["session_generation"], source["origin_session_generation"])
        self.assertEqual(member["checkpoint_id"], saved.mapping_checkpoint_id)
        self.assertEqual(member["frame_generation"], 0)
        self.assertEqual(source["origin_frame_generation"], 44)
        self.assertTrue(session.relocalization_terminal_v2.exact_replay_of(self.stub.terminal))
        self.assertIsNotNone(self.service.get_relocalized_pose(MAP_NAME))
        self.assertEqual(self.stub.exports, [2])
        self.assertEqual(len(self.stub.relocalization_requests), 1)
        self.assertTrue(self.stub.sample_requests)

    def test_second_pause_exports_new_checkpoint_and_asset_in_same_resumed_session(self):
        with self.cloud_only():
            first, context, request = self.prepare_second_pause()
            second = self.pause(context, request)
            self.assertNotEqual(second.checkpoint_id, first.checkpoint_id)
            self.assertNotEqual(second.asset_identity, first.asset_identity)
            self.assertNotEqual(second.mapping_checkpoint_id, first.mapping_checkpoint_id)
            self.assertEqual(second.mapping_checkpoint_id, self.stub.response.checkpoint_id)
            self.assertEqual(second.geometry_progress_json, self.stub.progress)
            self.assertEqual(second.request.request_id, request.request_id)
            self.assertNotIn("motion_admission", second.request.metadata)
            self.assertIsNone(second.request.resume_progress_json)
            self.assertEqual(self.stub.exports, [2, 2])
            self.assertEqual([value.checkpoint_id for value in self.stub.exported_checkpoints],
                [first.mapping_checkpoint_id, second.mapping_checkpoint_id])
            reopened = self.service.get_map_asset_bundle(map_name=MAP_NAME)
            self.assertEqual(reopened.asset_identity, second.asset_identity)
            self.assertEqual(reopened.metadata["save_operation_id"], second.checkpoint_id)
            self.assertEqual(self.service._repository.get_map_asset(MAP_NAME).asset_identity, second.asset_identity)
            with self.assertRaises(ValueError):
                ExplorationCheckpointStore(self.root).load(first.checkpoint_id)
            self.assertEqual(ExplorationCheckpointStore(self.root).load(first.checkpoint_id, validate_asset=False), first)
            self.resume(second, context=self.add_task("task:third", second.request), request_id="exploration:third")
        self.assertEqual(len(self.stub.relocalization_requests), 2)
        self.assertEqual(self.localization.get_active_session().asset_identity, second.asset_identity)
        self.assertEqual(self.localization.get_active_session().status, LocalizationSessionStatus.READY)

    def test_lost_reply_resume_retry_retains_request_timeout_and_reads_current_sample(self):
        with self.cloud_only():
            saved = self.pause()
            request = saved.request.model_copy(update={"resume_progress_json": saved.geometry_progress_json}, deep=True)
            context = self.add_task("task:retry", request)
            def metadata_before_dispatch(request):
                metadata = self.tasks.get_task_spec(context.task_id).metadata["resume_relocalization"]
                self.assertEqual(metadata["request_id"], request.request_id)
                self.assertEqual(metadata["logical_timeout_ms"], request.logical_timeout_ms)
                self.assertEqual(metadata["asset_identity"], saved.asset_identity.model_dump())

            self.stub.before_dispatch = metadata_before_dispatch
            self.stub.lose_relocalization_reply = True
            with self.assertRaises(GatewayError):
                gateway_checkpoint.prepare_resume(self.gateway, context, request, saved.checkpoint_id, "acceptance")
            original = AssignedRelocalizationRequest.from_proto(self.stub.relocalization_requests[0])
            self.assertEqual(original.logical_timeout_ms, 12_500)
            retained = deepcopy(self.tasks.get_task_spec(context.task_id).metadata["resume_relocalization"])
            self.assertEqual(retained["checkpoint_id"], saved.checkpoint_id)
            self.assertEqual(retained["asset_identity"], saved.asset_identity.model_dump())
            self.assertEqual(retained["mapping_checkpoint_id"], saved.mapping_checkpoint_id)
            self.assertEqual(retained["request_id"], original.request_id)
            self.assertEqual(retained["logical_timeout_ms"], original.logical_timeout_ms)
            self.assertNotEqual(self.localization.get_active_session().status, LocalizationSessionStatus.READY)
            context.remaining = 2.0
            samples_before = len(self.stub.sample_requests)
            self.stub.sample_mutation = lambda sample: sample.Clear()
            with self.assertRaises((ValueError, GatewayError)):
                gateway_checkpoint.prepare_resume(self.gateway, context, request, saved.checkpoint_id, "acceptance")
            self.assertNotEqual(self.localization.get_active_session().status, LocalizationSessionStatus.READY)
            self.assertEqual(len(self.stub.relocalization_requests), 1)
            self.stub.sample_mutation = None
            gateway_checkpoint.prepare_resume(self.gateway, context, request, saved.checkpoint_id, "acceptance")
        self.assertEqual(len(self.stub.relocalization_requests), 1)
        self.assertEqual(self.tasks.get_task_spec(context.task_id).metadata["resume_relocalization"], retained)
        self.assertEqual(self.stub.calls.count("ActivateMap"), 1)
        self.assertGreater(len(self.stub.sample_requests), samples_before)
        for wire in self.stub.query_requests:
            self.assertTrue(AssignedRelocalizationQuery.from_proto(wire).request.exact_replay_of(original))
        for wire in self.stub.sample_requests:
            self.assertTrue(AssignedRelocalizationRequest.from_proto(wire).exact_replay_of(original))
        self.assertEqual(self.localization.get_active_session().status, LocalizationSessionStatus.READY)

    def test_pause_rejects_native_source_skew_without_publishing_record(self):
        for key, value in (("origin_session_id", "wrong-source"), ("origin_session_generation", 99),
                ("map_frame_id", "odom")):
            with self.subTest(field=key), self.cloud_only():
                document = json.loads(progress_for(self.request.request_id, self.stub.response))
                document[key] = value
                self.stub.progress = json.dumps(document)
                pending = gateway_checkpoint.request_pause(self.gateway, self.request.request_id)
                with self.assertRaises((ValueError, GatewayError)):
                    gateway_checkpoint.save_after_stop(self.gateway, self.context, self.request)
                path = ExplorationCheckpointStore(self.root)._files.direct_path(pending["checkpoint_id"])
                self.assertFalse(path.exists())
                self.assertNotEqual(self.tasks.get_task_spec(self.context.task_id).metadata["exploration_checkpoint"]["status"], "succeeded")


    def test_resume_rejects_wrong_record_asset_checkpoint_or_source_before_activation(self):
        with self.cloud_only():
            saved = self.pause()
            repository = ExplorationCheckpointStore(self.root)
            path = repository._path(saved.checkpoint_id)
            original = path.read_bytes()
            for field in ("asset_identity", "mapping_checkpoint_id", "origin_session_id", "map_frame_id"):
                with self.subTest(field=field):
                    envelope = json.loads(original)
                    document = json.loads(envelope["payload"])
                    if field == "asset_identity":
                        document[field] = AssetIdentityV2(asset_id="e" * 32).model_dump()
                    elif field == "mapping_checkpoint_id":
                        document[field] = "f" * 32
                    else:
                        progress = json.loads(document["geometry_progress_json"])
                        progress[field] = "wrong-owner"
                        document["geometry_progress_json"] = json.dumps(progress)
                    envelope["payload"] = json.dumps(document)
                    path.write_text(json.dumps(envelope))
                    request = saved.request.model_copy(update={"resume_progress_json": document["geometry_progress_json"]})
                    before = list(self.stub.calls)
                    context = self.add_task("task:wrong:" + field, request)
                    with self.assertRaises((ValueError, GatewayError)):
                        gateway_checkpoint.prepare_resume(self.gateway, context, request, saved.checkpoint_id, "acceptance")
                    self.assertEqual(self.stub.calls, before)
                    path.write_bytes(original)

    def test_record_rename_fsync_failure_retry_keeps_exact_record_and_export_identity(self):
        with self.cloud_only():
            pending = gateway_checkpoint.request_pause(self.gateway, self.request.request_id)
            path = ExplorationCheckpointStore(self.root)._files.direct_path(pending["checkpoint_id"])
            original_fsync = catalog_module._fsync_directory
            faults = []

            def fail_record_directory(directory):
                if directory == path.parent and path.exists() and not faults:
                    faults.append(directory)
                    raise OSError("exploration record rename directory fsync failed")
                return original_fsync(directory)

            with patch.object(catalog_module, "_fsync_directory", side_effect=fail_record_directory), \
                    self.assertRaises(OSError):
                gateway_checkpoint.save_after_stop(self.gateway, self.context, self.request)
            self.assertEqual(len(faults), 1)
            written = path.read_bytes()
            retained = ExplorationCheckpointStore(self.root).load(pending["checkpoint_id"], validate_asset=False)
            self.assertNotEqual(self.tasks.get_task_spec(self.context.task_id).metadata["exploration_checkpoint"]["status"], "succeeded")
            with patch.object(checkpoint_module, "_fsync_directory",
                    side_effect=OSError("retained record still not durable")), self.assertRaises(OSError):
                gateway_checkpoint.save_after_stop(self.gateway, self.context, self.request)
            self.assertNotEqual(self.tasks.get_task_spec(self.context.task_id).metadata["exploration_checkpoint"]["status"], "succeeded")
            with patch.object(checkpoint_module, "_fsync_directory", wraps=checkpoint_module._fsync_directory) as retried:
                gateway_checkpoint.save_after_stop(self.gateway, self.context, self.request)
            self.assertIn((path.parent,), [call.args for call in retried.call_args_list])
            self.assertEqual(path.read_bytes(), written)
            self.assertEqual(ExplorationCheckpointStore(self.root).load(pending["checkpoint_id"]), retained)
            self.assertEqual(self.stub.exports, [2])
            self.assertEqual(self.tasks.get_task_spec(self.context.task_id).metadata["exploration_checkpoint"]["status"], "succeeded")

    def test_second_exchange_fsync_failure_retry_uses_retained_candidate_without_reexport(self):
        with self.cloud_only():
            first, context, request = self.prepare_second_pause()
            pending = gateway_checkpoint.request_pause(self.gateway, request.request_id)
            faults = []

            def fail(phase):
                if phase == "before_assigned_directory_fsync" and not faults:
                    faults.append(phase)
                    raise OSError("assigned exchange parent directory fsync failed")

            self.service._asset_repository = MapAssetBundleRepository(str(self.root), fault_injector=fail)
            with self.assertRaises((ValueError, GatewayError)):
                gateway_checkpoint.save_after_stop(self.gateway, context, request)
            self.assertEqual(faults, ["before_assigned_directory_fsync"])
            durable = MapAssetBundleRepository(str(self.root)).get_map_asset_bundle(MAP_NAME)
            self.assertNotEqual(durable.asset_identity, first.asset_identity)
            self.assertEqual(durable.geometry_asset.metadata["mapping_backend_checkpoint"]["checkpoint_id"], self.stub.response.checkpoint_id)
            self.assertEqual(self.stub.exports, [2, 2])
            self.assertFalse(ExplorationCheckpointStore(self.root)._files.direct_path(pending["checkpoint_id"]).exists())
            self.service._asset_repository = MapAssetBundleRepository(str(self.root))
            gateway_checkpoint.save_after_stop(self.gateway, context, request)
            saved = ExplorationCheckpointStore(self.root).load(pending["checkpoint_id"])
            self.assertEqual(saved.asset_identity, durable.asset_identity)
            self.assertEqual(saved.mapping_checkpoint_id, self.stub.response.checkpoint_id)
            self.assertEqual(self.stub.exports, [2, 2])
            self.assertEqual(self.service._repository.get_map_asset(MAP_NAME).asset_identity, durable.asset_identity)

    def test_second_catalog_fsync_failure_retry_keeps_new_asset_and_checkpoint(self):
        with self.cloud_only():
            first, context, request = self.prepare_second_pause()
            pending = gateway_checkpoint.request_pause(self.gateway, request.request_id)
            original_fsync = catalog_module._fsync_directory
            faults = []

            def fail_new_catalog(directory):
                current = self.service._repository.get_map_asset(MAP_NAME)
                if directory == self.service._asset_repository.current_directory(MAP_NAME).parent and current is not None \
                        and current.asset_identity != first.asset_identity and not faults:
                    faults.append(current.asset_identity)
                    raise OSError("new catalog owner directory fsync failed")
                return original_fsync(directory)

            with patch.object(catalog_module, "_fsync_directory", side_effect=fail_new_catalog), \
                    self.assertRaises((ValueError, GatewayError)):
                gateway_checkpoint.save_after_stop(self.gateway, context, request)
            self.assertEqual(len(faults), 1)
            durable = MapAssetBundleRepository(str(self.root)).get_map_asset_bundle(MAP_NAME)
            self.assertEqual(durable.asset_identity, faults[0])
            gateway_checkpoint.save_after_stop(self.gateway, context, request)
            saved = ExplorationCheckpointStore(self.root).load(pending["checkpoint_id"])
            self.assertEqual(saved.asset_identity, durable.asset_identity)
            self.assertEqual(saved.mapping_checkpoint_id, self.stub.response.checkpoint_id)
            self.assertEqual(self.stub.exports, [2, 2])

    def test_stale_native_sample_cannot_complete_resume(self):
        def stale(sample):
            for value in (sample.pose, sample.map_from_odom, sample.odom_from_body):
                value.timestamp_ns = 1

        with self.cloud_only():
            saved = self.pause()
            self.stub.sample_mutation = stale
            with self.assertRaises((ValueError, GatewayError)):
                self.resume(saved)
        session = self.localization.get_active_session()
        self.assertNotEqual(session.status, LocalizationSessionStatus.READY)
        self.assertFalse(session.pose_available)
        self.assertIsNone(self.service.get_relocalized_pose(MAP_NAME))

    def _assert_post_ready_change_rejected(self, change):
        with self.cloud_only():
            saved = self.pause()
            context = self.add_task("task:owner-change", saved.request)
            changed = []

            def after_ready():
                session = self.localization.get_active_session()
                if session is not None and session.status == LocalizationSessionStatus.READY and not changed:
                    change(session)
                    changed.append(True)

            context.on_active = after_ready
            with self.assertRaises((ValueError, GatewayError, RuntimeError)):
                self.resume(saved, context=context)
            self.assertEqual(changed, [True])

    def test_current_session_disappearing_after_ready_rejects_resume(self):
        self._assert_post_ready_change_rejected(lambda session:
            self.localization.clear_active_session(reason="replacement after relocalization returned"))

    def test_equal_runtime_reinstallation_after_ready_rejects_resume(self):
        def reinstall(session):
            with self.runtime._runtime_state_lock:
                previous = self.runtime._map_activation_snapshot_locked()
                self.runtime._map_activation_snapshot = replace(previous, generation=previous.generation + 1)
        self._assert_post_ready_change_rejected(reinstall)

    def test_current_catalog_asset_replacement_after_ready_rejects_resume(self):
        def replace_catalog(session):
            asset = self.service._repository.get_map_asset(MAP_NAME)
            self.service._repository.save_map_asset(asset.model_copy(update={
                "asset_identity": AssetIdentityV2(asset_id="e" * 32)}))
        self._assert_post_ready_change_rejected(replace_catalog)


    def test_current_session_downgraded_after_ready_rejects_resume(self):
        def remove_pose(session):
            with self.localization._lock:
                self.localization._active_session = session.model_copy(update={"status": LocalizationSessionStatus.LOCALIZING, "pose_available": False})
                self.localization._session_mutation_epoch += 1
        self._assert_post_ready_change_rejected(remove_pose)

    def test_current_terminal_removal_after_ready_rejects_resume(self):
        def remove_terminal(session):
            with self.localization._lock:
                values = {name: getattr(session, name) for name in type(session).model_fields}
                values["relocalization_terminal_v2"] = None
                self.localization._active_session = type(session).model_validate(values)
                self.localization._session_mutation_epoch += 1
        self._assert_post_ready_change_rejected(remove_terminal)


    def test_repeated_pause_and_save_retry_reuse_exact_original_record(self):
        with self.cloud_only():
            first = gateway_checkpoint.request_pause(self.gateway, self.request.request_id)
            retry = gateway_checkpoint.request_pause(self.gateway, self.request.request_id)
            self.assertEqual(first, retry)
            gateway_checkpoint.save_after_stop(self.gateway, self.context, self.request)
            repository = ExplorationCheckpointStore(self.root)
            saved = repository.load(first["checkpoint_id"])
            path = repository._path(saved.checkpoint_id)
            before = path.read_bytes()
            gateway_checkpoint.save_after_stop(self.gateway, self.context, self.request)
            self.assertEqual(path.read_bytes(), before)
            self.assertEqual(repository.load(saved.checkpoint_id), saved)
            self.assertEqual(self.stub.exports, [2])

    def test_missing_native_sample_cannot_complete_resume(self):
        with self.cloud_only():
            saved = self.pause()
            self.stub.sample_mutation = lambda sample: sample.Clear()
            with self.assertRaises((ValueError, GatewayError)):
                self.resume(saved)
        session = self.localization.get_active_session()
        self.assertNotEqual(session.status, LocalizationSessionStatus.READY)
        self.assertFalse(session.pose_available)
        self.assertIsNone(self.service.get_relocalized_pose(MAP_NAME))


    def test_ordinary_completion_exports_advanced_current_and_retries_exact_operation(self):
        with self.cloud_only():
            first, context, request = self.prepare_second_pause()
            self.assertNotIn("pause_checkpoint_id", self.tasks.get_task_spec(context.task_id).metadata)
            arguments = {"map_name": MAP_NAME, "trigger": "explore_area_completed",
                "exploration_request_id": request.request_id, "task_id": context.task_id}
            result = self.gateway._finalize_workspace_assets_after_task(**arguments)
            self.assertNotIn("map_asset_finalize_error", result)
            completed = result["finalized_map_asset"]
            self.assertNotEqual(completed.asset_identity, first.asset_identity)
            self.assertEqual(completed.metadata["save_operation_id"], "explore-finalize:" + context.task_id)
            self.assertEqual(completed.geometry_asset.metadata["mapping_backend_checkpoint"]["checkpoint_id"],
                self.stub.response.checkpoint_id)
            self.assertNotEqual(self.stub.response.checkpoint_id, first.mapping_checkpoint_id)
            self.assertEqual(self.stub.exports, [2, 2])
            retry = self.gateway._finalize_workspace_assets_after_task(**arguments)
            self.assertNotIn("map_asset_finalize_error", retry)
            self.assertEqual(retry["finalized_map_asset"].asset_identity, completed.asset_identity)
            self.assertEqual(self.stub.exports, [2, 2])
            self.assertEqual(self.service.get_map_asset_bundle(map_name=MAP_NAME).asset_identity, completed.asset_identity)
            self.assertEqual(retry["map_asset_finalize"]["asset_identity"], completed.asset_identity.model_dump())
            self.assertNotIn("manifest_sha256", retry["map_asset_finalize"])

    def test_prepared_fsync_failure_retry_dispatches_original_request_once(self):
        with self.cloud_only():
            saved = self.pause()
            request = saved.request.model_copy(update={"resume_progress_json": saved.geometry_progress_json})
            context = self.add_task("task:prepared-retry", request)
            original_fsync = catalog_module._fsync_directory
            faults = []

            def fail_prepared(directory):
                path = self.service._repository._relocalization_intent_path
                if path.exists() and json.loads(path.read_text())["phase"] == "prepared" and not faults:
                    faults.append(True)
                    raise OSError("prepared request directory fsync failed before dispatch")
                return original_fsync(directory)

            with patch.object(catalog_module, "_fsync_directory", side_effect=fail_prepared), \
                    self.assertRaises((GatewayError, OSError)):
                gateway_checkpoint.prepare_resume(self.gateway, context, request, saved.checkpoint_id, "acceptance")
            self.assertEqual(faults, [True])
            pending = self.service._repository.get_assigned_relocalization_intent()
            self.assertEqual(pending["phase"], "prepared")
            original = AssignedRelocalizationRequest.from_dict(pending["assigned_request_v2"])
            metadata = deepcopy(self.tasks.get_task_spec(context.task_id).metadata["resume_relocalization"])
            self.assertEqual(original.request_id, metadata["request_id"])
            self.assertEqual(original.logical_timeout_ms, 12_500)
            self.assertFalse(self.stub.relocalization_requests)
            self.assertEqual(self.stub.calls.count("ActivateMap"), 1)
            context.remaining = 1.0
            gateway_checkpoint.prepare_resume(self.gateway, context, request, saved.checkpoint_id, "acceptance")
            self.assertEqual(self.tasks.get_task_spec(context.task_id).metadata["resume_relocalization"], metadata)
            self.assertEqual(len(self.stub.relocalization_requests), 1)
            self.assertTrue(AssignedRelocalizationRequest.from_proto(
                self.stub.relocalization_requests[0]).exact_replay_of(original))
            self.assertEqual(self.stub.calls.count("ActivateMap"), 1)
            self.assertEqual(self.localization.get_active_session().status, LocalizationSessionStatus.READY)

    def test_recovery_cannot_adopt_another_ready_request_on_the_same_activation(self):
        with self.cloud_only():
            saved = self.pause()
            request = saved.request.model_copy(update={"resume_progress_json": saved.geometry_progress_json})
            context = self.add_task("task:recovery-replacement", request)
            self.stub.lose_relocalization_reply = True
            with self.assertRaises(GatewayError):
                gateway_checkpoint.prepare_resume(self.gateway, context, request, saved.checkpoint_id, "acceptance")
            original = AssignedRelocalizationRequest.from_proto(self.stub.relocalization_requests[0])
            checks = []
            replacement = []

            def another_request_after_pending_capture():
                checks.append(True)
                if len(checks) != 2:
                    return
                # An independent operation settles the old pending request, then
                # genuinely relocalizes this same installed map before recovery.
                self.service.recover_relocalization_receipt()
                self.stub.lose_relocalization_reply = False
                self.stub.terminal = None
                session, workspace = self.service.relocalize_active_map(map_name=MAP_NAME,
                    metadata={"business_request_id": "relocalize:independent-successor", "logical_timeout_ms": 9_000})
                self.assertTrue(workspace.localization_ready)
                self.assertEqual(session.status, LocalizationSessionStatus.READY)
                replacement.append(session.relocalization_terminal_v2.request)

            context.on_active = another_request_after_pending_capture
            with self.assertRaisesRegex(ValueError, "READY"):
                gateway_checkpoint.prepare_resume(self.gateway, context, request, saved.checkpoint_id, "acceptance")
            self.assertEqual(len(replacement), 1)
            self.assertTrue(replacement[0].activation_receipt.exact_replay_of(original.activation_receipt))
            self.assertNotEqual(replacement[0].request_id, original.request_id)
            self.assertNotEqual(replacement[0].logical_timeout_ms, original.logical_timeout_ms)
            self.assertEqual(self.stub.calls.count("ActivateMap"), 1)
            self.assertEqual(len(self.stub.relocalization_requests), 2)
            retained = self.tasks.get_task_spec(context.task_id).metadata["resume_relocalization"]
            self.assertEqual(retained["request_id"], original.request_id)
            self.assertEqual(retained["logical_timeout_ms"], original.logical_timeout_ms)
            self.assertTrue(self.localization.get_active_session().relocalization_terminal_v2.request.exact_replay_of(replacement[0]))

    def test_asset_body_reads_and_native_rpcs_stay_outside_runtime_and_catalog_guards(self):
        with self.cloud_only(), ExitStack() as probes:
            for cls, names in ((MapAssetBundleRepository, ("get_map_asset_bundle",
                    "get_validated_map_asset_bundle_snapshot", "finalize_assigned_map_asset_bundle")),
                    (checkpoint_module, ("read_record_text",))):
                for name in names:
                    original = getattr(cls, name)

                    def checked(*args, _original=original, _name=name, **kwargs):
                        self.assert_unlocked(_name)
                        return _original(*args, **kwargs)

                    probes.enter_context(patch.object(cls, name, checked))
            first, context, request = self.prepare_second_pause()
            self.assertIsNotNone(first.asset_identity)
            second = self.pause(context, request)
            self.resume(second, context=self.add_task("task:last", second.request))


if __name__ == "__main__":
    unittest.main()
