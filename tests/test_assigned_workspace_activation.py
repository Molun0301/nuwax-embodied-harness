"""Public schema-2 workspace integration with real stores, runtime and session CAS.

Only native protobuf/RPC, memory and navigation endpoints are simulated. No
native process, hardware, legacy hash authority or unrelated suite is needed.
"""

import hashlib
import json
import tempfile
import threading
import unittest
from contextlib import ExitStack
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from test_assigned_activation_loader import LoaderStub, LostReply, ack, predecessor
from test_assigned_asset_storage import (
    ASSET_ID,
    CHECKPOINT_ID,
    MAP_NAME,
    STAMP,
    bundle_fixture,
    checkpoint_fixture,
    write,
)
from test_checkpoint_identity_v2 import response_fixture

from contracts.geometry import Pose
from contracts.map_relocalization import AssignedRelocalizationRequest, current_assigned_relocalization_policy
from contracts.map_workspace import AssetIdentityV2, MapAsset, MapAssetBundle, MapAssetStatus
from contracts.runtime_views import LocalizationSession, LocalizationSessionStatus, LocalizationSnapshot
from core import StateNamespace, StateStore
from core.errors import GatewayError
from services.geometry import geometry_runtime_provider as provider_module
from services.geometry import runtime as runtime_module
from services.geometry.mapping_backend_checkpoint_store import MappingBackendCheckpointStore
from services.localization.service import LocalizationService
from services.mapping import catalog as catalog_module
from services.mapping.asset_store import MapAssetBundleRepository
from services.mapping.catalog import MapCatalogRepository
from services.mapping.service import MappingService
from skills.map_workspace import MapWorkspaceService


class ActivationCancelled(BaseException):
    """Cancellation sentinel distinct from ordinary recoverable exceptions."""


class WorkspaceNativeStub(LoaderStub):
    """A tiny exact-request native ledger; callers must stop a different owner."""

    def __init__(self):
        super().__init__(response_fixture())
        self.ledger = {}
        self.activation_requests = []
        self.stop_requests = []

    def ActivateMap(self, request, timeout):
        self.visit("ActivateMap")
        self.last_request = deepcopy(request)
        self.activation_requests.append(deepcopy(request))
        key = request.activation_request_id
        if key in self.ledger:
            assert self.ledger[key].SerializeToString() == request.SerializeToString()
            replay = True
        else:
            assert not self.state.active, "caller did not stop its captured predecessor"
            self.ledger[key] = deepcopy(request)
            self.installs += 1
            replay = False
        self.state = ack(request, replay=replay)
        if self.error is not None:
            raise self.error
        return self.reply(request) if self.reply else deepcopy(self.state)

    def StopMapping(self, request, timeout):
        self.stop_requests.append(deepcopy(request))
        return super().StopMapping(request, timeout)


class MemoryEndpoint:
    def __init__(self):
        self.active = None
        self.calls = []

    def get_active_library_name(self):
        return self.active

    def ensure_memory_library(self, *, library_name):
        self.calls.append(("ensure", library_name))
        return {"created": True}

    def activate_memory_library(self, **kwargs):
        self.calls.append(("activate", kwargs))
        self.active = kwargs["library_name"]
        return {"active_library_name": self.active}

    def get_summary(self):
        return None


class AssignedWorkspaceActivationTests(unittest.TestCase):
    def setUp(self):
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.root = Path(self.stack.enter_context(tempfile.TemporaryDirectory()))
        self.store = StateStore()
        self.memory = MemoryEndpoint()
        self.navigation = SimpleNamespace(
            load_forbidden_zones_for_map=lambda **kwargs: None,
            get_latest_navigation_context=lambda: None,
            get_latest_exploration_context=lambda: None,
        )
        self.stub = WorkspaceNativeStub()
        self.provider = provider_module.GeometryRuntimeProvider(None)
        self.provider._connected = True
        self.provider._stub = self.stub
        self.stub.owner = self.provider
        self.runtime = self.new_runtime()
        self.mapping = MappingService(provider_owner=SimpleNamespace(), state_store=self.store)
        self.localization = LocalizationService(provider_owner=SimpleNamespace(), state_store=self.store)
        self.service = self.new_service()
        self.bundle = self.publish_map(MAP_NAME, ASSET_ID)
        self.stub.during = self.assert_unlocked

    def new_runtime(self):
        runtime = object.__new__(runtime_module.PlatformGeometryRuntime)
        runtime._runtime_state_lock = threading.RLock()
        runtime._mapping_lifecycle_lock = threading.RLock()
        runtime._provider_attach_lock = threading.RLock()
        runtime._provider_attach_epoch = 2
        runtime._remote_provider = self.provider
        runtime._provider_attach_stopping = False
        return runtime

    def new_service(self):
        return MapWorkspaceService(
            catalog_root=str(self.root),
            geometry_runtime=self.runtime,
            mapping_service=self.mapping,
            localization_service=self.localization,
            memory_service=self.memory,
            navigation_service=self.navigation,
            state_store=self.store,
        )

    def publish_map(self, name, identity):
        repository = MapAssetBundleRepository(str(self.root))
        document = bundle_fixture(identity).model_dump(mode="python", warnings=False)
        document["map_name"] = name
        document["geometry_asset"]["map_name"] = name
        document["snapshot"]["map_name"] = name
        for layer in ("occupancy_grid", "cost_map"):
            document["snapshot"][layer]["map_id"] = name
        bundle = MapAssetBundle.model_validate(document)
        stage = repository.create_staging_directory(name)
        checkpoint, occupancy, cloud = checkpoint_fixture()
        MappingBackendCheckpointStore(stage).publish(checkpoint, occupancy_asset=occupancy, cloud_asset=cloud)
        write(stage / "geometry/test/state.bin", b"state")
        write(stage / "geometry/test/map.yaml", b"image: map.pgm\nresolution: 0.125\n")
        write(stage / "geometry/test/map.pgm", b"P5\n2 1\n255\n\x00\x64")
        finalized = repository.finalize_assigned_map_asset_bundle(bundle, staging_dir=stage, asset_identity=identity)
        self.service._repository.save_map_asset(
            MapAsset(
                map_name=name,
                display_name=name,
                bound_memory_library_name=name,
                identity_schema_version=2,
                asset_identity=identity,
                finalized=True,
                finalized_at=STAMP,
                frame_id="map",
            )
        )
        return finalized

    def assert_unlocked(self, operation):
        for name in ("_runtime_state_lock", "_mapping_lifecycle_lock", "_provider_attach_lock"):
            self.assertFalse(getattr(self.runtime, name)._is_owned(), (operation, name))
        for name in ("_activation_guard_state", "_activation_executor_guard_state"):
            self.assertFalse(getattr(self.service._repository, name).mutex._is_owned(), (operation, name))

    def no_legacy_or_hash(self):
        stack = ExitStack()
        for name in hashlib.algorithms_guaranteed | {"new", "file_digest"}:
            if hasattr(hashlib, name):
                stack.enter_context(
                    patch.object(hashlib, name, side_effect=AssertionError("application hash: " + name))
                )
        for name in ("_activate_legacy_map", "_build_workspace"):
            stack.enter_context(
                patch.object(self.service, name, side_effect=AssertionError("legacy workspace: " + name))
            )
        for name in (
            "_validate_bundle_manifest",
            "_materialize_assigned_checkpoint_pcd_asset",
            "_install_map_activation_snapshot_locked",
            "get_authoritative_mapping_session_state",
            "stop_mapping",
        ):
            stack.enter_context(patch.object(self.runtime, name, side_effect=AssertionError("legacy runtime: " + name)))
        for name in ("ensure_active_session", "compare_and_set_active_session", "refresh"):
            if hasattr(self.localization, name):
                stack.enter_context(
                    patch.object(self.localization, name, side_effect=AssertionError("legacy localization: " + name))
                )
        stack.enter_context(
            patch.object(
                self.localization, "_get_localization_provider", side_effect=AssertionError("old pose/provider refresh")
            )
        )
        return stack

    def activate(self, name=MAP_NAME, **kwargs):
        return self.service.activate_map(map_name=name, **kwargs)

    def assert_consistent(self, workspace, name=MAP_NAME):
        self.assertEqual(workspace.active_map_name, name)
        self.assertTrue(workspace.map_loaded)
        self.assertFalse(workspace.localization_ready)
        self.assertIsNone(workspace.latest_localization_snapshot)
        session = workspace.active_localization_session
        self.assertEqual(session.identity_schema_version, 2)
        self.assertEqual(session.status, LocalizationSessionStatus.LOCALIZING)
        self.assertFalse(session.pose_available)
        self.assertIsNone(session.relocalization_terminal_v2)
        self.assertIs(session, self.localization.get_active_session_token()[0])
        asset = self.service._repository.get_map_asset(name)
        self.assertEqual(asset.status, MapAssetStatus.LOCALIZING)
        self.assertEqual(asset.asset_identity, session.asset_identity)
        self.assertTrue(asset.active)
        self.assertIsNone(asset.asset_manifest_sha256)
        self.assertEqual(self.service._repository.get_active_map_name(), name)
        self.assertIsNone(self.service._repository.get_map_activation())
        self.assertFalse(self.runtime.is_map_asset_activation_recovery_pending())
        target = self.runtime.capture_assigned_map_activation_target(map_name=name, asset_identity=asset.asset_identity)
        self.assertEqual(session.activation_receipt_v2, target.activation_receipt)
        self.assertEqual(
            self.mapping.get_latest_snapshot().metadata["activation_receipt_v2"], target.activation_receipt.to_dict()
        )

    def test_public_activation_installs_real_runtime_catalog_mapping_and_localizing_session_without_old_pose(self):
        self.localization._active_session = LocalizationSession(
            session_id="old", map_name="old", status=LocalizationSessionStatus.READY, pose_available=True
        )
        self.localization._latest_snapshot = LocalizationSnapshot(source_name="old", current_pose=Pose(frame_id="map"))
        with self.no_legacy_or_hash():
            workspace = self.activate(load_memory_history=False)
        self.assert_consistent(workspace)
        self.assertIsNone(self.localization.get_latest_snapshot())
        self.assertEqual(self.stub.calls.count("ActivateMap"), 1)
        self.assertNotIn("StopMapping", self.stub.calls)
        self.assertEqual(self.memory.calls[-1][1]["load_history"], False)
        self.assertFalse(
            any(
                "pose" in field.name or "seed" in field.name or "sha256" in field.name
                for field, _value in self.stub.last_request.ListFields()
            )
        )

    def test_public_explicit_a_to_b_stops_exact_predecessor_before_dispatch(self):
        second = self.publish_map("second-map", AssetIdentityV2(asset_id="e" * 32))
        first = self.activate()
        old_owner = first.active_localization_session.activation_receipt_v2
        self.stub.calls.clear()
        with self.no_legacy_or_hash():
            result = self.activate(second.map_name)
        self.assert_consistent(result, second.map_name)
        self.assertEqual(self.stub.stop_requests[-1].session_id, old_owner.to_dict()["request"]["session_id"])
        self.assertEqual(self.stub.calls.count("StopMapping"), 1)
        self.assertLess(self.stub.calls.index("StopMapping"), self.stub.calls.index("ActivateMap"))
        self.assertFalse(self.service._repository.get_map_asset(MAP_NAME).active)
        self.assertEqual(self.stub.installs, 2)

    def test_repeat_same_owner_preserves_installation_session_epoch_and_map_snapshot(self):
        first = self.activate()
        target = self.runtime._map_activation_snapshot_read()
        session, epoch = self.localization.get_active_session_token()
        mapping = self.mapping.get_latest_snapshot()
        self.stub.calls.clear()
        with self.no_legacy_or_hash(), patch.object(
            self.mapping, "ingest_external_snapshot", side_effect=AssertionError("map imported again")
        ), patch.object(self.memory, "get_summary", wraps=self.memory.get_summary) as summary:
            repeated = self.activate()
        self.assertEqual(summary.call_count, 1)
        self.assert_consistent(repeated)
        self.assertIs(self.runtime._map_activation_snapshot_read(), target)
        self.assertIs(self.localization.get_active_session_token()[0], session)
        self.assertEqual(self.localization.get_active_session_token()[1], epoch)
        self.assertIs(self.mapping.get_latest_snapshot(), mapping)
        self.assertEqual(repeated.active_localization_session.session_id, first.active_localization_session.session_id)
        self.assertNotIn("ActivateMap", self.stub.calls)
        self.assertNotIn("StopMapping", self.stub.calls)

    def test_repeat_same_owner_handler_summarizes_service_and_refresh_only(self):
        from gateways.runtime import GatewayRuntime
        from skills.handlers.localization_skills import handle_activate_map

        self.activate()
        self.stub.calls.clear()
        events = []
        # Runtime/perception effects stay at the existing offline fixture boundary.
        # The handler and workspace refresh bodies are the real implementations.
        ctx = SimpleNamespace(
            map_workspace_service=self.service,
            ensure_geometry_runtime_started=lambda reason: events.append(("ensure", reason)),
            sync_perception_runtime_for_memory_state=lambda **kwargs: events.append(("perception", kwargs)),
        )

        def refresh(**kwargs):
            events.append(("refresh", kwargs))
            return GatewayRuntime._refresh_workspace_snapshots(ctx, **kwargs)

        ctx.refresh_workspace_snapshots = refresh
        with patch.object(self.memory, "get_summary", wraps=self.memory.get_summary) as summary:
            result = handle_activate_map(ctx, {"map_name": MAP_NAME})
        self.assertEqual(summary.call_count, 2)
        self.assert_consistent(result["active_workspace"])
        self.assertEqual(events, [
            ("ensure", "activate_map"),
            ("refresh", {"refresh_localization": False}),
            ("perception", {"enabled": True}),
        ])
        self.assertEqual(self.stub.installs, 1)
        self.assertNotIn("ActivateMap", self.stub.calls)
        self.assertNotIn("StopMapping", self.stub.calls)

    def test_complete_original_request_is_durable_before_any_native_mutation(self):
        self.stub.state = predecessor(active=True)
        observations = []

        def during(operation):
            self.assert_unlocked(operation)
            if operation == "StopMapping" or operation == "ActivateMap" or operation.startswith("Transfer:"):
                pending = MapCatalogRepository(str(self.root)).get_map_activation()
                self.assertIsNotNone(pending)
                request = pending["activation_request"]
                self.assertEqual(
                    set(request),
                    {
                        "activation_request_id",
                        "session_id",
                        "map_name",
                        "asset_identity",
                        "checkpoint_id",
                        "expected_runtime_generation",
                    },
                )
                self.assertEqual(request["asset_identity"], ASSET_ID.model_dump())
                self.assertEqual(request["checkpoint_id"], CHECKPOINT_ID)
                self.assertEqual(request["expected_runtime_generation"], 7)
                self.assertEqual(pending["predecessor"]["session_id"], "predecessor")
                observations.append((operation, deepcopy(request)))

        self.stub.during = during
        workspace = self.activate()
        expected = workspace.active_localization_session.activation_receipt_v2.to_dict()["request"]
        self.assertTrue(observations)
        self.assertTrue(all(request == expected for _, request in observations))

    def test_journal_write_failure_prevents_all_native_mutations(self):
        with patch.object(
            self.service._repository, "begin_assigned_map_activation", side_effect=OSError("disk full")
        ), self.assertRaises(OSError):
            self.activate()
        self.assertEqual(self.stub.calls, ["GetMappingState"])
        self.assertIsNone(self.runtime._map_activation_snapshot_read().bundle)
        self.assertIsNone(self.localization.get_active_session_token()[0])
        self.assertIsNone(self.service._repository.get_map_activation())

    def test_postdispatch_mapping_failure_keeps_exact_request_and_fence_then_reconciles_without_dispatch(self):
        with patch.object(
            self.mapping, "ingest_external_snapshot", side_effect=RuntimeError("map publication failed")
        ), self.assertRaises(GatewayError):
            self.activate()
        pending = self.service._repository.get_map_activation()
        self.assertEqual(pending["phase"], "reconciling")
        self.assertTrue(self.runtime.is_map_asset_activation_recovery_pending())
        self.assertIsNone(self.localization.get_active_session_token()[0])
        self.assertIsNone(self.service.get_active_workspace())
        request = deepcopy(pending["activation_request"])
        self.stub.calls.clear()
        with self.no_legacy_or_hash():
            restored = self.service.restore_active_workspace()
        self.assert_consistent(restored)
        self.assertEqual(restored.active_localization_session.activation_receipt_v2.to_dict()["request"], request)
        self.assertEqual(self.stub.calls, ["GetMappingState"])
        self.assertEqual(self.stub.installs, 1)

    def test_unknown_is_retained_until_explicit_exact_retry_and_never_reexecutes_in_exception(self):
        self.stub.error = LostReply()
        with self.assertRaises(GatewayError):
            self.activate()
        pending = self.service._repository.get_map_activation()
        request = deepcopy(pending["activation_request"])
        self.assertEqual(pending["phase"], "backend_dispatching")
        self.assertTrue(self.runtime.is_map_asset_activation_recovery_pending())
        self.assertEqual(self.stub.calls.count("ActivateMap"), 1)
        self.assertIsNone(self.runtime._map_activation_snapshot_read().bundle)
        self.assertIsNone(self.localization.get_active_session_token()[0])
        self.stub.error = None
        self.stub.calls.clear()
        with self.no_legacy_or_hash():
            retried = self.activate()
        self.assert_consistent(retried)
        self.assertEqual(retried.active_localization_session.activation_receipt_v2.to_dict()["request"], request)
        self.assertEqual(self.stub.calls.count("ActivateMap"), 1)
        self.assertNotIn("StopMapping", self.stub.calls)
        self.assertEqual(self.stub.installs, 1)
        self.assertEqual(
            self.stub.activation_requests[0].SerializeToString(), self.stub.activation_requests[1].SerializeToString()
        )

    def test_sealed_actual_state_can_restore_unknown_without_native_reexecution(self):
        self.stub.error = LostReply()
        with self.assertRaises(GatewayError):
            self.activate()
        request = deepcopy(self.service._repository.get_map_activation()["activation_request"])
        self.stub.calls.clear()
        self.service = self.new_service()
        self.assertEqual(self.stub.calls, [])
        self.assertTrue(self.runtime.is_map_asset_activation_recovery_pending())
        with self.no_legacy_or_hash():
            restored = self.service.restore_active_workspace()
        self.assert_consistent(restored)
        self.assertEqual(restored.active_localization_session.activation_receipt_v2.to_dict()["request"], request)
        self.assertEqual(self.stub.calls, ["GetMappingState"])
        self.assertEqual(self.stub.installs, 1)

    def test_restore_without_current_sealed_evidence_never_dispatches_or_rewrites_request(self):
        self.stub.error = LostReply()
        with self.assertRaises(GatewayError):
            self.activate()
        original = self.service._repository.get_map_activation()
        self.stub.state = predecessor()
        self.stub.calls.clear()
        self.service = self.new_service()
        self.assertEqual(self.stub.calls, [])
        with self.assertRaises(GatewayError):
            self.service.restore_active_workspace()
        self.assertEqual(self.stub.calls, ["GetMappingState"])
        self.assertEqual(self.service._repository.get_map_activation(), original)
        self.assertTrue(self.runtime.is_map_asset_activation_recovery_pending())

    def test_completed_restart_restore_observes_state_but_never_dispatches(self):
        original = self.activate()
        session, epoch = self.localization.get_active_session_token()
        self.stub.calls.clear()
        self.service = self.new_service()
        self.assertEqual(self.stub.calls, [])
        with self.no_legacy_or_hash():
            restored = self.service.restore_active_workspace()
        self.assert_consistent(restored)
        self.assertIs(restored.active_localization_session, session)
        self.assertEqual(self.localization.get_active_session_token()[1], epoch)
        self.assertEqual(
            restored.active_localization_session.activation_receipt_v2,
            original.active_localization_session.activation_receipt_v2,
        )
        self.assertEqual(self.stub.calls, ["GetMappingState"])

    def test_public_getters_never_dispatch_refresh_or_use_legacy_session_cas(self):
        self.activate()
        session, epoch = self.localization.get_active_session_token()
        self.stub.calls.clear()
        with self.no_legacy_or_hash():
            workspace = self.service.get_active_workspace()
            asset = self.service.get_map(MAP_NAME)
            maps = self.service.list_maps()
            active_session = self.service.get_active_localization_session()
            self.assertEqual(self.service.get_active_workspace_identity()["active_map_name"], MAP_NAME)
        self.assertIs(workspace.active_localization_session, session)
        self.assertIs(active_session, session)
        self.assertEqual(self.localization.get_active_session_token()[1], epoch)
        self.assertEqual(asset.status, MapAssetStatus.LOCALIZING)
        self.assertEqual(maps[0].status, MapAssetStatus.LOCALIZING)
        self.assertEqual(self.stub.calls, [])

    def test_map_asset_reads_and_checkpoint_preparation_run_outside_runtime_catalog_executor_locks(self):
        read_bundle = self.service._asset_repository.get_map_asset_bundle
        load_checkpoint = MappingBackendCheckpointStore.load

        def read(*args, **kwargs):
            self.assert_unlocked("asset bundle read")
            return read_bundle(*args, **kwargs)

        def load(store, *args, **kwargs):
            self.assert_unlocked("checkpoint read")
            return load_checkpoint(store, *args, **kwargs)

        with patch.object(self.service._asset_repository, "get_map_asset_bundle", side_effect=read), patch.object(
            MappingBackendCheckpointStore, "load", load
        ):
            self.assert_consistent(self.activate())

    def test_rpc_map_and_state_publication_callbacks_run_without_runtime_catalog_executor_guards(self):
        publications = []
        original = self.store.write

        def write_unlocked(namespace, key, value, **kwargs):
            self.assert_unlocked(("state publication", namespace, key))
            publications.append((namespace, key))
            return original(namespace, key, value, **kwargs)

        with patch.object(self.store, "write", side_effect=write_unlocked):
            self.assert_consistent(self.activate())
        self.assertIn((StateNamespace.LOCALIZATION, "active_session"), publications)
        self.assertIn((StateNamespace.MAP, "active_workspace"), publications)
        self.assertTrue(
            any(namespace == StateNamespace.MAP and key != "active_workspace" for namespace, key in publications)
        )

    def test_mapping_callback_supersession_is_detected_and_cannot_publish_success(self):
        original = self.store.write

        def replace_mapping(namespace, key, value, **kwargs):
            result = original(namespace, key, value, **kwargs)
            if namespace == StateNamespace.MAP and kwargs.get("metadata", {}).get("kind") == "latest_map":
                self.assert_unlocked("map callback")
                self.mapping._latest_snapshot = value.model_copy(update={"map_name": "successor"})
            return result

        with patch.object(self.store, "write", side_effect=replace_mapping), self.assertRaises(GatewayError):
            self.activate()
        self.assertEqual(self.mapping.get_latest_snapshot().map_name, "successor")
        self.assertEqual(self.service._repository.get_map_activation()["phase"], "reconciling")
        self.assertTrue(self.runtime.is_map_asset_activation_recovery_pending())
        self.assertIsNone(self.localization.get_active_session_token()[0])
        self.assertIsNone(self.store.read(StateNamespace.MAP, "active_workspace"))

    def test_localization_preparation_supersession_loses_cas_and_retains_recovery(self):
        prepare = self.localization.prepare_assigned_session_update

        def replace_session(**kwargs):
            result = prepare(**kwargs)
            with self.localization._lock:
                self.localization._set_active_session_locked(
                    LocalizationSession(session_id="successor", map_name="successor")
                )
            return result

        with patch.object(
            self.localization, "prepare_assigned_session_update", side_effect=replace_session
        ), self.assertRaises(GatewayError):
            self.activate()
        self.assertEqual(self.localization.get_active_session_token()[0].session_id, "successor")
        self.assertEqual(self.service._repository.get_map_activation()["phase"], "reconciling")
        self.assertTrue(self.runtime.is_map_asset_activation_recovery_pending())
        self.assertIsNone(self.service.get_active_workspace())

    def test_localization_callback_can_activate_successor_and_old_call_never_returns_stale_workspace(self):
        self.publish_map("second-map", AssetIdentityV2(asset_id="e" * 32))
        original = self.store.write
        successor = []

        def activate_successor(namespace, key, value, **kwargs):
            result = original(namespace, key, value, **kwargs)
            if namespace == StateNamespace.LOCALIZATION and key == "active_session" and value.map_name == MAP_NAME:
                self.assert_unlocked("localization callback")
                successor.append(self.activate("second-map"))
            return result

        with patch.object(self.store, "write", side_effect=activate_successor), self.assertRaises(GatewayError):
            self.activate()
        self.assertEqual(len(successor), 1)
        self.assert_consistent(successor[0], "second-map")
        self.assertEqual(self.store.read(StateNamespace.MAP, "active_workspace").value.active_map_name, "second-map")
        self.assertEqual(self.stub.installs, 2)

    def test_failure_to_clear_runtime_fence_retains_durable_journal(self):
        with patch.object(self.runtime, "complete_assigned_map_activation", return_value=False), self.assertRaises(
            GatewayError
        ):
            self.activate()
        self.assertTrue(self.runtime.is_map_asset_activation_recovery_pending())
        self.assertIsNotNone(self.service._repository.get_map_activation())
        self.assertIsNone(self.service.get_active_workspace())

    def test_cold_restore_reconstructs_runtime_mapping_and_session_from_actual_sealed_owner_without_dispatch(self):
        original = self.activate()
        request = original.active_localization_session.activation_receipt_v2.to_dict()["request"]
        self.runtime = self.new_runtime()
        self.mapping = MappingService(provider_owner=SimpleNamespace(), state_store=self.store)
        self.localization = LocalizationService(provider_owner=SimpleNamespace(), state_store=self.store)
        self.memory = MemoryEndpoint()
        self.stub.calls.clear()
        self.service = self.new_service()
        self.assertEqual(self.stub.calls, [])
        self.assertIsNone(self.service.get_active_workspace())
        with self.no_legacy_or_hash():
            restored = self.service.restore_active_workspace()
        self.assert_consistent(restored)
        self.assertEqual(restored.active_localization_session.activation_receipt_v2.to_dict()["request"], request)
        self.assertTrue(self.stub.calls)
        self.assertTrue(all(call == "GetMappingState" for call in self.stub.calls))
        self.assertEqual(self.stub.installs, 1)

    def test_active_workspace_callback_switch_cannot_make_outer_activation_return_stale_workspace(self):
        self.publish_map("second-map", AssetIdentityV2(asset_id="e" * 32))
        original = self.store.write
        successor = []

        def activate_successor(namespace, key, value, **kwargs):
            result = original(namespace, key, value, **kwargs)
            if namespace == StateNamespace.MAP and key == "active_workspace" and value.active_map_name == MAP_NAME:
                self.assert_unlocked("workspace callback")
                successor.append(self.activate("second-map"))
            return result

        with patch.object(self.store, "write", side_effect=activate_successor), self.assertRaises(GatewayError):
            self.activate()
        self.assertEqual(len(successor), 1)
        self.assert_consistent(successor[0], "second-map")
        self.assertEqual(self.store.read(StateNamespace.MAP, "active_workspace").value.active_map_name, "second-map")

    def test_same_asset_fresh_native_installation_updates_mapping_receipt_instead_of_deduplicating_old_owner(self):
        first = self.activate()
        old_session = first.active_localization_session
        old_snapshot = self.mapping.get_latest_snapshot()
        old_installation = self.runtime._map_activation_snapshot_read()
        self.stub.state = predecessor()
        self.stub.calls.clear()
        with self.no_legacy_or_hash():
            second = self.activate()
        self.assert_consistent(second)
        self.assertIsNot(second.active_localization_session, old_session)
        self.assertNotEqual(second.active_localization_session.activation_receipt_v2, old_session.activation_receipt_v2)
        self.assertIsNot(self.runtime._map_activation_snapshot_read(), old_installation)
        self.assertIsNot(self.mapping.get_latest_snapshot(), old_snapshot)
        self.assertEqual(self.stub.calls.count("ActivateMap"), 1)
        self.assertEqual(self.stub.installs, 2)

    def test_sealed_unknown_reconciliation_does_not_leave_provider_attempt_blocking_next_map(self):
        self.publish_map("second-map", AssetIdentityV2(asset_id="e" * 32))
        self.stub.error = LostReply()
        with self.assertRaises(GatewayError):
            self.activate()
        self.stub.error = None
        restored = self.service.restore_active_workspace()
        self.assert_consistent(restored)
        self.stub.calls.clear()
        with self.no_legacy_or_hash():
            successor = self.activate("second-map")
        self.assert_consistent(successor, "second-map")
        self.assertEqual(self.stub.calls.count("StopMapping"), 1)
        self.assertEqual(self.stub.calls.count("ActivateMap"), 1)

    def test_durable_receipt_failure_retains_journal_fence_and_original_owner_for_restore(self):
        with patch.object(
            self.service._repository,
            "complete_assigned_map_activation_durable",
            side_effect=OSError("receipt fsync failed"),
        ), self.assertRaises(GatewayError):
            self.activate()
        pending = self.service._repository.get_map_activation()
        self.assertEqual(pending["phase"], "reconciling")
        self.assertTrue(self.runtime.is_map_asset_activation_recovery_pending())
        self.assertIsNone(self.localization.get_active_session_token()[0])
        self.stub.calls.clear()
        restored = self.service.restore_active_workspace()
        self.assert_consistent(restored)
        self.assertEqual(
            restored.active_localization_session.activation_receipt_v2.to_dict()["request"],
            pending["activation_request"],
        )
        self.assertTrue(all(call == "GetMappingState" for call in self.stub.calls))

    def test_journal_clear_failure_retains_fence_and_releases_session_publication_outside_guards(self):
        published = []
        original = self.store.write

        def observe(namespace, key, value, **kwargs):
            self.assert_unlocked("publication after durable failure")
            if namespace == StateNamespace.LOCALIZATION and key == "active_session":
                published.append(value)
            return original(namespace, key, value, **kwargs)

        with patch.object(self.store, "write", side_effect=observe), patch.object(
            self.service._repository, "clear_assigned_map_activation", side_effect=OSError("journal unlink failed")
        ), self.assertRaises(GatewayError):
            self.activate()
        self.assertEqual(len(published), 1)
        self.assertIs(published[0], self.localization.get_active_session_token()[0])
        self.assertIsNotNone(self.service._repository.get_map_activation())
        self.assertTrue(self.runtime.is_map_asset_activation_recovery_pending())
        self.assertIsNone(self.service.get_active_workspace())
        self.stub.calls.clear()
        self.assert_consistent(self.service.restore_active_workspace())
        self.assertTrue(all(call == "GetMappingState" for call in self.stub.calls))

    def test_unknown_predecessor_stop_never_activates_until_explicit_retry_of_exact_journal(self):
        self.stub.state = predecessor(active=True)
        original = self.stub.StopMapping

        def lost_stop(*args, **kwargs):
            original(*args, **kwargs)
            raise LostReply()

        with patch.object(self.stub, "StopMapping", side_effect=lost_stop), self.assertRaises(GatewayError):
            self.activate()
        pending = self.service._repository.get_map_activation()
        self.assertEqual(pending["phase"], "predecessor_stop_dispatching")
        self.assertEqual(pending["predecessor"]["session_id"], "predecessor")
        self.assertEqual(self.stub.calls.count("StopMapping"), 1)
        self.assertNotIn("ActivateMap", self.stub.calls)
        self.assertTrue(self.runtime.is_map_asset_activation_recovery_pending())
        self.stub.calls.clear()
        result = self.activate()
        self.assert_consistent(result)
        self.assertEqual(
            result.active_localization_session.activation_receipt_v2.to_dict()["request"], pending["activation_request"]
        )
        self.assertNotIn("StopMapping", self.stub.calls)
        self.assertEqual(self.stub.calls.count("ActivateMap"), 1)

    def test_pending_original_map_blocks_different_target_before_any_native_rpc(self):
        self.publish_map("second-map", AssetIdentityV2(asset_id="e" * 32))
        self.stub.error = LostReply()
        with self.assertRaises(GatewayError):
            self.activate()
        pending = self.service._repository.get_map_activation()
        self.stub.calls.clear()
        with self.assertRaises(GatewayError):
            self.activate("second-map")
        self.assertEqual(self.service._repository.get_map_activation(), pending)
        self.assertEqual(self.stub.calls, [])
        self.assertTrue(self.runtime.is_map_asset_activation_recovery_pending())

    def test_successor_before_underlying_workspace_write_cannot_leave_stale_cache_or_return(self):
        self.publish_map("second-map", AssetIdentityV2(asset_id="e" * 32))
        original = self.store.write
        successor = []

        def activate_before_write(namespace, key, value, **kwargs):
            if namespace == StateNamespace.MAP and key == "active_workspace" and value.active_map_name == MAP_NAME:
                self.assert_unlocked("before workspace write")
                successor.append(self.activate("second-map"))
            return original(namespace, key, value, **kwargs)

        with patch.object(self.store, "write", side_effect=activate_before_write), self.assertRaises(GatewayError):
            self.activate()
        self.assertEqual(len(successor), 1)
        self.assert_consistent(successor[0], "second-map")
        cached = self.store.read(StateNamespace.MAP, "active_workspace")
        self.assertTrue(cached is None or cached.value.active_map_name == "second-map")
        self.assertEqual(self.service.get_active_workspace().active_map_name, "second-map")

    def test_mapping_clear_during_summary_prevents_workspace_publication_and_stale_return(self):
        def clear_mapping():
            self.assert_unlocked("memory summary callback")
            self.mapping.clear_latest_snapshot(cleared_map_name=MAP_NAME, reason="superseded during summary")

        with patch.object(self.memory, "get_summary", side_effect=clear_mapping), self.assertRaises(GatewayError):
            self.activate()
        self.assertIsNone(self.mapping.get_latest_snapshot())
        self.assertIsNone(self.store.read(StateNamespace.MAP, "active_workspace"))
        self.assertIsNone(self.service.get_active_workspace())

    def test_mapping_clear_during_workspace_publication_invalidates_just_written_workspace(self):
        original = self.store.write

        def clear_mapping(namespace, key, value, **kwargs):
            result = original(namespace, key, value, **kwargs)
            if namespace == StateNamespace.MAP and key == "active_workspace":
                self.assert_unlocked("workspace publication mapping clear")
                self.mapping.clear_latest_snapshot(cleared_map_name=MAP_NAME, reason="superseded during publication")
            return result

        with patch.object(self.store, "write", side_effect=clear_mapping), self.assertRaises(GatewayError):
            self.activate()
        self.assertIsNone(self.mapping.get_latest_snapshot())
        self.assertIsNone(self.store.read(StateNamespace.MAP, "active_workspace"))
        self.assertIsNone(self.service.get_active_workspace())

    def test_constructor_preserves_assigned_relocalization_intent_without_legacy_abort_or_rpc(self):
        workspace = self.activate()
        owner = workspace.active_localization_session.activation_receipt_v2
        request = AssignedRelocalizationRequest.admit(
            {
                "schema_version": 2,
                "request_id": "relocalize:pending",
                "assigned_cloud_v2": {
                    "target": owner.to_dict()["target"],
                    "policy": current_assigned_relocalization_policy(),
                    "logical_timeout_ms": 0,
                },
            }
        )
        args = {"assigned_request": request, "activation_receipt": owner, "target_asset_identity": owner.asset_identity}
        self.service._repository.begin_assigned_relocalization_durable(**args)
        for phase in ("prepared", "backend_dispatching"):
            with self.subTest(phase=phase):
                if phase == "backend_dispatching":
                    self.service._repository.mark_assigned_relocalization_backend_dispatching(**args)
                before = self.service._repository._relocalization_intent_path.read_bytes()
                self.stub.calls.clear()
                with patch.object(
                    MapCatalogRepository,
                    "abort_relocalization_before_apply",
                    side_effect=AssertionError("legacy abort used for assigned intent"),
                ):
                    self.service = self.new_service()
                self.assertEqual(self.stub.calls, [])
                self.assertEqual(self.service._repository._relocalization_intent_path.read_bytes(), before)
                self.assertEqual(self.service._repository.get_relocalization_intent()["phase"], phase)
                with self.assertRaises(GatewayError):
                    self.activate()
                self.assertEqual(self.stub.calls, [])

    def test_dispatch_phase_rename_requires_successful_directory_fsync_on_every_explicit_retry(self):
        for phase in ("backend_dispatching", "predecessor_stop_dispatching"):
            with self.subTest(phase=phase):
                self.stub.state = predecessor(active=phase == "predecessor_stop_dispatching")
                self.stub.calls.clear()
                original_fsync = catalog_module._fsync_directory
                failed_phases = []

                def fail_dispatch_phase_fsync(
                    path, *, _phase=phase, _failed_phases=failed_phases, _original_fsync=original_fsync
                ):
                    journal_path = self.service._repository._activation_journal_path
                    if journal_path.exists():
                        visible = json.loads(journal_path.read_text(encoding="utf-8"))
                        if visible["phase"] == _phase:
                            # The real atomic writer already renamed its file; the
                            # caller still lacks the directory durability barrier.
                            _failed_phases.append(visible["phase"])
                            raise OSError("directory fsync failed after dispatch-phase rename")
                    return _original_fsync(path)

                retained_request = None
                with patch.object(catalog_module, "_fsync_directory", side_effect=fail_dispatch_phase_fsync):
                    for attempt in (1, 2):
                        with self.subTest(attempt=attempt), self.assertRaises(GatewayError):
                            self.activate()
                        pending = self.service._repository.get_map_activation()
                        self.assertEqual(pending["phase"], phase)
                        if retained_request is None:
                            retained_request = deepcopy(pending["activation_request"])
                        self.assertEqual(pending["activation_request"], retained_request)
                        self.assertEqual(len(failed_phases), attempt)
                        self.assertFalse(
                            any(
                                call == "StopMapping" or call == "ActivateMap" or call.startswith("Transfer:")
                                for call in self.stub.calls
                            )
                        )
                        self.assertTrue(self.runtime.is_map_asset_activation_recovery_pending())
                completed = self.activate()
                self.assert_consistent(completed)
                self.assertEqual(
                    completed.active_localization_session.activation_receipt_v2.to_dict()["request"], retained_request
                )
                self.assertEqual(self.stub.calls.count("ActivateMap"), 1)
                self.assertEqual(self.stub.calls.count("StopMapping"), int(phase == "predecessor_stop_dispatching"))

    def test_journal_clear_cancellation_rearms_retained_fence_outside_guards_and_propagates_original(self):
        cancellation = ActivationCancelled("cancelled while clearing durable journal")
        original_arm = self.runtime.arm_assigned_map_activation_fence
        arm_reasons = []

        def arm_outside_guards(**kwargs):
            self.assert_unlocked("cancellation fence restoration")
            arm_reasons.append(kwargs["reason"])
            return original_arm(**kwargs)

        with patch.object(
            self.runtime, "arm_assigned_map_activation_fence", side_effect=arm_outside_guards
        ), patch.object(
            self.service._repository, "clear_assigned_map_activation", side_effect=cancellation
        ), self.assertRaises(ActivationCancelled) as raised:
            self.activate()
        self.assertIs(raised.exception, cancellation)
        self.assertIn("assigned_reconciliation_incomplete", arm_reasons)
        self.assertIsNotNone(self.service._repository.get_map_activation())
        self.assertTrue(self.runtime.is_map_asset_activation_recovery_pending())
        self.assertIsNone(self.service.get_active_workspace())
        self.assertIs(
            self.store.read(StateNamespace.LOCALIZATION, "active_session").value,
            self.localization.get_active_session_token()[0],
        )
        self.stub.calls.clear()
        self.assert_consistent(self.service.restore_active_workspace())
        self.assertTrue(all(call == "GetMappingState" for call in self.stub.calls))

    def test_workspace_write_failure_or_cancellation_discards_stale_value_and_preserves_original_error(self):
        self.publish_map("second-map", AssetIdentityV2(asset_id="e" * 32))
        for error_type in (RuntimeError, ActivationCancelled):
            with self.subTest(error_type=error_type.__name__):
                failure = error_type("workspace hook failed after writing superseded value")
                original = self.store.write
                successor = []

                def switch_write_raise(
                    namespace, key, value, *, _failure=failure, _original=original, _successor=successor, **kwargs
                ):
                    if (
                        namespace == StateNamespace.MAP
                        and key == "active_workspace"
                        and value.active_map_name == MAP_NAME
                    ):
                        self.assert_unlocked("failing workspace hook")
                        _successor.append(self.activate("second-map"))
                        _original(namespace, key, value, **kwargs)
                        raise _failure
                    return _original(namespace, key, value, **kwargs)

                with patch.object(self.store, "write", side_effect=switch_write_raise), self.assertRaises(
                    error_type
                ) as raised:
                    self.activate()
                self.assertIs(raised.exception, failure)
                self.assertEqual(len(successor), 1)
                self.assert_consistent(successor[0], "second-map")
                cached = self.store.read(StateNamespace.MAP, "active_workspace")
                self.assertTrue(cached is None or cached.value.active_map_name == "second-map")
                self.assertEqual(self.service.get_active_workspace().active_map_name, "second-map")


if __name__ == "__main__":
    unittest.main()
