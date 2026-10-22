"""Ordinary cloud save is paired with public assigned activation.

Use real runtime preparation, protobuf admission, checkpoint/asset repositories,
catalog and localization services. Only native RPCs and peripheral endpoints are
simulated; this suite starts no native process or hardware.
"""

import hashlib
import json
import os
import struct
import tempfile
import unittest
from contextlib import ExitStack
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from tests.conftest import LostReply
from tests.test_assigned_asset_storage import LEGACY_BUNDLE, LEGACY_MANIFEST, LEGACY_SHA, MAP_NAME, bundle_fixture, write
from tests.test_assigned_workspace_activation import MemoryEndpoint, WorkspaceNativeStub
from tests.conftest import TransferStub, response_fixture

from contracts.geometry import Pose
from contracts.map_workspace import AssetIdentityV2, MapAssetBundle, MapAssetStatus
from contracts.runtime_views import LocalizationSession, LocalizationSessionStatus, LocalizationSnapshot
from core import StateNamespace, StateStore
from core.errors import GatewayError
from services.geometry import geometry_runtime_provider as provider_module
from services.geometry import mapping_backend_checkpoint_store as checkpoint_module
from services.geometry import runtime as runtime_module
from services.geometry.config import PlatformGeometryConfig
from services.localization.service import LocalizationService
from services.mapping import catalog as catalog_module
from services.mapping.asset_store import MapAssetBundleRepository, MapAssetBundleValidationError
from services.mapping.catalog import MapCatalogRepository
from services.mapping.service import MappingService
from skills.map_workspace import MapWorkspaceService

pb = provider_module.pb2


class SaveCancelled(BaseException):
    """Cancellation must retain its type while owned staging is cleaned up."""


def cloud_response(backend_kind="voxel"):
    """Large enough for an actual PCD, with one coherent native receipt owner."""
    response = response_fixture()
    response.backend_kind = backend_kind
    response.occupancy_receipt.backend_kind = backend_kind
    response.cloud_receipt.point_count = 128
    response.canonical_cloud_checkpoint.point_count = 128
    response.canonical_cloud_checkpoint.xyz_f32_le = struct.pack("<fff", 1, 2, 3) * 128
    for member in (response.backend_identity, response.occupancy_identity,
                   response.canonical_cloud_checkpoint.member_identity,
                   response.fine_map_artifact.member_identity,
                   response.dense_cloud_artifact.member_identity):
        member.occupancy_receipt.CopyFrom(response.occupancy_receipt)
        member.cloud_receipt.CopyFrom(response.cloud_receipt)
    if backend_kind == "raytracing":
        response.payload = (b"NWMBCKP\0" + struct.pack("<IBBH", 1, 1, 0, 0)
                            + b"\0" * 64 + struct.pack("<QQ", 1, 0) + b"x")
        response.payload_size = len(response.payload)
    return response


class SaveNativeStub(WorkspaceNativeStub):
    def __init__(self, backend_kind="voxel"):
        super().__init__()
        self.response = cloud_response(backend_kind)
        self.entries = TransferStub(self.response).entries
        self.state = pb.MappingSessionState(success=True, phase="ACTIVE", active=True,
            session_id="source", map_name=MAP_NAME, session_generation=3, revision=6,
            runtime_generation=7, mapping_backend_kind=backend_kind)

    def ExportMappingBackendCheckpoint(self, request, timeout):
        self.visit("ExportMappingBackendCheckpoint")
        return TransferStub.ExportMappingBackendCheckpoint(self, request, timeout)

    def GetMappingReadiness(self, request, timeout):
        self.visit("GetMappingReadiness")
        return pb.MappingReadiness(ready=True, message=json.dumps({
            "mapping_session_id": "source", "map_name": MAP_NAME,
            "session_generation": 3, "last_occupancy_revision": 6,
        }))

    def GetStatus(self, request, timeout):
        self.visit("GetStatus")
        return pb.RuntimeStatus(phase="READY")

    def GetFrameTree(self, request, timeout):
        self.visit("GetFrameTree")
        return pb.FrameTree()

    def ActivateMap(self, request, timeout):
        result = super().ActivateMap(request, timeout)
        kind = request.mapping_backend_checkpoint.backend_kind
        result.mapping_backend_kind = kind
        result.activation_receipt_v2.activation.backend_kind = kind
        result.activation_receipt_v2.target.activation.backend_kind = kind
        result.activation_receipt_v2.target.occupancy_receipt.backend_kind = kind
        self.state = deepcopy(result)
        return result


class AssignedWorkspaceSaveTests(unittest.TestCase):
    def setUp(self):
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.root = Path(self.stack.enter_context(tempfile.TemporaryDirectory()))
        self.stack.enter_context(patch.dict(os.environ, {"NUWAX_PCD_MIN_POINTS": "64"}))
        self.store = StateStore()
        self.memory = MemoryEndpoint()
        self.navigation = SimpleNamespace(
            list_forbidden_zones=lambda **kwargs: SimpleNamespace(maps={}),
            load_forbidden_zones_for_map=lambda **kwargs: None,
            get_latest_navigation_context=lambda: None,
            get_latest_exploration_context=lambda: None,
        )
        self.stub = SaveNativeStub()
        self.provider = provider_module.GeometryRuntimeProvider(None)
        self.provider._connected = True
        self.provider._stub = self.stub
        self.provider._mapping_identity = ("source", MAP_NAME, 3)
        self.stub.owner = self.provider
        self.runtime = self.new_runtime()
        self.mapping = MappingService(provider_owner=SimpleNamespace(), state_store=self.store)
        self.localization = LocalizationService(provider_owner=SimpleNamespace(), state_store=self.store)
        self.service = self.new_service()
        self.service.create_map(map_name=MAP_NAME)
        self.service._repository.set_active_map_name(MAP_NAME)
        seed = bundle_fixture().snapshot
        self.mapping.ingest_external_snapshot(map_name=MAP_NAME, source_name="live-fixture",
            source_epoch="source:3", occupancy_grid=seed.occupancy_grid,
            metadata={"mapping_session_id": "source", "session_generation": 3, "map_name": MAP_NAME})

    def new_runtime(self):
        config = PlatformGeometryConfig()
        config.mapping.database_root = str(self.root / "live-db")
        config.mapping.pcd_root = str(self.root / "live-pcd")
        runtime = runtime_module.PlatformGeometryRuntime(config=config, catalog_root=str(self.root),
            robot=SimpleNamespace(providers=SimpleNamespace()))
        runtime._remote_provider = self.provider
        runtime._provider_attach_epoch = 2
        runtime._mapping_session_context = {"active": True, "phase": "ACTIVE",
            "session_id": "source", "map_name": MAP_NAME, "session_generation": 3,
            "revision": 6, "mapping_backend_kind": self.stub.response.backend_kind,
            "runtime_generation": 7, "backend_generation": 2}
        return runtime

    def new_service(self):
        return MapWorkspaceService(catalog_root=str(self.root), geometry_runtime=self.runtime,
            mapping_service=self.mapping, localization_service=self.localization,
            memory_service=self.memory, navigation_service=self.navigation, state_store=self.store)

    def save(self):
        return self.service.finalize_active_map(map_name=MAP_NAME, reason="paired save acceptance")

    def assert_unlocked(self, operation):
        for name in ("_runtime_state_lock", "_mapping_lifecycle_lock", "_provider_attach_lock"):
            self.assertFalse(getattr(self.runtime, name)._is_owned(), (operation, name))
        for name in ("_activation_guard_state", "_activation_executor_guard_state"):
            state = getattr(self.service._repository, name, None)
            if state is not None:
                self.assertFalse(state.mutex._is_owned(), (operation, name))

    def assigned_only(self):
        guard = ExitStack()
        for name in hashlib.algorithms_guaranteed | {"new", "file_digest"}:
            if hasattr(hashlib, name):
                guard.enter_context(patch.object(hashlib, name,
                    side_effect=AssertionError("application hash: " + name)))
        for owner, names in (
            (self.service._asset_repository, ("finalize_map_asset_bundle",)),
            (self.service, ("_activate_legacy_map",)),
            (provider_module.MappingBackendCheckpointExportBundle,
                ("canonical_cloud_checkpoint",)),
            (provider_module, ("_canonical_cloud_from_stored",)),
        ):
            for name in names:
                guard.enter_context(patch.object(owner, name,
                    side_effect=AssertionError("legacy/copy path: " + name)))
        return guard

    def assert_saved(self, bundle):
        self.assertEqual(bundle.identity_schema_version, 2)
        self.assertIsInstance(bundle.asset_identity, AssetIdentityV2)
        self.assertEqual(bundle.asset_state, "finalized")
        self.assertIsNone(bundle.manifest_sha256)
        catalog = MapCatalogRepository(str(self.root)).get_map_asset(MAP_NAME)
        self.assertTrue(catalog.finalized)
        self.assertEqual(catalog.identity_schema_version, 2)
        self.assertEqual(catalog.asset_identity, bundle.asset_identity)
        self.assertIsNone(catalog.asset_manifest_sha256)
        repository = MapAssetBundleRepository(str(self.root))
        reopened = repository.get_map_asset_bundle(MAP_NAME, expected_asset_identity=bundle.asset_identity)
        self.assertEqual(reopened.asset_identity, bundle.asset_identity)
        manifest = json.loads((repository.current_directory(MAP_NAME) / "manifest.json").read_text())
        self.assertEqual(manifest["schema_version"], 2)
        self.assertEqual(manifest["checkpoint_source"]["manifest"]["checkpoint_id"], self.stub.response.checkpoint_id)
        self.assertNotIn("sha256", json.dumps(manifest))
        return reopened

    def test_ordinary_cloud_save_reopens_and_public_load_installs_same_assigned_owner(self):
        with self.assigned_only():
            with patch.object(provider_module, "_protobuf_payload_bytes",
                    side_effect=AssertionError("save rebuilt a whole protobuf payload")):
                saved = self.save()
            reopened = self.assert_saved(saved)
            self.localization._active_session = LocalizationSession(session_id="old", map_name="old",
                status=LocalizationSessionStatus.READY, pose_available=True)
            self.localization._latest_snapshot = LocalizationSnapshot(source_name="old", current_pose=Pose(frame_id="map"))
            workspace = self.service.activate_map(map_name=MAP_NAME, load_memory_history=False)
        self.assertEqual(self.stub.exports, [2])
        self.assertTrue(workspace.map_loaded)
        self.assertFalse(workspace.localization_ready)
        self.assertIsNone(workspace.latest_localization_snapshot)
        session = workspace.active_localization_session
        self.assertEqual(session.asset_identity, reopened.asset_identity)
        self.assertEqual(session.status, LocalizationSessionStatus.LOCALIZING)
        self.assertFalse(session.pose_available)
        self.assertIsNone(session.relocalization_terminal_v2)
        self.assertIsNone(self.localization.get_latest_snapshot())
        self.assertEqual(self.stub.last_request.asset_identity.asset_id, saved.asset_identity.asset_id)
        self.assertEqual(self.stub.last_request.mapping_backend_checkpoint, self.stub.response)
        self.assertEqual(self.stub.calls.count("ActivateMap"), 1)
        self.assertEqual(self.stub.calls.count("StopMapping"), 1)
        self.assertEqual(self.mapping.get_latest_snapshot().metadata["asset_identity"], saved.asset_identity.model_dump())
        catalog = self.service._repository.get_map_asset(MAP_NAME)
        self.assertEqual(catalog.asset_identity, session.asset_identity)
        self.assertTrue(catalog.active)
        self.assertEqual(catalog.status.value, "localizing")
        target = self.runtime.capture_assigned_map_activation_target(
            map_name=MAP_NAME, asset_identity=saved.asset_identity)
        self.assertEqual(session.activation_receipt_v2, target.activation_receipt)
        self.assertEqual(self.mapping.get_latest_snapshot().metadata["activation_receipt_v2"],
            target.activation_receipt.to_dict())

    def test_raytracing_ordinary_save_selects_checkpoint_two_and_assigned_asset(self):
        self.stub = SaveNativeStub("raytracing")
        self.stub.owner = self.provider
        self.provider._stub = self.stub
        self.runtime._mapping_session_context["mapping_backend_kind"] = "raytracing"
        with self.assigned_only():
            saved = self.save()
            self.assert_saved(saved)
            workspace = self.service.activate_map(map_name=MAP_NAME)
        self.assertEqual(self.stub.exports, [2])
        self.assertEqual(saved.geometry_asset.metadata["mapping_backend_checkpoint"]["backend_kind"], "raytracing")
        self.assertEqual(workspace.active_localization_session.asset_identity, saved.asset_identity)

    def test_restart_recovers_durable_current_after_rename_without_reexport_or_new_identity(self):
        def fail(phase):
            if phase == "after_rename":
                raise OSError("injected post-rename failure")
        self.service._asset_repository = MapAssetBundleRepository(str(self.root), fault_injector=fail)
        with self.assigned_only(), self.assertRaises(GatewayError):
            self.save()
        durable = MapAssetBundleRepository(str(self.root)).get_map_asset_bundle(MAP_NAME)
        self.assertIsNotNone(durable)
        self.assertFalse(MapCatalogRepository(str(self.root)).get_map_asset(MAP_NAME).finalized)
        self.runtime = self.new_runtime()
        self.service = self.new_service()
        with self.assigned_only(), patch.object(self.runtime, "prepare_map_asset_bundle",
                side_effect=AssertionError("durable save was prepared again")):
            recovered = self.save()
            self.assert_saved(recovered)
        self.assertEqual(recovered.asset_identity, durable.asset_identity)
        self.assertEqual(self.stub.exports, [2])

    def test_restart_recovers_catalog_failure_from_same_durable_current(self):
        original = self.service._repository.save_map_asset
        def fail_finalized(asset):
            if asset.finalized:
                raise OSError("injected catalog write failure")
            return original(asset)
        with self.assigned_only(), patch.object(self.service._repository, "save_map_asset", fail_finalized), \
                self.assertRaises((GatewayError, OSError)):
            self.save()
        durable = MapAssetBundleRepository(str(self.root)).get_map_asset_bundle(MAP_NAME)
        self.runtime = self.new_runtime()
        self.service = self.new_service()
        with self.assigned_only(), patch.object(self.runtime, "prepare_map_asset_bundle",
                side_effect=AssertionError("catalog recovery prepared a new asset")):
            recovered = self.save()
            self.assert_saved(recovered)
            workspace = self.service.activate_map(map_name=MAP_NAME)
        self.assertEqual(recovered.asset_identity, durable.asset_identity)
        self.assertEqual(workspace.active_localization_session.asset_identity, durable.asset_identity)
        self.assertEqual(self.stub.exports, [2])

    def test_finalized_retry_reuses_exact_owner_without_export(self):
        with self.assigned_only():
            first = self.save()
            current = self.service._asset_repository.current_directory(MAP_NAME)
            inode = current.stat().st_ino
            with patch.object(self.runtime, "prepare_map_asset_bundle",
                    side_effect=AssertionError("finalized retry prepared again")):
                retry = self.save()
        self.assertEqual(retry.asset_identity, first.asset_identity)
        self.assertEqual(current.stat().st_ino, inode)
        self.assertEqual(self.stub.exports, [2])

    def test_rpc_asset_io_and_publication_callbacks_run_outside_catalog_runtime_locks(self):
        self.stub.during = self.assert_unlocked
        original_event = self.service._publish_event
        original_write = self.store.write
        def event(*args, **kwargs):
            self.assert_unlocked("workspace event callback")
            return original_event(*args, **kwargs)
        def write(*args, **kwargs):
            self.assert_unlocked("state publication callback")
            return original_write(*args, **kwargs)
        with self.assigned_only(), ExitStack() as probes:
            for cls, names in (
                (MapAssetBundleRepository, ("get_map_asset_bundle", "create_staging_directory",
                    "finalize_assigned_map_asset_bundle", "discard_staging_directory")),
                (checkpoint_module.MappingBackendCheckpointStore, ("publish", "load")),
            ):
                for name in names:
                    original = getattr(cls, name)
                    def checked(owner, *args, _original=original, _name=name, **kwargs):
                        self.assert_unlocked(_name)
                        return _original(owner, *args, **kwargs)
                    probes.enter_context(patch.object(cls, name, checked))
            probes.enter_context(patch.object(self.service, "_publish_event", event))
            probes.enter_context(patch.object(self.store, "write", write))
            saved = self.save()
            self.assert_saved(saved)
            self.service.activate_map(map_name=MAP_NAME)

    def test_catalog_identity_conflict_fails_before_reexport_or_activation(self):
        with self.assigned_only():
            saved = self.save()
        before = (self.service._asset_repository.current_directory(MAP_NAME) / "manifest.json").read_bytes()
        asset = self.service._repository.get_map_asset(MAP_NAME)
        self.service._repository.save_map_asset(asset.model_copy(update={
            "asset_identity": AssetIdentityV2(asset_id="e" * 32)}))
        with self.assigned_only(), patch.object(self.runtime, "prepare_map_asset_bundle",
                side_effect=AssertionError("conflicting identity reexported")):
            for action in (self.save, lambda: self.service.activate_map(map_name=MAP_NAME)):
                with self.subTest(action=action), self.assertRaises((GatewayError, MapAssetBundleValidationError)):
                    action()
        self.assertEqual(self.stub.exports, [2])
        self.assertNotIn("ActivateMap", self.stub.calls)
        self.assertEqual((self.service._asset_repository.current_directory(MAP_NAME) / "manifest.json").read_bytes(), before)
        self.assertNotEqual(self.service._repository.get_map_asset(MAP_NAME).asset_identity, saved.asset_identity)

    def test_changed_checkpoint_body_is_not_repaired_by_an_ordinary_save_retry(self):
        with self.assigned_only():
            self.save()
        current = self.service._asset_repository.current_directory(MAP_NAME)
        manifest = json.loads((current / "manifest.json").read_text())
        checkpoint = manifest["checkpoint_source"]["manifest"]
        backend_path = checkpoint["members"]["backend_state"]["path"]
        # Break the native magic without changing member length. Admission must
        # fail rather than hash the file, mint another ID, or overwrite current.
        path = current / "mapping_backend" / checkpoint["backend_kind"] / backend_path
        original = path.read_bytes()
        path.write_bytes(b"BROKEN!!" + original[8:])
        with self.assigned_only(), patch.object(self.runtime, "prepare_map_asset_bundle",
                side_effect=AssertionError("corrupt current silently reexported")):
            for action in (self.save, lambda: self.service.activate_map(map_name=MAP_NAME)):
                with self.subTest(action=action), self.assertRaises((GatewayError, MapAssetBundleValidationError)):
                    action()
        self.assertEqual(path.read_bytes(), b"BROKEN!!" + original[8:])
        self.assertEqual(self.stub.exports, [2])
        self.assertNotIn("ActivateMap", self.stub.calls)

    def test_low_level_prepare_default_keeps_explicit_legacy_export_contract(self):
        seed = self.mapping.get_latest_snapshot()
        draft = MapAssetBundle(map_name=MAP_NAME, source_name=seed.source_name,
            frame_id="map", snapshot=seed, metadata={"mapping_session_id": "source", "session_generation": 3})
        requested = []
        def stop(request, timeout):
            requested.append(request.checkpoint_schema_version)
            raise RuntimeError("stop after observing requested schema")
        stage = self.service._asset_repository.create_staging_directory(MAP_NAME)
        with patch.object(self.stub, "ExportMappingBackendCheckpoint", stop), \
                self.assertRaisesRegex(RuntimeError, "observing requested schema"):
            self.runtime.prepare_map_asset_bundle(draft, bundle_root=stage)
        self.assertEqual(requested, [1])

    def test_planar_public_save_preserves_legacy_export_selection(self):
        self.runtime.config.pipeline = "slam_2d"
        self.runtime.config.mapping_backend = "slam_toolbox"
        self.runtime._mapping_session_context.update(mapping_backend_kind="slam_toolbox", planar_mode="mapping")
        self.stub.state.mapping_backend_kind = "slam_toolbox"
        requested = []
        def stop(request, timeout):
            requested.append(request.checkpoint_schema_version)
            raise RuntimeError("planar fixture export boundary")
        with patch.object(self.stub, "ExportMappingBackendCheckpoint", stop), \
                patch.object(self.service._asset_repository, "finalize_assigned_map_asset_bundle",
                    side_effect=AssertionError("planar entered assigned cloud finalizer")), \
                self.assertRaises(GatewayError):
            self.save()
        self.assertEqual(requested, [1])
        self.assertIsNone(self.service._repository.get_map_asset(MAP_NAME).asset_identity)

    def test_unknown_live_backend_never_falls_back_to_legacy_save(self):
        self.runtime._mapping_session_context["mapping_backend_kind"] = "unknown"
        self.stub.state.mapping_backend_kind = "unknown"
        with self.assigned_only(), patch.object(self.runtime, "prepare_map_asset_bundle",
                side_effect=AssertionError("unknown backend entered preparation")), self.assertRaises(GatewayError):
            self.save()
        self.assertEqual(self.stub.exports, [])
        self.assertFalse(self.service._asset_repository.current_directory(MAP_NAME).exists())

    def test_session_change_during_export_cannot_publish_an_assigned_current(self):
        original = self.stub.ExportMappingBackendCheckpoint
        def changed(request, timeout):
            response = original(request, timeout)
            self.runtime._mapping_session_context["session_generation"] = 4
            return response
        with self.assigned_only(), patch.object(self.stub, "ExportMappingBackendCheckpoint", changed), \
                self.assertRaises(GatewayError):
            self.save()
        self.assertEqual(self.stub.exports, [2])
        self.assertFalse(self.service._asset_repository.current_directory(MAP_NAME).exists())
        asset = self.service._repository.get_map_asset(MAP_NAME)
        self.assertFalse(asset.finalized)
        self.assertIsNone(asset.asset_identity)

    def test_cancellation_after_preparation_cleans_only_its_unpublished_staging(self):
        original = self.runtime.prepare_map_asset_bundle
        stages = []
        cancellation = SaveCancelled("cancel prepared save")
        def cancel(bundle, **kwargs):
            stages.append(kwargs["bundle_root"])
            original(bundle, **kwargs)
            raise cancellation
        with self.assigned_only(), patch.object(self.runtime, "prepare_map_asset_bundle", cancel), \
                self.assertRaises(SaveCancelled) as raised:
            self.save()
        self.assertIs(raised.exception, cancellation)
        self.assertEqual(len(stages), 1)
        self.assertFalse(stages[0].exists())
        self.assertFalse(self.service._asset_repository.current_directory(MAP_NAME).exists())
        self.assertFalse(self.service._repository.get_map_asset(MAP_NAME).finalized)
        self.assertEqual(self.stub.exports, [2])

    def test_cancellation_after_rename_preserves_durable_owner_for_restart(self):
        cancellation = SaveCancelled("cancel after durable rename")
        def cancel(phase):
            if phase == "after_rename":
                raise cancellation
        self.service._asset_repository = MapAssetBundleRepository(str(self.root), fault_injector=cancel)
        with self.assigned_only(), self.assertRaises(SaveCancelled) as raised:
            self.save()
        self.assertIs(raised.exception, cancellation)
        repository = MapAssetBundleRepository(str(self.root))
        durable = repository.get_map_asset_bundle(MAP_NAME)
        self.assertIsNotNone(durable)
        self.assertFalse(list(repository.current_directory(MAP_NAME).parent.glob(".current.tmp-*")))
        self.runtime = self.new_runtime()
        self.service = self.new_service()
        with self.assigned_only(), patch.object(self.runtime, "prepare_map_asset_bundle",
                side_effect=AssertionError("cancelled committed save reexported")):
            recovered = self.save()
            self.assert_saved(recovered)
        self.assertEqual(recovered.asset_identity, durable.asset_identity)
        self.assertEqual(self.stub.exports, [2])

    def test_delete_recreate_same_name_during_preparation_rejects_old_save_owner(self):
        original = self.runtime.prepare_map_asset_bundle
        replacements = []
        def replace_owner(bundle, **kwargs):
            prepared = original(bundle, **kwargs)
            self.service._repository.delete_map_asset(MAP_NAME)
            replacement = self.service.create_map(map_name=MAP_NAME, metadata={"replacement": True})
            self.service._repository.set_active_map_name(MAP_NAME)
            replacements.append(replacement)
            return prepared
        with self.assigned_only(), patch.object(self.runtime, "prepare_map_asset_bundle", replace_owner), \
                self.assertRaises(GatewayError) as raised:
            self.save()
        self.assertEqual(raised.exception.error_code, "map_asset_owner_changed")
        self.assertEqual(len(replacements), 1)
        self.assertEqual(self.service._repository.get_map_asset(MAP_NAME), replacements[0])
        self.assertFalse(self.service._asset_repository.current_directory(MAP_NAME).exists())
        self.assertEqual(self.stub.exports, [2])

    def test_reentrant_publication_cannot_replace_a_new_same_name_catalog_owner(self):
        original_write = self.store.write
        replacements = []
        events = []
        def replace_after_write(namespace, key, value, **kwargs):
            result = original_write(namespace, key, value, **kwargs)
            if namespace == StateNamespace.MAP and key == "asset:" + MAP_NAME and not replacements:
                replacements.append(None)
                self.service._repository.delete_map_asset(MAP_NAME)
                replacements[0] = self.service.create_map(map_name=MAP_NAME, metadata={"reentrant": True})
                self.service._repository.set_active_map_name(MAP_NAME)
            return result
        with self.assigned_only(), patch.object(self.store, "write", replace_after_write), \
                patch.object(self.service, "_publish_event", side_effect=lambda kind, *args, **kwargs: events.append(kind)), \
                self.assertRaises(GatewayError) as raised:
            self.save()
        self.assertEqual(raised.exception.error_code, "map_asset_owner_changed")
        self.assertEqual(self.service._repository.get_map_asset(MAP_NAME), replacements[0])
        cached = self.store.read(StateNamespace.MAP, "asset:" + MAP_NAME)
        self.assertIsNotNone(cached)
        self.assertEqual(cached.value, replacements[0])
        self.assertNotIn("map.asset_finalized", events)
        self.assertFalse(replacements[0].finalized)

    def test_saved_but_unloaded_assigned_asset_does_not_install_legacy_localization(self):
        with self.assigned_only():
            saved = self.save()
            with patch.object(self.localization, "ensure_active_session",
                    side_effect=AssertionError("saved assigned owner used legacy localization")):
                self.assertIsNone(self.service.get_active_workspace())
                self.assertIsNone(self.service.sync_active_localization_session(refresh=True))
            self.assertEqual(self.service.get_map_asset_bundle(map_name=MAP_NAME).asset_identity, saved.asset_identity)
        self.assertNotIn("ActivateMap", self.stub.calls)

    def test_metadata_rename_visible_fsync_failure_requires_durable_exact_retry(self):
        directory = self.service._asset_repository.current_directory(MAP_NAME).parent
        original_fsync = catalog_module._fsync_directory
        failures = []
        def fail_metadata_sync(path):
            if path == directory:
                failures.append(path)
                raise OSError("metadata rename visible but directory fsync failed")
            return original_fsync(path)
        with self.assigned_only(), patch.object(catalog_module, "_fsync_directory", fail_metadata_sync):
            with self.assertRaises(GatewayError):
                self.save()
            durable = MapAssetBundleRepository(str(self.root)).get_map_asset_bundle(MAP_NAME)
            visible = MapCatalogRepository(str(self.root)).get_map_asset(MAP_NAME)
            self.assertTrue(visible.finalized)
            self.assertEqual(visible.asset_identity, durable.asset_identity)
            self.runtime = self.new_runtime()
            self.service = self.new_service()
            with patch.object(self.runtime, "prepare_map_asset_bundle",
                    side_effect=AssertionError("visible metadata reexported")), \
                    self.assertRaises((GatewayError, OSError)):
                self.save()
            self.assertEqual(len(failures), 2)
        with self.assigned_only(), patch.object(self.runtime, "prepare_map_asset_bundle",
                side_effect=AssertionError("durability repair reexported")):
            recovered = self.save()
            self.assert_saved(recovered)
        self.assertEqual(recovered.asset_identity, durable.asset_identity)
        self.assertEqual(self.stub.exports, [2])

    def test_finalized_event_replacement_preserves_the_new_catalog_and_cache(self):
        original = self.service._publish_event
        replacements = []
        def replace_on_event(kind, *args, **kwargs):
            if kind == "map.asset_finalized":
                self.assert_unlocked("finalized event")
                self.service._repository.delete_map_asset(MAP_NAME)
                replacements.append(self.service.create_map(map_name=MAP_NAME, metadata={"event_replacement": True}))
                self.service._repository.set_active_map_name(MAP_NAME)
            return original(kind, *args, **kwargs)
        with self.assigned_only(), patch.object(self.service, "_publish_event", replace_on_event), \
                self.assertRaises(GatewayError) as raised:
            self.save()
        self.assertEqual(raised.exception.error_code, "map_asset_owner_changed")
        self.assertEqual(self.service._repository.get_map_asset(MAP_NAME), replacements[0])
        self.assertEqual(self.store.read(StateNamespace.MAP, "asset:" + MAP_NAME).value, replacements[0])
        self.assertFalse(replacements[0].finalized)

    def test_finalized_event_cancellation_clears_only_own_cache_and_keeps_durable_owner(self):
        cancellation = SaveCancelled("cancel finalized event")
        def cancel(kind, *args, **kwargs):
            self.assert_unlocked("cancel finalized event")
            if kind == "map.asset_finalized":
                raise cancellation
        with self.assigned_only(), patch.object(self.service, "_publish_event", cancel), \
                self.assertRaises(SaveCancelled) as raised:
            self.save()
        self.assertIs(raised.exception, cancellation)
        self.assertIsNone(self.store.read(StateNamespace.MAP, "asset:" + MAP_NAME))
        durable = MapAssetBundleRepository(str(self.root)).get_map_asset_bundle(MAP_NAME)
        self.assertEqual(self.service._repository.get_map_asset(MAP_NAME).asset_identity, durable.asset_identity)
        self.runtime = self.new_runtime()
        self.service = self.new_service()
        with self.assigned_only(), patch.object(self.runtime, "prepare_map_asset_bundle",
                side_effect=AssertionError("event-cancelled save reexported")):
            self.assertEqual(self.save().asset_identity, durable.asset_identity)
        self.assertEqual(self.stub.exports, [2])

    def test_active_saved_asset_uses_actual_active_name_and_does_not_claim_localization_ready(self):
        asset = self.service._repository.get_map_asset(MAP_NAME)
        self.assertFalse(asset.active)
        self.service._repository.save_map_asset(asset.model_copy(update={
            "status": MapAssetStatus.READY, "localization_ready": True}))
        with self.assigned_only():
            saved = self.save()
        catalog = self.service._repository.get_map_asset(MAP_NAME)
        self.assertEqual(catalog.asset_identity, saved.asset_identity)
        self.assertTrue(catalog.active)
        self.assertEqual(catalog.status, MapAssetStatus.MAPPING)
        self.assertFalse(catalog.localization_ready)
        self.assertNotIn("ActivateMap", self.stub.calls)

    def test_planar_selection_cannot_publish_a_cloud_preparation_through_legacy_finalizer(self):
        self.runtime.config.pipeline = "slam_2d"
        self.runtime.config.mapping_backend = "slam_toolbox"
        self.runtime._mapping_session_context.update(mapping_backend_kind="slam_toolbox", planar_mode="mapping")
        stages = []
        def changed_backend(bundle, *, bundle_root, **kwargs):
            stages.append(bundle_root)
            # Simulate the boundary result after a producer switch. This case
            # tests public dispatch, not the independently tested producer.
            geometry = bundle_fixture().geometry_asset.model_copy(update={"metadata": {
                "target_kind": "cloud_map", "mapping_backend_checkpoint": {
                    "present": True, "schema_version": 1, "backend_kind": "voxel"}}})
            return bundle.model_copy(update={"geometry_asset": geometry})
        with patch.object(self.runtime, "prepare_map_asset_bundle", changed_backend), \
                patch.object(self.service._asset_repository, "finalize_map_asset_bundle",
                    side_effect=AssertionError("actual cloud fell back to legacy publication")), \
                patch.object(self.service._asset_repository, "finalize_assigned_map_asset_bundle",
                    side_effect=AssertionError("mismatched planar selection entered assigned publication")), \
                self.assertRaises(GatewayError):
            self.save()
        self.assertEqual(len(stages), 1)
        self.assertFalse(stages[0].exists())
        self.assertFalse(self.service._asset_repository.current_directory(MAP_NAME).exists())
        self.assertFalse(self.service._repository.get_map_asset(MAP_NAME).finalized)

    def test_newly_saved_owner_lost_activation_reply_restores_exact_request_without_redispatch(self):
        with self.assigned_only():
            saved = self.save()
            self.stub.error = LostReply()
            with self.assertRaises(GatewayError):
                self.service.activate_map(map_name=MAP_NAME)
            pending = self.service._repository.get_map_activation()
            request = deepcopy(pending["activation_request"])
            self.assertEqual(set(request), {"activation_request_id", "session_id", "map_name",
                "asset_identity", "checkpoint_id", "expected_runtime_generation"})
            self.assertEqual(request["asset_identity"], saved.asset_identity.model_dump())
            self.assertEqual(request["checkpoint_id"], self.stub.response.checkpoint_id)
            self.assertEqual(self.stub.installs, 1)
            self.assertEqual(self.stub.calls.count("ActivateMap"), 1)
            self.stub.calls.clear()
            self.runtime = self.new_runtime()
            self.service = self.new_service()
            self.assertEqual(self.stub.calls, [])
            with patch.object(self.runtime, "prepare_map_asset_bundle",
                    side_effect=AssertionError("activation recovery reexported saved map")):
                restored = self.service.restore_active_workspace()
        self.assertEqual(self.stub.calls, ["GetMappingState"])
        self.assertEqual(self.stub.installs, 1)
        self.assertEqual(self.stub.exports, [2])
        session = restored.active_localization_session
        self.assertEqual(session.activation_receipt_v2.to_dict()["request"], request)
        self.assertEqual(session.asset_identity, saved.asset_identity)
        self.assertEqual(session.status, LocalizationSessionStatus.LOCALIZING)
        self.assertFalse(session.pose_available)
        self.assertFalse(restored.localization_ready)
        self.assertIsNone(restored.latest_localization_snapshot)
        self.assertIsNone(self.service._repository.get_map_activation())

    def test_existing_save_cannot_adopt_old_bundle_after_getter_replaces_same_name_owner(self):
        with self.assigned_only():
            saved = self.save()
        original = self.service.get_map_asset_bundle
        replacements = []
        def replace_after_read(**kwargs):
            bundle = original(**kwargs)
            self.service._repository.delete_map_asset(MAP_NAME)
            replacements.append(self.service.create_map(map_name=MAP_NAME, metadata={"after_read": True}))
            self.service._repository.set_active_map_name(MAP_NAME)
            return bundle
        with self.assigned_only(), patch.object(self.service, "get_map_asset_bundle", replace_after_read), \
                patch.object(self.runtime, "prepare_map_asset_bundle",
                    side_effect=AssertionError("stale existing-bundle retry exported")), \
                self.assertRaises(GatewayError) as raised:
            self.save()
        self.assertEqual(raised.exception.error_code, "map_asset_owner_changed")
        replacement = self.service._repository.get_map_asset(MAP_NAME)
        self.assertEqual(replacement, replacements[0])
        self.assertNotEqual(replacement.asset_identity, saved.asset_identity)
        self.assertFalse(replacement.finalized)
        self.assertFalse(self.service._asset_repository.current_directory(MAP_NAME).exists())
        self.assertEqual(self.stub.exports, [2])

    def test_literal_schema_one_current_public_get_and_save_preserve_original_bytes_and_digest(self):
        current = self.service._asset_repository.current_directory(MAP_NAME)
        geometry = json.loads(LEGACY_BUNDLE)["geometry_asset"]
        files = {
            "bundle.json": LEGACY_BUNDLE,
            "manifest.json": LEGACY_MANIFEST,
            "geometry/test/geometry_asset.json":
                json.dumps(geometry, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode() + b"\n",
            "geometry/test/state.bin": b"state",
            "geometry/test/map.yaml": b"image: map.pgm\nresolution: 0.125\n",
            "geometry/test/map.pgm": b"P5\n2 1\n255\n\x00\x64",
        }
        for relative, payload in files.items():
            write(current / relative, payload)
        inode = current.stat().st_ino
        self.assertEqual(hashlib.sha256(LEGACY_MANIFEST).hexdigest(), LEGACY_SHA)
        with patch.object(self.runtime, "prepare_map_asset_bundle",
                side_effect=AssertionError("existing schema1 map was exported again")), \
                patch.object(self.service._asset_repository, "finalize_map_asset_bundle",
                    side_effect=AssertionError("existing schema1 map was republished")), \
                patch.object(self.service._asset_repository, "finalize_assigned_map_asset_bundle",
                    side_effect=AssertionError("existing schema1 map was upgraded")):
            reopened = self.service.get_map_asset_bundle(map_name=MAP_NAME)
            saved = self.save()
        for bundle in (reopened, saved):
            self.assertEqual(bundle.identity_schema_version, 1)
            self.assertIsNone(bundle.asset_identity)
            self.assertEqual(bundle.manifest_sha256, LEGACY_SHA)
        catalog = MapCatalogRepository(str(self.root)).get_map_asset(MAP_NAME)
        self.assertTrue(catalog.finalized)
        self.assertEqual(catalog.identity_schema_version, 1)
        self.assertIsNone(catalog.asset_identity)
        self.assertEqual(catalog.asset_manifest_sha256, LEGACY_SHA)
        self.assertEqual(current.stat().st_ino, inode)
        self.assertEqual({relative: (current / relative).read_bytes() for relative in files}, files)
        self.assertEqual(self.stub.exports, [])
        self.assertEqual(self.stub.calls, [])


if __name__ == "__main__":
    unittest.main()
