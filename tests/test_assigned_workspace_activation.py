"""Public schema-2 workspace integration with real stores, runtime and session CAS.

Only native protobuf/RPC, memory and navigation endpoints are simulated. No
native process, hardware, legacy hash authority or unrelated suite is needed.
"""

import hashlib
import tempfile
import threading
import unittest
from contextlib import ExitStack
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from tests.conftest import LoaderStub, ack, predecessor
from tests.test_assigned_asset_storage import (
    ASSET_ID,
    MAP_NAME,
    STAMP,
    bundle_fixture,
    checkpoint_fixture,
    write,
)
from tests.conftest import response_fixture

from contracts.geometry import Pose
from contracts.map_workspace import AssetIdentityV2, MapAsset, MapAssetBundle, MapAssetStatus
from contracts.runtime_views import LocalizationSession, LocalizationSessionStatus, LocalizationSnapshot
from core import StateNamespace, StateStore
from core.errors import GatewayError
from services.geometry import geometry_runtime_provider as provider_module
from services.geometry import runtime as runtime_module
from services.geometry.mapping_backend_checkpoint_store import MappingBackendCheckpointStore
from services.localization.service import LocalizationService
from services.mapping.asset_store import MapAssetBundleRepository
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
            clear_latest_exploration_context=lambda **kwargs: None,
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
        self.stub.during = self.assert_execution_locked

    def new_runtime(self):
        from contracts.geometry_runtime import PlatformGeometryConfig
        runtime = runtime_module.PlatformGeometryRuntime(config=PlatformGeometryConfig(),
            catalog_root=str(self.root), robot=SimpleNamespace(providers=SimpleNamespace()))
        runtime._provider_attach_epoch = 2
        runtime._remote_provider = self.provider
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

    def assert_execution_locked(self, operation):
        for name in ("_runtime_state_lock", "_mapping_lifecycle_lock", "_provider_attach_lock"):
            self.assertFalse(getattr(self.runtime, name)._is_owned(), (operation, name))
        self.assertFalse(self.service._repository._activation_guard_state.mutex._is_owned(), operation)
        self.assertTrue(self.service._repository._activation_executor_guard_state.mutex._is_owned(), operation)

    def no_legacy_or_hash(self):
        stack = ExitStack()
        for name in hashlib.algorithms_guaranteed | {"new", "file_digest"}:
            if hasattr(hashlib, name):
                stack.enter_context(
                    patch.object(hashlib, name, side_effect=AssertionError("application hash: " + name))
                )
        for name in ("_build_workspace",):
            stack.enter_context(
                patch.object(self.service, name, side_effect=AssertionError("legacy workspace: " + name))
            )
        for name in (
            "_materialize_checkpoint_pcd_asset",
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
        self.assertEqual(self.service._repository.get_active_map_name(), name)
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

    def test_map_asset_reads_and_checkpoint_preparation_hold_executor_without_catalog_or_runtime_locks(self):
        read_bundle = self.service._asset_repository.get_map_asset_bundle
        load_checkpoint = MappingBackendCheckpointStore.load

        def read(*args, **kwargs):
            self.assert_execution_locked("asset bundle read")
            return read_bundle(*args, **kwargs)

        def load(store, *args, **kwargs):
            self.assert_execution_locked("checkpoint read")
            return load_checkpoint(store, *args, **kwargs)

        with patch.object(self.service._asset_repository, "get_map_asset_bundle", side_effect=read), patch.object(
            MappingBackendCheckpointStore, "load", load
        ):
            self.assert_consistent(self.activate())

    def test_map_publication_holds_executor_and_localization_publication_releases_it(self):
        publications = []
        original = self.store.write
        def write(namespace, key, value, **kwargs):
            if namespace == StateNamespace.LOCALIZATION or key == "active_workspace":
                self.assert_unlocked((namespace, key))
            else:
                self.assert_execution_locked((namespace, key))
            publications.append((namespace, key))
            return original(namespace, key, value, **kwargs)
        with patch.object(self.store, "write", side_effect=write):
            self.assert_consistent(self.activate())
        self.assertIn((StateNamespace.LOCALIZATION, "active_session"), publications)
        self.assertIn((StateNamespace.MAP, "active_workspace"), publications)

    def test_mapping_callback_supersession_is_detected_and_cannot_publish_success(self):
        original = self.store.write

        def replace_mapping(namespace, key, value, **kwargs):
            result = original(namespace, key, value, **kwargs)
            if namespace == StateNamespace.MAP and kwargs.get("metadata", {}).get("kind") == "latest_map":
                self.assert_execution_locked("map callback")
                self.mapping._latest_snapshot = value.model_copy(update={"map_name": "successor"})
            return result

        with patch.object(self.store, "write", side_effect=replace_mapping), self.assertRaises(GatewayError):
            self.activate()
        self.assertEqual(self.mapping.get_latest_snapshot().map_name, "successor")
        self.assertIsNone(self.localization.get_active_session_token()[0])
        self.assertIsNone(self.store.read(StateNamespace.MAP, "active_workspace"))

    def test_localization_preparation_supersession_loses_cas(self):
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
        self.runtime.clear_active_map_asset(reason="new native installation")
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

    def test_failed_activation_allows_another_map_and_mapping_mutations(self):
        self.publish_map("second-map", AssetIdentityV2(asset_id="e" * 32))
        failure = RuntimeError("地图发布失败")
        with patch.object(self.mapping, "ingest_external_snapshot", side_effect=failure):
            with self.assertRaises(RuntimeError) as raised:
                self.activate()
        self.assertIs(raised.exception, failure)
        self.assert_consistent(self.activate("second-map"), "second-map")
        self.runtime._runtime_started = True
        def start(**kwargs):
            return {"phase": "ACTIVE", "active": True, "session_id": kwargs["session_id"],
                "map_name": kwargs["map_name"], "session_generation": 9}
        def stop(**kwargs):
            return {"phase": "STOPPED", "active": False, "session_id": kwargs["session_id"]}
        with patch.object(self.provider, "start_mapping", side_effect=start), \
                patch.object(self.provider, "stop_mapping", side_effect=stop), \
                patch.object(self.provider, "get_exploration_state", return_value={"status": "IDLE"}), \
                patch.object(self.provider, "get_mapping_diagnostics", return_value={}):
            self.assertTrue(self.runtime.stop_mapping()["stopped"])
            self.assertTrue(self.runtime.start_mapping(map_name="second-map")["mapping_active"])
        self.memory.has_memory_library = lambda name: False
        self.navigation.delete_forbidden_zones_for_map = lambda **kwargs: SimpleNamespace(maps={})
        self.assertTrue(self.service.delete_map(map_name=MAP_NAME)["deleted"])

    def test_new_service_restore_dispatches_and_returns_loaded_localizing_workspace(self):
        self.activate()
        original = self.stub.activation_requests[-1]
        self.runtime = self.new_runtime()
        self.service = self.new_service()
        self.stub.calls.clear()
        restored = self.service.activate_map(map_name=MAP_NAME)
        self.assert_consistent(restored)
        self.assertIn("ActivateMap", self.stub.calls)
        self.assertNotEqual(original.activation_request_id, self.stub.activation_requests[-1].activation_request_id)

    def test_start_mapping_waits_for_the_entire_activation(self):
        entered, release, attempted, finished = (threading.Event() for _ in range(4))
        errors = []
        original = self.stub.ActivateMap
        def blocked(request, timeout):
            entered.set()
            if not release.wait(3):
                raise TimeoutError("激活阻塞未释放")
            return original(request, timeout)
        def activate():
            try:
                self.activate()
            except BaseException as error:
                errors.append(error)
        def start():
            attempted.set()
            try:
                self.runtime.start_mapping(map_name=MAP_NAME)
            except BaseException as error:
                errors.append(error)
            finally:
                finished.set()
        self.runtime._runtime_started = True
        def native_start(**kwargs):
            return {"phase": "ACTIVE", "active": True, "session_id": kwargs["session_id"],
                    "map_name": kwargs["map_name"], "session_generation": 9}
        with patch.object(self.stub, "ActivateMap", side_effect=blocked), \
                patch.object(self.provider, "start_mapping", side_effect=native_start), \
                patch.object(self.provider, "get_mapping_diagnostics", return_value={}):
            first = threading.Thread(target=activate, daemon=True)
            second = threading.Thread(target=start, daemon=True)
            first.start()
            try:
                self.assertTrue(entered.wait(3))
                second.start()
                self.assertTrue(attempted.wait(3))
                self.assertFalse(finished.wait(0.1))
            finally:
                release.set()
            first.join(3)
            second.join(3)
        self.assertFalse(first.is_alive())
        self.assertFalse(second.is_alive())
        self.assertEqual(errors, [])
        self.assertTrue(finished.is_set())

    def test_floor_mapping_and_concurrent_activation_use_the_same_lock_order(self):
        from contextlib import contextmanager
        from contracts.building_navigation import FloorRecord, FloorPose
        from gateways.building_navigation import GatewayBuildingPorts
        repository = self.service._repository
        asset = repository.get_map_asset(MAP_NAME)
        repository.save_map_asset(asset.model_copy(update={"metadata": {"floor_creation_request_id": "floor-op"}}))
        context = SimpleNamespace(task_id="floor-task", ensure_active=lambda: None)
        runtime = SimpleNamespace(map_workspace_service=self.service,
            _get_building_catalog=lambda: SimpleNamespace(maps=repository))
        ports = GatewayBuildingPorts(runtime, context, None)
        floor = FloorRecord(floor_id="F1", display_name="一层", map_name=MAP_NAME, asset_identity=ASSET_ID)
        entered, release, waiting = (threading.Event() for _ in range(3))
        original_rpc = self.stub.ActivateMap
        original_guard = repository.map_activation_executor_guard
        errors, results = [], []
        def blocked(request, timeout):
            entered.set()
            if not release.wait(3):
                raise TimeoutError("激活阻塞未释放")
            return original_rpc(request, timeout)
        @contextmanager
        def guard():
            if threading.current_thread().name == "floor":
                waiting.set()
            with original_guard():
                yield
        def activate():
            try:
                self.activate()
            except BaseException as error:
                errors.append(error)
        def start_floor():
            try:
                results.append(ports.start_floor_mapping(floor, SimpleNamespace(arrival_pose=FloorPose(x=0,y=0)), "floor-op"))
            except BaseException as error:
                errors.append(error)
        def localize(target, seed):
            self.service.activate_map(map_name=target.map_name)
            return True
        with patch.object(self.stub, "ActivateMap", side_effect=blocked), \
                patch.object(repository, "map_activation_executor_guard", side_effect=guard), \
                patch.object(ports, "activate_and_localize", side_effect=localize):
            first = threading.Thread(target=activate, daemon=True)
            second = threading.Thread(target=start_floor, name="floor", daemon=True)
            first.start()
            try:
                self.assertTrue(entered.wait(3))
                second.start()
                self.assertTrue(waiting.wait(3))
            finally:
                release.set()
            first.join(3)
            second.join(3)
        self.assertFalse(first.is_alive(), "激活与楼层任务发生锁等待环")
        self.assertFalse(second.is_alive(), "楼层任务未完成")
        self.assertEqual(errors, [])
        self.assertEqual(results, [ASSET_ID])


if __name__ == "__main__":
    unittest.main()
