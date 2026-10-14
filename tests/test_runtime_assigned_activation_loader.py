"""Explicit backend-only assigned loader, with real schema-2 storage admission.

No robot, native process, RPC, default workspace switch, or pose is installed.
"""
import importlib.util
import json
import tempfile
import threading
import unittest
from contextlib import ExitStack
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from services.geometry import geometry_runtime_provider as provider_module
from services.geometry import mapping_backend_checkpoint_store as store_module
from services.geometry import runtime as module

_fixture_path = Path(__file__).with_name("test_checkpoint_identity_v2.py")
_fixture_spec = importlib.util.spec_from_file_location("_assigned_loader_fixtures", _fixture_path)
_fixture_module = importlib.util.module_from_spec(_fixture_spec)
_fixture_spec.loader.exec_module(_fixture_module)


def activation_request():
    return {
        "activation_request_id": "mapload:retained-request",
        "session_id": "retained-session",
        "map_name": "assigned room",
        "asset_identity": {"asset_id": "d" * 32},
        "checkpoint_id": "a" * 32,
        "expected_runtime_generation": 37,
    }


class FakeProvider:
    is_connected = True

    def __init__(self, owner):
        self.owner = owner
        self.calls = []
        self.result = {"backend_switch_applied": True, "backend_activation_disposition": "applied"}
        self.error = None

    def activate_assigned_platform_map(self, *, activation_request, stored_checkpoint):
        if self.owner._runtime_state_lock._is_owned():
            raise AssertionError("activation RPC holds the runtime lock")
        self.calls.append((activation_request, stored_checkpoint))
        if self.error is not None:
            raise self.error
        return self.result

    def activate_platform_map(self, **kwargs):
        raise AssertionError("legacy activation fallback")

    def get_mapping_state(self):
        raise AssertionError("caller request runtime generation must not be refreshed")


class AssignedBackendLoaderTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.owner = module.PlatformGeometryRuntime.__new__(module.PlatformGeometryRuntime)
        self.owner._runtime_state_lock = threading.RLock()
        self.provider = FakeProvider(self.owner)
        self.owner._remote_provider = self.provider
        self.owner._map_activation_snapshot = module._MapActivationSnapshot(
            bundle=None, bundle_root=None, activation_result_json="{}", generation=91,
        )
        self.owner._mapping_session_context = {"session_id": "unchanged-local-session"}
        self.request = activation_request()

    def publish(self):
        bundle = provider_module._checkpoint_export_bundle_from_proto(
            _fixture_module.response_fixture(),
            fine_payload=_fixture_module.fine_bytes(),
            dense_payload=_fixture_module.dense_bytes(),
        )
        return bundle.publish_to(self.root)

    def activate(self, **overrides):
        arguments = {
            "activation_request": self.request,
            "bundle_root": self.root,
            "backend_kind": "voxel",
        }
        arguments.update(overrides)
        return self.owner.activate_assigned_backend_checkpoint(**arguments)

    def test_loads_exact_schema2_checkpoint_and_forwards_retained_request(self):
        published = self.publish()
        expected = deepcopy(self.request)
        loads = []
        original_load = store_module.MappingBackendCheckpointStore.load

        def load(store, backend_kind, **kwargs):
            self.assertFalse(self.owner._runtime_state_lock._is_owned())
            self.assertEqual(store._version_dir, self.root)
            loads.append((backend_kind, kwargs))
            return original_load(store, backend_kind, **kwargs)

        with patch.object(store_module.MappingBackendCheckpointStore, "load", load):
            result = self.activate()

        self.assertIs(result, self.provider.result)
        self.assertEqual(loads, [("voxel", {"schema_version": 2, "expected_checkpoint_id": "a" * 32})])
        self.assertEqual(len(self.provider.calls), 1)
        retained, stored = self.provider.calls[0]
        self.assertEqual(retained, expected)
        self.assertEqual(self.request, expected)
        self.assertIsInstance(stored, store_module.StoredMappingBackendCheckpoint)
        self.assertEqual(stored.checkpoint.schema_version, 2)
        self.assertEqual(stored.checkpoint.checkpoint_id, expected["checkpoint_id"])
        self.assertEqual(stored.identity, published.identity)
        self.assertEqual(stored.checkpoint.cloud_identity, published.checkpoint.cloud_identity)
        self.assertIsNone(stored.legacy_alias)
        self.assertEqual(stored.manifest_sha256, "")

    def test_both_cloud_backend_kinds_are_selected_explicitly(self):
        stored = self.publish()
        for kind in ("voxel", "raytracing"):
            with self.subTest(kind=kind), patch.object(
                store_module.MappingBackendCheckpointStore, "load", return_value=stored,
            ) as load:
                self.activate(backend_kind=kind)
                load.assert_called_once_with(kind, schema_version=2, expected_checkpoint_id="a" * 32)
                self.assertIs(self.provider.calls[-1][1], stored)

    def test_caller_mutation_during_io_does_not_change_admitted_request(self):
        stored = self.publish()
        expected = deepcopy(self.request)

        def load(*args, **kwargs):
            self.request.update(activation_request_id="new-request", expected_runtime_generation=999)
            self.request["asset_identity"]["asset_id"] = "f" * 32
            return stored

        with patch.object(store_module.MappingBackendCheckpointStore, "load", side_effect=load):
            self.activate()
        retained, _stored = self.provider.calls[0]
        self.assertEqual(retained, expected)
        self.assertIsNot(retained, self.request)
        self.assertIsNot(retained["asset_identity"], self.request["asset_identity"])

    def test_missing_mixed_and_invalid_request_metadata_fail_before_io(self):
        mutations = [
            lambda value, field=field: value.pop(field) for field in self.request
        ] + [
            lambda value: value.update(asset_manifest_sha256="0" * 64),
            lambda value: value.update(schema_version=2),
            lambda value: value.update(activation_request_id=""),
            lambda value: value.update(activation_request_id=" retained "),
            lambda value: value.update(session_id=""),
            lambda value: value.update(map_name="room "),
            lambda value: value.update(checkpoint_id="0" * 32),
            lambda value: value.update(checkpoint_id="A" * 32),
            lambda value: value.update(expected_runtime_generation=True),
            lambda value: value.update(expected_runtime_generation="37"),
            lambda value: value.update(expected_runtime_generation=0),
            lambda value: value.update(expected_runtime_generation=1 << 64),
            lambda value: value["asset_identity"].update(manifest_sha256="0" * 64),
            lambda value: value["asset_identity"].update(asset_id="f" * 64),
        ]
        with patch.object(store_module.MappingBackendCheckpointStore, "load") as load:
            for mutate in mutations:
                candidate = deepcopy(self.request)
                mutate(candidate)
                with self.subTest(candidate=candidate), self.assertRaises(ValueError):
                    self.activate(activation_request=candidate)
            for candidate in (None, [], "request"):
                with self.subTest(candidate=candidate), self.assertRaises(ValueError):
                    self.activate(activation_request=candidate)
            load.assert_not_called()
        self.assertEqual(self.provider.calls, [])

    def test_unsupported_backend_fails_before_io_or_dispatch(self):
        with patch.object(store_module.MappingBackendCheckpointStore, "load") as load:
            for kind in ("slam_toolbox", "Voxel", " voxel", "", None, [], True):
                with self.subTest(kind=kind), self.assertRaisesRegex(ValueError, "voxel or raytracing"):
                    self.activate(backend_kind=kind)
            load.assert_not_called()
        self.assertEqual(self.provider.calls, [])

    def test_unavailable_or_noncallable_explicit_provider_fails_before_io(self):
        unsupported = [None, SimpleNamespace(is_connected=True),
                       SimpleNamespace(is_connected=True, activate_assigned_platform_map=object()),
                       SimpleNamespace(is_connected=False, activate_assigned_platform_map=lambda **kwargs: None)]
        with patch.object(store_module.MappingBackendCheckpointStore, "load") as load:
            for provider in unsupported:
                self.owner._remote_provider = provider
                with self.subTest(provider=provider), self.assertRaisesRegex(RuntimeError, "does not support"):
                    self.activate()
            load.assert_not_called()
        self.assertEqual(self.provider.calls, [])

    def test_missing_checkpoint_never_falls_back_to_legacy(self):
        # A legacy selector is deliberately present; this explicit path only
        # opens checkpoint-v2.json and must not even inspect its legacy peer.
        legacy = self.root / "mapping_backend/voxel/checkpoint-v1.json"
        legacy.parent.mkdir(parents=True)
        legacy.write_text("legacy sentinel that must remain untouched")
        with self.assertRaises(store_module.MappingBackendCheckpointNotFoundError):
            self.activate()
        self.assertEqual(self.provider.calls, [])
        self.assertEqual(legacy.read_text(), "legacy sentinel that must remain untouched")

    def test_changed_expected_checkpoint_is_rejected_without_dispatch(self):
        self.publish()
        self.request["checkpoint_id"] = "e" * 32
        with self.assertRaisesRegex(ValueError, "expected assigned identity"):
            self.activate()
        self.assertEqual(self.provider.calls, [])

    def test_missing_mixed_or_changed_saved_metadata_is_rejected(self):
        stored = self.publish()
        original = stored.manifest_path.read_bytes()
        mutations = [
            lambda value: value.pop("checkpoint_id"),
            lambda value: value.update(content_sha256="0" * 64),
            lambda value: value.update(checkpoint_id="e" * 32),
            lambda value: value.update(schema_version=1),
            lambda value: value.update(backend_kind="raytracing"),
            lambda value: value["members"]["canonical_cloud"]["identity"].update(checkpoint_id="e" * 32),
        ]
        for mutate in mutations:
            candidate = json.loads(original)
            mutate(candidate)
            # Keep canonical encoding so the test reaches metadata admission.
            raw = json.dumps(candidate, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode()
            stored.manifest_path.write_bytes(raw)
            with self.subTest(mutate=mutate), self.assertRaises(ValueError):
                self.activate()
            self.assertEqual(self.provider.calls, [])
        stored.manifest_path.write_bytes(original)

    def test_store_errors_propagate_unchanged_without_retry_or_dispatch(self):
        for error in (OSError("unreadable store"), ValueError("invalid checkpoint"),
                      store_module.MappingBackendCheckpointNotFoundError("missing checkpoint")):
            with self.subTest(error=error), patch.object(
                store_module.MappingBackendCheckpointStore, "load", side_effect=error,
            ) as load:
                with self.assertRaises(type(error)) as raised:
                    self.activate()
                self.assertIs(raised.exception, error)
                load.assert_called_once_with("voxel", schema_version=2, expected_checkpoint_id="a" * 32)
        self.assertEqual(self.provider.calls, [])

    def test_provider_errors_including_unknown_outcome_propagate_without_retry(self):
        self.publish()
        errors = [ValueError("request rejected"), RuntimeError("disconnected"),
                  provider_module.GeometryRuntimeMapActivationOutcomeUnknown("ACK lost")]
        for error in errors:
            with self.subTest(error=error):
                self.provider.error = error
                before = len(self.provider.calls)
                with self.assertRaises(type(error)) as raised:
                    self.activate()
                self.assertIs(raised.exception, error)
                self.assertEqual(len(self.provider.calls), before + 1)
                self.assertEqual(self.provider.calls[-1][0], self.request)
        self.assertTrue(errors[-1].backend_activation_outcome_unknown)

    def test_nonapplied_result_is_returned_unchanged(self):
        self.publish()
        self.provider.result = {"backend_switch_applied": False, "reason": "runtime_generation_changed"}
        self.assertIs(self.activate(), self.provider.result)
        self.assertEqual(len(self.provider.calls), 1)

    def test_no_python_activation_installation_or_default_workspace_mutation(self):
        self.publish()
        before = dict(vars(self.owner))
        session = deepcopy(self.owner._mapping_session_context)
        with ExitStack() as guard:
            for name in ("activate_map_asset_bundle", "_activate_backend_map_bundle",
                         "_install_map_activation_snapshot_locked", "_map_activation_snapshot_read",
                         "_require_bundle_root", "_resolve_mapping_backend"):
                guard.enter_context(patch.object(self.owner, name, side_effect=AssertionError(name)))
            guard.enter_context(patch.object(module.uuid, "uuid4", side_effect=AssertionError("request ID generated")))
            self.activate()
        self.assertEqual(vars(self.owner), before)
        self.assertEqual(self.owner._mapping_session_context, session)
        self.assertIs(self.owner._map_activation_snapshot, before["_map_activation_snapshot"])

    def test_default_backend_path_still_rejects_schema2_or_skips_switch(self):
        self.owner.config = SimpleNamespace(stack="test-stack")
        bundle = SimpleNamespace(
            map_name="assigned room", manifest_sha256=None, frame_id="map",
            geometry_asset=SimpleNamespace(mapping_database_path="map.db", pcd_path="map.pcd",
                metadata={"mapping_backend_checkpoint": {"present": True, "schema_version": 2}}),
        )
        with patch.object(self.owner, "_resolve_mapping_backend", return_value=self.provider), patch.object(
            self.owner, "_summarize_geometry_asset", return_value={},
        ), patch.object(self.owner, "activate_assigned_backend_checkpoint", side_effect=AssertionError("implicit opt-in")):
            with self.assertRaisesRegex(RuntimeError, "CHECKPOINT_V2_ACTIVATION_UNAVAILABLE"):
                self.owner._activate_backend_map_bundle(bundle, bundle_root=self.root)
            result = self.owner._activate_backend_map_bundle(
                bundle, bundle_root=self.root, require_backend_switch=False,
            )
        self.assertFalse(result["backend_switch_applied"])
        self.assertEqual(result["reason"], "require_backend_switch_false")
        self.assertEqual(self.provider.calls, [])


if __name__ == "__main__":
    unittest.main()
