"""Public gateway/handler identity routing; no service or robot is started."""
import unittest
import json
import tempfile
from copy import deepcopy
from types import SimpleNamespace
from unittest.mock import MagicMock

from contracts.relocalization_request_id import relocalization_request_identity
from core.errors import GatewayError
from gateways.runtime import GatewayRuntime
from skills.handlers.localization_skills import handle_relocalize_active_map
from skills.map_workspace import MapWorkspaceService
from services.mapping.catalog import MapCatalogRepository


class RelocalizationRequestIdTests(unittest.TestCase):
    def test_identity_is_not_trimmed_hashed_or_coerced(self):
        for value in ("000123", "Order-A._:-09", "x" * 128):
            self.assertEqual(relocalization_request_identity({"business_request_id": value}), (value, False))
            self.assertEqual(relocalization_request_identity({"request_id": value}), (value, True))
        for value in ("", " 123", "123 ", 123, True, "中文编号", "a/b", "x" * 129):
            with self.subTest(value=value), self.assertRaises(GatewayError):
                relocalization_request_identity({"business_request_id": value})
        with self.assertRaises(GatewayError):
            relocalization_request_identity({"business_request_id": "123", "request_id": "123"})

    def submit(self, payload, request_context=None):
        gateway = SimpleNamespace(
            capability_registry=SimpleNamespace(get_descriptor=lambda _: SimpleNamespace(required_resources=())),
            _resolve_async_timeout=lambda *args, **kwargs: 60,
            _build_task_request_metadata=lambda **kwargs: {},
            _async_task_polling_guidance=lambda _: None, task_manager=MagicMock())
        GatewayRuntime._submit_geometry_skill_task(gateway, capability_name="relocalize_active_map",
            skill_handler=MagicMock(), arguments=payload, requested_by="test", request_context=request_context)
        return gateway.task_manager.submit.call_args.kwargs["input_payload"]

    def test_gateway_preserves_explicit_ids_and_does_not_use_transport_id(self):
        for field in ("business_request_id", "request_id"):
            payload = {field: "000123"}
            self.assertEqual(self.submit(payload, {"request_id": "json-rpc-7"}), payload)
        first = self.submit({}, {"request_id": "json-rpc-7"})
        second = self.submit({}, {"request_id": "json-rpc-7"})
        self.assertNotEqual(first["business_request_id"], second["business_request_id"])
        self.assertTrue(first["business_request_id"].startswith("relocalize:"))
        self.assertNotIn("request_id", first)

    def test_handler_passes_both_modes_unchanged_and_rejects_ambiguous_input_before_startup(self):
        ctx = SimpleNamespace(ensure_geometry_runtime_for_capability=MagicMock(),
            map_workspace_service=SimpleNamespace(relocalize_active_map=MagicMock(return_value=(None, None))),
            refresh_workspace_snapshots=MagicMock(), publish_runtime_governance_stream_frame=MagicMock())
        for field in ("business_request_id", "request_id"):
            handle_relocalize_active_map(ctx, {field: "000123"})
            metadata = ctx.map_workspace_service.relocalize_active_map.call_args.kwargs["metadata"]
            self.assertEqual(metadata[field], "000123")
        ctx.ensure_geometry_runtime_for_capability.reset_mock()
        with self.assertRaises(GatewayError):
            handle_relocalize_active_map(ctx, {"business_request_id": "123", "request_id": "123"})
        ctx.ensure_geometry_runtime_for_capability.assert_not_called()


class LegacyRelocalizationRequestIdTests(unittest.TestCase):
    def setUp(self):
        self.service = object.__new__(MapWorkspaceService)
        self.repo = MagicMock()
        self.service._repository = self.repo
        self.service._localization_service = MagicMock()
        self.runtime = MagicMock()
        self.service._geometry_runtime = self.runtime
        self.repo.get_relocalization_intent.return_value = None
        self.repo.get_last_successful_relocalization_receipt.return_value = None
        self.activation = {"trusted_identity": True, "target_map_name": "map",
                           "target_manifest_sha256": "a" * 64}
        self.repo.get_last_successful_map_activation_receipt.return_value = self.activation
        workspace = SimpleNamespace(active_map_name="map", map_loaded=True, latest_map_snapshot=object(),
            map_asset=SimpleNamespace(finalized=True, asset_manifest_sha256="a" * 64))
        self.service.get_active_workspace = MagicMock(return_value=workspace)
        self.service._require_map_asset_bundle = MagicMock(return_value=SimpleNamespace(manifest_sha256="a" * 64))
        self.runtime.describe_backend.return_value = {"active_map_name": "map", "asset_manifest_sha256": "a" * 64}
        self.runtime.get_relocalization_terminal.return_value = {"status": "not_found", "terminal_receipt": None}

    def test_unknown_old_id_is_only_queried_without_intent_or_dispatch(self):
        self.runtime.get_relocalization_terminal.return_value = {"status": "not_found", "terminal_receipt": None}
        with self.assertRaises(GatewayError) as error:
            self.service._relocalize_legacy_active_map(map_name="map", metadata={"request_id": "old-123"})
        self.assertEqual(error.exception.error_code, "relocalization_request_not_found")
        self.repo.begin_relocalization_durable.assert_not_called()
        self.runtime.relocalize_active_map.assert_not_called()
        self.assertEqual(self.runtime.get_relocalization_terminal.call_args.kwargs["request_id"], "old-123")

    def test_business_id_reaches_durable_dispatch_unchanged_even_if_outcome_unknown(self):
        self.runtime.relocalize_active_map.side_effect = RuntimeError("lost reply")
        with self.assertRaises(GatewayError) as error:
            self.service._relocalize_legacy_active_map(map_name="map", metadata={"business_request_id": "000123"})
        self.assertEqual(error.exception.error_code, "relocalization_outcome_unknown")
        self.assertEqual(self.repo.begin_relocalization_durable.call_args.kwargs["request_id"], "000123")
        self.assertEqual(self.runtime.relocalize_active_map.call_args.kwargs["request_id"], "000123")
        self.repo.mark_relocalization_outcome_unknown.assert_called_once_with(request_id="000123")
        self.assertEqual(self.runtime.get_relocalization_terminal.call_count, 1)

    def test_old_prepared_id_keeps_original_intent(self):
        self.repo.get_relocalization_intent.return_value = {
            "target_map_name": "map", "target_manifest_sha256": "a" * 64,
            "request_id": "old-123", "phase": "prepared", "logical_timeout_ms": 10_000}
        with self.assertRaises(GatewayError) as error:
            self.service._relocalize_legacy_active_map(map_name="map", metadata={"request_id": "old-123"})
        self.assertEqual(error.exception.error_code, "relocalization_recovery_required")
        self.repo.abort_relocalization_before_apply.assert_not_called()
        self.runtime.relocalize_active_map.assert_not_called()

    @staticmethod
    def historical_terminal(request_id, disposition=1, disposition_name="applied"):
        # Complete legacy runtime projection, including its existing proof
        # fields. No partial request_id-only success fixture is used here.
        receipt = dict(schema_version=1, request_id=request_id, session_id="installed-session",
            map_name="map", disposition=disposition, disposition_name=disposition_name,
            logical_timeout_ms=10_000)
        for field in ("runtime_generation", "session_generation", "backend_generation",
                "target_cloud_revision", "source_policy_version", "deadline_policy_version",
                "algorithm_options_version", "candidate_sequence", "correction_revision"):
            receipt[field] = 1
        for field in ("request_identity_sha256", "activation_identity_sha256", "target_receipt_sha256",
                "target_cloud_sha256", "asset_manifest_sha256", "asset_bundle_sha256",
                "source_policy_sha256", "deadline_policy_sha256", "algorithm_options_sha256",
                "candidate_payload_sha256", "evidence_sha256", "terminal_proof_sha256"):
            receipt[field] = "a" * 64
        return receipt

    def assert_history_readonly(self, disposition, disposition_name, *, has_current, field="request_id"):
        with tempfile.TemporaryDirectory() as root:
            repository = MapCatalogRepository(root)
            self.service._repository = repository
            # The real catalog validates/reads persisted relocalization records;
            # only the unrelated sealed activation is supplied by this fixture.
            repository.get_last_successful_map_activation_receipt = lambda: self.activation
            historical = self.historical_terminal("operation-A", disposition, disposition_name)
            if disposition_name not in {"applied", "already_applied"}:
                with self.assertRaisesRegex(ValueError, "不是成功终态"):
                    repository._validate_relocalization_receipt(historical)
            else:
                self.assertEqual(repository._validate_relocalization_receipt(historical), historical)
            path = repository._relocalization_receipt_path
            before_receipt = None
            if has_current:
                receipt_b = self.historical_terminal("operation-B")
                self.assertEqual(repository._validate_relocalization_receipt(receipt_b), receipt_b)
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(json.dumps(receipt_b))
                before_receipt = path.read_bytes()
            session_b = SimpleNamespace(pose="current-B-pose", request_id="operation-B")
            self.service._localization_service.get_active_session.return_value = session_b
            before_session = deepcopy(session_b.__dict__)
            self.service._localization_service.reset_mock()
            self.runtime.reset_mock()
            self.runtime.get_relocalization_terminal.return_value = {
                "status": "terminal", "terminal_receipt": historical}
            with self.assertRaises(GatewayError) as error:
                self.service._relocalize_legacy_active_map(map_name="map",
                    metadata={field: "operation-A", "logical_timeout_ms": 10_000})
            self.assertEqual(error.exception.error_code, "relocalization_historical_terminal")
            self.assertEqual(error.exception.details["terminal"], historical)
            self.assertIsNone(repository.get_relocalization_intent())
            self.assertFalse(repository._relocalization_intent_path.exists())
            self.assertEqual(path.read_bytes() if path.exists() else None, before_receipt)
            self.assertEqual(session_b.__dict__, before_session)
            self.assertEqual(self.service._localization_service.mock_calls, [])
            self.runtime.relocalize_active_map.assert_not_called()

    def test_historical_success_after_newer_receipt_is_readonly(self):
        for disposition, name in ((1, "applied"), (2, "already_applied")):
            with self.subTest(disposition=name):
                self.assert_history_readonly(disposition, name, has_current=True)

    def test_historical_rejection_and_cancellation_never_create_owner(self):
        for disposition, name in ((3, "rejected"), (10, "cancelled")):
            for has_current in (False, True):
                with self.subTest(disposition=name, has_current=has_current):
                    self.assert_history_readonly(disposition, name, has_current=has_current)

    def test_business_historical_success_rejection_and_cancellation_are_readonly(self):
        for disposition, name in ((1, "applied"), (3, "rejected"), (10, "cancelled")):
            with self.subTest(disposition=name):
                self.assert_history_readonly(disposition, name, has_current=True, field="business_request_id")

    def real_catalog(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        repository = MapCatalogRepository(temp.name)
        activation = dict(schema_version=2, target_map_name="map", target_manifest_sha256="a" * 64,
            activation_request_id="mapload:" + "1" * 32, session_id="mapload:" + "1" * 32,
            activation_receipt_schema_version=1, activation_receipt_disposition=2,
            activation_receipt_disposition_name="sealed", mapping_backend_kind="voxel",
            relocalization_target_available=True)
        for field in ("runtime_generation", "session_generation", "backend_generation",
                "cloud_source_revision", "source_policy_version", "deadline_policy_version", "algorithm_options_version"):
            activation[field] = 1
        for field in ("activation_request_identity_sha256", "activation_identity_sha256",
                "target_receipt_sha256", "asset_bundle_sha256", "cloud_source_sha256",
                "source_policy_sha256", "deadline_policy_sha256", "algorithm_options_sha256"):
            activation[field] = "a" * 64
        repository._activation_receipt_path.write_text(json.dumps(activation))
        self.assertTrue(repository.get_last_successful_map_activation_receipt()["trusted_identity"])
        self.service._repository = repository
        return repository

    def test_business_preflight_non_not_found_never_changes_real_catalog(self):
        repository = self.real_catalog()
        receipt_b = self.historical_terminal("operation-B")
        repository._relocalization_receipt_path.write_text(json.dumps(receipt_b))
        before = repository._relocalization_receipt_path.read_bytes()
        for status in ("identity_conflict", "pending", "unknown", "terminal"):
            self.runtime.get_relocalization_terminal.return_value = {"status": status, "terminal_receipt": None}
            with self.subTest(status=status), self.assertRaises(GatewayError) as error:
                self.service._relocalize_legacy_active_map(map_name="map",
                    metadata={"business_request_id": "operation-A", "logical_timeout_ms": 250})
            self.assertEqual(error.exception.error_code,
                "relocalization_request_conflict" if status == "identity_conflict" else "relocalization_recovery_required")
            self.assertEqual(self.runtime.get_relocalization_terminal.call_args.kwargs["logical_timeout_ms"], 250)
            self.assertEqual(repository._relocalization_receipt_path.read_bytes(), before)
            self.assertIsNone(repository.get_relocalization_intent())
        self.runtime.get_relocalization_terminal.side_effect = RuntimeError("query unavailable")
        with self.assertRaisesRegex(RuntimeError, "query unavailable"):
            self.service._relocalize_legacy_active_map(map_name="map", metadata={"business_request_id": "operation-A"})
        self.assertEqual(repository._relocalization_receipt_path.read_bytes(), before)
        self.assertIsNone(repository.get_relocalization_intent())
        self.runtime.relocalize_active_map.assert_not_called()
        self.assertEqual(self.service._localization_service.mock_calls, [])

    def test_business_explicit_not_found_creates_real_durable_original_request(self):
        repository = self.real_catalog()
        def query(**kwargs):
            self.assertIsNone(repository.get_relocalization_intent())
            self.assertEqual(self.service._localization_service.mock_calls, [])
            return {"status": "not_found", "terminal_receipt": None}
        def dispatch(**kwargs):
            pending = MapCatalogRepository(str(repository._root_dir)).get_relocalization_intent()
            self.assertEqual(pending["phase"], "backend_dispatching")
            self.assertEqual(pending["request_id"], "000123")
            raise RuntimeError("reply lost after dispatch")
        self.runtime.get_relocalization_terminal.side_effect = query
        self.runtime.relocalize_active_map.side_effect = dispatch
        with self.assertRaises(GatewayError) as error:
            self.service._relocalize_legacy_active_map(map_name="map", metadata={"business_request_id": "000123"})
        self.assertEqual(error.exception.error_code, "relocalization_outcome_unknown")
        pending = MapCatalogRepository(str(repository._root_dir)).get_relocalization_intent()
        self.assertEqual(pending["request_id"], "000123")
        self.assertEqual(pending["phase"], "backend_outcome_unknown")
        self.assertEqual(self.runtime.get_relocalization_terminal.call_count, 1)
        self.assertEqual(self.runtime.relocalize_active_map.call_count, 1)

    def test_completed_business_id_retry_and_timeout_conflict(self):
        self.repo.get_last_successful_relocalization_receipt.return_value = {
            "request_id": "000123", "map_name": "map", "asset_manifest_sha256": "a" * 64,
            "logical_timeout_ms": 10_000}
        for field in ("business_request_id", "request_id"):
            self.service._relocalize_legacy_active_map(map_name="map",
                metadata={field: "000123", "logical_timeout_ms": 10_000})
        with self.assertRaises(ValueError):
            self.service._relocalize_legacy_active_map(map_name="map",
                metadata={"business_request_id": "000123", "logical_timeout_ms": 250})
        self.runtime.relocalize_active_map.assert_not_called()
        self.repo.begin_relocalization_durable.assert_not_called()
