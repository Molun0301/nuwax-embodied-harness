"""Public gateway/handler identity routing; no service or robot is started."""
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock

from contracts.relocalization_request_id import relocalization_request_identity as contract_identity
from core.errors import GatewayError
from core.relocalization_request_id import relocalization_request_identity
from gateways.runtime import GatewayRuntime
from skills.handlers.localization_skills import handle_relocalize_active_map


class RelocalizationRequestIdTests(unittest.TestCase):
    def test_identity_is_not_trimmed_hashed_or_coerced(self):
        for value in ("000123", "Order-A._:-09", "x" * 128):
            self.assertEqual(relocalization_request_identity({"business_request_id": value}), (value, False))
            self.assertEqual(relocalization_request_identity({"request_id": value}), (value, True))
        self.assertEqual(contract_identity({}), (None, False))
        for field in ("business_request_id", "request_id"):
            for value in ("", " 123", "123 ", 123, True, "中文编号", "a/b", "x" * 129):
                payload = {field: value}
                with self.subTest(field=field, value=value):
                    with self.assertRaises(ValueError) as contract_error:
                        contract_identity(payload)
                    with self.assertRaises(GatewayError) as error:
                        relocalization_request_identity(payload)
                    self.assertEqual(str(error.exception), str(contract_error.exception))
                    self.assertEqual(error.exception.error_code, "invalid_params")
                    self.assertEqual(error.exception.http_status, 422)
                    self.assertEqual(error.exception.jsonrpc_code, -32602)
        with self.assertRaises(GatewayError) as error:
            relocalization_request_identity({"business_request_id": "123", "request_id": "123"})
        self.assertEqual(error.exception.error_code, "invalid_params")
        self.assertEqual(error.exception.http_status, 422)
        self.assertEqual(error.exception.jsonrpc_code, -32602)

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
        with self.assertRaises(GatewayError) as error:
            handle_relocalize_active_map(ctx, {"business_request_id": "123", "request_id": "123"})
        self.assertEqual(error.exception.error_code, "invalid_params")
        self.assertEqual(error.exception.http_status, 422)
        self.assertEqual(error.exception.jsonrpc_code, -32602)
        ctx.ensure_geometry_runtime_for_capability.assert_not_called()
