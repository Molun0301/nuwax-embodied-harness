#!/usr/bin/env python3
"""离线验证 MCP 成功信封只编码一次，保留精确字节上限及错误合同。"""
from __future__ import annotations

import json
import sys
import unittest
from contextlib import ExitStack
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from starlette.requests import Request

from gateways.mcp import protocol


def wire_bytes(value):
    """原有字节计数器和 JSONResponse 使用的精确编码选项。"""
    return json.dumps(value, ensure_ascii=False, allow_nan=False,
                      separators=(",", ":")).encode("utf-8")


class McpResponseEncodingTests(unittest.IsolatedAsyncioTestCase):
    async def call_endpoint(self, method, *, tools=None, result=None, limit=None,
                            tool_result=None, request_id="回执-😀\n1"):
        invocation = Mock(return_value=result)
        runtime = SimpleNamespace(invoke_tool=invocation)
        router = protocol.build_mcp_router(runtime, SimpleNamespace())
        endpoint = next(route.endpoint for route in router.routes
                        if "POST" in route.methods)
        payload = {"jsonrpc": "2.0", "id": request_id, "method": method}
        if method == "tools/call":
            payload["params"] = {"name": "offline_tool", "arguments": {}}
        raw = wire_bytes(payload)

        async def receive():
            return {"type": "http.request", "body": raw, "more_body": False}

        request = Request({
            "type": "http", "http_version": "1.1", "method": "POST",
            "scheme": "http", "path": "/mcp", "query_string": b"",
            "headers": [(b"content-type", b"application/json")],
            "server": ("offline", 80), "client": ("127.0.0.1", 12345),
            "state": {"request_id": "offline-request"},
        }, receive)
        dumps = json.dumps
        encodings = []

        def count_envelopes(value, *args, **kwargs):
            encoded = dumps(value, *args, **kwargs)
            if (isinstance(value, dict) and value.get("jsonrpc") == "2.0"
                    and value.get("result") is not None):
                encodings.append(encoded.encode("utf-8"))
            return encoded

        with ExitStack() as stack:
            stack.enter_context(patch.object(protocol, "_build_collection_tools", return_value=tools))
            stack.enter_context(patch.object(protocol, "_resolve_tool_invocation",
                                             return_value=("offline_tool", {})))
            stack.enter_context(patch.object(protocol.MCP_LOGGER, "warning"))
            stack.enter_context(patch.object(protocol.MCP_LOGGER, "exception"))
            if limit is not None:
                stack.enter_context(patch.object(protocol, "MCP_RESULT_MAX_BYTES", limit))
            if tool_result is not None:
                stack.enter_context(patch.object(protocol, "build_mcp_tool_result",
                                                 return_value=tool_result))
            # JSONResponse 与 protocol 使用同一标准库模块；按完整信封计数，
            # 不把输入、工具内部检查或空信封预算混入重复编码指标。
            stack.enter_context(patch.object(json, "dumps", side_effect=count_envelopes))
            response = await endpoint(request)
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.headers["content-type"], "application/json")
        self.assertEqual(int(response.headers["content-length"]), len(response.body))
        self.assertEqual(json.loads(response.body)["id"], request_id)
        if method == "tools/call":
            invocation.assert_called_once()
            self.assertFalse(invocation.call_args.kwargs["project_result"])
        else:
            invocation.assert_not_called()
        return response, encodings

    def assert_error(self, response, code, jsonrpc_code):
        payload = json.loads(response.body)
        self.assertNotIn("result", payload)
        self.assertEqual(payload["error"]["code"], jsonrpc_code)
        self.assertEqual(payload["error"]["data"]["code"], code)

    async def test_tools_list_encodes_final_envelope_once_with_exact_bytes(self):
        tools = [{"name": "offline_tool", "description": "地图😀\n\t\"\\" * 2048,
                  "inputSchema": {"type": "object", "properties": {}}}]
        expected = wire_bytes(protocol._jsonrpc_result("回执-😀\n1", {"tools": tools}))
        response, encodings = await self.call_endpoint("tools/list", tools=tools)
        self.assertEqual(response.body, expected)
        self.assertEqual(len(encodings), 1)
        self.assertEqual(encodings[0], expected)

    async def test_tools_call_encodes_final_envelope_once_with_complete_result(self):
        result = {"capability_name": "offline_tool", "result": {
            "message": "地图😀\n\t\"\\", "values": [None, True, False, -0.0, 1e-12, 2**80],
            "trajectory": [{"x": index * 0.1, "y": -index * 0.1} for index in range(1024)],
            "api_key": "must-not-leak",
        }}
        projected = protocol.build_mcp_tool_result(result)
        expected = wire_bytes(protocol._jsonrpc_result("回执-😀\n1", projected))
        response, encodings = await self.call_endpoint("tools/call", result=result)
        self.assertEqual(response.body, expected)
        self.assertEqual(len(encodings), 1)
        self.assertEqual(encodings[0], expected)
        body = json.loads(response.body)["result"]
        self.assertEqual(json.loads(body["content"][0]["text"]), body["structuredContent"])
        self.assertEqual(len(body["structuredContent"]["result"]["trajectory"]), 1024)
        self.assertEqual(body["structuredContent"]["result"]["api_key"], "********")
        self.assertEqual(result["result"]["api_key"], "must-not-leak")

    async def test_tools_list_exact_limit_and_one_byte_over(self):
        tools = [{"name": "offline_tool", "description": "完整地图😀" * 128}]
        expected = wire_bytes(protocol._jsonrpc_result("回执-😀\n1", {"tools": tools}))
        response, _ = await self.call_endpoint("tools/list", tools=tools, limit=len(expected))
        self.assertEqual(response.body, expected)
        rejected, _ = await self.call_endpoint("tools/list", tools=tools, limit=len(expected) - 1)
        self.assert_error(rejected, "mcp_response_too_large", -32603)

    async def test_tools_call_exact_limit_and_one_byte_over(self):
        result = {"result": {"message": "完整地图😀" * 128}}
        projected = protocol.build_mcp_tool_result(result)
        expected = wire_bytes(protocol._jsonrpc_result("回执-😀\n1", projected))
        response, _ = await self.call_endpoint("tools/call", result=result, limit=len(expected))
        self.assertEqual(response.body, expected)
        rejected, _ = await self.call_endpoint("tools/call", result=result, limit=len(expected) - 1)
        self.assert_error(rejected, "mcp_response_too_large", -32603)

    async def test_tools_call_final_envelope_limit_is_still_checked(self):
        # 单独覆盖路由上限，避免只覆盖构建器内部的提前拒绝。
        tool_result = {"structuredContent": {"message": "完整地图😀" * 128}}
        expected = wire_bytes(protocol._jsonrpc_result("回执-😀\n1", tool_result))
        response, _ = await self.call_endpoint("tools/call", result={},
                                               tool_result=tool_result, limit=len(expected))
        self.assertEqual(response.body, expected)
        rejected, _ = await self.call_endpoint("tools/call", result={},
                                               tool_result=tool_result, limit=len(expected) - 1)
        self.assert_error(rejected, "mcp_response_too_large", -32603)

    async def test_tools_list_non_finite_values_keep_existing_error(self):
        for value in (float("nan"), float("inf"), -float("inf")):
            with self.subTest(value=value):
                response, _ = await self.call_endpoint("tools/list", tools=[{"value": value}])
                self.assert_error(response, "execution_failed", -32010)

    async def test_tools_call_non_finite_values_keep_existing_error(self):
        for value in (float("nan"), float("inf"), -float("inf")):
            with self.subTest(value=value):
                response, _ = await self.call_endpoint("tools/call", result={"value": value})
                self.assert_error(response, "mcp_result_unavailable", -32603)

    async def test_tools_list_unsupported_values_keep_existing_error(self):
        response, _ = await self.call_endpoint("tools/list", tools=[{"value": object()}])
        self.assert_error(response, "execution_failed", -32010)

    async def test_tools_call_business_error_preserves_both_channels(self):
        result = {"result": {"ok": False, "error_code": "not_found", "message": "没有地图"}}
        expected = protocol.build_mcp_tool_result(result)
        response, _ = await self.call_endpoint("tools/call", result=result)
        self.assertEqual(json.loads(response.body)["result"], expected)
        self.assertTrue(expected["isError"])


if __name__ == "__main__":
    unittest.main()
