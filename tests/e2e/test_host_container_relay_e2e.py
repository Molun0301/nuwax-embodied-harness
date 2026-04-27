from __future__ import annotations
from pathlib import Path

from fastapi.testclient import TestClient
import httpx

from gateways.relay.server import create_relay_app
from tests.integration.test_gateway_integration import _build_host_app, _mcp_tool_call, _relay_headers


def test_host_container_relay_e2e_path(tmp_path: Path) -> None:
    """容器内 relay 到宿主机网关的主链路应完整可用。"""

    host_app, runtime, _, config = _build_host_app(tmp_path, start_runtime_on_lifespan=False)
    runtime.start()
    try:
        relay_app = create_relay_app(
            config.relay,
            transport=httpx.ASGITransport(app=host_app),
        )
        with TestClient(relay_app) as client:
            relay_health = client.get("/relay/health", headers=_relay_headers())
            assert relay_health.status_code == 200
            assert "sse" in relay_health.json()["supports"]

            initialize = client.post(
                "/mcp",
                headers=_relay_headers(),
                json={"jsonrpc": "2.0", "id": "init", "method": "initialize"},
            )
            assert initialize.status_code == 200
            assert initialize.json()["result"]["serverInfo"]["version"] == "0.9.0"

            tools = client.post(
                "/mcp",
                headers=_relay_headers(),
                json={"jsonrpc": "2.0", "id": "tools", "method": "tools/list"},
            )
            assert tools.status_code == 200
            tool_items = tools.json()["result"]["tools"]
            tool_names = {item["name"] for item in tool_items}
            assert tool_names == {"platform", "device"}
            platform_tool = next(item for item in tool_items if item["name"] == "platform")
            device_tool = next(item for item in tool_items if item["name"] == "device")
            platform_functions = set(platform_tool["inputSchema"]["properties"]["function"]["enum"])
            device_functions = set(device_tool["inputSchema"]["properties"]["function"]["enum"])
            assert "list_capabilities" in platform_functions
            assert "list_memory_libraries" in platform_functions
            assert "create_memory_library" in platform_functions
            assert "enable_memory_library" in platform_functions
            assert "disable_memory_library" in platform_functions
            assert "delete_memory_library" in platform_functions
            assert "capture_image" in device_functions
            assert platform_tool["inputSchema"]["x-nuwax-function-catalog"]["create_memory_library"]["input_schema"]["type"] == "object"

            create_memory = client.post(
                "/mcp",
                headers=_relay_headers(),
                json={
                    "jsonrpc": "2.0",
                    "id": "create_memory",
                    "method": "tools/call",
                    "params": _mcp_tool_call(
                        "platform",
                        "create_memory_library",
                        {
                            "library_name": "容器预创建记忆库",
                        },
                    ),
                },
            )
            assert create_memory.status_code == 200
            create_result = create_memory.json()["result"]["structuredContent"]["result"]
            assert create_result["create_result"]["created"] is True
            assert any(item["library_name"] == "容器预创建记忆库" for item in create_result["libraries"])

            enable_memory = client.post(
                "/mcp",
                headers=_relay_headers(),
                json={
                    "jsonrpc": "2.0",
                    "id": "enable_memory",
                    "method": "tools/call",
                    "params": _mcp_tool_call(
                        "platform",
                        "enable_memory_library",
                        {
                            "library_name": "容器联调记忆库",
                            "load_history": False,
                            "reset_library": False,
                        },
                    ),
                },
            )
            assert enable_memory.status_code == 200
            assert (
                enable_memory.json()["result"]["structuredContent"]["result"]["memory_summary"]["metadata"]["active_library_name"]
                == "容器联调记忆库"
            )

            delete_memory = client.post(
                "/mcp",
                headers=_relay_headers(),
                json={
                    "jsonrpc": "2.0",
                    "id": "delete_memory",
                    "method": "tools/call",
                    "params": _mcp_tool_call(
                        "platform",
                        "delete_memory_library",
                        {
                            "library_name": "容器联调记忆库",
                        },
                    ),
                },
            )
            assert delete_memory.status_code == 200
            delete_result = delete_memory.json()["result"]["structuredContent"]["result"]
            assert delete_result["delete_result"]["deleted"] is True
            assert delete_result["memory_summary"]["metadata"]["memory_enabled"] is False

            capture = client.post(
                "/mcp",
                headers=_relay_headers(),
                json={
                    "jsonrpc": "2.0",
                    "id": "capture",
                    "method": "tools/call",
                    "params": _mcp_tool_call("device", "capture_image"),
                },
            )
            assert capture.status_code == 200
            artifact_id = capture.json()["result"]["structuredContent"]["result"]["artifact"]["artifact_id"]

            security = client.get("/api/security/policy", headers=_relay_headers())
            layout = client.get("/api/deployment/layout", headers=_relay_headers())
            assert security.status_code == 200
            assert layout.status_code == 200
            assert layout.json()["state_store_mode"] == "in_memory"

            raw = client.get(f"/artifacts/{artifact_id}", headers=_relay_headers())
            assert raw.status_code == 200
            assert raw.content == b"fake-jpeg-data"
    finally:
        runtime.stop()
