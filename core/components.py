"""网关调用本机业务监督者的唯一控制接口。"""
import json
import os
from pathlib import Path
import socket
from typing import Sequence

from core.errors import GatewayError


def request(payload: dict, timeout_sec: float) -> dict:
    home = Path(os.environ.get("NUWAX_INSTALL_HOME", Path(__file__).resolve().parents[1]))
    with socket.socket(socket.AF_UNIX) as connection:
        connection.settimeout(timeout_sec)
        connection.connect(str(home / "run/supervisor.sock"))
        connection.sendall((json.dumps(payload) + "\n").encode())
        with connection.makefile("rb") as response:
            return json.loads(response.readline())


def ensure(names: Sequence[str], timeout_sec: float) -> None:
    try:
        result = request({"op": "ensure", "components": list(names)}, timeout_sec)
    except OSError as error:
        result = {"component": ",".join(names), "error": str(error), "ok": False}
    if not result["ok"]:
        raise GatewayError("组件不可用：%s：%s" % (result["component"], result["error"]),
            error_code="component_unavailable", http_status=503, jsonrpc_code=-32010,
            details={"component": result["component"], "error": result["error"]})


def status() -> dict:
    return request({"op": "status"}, 2)
