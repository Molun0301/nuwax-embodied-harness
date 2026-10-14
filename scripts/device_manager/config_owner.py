"""用业务版本解释器持有共同配置快照；不启动业务或算法拥有方。"""
from __future__ import annotations

import argparse
import importlib.util
import json
import os
import signal
import socket
import struct
import sys
from dataclasses import asdict
from pathlib import Path

# 脚本直接运行，不把管理包的scripts命名空间混入业务解释器。
_spec = importlib.util.spec_from_file_location("neh_configuration_common", Path(__file__).with_name("common.py"))
_common = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_common)
MAX_MESSAGE = 8 * 1024 * 1024


def serve(home, prefix, address):
    home, prefix = Path(home).resolve(), Path(prefix).resolve()
    if prefix.parent != home / "releases" or (home / "current").resolve() != prefix:
        raise RuntimeError("配置拥有方只能服务当前受管业务版本")
    sys.path.insert(0, str(prefix / "source"))
    from core import config_io
    from core.config_profiles import ConfigurationConflict, ProfileRevisionOwner

    owner = ProfileRevisionOwner()
    stopping = [False]
    def stop(_signal, _frame):
        stopping[0] = True
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    listener.settimeout(0.5)
    state_path = home / "config/profile-owner.json"
    try:
        listener.bind(str(address))
        Path(address).chmod(0o600)
        listener.listen(16)
        _common.atomic_json(state_path, {"protocol": 1, "prefix": str(prefix), "process": _common.process_identity(os.getpid()), "address": str(address)})
        while not stopping[0] and (home / "current").resolve() == prefix:
            try:
                connection, _ = listener.accept()
            except socket.timeout:
                continue
            with connection:
                connection.settimeout(30)
                try:
                    peer = struct.unpack("3i", connection.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, struct.calcsize("3i")))
                    if peer[1] != os.getuid():
                        continue
                    with connection.makefile("rb") as reader:
                        raw = reader.readline(MAX_MESSAGE + 1)
                    if len(raw) > MAX_MESSAGE or not raw.endswith(b"\n"):
                        raise ValueError("配置请求过大或不完整")
                    request = json.loads(raw)
                    if not isinstance(request, dict) or request.get("prefix") != str(prefix) or (home / "current").resolve() != prefix:
                        raise ConfigurationConflict("业务版本已变化，请刷新配置。")
                    operation = request.get("operation")
                    if operation == "ping":
                        result = {"prefix": str(prefix), "protocol": 1}
                    elif operation == "stop":
                        stopping[0] = True
                        result = {"stopping": True}
                    elif operation in ("get", "revision"):
                        schema = config_io.build_schema(revision_owner=owner,
                            runtime_state_provider=lambda: {"runtime_values": {}, "saved": set()})
                        context = schema.get("profile_context") or {}
                        if not context.get("revision"):
                            raise ConfigurationConflict("当前没有可编辑的受管配置方案，请先恢复方案选择。")
                        result = schema if operation == "get" else {key: context.get(key) for key in ("robot", "profile", "revision")}
                    elif operation == "save":
                        expected = request.get("revision")
                        if not isinstance(expected, str) or not expected:
                            raise ConfigurationConflict("保存配置需要当前编辑版本，请刷新后重试。")
                        if not isinstance(request.get("values"), dict):
                            raise ValueError("配置更新必须是对象")
                        result = asdict(config_io.save_values(request["values"], expected_revision=expected, revision_owner=owner))
                    else:
                        raise ValueError("未知配置操作")
                    response = {"result": result}
                except ConfigurationConflict as exc:
                    response = {"error": {"code": "config_conflict", "message": str(exc), "status": 409}}
                except (ValueError, TypeError, KeyError):
                    response = {"error": {"code": "invalid_config", "message": "配置请求不合法。", "status": 400}}
                except Exception as exc:
                    response = {"error": {"code": "config_unavailable", "message": "共同配置读取或保存失败：" + type(exc).__name__, "status": 503}}
                try:
                    connection.sendall(json.dumps(response, ensure_ascii=False).encode() + b"\n")
                except OSError:
                    pass
    finally:
        owner.close()
        listener.close()
        Path(address).unlink(missing_ok=True)
        record = _common.read_json(state_path, {})
        if (record.get("process") or {}).get("pid") == os.getpid():
            state_path.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--home", required=True)
    parser.add_argument("--prefix", required=True)
    parser.add_argument("--socket", required=True)
    args = parser.parse_args()
    serve(args.home, args.prefix, args.socket)


if __name__ == "__main__":
    main()
