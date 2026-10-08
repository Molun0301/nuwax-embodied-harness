#!/usr/bin/env python3
"""Offline rollout compatibility gates: real incoming policy and frozen old rollback.

Only temporary, self-created installations and ledgers are touched. Builds, Git,
network, managed process operations, and old installer hash computations are test
doubles. One explicitly named child fixture exits abruptly to verify reopening;
it never starts a gateway, hardware adapter, installation, or external command.
The old reader baseline is the accepted hash-free, versioned combined-store
checkpoint immediately before split V1, not an arbitrary historical binary.
These are process-reopen checks, not power-loss/filesystem durability claims.

Run: PYTHONDONTWRITEBYTECODE=1 python scripts/check_agent_terminal_rollout_policy.py
Optional NEH_TERMINAL_ROLLOUT_TMP places all fixtures in an existing scratch dir.
"""
from __future__ import annotations

import contextlib
import io
import json
import os
import shlex
import socket
import sqlite3
import subprocess
import sys
import tempfile
import types
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from core.agent_call_store import AgentCallStore
from gateways.agent.service import AgentToolService
from scripts.module_installer import hardware, lifecycle, manager, runtime

POLICY_ENV = "NUWAX_AGENT_TERMINAL_FRESH_LAYOUT"
IDENTITY = {"robot_model": "offline", "robot_name": "rollout-fixture",
            "entrypoint": "synthetic", "robot_id": "fixture-one"}
RESPONSE = {"task_id": "synthetic-task", "result": {"executed": 1}}
SNAPSHOT = {"task_id": "synthetic-task", "status": {"state": "succeeded"}, "result": {"executed": 1}}
EXIT_CODE = 73
MISSING = object()

# Frozen on 2026-10-02 before the compatibility policy was added. Keep these
# literal snapshots independent of imports from the incoming implementation.
# Coordinator: scripts/self_update.py immediately before fresh_layout integration.
# Reader: core/agent_call_store.py at the accepted pre-split hash-free, versioned
# combined checkpoint, with persistent identity assignment and additive columns.
# Manifest reader: scripts/module_installer/manager.py before capability metadata.
# The coordinator contains exact old execute() plus its real local transaction
# helpers; omitted external operations are injected as fail-closed doubles.
# The exercised prior combined APIs are exact extracts of the accepted pre-split
# store snapshot; no methods or SQL are rewritten.

_OLD_UPDATER_SOURCE = r'''
from __future__ import annotations

import argparse

import fcntl

import ipaddress

import json

import os

import re

import shutil

import signal

import subprocess

import sys

import time

import urllib.request

import uuid

from contextlib import contextmanager

from pathlib import Path

BUSY = {"queued", "fetching", "stopping", "building", "starting", "verifying", "rolling_back"}

def read_json(path, default=None):
    try:
        value = json.loads(Path(path).read_text(encoding="utf-8"))
        if not isinstance(value, dict):
            raise ValueError("更新状态必须为对象")  # noqa: TRY004 - 磁盘JSON格式错误
        return value
    except FileNotFoundError:
        return dict(default or {})

def fsync_directory(path):
    fd = os.open(str(path), os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)

def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + "." + uuid.uuid4().hex + ".tmp")
    try:
        with temporary.open("x", encoding="utf-8") as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        fsync_directory(path.parent)
    finally:
        temporary.unlink(missing_ok=True)

@contextmanager
def lock(root, name="transaction"):
    directory = root / "runtime_data/updates"
    directory.mkdir(parents=True, exist_ok=True)
    with (directory / (name + ".lock")).open("a") as stream:
        try:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError("已有更新操作正在执行") from exc
        yield

def state(root):
    value = read_json(root / "runtime_data/updates/state.json", {"state": "idle"})
    if value.get("state") in BUSY and time.time() - value.get("updated_at", value.get("created_at", 0)) > 60:
        identity = process_identity(value.get("worker_pid"))
        if not identity or identity != value.get("worker_identity"):
            return dict(value, state="recovery_required", message="更新进程已中断，请检查更新日志和记录的恢复版本")
    return value

def phase(root, status, name, message, **extra):
    status.update(state=name, message=message, updated_at=time.time(), **extra)
    write_json(root / "runtime_data/updates/state.json", status)
    print(json.dumps({"event": "system.update", **status}, ensure_ascii=False), flush=True)

def require_clean(root):
    if git(root, "status", "--porcelain", "--untracked-files=normal"):
        raise RuntimeError("源码存在本地修改或未跟踪文件，拒绝覆盖；请先提交或备份处理")

def mark_generation(prefix, revision):
    manifest = read_json(prefix / "manifest.json")
    manifest.update(updater_managed=True, source_revision=revision)
    write_json(prefix / "manifest.json", manifest)

def execute(root, request_id):
    root = root.resolve()
    with lock(root):
        status = state(root)
        if status.get("request_id") != request_id or status.get("state") != "queued":
            raise RuntimeError("更新事务身份或阶段不匹配")
        status["worker_pid"] = os.getpid()
        status["worker_identity"] = process_identity(os.getpid())
        probe_url = status.get("probe_url", "http://127.0.0.1:8766")
        previous = status["previous_revision"]
        target = status["target_revision"]
        home, current = install_home(root)
        descriptor = status.get("deployment") or current.get("deployment")
        old_prefix = Path(current["prefix"])
        log = root / "runtime_data/updates/build.log"
        stopped = False
        changed = False
        new_process = None
        try:
            require_clean(root)
            if source_info(root)["current_revision"] != previous:
                raise RuntimeError("源码在排队后已变化")
            phase(root, status, "fetching", "正在获取并验证远端版本")
            info = source_info(root)
            git(root, "fetch", "--no-tags", "--no-recurse-submodules", info["remote"], info["merge_ref"])
            if git(root, "rev-parse", "FETCH_HEAD") != target:
                raise RuntimeError("远端版本已改变，请重新检查更新")
            git(root, "merge-base", "--is-ancestor", previous, target)
            # 停止前完成可检查前提。新版本构建依赖可能不同；不允许后台sudo交互。

            opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
            with opener.open(probe_url + "/api/updates/readiness", timeout=5) as response:
                readiness = json.load(response)
            if readiness.get("ready") is not True:
                raise RuntimeError("停止前安全复核失败：" + str(readiness.get("reason", "状态未知")))
            phase(root, status, "stopping", "正在停止网关及其管理的运行组件")
            stopped = True
            stop(root, old_prefix, descriptor=descriptor)
            require_clean(root)
            git(root, "merge", "--ff-only", target)
            changed = True
            phase(root, status, "building", "正在构建与安装已选模块，期间服务暂时离线")
            # 独立解释器读取新版本安装器。旧代际留到新进程验证通过后再回收。
            code = """from pathlib import Path
from scripts.module_installer.manager import InstallRequest,apply,read_current
from scripts.module_installer.hardware import detect_hardware
import os, signal

def cancel_install(signum, frame):
    raise KeyboardInterrupt
signal.signal(signal.SIGTERM, cancel_install)
root=Path(os.environ['NEH_UPDATE_ROOT']);home=Path(os.environ['NUWAX_UPDATE_INSTALL_DIR'])
current=read_current(home);hardware=detect_hardware(root)
request=InstallRequest(root,home,tuple(current['modules']),current['robot'],min(2,hardware.build_jobs),python=current['python'],wheel_dir=home/'cache/device-wheels',repair=True,keep_previous=True,noninteractive=True)
apply(request,hardware)
"""
            os.environ["NEH_UPDATE_ROOT"] = str(root)
            command([current["python"], "-c", code], root, log)
            from scripts.module_installer.manager import read_current

            new = read_current(home)
            mark_generation(Path(new["prefix"]), target)
            phase(root, status, "starting", "构建完成，正在启动新版本")
            new_process = start(root, home / "current/start.sh", log, descriptor=descriptor)

            phase(root, status, "verifying", "正在核对网关实际运行版本")
            verify(probe_url, target, new_process)
            warning = ""
            try:
                cleanup_cache(home)
                prune_generations(root, home, {old_prefix, Path(new["prefix"])})
            except (OSError, RuntimeError, ValueError):
                warning = "更新成功，但部分构建缓存清理失败，请查看日志"
            # 保留一代恢复环境，不属于临时缓存；避免删去配置引用的旧文件。
            phase(
                root,
                status,
                "succeeded",
                warning or "更新完成，构建缓存已清理",
                cleanup_warning=warning,
                rollback_prefix=str(old_prefix),
            )
        except Exception as error:  # noqa: BLE001 - 事务失败必须进入恢复流程
            message = str(error)
            if stopped:
                phase(root, status, "rolling_back", "更新失败，正在恢复原版本")
                try:
                    if new_process is not None or descriptor is not None:
                        stop(root, (home / "current").resolve(), new_process, descriptor=descriptor)
                    if changed:
                        require_clean(root)
                        git(root, "reset", "--keep", previous)
                    temporary = home / (".rollback-" + request_id)
                    temporary.symlink_to(old_prefix.relative_to(home))
                    os.replace(temporary, home / "current")
                    fsync_directory(home)
                    recovery = start(root, old_prefix / "start.sh", log, descriptor=descriptor)

                    verify(probe_url, previous, recovery)
                    phase(root, status, "rolled_back", "更新失败，已恢复并验证原版本：" + message)
                except Exception:  # noqa: BLE001 - 恢复失败保留明确人工处理状态
                    phase(
                        root,
                        status,
                        "recovery_required",
                        "自动恢复未完成，请检查构建日志并从记录的源码/安装版本恢复",
                        failure=message,
                    )
            else:
                phase(root, status, "failed", message)
        finally:
            (root / "runtime_data/updates" / ("runner-" + request_id + ".py")).unlink(missing_ok=True)
'''

_OLD_COMBINED_STORE_SOURCE = r'''
from __future__ import annotations

import json

import os

import sqlite3

from contextlib import contextmanager

from dataclasses import dataclass, field

from datetime import datetime, timezone

from pathlib import Path

from typing import Any, Dict, Optional, Tuple

from uuid import uuid4

from core.errors import GatewayError

def _json(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False)

class AgentCallStore:
    def __init__(self, path: Path, *, gateway_instance_id: str, scope: Optional[str] = None,
                 robot_identity: Optional[Dict[str, Any]] = None, artifact_dir: Optional[Path] = None) -> None:
        if robot_identity is not None:
            if artifact_dir is None or scope is not None:
                raise ValueError("身份作用域需要 artifact_dir，且不能同时传入原始 scope。")
            scope_identity = self._identity_json(robot_identity, artifact_dir)
        else:
            if scope is None:
                raise ValueError("必须指定作用域或完整机器人身份。")
            scope_identity = None
        self.path = Path(path)
        self.scope = scope or ""
        self._registered_scope = scope_identity is not None
        self.gateway_instance_id = gateway_instance_id
        self._terminal_preparation_token = object()
        self.path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        fd = os.open(str(self.path), os.O_CREAT | os.O_RDWR, 0o600)
        os.close(fd)
        self.path.chmod(0o600)
        with self._connection() as connection:
            connection.executescript("""
                CREATE TABLE IF NOT EXISTS agent_calls (
                    request_id TEXT PRIMARY KEY, scope TEXT NOT NULL,
                    gateway_instance_id TEXT NOT NULL, tool_name TEXT NOT NULL,
                    capability_name TEXT NOT NULL, arguments_json TEXT NOT NULL,
                    fingerprint TEXT NOT NULL, delivery_state TEXT NOT NULL,
                    response_json TEXT, error_json TEXT, task_id TEXT,
                    created_at TEXT NOT NULL, updated_at TEXT NOT NULL
                );
                CREATE INDEX IF NOT EXISTS agent_calls_task ON agent_calls(scope, task_id);
                CREATE INDEX IF NOT EXISTS agent_calls_by_task ON agent_calls(task_id, created_at);
                CREATE TABLE IF NOT EXISTS agent_scope_owners (
                    scope_id TEXT PRIMARY KEY, identity_json TEXT NOT NULL UNIQUE
                );
                CREATE TABLE IF NOT EXISTS agent_scope_bindings (
                    scope TEXT PRIMARY KEY, owner_scope TEXT NOT NULL
                );
                CREATE INDEX IF NOT EXISTS agent_scope_bindings_owner ON agent_scope_bindings(owner_scope);
                CREATE TABLE IF NOT EXISTS agent_task_terminals (
                    task_id TEXT PRIMARY KEY, scope TEXT NOT NULL,
                    gateway_instance_id TEXT NOT NULL, snapshot_json TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                );
            """)
            # 旧行保留 NULL 来源/版本，只读兼容；绝不猜测历史任务的所属源。
            connection.execute("BEGIN IMMEDIATE")
            try:
                columns = {row["name"] for row in connection.execute("PRAGMA table_info(agent_task_terminals)")}
                for name, sql_type in (("receipt_source_id", "TEXT"), ("receipt_revision", "INTEGER")):
                    if name not in columns:
                        connection.execute(f"ALTER TABLE agent_task_terminals ADD COLUMN {name} {sql_type}")
                if scope_identity is not None:
                    self.scope = self._resolve_owner(connection, scope_identity)
                connection.commit()
            except BaseException:
                connection.rollback()
                raise

    @staticmethod
    def _identity_json(robot_identity: Dict[str, Any], artifact_dir: Path) -> str:
        # 与旧作用域的原始输入完全一致；不重新计算或验证任何历史派生标识。
        directory = str(Path(artifact_dir).expanduser().resolve())
        return json.dumps([robot_identity, directory], sort_keys=True)

    @classmethod
    def for_identity(cls, path: Path, *, robot_identity: Dict[str, Any], artifact_dir: Path,
                     gateway_instance_id: str):
        """按精确身份选择持久分配的作用域；旧作用域只能显式核对绑定。"""
        return cls(path, gateway_instance_id=gateway_instance_id,
                   robot_identity=robot_identity, artifact_dir=artifact_dir)

    @staticmethod
    def _resolve_owner(connection, identity_json: str) -> str:
        row = connection.execute("SELECT scope_id FROM agent_scope_owners WHERE identity_json=?",
                                 (identity_json,)).fetchone()
        if row is not None:
            return row["scope_id"]
        for _ in range(8):
            scope = "scope_" + uuid4().hex
            occupied = connection.execute(
                "SELECT scope_id FROM agent_scope_owners WHERE scope_id=? UNION ALL "
                "SELECT scope FROM agent_scope_bindings WHERE scope=? UNION ALL "
                "SELECT scope FROM agent_calls WHERE scope=? UNION ALL "
                "SELECT scope FROM agent_task_terminals WHERE scope=? LIMIT 1",
                (scope, scope, scope, scope)).fetchone()
            if occupied is not None:
                continue
            connection.execute("INSERT INTO agent_scope_owners(scope_id,identity_json) VALUES (?,?)",
                               (scope, identity_json))
            connection.execute("INSERT INTO agent_scope_bindings(scope,owner_scope) VALUES (?,?)", (scope, scope))
            return scope
        raise GatewayError("无法分配独立调用作用域；原调用记录保持不变。",
                           error_code="agent_scope_identity_conflict", http_status=409)

    def _assert_scope(self, connection, scope: str, *, error_code: str, message: str) -> None:
        if scope == self.scope:
            return
        if self._registered_scope:
            row = connection.execute("SELECT owner_scope FROM agent_scope_bindings WHERE scope=?", (scope,)).fetchone()
            if row is not None and row["owner_scope"] == self.scope:
                return
            if row is None:
                raise GatewayError("原记录的历史作用域尚未核实绑定；原请求不能被重新认领或重放。",
                                   error_code="agent_scope_binding_required", http_status=409)
        raise GatewayError(message, error_code=error_code, http_status=404 if error_code == "not_found" else 409)

    @contextmanager
    def _connection(self):
        # 每次操作独立连接并及时关闭，数据库事务绝不跨越机器人调用。
        connection = sqlite3.connect(str(self.path), timeout=5.0, isolation_level=None)
        connection.row_factory = sqlite3.Row
        connection.execute("PRAGMA synchronous=FULL")
        try:
            yield connection
        finally:
            connection.close()

    def begin(self, request_id: str, tool_name: str, capability_name: str,
              arguments: Dict[str, Any]) -> Tuple[Dict[str, Any], bool]:
        arguments_json = _json(arguments)
        now = datetime.now(timezone.utc).isoformat()
        with self._connection() as connection:
            connection.execute("BEGIN IMMEDIATE")
            try:
                row = connection.execute("SELECT * FROM agent_calls WHERE request_id=?", (request_id,)).fetchone()
                if row is not None:
                    self._assert_scope(connection, row["scope"], error_code="agent_request_conflict",
                                       message="请求标识已用于另一机器人，请核对原调用。")
                    if row["tool_name"] != tool_name or row["arguments_json"] != arguments_json:
                        raise GatewayError("请求标识已用于其他参数或机器人，请核对原调用。",
                                           error_code="agent_request_conflict", http_status=409)
                    connection.commit()
                    return self._decode(row), False
                # fingerprint 仅为旧表结构保留；不再计算、读取比较或改写历史值。
                connection.execute(
                    "INSERT INTO agent_calls (request_id,scope,gateway_instance_id,tool_name,capability_name,"
                    "arguments_json,fingerprint,delivery_state,response_json,error_json,task_id,created_at,updated_at) "
                    "VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?)",
                    (request_id, self.scope, self.gateway_instance_id, tool_name, capability_name,
                     arguments_json, "", "pending", None, None, None, now, now),
                )
                row = connection.execute("SELECT * FROM agent_calls WHERE request_id=?", (request_id,)).fetchone()
                connection.commit()
                return self._decode(row), True
            except BaseException:
                connection.rollback()
                raise

    def finish(self, request_id: str, response: Any, error: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        response_json = _json(response)
        task_id = response.get("task_id") if isinstance(response, dict) else None
        with self._connection() as connection:
            cursor = connection.execute(
                "UPDATE agent_calls SET response_json=?,error_json=?,task_id=?,delivery_state='recorded',updated_at=? "
                "WHERE request_id=? AND scope=? AND gateway_instance_id=? AND delivery_state='pending'",
                (response_json, _json(error) if error else None, task_id,
                 datetime.now(timezone.utc).isoformat(), request_id, self.scope, self.gateway_instance_id),
            )
            if cursor.rowcount != 1:
                raise GatewayError("调用记录的拥有方或状态已改变，执行结果需核对。",
                                   error_code="agent_call_outcome_unknown", http_status=409)
        return self.get(request_id)

    def mark_unknown(self, request_id: str) -> None:
        with self._connection() as connection:
            connection.execute(
                "UPDATE agent_calls SET delivery_state='unknown',updated_at=? "
                "WHERE request_id=? AND scope=? AND gateway_instance_id=? AND delivery_state='pending'",
                (datetime.now(timezone.utc).isoformat(), request_id, self.scope, self.gateway_instance_id),
            )

    def get(self, request_id: str) -> Dict[str, Any]:
        with self._connection() as connection:
            row = connection.execute("SELECT * FROM agent_calls WHERE request_id=?", (request_id,)).fetchone()
            if row is not None:
                self._assert_scope(connection, row["scope"], error_code="not_found", message="当前机器人没有该调用记录。")
        if row is None:
            raise GatewayError("当前机器人没有该调用记录。", error_code="not_found", http_status=404)
        return self._decode(row)

    def get_terminal(self, task_id: str) -> Dict[str, Any]:
        with self._connection() as connection:
            row = connection.execute(
                "SELECT * FROM agent_task_terminals WHERE task_id=?", (task_id,)
            ).fetchone()
            if row is not None:
                self._assert_scope(connection, row["scope"], error_code="not_found", message="当前机器人没有该任务终态记录。")
        if row is None:
            raise GatewayError("没有已保存的任务终态记录。", error_code="not_found", http_status=404)
        return {"gateway_instance_id": row["gateway_instance_id"],
                "snapshot": json.loads(row["snapshot_json"]), "recorded_at": row["updated_at"]}

    def _decode(self, row) -> Dict[str, Any]:
        record = dict(row)
        response_json = record.pop("response_json")
        error_json = record.pop("error_json")
        record["response"] = json.loads(response_json) if response_json else None
        record["error"] = json.loads(error_json) if error_json else None
        record.pop("arguments_json", None)
        record.pop("fingerprint", None)
        # 旧实例的未完成意图只能恢复查看，不能由新实例自动认领执行。
        if record["delivery_state"] == "pending" and record["gateway_instance_id"] != self.gateway_instance_id:
            record["delivery_state"] = "unknown"
        return record

'''

_OLD_MANIFEST_READER_SOURCE = r'''
def read_current(home: Path) -> dict:
    current = home / "current"
    if not current.is_symlink():
        if current.exists():
            raise InstallError("安装入口 current 不是受管链接，拒绝覆盖。")
        return {}
    prefix = current.resolve()
    if prefix.parent != (home / "generations").resolve():
        raise InstallError("安装状态指向受管目录以外，拒绝继续。")
    try:
        state = json.loads((prefix / "manifest.json").read_text(encoding="utf-8"))
        if not isinstance(state, dict):
            raise TypeError("安装记录必须是对象")
        if state.get("schema") != 1 or state.get("complete") is not True:
            raise ValueError("未完成的安装记录")
        if tuple(state["modules"]) != resolve_modules(state["modules"]):
            raise ValueError("模块依赖记录不完整")
        if state["robot"] not in ROBOT_REQUIREMENTS:
            raise ValueError("未知机器人装配")
    except (OSError, ValueError, KeyError, TypeError) as exc:
        raise InstallError(f"安装状态损坏：{exc}；请检查 {prefix}") from exc
    state["prefix"] = str(prefix)
    return state
'''

def frozen_module(name, source, **bindings):
    """Compile literal frozen code, never read a moving source file as old code."""
    module = types.ModuleType(name)
    module.__dict__.update(bindings)
    # dataclass inspects the defining module while compiling the old fixture.
    sys.modules[name] = module
    exec(compile(source, "<frozen-" + name + ">", "exec"), module.__dict__)  # noqa: S102 - 固定旧实现字面量，仅用于隔离回滚测试。
    return module


def old_store_type():
    return frozen_module("neh_rollout_old_store", _OLD_COMBINED_STORE_SOURCE).AgentCallStore


def old_manifest_reader():
    return frozen_module("neh_rollout_old_manifest", _OLD_MANIFEST_READER_SOURCE,
                         Path=Path, json=json, InstallError=manager.InstallError,
                         resolve_modules=manager.resolve_modules,
                         ROBOT_REQUIREMENTS=manager.ROBOT_REQUIREMENTS).read_current


def service_fixture(artifacts, actions):
    """Real service and SQLite facade; no device, task thread, or network owner."""
    descriptor = NS(name="synthetic_action", exposed_to_agent=True)
    view = NS(descriptor=descriptor, supported=True, runnable=True,
              capability_name="fixture_action", input_schema={"type": "object"})
    def invoke(*args, **kwargs):
        actions.append((args, kwargs))
        return dict(RESPONSE)
    fixture = NS(
        robot=NS(manifest=NS(**{key: IDENTITY[key] for key in ("robot_model", "robot_name", "entrypoint")}),
                 defaults=NS(parameters={"robot_id": IDENTITY["robot_id"]})),
        config=NS(gateway=NS(artifact_dir=artifacts)),
        task_manager=None,
        event_bus=NS(subscribe=lambda *a, **kw: "fixture-subscription", unsubscribe=lambda *a: None),
        capability_registry=NS(get_descriptor=lambda name: descriptor),
        list_tools=lambda **kw: [view], invoke_tool=invoke,
    )
    return AgentToolService(fixture, NS(authorize_capability=lambda descriptor: None))


@contextlib.contextmanager
def offline_builds():
    """Keep actual apply, locks, read_current, write_runtime, manifest and switch."""
    fail = AssertionError("Unexpected external build/process/network operation in offline fixture")
    with contextlib.ExitStack() as stack:
        for name in ("run", "Popen"):
            stack.enter_context(patch.object(subprocess, name, side_effect=fail))
        stack.enter_context(patch.object(socket, "create_connection", side_effect=fail))
        stack.enter_context(patch.object(manager.CommandRunner, "__call__", side_effect=fail))
        stack.enter_context(patch.object(manager, "check_prerequisites", return_value=sys.executable))
        # Rollout policy does not depend on local file metadata enumeration.
        stack.enter_context(patch.object(manager, "installation_inputs", return_value={"version": 1, "fixture": True}))
        stack.enter_context(patch.object(lifecycle, "service_identifier", return_value="neh-fixture.service"))
        for name in ("_system_dependencies", "_python_environment", "_prepare_models", "_verify_prefix", "prune_build_cache"):
            stack.enter_context(patch.object(manager, name))
        stack.enter_context(patch.object(manager, "build_modules", return_value={}))
        stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
        yield


class RolloutFixture(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="neh-terminal-rollout-",
                                               dir=os.environ.get("NEH_TERMINAL_ROLLOUT_TMP"))
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.root = self.base / "source tree"
        self.home = self.base / "install home"
        self.root.mkdir()
        self.artifacts = self.root / "artifacts"
        self.path = self.root / "agent_calls.sqlite3"
        self.services = []
        self.addCleanup(self.close_services)
        self.install_safe_launcher_target()

    def close_services(self):
        for service in self.services:
            self.assertTrue(service.close(1, source_fenced=True)["owner_closed"])

    def install_safe_launcher_target(self):
        """Use the production shell/parser, but replace the gateway target entirely."""
        for relative in ("scripts/lib/config_env.sh", "core/config_values.py", "core/config_catalog.json"):
            target = self.root / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes((ROOT / relative).read_bytes())
        # The parser validates adapter paths without importing or running adapters.
        for adapter in manager.ROBOT_REQUIREMENTS:
            directory = self.root / "drivers" / adapter
            directory.mkdir(parents=True)
            for name in ("manifest.py", "assembly.py"):
                (directory / name).write_text("# inert fixture; never imported\n")
        (self.root / "scripts/start_gateway.sh").write_text(
            '#!/usr/bin/env bash\nset -euo pipefail\n'
            'export FIXTURE_BEFORE_KEYS="${_NUWAX_CONFIG_FILE_KEYS:-}"\n'
            'source "$(dirname "$0")/lib/config_env.sh"\n'
            'load_platform_config\nload_platform_config\n'
            + shlex.quote(sys.executable) + " -c 'import json, os; print(json.dumps(dict(os.environ)))'\n")

    def service(self, policy=MISSING, actions=None):
        environment = dict(os.environ)
        environment.pop(POLICY_ENV, None)
        if policy is not MISSING:
            environment[POLICY_ENV] = policy
        with patch.dict(os.environ, environment, clear=True):
            value = service_fixture(self.artifacts, [] if actions is None else actions)
        self.services.append(value)
        return value

    def previous(self, capability=MISSING, *, adapter="customrobot", deployment=True):
        prefix = self.home / "generations/old"
        prefix.mkdir(parents=True)
        state = {"schema": 1, "complete": True, "modules": ["core"], "robot": adapter,
                 "source_root": str(self.root), "python": sys.executable, "fingerprint": "old-fixture"}
        if capability is not MISSING:
            state["agent_terminal_storage"] = capability
        (prefix / "settings.env").write_text(POLICY_ENV + "=split\nNUWAX_GATEWAY_PORT=8766\n")
        (prefix / "start.sh").write_text("#!/bin/sh\n# Never executed by this fixture\n")
        if deployment:
            state["deployment"] = lifecycle.deployment_descriptor(
                self.root, self.home, prefix, ["core"], adapter, service_name="neh-fixture.service")
        (prefix / "manifest.json").write_text(json.dumps(state))
        (self.home / "current").symlink_to("generations/old")
        return manager.read_current(self.home)

    def apply(self, *, adapter="customrobot"):
        request = manager.InstallRequest(self.root, self.home, ("core",), adapter, 1,
                                         repair=True, keep_previous=True, noninteractive=True)
        with offline_builds(), patch.dict(os.environ, {POLICY_ENV: "split"}):
            return manager.apply(request, NS(build_jobs=1))

    def launcher_environment(self, prefix, *, stale="split", marker=None):
        """Execute only the self-created launcher and inert config-loader target."""
        environment = {"PATH": str(Path(sys.executable).parent) + os.pathsep + os.defpath,
                       "HOME": str(self.base), "PYTHONDONTWRITEBYTECODE": "1",
                       POLICY_ENV: stale, "FIXTURE_OTHER": "stale-other",
                       "_NUWAX_CONFIG_FILE_KEYS": marker or "FIXTURE_OTHER:" + POLICY_ENV + ":"}
        result = subprocess.run(["bash", str(prefix / "start.sh")], env=environment,
                                cwd=self.root, text=True, capture_output=True, timeout=15, check=True)
        self.assertEqual(result.stderr, "")
        return json.loads(result.stdout)

    def assert_combined(self):
        layout = AgentCallStore.terminal_storage_status(self.path)
        self.assertEqual(layout["layout_state"], "combined")
        self.assertFalse(layout["legacy_sealed"])
        self.assertIsNone(layout["pair_id"])
        self.assertEqual(Path(layout["terminal_path"]), self.path)
        self.assertFalse(self.path.with_name("agent_calls.terminals.sqlite3").exists())
        with sqlite3.connect(self.path) as db:
            self.assertEqual(db.execute("SELECT count(*) FROM sqlite_master WHERE type='trigger'").fetchone()[0], 0)
        return layout


class CreationPolicyTests(RolloutFixture):
    def test_low_level_default_and_identity_factory_remain_split(self):
        for constructor in (lambda path: AgentCallStore(path, scope="fixture", gateway_instance_id="low-level"),
                            lambda path: AgentCallStore.for_identity(path, robot_identity=IDENTITY,
                                artifact_dir=self.artifacts, gateway_instance_id="identity")):
            path = self.base / ("direct-" + str(len(list(self.base.glob("direct-*.sqlite3")))) + ".sqlite3")
            constructor(path)
            self.assertEqual(AgentCallStore.terminal_storage_status(path)["layout_state"], "active")

    def test_explicit_combined_factory_is_unsealed_and_old_reader_readable(self):
        store = AgentCallStore.for_identity(self.path, robot_identity=IDENTITY, artifact_dir=self.artifacts,
                                            gateway_instance_id="candidate", fresh_layout="combined")
        store.begin("request", "synthetic_action", "fixture_action", {})
        store.finish("request", RESPONSE)
        self.assert_combined()
        old = old_store_type().for_identity(self.path, robot_identity=IDENTITY, artifact_dir=self.artifacts,
                                            gateway_instance_id="rollback")
        self.assertEqual(old.get("request")["response"], RESPONSE)
        self.assertFalse(old.begin("request", "synthetic_action", "fixture_action", {})[1])

    def test_unmanaged_service_absent_or_unrecognized_policy_is_combined(self):
        for index, value in enumerate((MISSING, "", "combined", "SPLIT", " split ", "1", "future")):
            with self.subTest(policy=str(value)):
                self.artifacts = self.root / str(index) / "artifacts"
                self.path = self.artifacts.parent / "agent_calls.sqlite3"
                self.service(value)
                self.assert_combined()

    def test_explicit_service_split_and_existing_active_under_combined(self):
        actions = []
        first = self.service("split", actions)
        first.invoke("request", "synthetic_action", {})
        first.store.save_terminal("synthetic-task", SNAPSHOT, source_id="fixture-source", revision=1)
        before = AgentCallStore.terminal_storage_status(self.path)
        reopened = self.service("combined", actions)
        self.assertEqual(reopened.store.get_terminal("synthetic-task")["snapshot"], SNAPSHOT)
        reopened.invoke("request", "synthetic_action", {})
        self.assertEqual(len(actions), 1)
        self.assertEqual(AgentCallStore.terminal_storage_status(self.path), before)
        self.assertEqual(before["layout_state"], "active")

    def test_existing_combined_is_never_automatically_migrated(self):
        old = old_store_type().for_identity(self.path, robot_identity=IDENTITY, artifact_dir=self.artifacts,
                                            gateway_instance_id="old")
        old.begin("request", "synthetic_action", "fixture_action", {})
        old.finish("request", RESPONSE)
        new = self.service("split")
        self.assertEqual(new.store.get("request")["response"], RESPONSE)
        self.assert_combined()

    def test_fresh_managed_install_advertises_reader_and_selects_split(self):
        result = self.apply()
        self.assertEqual(result["agent_terminal_storage"], {"reader_version": 1, "fresh_layout": "split"})
        environment = self.launcher_environment(Path(result["prefix"]), stale="combined")
        service = self.service(environment[POLICY_ENV])
        self.assertEqual(AgentCallStore.terminal_storage_status(service.store.path)["layout_state"], "active")

    def test_compatible_prior_reader_selects_split_independent_of_prior_creation_choice(self):
        self.previous({"reader_version": 1, "fresh_layout": "combined"})
        result = self.apply()
        self.assertEqual(result["agent_terminal_storage"], {"reader_version": 1, "fresh_layout": "split"})
        self.assertEqual(self.launcher_environment(Path(result["prefix"]), stale="combined")[POLICY_ENV], "split")

    def test_invalid_reader_capabilities_are_conservative(self):
        values = (MISSING, None, {}, [], "split", 1, True,
                  {"reader_version": True}, {"reader_version": 1.0}, {"reader_version": "1"},
                  {"reader_version": 0}, {"reader_version": 2}, {"reader_version": None},
                  {"fresh_layout": "split"})
        for capability in values:
            with self.subTest(capability=str(capability)):
                current = {"prefix": "existing"}
                if capability is not MISSING:
                    current["agent_terminal_storage"] = capability
                self.assertEqual(manager._agent_terminal_storage_policy(current),
                                 {"reader_version": 1, "fresh_layout": "combined"})

    def test_old_manifest_reader_tolerates_new_top_level_capability(self):
        self.previous({"reader_version": 1, "fresh_layout": "combined"})
        self.assertEqual(old_manifest_reader()(self.home), manager.read_current(self.home))

    def test_unknown_prior_missing_descriptor_remains_adapter_neutral(self):
        self.previous(deployment=False, adapter="g1")
        result = self.apply(adapter="g1")
        self.assertEqual(result["robot"], "g1")
        self.assertNotIn("deployment", result)
        self.assertEqual(lifecycle.inspect_installation(self.home)["robot_adapter"], "g1")
        self.assertEqual(result["agent_terminal_storage"]["fresh_layout"], "combined")

    def test_real_launcher_and_common_loader_resist_stale_environment_and_profile(self):
        self.previous()
        profile = self.root / "drivers/customrobot/configs/默认.env"
        profile.parent.mkdir(parents=True)
        profile.write_text(POLICY_ENV + "=split\nFIXTURE_OTHER=current-other\n")
        for selected in ("combined", "split"):
            with self.subTest(selected=selected):
                prefix = self.home / "generations" / ("render-" + selected)
                opposite = "combined" if selected == "split" else "split"
                profile.write_text(POLICY_ENV + "=" + opposite + "\nFIXTURE_OTHER=current-other\n")
                runtime.write_runtime(self.root, prefix, ("core",), "customrobot",
                                      {POLICY_ENV: opposite}, fresh_layout=selected)
                self.assertEqual(runtime.read_settings(prefix / "settings.env")[POLICY_ENV], selected)
                marker = "FIXTURE_OTHER:" + POLICY_ENV + ":UNRELATED_KEY:" + POLICY_ENV + ":"
                actual = self.launcher_environment(prefix, stale=opposite, marker=marker)
                self.assertEqual(actual[POLICY_ENV], selected)
                self.assertNotIn(POLICY_ENV, actual["FIXTURE_BEFORE_KEYS"].split(":"))
                self.assertIn("FIXTURE_OTHER", actual["FIXTURE_BEFORE_KEYS"].split(":"))
                self.assertIn("UNRELATED_KEY", actual["FIXTURE_BEFORE_KEYS"].split(":"))
                self.assertEqual(actual["FIXTURE_OTHER"], "current-other")
                self.assertNotIn(POLICY_ENV, actual["_NUWAX_CONFIG_FILE_KEYS"].split(":"))

    def test_broken_existing_installation_is_rejected_instead_of_treated_as_fresh(self):
        old = self.previous()
        prefix = Path(old["prefix"])
        manifest = prefix / "manifest.json"
        valid = manifest.read_text()
        for corrupt in (None, "not-json", "[]", '{"schema":1,"complete":false}'):
            with self.subTest(manifest=corrupt):
                if corrupt is None:
                    manifest.unlink()
                else:
                    manifest.write_text(corrupt)
                with offline_builds(), patch.object(manager, "write_runtime") as renderer:
                    with self.assertRaises(manager.InstallError):
                        manager.apply(manager.InstallRequest(self.root, self.home, ("core",), "customrobot", 1,
                                                             repair=True, keep_previous=True), NS(build_jobs=1))
                    renderer.assert_not_called()
                self.assertEqual((self.home / "current").resolve(), prefix)
                self.assertFalse(self.path.exists())
                manifest.write_text(valid)
        (self.home / "current").unlink()
        (self.home / "current").symlink_to("generations/missing")
        with self.assertRaises(manager.InstallError):
            self.apply()
        self.assertFalse(self.path.exists())
        self.assertEqual(sorted(path.name for path in (self.home / "generations").iterdir()), ["old"])

    def test_real_apply_selects_policy_under_existing_installation_lock(self):
        self.previous()
        original = manager._agent_terminal_storage_policy
        selected = []
        def under_lock(current):
            with self.assertRaises(manager.InstallError), manager.installation_lock(self.home):
                pass
            with self.assertRaises(manager.InstallError), manager.idle_gateway(self.root):
                pass
            self.assertEqual(Path(current["prefix"]), self.home / "generations/old")
            result = original(current)
            selected.append(result)
            return result
        renderer = manager.write_runtime
        def render(*args, **kwargs):
            self.assertEqual(len(selected), 1)
            self.assertEqual(kwargs["fresh_layout"], selected[0]["fresh_layout"])
            return renderer(*args, **kwargs)
        with patch.object(manager, "_agent_terminal_storage_policy", under_lock), \
                patch.object(manager, "write_runtime", render):
            result = self.apply()
        self.assertEqual(result["agent_terminal_storage"], selected[0])

    def test_runtime_old_call_signature_defaults_to_explicit_combined(self):
        prefix = self.home / "generations/default"
        runtime.write_runtime(self.root, prefix, ("core",), "customrobot", {POLICY_ENV: "split"})
        self.assertEqual(self.launcher_environment(prefix)[POLICY_ENV], "combined")


class OldPinnedRollbackTests(RolloutFixture):
    def rollback_case(self, *, adapter, descriptor):
        previous = self.previous(adapter=adapter, deployment=descriptor)
        old_prefix = Path(previous["prefix"])
        frozen = frozen_module("neh_rollout_old_updater", _OLD_UPDATER_SOURCE)
        old_revision, target_revision = "1" * 40, "2" * 40  # Opaque fixture labels, never computed digests.
        revision = [old_revision]
        request_id = "offline-rollout-transaction"
        status = {"state": "queued", "request_id": request_id,
                  "previous_revision": old_revision, "target_revision": target_revision,
                  "created_at": frozen.time.time()}
        if descriptor:
            status["deployment"] = previous["deployment"]
        frozen.write_json(self.root / "runtime_data/updates/state.json", status)
        actions, events, candidates, before_rows = [], [], [], []
        readiness = MagicMock()
        readiness.open.return_value.__enter__.return_value = io.StringIO('{"ready":true}')

        def git(root, *argv):
            self.assertEqual(root, self.root)
            if argv[0] in ("fetch", "merge-base", "status"):
                return ""
            if argv == ("rev-parse", "FETCH_HEAD"):
                return target_revision
            if argv == ("merge", "--ff-only", target_revision):
                revision[0] = target_revision
                return ""
            if argv == ("reset", "--keep", old_revision):
                revision[0] = old_revision
                return ""
            raise AssertionError("Unexpected old-coordinator Git command: " + repr(argv))

        def build(argv, root, log):
            self.assertEqual(argv[:2], [sys.executable, "-c"])
            self.assertEqual(root, self.root)
            self.assertEqual(revision[0], target_revision)
            events.append("incoming_apply")
            # Execute the exact OLD coordinator's installer command. Its imports
            # resolve to the actual incoming manager; only external work is mocked.
            with offline_builds(), patch.object(hardware, "detect_hardware", return_value=NS(build_jobs=1)), \
                    patch("signal.signal"):
                exec(compile(argv[2], "<old-inline-incoming-installer>", "exec"), {"__name__": "fixture_installer"})  # noqa: S102 - 固定旧协调器生成的安装片段；外部操作已隔离。
            candidate = manager.read_current(self.home)
            self.assertEqual(candidate["robot"], adapter)
            self.assertEqual(candidate["agent_terminal_storage"], {"reader_version": 1, "fresh_layout": "combined"})
            # The frozen previous reader accepts the additional capability field.
            self.assertEqual(old_manifest_reader()(self.home), candidate)
            candidates.append(Path(candidate["prefix"]))

        def stop(root, prefix, *args, **kwargs):
            events.append("stop_old" if Path(prefix) == old_prefix else "stop_candidate")

        def start(root, launcher, log, **kwargs):
            selected = (self.home / "current").resolve()
            if selected == old_prefix:
                events.append("start_rollback")
                self.assertEqual(revision[0], old_revision)
                self.assertEqual(len(actions), 1)
                old = old_store_type().for_identity(self.path, robot_identity=IDENTITY,
                                                    artifact_dir=self.artifacts, gateway_instance_id="rolled-back-owner")
                self.assertEqual(old.get("request")["response"], RESPONSE)
                self.assertEqual(old.get_terminal("synthetic-task")["snapshot"], SNAPSHOT)
                for request in ("request", "pending", "unknown"):
                    record, acquired = old.begin(request, "synthetic_action", "fixture_action", {})
                    self.assertFalse(acquired, "Rollback must never re-acquire a previous intent")
                    self.assertEqual(record["delivery_state"], "recorded" if request == "request" else "unknown")
                with sqlite3.connect(self.path) as db:
                    self.assertEqual(db.execute("SELECT * FROM agent_calls ORDER BY request_id").fetchall(), before_rows)
                self.assert_combined()
            else:
                events.append("start_candidate")
                self.assertEqual(revision[0], target_revision)
                self.assertEqual(selected, candidates[0])
                # This executes only the inert, self-created launcher target and
                # real common config loader, never the repository gateway script.
                launched = self.launcher_environment(selected)
                self.assertEqual(launched[POLICY_ENV], "combined")
                service = self.service(launched[POLICY_ENV], actions)
                service.invoke("request", "synthetic_action", {})
                self.assertEqual(service.store.get("request")["delivery_state"], "recorded")
                service.store.save_terminal("synthetic-task", SNAPSHOT, source_id="fixture-source", revision=1)
                service.store.begin("pending", "synthetic_action", "fixture_action", {})
                service.store.begin("unknown", "synthetic_action", "fixture_action", {})
                service.store.mark_unknown("unknown")
                with sqlite3.connect(self.path) as db:
                    before_rows.extend(db.execute("SELECT * FROM agent_calls ORDER BY request_id").fetchall())
                self.assert_combined()
                self.assertEqual(len(actions), 1)
            return NS()  # A fixture lifecycle handle; no actual gateway process.

        def verify(url, expected, process):
            events.append("verify_candidate" if expected == target_revision else "verify_rollback")
            if expected == target_revision:
                self.assertEqual(len(actions), 1)
                raise RuntimeError("forced failure after durable finish")
            self.assertEqual(expected, old_revision)
            self.assertEqual((self.home / "current").resolve(), old_prefix)

        operations = {"git": git, "source_info": lambda root: {"current_revision": revision[0],
                       "remote": "fixture", "merge_ref": "refs/heads/fixture"},
                      "install_home": lambda root: (self.home, previous), "command": build,
                      "start": start, "stop": stop, "verify": verify,
                      "process_identity": lambda pid: {"boot_id": "fixture", "start_ticks": "fixture"},
                      "cleanup_cache": lambda *a: self.fail("Successful-update cleanup reached during forced failure"),
                      "prune_generations": lambda *a: self.fail("Successful-update pruning reached during forced failure")}
        with contextlib.ExitStack() as stack:
            for name, value in operations.items():
                stack.enter_context(patch.object(frozen, name, value, create=True))
            stack.enter_context(patch.object(frozen.urllib.request, "build_opener", return_value=readiness))
            stack.enter_context(patch.dict(os.environ, {"NUWAX_UPDATE_INSTALL_DIR": str(self.home), POLICY_ENV: "split"}))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            frozen.execute(self.root, request_id)
        result = frozen.state(self.root)
        self.assertEqual(result["state"], "rolled_back", result)
        self.assertIn("forced failure after durable finish", result["message"])
        self.assertEqual((self.home / "current").resolve(), old_prefix)
        self.assertEqual(revision[0], old_revision)
        self.assertEqual(events, ["stop_old", "incoming_apply", "start_candidate", "verify_candidate",
                                  "stop_candidate", "start_rollback", "verify_rollback"])
        self.assertEqual(len(actions), 1)
        self.assertTrue(candidates[0].is_dir())
        self.assert_combined()

    def test_old_pinned_update_rolls_back_after_durable_action_without_replay(self):
        self.rollback_case(adapter="customrobot", descriptor=True)

    def test_old_pinned_update_handles_unknown_descriptor_and_generic_adapter(self):
        self.rollback_case(adapter="g1", descriptor=False)

    def test_abrupt_fixture_exit_preserves_pending_unknown_and_old_readability(self):
        result = subprocess.run([sys.executable, str(Path(__file__).resolve()), "--fixture-exit", str(self.root)],
                                env={"PATH": os.defpath, "PYTHONDONTWRITEBYTECODE": "1"},
                                capture_output=True, text=True, timeout=15, check=False)
        self.assertEqual(result.returncode, EXIT_CODE, result.stderr)
        old = old_store_type().for_identity(self.path, robot_identity=IDENTITY,
                                            artifact_dir=self.artifacts, gateway_instance_id="after-exit")
        with sqlite3.connect(self.path) as db:
            before = db.execute("SELECT request_id,delivery_state,gateway_instance_id FROM agent_calls ORDER BY request_id").fetchall()
        self.assertEqual([(row[0], row[1]) for row in before], [("pending", "pending"), ("unknown", "unknown")])
        for request in ("pending", "unknown"):
            row, acquired = old.begin(request, "synthetic_action", "fixture_action", {})
            self.assertFalse(acquired)
            self.assertEqual(row["delivery_state"], "unknown")
        with sqlite3.connect(self.path) as db:
            self.assertEqual(db.execute("SELECT request_id,delivery_state,gateway_instance_id FROM agent_calls ORDER BY request_id").fetchall(), before)
        self.assert_combined()


def abrupt_exit_fixture(root):
    # Deliberately narrow child mode; rejects non-fixture paths before any write.
    root = Path(root).resolve()
    if root.name != "source tree" or not root.parent.name.startswith("neh-terminal-rollout-") or not root.is_dir():
        raise ValueError("Abrupt-exit fixture requires its own temporary source tree")
    with patch.dict(os.environ, {POLICY_ENV: "combined"}):
        service = service_fixture(root / "artifacts", [])
    service.store.begin("pending", "synthetic_action", "fixture_action", {})
    service.store.begin("unknown", "synthetic_action", "fixture_action", {})
    service.store.mark_unknown("unknown")
    os._exit(EXIT_CODE)


if __name__ == "__main__":
    if len(sys.argv) == 3 and sys.argv[1] == "--fixture-exit":
        abrupt_exit_fixture(sys.argv[2])
    else:
        unittest.main(verbosity=2)
