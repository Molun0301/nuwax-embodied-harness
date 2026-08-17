#!/usr/bin/env python3
"""Geometry 24 组合可部署性矩阵 runner。

默认仅 dry-run，不执行 systemd、MCP（模型上下文协议）或实机命令。Python 3.8 兼容。
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import shlex
import subprocess
import sys
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Mapping, Optional, Sequence

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from core.config_io import (  # noqa: E402
    _geometry_acceptance_ledger_path,
    _geometry_owner_env_path,
    geometry_backend_statuses,
)

OWNER_KEYS = {
    "NUWAX_GEOMETRY_LIO_BACKEND": "lio",
    "NUWAX_GEOMETRY_MAPPING_BACKEND": "mapping",
    "NUWAX_GEOMETRY_CORRECTION_BACKEND": "correction",
}
DEFAULT_SERVICES = ("nuwax-robot-bridge.service", "nuwax-geometry-runtime.service")


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _run(command: Sequence[str], timeout_sec: float, *, check: bool = True) -> Dict[str, object]:
    started = time.monotonic()
    try:
        completed = subprocess.run(
            list(command),
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout_sec,
        )
        result = {
            "command": list(command),
            "returncode": completed.returncode,
            "stdout": completed.stdout,
            "stderr": completed.stderr,
            "duration_sec": round(time.monotonic() - started, 3),
        }
    except subprocess.TimeoutExpired as exc:
        result = {
            "command": list(command),
            "returncode": 124,
            "stdout": exc.stdout or "",
            "stderr": exc.stderr or "",
            "duration_sec": round(time.monotonic() - started, 3),
            "timeout": True,
        }
    if check and int(result["returncode"]) != 0:
        raise RuntimeError(json.dumps(result, ensure_ascii=False))
    return result


def _read_env(path: Path) -> Dict[str, str]:
    values: Dict[str, str] = {}
    if not path.exists():
        return values
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        values[key.strip()] = value.strip()
    return values


def _atomic_write_env(path: Path, values: Mapping[str, str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    existing = path.stat() if path.exists() else None
    mode = existing.st_mode & 0o777 if existing else 0o600
    uid = existing.st_uid if existing else os.getuid()
    gid = existing.st_gid if existing else os.getgid()
    fd, temporary = tempfile.mkstemp(prefix=".%s." % path.name, dir=str(path.parent))
    try:
        os.fchmod(fd, mode)
        os.fchown(fd, uid, gid)
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            for key in sorted(values):
                handle.write("%s=%s\n" % (key, values[key]))
            handle.flush()
            os.fsync(handle.fileno())
        os.chmod(temporary, mode)
        os.replace(temporary, str(path))
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _atomic_write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    existing = path.stat() if path.exists() else None
    mode = existing.st_mode & 0o777 if existing else 0o600
    uid = existing.st_uid if existing else os.getuid()
    gid = existing.st_gid if existing else os.getgid()
    fd, temporary = tempfile.mkstemp(prefix=".%s." % path.name, dir=str(path.parent))
    try:
        os.fchmod(fd, mode)
        os.fchown(fd, uid, gid)
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, ensure_ascii=False, sort_keys=True, indent=2)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.chmod(temporary, mode)
        os.replace(temporary, str(path))
        directory_fd = os.open(str(path.parent), os.O_RDONLY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _restore_file(
    path: Path,
    existed: bool,
    content: bytes,
    mode: int,
    uid: int,
    gid: int,
) -> None:
    if not existed:
        try:
            path.unlink()
        except FileNotFoundError:
            pass
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=".%s.restore." % path.name, dir=str(path.parent))
    try:
        os.fchmod(fd, mode)
        os.fchown(fd, uid, gid)
        with os.fdopen(fd, "wb") as handle:
            handle.write(content)
            handle.flush()
            os.fsync(handle.fileno())
        os.chmod(temporary, mode)
        os.replace(temporary, str(path))
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _save_combination(status: object) -> Dict[str, str]:
    path = _geometry_owner_env_path()
    values = _read_env(path)
    for key, attribute in OWNER_KEYS.items():
        values[key] = str(getattr(status, attribute))
    _atomic_write_env(path, values)
    return {"path": str(path), "values": {key: values[key] for key in OWNER_KEYS}}


def _render_command(template: str, status: object) -> List[str]:
    rendered = template.format(
        lio=getattr(status, "lio"),
        mapping=getattr(status, "mapping"),
        correction=getattr(status, "correction"),
    )
    return shlex.split(rendered)


def _systemd_snapshot(services: Sequence[str], timeout_sec: float) -> List[Dict[str, object]]:
    snapshots = []
    for service in services:
        result = _run(
            [
                "systemctl", "show", service,
                "--property=ActiveState,SubState,MainPID,NRestarts,Result,ExecMainStatus",
            ],
            timeout_sec,
            check=False,
        )
        properties = {}
        for line in str(result["stdout"]).splitlines():
            if "=" in line:
                key, value = line.split("=", 1)
                properties[key] = value
        snapshots.append({"service": service, "properties": properties, "command": result})
    return snapshots


def _journal_snapshot(services: Sequence[str], timeout_sec: float) -> List[Dict[str, object]]:
    return [
        _run(["journalctl", "--unit", service, "--lines", "200", "--no-pager"], timeout_sec, check=False)
        for service in services
    ]


def _release_generation() -> str:
    try:
        from services.geometry.release_manifest import verify_release_manifest
        proof = verify_release_manifest()
    except Exception:
        return ""
    try:
        return proof.generation
    finally:
        proof.close()


def _record_acceptance(status: object, archive: Path) -> None:
    path = _geometry_acceptance_ledger_path()
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError):
        payload = {"schema_version": 1, "records": []}
    records = payload.get("records")
    if not isinstance(records, list):
        records = []
    key = (getattr(status, "lio"), getattr(status, "mapping"), getattr(status, "correction"))
    records = [
        item for item in records
        if not isinstance(item, dict)
        or (item.get("lio"), item.get("mapping"), item.get("correction")) != key
    ]
    archive_sha256 = hashlib.sha256(archive.read_bytes()).hexdigest()
    records.append({
        "lio": key[0], "mapping": key[1], "correction": key[2],
        "passed": True, "completed_at": _utc_now(),
        "release_generation": _release_generation(), "archive": str(archive),
        "archive_sha256": archive_sha256,
    })
    payload = {"schema_version": 1, "records": records}
    _atomic_write_json(path, payload)


def _execute_real(status: object, args: argparse.Namespace, archive: Path) -> Dict[str, object]:
    if not getattr(status, "prerequisites_ready"):
        raise RuntimeError("组合静态前置条件未满足：%s" % getattr(status, "reason"))
    command_templates = [args.activate_command, args.start_command, args.revision_command, args.stop_command]
    if any(not value for value in command_templates):
        raise RuntimeError("实机模式必须显式提供 activate/start/revision/stop 四个 MCP 命令模板")
    result: Dict[str, object] = {
        "configuration": _save_combination(status),
        "steps": [],
        "systemd_before": _systemd_snapshot(args.services, args.command_timeout_sec),
    }
    primary_error = None
    stop_error = None
    try:
        for service in args.services:
            result["steps"].append(_run(["systemctl", "restart", service], args.restart_timeout_sec))
        result["after_restart"] = _systemd_snapshot(args.services, args.command_timeout_sec)
        for name, template in zip(
            ("activate", "start", "revision"),
            (args.activate_command, args.start_command, args.revision_command),
        ):
            command_result = _run(_render_command(template, status), args.command_timeout_sec)
            command_result["name"] = name
            result["steps"].append(command_result)
    except Exception as exc:
        primary_error = str(exc)
    finally:
        try:
            command_result = _run(
                _render_command(args.stop_command, status),
                args.command_timeout_sec,
            )
            command_result["name"] = "stop"
            result["steps"].append(command_result)
        except Exception as exc:
            stop_error = str(exc)
    result["final_systemd"] = _systemd_snapshot(args.services, args.command_timeout_sec)
    result["owner_profile_logs"] = _journal_snapshot(args.services, args.command_timeout_sec)
    if primary_error or stop_error:
        raise RuntimeError(
            json.dumps(
                {"primary_error": primary_error, "stop_error": stop_error},
                ensure_ascii=False,
                sort_keys=True,
            )
        )
    before_by_service = {
        snapshot["service"]: snapshot["properties"]
        for snapshot in result["systemd_before"]
    }
    for snapshot in result["final_systemd"]:
        props = snapshot["properties"]
        if props.get("ActiveState") != "active" or props.get("SubState") != "running":
            raise RuntimeError("服务最终状态异常：%s" % snapshot["service"])
        if int(props.get("MainPID", "0") or "0") <= 0:
            raise RuntimeError("服务 MainPID 无效：%s" % snapshot["service"])
        before_restarts = int(before_by_service.get(snapshot["service"], {}).get("NRestarts", "0") or "0")
        after_restarts = int(props.get("NRestarts", "0") or "0")
        if after_restarts > before_restarts:
            raise RuntimeError("服务验收期间发生重启：%s" % snapshot["service"])
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Geometry 24 组合可部署性矩阵 runner")
    parser.add_argument("--mode", choices=("dry-run", "static", "real"), default="dry-run")
    parser.add_argument("--archive-dir", type=Path, default=REPO_ROOT / "runtime_data" / "geometry_matrix_runs")
    parser.add_argument("--services", nargs="+", default=list(DEFAULT_SERVICES))
    parser.add_argument("--restart-timeout-sec", type=float, default=60.0)
    parser.add_argument("--command-timeout-sec", type=float, default=30.0)
    parser.add_argument("--activate-command", default="")
    parser.add_argument("--start-command", default="")
    parser.add_argument("--revision-command", default="")
    parser.add_argument("--stop-command", default="")
    parser.add_argument("--combination", action="append", default=[], help="lio,mapping,correction")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    requested = {tuple(value.split(",")) for value in args.combination}
    statuses = [
        status for status in geometry_backend_statuses()
        if not requested or (status.lio, status.mapping, status.correction) in requested
    ]
    plan = [status.as_dict() for status in statuses]
    if args.mode == "dry-run":
        print(json.dumps({"mode": args.mode, "combination_count": len(plan), "plan": plan}, ensure_ascii=False, indent=2))
        return 0
    if args.mode == "static":
        print(json.dumps({"mode": args.mode, "combination_count": len(plan), "statuses": plan}, ensure_ascii=False, indent=2))
        return 0

    run_root = args.archive_dir / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_root.mkdir(parents=True, exist_ok=False)
    owner_path = _geometry_owner_env_path()
    owner_existed = owner_path.exists()
    owner_stat = owner_path.stat() if owner_existed else None
    owner_content = owner_path.read_bytes() if owner_existed else b""
    owner_mode = owner_stat.st_mode & 0o777 if owner_stat else 0o600
    owner_uid = owner_stat.st_uid if owner_stat else os.getuid()
    owner_gid = owner_stat.st_gid if owner_stat else os.getgid()
    failures = 0
    try:
        for status in statuses:
            name = "%s__%s__%s" % (status.lio, status.mapping, status.correction)
            archive = run_root / (name + ".json")
            record: Dict[str, object] = {
                "started_at": _utc_now(),
                "release_generation": _release_generation(),
                "combination": status.as_dict(),
            }
            try:
                record["result"] = _execute_real(status, args, archive)
                record["passed"] = True
            except Exception as exc:
                failures += 1
                record["passed"] = False
                record["error"] = str(exc)
                record["final_systemd"] = _systemd_snapshot(args.services, args.command_timeout_sec)
                record["owner_profile_logs"] = _journal_snapshot(args.services, args.command_timeout_sec)
            record["completed_at"] = _utc_now()
            _atomic_write_json(archive, record)
            if record["passed"]:
                _record_acceptance(status, archive)
    finally:
        _restore_file(
            owner_path,
            owner_existed,
            owner_content,
            owner_mode,
            owner_uid,
            owner_gid,
        )
        restore_errors = []
        for service in args.services:
            try:
                _run(["systemctl", "restart", service], args.restart_timeout_sec)
            except Exception as exc:
                restore_errors.append("%s restart: %s" % (service, exc))
        restored = _systemd_snapshot(args.services, args.command_timeout_sec)
        for snapshot in restored:
            properties = snapshot["properties"]
            if properties.get("ActiveState") != "active" or properties.get("SubState") != "running":
                restore_errors.append("%s 未稳定运行" % snapshot["service"])
        if restore_errors:
            raise RuntimeError("恢复原配置运行态失败：%s" % "；".join(restore_errors))
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
