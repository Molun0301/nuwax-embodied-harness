from __future__ import annotations

import argparse
from dataclasses import replace
import json
import os
import platform
import secrets
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional, Sequence

from .config import atomic_write
from .engine import (
    DeploymentEngine,
    DeploymentPlan,
    ExecutionContext,
    build_deployment_plan,
)
from .lock import InstallerLock
from .model import DeploymentOptions, validate_identifier
from .profile import ProfileError, load_named_profile, profile_sha256
from .runner import SubprocessRunner, source_user_command, source_user_environment


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _release_id(source_root: Path) -> str:
    try:
        revision = subprocess.check_output(
            source_user_command(("git", "rev-parse", "--short=12", "HEAD")),
            cwd=str(source_root),
            env=source_user_environment(),
            text=True,
            stderr=subprocess.DEVNULL,
            timeout=10,
        ).strip()
        dirty = subprocess.run(
            source_user_command(("git", "status", "--porcelain", "--untracked-files=normal")),
            cwd=str(source_root),
            env=source_user_environment(),
            capture_output=True,
            text=True,
            check=False,
            timeout=10,
        )
        is_dirty = dirty.returncode != 0 or bool(dirty.stdout.strip())
        if is_dirty:
            revision += "-dirty-" + time.strftime("%Y%m%d%H%M%S", time.gmtime())
        return revision
    except (OSError, subprocess.SubprocessError):
        return "source-" + time.strftime("%Y%m%d%H%M%S", time.gmtime())


def _new_transaction_id(release_id: str) -> str:
    return f"{release_id}-{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())}-{secrets.token_hex(4)}"


def _trusted_profile_root(options: DeploymentOptions) -> Path:
    if options.offline_bundle is not None:
        return options.trust_root or _repo_root()
    return options.source_root


def _load_profile_argument(profile: str, source_root: Path):
    validate_identifier(profile, "profile name")
    return load_named_profile(profile, source_root)


def _add_common_flags(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--profile", default="go2-jetson", help="deploy/profiles 下的 profile 名称")
    parser.add_argument("--source-root", type=Path, default=_repo_root())
    parser.add_argument("--install-root", type=Path, default=Path("/opt/nuwax"))
    parser.add_argument("--config-dir", type=Path, default=Path("/etc/nuwax"))
    parser.add_argument("--state-dir", type=Path, default=Path("/var/lib/nuwax/installer"))
    parser.add_argument("--cache-dir", type=Path, default=Path("/var/cache/nuwax"))
    parser.add_argument("--python-bin", default="/usr/bin/python3")
    parser.add_argument("--release-id", default="")
    unitree = parser.add_mutually_exclusive_group()
    unitree.add_argument("--go2-sdk", type=Path)
    unitree.add_argument("--go2-sdk-wheel", type=Path)
    clip = parser.add_mutually_exclusive_group()
    clip.add_argument("--clip-source", type=Path)
    clip.add_argument("--clip-wheel", type=Path)
    cyclone = parser.add_mutually_exclusive_group()
    cyclone.add_argument("--cyclonedds-source", type=Path)
    cyclone.add_argument("--cyclonedds-wheel", type=Path)
    parser.add_argument("--actuator-manifest", type=Path)
    parser.add_argument("--dds-interface", default="")
    parser.add_argument(
        "--go2-robot-ip",
        default="",
        help=(
            "Go2 本体 WebRTC 信令 IPv4 地址，不是部署 Jetson 的 SSH 地址；"
            "启用常驻视频流时必须显式提供"
        ),
    )
    parser.add_argument("--offline", action="store_true", help="禁止联网并只使用签名 bundle")
    parser.add_argument("--bundle", "--offline-bundle", dest="offline_bundle", type=Path)
    parser.add_argument("--bundle-public-key", type=Path)
    parser.add_argument(
        "--legacy-config",
        type=Path,
        help="显式授权从旧 .env 迁移配置和可变数据；不会自动读取仓库 .env",
    )
    parser.add_argument("--allow-unsupported-host", action="store_true")
    parser.add_argument("--allow-unlocked-python", action="store_true")
    parser.add_argument(
        "--allow-unverified-third-party",
        action="store_true",
        help="仅用于受控 bring-up；绕过 profile deployable=false 门，生产验收禁止使用",
    )
    parser.add_argument(
        "--allow-dirty-source",
        action="store_true",
        help="仅用于开发机；生产 profile 默认拒绝包含未提交改动的源码",
    )
    parser.add_argument("--skip-models", action="store_true")
    parser.add_argument("--export-yolo-engine", action="store_true")
    parser.add_argument("--no-systemd", action="store_true")
    parser.add_argument("--no-start", action="store_true")
    parser.add_argument("--no-verify", action="store_true")
    parser.add_argument("--json", action="store_true", help="输出机器可读 JSON")
    exposure = parser.add_mutually_exclusive_group()
    exposure.add_argument("--gateway-host", default="127.0.0.1")
    exposure.add_argument(
        "--expose-lan",
        action="store_true",
        help="显式把 gateway 绑定到 0.0.0.0；部署者负责防火墙隔离",
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python3 install.py",
        description="nuwax 可回滚一键部署工具",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)
    plan = subparsers.add_parser("plan", help="渲染真实部署动作，不执行副作用")
    _add_common_flags(plan)
    check = subparsers.add_parser("check", help="只执行严格 preflight，不修改系统")
    _add_common_flags(check)
    deploy = subparsers.add_parser("deploy", help="执行完整部署事务")
    _add_common_flags(deploy)
    resume = subparsers.add_parser("resume", help="按相同 profile 幂等重试部署")
    _add_common_flags(resume)
    resume.add_argument("--transaction-id", required=True)
    verify = subparsers.add_parser("verify", help="验证当前已部署 release")
    _add_common_flags(verify)
    status = subparsers.add_parser("status", help="显示最近一次部署状态")
    status.add_argument("--state-dir", type=Path, default=Path("/var/lib/nuwax/installer"))
    status.add_argument("--install-root", type=Path, default=Path("/opt/nuwax"))
    rollback = subparsers.add_parser("rollback", help="恢复指定事务记录的上一 committed release")
    rollback.add_argument("--state-dir", type=Path, default=Path("/var/lib/nuwax/installer"))
    rollback.add_argument("--install-root", type=Path, default=Path("/opt/nuwax"))
    rollback.add_argument("--config-dir", type=Path, default=Path("/etc/nuwax"))
    rollback.add_argument("--cache-dir", type=Path, default=Path("/var/cache/nuwax"))
    rollback.add_argument("--source-root", type=Path, default=_repo_root())
    rollback.add_argument("--transaction-id")
    rollback.add_argument("--to-release", type=Path)
    rollback.add_argument("--no-systemd", action="store_true")
    rollback.add_argument("--no-verify", action="store_true")
    rollback.add_argument("--json", action="store_true")
    bundle = subparsers.add_parser("verify-bundle", help="校验离线 bundle 签名、完整性和 SHA-256")
    bundle.add_argument("path", type=Path)
    bundle.add_argument("--public-key", type=Path, required=True)
    create_bundle = subparsers.add_parser("create-bundle", help="从已批准制品目录创建离线 bundle")
    create_bundle.add_argument("--output", type=Path, required=True)
    create_bundle.add_argument("--wheelhouse", type=Path, required=True)
    create_bundle.add_argument("--cargo-vendor", type=Path, required=True)
    create_bundle.add_argument("--apt-repo", type=Path, required=True)
    create_bundle.add_argument("--vendor", type=Path, required=True)
    create_bundle.add_argument("--rustup-home", type=Path, required=True)
    create_bundle.add_argument("--cargo-bin", type=Path, required=True)
    create_bundle.add_argument("--source", type=Path, required=True)
    create_bundle.add_argument("--licenses", type=Path, required=True)
    create_bundle.add_argument("--signing-key", type=Path, required=True)
    create_bundle.add_argument("--profile", default="go2-jetson")
    create_bundle.add_argument("--release-id", required=True)
    create_bundle.add_argument("--source-revision", required=True)
    create_bundle.add_argument("--models", type=Path)
    return parser


def _options(args: argparse.Namespace, *, dry_run: bool) -> DeploymentOptions:
    if args.offline and args.offline_bundle is None:
        raise ProfileError("--offline requires --bundle <path>")
    if args.offline_bundle is not None and not args.offline:
        raise ProfileError("--bundle requires explicit --offline")
    if args.offline and args.bundle_public_key is None:
        raise ProfileError("--offline requires --bundle-public-key <approved.pem>")
    if args.offline and not args.release_id:
        raise ProfileError("--offline requires the signed bundle --release-id")
    offline_bundle = None
    if args.offline_bundle:
        raw_bundle = args.offline_bundle.expanduser()
        if raw_bundle.is_symlink():
            raise ProfileError("offline bundle path must not be a symlink")
        offline_bundle = raw_bundle.resolve()
    source_root = args.source_root.expanduser().resolve()
    if offline_bundle is not None:
        source_root = (offline_bundle / "source").resolve()
    release_id = args.release_id or _release_id(source_root)
    validate_identifier(release_id, "release_id")
    requested_transaction = getattr(args, "transaction_id", "") or ""
    transaction_id = (
        validate_identifier(requested_transaction, "transaction_id")
        if requested_transaction
        else _new_transaction_id(release_id)
    )
    gateway_host = "0.0.0.0" if args.expose_lan else args.gateway_host
    return DeploymentOptions(
        source_root=source_root,
        trust_root=_repo_root(),
        install_root=args.install_root.expanduser(),
        config_dir=args.config_dir.expanduser(),
        state_dir=args.state_dir.expanduser(),
        cache_dir=args.cache_dir.expanduser(),
        python_bin=args.python_bin,
        release_id=release_id,
        transaction_id=transaction_id,
        go2_sdk=args.go2_sdk.expanduser().resolve() if args.go2_sdk else None,
        go2_sdk_wheel=(args.go2_sdk_wheel.expanduser().resolve() if args.go2_sdk_wheel else None),
        clip_source=args.clip_source.expanduser().resolve() if args.clip_source else None,
        clip_wheel=args.clip_wheel.expanduser().resolve() if args.clip_wheel else None,
        cyclonedds_source=(args.cyclonedds_source.expanduser().resolve() if args.cyclonedds_source else None),
        cyclonedds_wheel=(args.cyclonedds_wheel.expanduser().resolve() if args.cyclonedds_wheel else None),
        actuator_manifest=(args.actuator_manifest.expanduser().resolve() if args.actuator_manifest else None),
        dds_interface=args.dds_interface,
        go2_robot_ip=str(args.go2_robot_ip or "").strip(),
        gateway_host=gateway_host,
        offline_bundle=offline_bundle,
        bundle_public_key=(args.bundle_public_key.expanduser().resolve() if args.bundle_public_key else None),
        legacy_config=(args.legacy_config.expanduser().resolve() if args.legacy_config else None),
        dry_run=dry_run,
        allow_unsupported_host=args.allow_unsupported_host,
        allow_unlocked_python=args.allow_unlocked_python,
        allow_unverified_third_party=args.allow_unverified_third_party,
        allow_dirty_source=args.allow_dirty_source,
        skip_models=args.skip_models,
        export_yolo_engine=args.export_yolo_engine,
        no_systemd=args.no_systemd,
        no_start=args.no_start,
        no_verify=args.no_verify or args.no_start or args.no_systemd,
        persist_state=not dry_run,
    )


def _print_plan(plan: DeploymentPlan, *, json_output: bool = False) -> None:
    if json_output:
        print(
            json.dumps(
                {
                    "profile": plan.profile_name,
                    "fingerprint": plan.fingerprint,
                    "phases": [
                        {
                            "name": phase.name,
                            "description": phase.description,
                            "actions": [{"id": action.name, "display": action.display} for action in phase.actions],
                        }
                        for phase in plan.phases
                    ],
                },
                ensure_ascii=False,
                sort_keys=True,
            )
        )
        return
    print(f"profile={plan.profile_name} fingerprint={plan.fingerprint}")
    for index, phase in enumerate(plan.phases, start=1):
        print(f"[{index:02d}/{len(plan.phases):02d}] {phase.name}: {phase.description}")
        if not phase.actions:
            print("  - skipped by explicit option")
        for action in phase.actions:
            print(f"  + {action.display}")


def _load_json(path: Path, label: str) -> dict:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"cannot read {label} {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise RuntimeError(f"{label} must be a JSON object: {path}")
    return data


def _newest_transaction_state(state_dir: Path) -> dict:
    latest_path = state_dir / "state.json"
    latest_record = None
    if latest_path.is_file():
        if latest_path.is_symlink():
            raise RuntimeError(f"deployment state must not be a symlink: {latest_path}")
        latest_record = _load_json(latest_path, "deployment state")
    journal_records = {}
    transactions_dir = state_dir / "transactions"
    if transactions_dir.is_dir():
        candidate_paths = sorted(transactions_dir.glob("*.json"))
    else:
        candidate_paths = []
    for path in candidate_paths:
        if path.is_symlink():
            raise RuntimeError(f"deployment state must not be a symlink: {path}")
        record = _load_json(path, "deployment transaction")
        transaction_id = record.get("transaction_id")
        started_at = record.get("started_at")
        if not isinstance(transaction_id, str) or not isinstance(started_at, (int, float)):
            raise RuntimeError(f"deployment transaction identity is invalid: {path}")
        if path.stem != transaction_id:
            raise RuntimeError(f"deployment transaction filename does not match its identity: {path}")
        journal_records[transaction_id] = record
    records = list(journal_records.values())
    if latest_record is not None:
        latest_id = latest_record.get("transaction_id")
        if not isinstance(latest_id, str):
            raise RuntimeError("latest deployment state has no transaction_id")
        records.append(latest_record)
    if not records:
        return {}
    generated = [
        record for record in records if isinstance(record.get("generation"), int) and record["generation"] >= 0
    ]
    if generated:
        highest_generation = max(record["generation"] for record in generated)
        newest_ids = {record["transaction_id"] for record in generated if record["generation"] == highest_generation}
        if len(newest_ids) != 1:
            raise RuntimeError("deployment generation is ambiguous")
        newest_id = newest_ids.pop()
        return journal_records.get(newest_id) or next(
            record for record in generated if record["transaction_id"] == newest_id
        )
    if latest_record is not None:
        latest_id = latest_record["transaction_id"]
        return journal_records.get(latest_id, latest_record)
    if len(journal_records) == 1:
        return next(iter(journal_records.values()))
    raise RuntimeError("deployment state has no generation pointer")


def _save_transaction_state(state_dir: Path, state: dict) -> None:
    transaction_id = state.get("transaction_id")
    if not isinstance(transaction_id, str):
        raise RuntimeError("deployment transaction_id is missing")
    validate_identifier(transaction_id, "transaction_id")
    rendered = json.dumps(state, indent=2, sort_keys=True) + "\n"
    atomic_write(
        state_dir / "transactions" / f"{transaction_id}.json",
        rendered,
        mode=0o600,
    )
    atomic_write(state_dir / "state.json", rendered, mode=0o600)


def _assert_latest_transaction(selected: dict, latest: dict) -> None:
    selected_id = selected.get("transaction_id")
    if not latest or latest.get("transaction_id") != selected_id:
        raise RuntimeError("requested deployment transaction is stale; operate on the latest transaction")
    for key in ("profile", "fingerprint", "release_id", "transaction_id"):
        if latest.get(key) != selected.get(key):
            raise RuntimeError(f"latest deployment transaction {key} is inconsistent")


def _execute(
    args: argparse.Namespace,
    *,
    dry_run: bool,
    verify_only: bool = False,
    preflight_only: bool = False,
    resume: bool = False,
) -> int:
    options = _options(args, dry_run=dry_run)
    if options.offline_bundle is not None:
        validate_identifier(args.profile, "offline profile name")
        profile = load_named_profile(args.profile, _trusted_profile_root(options))
    else:
        profile = _load_profile_argument(args.profile, _trusted_profile_root(options))
    if (
        profile.robot == "go2"
        and "nuwax-go2-video-streamer.service" in profile.services
        and not options.go2_robot_ip
    ):
        raise ProfileError(
            "Go2 deployment with the always-on video streamer requires "
            "--go2-robot-ip <address>"
        )
    if not dry_run and not preflight_only and not verify_only and os.geteuid() != 0:
        print("部署需要 root；请使用 ./install.sh deploy ...", file=sys.stderr)
        return 77
    if not dry_run and os.geteuid() == 0:
        from .operations import validate_protected_paths

        validate_protected_paths(options)
    if options.offline_bundle is not None:
        from .operations import (
            stage_verified_bundle,
            validate_bundle_public_key,
            validate_offline_bundle_contract,
            verify_offline_bundle,
        )

        approved_key = validate_bundle_public_key(
            profile,
            options,
            options.bundle_public_key,
        )
        manifest = verify_offline_bundle(options.offline_bundle, approved_key)
        validate_offline_bundle_contract(profile, options, manifest)
        if not dry_run and not preflight_only and not verify_only:
            staged_bundle = stage_verified_bundle(
                options.offline_bundle,
                approved_key,
                options.cache_dir,
            )
            options = replace(
                options,
                offline_bundle=staged_bundle,
                source_root=staged_bundle / "source",
            )
    if (
        options.offline_bundle is None
        and profile.production
        and not options.allow_dirty_source
        and not dry_run
        and not preflight_only
        and not verify_only
    ):
        from .operations import stage_source_snapshot

        live_source = options.source_root
        with InstallerLock():
            source_snapshot, source_revision, source_tree_id = stage_source_snapshot(
                live_source,
                options.cache_dir,
            )
        snapshot_profile = load_named_profile(args.profile, source_snapshot)
        if profile_sha256(snapshot_profile) != profile_sha256(profile):
            raise RuntimeError("deployment profile changed while creating the production source snapshot")
        profile = snapshot_profile
        options = replace(
            options,
            source_root=source_snapshot,
            trust_root=source_snapshot,
            fingerprint_source_root=source_snapshot,
            source_is_snapshot=True,
            source_revision=source_revision,
            source_tree_id=source_tree_id,
        )
    plan = build_deployment_plan(profile, options)
    if verify_only or preflight_only:
        selected = "verify" if verify_only else "preflight"
        phases = tuple(phase for phase in plan.phases if phase.name == selected)
        plan = DeploymentPlan(
            profile_name=plan.profile_name,
            fingerprint=plan.fingerprint,
            phases=phases,
            rollback=None,
        )
        options = replace(options, persist_state=False)
    if dry_run:
        _print_plan(plan, json_output=args.json)
        return 0
    if os.geteuid() != 0:
        print("部署需要 root；请使用 ./install.sh deploy ...", file=sys.stderr)
        return 77
    runner = SubprocessRunner()
    unchanged = False
    result = None
    with InstallerLock():
        latest = _newest_transaction_state(options.state_dir)
        if not resume and not verify_only and not preflight_only:
            if latest.get("status") in {"running", "rollback_running", "rollback_failed"}:
                raise RuntimeError("an unresolved deployment transaction exists; resume or roll it back first")
            if (
                latest.get("status") == "committed"
                and latest.get("fingerprint") == plan.fingerprint
                and latest.get("release_id") == options.release_id
                and options.current_link.is_symlink()
                and options.current_link.resolve() == options.release_dir.resolve()
            ):
                selected_phases = tuple(phase for phase in plan.phases if phase.name in {"preflight", "verify"})
                validation_plan = DeploymentPlan(
                    profile_name=plan.profile_name,
                    fingerprint=plan.fingerprint,
                    phases=selected_phases,
                    rollback=None,
                )
                result = DeploymentEngine(runner=runner).execute(
                    validation_plan,
                    replace(options, persist_state=False),
                    profile,
                )
                unchanged = result.status == "success"
        if resume:
            transaction_path = options.state_dir / "transactions" / f"{args.transaction_id}.json"
            previous_state = _load_json(transaction_path, "deployment transaction")
            _assert_latest_transaction(previous_state, latest)
            if previous_state.get("fingerprint") != plan.fingerprint:
                raise RuntimeError("resume fingerprint does not match persisted transaction")
            host = previous_state.get("host")
            if not isinstance(host, dict) or (
                host.get("hostname") != platform.node() or host.get("machine") != platform.machine()
            ):
                raise RuntimeError("resume host facts do not match persisted transaction")
            if previous_state.get("status") in {
                "committed",
                "recovered",
                "rolled_back",
                "staged",
                "superseded",
            }:
                raise RuntimeError("terminal deployment transaction does not need resume")
            from . import operations

            recovery_context = ExecutionContext(plan, profile, options, runner)
            previous_state["status"] = "rollback_running"
            previous_state["finished_at"] = None
            previous_state.pop("recovery_error", None)
            _save_transaction_state(options.state_dir, previous_state)
            recovered = False
            try:
                if operations.load_persisted_recovery(recovery_context):
                    operations.rollback(recovery_context)
                    recovered = True
            except Exception as exc:
                previous_state["status"] = "rollback_failed"
                previous_state["finished_at"] = time.time()
                previous_state["recovery_error"] = str(exc)
                _save_transaction_state(options.state_dir, previous_state)
                raise
            new_transaction_id = _new_transaction_id(options.release_id)
            previous_state["status"] = "recovered" if recovered else "superseded"
            previous_state["finished_at"] = time.time()
            previous_state["resumed_by_transaction_id"] = new_transaction_id
            previous_state["active_release"] = (
                str(options.current_link.resolve()) if options.current_link.is_symlink() else None
            )
            _save_transaction_state(options.state_dir, previous_state)
            options = replace(
                options,
                transaction_id=new_transaction_id,
            )
        if result is None:
            result = DeploymentEngine(runner=runner).execute(plan, options, profile)
    if result.status == "failed":
        payload = {
            "status": "failed",
            "phase": result.failed_phase,
            "error": result.message,
        }
        print(
            json.dumps(payload, ensure_ascii=False)
            if args.json
            else (f"部署失败: phase={result.failed_phase} error={result.message}"),
            file=sys.stderr,
        )
        return 1
    if preflight_only:
        payload = {"status": "checked", "profile": profile.name, "fingerprint": plan.fingerprint}
        print(json.dumps(payload, ensure_ascii=False) if args.json else "部署 preflight 检查通过")
        return 0
    if verify_only:
        active_release = (
            str(options.current_link.resolve()) if options.current_link.is_symlink() else str(options.current_link)
        )
        payload = {"status": "verified", "profile": profile.name, "release": active_release}
        print(
            json.dumps(payload, ensure_ascii=False)
            if args.json
            else (f"当前 release 重新验收通过: profile={profile.name} release={active_release}")
        )
        return 0
    if unchanged:
        payload = {"status": "unchanged", "profile": profile.name, "release": options.release_id}
        print(
            json.dumps(payload, ensure_ascii=False)
            if args.json
            else (f"部署无变化并重新验收通过: profile={profile.name} release={options.release_id}")
        )
        return 0
    if result.status == "staged":
        payload = {"status": "staged", "profile": profile.name, "release": options.release_id}
        print(
            json.dumps(payload, ensure_ascii=False)
            if args.json
            else (f"部署制品已预置但未完成生产验收: profile={profile.name} release={options.release_id}")
        )
        return 0
    payload = {"status": "committed", "profile": profile.name, "release": options.release_id}
    print(
        json.dumps(payload, ensure_ascii=False)
        if args.json
        else (f"部署完成并通过验收: profile={profile.name} release={options.release_id}")
    )
    return 0


def _rollback_command(args: argparse.Namespace) -> int:
    if os.geteuid() != 0:
        print("rollback requires root", file=sys.stderr)
        return 77
    from .operations import validate_protected_paths

    state_dir = args.state_dir.expanduser()
    if args.transaction_id:
        validate_identifier(args.transaction_id, "transaction_id")
    with InstallerLock():
        validation_options = DeploymentOptions(
            source_root=_repo_root(),
            install_root=args.install_root.expanduser(),
            config_dir=args.config_dir.expanduser(),
            state_dir=state_dir,
            cache_dir=args.cache_dir.expanduser(),
            release_id="rollback-validation",
        )
        validate_protected_paths(validation_options)
        latest = _newest_transaction_state(state_dir)
        if not latest:
            raise RuntimeError("no deployment transaction is available for rollback")
        if args.transaction_id:
            state_path = state_dir / "transactions" / f"{args.transaction_id}.json"
            state = _load_json(state_path, "deployment transaction")
        else:
            state = latest
        _assert_latest_transaction(state, latest)
        if state.get("status") in {"recovered", "rolled_back", "superseded"}:
            raise RuntimeError("deployment transaction was already recovered")
        fingerprint = state.get("fingerprint")
        release_id = state.get("release_id")
        profile_name = state.get("profile")
        if not all(isinstance(value, str) and value for value in (fingerprint, release_id, profile_name)):
            raise RuntimeError("deployment transaction is missing rollback identity fields")
        source_root = args.source_root.expanduser().resolve()
        profile = load_named_profile(profile_name, source_root)
        options = DeploymentOptions(
            source_root=source_root,
            trust_root=_repo_root(),
            install_root=args.install_root.expanduser(),
            config_dir=args.config_dir.expanduser(),
            state_dir=state_dir,
            cache_dir=args.cache_dir.expanduser(),
            release_id=release_id,
            transaction_id=state.get("transaction_id", ""),
            no_systemd=args.no_systemd,
            no_verify=args.no_verify,
        )
        plan = DeploymentPlan(profile_name, fingerprint, ())
        context = ExecutionContext(plan, profile, options, SubprocessRunner())
        from . import operations

        state["status"] = "rollback_running"
        state["finished_at"] = None
        state.pop("rollback_error", None)
        _save_transaction_state(state_dir, state)
        try:
            if not operations.load_persisted_recovery(context):
                raise RuntimeError("transaction has no active candidate release to roll back")
            if args.to_release is not None:
                requested = args.to_release.expanduser().resolve()
                if context.previous_release is None or Path(context.previous_release).resolve() != requested:
                    raise RuntimeError("--to-release must match the transaction's recorded previous release")
            operations.rollback(context)
        except Exception as exc:
            state["status"] = "rollback_failed"
            state["finished_at"] = time.time()
            state["rollback_error"] = str(exc)
            _save_transaction_state(state_dir, state)
            raise
        state["status"] = "rolled_back"
        state["finished_at"] = time.time()
        active_release = str(options.current_link.resolve()) if options.current_link.is_symlink() else None
        state["active_release"] = active_release
        _save_transaction_state(state_dir, state)

    payload = {"status": "rolled_back", "release": active_release}
    print(
        json.dumps(payload, ensure_ascii=False)
        if args.json
        else (f"回滚完成并重新验收: release={active_release or '<none>'}")
    )
    return 0


def _status(state_dir: Path, install_root: Path) -> int:
    path = state_dir.expanduser() / "state.json"
    try:
        data = _newest_transaction_state(state_dir.expanduser())
    except FileNotFoundError:
        print(f"尚无部署状态: {path}")
        return 1
    except (OSError, json.JSONDecodeError, RuntimeError) as exc:
        print(f"无法读取部署状态: {exc}", file=sys.stderr)
        return 1
    if not data:
        print(f"尚无部署状态: {path}")
        return 1
    current = install_root.expanduser() / "current"
    current_is_symlink = current.is_symlink()
    active_release = str(current.resolve()) if current_is_symlink else None
    managed_releases = (install_root.expanduser() / "releases").resolve()
    recorded_active = data.get("active_release")
    if isinstance(recorded_active, str) and recorded_active:
        expected_path = Path(recorded_active).resolve()
    elif data.get("status") == "committed" and isinstance(data.get("release_id"), str):
        expected_path = (managed_releases / data["release_id"]).resolve()
    else:
        expected_path = None
    active_path = Path(active_release) if active_release else None
    active_exists = bool(active_path is not None and active_path.is_dir())
    data = dict(data)
    data["runtime"] = {
        "current_link": str(current),
        "active_release": active_release,
        "recorded_active_release": str(expected_path) if expected_path else None,
        "current_is_symlink": current_is_symlink,
        "dangling_symlink": current_is_symlink and not active_exists,
        "matches_recorded_release": (
            active_path == expected_path if active_exists and expected_path is not None else None
        ),
        "escapes_managed_releases": bool(active_path is not None and active_path.parent.resolve() != managed_releases),
        "unsafe_non_symlink": current.exists() and not current_is_symlink,
    }
    print(json.dumps(data, indent=2, ensure_ascii=False, sort_keys=True))
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    try:
        args = parser.parse_args(argv)
    except SystemExit as exc:
        return int(exc.code)
    try:
        if args.command == "plan":
            return _execute(args, dry_run=True)
        if args.command == "check":
            return _execute(args, dry_run=False, preflight_only=True)
        if args.command == "deploy":
            return _execute(args, dry_run=False)
        if args.command == "resume":
            return _execute(args, dry_run=False, resume=True)
        if args.command == "verify":
            return _execute(args, dry_run=False, verify_only=True)
        if args.command == "status":
            return _status(args.state_dir, args.install_root)
        if args.command == "rollback":
            return _rollback_command(args)
        if args.command == "verify-bundle":
            from .operations import verify_offline_bundle

            verify_offline_bundle(
                args.path.expanduser().resolve(),
                args.public_key.expanduser().resolve(),
            )
            print(f"offline bundle verified: {args.path}")
            return 0
        if args.command == "create-bundle":
            from .bundle import create_bundle

            validate_identifier(args.release_id, "release_id")
            source = args.source.expanduser().resolve()
            profile = _load_profile_argument(args.profile, source)
            output = create_bundle(
                args.output,
                wheelhouse=args.wheelhouse,
                cargo_vendor=args.cargo_vendor,
                apt_repo=args.apt_repo,
                vendor=args.vendor,
                rustup_home=args.rustup_home,
                cargo_bin=args.cargo_bin,
                source=source,
                licenses=args.licenses,
                signing_key=args.signing_key,
                metadata={
                    "profile": profile.name,
                    "profile_sha256": profile_sha256(profile),
                    "release_id": args.release_id,
                    "source_revision": args.source_revision,
                    "supported_os": {key: list(value) for key, value in profile.supported_os.items()},
                    "architectures": list(profile.architectures),
                    "python_version": profile.python.version,
                    "rust_toolchain": profile.rust.toolchain,
                },
                models=args.models,
            )
            print(f"offline bundle created: {output}")
            return 0
        parser.error(f"unknown command: {args.command}")
        return 64
    except (ProfileError, RuntimeError, OSError, ValueError) as exc:
        print(f"installer error: {exc}", file=sys.stderr)
        return 1


__all__ = ["build_parser", "main"]
