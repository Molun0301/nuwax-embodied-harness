from __future__ import annotations

import json
import logging
import os
import stat
import subprocess
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, List, Mapping, Optional, Sequence, Tuple

from .model import DeploymentOptions
from .profile import DeploymentProfile
from .runner import Runner, resolve_build_account, source_user_command, source_user_environment
from .state import StateStore

logger = logging.getLogger(__name__)

PHASE_ORDER: Tuple[str, ...] = (
    "preflight",
    "system",
    "toolchain",
    "python",
    "rust_build",
    "assets",
    "release",
    "config",
    "systemd",
    "safety_provision",
    "enable_start",
    "verify",
)


Callback = Callable[["ExecutionContext"], None]


@dataclass(frozen=True)
class Action:
    name: str
    display: str
    argv: Tuple[str, ...] = ()
    cwd: Optional[Path] = None
    env: Mapping[str, str] = field(default_factory=dict)
    as_build_user: bool = False
    network_isolated: bool = False
    timeout: Optional[int] = None
    callback_fn: Optional[Callback] = None

    @classmethod
    def command(
        cls,
        name: str,
        display: str,
        argv: Sequence[str],
        *,
        cwd: Optional[Path] = None,
        env: Optional[Mapping[str, str]] = None,
        as_build_user: bool = False,
        network_isolated: bool = False,
        timeout: Optional[int] = None,
    ) -> Action:
        return cls(
            name=name,
            display=display,
            argv=tuple(str(item) for item in argv),
            cwd=cwd,
            env=dict(env or {}),
            as_build_user=as_build_user,
            network_isolated=network_isolated,
            timeout=timeout,
        )

    @classmethod
    def callback(cls, name: str, callback_fn: Callback, display: Optional[str] = None) -> Action:
        return cls(name=name, display=display or name, callback_fn=callback_fn)

    def execute(self, context: ExecutionContext) -> None:
        if self.callback_fn is not None:
            self.callback_fn(context)
            return
        context.runner.run(
            self.argv,
            cwd=self.cwd,
            env=self.env,
            as_build_user=self.as_build_user,
            network_isolated=self.network_isolated,
            timeout=self.timeout,
        )


@dataclass(frozen=True)
class Phase:
    name: str
    description: str
    actions: Tuple[Action, ...]


@dataclass(frozen=True)
class DeploymentPlan:
    profile_name: str
    fingerprint: str
    phases: Tuple[Phase, ...]
    rollback: Optional[Callback] = None
    installation_inputs: Optional[Dict[str, Any]] = None
    release_owner_id: str = ""
    resumed_from_transaction_id: str = ""


@dataclass
class ExecutionContext:
    plan: DeploymentPlan
    profile: DeploymentProfile
    options: DeploymentOptions
    runner: Runner
    previous_release: Optional[str] = None
    release_switched: bool = False
    config_backups: Dict[Path, Optional[bytes]] = field(default_factory=dict)
    notes: Dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class ExecutionResult:
    status: str
    failed_phase: Optional[str] = None
    message: str = ""
    rendered_actions: Tuple[str, ...] = ()


class DeploymentEngine:
    def __init__(self, runner: Runner) -> None:
        self.runner = runner

    def execute(
        self,
        plan: DeploymentPlan,
        options: DeploymentOptions,
        profile: Optional[DeploymentProfile] = None,
    ) -> ExecutionResult:
        rendered = tuple(f"[{phase.name}] {action.display}" for phase in plan.phases for action in phase.actions)
        if options.dry_run:
            return ExecutionResult(status="planned", rendered_actions=rendered)
        if profile is None:
            profile = DeploymentProfile(
                schema_version=1,
                name=plan.profile_name,
                robot="test",
                production=False,
                supported_os={},
                architectures=(),
                system_packages=(),
                python=_empty_python_profile(),
                rust=_empty_rust_profile(),
                services=(),
            )
        context = ExecutionContext(
            plan=plan,
            profile=profile,
            options=options,
            runner=self.runner,
        )
        state = StateStore(options.state_file, enabled=options.persist_state)
        state.begin(
            plan.profile_name,
            plan.fingerprint,
            options.release_id,
            options.transaction_id,
            list(rendered),
            release_owner_id=plan.release_owner_id,
            installation_inputs=plan.installation_inputs,
            resumed_from_transaction_id=plan.resumed_from_transaction_id,
        )
        for phase in plan.phases:
            if context.notes.get("reuse_release") and phase.name in {
                "toolchain",
                "python",
                "rust_build",
                "assets",
            }:
                state.record_phase(phase.name, "skipped", "verified immutable release reused")
                continue
            state.record_phase(phase.name, "running")
            try:
                for action in phase.actions:
                    action.execute(context)
            except Exception as exc:
                logger.exception("Deployment phase %s failed", phase.name)
                rollback_error = ""
                terminal_status = "failed"
                if plan.rollback is not None:
                    try:
                        rollback_result = plan.rollback(context)
                        terminal_status = "failed" if rollback_result is False else "rolled_back"
                    except Exception as rollback_exc:
                        logger.exception("Deployment rollback failed after phase %s", phase.name)
                        rollback_error = f"; rollback failed: {rollback_exc}"
                        terminal_status = "rollback_failed"
                active_release = str(options.current_link.resolve()) if options.current_link.is_symlink() else None
                state.record_active_release(active_release)
                message = f"{exc}{rollback_error}"
                state.record_phase(phase.name, "failed", message)
                state.finish(terminal_status, phase.name)
                return ExecutionResult(
                    status="failed",
                    failed_phase=phase.name,
                    message=message,
                    rendered_actions=rendered,
                )
            state.record_phase(phase.name, "success")
        controlled_bringup = profile.production and any(
            (
                options.allow_unlocked_python,
                options.allow_unverified_third_party,
                options.allow_dirty_source,
                options.skip_models,
            )
        )
        final_status = (
            "staged"
            if (options.no_systemd or options.no_start or options.no_verify or controlled_bringup)
            else "success"
        )
        active_release = str(options.current_link.resolve()) if options.current_link.is_symlink() else None
        state.record_active_release(active_release)
        state.finish("committed" if final_status == "success" else final_status)
        return ExecutionResult(status=final_status, rendered_actions=rendered)


def _empty_python_profile():
    from .profile import PythonProfile

    return PythonProfile(requirements=())


def _empty_rust_profile():
    from .profile import RustProfile

    return RustProfile(toolchain="", builds=())


def _path_input_record(path: Optional[Path], options: DeploymentOptions, bundle_manifest=None) -> dict:
    if path is None:
        return {"kind": "absent"}
    path = path.resolve()
    if options.offline_bundle is not None and bundle_manifest is not None:
        try:
            relative = path.relative_to(options.offline_bundle.resolve()).as_posix()
        except ValueError:
            pass
        else:
            return {"kind": "signed-bundle-member", "path": relative}
    if options.source_is_snapshot and path == options.source_root.resolve():
        return {"kind": "git-snapshot", "revision": options.source_revision, "tree_id": options.source_tree_id}
    if not path.exists():
        return {"kind": "missing", "path": str(path)}
    if path.is_file():
        info = path.stat()
        return {"kind": "file", "path": str(path),
                "stat": [info.st_size, info.st_mtime_ns, info.st_ctime_ns, info.st_mode]}
    # Keep source enumeration local. No source payload or binary Git diff is read.
    git = subprocess.run(
        source_user_command(("git", "ls-files", "--cached", "--others", "--exclude-standard", "-z")),
        cwd=str(path), env=source_user_environment(), capture_output=True, check=False, timeout=30,
    )
    candidates = ([path / os.fsdecode(item) for item in git.stdout.split(b"\0") if item]
                  if git.returncode == 0 else list(path.rglob("*")))
    outputs = tuple(p.resolve() for p in (options.install_root, options.state_dir, options.cache_dir, options.config_dir))
    files = []
    for candidate in sorted(set(candidates)):
        relative = candidate.relative_to(path)
        if ".git" in relative.parts or "__pycache__" in relative.parts:
            continue
        if any(candidate == output or output in candidate.parents for output in outputs):
            continue
        try:
            info = candidate.lstat()
        except FileNotFoundError:
            continue
        if stat.S_ISREG(info.st_mode):
            files.append([relative.as_posix(), info.st_size, info.st_mtime_ns, info.st_ctime_ns, info.st_mode])
        elif stat.S_ISLNK(info.st_mode):
            files.append([relative.as_posix(), "symlink", os.readlink(candidate)])
    return {"kind": "tree", "path": str(path), "files": files}


def capture_installation_inputs(profile: DeploymentProfile, options: DeploymentOptions, bundle_manifest=None) -> dict:
    source = _path_input_record(options.fingerprint_source_root or options.source_root, options, bundle_manifest)
    if bundle_manifest is not None:
        source = {"kind": "signed-bundle", "manifest": bundle_manifest}
    paths = {name: _path_input_record(getattr(options, name), options, bundle_manifest) for name in (
        "trust_root", "go2_sdk", "go2_sdk_wheel", "clip_source", "clip_wheel",
        "cyclonedds_source", "cyclonedds_wheel", "actuator_manifest", "bundle_public_key", "legacy_config",
    )}
    paths["python"] = _path_input_record(Path(options.python_bin), options, bundle_manifest)
    payload = {
        "version": 1, "profile": asdict(profile), "source": source, "files": paths,
        "options": {name: getattr(options, name) for name in (
            "dds_interface", "go2_robot_ip", "gateway_host", "gateway_port", "skip_models", "export_yolo_engine",
        )},
        "reusable": not options.allow_dirty_source,
    }
    return json.loads(json.dumps(payload, sort_keys=True))


def _build_user_rust_paths() -> Tuple[Path, Path, Path]:
    account = resolve_build_account()
    cargo_home = Path(account.pw_dir) / ".cargo"
    rustup_home = Path(account.pw_dir) / ".rustup"
    return cargo_home, rustup_home, cargo_home / "bin"


def _cargo_environment(options: DeploymentOptions, target_dir: Path) -> Dict[str, str]:
    cargo_home, rustup_home, _toolchain_bin = _build_user_rust_paths()
    return {
        "AR": "/usr/bin/ar",
        "CC": "/usr/bin/cc",
        "CXX": "/usr/bin/c++",
        "CARGO_BUILD_RUSTC_WRAPPER": "",
        "CARGO_ENCODED_RUSTFLAGS": "",
        "CARGO_HOME": str(cargo_home),
        "CARGO_INCREMENTAL": "0",
        "CARGO_NET_OFFLINE": "true",
        "CARGO_TARGET_DIR": str(target_dir),
        "HOME": str(options.build_home),
        "PATH": (f"{options.toolchain_bin}:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"),
        "PKG_CONFIG": "/usr/bin/pkg-config",
        "PROTOC": "/usr/bin/protoc",
        "RUSTUP_HOME": str(rustup_home),
        "RUSTC_WRAPPER": "",
        "RUSTC_WORKSPACE_WRAPPER": "",
        "RUSTDOCFLAGS": "",
        "RUSTFLAGS": "",
    }


def _cargo_action(profile: DeploymentProfile, options: DeploymentOptions, build) -> Action:
    toolchain = profile.rust.toolchain
    _cargo_home, rustup_home, _build_user_bin = _build_user_rust_paths()
    cargo = str(options.toolchain_bin / "cargo")
    env = _cargo_environment(options, options.rust_target_dir(build.manifest))
    env.update(
        {
            "RUSTUP_HOME": str(rustup_home),
            "RUSTUP_TOOLCHAIN": toolchain,
        }
    )
    if options.offline_bundle:
        cargo = str(options.offline_bundle / "cargo-bin" / "cargo")
        vendor = options.offline_bundle / "cargo-vendor"
        env.update(
            {
                "CARGO_HOME": str(options.cargo_home),
                "CARGO_NET_OFFLINE": "true",
                "PATH": (
                    f"{options.offline_bundle / 'cargo-bin'}:"
                    "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
                ),
                "RUSTUP_HOME": str(options.offline_rustup_home),
                "RUSTUP_TOOLCHAIN": toolchain,
            }
        )
    argv: List[str] = [cargo, f"+{toolchain}"]
    if options.offline_bundle:
        argv.extend(
            (
                "--config",
                'source.crates-io.replace-with="vendored-sources"',
                "--config",
                f'source.vendored-sources.directory="{vendor}"',
            )
        )
    argv.extend(
        (
            "build",
            "--locked",
            "--release",
            "--manifest-path",
            str(options.source_root / build.manifest),
        )
    )
    if build.package:
        argv.extend(("--package", build.package))
    if build.binary:
        argv.extend(("--bin", build.binary))
    if build.features:
        argv.extend(("--features", ",".join(build.features)))
    display_parts = [
        f"cargo +{toolchain} build --locked --release",
        f"--manifest-path {build.manifest}",
    ]
    if build.package:
        display_parts.append(f"--package {build.package}")
    if build.binary:
        display_parts.append(f"--bin {build.binary}")
    if build.features:
        display_parts.append(f"--features {','.join(build.features)}")
    display = " ".join(display_parts)
    return Action.command(
        f"cargo-{build.name}",
        display,
        argv,
        cwd=options.source_root,
        env=env,
        as_build_user=True,
        network_isolated=profile.production or bool(options.offline_bundle),
        timeout=7200,
    )


def _cargo_metadata_action(
    profile: DeploymentProfile,
    options: DeploymentOptions,
    manifest: str,
    index: int,
) -> Action:
    toolchain = profile.rust.toolchain
    _cargo_home, rustup_home, _build_user_bin = _build_user_rust_paths()
    cargo = str(options.toolchain_bin / "cargo")
    env = _cargo_environment(options, options.rust_target_dir(manifest))
    env.update(
        {
            "RUSTUP_HOME": str(rustup_home),
            "RUSTUP_TOOLCHAIN": toolchain,
        }
    )
    argv: List[str] = []
    if options.offline_bundle:
        cargo = str(options.offline_bundle / "cargo-bin" / "cargo")
        vendor = options.offline_bundle / "cargo-vendor"
        env.update(
            {
                "CARGO_HOME": str(options.cargo_home),
                "CARGO_NET_OFFLINE": "true",
                "PATH": (
                    f"{options.offline_bundle / 'cargo-bin'}:"
                    "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
                ),
                "RUSTUP_HOME": str(options.offline_rustup_home),
                "RUSTUP_TOOLCHAIN": toolchain,
            }
        )
        argv.extend(
            (
                "--config",
                'source.crates-io.replace-with="vendored-sources"',
                "--config",
                f'source.vendored-sources.directory="{vendor}"',
            )
        )
    command = (
        cargo,
        f"+{toolchain}",
        *argv,
        "metadata",
        "--locked",
        "--no-deps",
        "--format-version",
        "1",
        "--manifest-path",
        str(options.source_root / manifest),
    )
    return Action.command(
        f"cargo-metadata-{index}",
        f"cargo +{toolchain} metadata --locked --manifest-path {manifest}",
        command,
        cwd=options.source_root,
        env=env,
        as_build_user=True,
        network_isolated=profile.production or bool(options.offline_bundle),
        timeout=300,
    )


def _geometry_release_action(
    profile: DeploymentProfile,
    options: DeploymentOptions,
) -> Optional[Action]:
    contract = profile.metadata.get("geometry_release")
    if not isinstance(contract, dict):
        return None
    native_payloads = contract.get("native_lio_payloads")
    if not isinstance(native_payloads, list) or not native_payloads:
        raise ValueError("geometry_release.native_lio_payloads must be a non-empty list")

    native_paths: List[Path] = []
    native_roles = set()
    for item in native_payloads:
        if not isinstance(item, dict) or set(item) != {"role", "source"}:
            raise ValueError("geometry_release native LIO entries must contain role and source")
        role = item.get("role")
        source = item.get("source")
        if role not in ("fastlio2", "pointlio") or source != f"{role}_native":
            raise ValueError(
                "geometry_release 原生 LIO role/source 必须匹配真实 C++ 程序，"
                f"不得使用 Rust 离线程序：role={role!r} source={source!r}"
            )
        if role in native_roles:
            raise ValueError(f"geometry_release 原生 LIO role 重复：{role}")
        native_roles.add(role)
        # 这里只引用按原生构建规范预先生成的程序；计划渲染不构建或创建占位文件。
        native_paths.append(
            options.source_root
            / "services/nuwax-workspace/crates/nuwax-lio-backends/native"
            / role / "cpp/result/bin" / source
        )

    _cargo_home, rustup_home, _build_user_bin = _build_user_rust_paths()
    cargo = str(options.toolchain_bin / "cargo")
    env = _cargo_environment(
        options,
        options.build_dir / "geometry-release" / "isolated-target",
    )
    env.update(
        {
            "RUSTUP_HOME": str(rustup_home),
            "RUSTUP_TOOLCHAIN": profile.rust.toolchain,
        }
    )
    if options.offline_bundle:
        cargo = str(options.offline_bundle / "cargo-bin" / "cargo")
        env["CARGO_HOME"] = str(options.cargo_home)
        env["CARGO_NET_OFFLINE"] = "true"
        env["CARGO_SOURCE_CRATES_IO_REPLACE_WITH"] = "vendored-sources"
        env["CARGO_SOURCE_VENDORED_SOURCES_DIRECTORY"] = str(
            options.offline_bundle / "cargo-vendor"
        )
        env["PATH"] = (
            f"{options.offline_bundle / 'cargo-bin'}:"
            "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
        )
        env["RUSTUP_HOME"] = str(options.offline_rustup_home)

    output_root = options.build_dir / "geometry-release" / "nuwax-geometry-releases"
    argv: List[str] = [
        options.python_bin,
        str(options.source_root / "scripts" / "build_geometry_release.py"),
        "--cargo",
        cargo,
        "--installer-output-root",
        str(output_root),
    ]
    for path in native_paths:
        argv.extend(("--native-lio", str(path)))
    return Action.command(
        "build-geometry-release",
        "使用预先构建的 C++ 原生 LIO 程序封存 Geometry 发布制品（不重建原生程序）："
        + ", ".join(str(path) for path in native_paths),
        argv,
        cwd=options.source_root,
        env=env,
        as_build_user=True,
        network_isolated=profile.production or bool(options.offline_bundle),
        timeout=7200,
    )


def build_deployment_plan(
    profile: DeploymentProfile,
    options: DeploymentOptions,
    *,
    installation_inputs: Optional[Dict[str, Any]] = None,
    release_owner_id: str = "",
    resumed_from_transaction_id: str = "",
) -> DeploymentPlan:
    from . import operations

    system_actions: List[Action]
    if options.offline_bundle:
        system_actions = [
            Action.callback(
                "install-offline-system-packages",
                operations.install_offline_system_packages,
                "install system packages from verified offline bundle",
            )
        ]
    else:
        system_actions = [
            Action.callback(
                "install-system-packages",
                operations.install_online_system_packages,
                "apt-get install -y " + " ".join(profile.system_packages),
            ),
        ]

    manifests = tuple(dict.fromkeys(build.manifest for build in profile.rust.builds))
    rust_actions = tuple(
        _cargo_metadata_action(profile, options, manifest, index) for index, manifest in enumerate(manifests, start=1)
    ) + tuple(_cargo_action(profile, options, build) for build in profile.rust.builds)
    geometry_release_action = _geometry_release_action(profile, options)
    if geometry_release_action is not None:
        native_input_check = Action.command(
            "check-geometry-native-inputs",
            "先检查预先构建的 C++ 原生 LIO 文件，再开始 Rust 构建（不启动算法）",
            (*geometry_release_action.argv, "--check-native-inputs"),
            cwd=geometry_release_action.cwd,
            env=geometry_release_action.env,
            as_build_user=geometry_release_action.as_build_user,
            network_isolated=geometry_release_action.network_isolated,
            timeout=30,
        )
        rust_actions = (native_input_check,) + rust_actions + (geometry_release_action,)

    admission_actions: List[Action] = []
    if options.offline_bundle:
        admission_actions.append(
            Action.callback(
                "offline-bundle-admission",
                operations.verify_input_admission,
                "verify approved key/profile and stage signed root-managed read-only bundle cache",
            )
        )
    elif profile.production and not options.allow_dirty_source:
        admission_actions.append(
            Action.callback(
                "source-snapshot-admission",
                operations.verify_input_admission,
                "verify clean Git identity and stage root-managed read-only source snapshot",
            )
        )
    model_contract = profile.models.get("yolo_weights")
    if isinstance(model_contract, dict):
        model_display = (
            "prepare verified model artifacts "
            f"url={model_contract.get('url') or '<unresolved>'} "
            f"sha256={model_contract.get('sha256') or '<unresolved>'}"
        )
    else:
        model_display = "prepare profile assets and model artifacts"

    phases = (
        Phase(
            "preflight",
            "Validate host, inputs, disk, network, and profile",
            (
                *admission_actions,
                Action.callback(
                    "preflight",
                    operations.preflight,
                    "strict host and input preflight",
                ),
            ),
        ),
        Phase("system", "Install system dependencies", tuple(system_actions)),
        Phase(
            "toolchain",
            "Install the pinned Rust toolchain",
            (
                Action.callback(
                    "rust-toolchain",
                    operations.ensure_rust_toolchain,
                    f"install and verify rustup toolchain {profile.rust.toolchain}",
                ),
            ),
        ),
        Phase(
            "python",
            "Create the application virtual environment and install locked dependencies",
            (
                Action.callback(
                    "python-environment",
                    operations.install_python_environment,
                    "create application venv and install profile requirements",
                ),
            ),
        ),
        Phase(
            "rust_build",
            "Build all production Rust binaries",
            rust_actions,
        ),
        Phase(
            "assets",
            "Prepare verified model and static assets",
            (
                Action.callback(
                    "assets",
                    operations.prepare_assets,
                    model_display,
                ),
            ),
        ),
        Phase(
            "release",
            "Stage an immutable candidate release without changing current",
            (
                Action.callback(
                    "install-release",
                    operations.install_release,
                    f"stage release {options.release_id or '<auto>'} under {options.releases_dir}",
                ),
            ),
        ),
        Phase(
            "config",
            "Write production configuration atomically",
            ()
            if options.no_systemd or options.no_start
            else (
                Action.callback(
                    "config",
                    operations.write_configuration,
                    f"write protected configuration under {options.config_dir}",
                ),
            ),
        ),
        Phase(
            "systemd",
            "Install service identities, directories, and units",
            ()
            if options.no_systemd or options.no_start
            else (
                Action.callback(
                    "systemd-install",
                    operations.install_systemd,
                    "install systemd identities, tmpfiles, and rendered units",
                ),
            ),
        ),
        Phase(
            "safety_provision",
            "Provision the verified actuator safety manifest",
            ()
            if options.no_systemd or options.no_start
            else (
                Action.callback(
                    "safety-provision",
                    operations.provision_safety,
                    "provision actuator manifest and durable safety records",
                ),
            ),
        ),
        Phase(
            "enable_start",
            "Enable and start the ordered service stack",
            ()
            if options.no_systemd or options.no_start
            else (
                Action.callback(
                    "activate-release",
                    operations.activate_release,
                    f"atomically activate release {options.release_id}",
                ),
                Action.callback(
                    "enable-start",
                    operations.enable_and_start,
                    "enable and start guard -> nav -> geometry -> person-follow -> gateway",
                ),
            ),
        ),
        Phase(
            "verify",
            "Verify services, LCM, and authoritative health",
            ()
            if options.no_verify
            else (
                Action.callback(
                    "verify",
                    operations.verify_deployment,
                    "verify runtime and require /api/health/full ready=true",
                ),
            ),
        ),
    )
    return DeploymentPlan(
        profile_name=profile.name,
        fingerprint="",
        installation_inputs=installation_inputs,
        release_owner_id=release_owner_id,
        resumed_from_transaction_id=resumed_from_transaction_id,
        phases=phases,
        rollback=operations.rollback,
    )
