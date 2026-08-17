from __future__ import annotations

import hashlib
import json
import os
import subprocess
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, List, Mapping, Optional, Sequence, Tuple

from .model import DeploymentOptions
from .profile import DeploymentProfile
from .runner import Runner, resolve_build_account, source_user_command, source_user_environment
from .state import StateStore


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
    ) -> "Action":
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
    def callback(cls, name: str, callback_fn: Callback, display: Optional[str] = None) -> "Action":
        return cls(name=name, display=display or name, callback_fn=callback_fn)

    def execute(self, context: "ExecutionContext") -> None:
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
                rollback_error = ""
                terminal_status = "failed"
                if plan.rollback is not None:
                    try:
                        rollback_result = plan.rollback(context)
                        terminal_status = "failed" if rollback_result is False else "rolled_back"
                    except Exception as rollback_exc:
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


def _file_digest(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _path_identity(path: Optional[Path]) -> str:
    if path is None:
        return ""
    path = path.resolve()
    if path.is_file():
        return f"file:{_file_digest(path)}"
    if not path.is_dir():
        return f"missing:{path}"
    git_environment = source_user_environment()
    git = subprocess.run(
        source_user_command(("git", "rev-parse", "HEAD")),
        cwd=str(path),
        env=git_environment,
        capture_output=True,
        check=False,
        timeout=30,
    )
    if git.returncode == 0:
        diff = subprocess.run(
            source_user_command(("git", "diff", "--binary", "HEAD", "--")),
            cwd=str(path),
            env=git_environment,
            capture_output=True,
            check=False,
            timeout=60,
        )
        status = subprocess.run(
            source_user_command(
                (
                    "git",
                    "status",
                    "--porcelain=v1",
                    "-z",
                    "--untracked-files=all",
                )
            ),
            cwd=str(path),
            env=git_environment,
            capture_output=True,
            check=False,
            timeout=30,
        )
        if diff.returncode == 0 and status.returncode == 0:
            untracked = hashlib.sha256()
            for record in status.stdout.split(b"\0"):
                if not record.startswith(b"?? "):
                    continue
                relative = Path(os.fsdecode(record[3:]))
                candidate = path / relative
                if relative.is_absolute() or ".." in relative.parts:
                    raise ValueError(f"unsafe untracked source path: {relative}")
                if candidate.is_symlink() or not candidate.is_file():
                    raise ValueError(f"unsupported untracked source input: {relative}")
                untracked.update(os.fsencode(relative.as_posix()))
                untracked.update(b"\0")
                untracked.update(_file_digest(candidate).encode("ascii"))
                untracked.update(b"\0")
            payload = git.stdout.strip() + b"\0" + diff.stdout + b"\0" + status.stdout + b"\0" + untracked.digest()
            return "git:" + hashlib.sha256(payload).hexdigest()
    digest = hashlib.sha256()
    for candidate in sorted(path.rglob("*")):
        relative = candidate.relative_to(path)
        if ".git" in relative.parts or "__pycache__" in relative.parts:
            continue
        if candidate.is_symlink() or not candidate.is_file():
            continue
        digest.update(os.fsencode(relative.as_posix()))
        digest.update(b"\0")
        digest.update(_file_digest(candidate).encode("ascii"))
        digest.update(b"\0")
    return "tree:" + digest.hexdigest()


def _fingerprint(profile: DeploymentProfile, options: DeploymentOptions) -> str:
    payload = {
        "profile": asdict(profile),
        "source_identity": _path_identity(options.fingerprint_source_root or options.source_root),
        "trust_identity": _path_identity(options.trust_root),
        "release_id": options.release_id,
        "go2_sdk": _path_identity(options.go2_sdk),
        "go2_sdk_wheel": _path_identity(options.go2_sdk_wheel),
        "clip_source": _path_identity(options.clip_source),
        "clip_wheel": _path_identity(options.clip_wheel),
        "cyclonedds_source": _path_identity(options.cyclonedds_source),
        "cyclonedds_wheel": _path_identity(options.cyclonedds_wheel),
        "actuator_manifest": _path_identity(options.actuator_manifest),
        "offline_bundle_manifest": _path_identity(
            options.offline_bundle / "bundle-manifest.json" if options.offline_bundle else None
        ),
        "bundle_public_key": _path_identity(options.bundle_public_key),
        "legacy_config": _path_identity(options.legacy_config),
        "dds_interface": options.dds_interface,
        "gateway_host": options.gateway_host,
        "allow_unverified_third_party": options.allow_unverified_third_party,
        "allow_dirty_source": options.allow_dirty_source,
        "controlled_bringup_transaction": (options.transaction_id if options.allow_dirty_source else ""),
        "skip_models": options.skip_models,
        "export_yolo_engine": options.export_yolo_engine,
    }
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _build_user_rust_paths() -> Tuple[Path, Path, Path]:
    account = resolve_build_account()
    cargo_home = Path(account.pw_dir) / ".cargo"
    rustup_home = Path(account.pw_dir) / ".rustup"
    return cargo_home, rustup_home, cargo_home / "bin"


def _cargo_environment(options: DeploymentOptions, target_dir: Path) -> Dict[str, str]:
    cargo_home, rustup_home, toolchain_bin = _build_user_rust_paths()
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

    builds_by_binary = {build.binary: build for build in profile.rust.builds if build.binary}
    native_paths: List[Path] = []
    for item in native_payloads:
        if not isinstance(item, dict) or set(item) != {"role", "source"}:
            raise ValueError("geometry_release native LIO entries must contain role and source")
        source = item.get("source")
        build = builds_by_binary.get(source) if isinstance(source, str) else None
        if build is None:
            raise ValueError(f"geometry_release source has no Rust build: {source}")
        native_paths.append(options.rust_target_dir(build.manifest) / "release" / source)

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
        "build verified Geometry release manifest and artifact generation",
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
        rust_actions += (geometry_release_action,)

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
        fingerprint=_fingerprint(profile, options),
        phases=phases,
        rollback=operations.rollback,
    )
