from __future__ import annotations

from dataclasses import dataclass, replace
import hashlib
from pathlib import Path
import re
from typing import Optional


_IDENTIFIER_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,191}")


def validate_identifier(value: str, label: str) -> str:
    if not isinstance(value, str) or _IDENTIFIER_RE.fullmatch(value) is None:
        raise ValueError(f"{label} must match {_IDENTIFIER_RE.pattern!r}: {value!r}")
    return value


@dataclass(frozen=True)
class DeploymentOptions:
    source_root: Path
    trust_root: Optional[Path] = None
    fingerprint_source_root: Optional[Path] = None
    source_is_snapshot: bool = False
    source_revision: str = ""
    source_tree_id: str = ""
    install_root: Path = Path("/opt/nuwax")
    config_dir: Path = Path("/etc/nuwax")
    state_dir: Path = Path("/var/lib/nuwax/installer")
    cache_dir: Path = Path("/var/cache/nuwax")
    python_bin: str = "/usr/bin/python3"
    release_id: str = ""
    transaction_id: str = ""
    go2_sdk: Optional[Path] = None
    go2_sdk_wheel: Optional[Path] = None
    clip_source: Optional[Path] = None
    clip_wheel: Optional[Path] = None
    cyclonedds_source: Optional[Path] = None
    cyclonedds_wheel: Optional[Path] = None
    actuator_manifest: Optional[Path] = None
    dds_interface: str = ""
    go2_robot_ip: str = ""
    gateway_host: str = "127.0.0.1"
    offline_bundle: Optional[Path] = None
    bundle_public_key: Optional[Path] = None
    legacy_config: Optional[Path] = None
    dry_run: bool = False
    allow_unsupported_host: bool = False
    allow_unlocked_python: bool = False
    allow_unverified_third_party: bool = False
    allow_dirty_source: bool = False
    skip_models: bool = False
    export_yolo_engine: bool = False
    no_systemd: bool = False
    no_start: bool = False
    no_verify: bool = False
    persist_state: bool = True

    def __post_init__(self) -> None:
        validate_identifier(self.release_id, "release_id")
        if self.transaction_id:
            validate_identifier(self.transaction_id, "transaction_id")

    @property
    def releases_dir(self) -> Path:
        return self.install_root / "releases"

    @property
    def current_link(self) -> Path:
        return self.install_root / "current"

    @property
    def build_dir(self) -> Path:
        release_id = self.release_id or "unresolved"
        return self.cache_dir / "build" / release_id

    @property
    def release_dir(self) -> Path:
        release_id = self.release_id or "unresolved"
        return self.releases_dir / release_id

    @property
    def state_file(self) -> Path:
        return self.state_dir / "state.json"

    def rust_target_dir(self, manifest: str) -> Path:
        key = hashlib.sha256(manifest.encode("utf-8")).hexdigest()[:16]
        return self.build_dir / "rust-targets" / key

    @property
    def offline_rustup_home(self) -> Path:
        return self.build_dir / "offline-rustup-home"

    @property
    def rustup_home(self) -> Path:
        return self.build_dir / "rustup-home"

    @property
    def cargo_home(self) -> Path:
        return self.build_dir / "cargo-home"

    @property
    def build_home(self) -> Path:
        return self.build_dir / "home"

    @property
    def toolchain_bin(self) -> Path:
        return self.build_dir / "toolchain-bin"

    @classmethod
    def for_tests(
        cls,
        *,
        source_root: Path,
        dry_run: bool = False,
    ) -> "DeploymentOptions":
        root = source_root.resolve()
        return cls(
            source_root=root,
            trust_root=root,
            install_root=root / "_install",
            config_dir=root / "_config",
            state_dir=root / "_state",
            cache_dir=root / "_cache",
            release_id="test-release",
            transaction_id="test-release-attempt",
            dry_run=dry_run,
            allow_unsupported_host=True,
            allow_unlocked_python=True,
            bundle_public_key=None,
            no_systemd=False,
            persist_state=False,
        )

    def with_release_id(self, release_id: str) -> "DeploymentOptions":
        return replace(self, release_id=release_id)
