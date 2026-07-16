from __future__ import annotations

import json
import hashlib
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, Mapping, Tuple


class ProfileError(ValueError):
    pass


def _reject_duplicate_keys(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ProfileError(f"duplicate deployment profile key: {key}")
        result[key] = value
    return result


@dataclass(frozen=True)
class PythonProfile:
    requirements: Tuple[str, ...]
    version: str = ""
    constraints: str = ""
    lock_status: str = "pending"
    required_imports: Tuple[str, ...] = ()
    source_artifact_manifest: str = ""
    deny_distributions: Tuple[str, ...] = ()
    vendor_packages: Tuple[Mapping[str, Any], ...] = ()


@dataclass(frozen=True)
class RustBuild:
    name: str
    manifest: str
    binary: str
    output: str
    package: str = ""
    features: Tuple[str, ...] = ()


@dataclass(frozen=True)
class RustProfile:
    toolchain: str
    builds: Tuple[RustBuild, ...]
    bootstrap: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class DeploymentProfile:
    schema_version: int
    name: str
    robot: str
    production: bool
    supported_os: Mapping[str, Tuple[str, ...]]
    architectures: Tuple[str, ...]
    system_packages: Tuple[str, ...]
    python: PythonProfile
    rust: RustProfile
    services: Tuple[str, ...]
    models: Mapping[str, Any] = field(default_factory=dict)
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def supports(self, os_id: str, version_id: str, architecture: str) -> bool:
        allowed_versions = self.supported_os.get(os_id, ())
        return architecture in self.architectures and (not allowed_versions or version_id in allowed_versions)


def profile_sha256(profile: DeploymentProfile) -> str:
    payload = json.dumps(
        asdict(profile),
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _strings(value: Any, field_name: str) -> Tuple[str, ...]:
    if value is None:
        return ()
    if not isinstance(value, list) or not all(isinstance(item, str) and item for item in value):
        raise ProfileError(f"{field_name} must be a list of non-empty strings")
    return tuple(value)


def _supported_os(value: Any) -> Mapping[str, Tuple[str, ...]]:
    if isinstance(value, list):
        result: Dict[str, Tuple[str, ...]] = {}
        for entry in value:
            if not isinstance(entry, dict) or not isinstance(entry.get("id"), str):
                raise ProfileError("supported_os entries must contain id")
            result[entry["id"]] = _strings(entry.get("versions", []), "supported_os.versions")
        return result
    if not isinstance(value, dict):
        raise ProfileError("supported_os must be an object or list")
    return {str(key): _strings(versions, f"supported_os.{key}") for key, versions in value.items()}


def _rust_builds(value: Any) -> Tuple[RustBuild, ...]:
    if not isinstance(value, list) or not value:
        raise ProfileError("rust.builds must be a non-empty list")
    builds = []
    for raw in value:
        if not isinstance(raw, dict):
            raise ProfileError("rust.builds entries must be objects")
        binary = raw.get("bin") or raw.get("binary")
        required = (raw.get("name"), raw.get("manifest"), binary, raw.get("output"))
        if not all(isinstance(item, str) and item for item in required):
            raise ProfileError("rust build requires name, manifest, bin, and output")
        builds.append(
            RustBuild(
                name=str(raw["name"]),
                manifest=str(raw["manifest"]),
                binary=str(binary),
                output=str(raw["output"]),
                package=str(raw.get("package") or ""),
                features=_strings(raw.get("features", []), "rust.builds.features"),
            )
        )
    return tuple(builds)


def profile_path(profile: str, source_root: Path) -> Path:
    path = source_root / "deploy" / "profiles" / f"{profile}.json"
    if not path.is_file():
        raise ProfileError(f"deployment profile not found: {profile}")
    return path.resolve()


def load_profile(path: Path) -> DeploymentProfile:
    try:
        data = json.loads(
            path.read_text(encoding="utf-8"),
            object_pairs_hook=_reject_duplicate_keys,
        )
    except (OSError, json.JSONDecodeError, ProfileError) as exc:
        raise ProfileError(f"cannot read deployment profile {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise ProfileError("deployment profile root must be an object")
    if data.get("schema_version") != 1:
        raise ProfileError("unsupported deployment profile schema_version")
    python_raw = data.get("python")
    rust_raw = data.get("rust")
    if not isinstance(python_raw, dict) or not isinstance(rust_raw, dict):
        raise ProfileError("profile requires python and rust objects")
    for key in ("name", "robot"):
        if not isinstance(data.get(key), str) or not data[key]:
            raise ProfileError(f"profile requires {key}")
    production = data.get("production", False)
    if not isinstance(production, bool):
        raise ProfileError("production must be a boolean")
    toolchain = rust_raw.get("toolchain")
    if not isinstance(toolchain, str) or not toolchain:
        raise ProfileError("rust.toolchain is required")
    index_policy = python_raw.get("index_policy")
    if index_policy is None:
        index_policy = {}
    elif not isinstance(index_policy, dict):
        raise ProfileError("python.index_policy must be an object")
    vendor_packages_raw = python_raw.get("vendor_packages")
    if vendor_packages_raw is None:
        vendor_packages_raw = []
    elif not isinstance(vendor_packages_raw, list) or not all(isinstance(item, dict) for item in vendor_packages_raw):
        raise ProfileError("python.vendor_packages must be a list of objects")
    bootstrap = rust_raw.get("bootstrap")
    if bootstrap is None:
        bootstrap = {}
    elif not isinstance(bootstrap, dict):
        raise ProfileError("rust.bootstrap must be an object")
    models = data.get("models")
    if models is None:
        models = {}
    elif not isinstance(models, dict):
        raise ProfileError("models must be an object")
    metadata = data.get("metadata")
    if metadata is None:
        metadata = {}
    elif not isinstance(metadata, dict):
        raise ProfileError("metadata must be an object")
    return DeploymentProfile(
        schema_version=1,
        name=data["name"],
        robot=data["robot"],
        production=production,
        supported_os=_supported_os(data.get("supported_os", {})),
        architectures=_strings(data.get("architectures", []), "architectures"),
        system_packages=_strings(data.get("system_packages", []), "system_packages"),
        python=PythonProfile(
            requirements=_strings(python_raw.get("requirements", []), "python.requirements"),
            version=str(python_raw.get("version") or ""),
            constraints=str(python_raw.get("constraints") or ""),
            lock_status=str(python_raw.get("lock_status") or "pending"),
            required_imports=_strings(python_raw.get("required_imports", []), "python.required_imports"),
            source_artifact_manifest=str(python_raw.get("source_artifact_manifest") or ""),
            deny_distributions=_strings(
                index_policy.get("deny_distributions", []),
                "python.index_policy.deny_distributions",
            ),
            vendor_packages=tuple(dict(item) for item in vendor_packages_raw),
        ),
        rust=RustProfile(
            toolchain=toolchain,
            builds=_rust_builds(rust_raw.get("builds")),
            bootstrap=dict(bootstrap),
        ),
        services=_strings(data.get("services", []), "services"),
        models=models,
        metadata=metadata,
    )


def load_named_profile(profile: str, source_root: Path) -> DeploymentProfile:
    return load_profile(profile_path(profile, source_root))
