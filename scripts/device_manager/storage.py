"""Retain referenced installations; reclaim only completed, owned generations.

Call prune_storage after a successful commit while holding the installation's
transaction lock; bootstrap calls it only after its launch succeeds.
This never cleans data, models, source/build caches, downloads or unknown paths.
"""
from __future__ import annotations

import os
import shutil
from pathlib import Path

from . import PACKAGE_ROOT
from .common import ManagerError, process_alive, read_json

TERMINAL = {"succeeded", "failed", "cancelled", "interrupted"}


def _record(path):
    # A missing, partial, symlinked or unknown completion record grants no ownership.
    if path.is_symlink():
        return {}
    try:
        value = read_json(path, {})
        return value if isinstance(value, dict) else {}
    except ManagerError:
        return {}


def _paths(value):
    if isinstance(value, str):
        if value.startswith("/"):
            yield Path(value)
    elif isinstance(value, dict):
        for item in value.values():
            yield from _paths(item)
    elif isinstance(value, list):
        for item in value:
            yield from _paths(item)


def _process_paths():
    """Physical cwd plus absolute argv preserve old managers and runtime owners."""
    for process in Path("/proc").iterdir():
        if not process.name.isdigit():
            continue
        try:
            if process.stat().st_uid != os.getuid():
                continue
        except OSError:
            continue
        for name in ("cwd", "exe"):
            try:
                yield (process / name).resolve(strict=True)
            except (OSError, RuntimeError):
                pass
        try:
            for argument in (process / "cmdline").read_bytes().split(b"\0"):
                if argument.startswith(b"/"):
                    yield Path(os.fsdecode(argument))
        except OSError:
            # Processes can exit during the scan. Explicit owned state below also
            # protects runtime/config owners whose /proc cwd is not readable.
            pass


def _generations(root, *, include_links=False):
    if root.resolve() != root or not root.is_dir():
        return []
    return sorted(path for path in root.iterdir() if path.is_dir() and (include_links or not path.is_symlink()))


def _references(home, roots, package_root):
    references = set()

    def keep(path):
        path = Path(path)
        # Preserve the named environment even when its interpreter resolves to /usr.
        for location in (path, path.resolve()):
            for root in roots:
                try:
                    relative = location.relative_to(root)
                except ValueError:
                    continue
                if relative.parts:
                    references.add(root / relative.parts[0])

    for link in (home / "current", home / "manager/current", home / "manager/previous"):
        keep(link)
    previous = read_json(home / "config/history.json", {}).get("previous")
    if isinstance(previous, str) and Path(previous).name == previous:
        keep(home / "releases" / previous)
    keep(package_root)
    for path in (home / "jobs").glob("*.json"):
        job = read_json(path, {})
        if job.get("status") not in TERMINAL or job.get("error", {}).get("code") == "recovery_failed":
            for reference in _paths(job):
                keep(reference)
            previous = job.get("_plan", {}).get("_previous")
            if isinstance(previous, str) and Path(previous).name == previous:
                keep(home / "releases" / previous)
    for name in ("runtime.json", "profile-owner.json", "server.json"):
        state = read_json(home / "config" / name, {})
        if process_alive(state.get("process")):
            for reference in _paths(state):
                keep(reference)
    for reference in _process_paths():
        keep(reference)
    return references


def storage_plan(home, *, package_root=PACKAGE_ROOT):
    """List currently unreferenced complete generations without changing anything.

    Environment references include every remaining release, even incomplete or
    unknown directories. Pruning releases first may free more environments.
    """
    home = Path(home).expanduser().resolve()
    releases = home / "releases"
    environments = home / "python-environments"
    managers = home / "manager/versions"
    roots = (releases, environments, managers)
    retained = _references(home, roots, package_root)
    release_paths = _generations(releases)
    for release in _generations(releases, include_links=True):
        record = _record(release / "installed.json")
        dependency = record.get("dependency_environment")
        for reference in ([Path(dependency)] if isinstance(dependency, str) else []) + [release / "venv"]:
            resolved = reference.resolve()
            try:
                relative = resolved.relative_to(environments)
            except ValueError:
                continue
            if relative.parts:
                retained.add(environments / relative.parts[0])
    result = {"releases": [], "python_environments": [], "manager_versions": []}
    for path in release_paths:
        record = _record(path / "installed.json")
        if (path not in retained and record.get("complete") is True
                and record.get("source") in {"git", "release"} and isinstance(record.get("version"), str)):
            result["releases"].append(str(path))
    for path in _generations(environments):
        record = _record(path / "complete.json")
        if (path not in retained and isinstance(record.get("inputs"), dict)
                and record["inputs"].get("schema") in (1, 2)
                and isinstance(record.get("packages"), list)):
            result["python_environments"].append(str(path))
    for path in _generations(managers):
        record = _record(path / "installed.json")
        manifest = _record(path / "manager-manifest.json")
        if (path not in retained and record.get("complete") is True and record.get("product") == "neh-manager"
                and manifest.get("product") == "neh-manager" and manifest.get("schema_version") == 1
                and isinstance(record.get("version"), str) and record["version"] == manifest.get("version")):
            result["manager_versions"].append(str(path))
    return result


def prune_storage(home, *, package_root=PACKAGE_ROOT):
    """Best-effort post-commit cleanup; returned warnings never undo an install."""
    result = {"removed": [], "warnings": []}

    def remove(paths):
        for name in paths:
            try:
                shutil.rmtree(name)
                result["removed"].append(name)
            except OSError as exc:
                result["warnings"].append({"path": name, "error": str(exc)})

    try:
        plan = storage_plan(home, package_root=package_root)
        remove(plan["releases"] + plan["manager_versions"])
        # Re-read after deletion: a release that could not be removed still owns
        # its environment, and newly orphaned environments can now be reclaimed.
        plan = storage_plan(home, package_root=package_root)
        remove(plan["python_environments"])
    except (ManagerError, OSError, ValueError, TypeError, AttributeError, RuntimeError) as exc:
        result["warnings"].append({"error": str(exc)})
    return result
