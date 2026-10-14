"""源码快照与构建选择；不读取机器人私有配置。"""
import hashlib
import json
import os
import subprocess
from pathlib import Path, PurePosixPath

WORKSPACE = "services/nuwax-workspace/"
NATIVE = WORKSPACE + "crates/nuwax-lio-backends/native/"
RUST = ("geometry", "navigation", "follow", "mapping")


def digest(path):
    value = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def safe_relative(name):
    path = PurePosixPath(name)
    if not name or path.is_absolute() or ".." in path.parts or str(path) != name:
        raise ValueError("非法相对路径：" + name)
    return path


def is_source(name):
    parts = safe_relative(name).parts
    return not (
        parts[0] in {".git", ".gitnexus", "runtime_data", "bin", "tests", "docs", ".nuwax-test-env"}
        or any(p in {"target", "__pycache__", ".venv", "result"} for p in parts)
        or any(p.startswith(".env") and p != ".env.example" for p in parts)
        or (parts[0] == "drivers" and ("configs" in parts or any(p.startswith(".active_") or p == ".settings.lock" for p in parts)))
    )


def inventory(root, paths):
    result = {}
    for name in sorted(set(paths)):
        if not is_source(name):
            continue
        path = Path(root) / name
        if path.is_symlink():
            raise ValueError("部署源码不接受符号链接：" + name)
        if path.is_file():
            result[name] = {"sha256": digest(path), "mode": 0o755 if path.stat().st_mode & 0o111 else 0o644}
    return result


def changed_files(old, new):
    return sorted(name for name in set(old) | set(new) if old.get(name) != new.get(name))


def select_components(changed, first=False):
    if first:
        return list(RUST) + ["fastlio2", "pointlio"]
    selected = set()
    for name in changed:
        if name.startswith((WORKSPACE, "services/navigation/proto/", ".cargo/")) or name in {"rust-toolchain", "rust-toolchain.toml", "Cargo.toml", "Cargo.lock"}:
            selected.update(RUST)
        if name.startswith("services/geometry/nuwax_geometry_runtime/"):
            selected.add("geometry")
        if name.startswith("scripts/go2_update/") or name == "scripts/module_installer/builds.py":
            selected.update(RUST)
            selected.update(("fastlio2", "pointlio"))
        for backend in ("fastlio2", "pointlio"):
            if name.startswith((NATIVE + backend + "/", NATIVE + "common/", NATIVE + "livox/")):
                selected.add(backend)
    return [name for name in (*RUST, "fastlio2", "pointlio") if name in selected]


def sync_snapshot(source, destination, files, previous):
    """只重写内容变化的文件，保留未变源码时间戳供构建工具复用。"""
    import shutil
    changed = changed_files(previous, files)
    root = Path(destination)
    # 先处理删除及其空目录，支持文件与目录之间的版本切换。
    for name in sorted(set(previous) - set(files), key=lambda item: item.count("/"), reverse=True):
        target = root / safe_relative(name)
        target.unlink(missing_ok=True)
        parent = target.parent
        while parent != root:
            try:
                parent.rmdir()
            except OSError:
                break
            parent = parent.parent
    for name in changed:
        target = Path(destination) / safe_relative(name)
        if name not in files:
            continue
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(Path(source) / name, target)
        target.chmod(files[name]["mode"])
        # Git归档时间不能成为新源码比旧产物更早的依据。
        os.utime(target, None)


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def read_json(path, default=None):
    path = Path(path)
    return json.loads(path.read_text(encoding="utf-8")) if path.exists() else default


def git(root, *args):
    return subprocess.check_output(["git", "-C", str(root), *args])
