"""从固定源码组装独立设备管理程序，不构建机器人业务产物。"""
from __future__ import annotations

import argparse
import gzip
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tarfile
import tempfile

from scripts.neh_source import SourceError, source_allowed

from .common import atomic_json, digest_file


SOURCE_DIRECTORIES = ("scripts/device_manager", "scripts/module_installer")
SOURCE_FILES = (
    "scripts/__init__.py", "core/config_values.py", "core/config_catalog.json",
    "core/config_descriptions.json", "scripts/neh_bootstrap.py", "scripts/neh_system_helper.py",
    "scripts/neh_source.py", "scripts/neh_source_build.py", "scripts/go2_dependencies.py",
    "scripts/systemd/check_lcm_multicast.sh", "scripts/systemd/prepare_lcm_network.py",
)
REQUIRED_FILES = (
    "scripts/device_manager/__main__.py", "scripts/device_manager/static/index.html",
    "scripts/device_manager/common.py", "scripts/device_manager/source_package.py",
    "scripts/module_installer/catalog.py", "scripts/module_installer/builds.py",
    "scripts/module_installer/manager.py", "core/config_values.py", "scripts/neh_bootstrap.py",
    "scripts/neh_system_helper.py", "scripts/neh_source.py", "scripts/neh_source_build.py",
    "scripts/go2_dependencies.py", "scripts/systemd/check_lcm_multicast.sh",
    "scripts/systemd/prepare_lcm_network.py",
)
MANIFEST_NAME = "manager-manifest.json"


class SourcePackageError(RuntimeError):
    """管理源码或组装输出不能满足启动条件。"""


def _git(root: Path, *args: str) -> str:
    environment = {key: value for key, value in os.environ.items() if not key.startswith("GIT_")}
    environment.update(GIT_CONFIG_NOSYSTEM="1", GIT_CONFIG_GLOBAL=os.devnull, GIT_TERMINAL_PROMPT="0")
    result = subprocess.run(
        ["git", "-c", "core.hooksPath=/dev/null", "-c", "core.fsmonitor=false", "-C", str(root), *args],
        env=environment, capture_output=True, text=True, check=False,
    )
    if result.returncode:
        raise SourcePackageError("无法读取Git源码：" + result.stderr)
    return result.stdout.strip()


def _copy_regular(source: Path, target: Path) -> None:
    if source.is_symlink() or not source.is_file():
        raise SourcePackageError("输入包含链接或非普通文件：" + str(source))
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source, target)
    target.chmod(0o755 if source.stat().st_mode & 0o111 else 0o644)


def _pack(stage: Path, output: Path, manifest: dict) -> dict:
    if output.exists():
        raise SourcePackageError("管理归档已存在，禁止覆盖：" + str(output))
    files = [
        {"path": path.relative_to(stage).as_posix(), "size": path.stat().st_size, "sha256": digest_file(path)}
        for path in sorted(stage.rglob("*")) if path.is_file()
    ]
    manifest = {**manifest, "files": files}
    atomic_json(stage / MANIFEST_NAME, manifest)
    temporary = output.with_name(output.name + ".partial")
    raw = temporary.open("xb")
    try:
        with raw, gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=0) as compressed:
            with tarfile.open(fileobj=compressed, mode="w|", format=tarfile.PAX_FORMAT) as archive:
                for path in sorted(stage.rglob("*")):
                    if not path.is_file():
                        continue
                    info = archive.gettarinfo(str(path), arcname=path.relative_to(stage).as_posix())
                    info.uid = info.gid = info.mtime = 0
                    info.uname = info.gname = ""
                    info.mode = 0o755 if path.stat().st_mode & 0o111 else 0o644
                    with path.open("rb") as source:
                        archive.addfile(info, source)
        os.replace(temporary, output)
    finally:
        temporary.unlink(missing_ok=True)
    return manifest


def build_manager(root: Path, output: Path, *, version: str, workspace: bool = False) -> dict:
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._+-]{0,95}", version):
        raise SourcePackageError("版本只允许字母、数字、点、下划线、加减号，最长96字符")
    root, output = root.resolve(), output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    revision = _git(root, "rev-parse", "HEAD")
    dirty = bool(_git(root, "status", "--porcelain", "--untracked-files=normal"))
    if dirty and not workspace:
        raise SourcePackageError("管理源码必须干净；本地验证包含未提交修改时显式使用--source workspace")
    with tempfile.TemporaryDirectory(prefix="neh-manager-", dir=str(output.parent)) as directory:
        stage = Path(directory)
        for relative in SOURCE_DIRECTORIES:
            source = root / relative
            if source.is_symlink() or not source.is_dir():
                raise SourcePackageError("管理包源码目录缺失或为链接：" + relative)
            for path in sorted(source.rglob("*")):
                if not source_allowed(path.relative_to(root).as_posix()):
                    continue
                if path.is_symlink():
                    raise SourcePackageError("输入包含链接：" + str(path))
                if path.is_file():
                    _copy_regular(path, stage / path.relative_to(root))
        for relative in SOURCE_FILES:
            if (root / relative).is_file():
                _copy_regular(root / relative, stage / relative)
        for relative in REQUIRED_FILES:
            if not (stage / relative).is_file():
                raise SourcePackageError("管理包缺少运行文件：" + relative)
        initializer = stage / "scripts/device_manager/__init__.py"
        content, count = re.subn(
            r"^__version__\s*=\s*[^\n]+$", "__version__ = " + repr(version),
            initializer.read_text(encoding="utf-8"), flags=re.MULTILINE,
        )
        if count != 1:
            raise SourcePackageError("管理源码缺少唯一__version__赋值，无法写入版本")
        initializer.write_text(content, encoding="utf-8")
        manifest = _pack(stage, output, {
            "schema_version": 1, "product": "neh-manager", "kind": "manager", "installation_layout": 2,
            "version": version, "revision": revision, "source_mode": "workspace" if workspace else "git",
            "source_dirty": dirty, "python": {"minimum": "3.8", "maximum_exclusive": "3.13"},
        })
        atomic_json(output.with_name(output.name + ".manifest.json"), manifest)
        return manifest


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="从固定源码组装独立设备管理程序")
    parser.add_argument("--root", type=Path, default=Path.cwd())
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--version", required=True)
    parser.add_argument("--source", choices=("git", "workspace"), default="git")
    args = parser.parse_args(argv)
    try:
        manifest = build_manager(args.root, args.output, version=args.version, workspace=args.source == "workspace")
    except (SourcePackageError, SourceError, ValueError, OSError) as exc:
        parser.exit(1, "管理源码组装失败：" + str(exc) + "\n")
    print(json.dumps({"output": str(args.output), "version": manifest["version"], "files": len(manifest["files"])},
                     ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
