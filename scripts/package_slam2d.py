#!/usr/bin/env python3
"""打包二维工作程序、真实动态库依赖和核心源码许可，支持交叉构建产物。"""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tarfile
import tempfile

ROOT = Path(__file__).resolve().parents[1]
_SYSTEM_ABI = {"libc.so.6", "libm.so.6", "libpthread.so.0", "librt.so.1", "libdl.so.2", "ld-linux-aarch64.so.1", "ld-linux-x86-64.so.2"}


def elf_info(path):
    result = subprocess.run(["readelf", "-h", "-d", str(path)], check=True, text=True, capture_output=True).stdout
    machine = re.search(r"Machine:\s*(.+)", result).group(1).strip()
    needed = re.findall(r"\(NEEDED\).*\[([^\]]+)\]", result)
    search = []
    for value in re.findall(r"\((?:RUNPATH|RPATH)\).*\[([^\]]+)\]", result):
        for item in value.split(":"):
            search.append(Path(item.replace("$ORIGIN", str(path.parent))))
    return machine, needed, search


def package(worker, driver, output, *, library_roots=(), notice_roots=()):
    worker, driver = Path(worker).resolve(strict=True), Path(driver).resolve(strict=True)
    output = Path(output).absolute()
    if output.exists():
        raise ValueError("二维打包输出必须是新目录")
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix=".slam2d-package-", dir=str(output.parent)))
    try:
        (temporary / "bin").mkdir()
        (temporary / "lib").mkdir()
        machine, needed, rpath = elf_info(worker)
        roots = [Path(p).resolve() for p in library_roots] + rpath
        libraries = {"libnuwax_slamtec.so.1": driver}
        pending = list(needed)
        while pending:
            name = pending.pop()
            if name in _SYSTEM_ABI or name in libraries:
                continue
            if Path(name).name != name:
                raise ValueError("二维动态依赖包含非规范名称：" + name)
            candidates = [root / name for root in roots if (root / name).is_file()]
            if not candidates:
                raise RuntimeError("缺少目标动态库：" + name)
            selected = candidates[0].resolve(strict=True)
            architecture, transitive, search = elf_info(selected)
            if architecture != machine:
                raise RuntimeError("二维包混入另一架构动态库：" + str(selected))
            libraries[name] = selected
            roots.extend(p for p in search if p not in roots)
            pending.extend(transitive)
        driver_machine, driver_needed, _ = elf_info(driver)
        if driver_machine != machine or set(driver_needed) - _SYSTEM_ABI - set(libraries):
            raise RuntimeError("二维雷达库架构或依赖与工作程序不一致")
        shutil.copyfile(worker, temporary / "bin/nuwax-slam-2d-worker")
        (temporary / "bin/nuwax-slam-2d-worker").chmod(0o755)
        entries = [{"role": "planar_worker", "path": "bin/nuwax-slam-2d-worker", "executable": True}]
        for name, source in sorted(libraries.items()):
            destination = temporary / "lib" / name
            shutil.copyfile(source, destination)
            destination.chmod(0o644)
            role = "planar_core" if name.startswith("libnuwax_slam_toolbox.") else "planar_driver" if name.startswith("libnuwax_slamtec.") else "planar_library:" + name
            entries.append({"role": role, "path": "lib/" + name, "executable": False})
        archive = temporary / "二维算法源码与许可.tar.gz"
        with tarfile.open(archive, "w:gz") as bundle:
            for source in (ROOT / "services/nuwax-workspace/crates/nuwax-slam-toolbox-sys/native", ROOT / "drivers/sensors/slamtec_native"):
                bundle.add(source, arcname=source.relative_to(ROOT).as_posix())
            for source in (ROOT / "scripts/build_slam2d.sh", ROOT / "scripts/build_slamtec_driver.sh", ROOT / "scripts/prepare_slam2d_dependencies.py"):
                bundle.add(source, arcname=source.relative_to(ROOT).as_posix())
            for index, root in enumerate(notice_roots):
                root = Path(root)
                if root.is_dir():
                    for path in sorted(root.glob("*/copyright")):
                        bundle.add(path.resolve(strict=True), arcname="依赖许可/%d/%s/copyright" % (index, path.parent.name))
                    for path in sorted(root.iterdir()):
                        if path.is_file():
                            bundle.add(path.resolve(strict=True), arcname="依赖许可/%d/%s" % (index, path.name))
        entries.append({"role": "planar_sources", "path": archive.name, "executable": False})
        for entry in entries:
            path = temporary / entry["path"]
            digest = hashlib.sha256()
            with path.open("rb") as source:
                for chunk in iter(lambda: source.read(1024 * 1024), b""):
                    digest.update(chunk)
            entry.update(size=path.stat().st_size, sha256=digest.hexdigest())
        manifest = {"schema_version": 1, "machine": machine, "system_abi_libraries": sorted(_SYSTEM_ABI), "artifacts": entries}
        (temporary / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
        temporary.rename(output)
        return manifest
    finally:
        if temporary.exists():
            shutil.rmtree(temporary)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--worker", type=Path, required=True)
    parser.add_argument("--driver", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--library-root", action="append", default=[])
    parser.add_argument("--notice-root", action="append", default=[])
    args = parser.parse_args()
    package(args.worker, args.driver, args.output, library_roots=args.library_root, notice_roots=args.notice_root)

if __name__ == "__main__":
    main()
