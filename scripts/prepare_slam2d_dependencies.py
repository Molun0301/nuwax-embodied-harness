#!/usr/bin/env python3
"""从固定提交构建二维算法依赖，支持本机和显式目标工具链。"""
from __future__ import annotations
import argparse
import hashlib
import json
import re
import shutil
import tarfile
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.module_installer.builds import _checkout, _download
from scripts.module_installer.hardware import resolve_build_jobs  # noqa: E402

SOURCES = {
    "oneTBB": ("https://github.com/oneapi-src/oneTBB.git", "8b829acc65569019edb896c5150d427f288e8aba"),
    "ceres": ("https://github.com/ceres-solver/ceres-solver.git", "85331393dc0dff09f6fb9903ab0c4bfa3e134b01"),
}
SOURCE_ROLES = {"oneTBB": "slam2d-onetbb", "ceres": "slam2d-ceres"}


BOOST_URL = "https://archives.boost.io/release/1.83.0/source/boost_1_83_0.tar.bz2"
BOOST_SHA256 = "6478edfe2f3305127cffe8caf73ea0176c53769f4bf1585be237eb30798c3b8e"


def _boost(prefix, cache, compiler, jobs, run):
    archive = cache / "downloads/boost_1_83_0.tar.bz2"
    if not archive.is_file():
        _download(BOOST_URL, archive)
    digest = hashlib.sha256()
    with archive.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    if digest.hexdigest() != BOOST_SHA256:
        raise RuntimeError("Boost源码归档摘要不一致")
    source_root = cache / "boost-source/boost_1_83_0"
    if not source_root.is_dir():
        extraction = cache / "boost-source"
        extraction.mkdir(parents=True, exist_ok=True)
        with tarfile.open(archive) as source:
            for item in source.getmembers():
                target = (extraction / item.name).resolve()
                if extraction.resolve() not in target.parents or item.isdev():
                    raise RuntimeError("Boost源码归档路径非法")
                if item.issym() or item.islnk():
                    link = (target.parent / item.linkname).resolve()
                    if extraction.resolve() not in link.parents:
                        raise RuntimeError("Boost源码归档链接越界")
            source.extractall(extraction)
    if not (source_root / "b2").is_file():
        host_env = dict(os.environ)
        host_env.update(CC=host_env.get("HOST_CC", "cc"), CXX=host_env.get("HOST_CXX", "c++"))
        run(["bash", "bootstrap.sh", "--with-toolset=gcc"], cwd=source_root, env=host_env)
    configuration = cache / "boost-user-config.jam"
    configuration.write_text('using gcc : nuwax : "' + str(compiler) + '" ;\n', encoding="utf-8")
    run([str(source_root / "b2"), "--user-config=" + str(configuration), "toolset=gcc-nuwax", "target-os=linux",
         "address-model=64", "link=shared", "runtime-link=shared", "threading=multi", "variant=release", "cxxstd=17",
         "--layout=system", "--prefix=" + str(prefix), "--build-dir=" + str(cache / "build/boost"), "-j" + str(jobs),
         "--with-serialization", "--with-filesystem", "--with-thread", "--with-system", "--with-chrono", "--with-atomic", "--with-date_time", "install"], cwd=source_root)
    return source_root


def _build_directory(cache, name, source):
    # CMake binds its cache to one source directory; retain matching legacy builds.
    legacy = cache / "build" / name
    try:
        lines = (legacy / "CMakeCache.txt").read_text().splitlines()
    except FileNotFoundError:
        lines = ()
    for line in lines:
        if line.startswith("CMAKE_HOME_DIRECTORY:INTERNAL="):
            if Path(line.split("=", 1)[1]).resolve() == source.resolve():
                return legacy
            break
    return cache / "build" / source.name


def build_dependencies(prefix, cache, *, jobs=None, toolchain=None, run=None):
    prefix, cache = Path(prefix).absolute(), Path(cache).absolute()
    jobs = resolve_build_jobs(cache, jobs)
    print(f"二维依赖构建并行预算：{jobs}（CMake 与 Boost）", flush=True)
    if prefix.is_symlink() or cache.is_symlink():
        raise ValueError("二维依赖目录不能是符号链接")
    if run is None:
        def run(argv, **kwargs):
            subprocess.run(argv, check=True, **kwargs)
    common = ["-DCMAKE_BUILD_TYPE=Release", "-DCMAKE_INSTALL_PREFIX=" + str(prefix),
              "-DBUILD_SHARED_LIBS=ON", "-DCMAKE_INSTALL_RPATH=$ORIGIN", "-DCMAKE_BUILD_WITH_INSTALL_RPATH=ON"]
    if toolchain:
        common.append("-DCMAKE_TOOLCHAIN_FILE=" + str(Path(toolchain).resolve(strict=True)))
    source_roots = []
    for name, source in SOURCES.items():
        source_root = _checkout(cache, source, run, role=SOURCE_ROLES[name])
        source_roots.append(source_root)
        build_root = _build_directory(cache, name, source_root)
        options = ["-DTBB_TEST=OFF", "-DTBB_STRICT=OFF", "-DTBB_EXAMPLES=OFF"] if name == "oneTBB" else [
            "-DBUILD_TESTING=OFF", "-DBUILD_EXAMPLES=OFF", "-DBUILD_BENCHMARKS=OFF",
            "-DSUITESPARSE=OFF", "-DCXSPARSE=OFF", "-DEIGENSPARSE=ON", "-DUSE_CUDA=OFF"]
        run(["cmake", "-S", str(source_root), "-B", str(build_root), *common, *options,
             "-DCMAKE_PREFIX_PATH=" + str(prefix)])
        run(["cmake", "--build", str(build_root), "--parallel", str(jobs)])
        run(["cmake", "--install", str(build_root)])
        if name == "oneTBB":
            descriptions = list((build_root / "CMakeFiles").glob("*/CMakeCXXCompiler.cmake"))
            if len(descriptions) != 1:
                raise RuntimeError("二维依赖编译器身份不唯一")
            compiler = re.search(r'set\(CMAKE_CXX_COMPILER "([^"]+)"\)', descriptions[0].read_text()).group(1)
            source_roots.append(_boost(prefix, cache, compiler, jobs, run))
    notices = prefix / "share/nuwax-slam2d"
    notices.mkdir(parents=True, exist_ok=True)
    for index, root in enumerate(source_roots):
        for source in root.iterdir():
            if source.is_file() and source.name.upper().startswith(("LICENSE", "COPYING", "NOTICE")):
                shutil.copyfile(source, notices / (str(index) + "-" + source.name))
    (notices / "来源.json").write_text(json.dumps({"sources": SOURCES, "boost": {"url": BOOST_URL, "sha256": BOOST_SHA256}}, ensure_ascii=False, indent=2), encoding="utf-8")
    return prefix


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prefix", type=Path, required=True)
    parser.add_argument("--cache", type=Path, required=True)
    parser.add_argument("--toolchain", type=Path)
    parser.add_argument("--jobs", type=int)
    args = parser.parse_args()
    if args.jobs is not None and args.jobs < 1:
        parser.error("并行数必须为正整数")
    build_dependencies(args.prefix, args.cache, jobs=args.jobs, toolchain=args.toolchain)

if __name__ == "__main__":
    main()
