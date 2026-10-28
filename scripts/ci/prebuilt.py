#!/usr/bin/env python3
"""在两个目标环境逐程序编译、审计、封装并发布专用Release。"""
from __future__ import annotations

import argparse
import hashlib
import datetime
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys
import tempfile
import traceback

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from scripts import neh_prebuilt as common
from scripts.module_installer import builds


def run(argv, *, cwd=None, env=None):
    print("执行：" + " ".join(map(str, argv)), flush=True)
    subprocess.run(list(map(str, argv)), cwd=cwd, env={**os.environ, **(env or {})}, check=True)


def capture(argv, *, cwd=None, env=None):
    return subprocess.check_output(list(map(str, argv)), cwd=cwd, env={**os.environ, **(env or {}), "LC_ALL": "C"}, text=True).strip()


def dump(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, sort_keys=True, indent=2) + "\n", encoding="utf-8")


def probe(output):
    """在任何长编译前确认协议、包配置、目标库与OpenMP最终链接。"""
    generator = os.environ.get("LCM_GEN_EXECUTABLE") or shutil.which("lcm-gen")
    if not generator or capture([generator, "--version"]) != "lcm-gen 1.5.2":
        raise RuntimeError("PGO wire V3要求固定lcm-gen 1.5.2，请设置LCM_GEN_EXECUTABLE")
    with tempfile.TemporaryDirectory(prefix="neh-prebuilt-probe-") as directory:
        temporary = Path(directory)
        protocol = temporary / "probe.proto"
        protocol.write_text('syntax = "proto3"; message Probe { optional int32 number = 1; }\n', encoding="utf-8")
        run([os.environ.get("PROTOC", "protoc"), "--experimental_allow_proto3_optional", "--proto_path=" + str(temporary), "--descriptor_set_out=" + str(temporary / "probe.pb"), str(protocol)])
        idl = ROOT / common.WORKSPACE / "lcmtypes/nuwax_msgs"
        run([generator, "-x", "--cpp-std", "c++11", "--cpp-hpath", str(temporary), *[str(idl / (name + ".lcm")) for name in ("NativePgoRequest", "NativePgoResult", "NativePgoStatus")]])
        for name in ("NativePgoRequest", "NativePgoResult", "NativePgoStatus"):
            if not (temporary / "nuwax_msgs" / (name + ".hpp")).is_file():
                raise RuntimeError("PGO wire V3真实消息生成失败：" + name)
        for package in ("lcm", "glib-2.0", "eigen3"):
            run(["pkg-config", "--exists", package])
        (temporary / "probe.cpp").write_text('#include <lcm/lcm.h>\n#include <pcl/point_types.h>\n#include <omp.h>\n#include "nuwax_msgs/NativePgoRequest.hpp"\n#include "nuwax_msgs/NativePgoResult.hpp"\n#include "nuwax_msgs/NativePgoStatus.hpp"\nint main(){ lcm_t* l=lcm_create("memq://"); if(!l)return 1; int n=omp_get_max_threads(); lcm_destroy(l); return n<1 || nuwax_msgs::NativePgoRequest::getHash()==0 || nuwax_msgs::NativePgoResult::getHash()==0 || nuwax_msgs::NativePgoStatus::getHash()==0; }\n', encoding="utf-8")
        (temporary / "CMakeLists.txt").write_text('cmake_minimum_required(VERSION 3.16)\nproject(neh_probe LANGUAGES C CXX)\nset(CMAKE_CXX_STANDARD 17)\nfind_package(PkgConfig REQUIRED)\npkg_check_modules(LCM REQUIRED IMPORTED_TARGET lcm)\nfind_package(PCL REQUIRED COMPONENTS common filters registration)\nfind_package(OpenMP REQUIRED COMPONENTS CXX)\nadd_executable(probe probe.cpp)\ntarget_include_directories(probe PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${PCL_INCLUDE_DIRS})\ntarget_link_libraries(probe PRIVATE PkgConfig::LCM ${PCL_LIBRARIES} OpenMP::OpenMP_CXX)\n', encoding="utf-8")
        run(["cmake", "-S", str(temporary), "-B", str(temporary / "build"), "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"])
        run(["cmake", "--build", str(temporary / "build"), "--parallel", "2"])
        run([str(temporary / "build/probe")])
        if common.platform_key() == "focal-arm64":
            expected = {"lcm": "1.3.1", "glib-2.0": "2.64", "eigen3": "3.3.7"}
            for name, version in expected.items():
                if not capture(["pkg-config", "--modversion", name]).startswith(version):
                    raise RuntimeError("Go2系统库版本不符：" + name)
            if not capture(["dpkg-query", "-W", "-f=${Version}", "libpcl-dev"]).startswith("1.10."):
                raise RuntimeError("Go2要求PCL1.10")
            (temporary / "concept.cpp").write_text('#include <concepts>\ntemplate <std::integral T> T f(T x){return x;}\nint main(){return f(0);}\n', encoding="utf-8")
            run([os.environ.get("NUWAX_CXX20", "g++-10"), "-std=c++20", str(temporary / "concept.cpp"), "-o", str(temporary / "concept")])
            run([str(temporary / "concept")])
    dump(output, {"protocol": capture([os.environ.get("PROTOC", "protoc"), "--version"]), "lcm_generator": {"version": capture([generator, "--version"]), "sha256": common.sha256(Path(generator)), "wire_v3": "generated-and-linked"}, "packages": {name: capture(["pkg-config", "--modversion", name]) for name in ("lcm", "glib-2.0", "eigen3")}, "link": "passed"})


def enable_ci_native_diagnostics(cache):
    for name in ("CFLAGS", "CXXFLAGS"):
        flags = os.environ.get(name, "").strip()
        if not shlex.split(flags) or shlex.split(flags)[-1] != "-MD":
            os.environ[name] = (flags + " -MD").strip()
    os.environ["CC_ENABLE_DEBUG_OUTPUT"] = "1"
    temporary = cache / "compiler-tmp"
    if temporary.is_symlink():
        raise RuntimeError("CI编译临时目录不能为符号链接")
    temporary.mkdir(parents=True, exist_ok=True)
    os.environ["TMPDIR"] = str(temporary)
    real_cmake = Path(os.environ.get("NEH_CI_REAL_CMAKE") or shutil.which("cmake")).resolve(strict=True)
    version = re.search(r"cmake version (\d+)\.(\d+)", capture([str(real_cmake), "--version"]))
    if version is None:
        raise RuntimeError("无法确认真实CMake版本以选择编译器依赖诊断")
    os.environ["NEH_CI_REAL_CMAKE"] = str(real_cmake)
    os.environ["NEH_CI_LEGACY_CMAKE"] = "1" if tuple(map(int, version.groups())) < (3, 20) else "0"
    wrapper = cache / "audit-tools/cmake"
    wrapper.parent.mkdir(parents=True, exist_ok=True)
    command = [sys.executable, str(Path(__file__).resolve()), "native-cmake", "--cmake", str(real_cmake), "--cache", str(cache), "--"]
    wrapper.write_text("#!/bin/sh\nexec " + " ".join(shlex.quote(value) for value in command) + ' "$@"\n')
    wrapper.chmod(0o755)
    os.environ["PATH"] = str(wrapper.parent) + os.pathsep + os.environ["PATH"]


def cmake_cache_values(path):
    return dict(re.findall(r"^([^/#:\r\n][^:=\r\n]*):[^=\r\n]+=(.*)$", path.read_text(), re.MULTILINE))


def legacy_cmake_dependencies(values):
    return os.environ.get("NEH_CI_LEGACY_CMAKE") == "1" or values.get("CMAKE_DEPENDS_USE_COMPILER", "").upper() in ("FALSE", "OFF", "0", "NO")


def native_cmake(args):
    arguments = args.arguments[1:] if args.arguments[:1] == ["--"] else list(args.arguments)
    special = ("--build", "--install", "--open", "--find-package", "-E", "-P", "--version", "--help", "-h")
    if any(value in special or value.startswith("--help-") for value in arguments):
        return subprocess.run([args.cmake, *arguments]).returncode
    directory = next((arguments[index + 1] for index, value in enumerate(arguments[:-1]) if value == "-B"), None)
    directory = directory or next((value[2:] for value in arguments if value.startswith("-B") and len(value) > 2), None)
    build = (Path.cwd() / directory).resolve() if directory else Path.cwd().resolve()
    cache_file = build / "CMakeCache.txt"
    changes = {}
    if args.cache.resolve() in build.parents and cache_file.is_file():
        values = cmake_cache_values(cache_file)
        if legacy_cmake_dependencies(values):
            flattened = [value for value in arguments if value != "-D"]
            explicit = dict(re.findall(r"^-?D?(CMAKE_(?:C|CXX)_FLAGS)(?::[^=]+)?=(.*)$", "\n".join(flattened), re.MULTILINE))
            for name in ("CMAKE_C_FLAGS", "CMAKE_CXX_FLAGS"):
                before = explicit.get(name, values.get(name))
                if before is not None and "-MD" not in shlex.split(before):
                    after = (before + " -MD").strip()
                    arguments.append("-D" + name + ":STRING=" + after)
                    changes[name] = {"before": before, "after": after}
    status = subprocess.run([args.cmake, *arguments]).returncode
    if args.cache.resolve() in build.parents and cache_file.is_file():
        dump(args.cache / "audit-cmake-configs" / (hashlib.sha256(str(build).encode()).hexdigest() + ".json"), {"build_directory": str(build), "working_directory": str(Path.cwd()), "arguments": [args.cmake, *arguments], "diagnostic_changes": changes, "success": status == 0})
    return status


def migrate_cmake_diagnostics(program, cache, spec):
    target = builds._rust_target(cache, ROOT, common.PROGRAMS[program]["manifest"])
    result = []
    for package_directory in sorted((target / "release/build").glob("*")):
        if not package_directory.is_dir():
            continue
        name = package_directory.name.rsplit("-", 1)[0]
        packages = [item for item in spec["cargo_packages"] if item["name"] == name]
        if not packages:
            continue
        out = package_directory / "out"
        pending = []
        for record in sorted(out.rglob("CMakeCache.txt")):
            if any((parent / "CMakeCache.txt").is_file() for parent in record.parent.parents if out == parent or out in parent.parents):
                continue
            values = cmake_cache_values(record)
            missing = {key: values[key] for key in ("CMAKE_C_FLAGS", "CMAKE_CXX_FLAGS") if key in values and "-MD" not in shlex.split(values[key])}
            if not legacy_cmake_dependencies(values) or not missing:
                continue
            home = Path(values.get("CMAKE_HOME_DIRECTORY", "/nonexistent-cmake-source")).resolve()
            owners = [item for item in packages if Path(item["manifest_path"]).parent.resolve() == home or Path(item["manifest_path"]).parent.resolve() in home.parents]
            if not owners:
                continue
            if len(owners) != 1 or not re.fullmatch(re.escape(name) + r"-[0-9a-f]+", package_directory.name) or record.is_symlink() or target.resolve() not in record.resolve().parents:
                raise RuntimeError("旧CMake诊断迁移的Cargo包/构建路径无法唯一归属：" + str(record))
            pending.append({"package_id": owners[0]["id"], "manifest_path": owners[0]["manifest_path"], "cmake_cache": str(record), "source_directory": str(home), "original_flags": missing})
        if pending:
            output = package_directory / "output"
            if output.is_symlink():
                raise RuntimeError("旧CMake诊断迁移拒绝符号链接output")
            receipt = {"program": program, "basis": "current-package-legacy-cmake-cache-lacks-compiler-MD", "configurations": pending, "invalidated_record": str(output), "original_sha256": common.sha256(output) if output.is_file() else None, "action": "remove-single-build-script-output" if output.is_file() else "already-missing-build-script-output"}
            dump(cache / "audit-cmake-migrations" / (hashlib.sha256(str(package_directory).encode()).hexdigest() + ".json"), receipt)
            output.unlink(missing_ok=True)
            result.append(receipt)
    return result


def identity(platform_key, root):
    cargo = Path(shutil.which("cargo") or "cargo")
    rustc = Path(shutil.which("rustc") or "rustc")
    if not capture([rustc, "--version"]).startswith("rustc " + builds.RUST_VERSION + " "):
        raise RuntimeError("云端工具链必须使用Rust " + builds.RUST_VERSION)
    base_image = os.environ.get("NEH_BASE_IMAGE", "")
    if not re.fullmatch(r"ubuntu:(20\.04|24\.04)@sha256:[0-9a-f]{64}", base_image):
        raise RuntimeError("构建身份缺少实际基础镜像摘要")
    return cargo, {
        "rustc": capture([rustc, "-vV"]),
        "cc": capture(["cc", "--version"]).splitlines()[0],
        "cxx": capture(["c++", "--version"]).splitlines()[0],
        "cxx20": capture([os.environ.get("NUWAX_CXX20", "c++"), "--version"]).splitlines()[0],
        "cmake": capture(["cmake", "--version"]).splitlines()[0],
        "protoc": capture([os.environ.get("PROTOC", "protoc"), "--version"]),
        "lcm_generator": {"version": capture([os.environ["LCM_GEN_EXECUTABLE"], "--version"]), "sha256": common.sha256(Path(os.environ["LCM_GEN_EXECUTABLE"]))},
        "system_packages": capture(["dpkg-query", "-W", "-f=${binary:Package}=${Version}\n"]),
        "image": {"base": base_image, "dockerfile_sha256": common.sha256(root / "scripts/ci/prebuilt" / (platform_key + ".Dockerfile"))},
        # 不含路径和并行数；它们不改变代码语义，源码内优化配置由Git树覆盖。
        "parameters": {"cargo": ["rustc", "--locked", "--release", "-vv", "--message-format=json-render-diagnostics", "--print=link-args", "-Clink-arg=-Wl,-Map=$CACHE/compiler-tmp/linkmaps/<program>.map"], "native_diagnostics": {"dependency": "-MD", "environment_audit": "registered-roots-and-components-v1", "temporary_directory": "$CACHE/compiler-tmp", "legacy_cmake_cached_flags": "preserve-and-append-MD-v1", "cargo_migration": "current-package-build-script-output-v1"}, "cmake": ["CMAKE_BUILD_TYPE=Release", "CMAKE_GENERATOR=Unix Makefiles"], "environment": {name: os.environ.get(name, "") for name in ("RUSTFLAGS", "CFLAGS", "CXXFLAGS", "LDFLAGS", "NUWAX_CXX20", "CC_ENABLE_DEBUG_OUTPUT")}},
    }


def asset_entry(asset):
    name = asset["name"]
    digest = asset.get("digest") or ""
    if not digest.startswith("sha256:") or not re.fullmatch(r"[0-9a-f]{64}", digest[7:]):
        return None
    return {"asset": name, "sha256": digest[7:], "size": asset["size"]}


def private_libraries(program, prefix, cache):
    """只封装私有动态库；目标系统ABI由平台系统包与候选验证负责。"""
    result, pending = {}, [prefix / "bin" / program]
    if program == "nuwax-slam-2d-worker":
        pending.append(prefix / "lib/libnuwax_slamtec.so.1")
    seen = set()
    while pending:
        binary = pending.pop()
        resolved = binary.resolve(strict=True)
        if resolved in seen:
            continue
        seen.add(resolved)
        if binary.name != program:
            result[binary.name] = resolved
        listing = capture(["ldd", str(resolved)])
        if "not found" in listing:
            raise RuntimeError("私有动态依赖无法解析：" + listing)
        for name, path in re.findall(r"^\s*(\S+)\s+=>\s+(/\S+)", listing, flags=re.MULTILINE):
            selected = Path(path).resolve(strict=True)
            if prefix in selected.parents or cache in selected.parents:
                if name not in result:
                    result[name] = selected
                    pending.append(selected)
    return result


def copy_notices(root, prefix, cache, staging, spec, audit):
    """按实际组件收集声明；不再沿每个编译头文件追溯许可。"""
    notice_base = common.program_share(audit["program"]) + "/licenses"
    selected = {root / "LICENSE", root / "docs/公开文档/第三方许可证说明.md"}
    for component in audit["external_sources"]:
        directories = component.get("notice_roots", [component["root"]])
        for directory in map(Path, directories):
            if directory.is_dir():
                candidates = directory.rglob("*") if component["kind"] in ("crate", "repository-embedded", "embedded-component") else directory.iterdir()
                selected.update(path for path in candidates if path.is_file() and path.name.upper().startswith(("LICENSE", "LICENCE", "COPYING", "NOTICE", "THIRD_PARTY_NOTICES")))
        selected.update(Path(value) for value in component.get("notice_files", []))
        if component["kind"] == "rust-toolchain":
            directory = Path(component["root"]) / "share/doc/rust"
            selected.add(directory / "COPYRIGHT-library.html")
            selected.update(path for path in (directory / "licenses").glob("*") if path.is_file())
    # 系统组件由实际链接路径确定；header-only组件只按已知根归类一次。
    system_paths = {str(Path(item["path"]).resolve()) for item in audit["link_inputs"] if item["path"].startswith(("/usr/", "/lib/"))}
    header_components = {name: Path(directory) for name, directory in (("Eigen3", "/usr/include/eigen3"), ("Boost-system-headers", "/usr/include/boost")) if any(Path(directory) == Path(value) or Path(directory) in Path(value).parents for value in audit["external_inputs"])}
    system_paths.update(str(directory) for directory in header_components.values())
    system_paths.update(value[4:] for value in list(system_paths) if value.startswith("/usr/lib/"))
    packages, owners = set(), {}
    paths = sorted(system_paths)
    for offset in range(0, len(paths), 200):
        query = subprocess.run(["dpkg-query", "-S", *paths[offset:offset + 200]], text=True, capture_output=True)
        for line in query.stdout.splitlines():
            if ": /" in line:
                owner, path = line.split(": /", 1)
                names = set(owner.split(", "))
                packages.update(names)
                owners.setdefault("/" + path, set()).update(names)
    audit["system_packages_used"] = capture(["dpkg-query", "-W", "-f=${binary:Package}=${Version}\n", *sorted(packages)]).splitlines() if packages else []
    component_index = {item["kind"] + ":" + item["id"]: item for item in audit["external_sources"]}
    for name, directory in header_components.items():
        component_index["header-only-component:" + name] = {"kind": "header-only-component", "id": name, "root": str(directory), "package_owners": sorted(owners.get(str(directory), [])), "usage": ["compile-environment"]}
    for package in audit["system_packages_used"]:
        name, version = package.split("=", 1)
        directory = Path("/usr/share/doc") / name.split(":", 1)[0]
        selected.add(directory / "copyright")
        component_index["system-package:" + name] = {"kind": "system-package", "id": name, "version": version, "root": str(directory)}
    audit["external_sources"] = sorted(component_index.values(), key=lambda item: (item["kind"], item["id"]))
    for name in ("GPL-2", "GPL-3", "LGPL-2.1", "LGPL-3", "Apache-2.0", "BSD", "MPL-2.0", "BSL-1.0"):
        selected.add(Path("/usr/share/common-licenses") / name)
    if not (root / "LICENSE").is_file():
        raise RuntimeError("发布程序缺少项目LICENSE")
    files = []
    for index, source in enumerate(sorted(path for path in selected if path.is_file())):
        relative = notice_base + "/" + str(index) + "/" + source.name
        destination = staging / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, destination)
        files.append({"path": relative, "source": canonical(source, root, cache), "sha256": common.sha256(source), "scope": "project-overview" if source.name == "第三方许可证说明.md" else "common-license-text" if source.parent == Path("/usr/share/common-licenses") else "component-declaration"})
    audit["license_files"] = files
    return len(files)


def corresponding_sources(root, program, spec, audit, revision, repository):
    source_url = repository[:-4] if repository.endswith(".git") else repository.rstrip("/")
    upstreams = []
    actual_ids = {source["id"] for source in audit["external_sources"]}
    for source in audit["external_sources"]:
        if source["kind"] == "pinned-source":
            upstreams.append({key: source[key] for key in ("id", "repository", "revision", "url", "sha256", "recipe") if key in source})
    crates = [{key: item[key] for key in ("id", "name", "version", "source", "license")} for item in spec["cargo_packages"] if item["source"] and item["id"] in actual_ids]
    manifest = common.PROGRAMS[program]["manifest"]
    cargo_lock = str(Path(manifest).with_name("Cargo.lock"))
    return {
        "project": {"repository": repository, "revision": revision, "source_archive": source_url + "/archive/" + revision + ".tar.gz"},
        "actual_upstreams": upstreams,
        "actual_external_crates": crates,
        "component_declarations": [{key: value for key, value in item.items() if key not in ("root", "notice_roots", "notice_files")} for item in audit["external_sources"]],
        "cargo": {"manifest": manifest, "lockfile": cargo_lock, "lockfile_sha256": common.sha256(root / cargo_lock), "retrieve": "cargo fetch --locked --manifest-path " + manifest, "说明": "依照此固定提交的Cargo.lock重新取得实际crate和Git依赖；保留其中的固定版本、提交与checksum，不能换成未锁定版本。"},
        "rebuild": {"entry": "scripts/module_installer/builds.py:build_modules", "module": common.PROGRAMS[program]["module"], "programs": [program], "local_build": True, "recipe": "scripts/ci/prebuilt.py", "container": "scripts/ci/prebuilt/" + audit.get("platform", "目标平台") + ".Dockerfile", "rust": builds.RUST_VERSION, "input_spec": {key: spec[key] for key in ("paths", "exclude")}, "说明": "取得固定项目源码后，按镜像配方准备编译器与系统开发库；以独立prefix/cache调用build_modules，local_build=True且programs只选本程序。构建脚本中固定的上游Git提交、归档SHA256、补丁和构建参数均属于对应源码，不得省略。"},
        "GPL3完整源码与替换说明": [
            "许可组合以逐程序批准清单license_expression为准；此处列出实际固定上游与重建配方，不将Cephes、METIS/GKlib或其它依赖统一归为BSD。原文件中的MIT、Boost、Apache、LGPL等声明以及包内全文须一并保留。",
            "本程序对应源码包括固定项目提交中的Rust/C/C++源码、协议与contracts、Cargo.lock、实际上游源码、补丁及构建脚本。按上述固定URL/提交/SHA和锁文件取得全部材料后，可在相同目标平台完整重建此GPL3组合程序。",
            "修改静态链接的LGPL组件时，应使用修改后的组件源码重新执行完整构建和最终链接；不能只替换静态归档而保留旧可执行程序。GTSAM内部的Cephes、METIS/GKlib仍按各自原文件许可保留，不会因组合发布而抹去声明。",
            "修改动态链接的LGPL组件时，可重新构建相应私有共享库并替换候选版本lib中的对应文件；需要改接口或绑定时同时重建本程序。包内二维原生源码、实际审计中的native_link_records、构建配方及固定依赖来源提供重链接依据。",
            "将重新构建的bin与私有lib组装到独立候选版本，通过原管理流程验证动态依赖与健康状态后切换；本说明不要求直接改写正在运行的版本。",
        ],
    }


def package(root, prefix, cache, output, program, fingerprint_value, spec, audit):
    if program not in common.PROGRAMS:
        raise RuntimeError("禁止打包仅限本机源码构建的程序")
    if audit["program"] != program:
        raise RuntimeError("程序打包与实际审计身份不一致")
    with tempfile.TemporaryDirectory(prefix=".package-", dir=output) as directory:
        staging = Path(directory)
        (staging / "bin").mkdir()
        binary = staging / "bin" / program
        shutil.copyfile(prefix / "bin" / program, binary)
        binary.chmod(0o755)
        libraries = private_libraries(program, prefix, cache)
        for name, source in libraries.items():
            destination = staging / "lib" / name
            destination.parent.mkdir(exist_ok=True)
            shutil.copyfile(source, destination)
            destination.chmod(0o644)
            run(["patchelf", "--set-rpath", "$ORIGIN", str(destination)])
        run(["patchelf", "--set-rpath", "$ORIGIN/../lib:$ORIGIN/../lib64", str(binary)])
        notices = copy_notices(root, prefix, cache, staging, spec, audit)
        revision = capture(["git", "-C", str(root), "rev-parse", "HEAD"])
        repository = capture(["git", "-C", str(root), "remote", "get-url", "origin"])
        materials = Path(common.program_share(program))
        sources = materials / "sources"
        dump(staging / sources / "对应源码.json", corresponding_sources(root, program, spec, audit, revision, repository))
        # LGPL组件源码及替换共享库所需的包装源码随程序一起提供。
        if program == "nuwax-slam-2d-worker":
            for path in ("services/nuwax-workspace/crates/nuwax-slam-toolbox-sys/native", "drivers/sensors/slamtec_native"):
                shutil.copytree(root / path, staging / sources / path)
        audit["relink_materials"] = {"sources": sources.as_posix(), "corresponding_source": (sources / "对应源码.json").as_posix(), "private_libraries": sorted(libraries), "build_recipe": "scripts/module_installer/builds.py"}
        audit["review_key"] = review_key(audit, root, cache)
        dump(staging / materials / "audit/inputs.json", audit)
        files = [path.relative_to(staging).as_posix() for path in staging.rglob("*") if path.is_file()]
        manifest = {"schema": common.SCHEMA, "program": program, "fingerprint": fingerprint_value, "files": common._receipt_files(staging, files), "license_files": notices}
        dump(staging / "prebuilt-manifest.json", manifest)
        archive = output / (program + "-" + fingerprint_value + ".tar.zst")
        # 名称、时间和属主固定；相同输入重建的封装层不会引入随机元数据。
        raw_tar = staging.parent / (program + ".tar")
        try:
            run(["tar", "--sort=name", "--mtime=@0", "--owner=0", "--group=0", "--numeric-owner", "-C", str(staging), "-cf", str(raw_tar), *sorted(path.name for path in staging.iterdir())])
            run(["zstd", "-q", "-T2", "-8", str(raw_tar), "-o", str(archive)])
        finally:
            raw_tar.unlink(missing_ok=True)
    return {"asset": archive.name, "sha256": common.sha256(archive), "size": archive.stat().st_size}


def rust_target(program, cache):
    manifest = common.PROGRAMS[program]["manifest"]
    slots = json.loads((cache / "build/rust/target-paths-v1.json").read_text())["slots"]
    return cache / "build/rust" / slots[builds._TARGET_ROLES[manifest]]["path"]


def cmake_link_commands(text):
    commands = []
    for line in text.splitlines():
        lexer = shlex.shlex(line, posix=True, punctuation_chars=";&|")
        lexer.whitespace_split = True
        lexer.commenters = ""
        current = []
        for token in lexer:
            if token in ("&&", ";", "||"):
                if current:
                    commands.append(current)
                current = []
            else:
                current.append(token)
        if current:
            commands.append(current)
    return commands


def native_command_output(arguments):
    tool = Path(arguments[0]).name
    if tool in ("cmake", "cmake3") and arguments[1:3] == ["-E", "env"]:
        position = next((index for index in range(3, len(arguments)) if "=" not in arguments[index]), None)
        if position is None:
            return None
        return native_command_output(arguments[position:])
    if tool in ("cmake", "cmake3") or "ranlib" in tool:
        return None
    if re.search(r"(?:^|[-_])ar(?:-[0-9.]+)?$", tool):
        operation = next((index for index, value in enumerate(arguments[1:], 1) if re.fullmatch(r"-?[abcdDfilmMpNoOPqrstxSTuvV]+", value) and any(letter in value for letter in "dmpqrstx")), None)
        if operation is None or operation + 1 >= len(arguments):
            return None
        return {"archive": True, "output": arguments[operation + 1], "object_arguments": arguments[operation + 2:], "arguments": arguments}
    if "-o" in arguments:
        index = arguments.index("-o")
        if index + 1 < len(arguments):
            return {"archive": False, "output": arguments[index + 1], "arguments": arguments}
    combined = next((value[2:] for value in arguments if value.startswith("-o") and len(value) > 2), None)
    if combined:
        return {"archive": False, "output": combined, "arguments": arguments}
    return None


def native_link_evidence(program, cache, spec, linked, actual_inputs):
    roots = [Path(event["out_dir"]) for event in cargo_build_scripts(spec, linked)]
    if program == "nuwax-slam-2d-worker":
        roots.extend((cache / "build/slamtec", cache / "slam2d-dependencies/build"))
    private = private_libraries(program, cache / "prefix", cache)
    private_ids = {}
    for name, library in private.items():
        private_ids[name] = re.findall(r"Build ID: (\S+)", capture(["readelf", "--notes", str(library)]))
        private_ids[library.name] = private_ids[name]
    needed = {str(Path(item["path"]).resolve()) for item in resolve_link_inputs(linked["linker_arguments"], ROOT)}
    needed.update(str(Path(value).resolve()) for value in actual_inputs if Path(value).is_absolute())
    candidates = [(path, index, command) for path in sorted({path for directory in roots for path in directory.rglob("link.txt")}) for index, command in enumerate(cmake_link_commands(path.read_text()))]
    result, visited = [], set()
    while True:
        changed = False
        for path, command_index, arguments in candidates:
            if (path, command_index) in visited:
                continue
            command = shlex.join(arguments)
            cmake_root = next((parent for parent in path.parents if (parent / "CMakeCache.txt").is_file()), None)
            if cmake_root is None or path.parent.parent.name != "CMakeFiles":
                raise RuntimeError("原生链接记录缺少CMake工作目录或构建根：" + str(path))
            cwd = path.parents[2]
            parsed_command = native_command_output(arguments)
            if parsed_command is None:
                if any(str(value) in needed or Path(value).name in private_ids for value in arguments) and not (Path(arguments[0]).name in ("cmake", "cmake3") or "ranlib" in Path(arguments[0]).name):
                    raise RuntimeError("实际采用的原生链接命令无法解析：" + command)
                continue
            archive = parsed_command["archive"]
            arguments = parsed_command["arguments"]
            destination = (cwd / parsed_command["output"]).resolve()
            matches = str(destination) in needed
            if not matches and destination.name in private_ids and destination.is_file():
                build_ids = re.findall(r"Build ID: (\S+)", capture(["readelf", "--notes", str(destination)]))
                matches = bool(build_ids and build_ids == private_ids[destination.name])
            if not matches:
                continue
            visited.add((path, command_index))
            if archive:
                parsed = [{"kind": "object", "path": str((cwd / value).resolve()), "exists_after_link": (cwd / value).is_file()} for value in parsed_command["object_arguments"] if value.endswith(".o")]
                if not parsed:
                    raise RuntimeError("静态归档链接记录无法解析：" + str(path))
            else:
                parsed = resolve_link_inputs(arguments, cwd)
            needed.update(str(Path(item["path"]).resolve()) for item in parsed)
            result.append({"record": str(path), "command": command, "output": str(destination), "inputs": parsed, "cmake_root": str(cmake_root), "working_directory": str(cwd), "command_index": command_index})
            changed = True
        if not changed:
            break
    return result, private


def object_dependency(path):
    for candidate in (Path(str(path) + ".d"), path.with_suffix(".d")):
        if candidate.is_file():
            return candidate
    raise RuntimeError("实际采用的原生对象缺少编译器依赖记录（object.d/object.o.d）：" + str(path))


def diagnostic_command(argv, link_map):
    arguments = list(map(str, argv))
    if len(arguments) > 1 and arguments[1] == "rustc":
        arguments.insert(arguments.index("--"), "--message-format=json-render-diagnostics")
        arguments.extend(("-C", "link-arg=-Wl,-Map=" + str(link_map)))
    return arguments


def cargo_build_scripts(spec, linked):
    """Cargo每次都会报告实际OUT_DIR，fresh也不能靠名称扫描其它版本。"""
    packages = {item["id"]: item for item in spec["cargo_packages"]}
    result = []
    for event in linked.get("build_script_events", []):
        package = packages.get(event["package_id"])
        if package is None:
            raise RuntimeError("本次构建脚本不属于固定Cargo闭包：" + event["package_id"])
        out = Path(event["out_dir"]).resolve()
        if not out.is_dir():
            raise RuntimeError("实际Cargo OUT_DIR缺失：" + str(out))
        result.append({"package": package, "out_dir": str(out), "linked_libs": event["linked_libs"], "linked_paths": event["linked_paths"]})
    return result


def declared_static_archives(spec, linked):
    result = []
    for event in cargo_build_scripts(spec, linked):
        roots = [Path(value.split("=", 1)[-1]) for value in event["linked_paths"]]
        for value in event["linked_libs"]:
            if not value.startswith("static=") and not value.startswith("static:"):
                continue
            name = value.rsplit("=", 1)[-1]
            path = next((root / ("lib" + name + ".a") for root in roots if (root / ("lib" + name + ".a")).is_file()), None)
            if path is None:
                found = capture(["cc", "-print-file-name=lib" + name + ".a"])
                path = Path(found) if Path(found).is_absolute() and Path(found).is_file() else None
            if path is None:
                raise RuntimeError("实际Cargo静态链接声明无法定位归档：" + value)
            result.append({"path": str(path.resolve()), "package_id": event["package"]["id"]})
    return sorted({(item["path"], item["package_id"]): item for item in result}.values(), key=lambda item: (item["path"], item["package_id"]))


def classify_native_usage(program, cache, spec, linked, records, private):
    """成员名只用于组件级采用证据，不追逐单成员字节或符号偏移。"""
    artifact_packages = {Path(path).name: item["package_id"] for item in linked["compiler_artifacts"] for path in item["filenames"]}
    by_package = {}
    static = {item["path"]: item["package_id"] for item in declared_static_archives(spec, linked)}
    for archive, member in re.findall(r"([^\s()]+\.(?:rlib|a))\(([^()\n]+)\)", Path(linked["link_map"]).read_text()):
        package = artifact_packages.get(Path(archive).name) or static.get(str(Path(archive).resolve()))
        if package and member.endswith(".o") and not member.endswith(".rcgu.o"):
            by_package.setdefault(package, set()).add(member)
    linked["map_native_packages"] = sorted(by_package)
    runtime = {str(path.resolve()) for path in private.values()}
    binary = None
    for record in records:
        path = Path(record["output"])
        record["usage"] = "build-component"
        package = static.get(str(path))
        if package in by_package:
            members = set(capture(["ar", "t", str(path)]).splitlines())
            matched = sorted(members & by_package[package])
            if matched:
                record.update(usage="linked-native", map_member_names=matched, package_id=package)
                runtime.add(str(path))
        if path.name in {value.name for value in private.values()}:
            runtime.add(str(path))
        elif path.is_file() and path.suffix not in (".a", ".so") and ".so." not in path.name and os.access(path, os.X_OK):
            data = path.read_bytes()
            if data.startswith(b"\x7fELF"):
                if binary is None:
                    binary = (cache / "prefix/bin" / program).read_bytes()
                if data in binary:
                    record["usage"] = "embedded-runtime"
                    runtime.add(str(path))
    changed = True
    while changed:
        changed = False
        for record in records:
            if record["output"] in runtime:
                if record["usage"] == "build-component":
                    record["usage"] = "runtime-target-link"
                for item in record["inputs"]:
                    if item["path"] not in runtime:
                        runtime.add(item["path"])
                        changed = True


def dependency_records(program, cache, spec, linked):
    target = rust_target(program, cache)
    records = [target / "release" / (program + ".d")]
    record_cwds = {}
    rust_records = {}
    packages = {item["id"]: item for item in spec["cargo_packages"]}
    artifacts = linked.get("compiler_artifacts", [])
    if not artifacts:
        raise RuntimeError("缺少本次Cargo精确产物事件，拒绝扫描共享target推断版本")
    for artifact in artifacts:
        package = packages.get(artifact["package_id"])
        if package is None or Path(artifact["manifest_path"]).resolve() != Path(package["manifest_path"]).resolve():
            raise RuntimeError("本次Cargo产物不属于当前固定依赖闭包：" + artifact["package_id"])
        package_root = Path(package["manifest_path"]).parent.resolve()
        crate = artifact["target"]["name"].replace("-", "_")
        candidates = set()
        for filename in artifact["filenames"]:
            path = Path(filename)
            name = path.name
            if name.startswith("lib") and path.suffix in (".rlib", ".rmeta", ".so", ".a"):
                candidates.add(path.with_name(name[3:]).with_suffix(".d"))
            elif "custom-build" in artifact["target"]["kind"]:
                candidates.update(path.parent.glob(crate + "-*.d"))
                candidates.add(path.parent / (crate + ".d"))
            else:
                candidates.add(path.with_name(name + ".d"))
        candidates = {path for path in candidates if path.is_file()}
        if not candidates:
            raise RuntimeError("本次Cargo精确产物缺少对应Rust依赖记录：" + artifact["package_id"])
        for record in candidates:
            rust_records.setdefault(record, set()).add(package_root)
    rust_origins = []
    for record, package_roots in sorted(rust_records.items()):
        dependencies = common.dependency_paths(record)
        relative = [value for value in dependencies if not Path(value).is_absolute()]
        if relative:
            roots = package_roots | {ROOT.resolve(), (ROOT / common.WORKSPACE).resolve(), (ROOT / common.PROGRAMS[program]["manifest"]).parent.resolve()}
            possibilities = {}
            for base in roots:
                resolved = tuple(str((base / value).resolve()) for value in relative)
                if all(Path(value).exists() for value in resolved):
                    possibilities.setdefault(resolved, base)
            if len(possibilities) > 1:
                # 新执行的真实Rustc命令可用extra-filename与源码参数消除同名包歧义。
                suffix = record.stem.rsplit("-", 1)[-1]
                hinted = {}
                for command in linked["compiler_commands"]:
                    if "extra-filename=-" + suffix not in command or "Running `" not in command:
                        continue
                    arguments = shlex.split(command.split("Running `", 1)[1].rsplit("`", 1)[0])
                    manifest_dir = next((Path(value.split("=", 1)[1]).resolve() for value in arguments if value.startswith("CARGO_MANIFEST_DIR=")), None)
                    source = next((Path(value) for value in arguments if value.endswith(".rs") and not value.startswith("-") and "=" not in value), None)
                    if source is None or manifest_dir is None:
                        continue
                    for resolved, base in possibilities.items():
                        actual_source = source.resolve() if source.is_absolute() else (base / source).resolve()
                        if str(actual_source) in resolved and (actual_source == manifest_dir or manifest_dir in actual_source.parents):
                            hinted[resolved] = base
                if hinted:
                    possibilities = hinted
            if len(possibilities) != 1:
                raise RuntimeError("Rust依赖记录无法唯一归属已知crate/workspace/repo工作目录，拒绝静默丢弃：" + str(record))
            base = next(iter(possibilities.values()))
            record_cwds[str(record.resolve())] = str(base)
            rust_origins.append({"record": str(record), "working_directory": str(base), "basis": "unique-known-cwd-or-record-matched-rustc-source"})
        else:
            rust_origins.append({"record": str(record), "basis": "absolute-dependency-paths"})
        records.append(record)
    linked["rust_dependency_record_origins"] = rust_origins
    actual = common.dependency_paths(records[0])
    actual.extend(item["path"] for item in declared_static_archives(spec, linked))
    native, private = native_link_evidence(program, cache, spec, linked, actual)
    classify_native_usage(program, cache, spec, linked, native, private)
    native_objects = []
    for entry in native:
        records.extend((Path(entry["cmake_root"]) / "CMakeFiles").rglob("Makefile.cmake"))
        for item in entry["inputs"]:
            if item["kind"] == "object":
                obj = Path(item["path"])
                record = object_dependency(obj)
                records.append(record)
                record_cwds[str(record.resolve())] = entry["working_directory"]
                native_objects.append({"object": str(obj), "dependency_record": str(record), "working_directory": entry["working_directory"], "backend": "cmake-link-record", "usage": entry["usage"]})
    for event in cargo_build_scripts(spec, linked):
        out = Path(event["out_dir"])
        for obj in out.rglob("*.o"):
            # CMake对象由实际link.txt选择；其配置探针不属于cc::Build对象集合。
            if any((parent / "CMakeCache.txt").is_file() for parent in obj.parents if parent == out or out in parent.parents):
                continue
            record = object_dependency(obj)
            records.append(record)
            cwd = str(Path(event["package"]["manifest_path"]).parent)
            record_cwds[str(record.resolve())] = cwd
            native_objects.append({"object": str(obj), "dependency_record": str(record), "working_directory": cwd, "package_id": event["package"]["id"], "backend": "cargo-build-script"})
    linked["native_link_records"] = native
    linked["native_compilation_records"] = native_objects
    return sorted(set(records)), record_cwds


def reject_lio_assets(items):
    for item in items:
        name = item["name"] if isinstance(item, dict) else str(item)
        if re.search(r"(?:fastlio2|pointlio)[_-]native", name) or name.startswith(("source-fastlio2-", "source-pointlio-")):
            raise RuntimeError("云端禁止编译、打包或发布LIO附件：" + name)
        if name.endswith((".tar.zst", ".tar.gz")) and not any(re.fullmatch(re.escape(program) + r"-[0-9a-f]{64}\.tar\.zst", name) for program in common.PROGRAMS):
            raise RuntimeError("专用预编译仓库只接受五程序指纹归档：" + name)


def canonical(value, root, cache):
    value = str(value).replace(str(root), "$SOURCE").replace(str(cache), "$CACHE")
    home = os.environ.get("RUSTUP_HOME")
    if home:
        value = value.replace(home, "$RUSTUP")
    value = re.sub(r"-[0-9a-f]{32}(?=/|$)", "-<slot>", value)
    value = re.sub(r"-[0-9a-f]{16}(?=[./]|$)", "-<artifact>", value)
    return value


def link_arguments(logged, program, binary, cache, link_map=None):
    candidates = []
    compiler = []
    artifacts, build_scripts = [], []
    for line in logged.splitlines():
        if line.startswith("{"):
            try:
                event = json.loads(line)
            except ValueError:
                event = {}
            if event.get("reason") == "compiler-artifact":
                artifacts.append(event)
            elif event.get("reason") == "build-script-executed":
                build_scripts.append(event)
        if line.startswith("LC_ALL=") and '"-o"' in line and program.replace("-", "_") in line:
            arguments = shlex.split(line)
            position = next((i for i, value in enumerate(arguments) if "=" not in value), None)
            if position is not None:
                candidates.append(arguments[position:])
        if "Running `" in line and "rustc " in line:
            compiler.append(line.strip())
    receipt = cache / "audit-links" / (program + ".json")
    digest = common.sha256(binary)
    if candidates:
        result = {"binary_sha256": digest, "linker_arguments": candidates[-1], "compiler_commands": compiler, "compiler_artifacts": artifacts}
        if link_map is not None:
            if not link_map.is_file():
                raise RuntimeError("实际最终链接没有产生GNU链接图：" + program)
            result.update(link_map=str(link_map), link_map_sha256=common.sha256(link_map))
        dump(receipt, result)
    else:
        try:
            result = json.loads(receipt.read_text())
        except (OSError, ValueError) as exc:
            raise RuntimeError("未捕获实际最终链接参数，且无有效链接回执：" + program) from exc
        if result.get("binary_sha256") != digest or not result.get("linker_arguments"):
            raise RuntimeError("最终链接回执与当前ELF不符：" + program)
    if link_map is not None and (result.get("link_map") != str(link_map) or not link_map.is_file() or result.get("link_map_sha256") != common.sha256(link_map)):
        raise RuntimeError("最终链接图回执与当前ELF/链接记录不符：" + program)
    if not artifacts:
        raise RuntimeError("本次Cargo未提供compiler-artifact事件，无法核对依赖产物")
    result["compiler_artifacts"] = artifacts
    result["build_script_events"] = build_scripts
    dump(receipt, result)
    return {**result, "captured_this_build": bool(candidates)}


def resolve_link_inputs(arguments, cwd):
    def expand(values, depth=0):
        if depth > 4:
            raise RuntimeError("链接响应文件递归层数过多")
        result = []
        for value in values:
            if value.startswith("@"):
                response = Path(value[1:])
                if not response.is_absolute():
                    response = cwd / response
                if not response.is_file() or response.stat().st_size > 4 * 1024 * 1024:
                    raise RuntimeError("实际链接响应文件缺失或过大，不能推断参数：" + str(response))
                result.extend(expand(shlex.split(response.read_text()), depth + 1))
            elif value.startswith("-Wl,"):
                result.extend(expand(value[4:].split(","), depth + 1))
            else:
                result.append(value)
        return result
    values = expand(arguments[1:])
    roots = []
    for index, value in enumerate(values):
        directory = values[index + 1] if value == "-L" and index + 1 < len(values) else value[2:] if value.startswith("-L") and len(value) > 2 else None
        if directory:
            path = Path(directory)
            roots.append(path if path.is_absolute() else cwd / path)
    result, static, index = [], False, 0
    skip_values = {"-o", "-L", "-soname", "--soname", "-rpath", "--rpath", "-rpath-link", "--rpath-link", "--dynamic-linker", "-dynamic-linker"}
    while index < len(values):
        value = values[index]
        index += 1
        if value in skip_values:
            if index >= len(values):
                raise RuntimeError("实际链接参数缺少值：" + value)
            index += 1
            continue
        if value == "-Bstatic":
            static = True
        elif value == "-Bdynamic":
            static = False
        if value == "-l":
            if index >= len(values):
                raise RuntimeError("实际链接-l缺少库名")
            value += values[index]
            index += 1
        script = None
        if value in ("-T", "--script", "--version-script"):
            if index >= len(values):
                raise RuntimeError("实际链接脚本参数缺少文件")
            script = values[index]
            index += 1
        elif value.startswith(("--script=", "--version-script=")):
            script = value.split("=", 1)[1]
        if script is not None:
            path = Path(script)
            result.append({"kind": "linker-script", "path": str(path if path.is_absolute() else (cwd / path).resolve()), "exists_after_link": path.is_file() if path.is_absolute() else (cwd / path).is_file()})
        elif not value.startswith("-") and (value.endswith((".o", ".a", ".rlib", ".so")) or ".so." in value):
            path = Path(value)
            if not path.is_absolute():
                path = cwd / path
            result.append({"kind": "archive" if value.endswith((".a", ".rlib")) else "object" if value.endswith(".o") else "shared-library", "path": str(path), "exists_after_link": path.exists()})
        elif value.startswith("-l") and len(value) > 2:
            name = value[2:]
            filenames = [name[1:]] if name.startswith(":") else ["lib" + name + suffix for suffix in ((".a",) if static else (".so", ".a"))]
            selected = next((directory / filename for directory in roots for filename in filenames if (directory / filename).is_file()), None)
            if selected is None:
                for filename in filenames:
                    found = capture([arguments[0], "-print-file-name=" + filename])
                    if found != filename and Path(found).is_file():
                        selected = Path(found)
                        break
            if selected is None:
                raise RuntimeError("无法解析实际链接库来源：" + value)
            result.append({"kind": "static-library" if static else "library", "name": name, "path": str(selected.resolve()), "exists_after_link": True})
    if not result:
        raise RuntimeError("实际链接参数未包含可解析对象或库")
    return result


def pinned_cmake_sources(spec, root):
    if "_pinned_cmake_sources" not in spec:
        sources = []
        for relative in spec["paths"]:
            directory = root / relative
            if not directory.is_dir():
                continue
            for recipe in directory.rglob("CMakeLists.txt"):
                if not common.covered(recipe.relative_to(root).as_posix(), spec):
                    continue
                text = recipe.read_text(encoding="utf-8")
                for name, url, digest in re.findall(r"FetchContent_Declare\(\s*(\w+)\s+URL\s+(\S+).*?URL_HASH\s+SHA256=([0-9a-f]+)", text, re.DOTALL):
                    sources.append({"name": name, "url": url, "sha256": digest, "recipe": recipe.relative_to(root).as_posix()})
        spec["_pinned_cmake_sources"] = sorted(sources, key=lambda item: (item["name"], item["recipe"]))
    return spec["_pinned_cmake_sources"]


def registered_environment(root, cache, spec, program):
    """根的身份由镜像/配方/锁文件进入指纹，根内不做安装文件白名单。"""
    manifest = root / common.PROGRAMS[program]["manifest"]
    lock = manifest.with_name("Cargo.lock")
    lock_relative = lock.relative_to(root).as_posix()
    if not common.covered(lock_relative, spec):
        raise RuntimeError("登记Cargo环境根的锁文件未进入程序指纹：" + lock_relative)
    def recipe_identity(relative):
        if not common.covered(relative, spec):
            raise RuntimeError("登记环境根的配方未进入程序指纹：" + relative)
        return {"path": relative, "git_blob": capture(["git", "-C", str(root), "rev-parse", "HEAD:" + relative])}
    program_identity = {"program": program, "binding": "current program fingerprint+Cargo.lock", "recipe": recipe_identity("scripts/module_installer/builds.py")}
    entries = []
    def add(path, kind, identity):
        entries.append({"path": str(Path(path).resolve()), "kind": kind, "identity": identity})
    for path in ("/usr", "/lib", "/lib64"):
        add(path, "system-image", {"binding": "build_identity.image+system_packages"})
    add(os.environ["RUSTUP_HOME"], "rust-toolchain", {"version": builds.RUST_VERSION})
    for path in ("/opt/protoc", "/opt/lcm-gen-1.5.2"):
        if Path(path).is_dir():
            add(path, "image-tool-prefix", {"binding": "build_identity.image.dockerfile_sha256"})
    for path in ("registry", "git"):
        add(cache / "cargo" / path, "cargo-sources", {"lockfile": lock_relative, "sha256": common.sha256(lock)})
    add(cache / "prefix", "program-build-output", program_identity)
    if program == "nuwax-slam-2d-worker":
        from scripts.prepare_slam2d_dependencies import BOOST_URL, BOOST_SHA256, SOURCES, SOURCE_ROLES
        native_identity = {"recipe": recipe_identity("scripts/prepare_slam2d_dependencies.py"), "boost": {"url": BOOST_URL, "sha256": BOOST_SHA256}, "sources": SOURCES}
        for path in (cache / "build/slamtec", cache / "slam2d-dependencies/build", cache / "slam2d-dependencies/boost-source/boost_1_83_0"):
            add(path, "fixed-native-environment", native_identity)
        registry = cache / "slam2d-dependencies/sources/source-paths-v1.json"
        if registry.is_file():
            slots = json.loads(registry.read_text())["slots"]
            for name, (url, revision) in SOURCES.items():
                entry = slots.get(SOURCE_ROLES[name])
                if entry:
                    if entry["url"] != url or entry["revision"] != revision:
                        raise RuntimeError("固定原生源码根身份与受审配方不符：" + name)
                    directory = (registry.parent / common._relative(entry["path"])).resolve()
                    if registry.parent.resolve() not in directory.parents:
                        raise RuntimeError("固定原生源码登记根越出受管缓存")
                    add(directory, "fixed-source-checkout", {"repository": url, "revision": revision, "recipe": native_identity["recipe"]})
    add(rust_target(program, cache), "cargo-build-output", {"binding": "program fingerprint+Cargo.lock"})
    add(cache / "compiler-tmp", "compiler-temporary-output", {"version": builds.RUST_VERSION, "binding": "build_identity.image+fixed TMPDIR", "recipe": recipe_identity("scripts/ci/prebuilt.py")})
    return sorted({json.dumps(item, sort_keys=True): item for item in entries}.values(), key=lambda item: item["path"])


def check_registered_path(path, roots):
    selected = Path(path).resolve()
    candidates = [item for item in roots if selected == Path(item["path"]) or Path(item["path"]) in selected.parents]
    if not candidates:
        raise RuntimeError("实际输入不在登记构建环境根内：" + str(selected))
    return max(candidates, key=lambda item: len(Path(item["path"]).parts))


def actual_cargo_packages(spec, linked):
    packages = {item["id"]: item for item in spec["cargo_packages"]}
    return [packages[key] for key in sorted({item["package_id"] for item in linked["compiler_artifacts"]})]


def component_sources(spec, linked, native_links, private, root, cache):
    """组件组成来自实际Cargo事件、链接记录和固定内嵌目录。"""
    from scripts.prepare_slam2d_dependencies import BOOST_URL, BOOST_SHA256
    components = {}
    locks = {path: common.sha256(root / path) for path in spec["paths"] if path.endswith("Cargo.lock") and (root / path).is_file()}
    def add(item):
        components[item["kind"] + ":" + item["id"]] = item
    for package in actual_cargo_packages(spec, linked):
        directory = Path(package["manifest_path"]).parent
        add({"kind": "crate" if package["source"] else "repository-crate", "id": package["id"], "name": package["name"], "version": package["version"], "source": package["source"], "license": package["license"], "role": "compiled-cargo-package", "dependency_roles": package["dependency_roles"], "native_linked": package["id"] in linked.get("map_native_packages", []), "lockfiles": locks, "root": str(directory), "notice_files": [str(directory / package["license_file"])] if package.get("license_file") else []})
        for embedded in (directory / "native/upstream", directory / "native/sdk"):
            if embedded.is_dir() and root in embedded.parents:
                relative = embedded.relative_to(root).as_posix()
                add({"kind": "repository-embedded", "id": relative, "tree": capture(["git", "-C", str(root), "rev-parse", "HEAD:" + relative]), "root": str(embedded), "usage": ["linked-native" if package["id"] in linked.get("map_native_packages", []) else "build-component"], "notice_roots": [str(directory), str(embedded.parent), str(embedded)]})
    add({"kind": "rust-toolchain", "id": "Rust-" + builds.RUST_VERSION, "version": builds.RUST_VERSION, "root": capture(["rustc", "--print", "sysroot"])})
    for record in native_links:
        values = cmake_cache_values(Path(record["cmake_root"]) / "CMakeCache.txt")
        home = Path(values["CMAKE_HOME_DIRECTORY"]).resolve()
        if root in home.parents:
            for embedded in (home / "upstream", home / "sdk"):
                if embedded.is_dir():
                    relative = embedded.relative_to(root).as_posix()
                    add({"kind": "repository-embedded", "id": relative, "tree": capture(["git", "-C", str(root), "rev-parse", "HEAD:" + relative]), "root": str(embedded), "usage": [record["usage"]], "notice_roots": [str(home), str(embedded)]})
    for recipe in pinned_cmake_sources(spec, root):
        marker = "/_deps/" + recipe["name"].lower()
        bases = {item["output"].split(marker, 1)[0] for item in native_links if marker in item["output"]}
        for base in sorted(bases):
            directory = Path(base + marker + "-src")
            component = {"kind": "pinned-source", "id": recipe["name"], "root": str(directory), "usage": sorted({item["usage"] for item in native_links if marker in item["output"]}), **{key: recipe[key] for key in ("url", "sha256", "recipe")}}
            add(component)
            if recipe["name"].lower() == "gtsam_source":
                declarations = {
                    "Cephes": ("gtsam/3rdparty/cephes", ["LICENSE.txt", "README.md", "cephes/readme", "cephes/igami.c", "cephes/lanczos.c", "cephes/lanczos.h"]),
                    "METIS": ("gtsam/3rdparty/metis", ["LICENSE.txt"]),
                    "GKlib": ("gtsam/3rdparty/metis/GKlib", ["getopt.c", "gk_mksort.h", "gkregex.c", "gkregex.h"]),
                }
                for name, (relative, notices) in declarations.items():
                    nested = directory / relative
                    add({"kind": "embedded-component", "id": "GTSAM/" + name, "root": str(nested), "usage": component["usage"], "url": recipe["url"], "sha256": recipe["sha256"], "notice_files": [str(nested / value) for value in notices if (nested / value).is_file()]})
    for source_cache in (cache, cache / "slam2d-dependencies"):
        registry = source_cache / "sources/source-paths-v1.json"
        if registry.is_file():
            for role, item in json.loads(registry.read_text())["slots"].items():
                directory = registry.parent / item["path"]
                if any(str(directory) in str(value) for record in native_links for value in (cmake_cache_values(Path(record["cmake_root"]) / "CMakeCache.txt").get("CMAKE_HOME_DIRECTORY", ""),)):
                    add({"kind": "pinned-source", "id": role, "root": str(directory), "repository": item["url"], "revision": item["revision"], "usage": sorted({record["usage"] for record in native_links if str(directory) in cmake_cache_values(Path(record["cmake_root"]) / "CMakeCache.txt").get("CMAKE_HOME_DIRECTORY", "")})})
    if any(name.startswith("libboost_") for name in private):
        add({"kind": "pinned-source", "id": "boost-1.83.0", "root": str(cache / "slam2d-dependencies/boost-source/boost_1_83_0"), "url": BOOST_URL, "sha256": BOOST_SHA256, "usage": ["runtime-library"]})
    return sorted(components.values(), key=lambda item: (item["kind"], item["id"]))


def collect_audit(program, spec, checked, linked, root, cache):
    link_inputs = resolve_link_inputs(linked["linker_arguments"], root)
    native_links = linked["native_link_records"]
    private = private_libraries(program, cache / "prefix", cache)
    for record in native_links:
        link_inputs.extend(record["inputs"])
    map_text = Path(linked["link_map"]).read_text()
    map_archives = sorted(set(re.findall(r"([^\s()]+\.(?:rlib|a))\(", map_text)))
    link_inputs.extend({"kind": "map-archive", "path": value, "exists_after_link": Path(value).is_file()} for value in map_archives)
    runtime, private_records = {}, []
    for binary in [cache / "prefix/bin" / program, *private.values()]:
        listing = capture(["ldd", str(binary)])
        if "not found" in listing:
            raise RuntimeError("实际程序动态依赖闭包有缺失")
        for name, path in re.findall(r"^\s*(\S+)\s+=>\s+(/\S+)", listing, re.MULTILINE):
            runtime[str(Path(path).resolve())] = {"kind": "runtime-library", "name": name, "path": str(Path(path).resolve()), "exists_after_link": True}
    link_inputs.extend(runtime.values())
    roots = registered_environment(root, cache, spec, program)
    for value in checked["external_inputs"] + [item["path"] for item in link_inputs]:
        selected = Path(value).resolve()
        if selected == root or root in selected.parents:
            if not common.covered(selected.relative_to(root).as_posix(), spec):
                raise RuntimeError("程序指纹漏掉实际链接输入：" + str(selected))
        else:
            check_registered_path(selected, roots)
    for name, path in sorted(private.items()):
        dynamic = capture(["readelf", "--dynamic", str(path)])
        private_records.append({"name": name, "path": str(path), "sha256": common.sha256(path), "needed": re.findall(r"\(NEEDED\).*\[([^]]+)\]", dynamic), "build_id": re.findall(r"Build ID: (\S+)", capture(["readelf", "--notes", str(path)])), "environment": check_registered_path(path, roots), "load": "explicit-dlopen" if name.startswith("libnuwax_slamtec") else "dynamic-dependency"})
    components = component_sources(spec, linked, native_links, private, root, cache)
    return {**checked, "schema": 2, "program": program, "compiler_commands": linked["compiler_commands"], "compiler_artifacts": linked["compiler_artifacts"], "build_script_events": linked["build_script_events"], "compiled_binary_sha256": linked["binary_sha256"], "link_record_origin": "executed" if linked["captured_this_build"] else "binary-sha-matched-cache", "linker_arguments": linked["linker_arguments"], "link_inputs": link_inputs, "native_link_records": native_links, "native_compilation_records": linked["native_compilation_records"], "rust_dependency_record_origins": linked["rust_dependency_record_origins"], "environment_roots": roots, "private_library_records": private_records, "external_sources": components, "cargo_resolution": actual_cargo_packages(spec, linked), "source_revision": capture(["git", "-C", str(root), "rev-parse", "HEAD"])}


def review_key(audit, root, cache):
    identities = [{key: value for key, value in item.items() if key not in ("root", "notice_roots", "notice_files")} for item in audit["external_sources"]]
    payload = {"repository_inputs": sorted(audit["repository_inputs"]), "components": sorted(json.dumps(item, sort_keys=True) for item in identities), "native_libraries": sorted(item["name"] for item in audit["private_library_records"]), "licenses": sorted((item["source"], item["sha256"]) for item in audit["license_files"]), "system_packages": audit["system_packages_used"]}
    return hashlib.sha256(canonical(json.dumps(payload, sort_keys=True), root, cache).encode()).hexdigest()


def require_license_review(indexes, policy_path):
    try:
        policy = json.loads(policy_path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise RuntimeError("缺少逐程序实际编译/链接许可核查清单，拒绝发布：" + str(policy_path)) from exc
    if policy.get("schema") != 1 or set(policy.get("programs", {})) != set(common.PROGRAMS):
        raise RuntimeError("许可核查清单必须精确覆盖五个可预编译程序")
    for index in indexes:
        for program, entry in index["programs"].items():
            reviewed = policy["programs"][program].get("platforms", {}).get(index["platform"], {})
            if reviewed.get("decision") != "approved" or reviewed.get("review_key") != entry["audit"]["review_key"] or not reviewed.get("license_expression") or not reviewed.get("reviewed_components"):
                raise RuntimeError("实际输入未获完整许可核查批准：" + index["platform"] + "/" + program)


def build(args):
    root, cache, output = ROOT, args.cache.resolve(), args.output.resolve()
    if common.platform_key() != args.platform:
        raise RuntimeError("构建环境与所选目标平台不符")
    output.mkdir(parents=True, exist_ok=True)
    cache.mkdir(parents=True, exist_ok=True)
    (output / ("platform-" + args.platform + ".json")).unlink(missing_ok=True)
    prefix = cache / "prefix"
    prefix.mkdir(exist_ok=True)
    enable_ci_native_diagnostics(cache)
    probe(output / "probe.json")
    cargo, build_identity = identity(args.platform, root)
    environment = {"CARGO_INCREMENTAL": "0", "CARGO_HOME": str(cache / "cargo")}
    target = {"focal-arm64": "aarch64-unknown-linux-gnu", "noble-x86_64": "x86_64-unknown-linux-gnu"}[args.platform]
    inventory = json.loads(args.assets.read_text(encoding="utf-8"))
    reject_lio_assets(inventory)
    assets = {item["name"]: item for item in inventory}
    programs, compiled, reused, packaged = {}, [], [], []
    failures = []
    for program, recipe in common.PROGRAMS.items():
        try:
            spec = common.discover_inputs(root, program, cargo, environment, target)
            public_spec = {name: spec[name] for name in ("paths", "exclude")}
            value = common.fingerprint(root, program, args.platform, spec, build_identity)
            name = program + "-" + value + ".tar.zst"
            existing = asset_entry(assets[name]) if name in assets else None
            audit_cache = cache / "audits" / (value + ".json")
            if existing:
                if not audit_cache.is_file():
                    with tempfile.TemporaryDirectory(prefix=".read-audit-", dir=cache) as temporary:
                        directory = Path(temporary)
                        archive = directory / name
                        with common._urlopen(common.release_url(args.repository, name), timeout=60) as source, archive.open("wb") as destination:
                            shutil.copyfileobj(source, destination)
                        if common.sha256(archive) != existing["sha256"]:
                            raise RuntimeError("已有程序附件摘要错误")
                        unpacked = directory / "unpacked"
                        unpacked.mkdir()
                        common._extract(archive, unpacked, program, value)
                        audit = json.loads((unpacked / common.program_share(program) / "audit/inputs.json").read_text())
                        dump(audit_cache, audit)
                audit = json.loads(audit_cache.read_text())
                if audit.get("program") != program or audit.get("fingerprint") != value or not audit.get("review_key"):
                    raise RuntimeError("已有程序缺少可核查的实际输入审计")
                dump(output / ("audit-" + program + ".json"), audit)
                programs[program] = {**existing, "fingerprint": value, "input_spec": public_spec, "audit": {"review_key": audit["review_key"], "coverage": "complete"}}
                reused.append(program)
                print("云端跳过同指纹程序：" + program, flush=True)
                continue
            migrations = migrate_cmake_diagnostics(program, cache, spec)
            dump(output / ("cmake-migrations-" + program + ".json"), migrations)
            link_map = cache / "compiler-tmp/linkmaps" / (program + ".map")
            link_map.parent.mkdir(parents=True, exist_ok=True)
            log_path = output / (program + ".build.log")
            with log_path.open("w", encoding="utf-8") as log:
                def recording_run(argv, *, cwd=None, env=None):
                    actual_argv = diagnostic_command(argv, link_map)
                    log.write("执行：" + " ".join(map(str, actual_argv)) + "\n")
                    log.flush()
                    process = subprocess.Popen(actual_argv, cwd=cwd, env={**os.environ, **(env or {})}, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                    try:
                        for line in process.stdout:
                            sys.stdout.write(line)
                            sys.stdout.flush()
                            log.write(line)
                        status = process.wait()
                    finally:
                        process.stdout.close()
                        log.flush()
                        if process.poll() is None:
                            process.kill()
                            process.wait()
                    if status:
                        raise subprocess.CalledProcessError(status, argv)
                builds.build_modules(root, prefix, cache, [recipe["module"]], args.jobs, recording_run, local_build=True, programs=[program], capture_link_args=True)
            shutil.copyfile(prefix / "bin" / program, output / (program + ".elf"))
            linked = link_arguments(log_path.read_text(), program, prefix / "bin" / program, cache, link_map)
            shutil.copyfile(link_map, output / (program + ".link.map"))
            dump(output / ("audit-" + program + ".json"), {"program": program, "link_record": linked, "status": "dependency-record-mapping-in-progress"})
            records, record_cwds = dependency_records(program, cache, spec, linked)
            roots = registered_environment(root, cache, spec, program)
            dependency_artifacts = []
            for record in records:
                name = hashlib.sha256(str(record).encode()).hexdigest() + record.suffix
                saved = output / "native-dependencies" / program / name
                saved.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(record, saved)
                dependency_artifacts.append({"source": str(record), "artifact": saved.relative_to(output).as_posix(), "sha256": common.sha256(record), "working_directory": record_cwds.get(str(record.resolve()), str(root))})
            checked = common.audit_dependencies(root, spec, records, environment_roots=roots, record_cwds=record_cwds)
            dump(output / ("audit-" + program + ".json"), {**checked, "link_record": linked, "status": "source-mapping-in-progress"})
            audit = collect_audit(program, spec, checked, linked, root, cache)
            audit.update(platform=args.platform, fingerprint=value, dependency_record_artifacts=dependency_artifacts, cmake_diagnostic_migrations=migrations)
            # 即使许可尚未批准，也保存真实输入与构建证据；发布在另一作业严格拒绝。
            dump(output / ("audit-" + program + ".json"), audit)
            entry = package(root, prefix, cache, output, program, value, spec, audit)
            dump(output / ("audit-" + program + ".json"), audit)
            dump(audit_cache, audit)
            programs[program] = {**entry, "fingerprint": value, "input_spec": public_spec, "audit": {"review_key": audit["review_key"], "coverage": "complete"}}
            packaged.append(program)
            if linked["captured_this_build"]:
                compiled.append(program)
        except Exception as exc:
            failure = {"program": program, "error": str(exc), "traceback": traceback.format_exc()}
            failures.append(failure)
            dump(output / ("error-" + program + ".json"), failure)
            print("程序失败，保留证据后继续其余程序：" + program + "：" + str(exc), flush=True)
    for receipt in (cache / "audit-cmake-configs").glob("*.json"):
        destination = output / "cmake-configuration-ledger" / receipt.name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(receipt, destination)
    if failures:
        dump(output / "build-report.json", {"success": False, "failures": failures, "compiled": compiled, "reused": reused, "packaged": packaged})
        raise RuntimeError("五程序构建/审计存在失败，拒绝生成发布索引：" + ", ".join(item["program"] for item in failures))
    revision = capture(["git", "rev-parse", "HEAD"], cwd=root)
    dump(output / ("platform-" + args.platform + ".json"), {"schema": common.SCHEMA, "revision": revision, "platform": args.platform, "build_identity": build_identity, "programs": programs})
    dump(output / "build-report.json", {"compiled": compiled, "reused": reused, "packaged": packaged, "counts": {"compiled": len(compiled), "reused": len(reused), "packaged": len(packaged)}})
    print("五程序构建结果：" + json.dumps({"compiled": compiled, "reused": reused}, ensure_ascii=False), flush=True)


def gh_json(args):
    return json.loads(capture(["gh", *args]))


def list_assets(repository, release_id):
    pages = gh_json(["api", "--paginate", "--slurp", "repos/" + repository + "/releases/" + str(release_id) + "/assets?per_page=100"])
    return [item for page in pages for item in page]


def inventory(args):
    try:
        release = gh_json(["api", "repos/" + args.repository + "/releases/tags/" + common.RELEASE])
    except subprocess.CalledProcessError:
        # 存在性失败只影响复用，上传阶段仍须明确创建/校验专用Release。
        dump(args.output, [])
        return
    inventory = list_assets(args.repository, release["id"])
    reject_lio_assets(inventory)
    for item in inventory:
        match = re.fullmatch(r"index-([0-9a-f]{40})\.json", item["name"])
        if match:
            common.fetch_index(args.repository, match.group(1))
    dump(args.output, inventory)


def referenced_assets(index):
    result = {"index-" + index["revision"] + ".json"}
    for entry in index.get("platforms", {}).values():
        result.update(program["asset"] for program in entry.get("programs", {}).values())
    return result


def publish(args):
    directory = args.output.resolve()
    indexes = [json.loads(path.read_text(encoding="utf-8")) for path in sorted(directory.rglob("platform-*.json"))]
    if len(indexes) != 2 or {item["platform"] for item in indexes} != common.PLATFORMS or any(item["revision"] != args.revision or item.get("schema") != common.SCHEMA for item in indexes):
        raise RuntimeError("两平台完整提交索引不齐全，拒绝发布")
    payload = {"schema": common.SCHEMA, "revision": args.revision, "created_at": datetime.datetime.now(datetime.timezone.utc).isoformat(), "repository": args.repository, "platforms": {item["platform"]: {"build_identity": item["build_identity"], "programs": item["programs"]} for item in indexes}}
    for item in indexes:
        if set(item["programs"]) != set(common.PROGRAMS):
            raise RuntimeError("预编译索引缺少程序")
        if item.get("source_archives"):
            raise RuntimeError("五程序索引禁止包含LIO源码归档")
    require_license_review(indexes, ROOT / "third_party/prebuilt-license-review.json")
    try:
        release = gh_json(["api", "repos/" + args.repository + "/releases/tags/" + common.RELEASE])
    except subprocess.CalledProcessError:
        run(["gh", "release", "create", common.RELEASE, "--repo", args.repository, "--target", args.revision, "--prerelease", "--title", "NEH按程序预编译仓库", "--notes", "此专用Release供匿名设备按程序指纹下载；完整提交索引与固定第三方源码随附件保存。"])
        release = gh_json(["api", "repos/" + args.repository + "/releases/tags/" + common.RELEASE])
    if release["tag_name"] != common.RELEASE or release.get("draft"):
        raise RuntimeError("专用预编译Release身份错误")
    assets = list_assets(args.repository, release["id"])
    reject_lio_assets(assets)
    protected = referenced_assets(payload)
    # 清理必须读到main通道状态；若已存在通道但无法解析，则拒绝清理而非猜测引用。
    channel_name = "channels-main.json"
    channel_assets = [item for item in assets if item["name"] == channel_name]
    if channel_assets:
        with tempfile.TemporaryDirectory(prefix="neh-channel-protect-") as raw:
            run(["gh", "release", "download", common.RELEASE, "--repo", args.repository, "--pattern", channel_name, "--dir", raw])
            channel = json.loads((Path(raw) / channel_name).read_text())
            main_revision = channel["revision"]
            main_index = common.fetch_index(args.repository, main_revision)
            protected.update(referenced_assets(main_index))
            protected.add(channel_name)
    cutoff = datetime.datetime.now(datetime.timezone.utc) - datetime.timedelta(days=30)
    for asset in assets:
        created = datetime.datetime.fromisoformat(asset["created_at"].replace("Z", "+00:00"))
        if created < cutoff and asset["name"] not in protected:
            run(["gh", "api", "--method", "DELETE", "repos/" + args.repository + "/releases/assets/" + str(asset["id"])])
    current = {item["name"]: item for item in list_assets(args.repository, release["id"])}
    uploads = {}
    for path in directory.rglob("*"):
        if path.is_file():
            reject_lio_assets([path.name])
        if path.is_file() and path.name.endswith(".tar.zst"):
            if path.name not in {entry["asset"] for platform in indexes for entry in platform["programs"].values()}:
                raise RuntimeError("程序附件不属于五程序完整提交索引：" + path.name)
            if path.name in uploads and common.sha256(uploads[path.name]) != common.sha256(path):
                raise RuntimeError("同名程序附件内容冲突：" + path.name)
            uploads[path.name] = path
    index = directory / ("index-" + args.revision + ".json")
    dump(index, payload)
    uploads[index.name] = index
    required_new = set(uploads) - set(current)
    if len(current) + len(required_new) > 1000:
        raise RuntimeError("专用Release达到1000附件上限；受保护或30天内制品不能自动删除")
    for name, path in uploads.items():
        if name in current:
            existing = asset_entry(current[name])
            if existing and existing["sha256"] == common.sha256(path):
                continue
            if name != index.name:
                raise RuntimeError("已有同指纹附件内容冲突，拒绝覆盖：" + name)
            # 索引完全通过两平台验证后可更新同提交；程序附件永不覆盖。
            run(["gh", "release", "upload", common.RELEASE, str(path), "--repo", args.repository, "--clobber"])
        else:
            run(["gh", "release", "upload", common.RELEASE, str(path), "--repo", args.repository])
    # 最后匿名读取索引，保证公开下载路径可用。
    published = common.fetch_index(args.repository, args.revision)
    if published != payload:
        raise RuntimeError("匿名索引下载与发布内容不符")
    print("专用预编译索引已发布：" + common.release_url(args.repository, index.name), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    command = commands.add_parser("probe")
    command.add_argument("--output", type=Path, required=True)
    command = commands.add_parser("inventory")
    command.add_argument("--repository", required=True)
    command.add_argument("--output", type=Path, required=True)
    command = commands.add_parser("build")
    command.add_argument("--platform", choices=sorted(common.PLATFORMS), required=True)
    command.add_argument("--repository", required=True)
    command.add_argument("--cache", type=Path, required=True)
    command.add_argument("--output", type=Path, required=True)
    command.add_argument("--assets", type=Path, required=True)
    command.add_argument("--jobs", type=int, default=4)
    command = commands.add_parser("native-cmake")
    command.add_argument("--cmake", required=True)
    command.add_argument("--cache", type=Path, required=True)
    command.add_argument("arguments", nargs=argparse.REMAINDER)
    command = commands.add_parser("publish")
    command.add_argument("--repository", required=True)
    command.add_argument("--revision", required=True)
    command.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.command == "native-cmake":
        return native_cmake(args)
    if args.command == "probe":
        probe(args.output)
    else:
        globals()[args.command](args)


if __name__ == "__main__":
    raise SystemExit(main())
