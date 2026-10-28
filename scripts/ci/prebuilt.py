#!/usr/bin/env python3
"""在两个目标环境逐程序编译、审计、封装并发布专用Release。"""
from __future__ import annotations

import argparse
import contextlib
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
import tarfile
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


def enable_ci_native_diagnostics():
    for name in ("CFLAGS", "CXXFLAGS"):
        flags = os.environ.get(name, "").strip()
        if not shlex.split(flags) or shlex.split(flags)[-1] != "-MD":
            os.environ[name] = (flags + " -MD").strip()
    os.environ["CC_ENABLE_DEBUG_OUTPUT"] = "1"


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
        "parameters": {"cargo": ["rustc", "--locked", "--release", "-vv", "--message-format=json-render-diagnostics", "--print=link-args", "-Clink-arg=-Wl,-Map=$CACHE/audit-linkmaps/<program>.map"], "native_diagnostics": {"dependency": "-MD", "boost_compiler_wrapper": 1, "boost_map": "$CACHE/audit-linkmaps/boost/<output-digest>.map"}, "cmake": ["CMAKE_BUILD_TYPE=Release", "CMAKE_GENERATOR=Unix Makefiles"], "environment": {name: os.environ.get(name, "") for name in ("RUSTFLAGS", "CFLAGS", "CXXFLAGS", "LDFLAGS", "NUWAX_CXX20", "CC_ENABLE_DEBUG_OUTPUT")}},
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
    """仅收集实际读取/链接组件的声明，不能用历史缓存推断组成。"""
    notice_root = staging / "share/licenses"
    notice_root.mkdir(parents=True)
    selected = {root / "LICENSE", root / "docs/公开文档/第三方许可证说明.md"}
    for value in audit["repository_inputs"] + audit["external_inputs"]:
        path = root / value if not Path(value).is_absolute() else Path(value)
        owner = source_owner(path, spec, root, cache, audit["program"])
        if owner["kind"] == "system":
            continue
        current = path.parent
        boundary = Path(owner["root"])
        if not (current == boundary or boundary in current.parents):
            current = boundary
        while current == boundary or boundary in current.parents:
            if current.is_dir():
                for entry in current.iterdir():
                    if entry.is_file() and entry.name.upper().startswith(("LICENSE", "LICENCE", "COPYING", "NOTICE")):
                        selected.add(entry)
            if current == boundary:
                break
            current = current.parent
    system_paths = [value for value in audit["external_inputs"] if value.startswith(("/usr/", "/lib/"))]
    system_paths.extend(item["path"] for item in audit["link_inputs"] if item["path"].startswith(("/usr/", "/lib/")))
    if any(item["kind"] == "rust-toolchain" for item in audit["external_sources"]):
        sysroot = Path(capture(["rustc", "--print", "sysroot"]))
        notices = sysroot / "share/doc/rust"
        if (notices / "COPYRIGHT-library.html").is_file():
            selected.add(notices / "COPYRIGHT-library.html")
        selected.update(path for path in (notices / "licenses").glob("*") if path.is_file())
    aliases = []
    for path in system_paths:
        aliases.append(path)
        if path.startswith(("/usr/lib/", "/usr/bin/", "/usr/sbin/")):
            aliases.append(path[4:])
    system_paths = sorted(set(aliases))
    system_packages = set()
    for offset in range(0, len(system_paths), 200):
        query = subprocess.run(["dpkg-query", "-S", *system_paths[offset:offset + 200]], text=True, capture_output=True)
        for line in query.stdout.splitlines():
            if ": /" in line:
                system_packages.update(line.split(": /", 1)[0].split(", "))
    for package_name in system_packages:
        copyright_file = Path("/usr/share/doc") / package_name.split(":", 1)[0] / "copyright"
        if copyright_file.is_file():
            selected.add(copyright_file)
    for name in ("GPL-2", "GPL-3", "LGPL-2.1", "LGPL-3", "Apache-2.0", "BSD"):
        source = Path("/usr/share/common-licenses") / name
        if source.is_file():
            selected.add(source)
    if not (root / "LICENSE").is_file():
        raise RuntimeError("发布程序缺少项目LICENSE")
    files = []
    for index, source in enumerate(sorted(selected)):
        if not source.is_file():
            continue
        relative = str(index) + "/" + source.name
        destination = notice_root / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, destination)
        files.append({"path": "share/licenses/" + relative, "source": canonical(source, root, cache), "sha256": common.sha256(source), "scope": "project-overview" if source.name == "第三方许可证说明.md" else "common-license-text" if source.parent == Path("/usr/share/common-licenses") else "actual-component"})
    audit["license_files"] = files
    audit["system_packages_used"] = capture(["dpkg-query", "-W", "-f=${binary:Package}=${Version}\n", *sorted(system_packages)]).splitlines() if system_packages else []
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
        dump(staging / "share/sources/对应源码.json", corresponding_sources(root, program, spec, audit, revision, repository))
        # LGPL组件源码及替换共享库所需的包装源码随程序一起提供。
        if program == "nuwax-slam-2d-worker":
            for path in ("services/nuwax-workspace/crates/nuwax-slam-toolbox-sys/native", "drivers/sensors/slamtec_native"):
                shutil.copytree(root / path, staging / "share/sources" / path)
        audit["relink_materials"] = {"sources": "share/sources", "corresponding_source": "share/sources/对应源码.json", "private_libraries": sorted(libraries), "build_recipe": "scripts/module_installer/builds.py"}
        audit["review_key"] = review_key(audit, root, cache)
        dump(staging / "share/audit/inputs.json", audit)
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
    target = rust_target(program, cache)
    packages = {item["name"] for item in spec["cargo_packages"]}
    active = actual_inputs + linked["linker_arguments"]
    roots = [directory for directory in (target / "release/build").iterdir() if directory.name.rsplit("-", 1)[0] in packages and any(str(directory) in value for value in active)]
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
    matched_outputs = {Path(item["output"]).name for item in result}
    for name, library in private.items():
        if not name.startswith("libboost_") and library.name not in matched_outputs:
            raise RuntimeError("实际私有动态库缺少匹配的原生最终链接记录：" + name)
    return result, private


def object_dependency(path):
    for candidate in (Path(str(path) + ".d"), path.with_suffix(".d")):
        if candidate.is_file():
            return candidate
    raise RuntimeError("实际采用的原生对象缺少编译器依赖记录（object.d/object.o.d）：" + str(path))


def archive_index(path):
    """读取普通GNU/BSD归档的真实成员偏移与符号表，保留重复成员名。"""
    members, tables, names = [], [], b""
    with path.open("rb") as source:
        if source.read(8) != b"!<arch>\n":
            raise RuntimeError("实际采用的原生归档不是受支持的普通ar格式：" + str(path))
        while True:
            offset = source.tell()
            header = source.read(60)
            if not header:
                break
            if len(header) != 60 or header[58:] != b"`\n":
                raise RuntimeError("原生归档成员头损坏：" + str(path))
            label = header[:16].decode("utf-8").strip()
            size = int(header[48:58])
            if size < 0 or offset + 60 + size > path.stat().st_size:
                raise RuntimeError("原生归档成员大小越界：" + str(path))
            data_offset = source.tell()
            if label == "//":
                names = source.read(size)
            elif label in ("/", "/SYM64/"):
                tables.append((8 if label == "/SYM64/" else 4, source.read(size)))
            else:
                name, payload_size = label.rstrip("/"), size
                if label.startswith("#1/"):
                    length = int(label[3:])
                    name = source.read(length).rstrip(b"\0").decode("utf-8")
                    data_offset += length
                    payload_size -= length
                elif label.startswith("/"):
                    position = int(label[1:])
                    name = names[position:].split(b"/\n", 1)[0].rstrip(b"\0").decode("utf-8")
                digest = hashlib.sha256()
                remaining = payload_size
                while remaining:
                    block = source.read(min(1024 * 1024, remaining))
                    if not block:
                        raise RuntimeError("原生归档成员被截断：" + str(path))
                    digest.update(block)
                    remaining -= len(block)
                members.append({"name": name, "offset": offset, "data_offset": data_offset, "size": payload_size, "sha256": digest.hexdigest(), "symbols": []})
            source.seek(offset + 60 + size + size % 2)
    by_offset = {item["offset"]: item for item in members}
    for width, table in tables:
        count = int.from_bytes(table[:width], "big")
        end = width * (count + 1)
        if end > len(table):
            raise RuntimeError("原生归档符号表长度无效：" + str(path))
        symbols = table[end:].split(b"\0")
        if len(symbols) < count:
            raise RuntimeError("原生归档符号表被截断：" + str(path))
        for index in range(count):
            offset = int.from_bytes(table[width * (index + 1):width * (index + 2)], "big")
            if offset not in by_offset:
                raise RuntimeError("原生归档符号指向未知成员偏移：" + str(path))
            by_offset[offset]["symbols"].append(symbols[index].decode("utf-8"))
    return members


def linker_map_members(linked):
    path = Path(linked.get("link_map", "/nonexistent-link-map"))
    if not path.is_file() or common.sha256(path) != linked.get("link_map_sha256"):
        raise RuntimeError("缺少经最终ELF回执验证的实际GNU链接图")
    text = path.read_text(encoding="utf-8", errors="strict")
    if "Archive member included" not in text and "Linker script and memory map" not in text:
        raise RuntimeError("无法解析实际GNU链接图格式")
    pattern = r"([^\s()]+\.(?:rlib|a))\(([^()\n]+)\)"
    selections, pending = {}, None
    if "Archive member included" in text:
        header = text.split("Archive member included", 1)[1].split("\n", 1)[1]
        for line in header.splitlines():
            match = re.match(pattern, line)
            if match:
                pending = {"archive": match.group(1), "member": match.group(2), "map_reference": line}
                selections.setdefault((match.group(1), match.group(2)), []).append(pending)
                tail = line[match.end():]
            elif pending is not None and line.startswith(" "):
                pending["map_reference"] += "\n" + line
                tail = line
            elif line and not line.startswith(" "):
                break
            else:
                continue
            symbol = re.search(r"\s\((.+)\)\s*$", tail)
            if symbol:
                pending["selection_symbol"] = symbol.group(1)
    result = []
    for archive, member in sorted(set(re.findall(pattern, text))):
        if member.endswith((".o", ".obj", ".oS", ".os")):
            result.extend(selections.get((archive, member), [{"archive": archive, "member": member}]))
    return result


def archive_adoption_evidence(program, cache, spec, linked):
    target = rust_target(program, cache)
    adopted = linker_map_members(linked)
    def rust_member(name):
        return bool(re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*-[0-9a-f]{16}\..*\.rcgu\.o", name))
    wanted = {item["member"] for item in adopted if not rust_member(item["member"])}
    candidates = {}
    archive_members = {}
    active_directories = []
    for directory in (target / "release/build").iterdir():
        if not any(str(directory) in argument for argument in linked["linker_arguments"]):
            continue
        output = directory / "output"
        if not output.is_file():
            continue
        package_name = directory.name.rsplit("-", 1)[0]
        owners = [item for item in spec["cargo_packages"] if item["name"] == package_name]
        text = output.read_text()
        names = set(re.findall(r"cargo(?:::|:)rustc-link-lib=static(?::[^=\n]+)?=([^\s]+)", text))
        if not names:
            continue
        if len(owners) != 1:
            raise RuntimeError("实际原生静态库的构建脚本包归属不唯一：" + str(directory))
        active_directories.append({"directory": str(directory), "package": owners[0], "build_script_output": str(output)})
        roots = [directory / "out"]
        roots.extend(Path(value) for value in re.findall(r"cargo(?:::|:)rustc-link-search=(?:native=)?([^\n]+)", text))
        archives = {archive.resolve() for root in roots if root.is_dir() and (root.resolve() == cache or cache in root.resolve().parents) for name in names for archive in root.rglob("lib" + name + ".a")}
        for archive in archives:
            members = archive_index(archive)
            archive_members[str(archive)] = members
            for member in wanted.intersection(item["name"] for item in members):
                candidates.setdefault(member, []).append({"archive": str(archive), "package": owners[0], "build_script_output": str(output), "build_directory": str(directory)})
    resolved, skipped, seen = [], [], set()
    sysroot = Path(capture(["rustc", "--print", "sysroot"]))
    for used in adopted:
        member, archive = used["member"], Path(used["archive"])
        if rust_member(member):
            continue
        stable = archive
        if not stable.is_file():
            choices = [target / "release/deps" / archive.name]
            choices.extend(sysroot.glob("lib/rustlib/*/lib/" + archive.name))
            stable = next((choice for choice in choices if choice.is_file()), None)
        if stable is not None and (sysroot == stable or sysroot in stable.parents or str(stable).startswith(("/usr/", "/lib/"))):
            ownership = None
            if not (sysroot == stable or sysroot in stable.parents):
                aliases = [str(stable)] + ([str(stable)[4:]] if str(stable).startswith("/usr/lib/") else [])
                query = subprocess.run(["dpkg-query", "-S", *aliases], capture_output=True, text=True)
                ownership = sorted({line.split(": /", 1)[0] for line in query.stdout.splitlines() if ": /" in line})
                if not ownership:
                    raise RuntimeError("实际采用的系统原生归档无法映射已安装软件包：" + str(stable))
            skipped.append({**used, "stable_archive": str(stable), "system_packages": ownership, "basis": "prebuilt-toolchain-or-system-package"})
            continue
        if stable is None:
            raise RuntimeError("实际链接采用的归档无法回溯稳定原件：" + used["archive"])
        available = archive_members.get(str(stable))
        if available is None:
            available = archive_index(stable)
            archive_members[str(stable)] = available
        selected = [item for item in available if item["name"] == member]
        if len(selected) > 1:
            symbol = used.get("selection_symbol")
            selected = [item for item in selected if symbol in item["symbols"]]
        if len(selected) != 1:
            raise RuntimeError("实际重复归档成员无法凭Map采用符号与符号表偏移唯一定位：" + str(stable) + "(" + member + ")")
        bundled = selected[0]
        bundled_sha = bundled["sha256"]
        matches = []
        for candidate in candidates.get(member, []):
            original = Path(candidate["archive"])
            original_members = [item for item in archive_members[str(original)] if item["name"] == member and item["sha256"] == bundled_sha]
            if len(original_members) != 1:
                continue
            for obj in original.parent.rglob(Path(member).name):
                if obj.is_file() and common.sha256(obj) == bundled_sha:
                    matches.append({**candidate, **used, "archive": str(original), "object": str(obj), "member": member, "object_sha256": bundled_sha, "adopted_archive": used["archive"], "stable_bundle": str(stable), "stable_member_offset": bundled["offset"], "original_member_offset": original_members[0]["offset"], "stable_member_symbols": bundled["symbols"], "original_member_symbols": original_members[0]["symbols"], "dependency_record": str(object_dependency(obj)), "working_directory": str(Path(candidate["package"]["manifest_path"]).parent)})
        unique = {(item["object"], item["package"]["id"]): item for item in matches}
        if len(unique) != 1:
            raise RuntimeError("实际采用的原生成员无法用build-script、原生归档及rlib成员SHA唯一绑定：" + used["archive"] + "(" + member + ")")
        record = next(iter(unique.values()))
        key = record["object"]
        if key not in seen:
            resolved.append(record)
            seen.add(key)
    return {"adopted_native_objects": resolved, "prebuilt_native_members": skipped, "build_script_sources": active_directories, "link_map": linked["link_map"], "link_map_sha256": linked["link_map_sha256"]}


def native_compiler(args):
    arguments = args.arguments[1:] if args.arguments[:1] == ["--"] else list(args.arguments)
    output = None
    if "-o" in arguments:
        index = arguments.index("-o")
        if index + 1 < len(arguments):
            output = (Path.cwd() / arguments[index + 1]).resolve()
    cache = args.records.parent.resolve()
    kind = "compile" if "-c" in arguments else "link" if "-shared" in arguments else None
    observed = kind and output is not None and cache in output.parents
    link_map = None
    snapshots = []
    if observed and kind == "compile" and "-MD" not in arguments:
        arguments.append("-MD")
    if observed and kind == "link":
        args.map_dir.mkdir(parents=True, exist_ok=True)
        link_map = args.map_dir / (hashlib.sha256(str(output).encode()).hexdigest() + ".map")
        arguments.append("-Wl,-Map=" + str(link_map))
        for value in arguments:
            if not value.startswith("-") and value.endswith(".o"):
                obj = (Path.cwd() / value).resolve()
                if not obj.is_file():
                    raise RuntimeError("Boost实际链接对象缺失：" + str(obj))
                snapshots.append({"path": str(obj), "sha256": common.sha256(obj)})
    status = subprocess.run([args.compiler, *arguments]).returncode
    if observed:
        record = {"kind": kind, "success": status == 0, "compiler_arguments": [args.compiler, *arguments], "working_directory": str(Path.cwd()), "output": str(output), "output_sha256": common.sha256(output) if status == 0 and output.is_file() else None}
        if status == 0 and kind == "compile":
            dependency = object_dependency(output)
            record.update(dependency_record=str(dependency), dependency_sha256=common.sha256(dependency))
        if status == 0 and kind == "link":
            if not link_map.is_file():
                raise RuntimeError("Boost实际共享库链接未产生Map")
            record.update(link_map=str(link_map), link_map_sha256=common.sha256(link_map), input_objects=snapshots, build_id=re.findall(r"Build ID: (\S+)", capture(["readelf", "--notes", str(output)])))
        dump(args.records / (hashlib.sha256((str(output) + str(kind)).encode()).hexdigest() + ".json"), record)
    return status


@contextlib.contextmanager
def diagnostic_command(argv, cwd, cache, link_map):
    arguments = list(map(str, argv))
    if len(arguments) > 1 and arguments[1] == "rustc":
        arguments.insert(arguments.index("--"), "--message-format=json-render-diagnostics")
        arguments.extend(("-C", "link-arg=-Wl,-Map=" + str(link_map)))
    if Path(arguments[0]).name != "b2":
        yield arguments
        return
    config_arg = next((value for value in arguments if value.startswith("--user-config=")), None)
    if config_arg is None:
        raise RuntimeError("Boost审计缺少受管user-config")
    configuration = Path(config_arg.split("=", 1)[1])
    original = configuration.read_text()
    match = re.search(r'using gcc\s*:\s*nuwax\s*:\s*"([^"\n]+)"\s*;', original)
    if match is None:
        raise RuntimeError("Boost实际编译器配置无法唯一解析")
    compiler = match.group(1)
    records = cache / "audit-boost-commands"
    wrapper = cache / "audit-tools/boost-cxx"
    wrapper.parent.mkdir(parents=True, exist_ok=True)
    command = [sys.executable, str(Path(__file__).resolve()), "native-compiler", "--compiler", compiler, "--records", str(records), "--map-dir", str(cache / "audit-linkmaps/boost"), "--"]
    wrapper.write_text("#!/bin/sh\nexec " + " ".join(shlex.quote(value) for value in command) + ' "$@"\n')
    wrapper.chmod(0o755)
    marker = records / "complete-v1.json"
    expected = {"recipe": 1, "compiler": compiler, "compiler_sha256": common.sha256(Path(compiler).resolve()), "arguments": [canonical(value, ROOT, cache) for value in arguments], "dependency": "-MD", "link_map": True}
    try:
        previous = json.loads(marker.read_text())
    except (OSError, ValueError):
        previous = None
    if previous != expected:
        arguments.append("-a")
    arguments.append("-d+2")
    done = False
    try:
        configuration.write_text(original[:match.start(1)] + str(wrapper) + original[match.end(1):])
        yield arguments
        done = True
    finally:
        configuration.write_text(original)
        if done:
            dump(marker, expected)
            if "install" in arguments:
                record_boost_installation(arguments, Path(cwd), cache)


def record_boost_installation(arguments, cwd, cache):
    from scripts.prepare_slam2d_dependencies import BOOST_URL, BOOST_SHA256
    source = cache / "slam2d-dependencies/boost-source/boost_1_83_0"
    archive = cache / "slam2d-dependencies/downloads/boost_1_83_0.tar.bz2"
    prefix = cache / "prefix"
    if cwd.resolve() != source.resolve() or Path(arguments[0]).resolve() != (source / "b2").resolve() or "--prefix=" + str(prefix) not in arguments:
        raise RuntimeError("Boost安装命令不属于固定来源与受管前缀")
    if not archive.is_file() or common.sha256(archive) != BOOST_SHA256:
        raise RuntimeError("Boost安装缺少固定源码归档摘要证明")
    components = sorted(value.split("--with-", 1)[1] for value in arguments if value.startswith("--with-"))
    directories = [prefix / "lib/cmake" / (name + "-1.83.0") for name in ["Boost", "boost_headers", *["boost_" + component for component in components]]]
    files = {path.relative_to(prefix).as_posix(): common.sha256(path) for directory in directories for path in directory.glob("*.cmake") if path.is_file()}
    for component in components:
        files.update({path.relative_to(prefix).as_posix(): common.sha256(path) for path in (prefix / "lib").glob("libboost_" + component + ".so*") if path.is_file()})
    dump(cache / "audit-boost-install/installation-v1.json", {"source": str(source), "url": BOOST_URL, "sha256": BOOST_SHA256, "prefix": str(prefix), "components": components, "b2_sha256": common.sha256(source / "b2"), "arguments": [canonical(value, ROOT, cache) for value in arguments if value not in ("-a", "-d+2")], "files": files})


def boost_installation_owner(selected, spec, cache):
    prefix = cache / "prefix"
    if prefix not in selected.parents:
        return None
    relative = selected.relative_to(prefix).as_posix()
    if not (relative.startswith("include/boost/") or relative.startswith("lib/libboost_") or re.match(r"lib/cmake/(?:Boost|boost_[A-Za-z0-9_]+)-1\.83\.0/", relative)):
        return None
    if "_boost_installation" not in spec:
        try:
            record = json.loads((cache / "audit-boost-install/installation-v1.json").read_text())
        except (OSError, ValueError) as exc:
            raise RuntimeError("实际Boost安装输入缺少B2安装回执：" + str(selected)) from exc
        from scripts.prepare_slam2d_dependencies import BOOST_URL, BOOST_SHA256
        source = cache / "slam2d-dependencies/boost-source/boost_1_83_0"
        if record["prefix"] != str(prefix) or record["source"] != str(source) or record["url"] != BOOST_URL or record["sha256"] != BOOST_SHA256 or "--prefix=$CACHE/prefix" not in record["arguments"] or "install" not in record["arguments"] or common.sha256(source / "b2") != record["b2_sha256"]:
            raise RuntimeError("实际Boost安装回执与固定来源/前缀不符")
        spec["_boost_installation"] = record
    record = spec["_boost_installation"]
    expected = record["files"].get(relative)
    if relative.startswith("include/boost/"):
        original = Path(record["source"]) / relative[len("include/"):]
        expected = common.sha256(original) if original.is_file() else None
    if expected is None or not selected.is_file() or common.sha256(selected) != expected:
        raise RuntimeError("实际Boost安装输入未被精确安装回执/源码副本覆盖：" + str(selected))
    return {"kind": "pinned-source", "id": "boost-1.83.0", "root": record["source"], "url": record["url"], "sha256": record["sha256"], "installation_components": record["components"], "installation_receipt": "$CACHE/audit-boost-install/installation-v1.json"}


def boost_native_evidence(private, cache):
    libraries = {str(path): path for name, path in private.items() if name.startswith("libboost_")}
    if not libraries:
        return {"links": [], "objects": []}
    records = [json.loads(path.read_text()) for path in (cache / "audit-boost-commands").glob("*.json") if path.name != "complete-v1.json"]
    compiles = {item["output"]: item for item in records if item.get("kind") == "compile" and item.get("success")}
    result, objects = [], {}
    for path in libraries.values():
        digest = common.sha256(path)
        build_ids = re.findall(r"Build ID: (\S+)", capture(["readelf", "--notes", str(path)]))
        matches = [item for item in records if item.get("kind") == "link" and item.get("success") and (item["output_sha256"] == digest or build_ids and item.get("build_id") == build_ids)]
        if len(matches) != 1:
            raise RuntimeError("Boost私有库缺少唯一的实际编译器/链接图回执：" + str(path))
        link = matches[0]
        map_file = Path(link["link_map"])
        if not map_file.is_file() or common.sha256(map_file) != link["link_map_sha256"]:
            raise RuntimeError("Boost私有库链接图不存在或被改变")
        map_text = map_file.read_text()
        for adopted in link["input_objects"]:
            obj = Path(adopted["path"])
            compiled = compiles.get(str(obj))
            if compiled is None or compiled.get("output_sha256") != adopted["sha256"] or not obj.is_file() or common.sha256(obj) != adopted["sha256"]:
                raise RuntimeError("Boost实际采用对象无法绑定到编译回执：" + str(obj))
            if str(obj) not in map_text:
                raise RuntimeError("Boost对象未出现在该实际共享库链接图：" + str(obj))
            dependency = Path(compiled["dependency_record"])
            if not dependency.is_file() or common.sha256(dependency) != compiled["dependency_sha256"]:
                raise RuntimeError("Boost实际对象的MD依赖记录缺失或改变")
            objects[str(obj)] = {"object": str(obj), "object_sha256": adopted["sha256"], "dependency_record": str(dependency), "dependency_sha256": compiled["dependency_sha256"], "working_directory": compiled["working_directory"], "compiler_arguments": compiled["compiler_arguments"], "library": str(path)}
        if not link["input_objects"]:
            raise RuntimeError("Boost共享库实际链接没有可核对的原生对象")
        result.append(link)
    return {"links": result, "objects": sorted(objects.values(), key=lambda item: item["object"])}


def dependency_records(program, cache, spec, linked):
    target = rust_target(program, cache)
    records = [target / "release" / (program + ".d")]
    record_cwds = {}
    rust_records = {}
    packages = {item["id"]: item for item in spec["cargo_packages"]}
    artifacts = linked.get("compiler_artifacts", [])
    if not artifacts:
        raise RuntimeError("缺少本次Cargo精确产物事件，拒绝扫描共享target推断版本")
    artifact_owners = {}
    for artifact in artifacts:
        package = packages.get(artifact["package_id"])
        if package is None or Path(artifact["manifest_path"]).resolve() != Path(package["manifest_path"]).resolve():
            raise RuntimeError("本次Cargo产物不属于当前固定依赖闭包：" + artifact["package_id"])
        package_root = Path(package["manifest_path"]).parent.resolve()
        crate = artifact["target"]["name"].replace("-", "_")
        candidates = set()
        for filename in artifact["filenames"]:
            path = Path(filename)
            artifact_owners[str(path.resolve())] = package
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
    spec["_artifact_owners"] = artifact_owners
    codegen_owners = {}
    for command in linked["compiler_commands"]:
        if "Running `" not in command:
            continue
        arguments = shlex.split(command.split("Running `", 1)[1].rsplit("`", 1)[0])
        if "--crate-name" not in arguments or "--out-dir" not in arguments:
            continue
        crate = arguments[arguments.index("--crate-name") + 1]
        out_dir = Path(arguments[arguments.index("--out-dir") + 1])
        suffix = next((value.split("=-", 1)[1] for value in arguments if value.startswith("extra-filename=-")), None)
        manifest_dir = next((value.split("=", 1)[1] for value in arguments if value.startswith("CARGO_MANIFEST_DIR=")), None)
        owners = {event["package_id"]: packages[event["package_id"]] for event in artifacts if event["target"]["name"].replace("-", "_") == crate and str(Path(event["manifest_path"]).parent) == manifest_dir}
        if suffix and len(owners) == 1:
            codegen_owners[str((out_dir / (crate + "-" + suffix)).resolve())] = next(iter(owners.values()))
    spec["_codegen_owners"] = codegen_owners
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
    adoption = archive_adoption_evidence(program, cache, spec, linked)
    # 原始.a即便被rustc bundle进rlib，也必须沿实际成员SHA链回到CMake。
    actual.extend(item["archive"] for item in adoption["adopted_native_objects"])
    native, private = native_link_evidence(program, cache, spec, linked, actual)
    cmake_cwds = {}
    for entry in native:
        records.extend((Path(entry["cmake_root"]) / "CMakeFiles").rglob("Makefile.cmake"))
        for item in entry["inputs"]:
            if item["kind"] == "object":
                obj = Path(item["path"])
                record = object_dependency(obj)
                records.append(record)
                record_cwds[str(record.resolve())] = entry["working_directory"]
                cmake_cwds[str(obj)] = entry["working_directory"]
    for item in adoption["adopted_native_objects"]:
        record = Path(item["dependency_record"])
        item["working_directory"] = cmake_cwds.get(item["object"], item["working_directory"])
        item["backend"] = "cmake" if item["object"] in cmake_cwds else "build-script-static-archive"
        records.append(record)
        record_cwds[str(record.resolve())] = item["working_directory"]
    boost = boost_native_evidence(private, cache)
    for item in boost["objects"]:
        record = Path(item["dependency_record"])
        records.append(record)
        record_cwds[str(record.resolve())] = item["working_directory"]
    linked["native_adoption"] = adoption
    linked["boost_adoption"] = boost
    return sorted({path for path in records if path.is_file()}), record_cwds


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
    value = re.sub(r"/tmp/rustc[^/ ]+", "$RUSTC_TEMP", value)
    value = re.sub(r"-[0-9a-f]{32}(?=/|$)", "-<slot>", value)
    value = re.sub(r"-[0-9a-f]{16}(?=[./]|$)", "-<artifact>", value)
    value = re.sub(r"(?<=/)[0-9a-f]{16}-", "<cc-object>-", value)
    return value


def link_arguments(logged, program, binary, cache, link_map=None):
    candidates = []
    compiler = []
    artifacts = []
    for line in logged.splitlines():
        if line.startswith("{"):
            try:
                event = json.loads(line)
            except ValueError:
                event = {}
            if event.get("reason") == "compiler-artifact":
                artifacts.append(event)
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


def source_owner(path, spec, root, cache, program):
    selected = Path(path).resolve()
    for item in spec["cargo_packages"]:
        directory = Path(item["manifest_path"]).parent.resolve()
        if selected == directory or directory in selected.parents:
            return {"kind": "crate" if item["source"] else "repository", "id": item["id"] if item["source"] else item["name"], "root": str(directory), "license": item["license"], "version": item["version"]}
    if selected == root or root in selected.parents:
        return {"kind": "repository", "id": selected.relative_to(root).as_posix(), "root": str(root)}
    artifact_package = spec.get("_artifact_owners", {}).get(str(selected))
    if artifact_package is None and ("/release/deps/" in str(selected) or str(selected).startswith("/tmp/rustc")):
        matches = {item["id"]: item for name, item in spec.get("_artifact_owners", {}).items() if Path(name).name == selected.name}
        if len(matches) == 1:
            artifact_package = next(iter(matches.values()))
        elif matches:
            raise RuntimeError("实际链接Cargo产物无法唯一匹配本次包身份：" + str(selected))
    codegen = re.fullmatch(r"([A-Za-z_][A-Za-z0-9_]*-[0-9a-f]{16})\..*\.rcgu\.o", selected.name)
    if artifact_package is None and codegen:
        artifact_package = spec.get("_codegen_owners", {}).get(str(selected.parent / codegen.group(1)))
    if artifact_package:
        item = artifact_package
        return {"kind": "compiled-crate", "id": item["id"] if item["source"] else item["name"], "root": str(Path(item["manifest_path"]).parent), "license": item["license"], "version": item["version"]}
    # Cargo的OUT_DIR是生成输入；来源绑定到实际执行build.rs的包，而非整个cache。
    match = re.search(r"/release/build/([^/]+)-[0-9a-f]+/out(?:/|$)", str(selected))
    if match:
        package = next((item for item in spec["cargo_packages"] if item["name"] == match.group(1)), None)
        if package:
            native = Path(package["manifest_path"]).parent
            upstream = pinned_cmake_sources(spec, root) if "/_deps/" in str(selected) else []
            for origin in upstream:
                prefix = str(selected).split("/_deps/", 1)[0]
                marker = "/_deps/" + origin["name"].lower()
                if marker + "-src/" in str(selected) or marker + "-build/" in str(selected):
                    return {"kind": "pinned-source", "id": origin["name"], "root": prefix + marker + "-src", "url": origin["url"], "sha256": origin["sha256"], "recipe": origin["recipe"]}
            if "/_deps/" in str(selected):
                raise RuntimeError("生成目录中的外部源码缺少受指纹约束的固定来源：" + str(selected))
            return {"kind": "generated", "id": package["name"], "root": str(native), "upstream": upstream}
    for source_cache in (cache, cache / "slam2d-dependencies"):
        registry = source_cache / "sources/source-paths-v1.json"
        if registry.is_file():
            for role, entry in json.loads(registry.read_text())["slots"].items():
                directory = source_cache / "sources" / entry["path"]
                if selected == directory or directory in selected.parents:
                    return {"kind": "pinned-source", "id": role, "root": str(directory), "repository": entry["url"], "revision": entry["revision"]}
    if "boost-source/boost_1_83_0" in str(selected):
        from scripts.prepare_slam2d_dependencies import BOOST_URL, BOOST_SHA256
        directory = Path(str(selected).split("boost_1_83_0", 1)[0] + "boost_1_83_0")
        return {"kind": "pinned-source", "id": "boost-1.83.0", "root": str(directory), "url": BOOST_URL, "sha256": BOOST_SHA256}
    if cache / "build/slamtec" in selected.parents:
        return {"kind": "generated", "id": "slamtec-driver", "root": str(root / "drivers/sensors/slamtec_native")}
    if cache / "slam2d-dependencies/build" in selected.parents:
        for parent in selected.parents:
            cmake_cache = parent / "CMakeCache.txt"
            if cmake_cache.is_file():
                match = re.search(r"^CMAKE_HOME_DIRECTORY:INTERNAL=(.+)$", cmake_cache.read_text(), re.MULTILINE)
                if match:
                    origin = source_owner(Path(match.group(1)), spec, root, cache, program)
                    return {**origin, "generated_from": origin["root"]}
            if parent == cache / "slam2d-dependencies/build":
                break
        if cache / "slam2d-dependencies/build/boost" in selected.parents:
            from scripts.prepare_slam2d_dependencies import BOOST_URL, BOOST_SHA256
            return {"kind": "pinned-source", "id": "boost-1.83.0", "root": str(cache / "slam2d-dependencies/boost-source/boost_1_83_0"), "url": BOOST_URL, "sha256": BOOST_SHA256}
    if any(str(selected).startswith(prefix) for prefix in ("/usr/", "/lib/", "/lib64/")):
        return {"kind": "system", "id": str(selected), "root": "/usr/share/doc"}
    rustup = os.environ.get("RUSTUP_HOME", "/nonexistent-rustup")
    if str(selected).startswith(rustup + "/") or str(selected).startswith("/tmp/rustc"):
        return {"kind": "rust-toolchain", "id": "Rust-" + builds.RUST_VERSION, "root": rustup}
    if selected == cache / "prefix" or cache / "prefix" in selected.parents:
        text = str(selected)
        boost_owner = boost_installation_owner(selected, spec, cache)
        if boost_owner:
            return boost_owner
        role = "slam2d-ceres" if "ceres" in text else "slam2d-onetbb" if "tbb" in text else None
        registry = cache / "slam2d-dependencies/sources/source-paths-v1.json"
        if role and registry.is_file():
            entry = json.loads(registry.read_text())["slots"][role]
            return {"kind": "pinned-source", "id": role, "root": str(registry.parent / entry["path"]), "repository": entry["url"], "revision": entry["revision"]}
        directory = root / ("drivers/sensors/slamtec_native" if "slamtec" in text else common.WORKSPACE + "/crates/nuwax-slam-toolbox-sys/native" if "slam_toolbox" in text else "")
        if directory != root:
            return {"kind": "repository", "id": directory.relative_to(root).as_posix(), "root": str(directory)}
    if str(selected).startswith(("/opt/protoc/", "/opt/lcm-gen-1.5.2/")):
        return {"kind": "build-tool", "id": selected.name, "root": str(selected.parent.parent)}
    raise RuntimeError("实际编译/链接输入无法映射到受审源码来源：" + str(selected))


def collect_audit(program, spec, checked, linked, root, cache):
    link_inputs = resolve_link_inputs(linked["linker_arguments"], root)
    adoption = linked.get("native_adoption", {})
    actual = checked["external_inputs"] + [item["archive"] for item in adoption.get("adopted_native_objects", [])]
    native_links, private = native_link_evidence(program, cache, spec, linked, actual)
    for record in native_links:
        link_inputs.extend(record["inputs"])
    for item in adoption.get("prebuilt_native_members", []):
        link_inputs.append({"kind": "adopted-prebuilt-native-archive", "path": item["stable_archive"], "member": item["member"], "exists_after_link": True})
    for item in adoption.get("adopted_native_objects", []):
        link_inputs.append({"kind": "adopted-native-object", "path": item["object"], "object_sha256": item["object_sha256"], "exists_after_link": True})
    for item in linked.get("boost_adoption", {}).get("objects", []):
        link_inputs.append({"kind": "adopted-native-object", "path": item["object"], "object_sha256": item["object_sha256"], "exists_after_link": True})
    runtime = {}
    for binary in [cache / "prefix/bin" / program, *private.values()]:
        listing = capture(["ldd", str(binary)])
        if "not found" in listing:
            raise RuntimeError("实际程序私有库闭包有无法解析的动态依赖")
        for name, path in re.findall(r"^\s*(\S+)\s+=>\s+(/\S+)", listing, re.MULTILINE):
            runtime[str(Path(path).resolve())] = {"kind": "transitive-runtime-library", "name": name, "path": str(Path(path).resolve()), "exists_after_link": True}
        for path in re.findall(r"^\s*(/\S+)\s+\(", listing, re.MULTILINE):
            runtime[str(Path(path).resolve())] = {"kind": "runtime-loader", "path": str(Path(path).resolve()), "exists_after_link": True}
    link_inputs.extend(runtime.values())
    private_records = []
    for name, path in sorted(private.items()):
        symbols = capture(["readelf", "--wide", "--symbols", str(path)])
        units = sorted(set(re.findall(r"FILE\s+LOCAL\s+DEFAULT\s+ABS\s+(.+)$", symbols, re.MULTILINE)))
        dynamic = capture(["readelf", "--dynamic", str(path)])
        notes = capture(["readelf", "--notes", str(path)])
        origin = source_owner(path, spec, root, cache, program)
        record = {"name": name, "path": str(path), "sha256": common.sha256(path), "needed": re.findall(r"\(NEEDED\).*\[([^]]+)\]", dynamic), "build_id": re.findall(r"Build ID: (\S+)", notes), "elf_compilation_units": units, "origin": origin, "load": "explicit-dlopen" if name.startswith("libnuwax_slamtec") else "dynamic-dependency"}
        if name.startswith("libboost_"):
            evidence = [item for item in linked["boost_adoption"]["objects"] if item["library"] == str(path)]
            if not evidence:
                raise RuntimeError("Boost私有库缺少真实MD/对象SHA/链接图证据")
            record["native_object_evidence"] = evidence
            record["dependency_basis"] = "compiler-MD + object-SHA + actual-link-map"
        private_records.append(record)
        link_inputs.append({"kind": "runtime-private-library", "name": name, "path": str(path), "exists_after_link": True})
    checked["external_inputs"] = sorted(set(checked["external_inputs"]))
    origins = {}
    inputs = checked["repository_inputs"] + checked["external_inputs"] + [item["path"] for item in link_inputs]
    for value in inputs:
        path = root / value if not Path(value).is_absolute() else Path(value)
        try:
            relative = path.resolve().relative_to(root).as_posix()
        except ValueError:
            relative = None
        if relative is not None and not common.covered(relative, spec):
            raise RuntimeError("程序指纹漏掉实际链接输入：" + relative)
        owner = source_owner(path, spec, root, cache, program)
        owner_key = owner["kind"] + ":" + owner["id"]
        origins[owner_key] = owner
    return {**checked, "schema": 1, "program": program, "compiler_commands": linked["compiler_commands"], "compiler_artifacts": linked["compiler_artifacts"], "compiled_binary_sha256": linked["binary_sha256"], "link_record_origin": "executed" if linked["captured_this_build"] else "binary-sha-matched-cache", "linker_arguments": linked["linker_arguments"], "link_inputs": link_inputs, "native_link_records": native_links, "native_archive_adoption": adoption, "rust_dependency_record_origins": linked.get("rust_dependency_record_origins", []), "boost_native_adoption": linked.get("boost_adoption", {}), "boost_installation": spec.get("_boost_installation"), "private_library_records": private_records, "external_sources": sorted(origins.values(), key=lambda item: (item["kind"], item["id"])), "cargo_resolution": spec["cargo_packages"], "source_revision": capture(["git", "-C", str(root), "rev-parse", "HEAD"])}


def review_key(audit, root, cache):
    def normal(value):
        value = canonical(value, root, cache)
        value = re.sub(r"[^/ ]+\.rcgu\.o", "<rust-codegen>.o", value)
        value = re.sub(r"\$RUSTC_TEMP/[^/ ]+", "$RUSTC_TEMP/<generated-object>", value)
        return value
    payload = {
        "repository_inputs": sorted(audit["repository_inputs"]),
        "external_sources": sorted({json.dumps({key: normal(str(value)) for key, value in source.items() if key != "root"}, sort_keys=True) for source in audit["external_sources"]}),
        "link_inputs": sorted({normal(item["path"]) for item in audit["link_inputs"]}),
        "licenses": sorted((item["source"], item["sha256"]) for item in audit["license_files"]),
        "system_packages": audit["system_packages_used"],
    }
    return hashlib.sha256(json.dumps(payload, sort_keys=True, ensure_ascii=False).encode()).hexdigest()


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
    enable_ci_native_diagnostics()
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
                        audit = json.loads((unpacked / "share/audit/inputs.json").read_text())
                        dump(audit_cache, audit)
                audit = json.loads(audit_cache.read_text())
                if audit.get("program") != program or audit.get("fingerprint") != value or not audit.get("review_key"):
                    raise RuntimeError("已有程序缺少可核查的实际输入审计")
                dump(output / ("audit-" + program + ".json"), audit)
                programs[program] = {**existing, "fingerprint": value, "input_spec": public_spec, "audit": {"review_key": audit["review_key"], "coverage": "complete"}}
                reused.append(program)
                print("云端跳过同指纹程序：" + program, flush=True)
                continue
            link_map = cache / "audit-linkmaps" / (program + ".map")
            link_map.parent.mkdir(parents=True, exist_ok=True)
            log_path = output / (program + ".build.log")
            with log_path.open("w", encoding="utf-8") as log:
                def recording_run(argv, *, cwd=None, env=None):
                    with diagnostic_command(argv, cwd, cache, link_map) as actual_argv:
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
            generated = [cache / path for path in ("build", "sources", "slam2d-dependencies", "prefix", "cargo/registry", "cargo/git")]
            generated.extend((Path(os.environ["RUSTUP_HOME"]), Path("/opt/protoc"), Path("/opt/lcm-gen-1.5.2")))
            dependency_artifacts = []
            for record in records:
                name = hashlib.sha256(str(record).encode()).hexdigest() + record.suffix
                saved = output / "native-dependencies" / program / name
                saved.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(record, saved)
                dependency_artifacts.append({"source": str(record), "artifact": saved.relative_to(output).as_posix(), "sha256": common.sha256(record), "working_directory": record_cwds.get(str(record.resolve()), str(root))})
            for native_map in linked["boost_adoption"]["links"]:
                saved = output / "native-maps" / (program + "-" + Path(native_map["link_map"]).name)
                saved.parent.mkdir(exist_ok=True)
                shutil.copyfile(native_map["link_map"], saved)
            checked = common.audit_dependencies(root, spec, records, generated_roots=generated, record_cwds=record_cwds)
            dump(output / ("audit-" + program + ".json"), {**checked, "link_record": linked, "status": "source-mapping-in-progress"})
            audit = collect_audit(program, spec, checked, linked, root, cache)
            audit.update(platform=args.platform, fingerprint=value, dependency_record_artifacts=dependency_artifacts)
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
    command = commands.add_parser("native-compiler")
    command.add_argument("--compiler", required=True)
    command.add_argument("--records", type=Path, required=True)
    command.add_argument("--map-dir", type=Path, required=True)
    command.add_argument("arguments", nargs=argparse.REMAINDER)
    command = commands.add_parser("publish")
    command.add_argument("--repository", required=True)
    command.add_argument("--revision", required=True)
    command.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.command == "native-compiler":
        return native_compiler(args)
    if args.command == "probe":
        probe(args.output)
    else:
        globals()[args.command](args)


if __name__ == "__main__":
    raise SystemExit(main())
