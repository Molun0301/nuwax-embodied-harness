#!/usr/bin/env python3
"""在两个目标环境逐程序编译、审计、封装并发布专用Release。"""
from __future__ import annotations

import argparse
import datetime
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile

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
        "parameters": {"cargo": ["build", "--locked", "--release"], "cmake": ["CMAKE_BUILD_TYPE=Release", "CMAKE_GENERATOR=Unix Makefiles"], "environment": {name: os.environ.get(name, "") for name in ("RUSTFLAGS", "CFLAGS", "CXXFLAGS", "LDFLAGS", "NUWAX_CXX20")}},
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


def copy_notices(root, prefix, cache, staging, spec):
    notice_root = staging / "share/licenses"
    notice_root.mkdir(parents=True)
    source_roots = [root / path for path in spec["paths"] if (root / path).is_dir()]
    source_roots.extend([cache / "sources", cache / "build", cache / "slam2d-dependencies", Path(os.environ.get("CARGO_HOME", str(Path.home() / ".cargo"))) / "registry/src"])
    source_roots.extend(path for path in (prefix / "share", Path("/usr/share/doc")) if path.is_dir())
    copied = set()
    for source in (root / "LICENSE", root / "THIRD_PARTY_NOTICES.md"):
        if source.is_file():
            shutil.copyfile(source, notice_root / source.name)
    third_party = root / "docs/公开文档/第三方许可证说明.md"
    if third_party.is_file():
        shutil.copyfile(third_party, notice_root / third_party.name)
    common_licenses = notice_root / "通用许可"
    common_licenses.mkdir()
    for name in ("GPL-2", "GPL-3", "LGPL-2.1", "LGPL-3", "Apache-2.0", "BSD"):
        source = Path("/usr/share/common-licenses") / name
        if source.is_file():
            shutil.copyfile(source, common_licenses / name)
    if not (notice_root / "LICENSE").is_file():
        raise RuntimeError("发布程序缺少项目LICENSE")
    count = 0
    for index, directory in enumerate(source_roots):
        if not directory.exists():
            continue
        for base, folders, files in os.walk(directory, followlinks=False):
            folders[:] = [name for name in folders if name not in {".git", "incremental", "node_modules"}]
            for name in files:
                if not (name.upper().startswith(("LICENSE", "LICENCE", "COPYING", "NOTICE")) or name == "copyright"):
                    continue
                source = Path(base) / name
                resolved = source.resolve()
                if resolved in copied or not resolved.is_file():
                    continue
                copied.add(resolved)
                destination = notice_root / str(index) / source.relative_to(directory)
                destination.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(source, destination)
                count += 1
    return sum(1 for path in notice_root.rglob("*") if path.is_file())


def package(root, prefix, cache, output, program, fingerprint_value, spec):
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
        notices = copy_notices(root, prefix, cache, staging, spec)
        revision = capture(["git", "-C", str(root), "rev-parse", "HEAD"])
        repository = capture(["git", "-C", str(root), "remote", "get-url", "origin"])
        source_url = repository[:-4] if repository.endswith(".git") else repository.rstrip("/")
        dump(staging / "share/sources/对应源码.json", {"revision": revision, "repository": repository, "source_archive": source_url + "/archive/" + revision + ".tar.gz", "input_spec": spec})
        if "native" in common.PROGRAMS[program]:
            module = common.PROGRAMS[program]["native"]
            for path in (common.NATIVE_ROOT + "/" + module + "/cpp", common.NATIVE_ROOT + "/common"):
                shutil.copytree(root / path, staging / "share/sources" / path)
        # LGPL组件源码及替换共享库所需的包装源码随程序一起提供。
        if program == "nuwax-slam-2d-worker":
            for path in ("services/nuwax-workspace/crates/nuwax-slam-toolbox-sys/native", "drivers/sensors/slamtec_native"):
                shutil.copytree(root / path, staging / "share/sources" / path)
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


def dependency_records(program, cache, spec):
    recipe = common.PROGRAMS[program]
    if "native" in recipe:
        base = cache / "build/native" / recipe["native"]
        records = list(base.rglob("*.o.d")) + list(base.rglob("Makefile.cmake"))
    else:
        manifest = recipe["manifest"]
        slots = json.loads((cache / "build/rust/target-paths-v1.json").read_text())["slots"]
        role = builds._TARGET_ROLES[manifest]
        target = cache / "build/rust" / slots[role]["path"]
        records = [target / "release" / (program + ".d")]
        # 编译器.d包含Rust闭包与build.rs声明；补核本闭包原生crate的C++头文件。
        metadata = json.loads(capture([shutil.which("cargo"), "metadata", "--locked", "--format-version", "1", "--no-deps", "--manifest-path", str(ROOT / manifest)]))
        packages = {item["name"] for item in metadata["packages"] if common.covered(Path(item["manifest_path"]).parent.relative_to(ROOT).as_posix(), spec)}
        for directory in (target / "release/build").iterdir():
            crate = directory.name.rsplit("-", 1)[0]
            if crate in packages:
                records.extend(directory.rglob("*.o.d"))
                records.extend(directory.rglob("Makefile.cmake"))
    return [path for path in records if path.is_file()]


def source_archives(root, cache, output, assets):
    result = {}
    for module in ("fastlio2", "pointlio"):
        lock = root / common.NATIVE_ROOT / module / "cpp/flake.lock"
        source = builds._locked_source(lock, "fast-lio")
        name = "source-" + module + "-" + source[1] + ".tar.gz"
        existing = asset_entry(assets[name]) if name in assets else None
        if existing:
            result[module] = {**existing, "revision": source[1], "repository": source[0]}
            continue
        checkout = builds._checkout(cache, source, run, role=module + "-algorithm")
        archive = output / name
        with archive.open("wb") as handle:
            subprocess.run(["git", "-C", str(checkout), "archive", "--format=tar.gz", "--prefix=" + module + "-" + source[1] + "/", source[1]], stdout=handle, check=True)
        result[module] = {"asset": name, "sha256": common.sha256(archive), "size": archive.stat().st_size, "revision": source[1], "repository": source[0]}
    return result


def build(args):
    root, cache, output = ROOT, args.cache.resolve(), args.output.resolve()
    if common.platform_key() != args.platform:
        raise RuntimeError("构建环境与所选目标平台不符")
    output.mkdir(parents=True, exist_ok=True)
    cache.mkdir(parents=True, exist_ok=True)
    prefix = cache / "prefix"
    prefix.mkdir(exist_ok=True)
    probe(output / "probe.json")
    cargo, build_identity = identity(args.platform, root)
    environment = {"CARGO_INCREMENTAL": "0", "CARGO_HOME": str(cache / "cargo")}
    target = {"focal-arm64": "aarch64-unknown-linux-gnu", "noble-x86_64": "x86_64-unknown-linux-gnu"}[args.platform]
    assets = {item["name"]: item for item in json.loads(args.assets.read_text(encoding="utf-8"))}
    programs, compiled, reused = {}, [], []
    specifications = {}
    for program, recipe in common.PROGRAMS.items():
        spec = common.discover_inputs(root, program, cargo, environment, target)
        specifications[program] = spec
        value = common.fingerprint(root, program, args.platform, spec, build_identity)
        name = program + "-" + value + ".tar.zst"
        existing = asset_entry(assets[name]) if name in assets else None
        if existing:
            programs[program] = {**existing, "fingerprint": value, "input_spec": spec}
            reused.append(program)
            print("云端跳过同指纹程序：" + program, flush=True)
            continue
        builds.build_modules(root, prefix, cache, [recipe["module"]], args.jobs, run, local_build=True, programs=[program])
        records = dependency_records(program, cache, spec)
        generated = [cache / path for path in ("build", "sources", "slam2d-dependencies", "prefix", "cargo/registry", "cargo/git")]
        generated.extend((Path(os.environ["RUSTUP_HOME"]), Path("/opt/protoc"), Path("/opt/lcm-gen-1.5.2")))
        audit = common.audit_dependencies(root, spec, records, generated_roots=generated)
        entry = package(root, prefix, cache, output, program, value, spec)
        programs[program] = {**entry, "fingerprint": value, "input_spec": spec, "audit": audit}
        compiled.append(program)
    sources = source_archives(root, cache, output, assets)
    revision = capture(["git", "rev-parse", "HEAD"], cwd=root)
    dump(output / ("platform-" + args.platform + ".json"), {"schema": common.SCHEMA, "revision": revision, "platform": args.platform, "build_identity": build_identity, "programs": programs, "source_archives": sources})
    dump(output / "build-report.json", {"compiled": compiled, "reused": reused, "counts": {"compiled": len(compiled), "reused": len(reused)}})
    # T1对照实际Cargo闭包：PointLIO输入不应落入其它6程序声明。
    pointlio = common.NATIVE_ROOT + "/pointlio/cpp/main.cpp"
    affected = [name for name, spec in specifications.items() if common.covered(pointlio, spec)]
    if affected != ["pointlio_native"]:
        raise RuntimeError("PointLIO独立增量范围错误：" + repr(affected))
    print("构建结果：" + json.dumps({"compiled": compiled, "reused": reused}, ensure_ascii=False), flush=True)


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
    dump(args.output, list_assets(args.repository, release["id"]))


def referenced_assets(index):
    result = {"index-" + index["revision"] + ".json"}
    for entry in index.get("platforms", {}).values():
        result.update(program["asset"] for program in entry.get("programs", {}).values())
    result.update(item["asset"] for item in index.get("source_archives", {}).values())
    return result


def publish(args):
    directory = args.output.resolve()
    indexes = [json.loads(path.read_text(encoding="utf-8")) for path in sorted(directory.rglob("platform-*.json"))]
    if len(indexes) != 2 or {item["platform"] for item in indexes} != common.PLATFORMS or any(item["revision"] != args.revision for item in indexes):
        raise RuntimeError("两平台完整提交索引不齐全，拒绝发布")
    payload = {"schema": common.SCHEMA, "revision": args.revision, "created_at": datetime.datetime.now(datetime.timezone.utc).isoformat(), "repository": args.repository, "platforms": {item["platform"]: {"build_identity": item["build_identity"], "programs": item["programs"]} for item in indexes}, "source_archives": indexes[0]["source_archives"]}
    for item in indexes:
        if set(item["programs"]) != set(common.PROGRAMS):
            raise RuntimeError("预编译索引缺少程序")
        if item["source_archives"] != payload["source_archives"]:
            raise RuntimeError("两平台固定源码归档摘要不符")
    try:
        release = gh_json(["api", "repos/" + args.repository + "/releases/tags/" + common.RELEASE])
    except subprocess.CalledProcessError:
        run(["gh", "release", "create", common.RELEASE, "--repo", args.repository, "--target", args.revision, "--prerelease", "--title", "NEH按程序预编译仓库", "--notes", "此专用Release供匿名设备按程序指纹下载；完整提交索引与固定第三方源码随附件保存。"])
        release = gh_json(["api", "repos/" + args.repository + "/releases/tags/" + common.RELEASE])
    if release["tag_name"] != common.RELEASE or release.get("draft"):
        raise RuntimeError("专用预编译Release身份错误")
    assets = list_assets(args.repository, release["id"])
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
        if path.is_file() and (path.name.endswith(".tar.zst") or path.name.startswith("source-") and path.name.endswith(".tar.gz")):
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
    command.add_argument("--cache", type=Path, required=True)
    command.add_argument("--output", type=Path, required=True)
    command.add_argument("--assets", type=Path, required=True)
    command.add_argument("--jobs", type=int, default=4)
    command = commands.add_parser("publish")
    command.add_argument("--repository", required=True)
    command.add_argument("--revision", required=True)
    command.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.command == "probe":
        probe(args.output)
    else:
        globals()[args.command](args)


if __name__ == "__main__":
    main()
