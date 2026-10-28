#!/usr/bin/env python3
"""预编译发布前的隔离行为检查：增量、零下载、坏附件回退与漏输入拒绝。"""
from __future__ import annotations

import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile
import urllib.error
from unittest import mock

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from scripts import neh_prebuilt as common
from scripts.module_installer import builds


def run(argv, **kwargs):
    subprocess.run(list(map(str, argv)), check=True, stdout=subprocess.DEVNULL, **kwargs)


def commit(root):
    run(["git", "-C", root, "add", "."])
    run(["git", "-C", root, "-c", "core.hooksPath=/dev/null", "-c", "commit.gpgsign=false", "commit", "-qm", "隔离测试"])
    return subprocess.check_output(["git", "-C", str(root), "rev-parse", "HEAD"], text=True).strip()


def main():
    with tempfile.TemporaryDirectory(prefix="neh-prebuilt-check-") as temporary:
        base = Path(temporary)
        root, prefix, cache = base / "source", base / "prefix", base / "cache"
        root.mkdir()
        prefix.mkdir()
        cache.mkdir()
        run(["git", "init", "-q", root])
        run(["git", "-C", root, "config", "user.name", "预编译隔离验证"])
        run(["git", "-C", root, "config", "user.email", "isolated@example.invalid"])
        specs = {}
        for name in common.PROGRAMS:
            directory = root / name
            directory.mkdir()
            (directory / "input.cpp").write_text("// 初始输入\n", encoding="utf-8")
            specs[name] = {"paths": [name], "exclude": []}
        revision = commit(root)
        identity = {"compiler": "隔离夹具"}
        original = {name: common.fingerprint(root, name, "noble-x86_64", spec, identity) for name, spec in specs.items()}
        (root / "nuwax-person-follow/input.cpp").write_text("// 人物跟随输入已改变\n", encoding="utf-8")
        revision = commit(root)
        changed = {name: common.fingerprint(root, name, "noble-x86_64", spec, identity) for name, spec in specs.items()}
        assert [name for name in original if changed[name] != original[name]] == ["nuwax-person-follow"], "T1：只允许人物跟随指纹改变"
        assert changed == {name: common.fingerprint(root, name, "noble-x86_64", spec, identity) for name, spec in specs.items()}, "T2：相同输入指纹必须稳定"

        program = "nuwax-person-follow"
        value = changed[program]
        staging = base / "staging"
        (staging / "bin").mkdir(parents=True)
        shutil.copyfile(shutil.which("true"), staging / "bin" / program)
        (staging / "bin" / program).chmod(0o755)
        notices = common.program_share(program) + "/licenses"
        (staging / notices).mkdir(parents=True)
        (staging / notices / "LICENSE").write_text("隔离制品许可夹具\n", encoding="utf-8")
        files = ["bin/" + program, notices + "/LICENSE"]
        manifest = {"schema": common.SCHEMA, "program": program, "fingerprint": value, "files": common._receipt_files(staging, files)}
        (staging / "prebuilt-manifest.json").write_text(json.dumps(manifest))
        raw = base / "program.tar"
        with tarfile.open(raw, "w") as archive:
            for file in files + ["prebuilt-manifest.json"]:
                archive.add(staging / file, arcname=file)
        archive = base / "program.tar.zst"
        run(["zstd", "-q", str(raw), "-o", archive])
        entry = {"fingerprint": value, "asset": program + "-" + value + ".tar.zst", "sha256": common.sha256(archive), "size": archive.stat().st_size, "input_spec": specs[program]}
        index = {"schema": common.SCHEMA, "revision": revision, "platforms": {"noble-x86_64": {"build_identity": identity, "programs": {program: entry}}}}
        config = {"revision": revision, "repository": "example/repository"}
        payload = archive.read_bytes()
        original_which = shutil.which
        def without_zstd(name):
            return None if name == "zstd" else original_which(name)
        # 老设备未安装zstd命令时，必须通过基础系统libzstd完成真实归档往返。
        with mock.patch.object(common.shutil, "which", side_effect=without_zstd), mock.patch.object(common, "_urlopen", return_value=io.BytesIO(payload)):
            common.install_archive(root, prefix, cache, program, entry, config["repository"])
        assert common.reusable(prefix, cache / "prebuilt/receipts" / (program + ".json"), value)
        damaged = base / "truncated.tar.zst"
        damaged.write_bytes(payload[:-4])
        with mock.patch.object(common.shutil, "which", side_effect=without_zstd), tempfile.TemporaryFile() as decoded:
            try:
                common._decode_zstd(damaged, decoded)
            except ValueError:
                pass
            else:
                raise AssertionError("libzstd必须拒绝截断归档")
        with mock.patch.object(common.shutil, "which", side_effect=without_zstd), mock.patch.object(common, "_MAX_EXPANDED", 64), tempfile.TemporaryFile() as decoded:
            try:
                common._decode_zstd(archive, decoded)
            except ValueError as exc:
                assert "超限" in str(exc)
            else:
                raise AssertionError("libzstd必须执行解码大小上限")
        with mock.patch.object(common.ctypes, "CDLL", side_effect=OSError("libzstd missing")), tempfile.TemporaryFile() as decoded:
            try:
                common._decode_zstd_library(archive, decoded)
            except common.PrebuiltUnavailable:
                pass
            else:
                raise AssertionError("基础库缺失必须允许本机编译回退")
        with mock.patch.object(common, "platform_key", return_value="noble-x86_64"), mock.patch.object(common, "fetch_index", return_value=index), mock.patch.object(common, "_urlopen", side_effect=AssertionError("T2：有效本机制品不能下载")):
            resolved, _ = common.prepare_prebuilt(root, prefix, cache, [program], config)
            assert set(resolved) == {program}

        compiled = []
        target = base / "rust-target"
        def rust_build(command, **kwargs):
            compiled.append(tuple(command))
            binary = target / "release" / program
            binary.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(original_which("true"), binary)
            binary.chmod(0o755)

        for scenario in ("missing", "corrupt"):
            (prefix / "bin" / program).unlink()
            cached = cache / "prebuilt/archives" / entry["asset"]
            cached.unlink(missing_ok=True)
            failure = urllib.error.HTTPError("https://github.com/example/repository", 404, "missing", {}, None)
            effect = failure if scenario == "missing" else None
            manifest = root / common.PROGRAMS[program]["manifest"]
            manifest.parent.mkdir(parents=True, exist_ok=True)
            manifest.with_name("Cargo.lock").write_text("# 隔离调度夹具\n")
            with mock.patch.object(common, "platform_key", return_value="noble-x86_64"), mock.patch.object(common, "fetch_index", return_value=index), mock.patch.object(common, "_urlopen", side_effect=effect, return_value=io.BytesIO(b"tampered")), mock.patch.object(builds, "_rust_toolchain", return_value=(Path("cargo"), {})), mock.patch.object(builds, "_protocol_compiler", return_value={}), mock.patch.object(builds, "_rust_target", return_value=target):
                result = builds.build_modules(root, prefix, cache, ["follow"], 1, rust_build, prebuilt=config)
                assert set(result) == {program}
                assert (prefix / "bin" / program).read_bytes()[:4] == b"\x7fELF"
            assert not cached.exists(), "T3/T4：坏下载不能成为有效制品缓存"
        assert len(compiled) == 2 and all("--package" in command and "nuwax-nav-visual-servoing" in command for command in compiled), "T3/T4：必须回到人物跟随Rust编译入口"
        # 这里只验证调度；独立云端故障作业使用真正Rust编译器，不替换构建器。
        assert common.eligible_program_names(["fastlio2", "pointlio"]) == ()
        assert set(common.program_names(["fastlio2", "pointlio"])) == set(common.LOCAL_PROGRAMS)
        def local_native(_root, _prefix, _cache, name, _jobs, _run):
            path = _prefix / "bin" / (name + "_native")
            shutil.copyfile(original_which("true"), path)
            path.chmod(0o755)
            return path
        with mock.patch.object(common, "fetch_index", side_effect=AssertionError("LIO-only不允许读取索引")), mock.patch.object(builds, "_build_native", side_effect=local_native):
            result = builds.build_modules(root, prefix, cache, ["pointlio"], 1, run, prebuilt=config)
            assert set(result) == {"pointlio_native"}
        try:
            common.install_archive(root, prefix, cache, "pointlio_native", entry, config["repository"])
        except ValueError:
            pass
        else:
            raise AssertionError("LIO不能安装任何预编译附件")

        # T5通过真实编译器依赖记录发现未声明的跨目录头文件。
        (root / "declared").mkdir()
        (root / "outside").mkdir()
        (root / "outside/header.h").write_text("#define NUMBER 7\n")
        (root / "declared/main.c").write_text('#include "../outside/header.h"\nint main(void){return NUMBER;}\n')
        record = base / "actual.d"
        run(["cc", "-MMD", "-MF", record, "-c", root / "declared/main.c", "-o", base / "actual.o"])
        try:
            common.audit_dependencies(root, {"paths": ["declared"], "exclude": []}, [record])
        except RuntimeError as exc:
            assert "outside/header.h" in str(exc)
        else:
            raise AssertionError("T5：漏掉编译输入必须失败")
        report = common.audit_dependencies(root, {"paths": ["declared", "outside"], "exclude": []}, [record])
        assert report["coverage"] == "complete"

        # 登记的是固定环境根，Boost配置名和新子路径无需逐文件安装回执。
        environment = base / "fixed-environment"
        headers = [environment / "lib/cmake/Boost-1.83.0/BoostConfig.cmake", environment / "lib/cmake/BoostDetectToolset-1.83.0.cmake", environment / "new/arbitrary/input.inc"]
        for index, header in enumerate(headers):
            header.parent.mkdir(parents=True, exist_ok=True)
            header.write_text("#define ENV_NUMBER_" + str(index) + " 7\n")
        source = root / "declared/environment.c"
        source.write_text("".join('#include "' + str(path) + '"\n' for path in headers) + "int value(void){return ENV_NUMBER_0+ENV_NUMBER_1+ENV_NUMBER_2;}\n")
        external_record = base / "external.d"
        run(["cc", "-MD", "-MF", external_record, "-c", source, "-o", base / "external.o"])
        system_roots = [Path("/usr"), Path("/lib"), Path("/lib64")]
        try:
            common.audit_dependencies(root, {"paths": ["declared"], "exclude": []}, [external_record], environment_roots=system_roots)
        except RuntimeError as exc:
            assert str(environment) in str(exc)
        else:
            raise AssertionError("登记根之外的真实编译输入必须拒绝")
        checked = common.audit_dependencies(root, {"paths": ["declared"], "exclude": []}, [external_record], environment_roots=[*system_roots, environment])
        assert all(str(path) in checked["external_inputs"] for path in headers)

        with mock.patch.object(common, "_urlopen", side_effect=urllib.error.HTTPError("https://github.com/", 404, "pending", {}, None)):
            try:
                common.fetch_index("example/repository", revision)
            except common.PrebuiltPending as exc:
                assert exc.as_dict()["status"] == "prebuilt_pending"
            else:
                raise AssertionError("索引404必须明确表示云端编译中")
        with mock.patch.object(common, "_urlopen", side_effect=OSError("network unavailable")):
            try:
                common.fetch_index("example/repository", revision)
            except common.PrebuiltUnavailable:
                pass
            else:
                raise AssertionError("网络故障必须与云端待编译分开")
    print("预编译隔离行为检查通过：T1/T2指纹与零下载；T3/T4本机回退；T5真实编译器漏输入拒绝；登记环境根核查；索引等待与网络故障区分。")


if __name__ == "__main__":
    main()
