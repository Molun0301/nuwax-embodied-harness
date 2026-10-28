#!/usr/bin/env python3
"""以每运行、每场景独立Release验证匿名下载故障及真实本机编译回退。"""
from __future__ import annotations

import argparse
import contextlib
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import time
import urllib.error

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from scripts import neh_prebuilt as common
from scripts.module_installer import builds

VALIDATION_PREFIX = "neh-prebuilt-validation-"
# 沿用既有隔离编译缓存身份；该值不再用作任何Release名称。
CACHE_PURPOSE = "neh-prebuilt-validation-v1"
PRODUCTION_RELEASE = "neh-prebuilt-v1"
PROGRAM = "nuwax-person-follow"
PLATFORM = "noble-x86_64"


def dump(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, sort_keys=True, indent=2) + "\n", encoding="utf-8")


def load_state(directory):
    payload = json.loads((directory / "验收状态.json").read_text(encoding="utf-8"))
    expected_tag = validation_tag(payload["run_id"], payload["run_attempt"], payload["case"])
    if payload.get("schema") != 2 or payload.get("validation_release") != expected_tag or payload.get("production_release") != PRODUCTION_RELEASE:
        raise RuntimeError("验收状态来源错误，拒绝操作Release")
    common._repository(payload["repository"])
    if not common._REVISION.fullmatch(payload["revision"]):
        raise RuntimeError("验收状态缺少完整提交")
    owner = "NEH附件故障验收 " + ":".join((payload["run_id"], payload["run_attempt"], payload["case"], payload["revision"]))
    if payload.get("owner") != owner or (payload.get("tag_created") and payload.get("tag_sha") != payload["revision"]):
        raise RuntimeError("验收资源所有权记录错误")
    entry = payload["entry"]
    if not common._HEX.fullmatch(entry["fingerprint"]) or entry["asset"] != PROGRAM + "-" + entry["fingerprint"] + ".tar.zst" or not common._HEX.fullmatch(entry["sha256"]):
        raise RuntimeError("验收程序附件身份错误")
    return payload


def validation_tag(run_id, attempt, case):
    if not re.fullmatch(r"(?:[1-9][0-9]*|test-[a-z0-9][a-z0-9-]{0,47})", str(run_id)) or not re.fullmatch(r"[1-9][0-9]*", str(attempt)) or case not in {"missing", "corrupt"}:
        raise RuntimeError("验收运行/重试/场景身份不合法")
    return VALIDATION_PREFIX + str(run_id) + "-" + str(attempt) + "-" + case


def run_identity(args):
    run_id, attempt = str(args.run_id or ""), str(args.run_attempt or "")
    validation_tag(run_id, attempt, "missing")
    if os.environ.get("GITHUB_RUN_ID"):
        if run_id != os.environ["GITHUB_RUN_ID"] or attempt != os.environ.get("GITHUB_RUN_ATTEMPT"):
            raise RuntimeError("验收运行身份与当前Actions作业不符")
    elif not run_id.startswith("test-"):
        raise RuntimeError("本地验收必须显式使用test-开头的运行身份")
    return run_id, attempt


def gh(arguments, *, capture=False, allow_missing=False):
    if not os.environ.get("GH_TOKEN"):
        raise RuntimeError("Release故障编排仅允许在显式GH_TOKEN的主机步骤执行")
    command = ["gh", *map(str, arguments)]
    completed = subprocess.run(command, check=False, text=True, capture_output=True)
    if completed.returncode:
        if allow_missing and "HTTP 404" in completed.stderr:
            return None
        raise RuntimeError("GitHub操作失败：" + completed.stderr.strip())
    if capture:
        return json.loads(completed.stdout)
    return completed.stdout


def create_release(directory, payload):
    repository, tag = payload["repository"], payload["validation_release"]
    endpoint = "repos/" + repository
    # 先拒绝已有资源，再依赖创建API的原子冲突检查；绝不复用或覆盖旧tag。
    if gh(["api", endpoint + "/git/ref/tags/" + tag], capture=True, allow_missing=True) is not None or gh(["api", endpoint + "/releases/tags/" + tag], capture=True, allow_missing=True) is not None:
        raise RuntimeError("本轮验收tag或Release已存在，拒绝复用")
    created = gh(["api", "--method", "POST", endpoint + "/git/refs", "-f", "ref=refs/tags/" + tag, "-f", "sha=" + payload["revision"]], capture=True)
    if created["ref"] != "refs/tags/" + tag or created["object"]["sha"] != payload["revision"]:
        raise RuntimeError("新建验收tag身份错误")
    payload.update(tag_created=True, tag_sha=created["object"]["sha"])
    dump(directory / "验收状态.json", payload)
    request = directory / "创建Release请求.json"
    dump(request, {"tag_name": tag, "name": "NEH附件故障验收 " + payload["case"], "body": payload["owner"], "prerelease": True, "make_latest": "false"})
    created = gh(["api", "--method", "POST", endpoint + "/releases", "--input", str(request)], capture=True)
    if created["tag_name"] != tag or created.get("body") != payload["owner"] or created.get("draft"):
        raise RuntimeError("新建验收Release身份错误")
    payload["release_id"] = created["id"]
    dump(directory / "验收状态.json", payload)


def observe_asset(url, destination, *, size, timeout=60):
    """使用和设备相同的匿名直链，记录完整对象或明确的404。"""
    digest, received = hashlib.sha256(), 0
    destination.parent.mkdir(parents=True, exist_ok=True)
    header_names = ("Date", "Age", "Cache-Control", "X-Cache", "Via", "ETag", "X-GitHub-Request-Id")
    try:
        with common._urlopen(url, timeout=timeout) as response, destination.open("wb") as output:
            if response.status != 200:
                raise RuntimeError("匿名附件下载需要完整HTTP 200响应")
            headers = {name: response.headers[name] for name in header_names if response.headers.get(name) is not None}
            while received <= size:
                block = response.read(min(1024 * 1024, size + 1 - received))
                if not block:
                    break
                received += len(block)
                digest.update(block)
                output.write(block)
    except urllib.error.HTTPError as exc:
        try:
            if exc.code != 404:
                raise
            return {"http_status": 404, "url": url, "headers": {name: exc.headers[name] for name in header_names if exc.headers.get(name) is not None}}
        finally:
            exc.close()
    return {"http_status": 200, "bytes": received, "sha256": digest.hexdigest(), "url": url, "headers": headers}


def anonymous_download(url, destination, *, size, sha256):
    observation = observe_asset(url, destination, size=size)
    if observation["http_status"] != 200 or observation["bytes"] != size or observation["sha256"] != sha256:
        raise RuntimeError("匿名附件下载与目标大小或SHA-256不符：" + json.dumps(observation, ensure_ascii=False))
    return observation


def wait_visibility(url, destination, *, size, sha256, report_path, missing=False):
    """仅等待验收控制的对象传播；普通网络错误和未知内容直接失败。"""
    started = time.monotonic()
    report = {"url": url, "expected": {"http_status": 404 if missing else 200, "bytes": size, "sha256": sha256}, "attempts": [], "success": False}
    try:
        # 有响应才判定是否传播中；不使用查询参数绕开设备将访问的缓存键。
        for delay in (0, 1, 2, 4, 8, 16, 16, 16):
            remaining = 120 - (time.monotonic() - started)
            if remaining <= delay:
                break
            if delay:
                time.sleep(delay)
            observation = observe_asset(url, destination, size=size, timeout=min(30, remaining - delay))
            report["attempts"].append({"elapsed_seconds": round(time.monotonic() - started, 3), **observation})
            if observation["http_status"] == 404:
                ready = missing
            else:
                exact = observation["bytes"] == size and observation["sha256"] == sha256
                ready = not missing and exact
                if not ready:
                    raise RuntimeError("独立验收URL出现非目标内容，拒绝重试或覆盖")
            if ready:
                report["success"] = True
                return observation
            print("验收附件尚未呈现目标状态，第" + str(len(report["attempts"])) + "次匿名核验：" + json.dumps(observation, ensure_ascii=False), flush=True)
        raise RuntimeError("验收附件在8次/120秒上限内未呈现目标状态")
    except BaseException as exc:
        report["error"] = str(exc)
        raise
    finally:
        report["elapsed_seconds"] = round(time.monotonic() - started, 3)
        dump(report_path, report)
        destination.unlink(missing_ok=True)


def prepare(args):
    if common.RELEASE != PRODUCTION_RELEASE:
        raise RuntimeError("生产预编译Release常量异常")
    run_id, attempt = run_identity(args)
    repository = common._repository(args.repository)
    tag = validation_tag(run_id, attempt, args.case)
    state = args.state.resolve() / args.case
    state.mkdir(parents=True, exist_ok=True)
    if (state / "验收状态.json").exists():
        raise RuntimeError("本场景状态已存在，拒绝重复准备")
    index = common.fetch_index(repository, args.revision)
    target = index["platforms"][PLATFORM]
    entry = target["programs"][PROGRAM]
    if entry["asset"] != PROGRAM + "-" + entry["fingerprint"] + ".tar.zst":
        raise RuntimeError("生产程序附件命名不符合指纹")
    owner = "NEH附件故障验收 " + ":".join((run_id, attempt, args.case, args.revision))
    payload = {"schema": 2, "repository": repository, "revision": args.revision, "run_id": run_id, "run_attempt": attempt, "case": args.case, "production_release": PRODUCTION_RELEASE, "validation_release": tag, "owner": owner, "tag_created": False, "release_id": None, "entry": entry, "prepared": False}
    dump(state / "验收状态.json", payload)
    # 每个URL只有一种内容：缺失场景从不上传程序，损坏场景只上传损坏副本。
    with tempfile.TemporaryDirectory(prefix="neh-acceptance-upload-") as scratch:
        uploads = []
        if args.case == "corrupt":
            damaged = Path(scratch) / entry["asset"]
            payload["production_download"] = anonymous_download(common.release_url(repository, entry["asset"]), damaged, size=entry["size"], sha256=entry["sha256"])
            with damaged.open("r+b") as output:
                byte = output.read(1)
                if not byte:
                    raise RuntimeError("生产归档为空，不能构造损坏副本")
                output.seek(0)
                output.write(bytes([byte[0] ^ 0xFF]))
            corrupted_sha = common.sha256(damaged)
            if damaged.stat().st_size != entry["size"] or corrupted_sha == entry["sha256"]:
                raise RuntimeError("故障附件未按计划保持大小并改变摘要")
            payload["fault_expected"] = {"http_status": 200, "bytes": entry["size"], "sha256": corrupted_sha}
            uploads.append(damaged)
        else:
            payload["fault_expected"] = {"http_status": 404}
        dump(state / "验收状态.json", payload)
        create_release(state, payload)
        # 原生产索引SHA保持不变，保证真实设备下载器必须拒绝损坏副本。
        copied = {**index, "platforms": {PLATFORM: {**target, "programs": {PROGRAM: entry}}}, "validation_copy_of": PRODUCTION_RELEASE}
        copied_index = state / ("index-" + args.revision + ".json")
        dump(copied_index, copied)
        gh(["release", "upload", tag, str(copied_index), *map(str, uploads), "--repo", repository])
    common.RELEASE = tag
    wait_visibility(common.release_url(repository, copied_index.name), state / "索引核验临时文件", size=copied_index.stat().st_size, sha256=common.sha256(copied_index), report_path=state / "索引可见性.json")
    payload["fault_visibility"] = wait_visibility(common.release_url(repository, entry["asset"]), state / "故障核验临时文件", size=entry["size"], sha256=payload["fault_expected"].get("sha256", entry["sha256"]), missing=args.case == "missing", report_path=state / "附件可见性.json")
    if common.fetch_index(repository, args.revision)["platforms"][PLATFORM]["programs"][PROGRAM] != entry:
        raise RuntimeError("验收索引与生产条目不符")
    payload["prepared"] = True
    dump(state / "验收状态.json", payload)
    print("独立验收Release已就绪：" + tag, flush=True)


class Tee:
    def __init__(self, output):
        self.output = output
    def write(self, value):
        sys.__stdout__.write(value)
        sys.__stdout__.flush()
        return self.output.write(value)
    def flush(self):
        self.output.flush()


def client(args):
    # 此命令在无令牌容器内执行，直接调用真实build_modules及其原生构建器。
    if any(os.environ.get(name) for name in ("GH_TOKEN", "GITHUB_TOKEN", "GITHUB_API_TOKEN")):
        raise RuntimeError("设备验收客户端不能接收GitHub令牌")
    if os.geteuid() == 0 or common.platform_key() != PLATFORM:
        raise RuntimeError("设备验收必须在非root的noble-x86_64环境执行")
    args.state = args.state.resolve() / args.case
    payload = load_state(args.state)
    if not payload.get("prepared") or payload["case"] != args.case:
        raise RuntimeError("验收状态与故障场景不符")
    root, cache = args.source_root.resolve(), args.cache.resolve()
    revision = subprocess.check_output(["git", "-C", str(root), "rev-parse", "HEAD"], text=True).strip()
    if revision != payload["revision"]:
        raise RuntimeError("真实本机构建源码与生产预编译提交不一致")
    if subprocess.check_output(["git", "-C", str(root), "status", "--porcelain", "--untracked-files=no"], text=True).strip():
        raise RuntimeError("验收源码有未提交变化")
    cache.mkdir(parents=True, exist_ok=True)
    owner = cache / "验收缓存拥有方.json"
    expected_owner = {"schema": 1, "purpose": CACHE_PURPOSE}
    if owner.exists():
        if json.loads(owner.read_text()) != expected_owner:
            raise RuntimeError("拒绝使用非验收编译缓存")
    elif any(cache.iterdir()):
        raise RuntimeError("验收缓存非空且无拥有方记录")
    else:
        dump(owner, expected_owner)
    prefix = cache / "prefix"
    prefix.mkdir(exist_ok=True)
    # 只移除本程序的Rust输出，依赖与固定target槽复用，确保本轮真实重新链接。
    target = builds._rust_target(cache, root, common.PROGRAMS[PROGRAM]["manifest"])
    own_outputs = [target / "release" / PROGRAM]
    own_outputs.extend((target / "release/deps").glob(PROGRAM.replace("-", "_") + "-*"))
    for output in own_outputs:
        if output.is_symlink():
            raise RuntimeError("验收Rust输出不能为符号链接")
        if output.is_file():
            output.unlink()
    for path in (prefix / "bin" / PROGRAM, cache / "prebuilt/receipts" / (PROGRAM + ".json"), cache / "prebuilt/archives" / payload["entry"]["asset"]):
        if path.is_symlink():
            raise RuntimeError("验收清理目标不能为符号链接")
        path.unlink(missing_ok=True)
    common.RELEASE = payload["validation_release"]
    requests, commands = [], []
    def audit(event, values):
        if event == "urllib.Request":
            url = values[0]
            if url.startswith("https://github.com/"):
                headers = values[2]
                if any(name.lower() == "authorization" for name in headers):
                    raise RuntimeError("匿名设备下载不能发送授权请求头")
                requests.append(url)
    sys.addaudithook(audit)
    started = time.monotonic()
    report = {"schema": 2, "case": args.case, "run_id": payload["run_id"], "run_attempt": payload["run_attempt"], "source_revision": revision, "harness_revision": os.environ.get("NEH_ACCEPTANCE_HARNESS_REVISION", ""), "validation_release": payload["validation_release"], "repository": payload["repository"], "expected_asset": payload["entry"], "fault_expected": payload["fault_expected"], "fault_visibility": payload["fault_visibility"], "success": False}
    log_path = args.state / (args.case + "-客户端日志.txt")
    def run(command, *, cwd=None, env=None):
        argv = list(map(str, command))
        commands.append({"argv": argv, "cwd": str(cwd) if cwd else None})
        print("真实编译命令：" + " ".join(argv), flush=True)
        process = subprocess.Popen(argv, cwd=cwd, env={**os.environ, **(env or {})}, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        try:
            for line in process.stdout:
                print(line, end="", flush=True)
            code = process.wait()
        finally:
            process.stdout.close()
            if process.poll() is None:
                process.kill()
                process.wait()
        if code:
            raise subprocess.CalledProcessError(code, argv)
    try:
        with log_path.open("w", encoding="utf-8") as output, contextlib.redirect_stdout(Tee(output)):
            # 容器与编排主机可能看到不同暂态，客户端也独立确认同一故障目标。
            asset_url = common.release_url(payload["repository"], payload["entry"]["asset"])
            expected = payload["fault_expected"]
            if args.case == "corrupt":
                if expected["bytes"] != payload["entry"]["size"] or expected["sha256"] == payload["entry"]["sha256"]:
                    raise RuntimeError("T4故障目标未保持大小并改变摘要")
            report["fault_observation"] = wait_visibility(asset_url, args.state / "客户端故障核验临时文件", size=payload["entry"]["size"], sha256=expected.get("sha256", payload["entry"]["sha256"]), missing=args.case == "missing", report_path=args.state / (args.case + "-客户端附件可见性.json"))
            result = builds.build_modules(root, prefix, cache, ["follow"], args.jobs, run, prebuilt={"repository": payload["repository"], "revision": revision}, programs=[PROGRAM], capture_link_args=True)
            if set(result) != {PROGRAM}:
                raise RuntimeError("真实构建器返回程序集合错误")
            binary = Path(result[PROGRAM])
            builds._executable(binary)
            info = subprocess.check_output(["readelf", "-h", str(binary)], text=True, env={**os.environ, "LC_ALL": "C"})
            if "Advanced Micro Devices X86-64" not in info:
                raise RuntimeError("回退程序不是x86_64 ELF")
            dependencies = subprocess.check_output(["ldd", str(binary)], text=True, env={**os.environ, "LC_ALL": "C"})
            if "not found" in dependencies:
                raise RuntimeError("回退程序有无法解析的动态依赖")
            if not any("rustc" in command["argv"] and "--bin" in command["argv"] and PROGRAM in command["argv"] and "nuwax-nav-visual-servoing" in command["argv"] for command in commands):
                raise RuntimeError("未经过真正人物跟随Rust源码编译")
            expected_index = common.release_url(payload["repository"], "index-" + revision + ".json")
            if expected_index not in requests or requests.count(asset_url) < 2:
                raise RuntimeError("设备没有实际匿名请求验收索引和故障程序")
            receipt = cache / "prebuilt/receipts" / (PROGRAM + ".json")
            if not common.reusable(prefix, receipt, payload["entry"]["fingerprint"]):
                raise RuntimeError("本机回退未产生可复用程序收据")
            report.update(success=True, binary_sha256=common.sha256(binary), elf_header=info, dynamic_dependencies=dependencies, rust_target=str(target))
        logged = log_path.read_text(encoding="utf-8")
        expected_error = "HTTP Error 404" if args.case == "missing" else "预编译下载SHA-256（文件校验摘要）不符"
        if expected_error not in logged or "退回本机编译 " + PROGRAM not in logged:
            raise RuntimeError("设备日志缺少明确故障拒绝与本机编译回退证据")
        if '--crate-name nuwax_person_follow' not in logged or not any(line.startswith("LC_ALL=") and '"-o"' in line for line in logged.splitlines()):
            raise RuntimeError("缺少真实Rustc执行与最终链接命令证据")
    except BaseException as exc:
        report.update(success=False, error=str(exc))
        raise
    finally:
        report.update(elapsed_seconds=round(time.monotonic() - started, 3), requests=requests, commands=commands)
        dump(args.state / (args.case + "-真实回退报告.json"), report)
    print("真实附件故障→本机编译回退通过：" + args.case, flush=True)


def cleanup(args):
    run_id, attempt = run_identity(args)
    failures = []
    for case in ("missing", "corrupt"):
        directory = args.state.resolve() / case
        if not (directory / "验收状态.json").is_file():
            continue
        report = {"success": False, "case": case, "run_id": run_id, "run_attempt": attempt}
        try:
            payload = load_state(directory)
            if payload["run_id"] != run_id or payload["run_attempt"] != attempt or payload["case"] != case:
                raise RuntimeError("清理状态不属于本次运行/场景")
            report["validation_release"] = payload["validation_release"]
            if not payload["tag_created"]:
                report.update(success=True, skipped="本轮未创建tag，不操作任何远端资源")
                continue
            endpoint = "repos/" + payload["repository"]
            tag = payload["validation_release"]
            reference = gh(["api", endpoint + "/git/ref/tags/" + tag], capture=True, allow_missing=True)
            released = gh(["api", endpoint + "/releases/tags/" + tag], capture=True, allow_missing=True)
            if reference is not None and (reference["ref"] != "refs/tags/" + tag or reference["object"]["sha"] != payload["tag_sha"]):
                raise RuntimeError("验收tag已被替换，拒绝删除")
            if released is not None:
                # body所有权亦覆盖创建响应后、状态写盘前中断的恢复清理。
                if released["tag_name"] != tag or released.get("body") != payload["owner"] or (payload["release_id"] is not None and released["id"] != payload["release_id"]):
                    raise RuntimeError("验收Release所有权或ID变化，拒绝删除")
                payload["release_id"] = released["id"]
                dump(directory / "验收状态.json", payload)
                gh(["api", "--method", "DELETE", endpoint + "/releases/" + str(released["id"])])
                payload["release_deleted"] = True
                dump(directory / "验收状态.json", payload)
            if reference is not None:
                gh(["api", "--method", "DELETE", endpoint + "/git/refs/tags/" + tag])
            # 即使API返回成功，也再次确认两种资源都已不存在。
            if gh(["api", endpoint + "/releases/tags/" + tag], capture=True, allow_missing=True) is not None or gh(["api", endpoint + "/git/ref/tags/" + tag], capture=True, allow_missing=True) is not None:
                raise RuntimeError("验收临时资源删除后仍存在")
            payload["cleaned"] = True
            dump(directory / "验收状态.json", payload)
            report.update(success=True, release_id=payload["release_id"], tag_sha=payload["tag_sha"])
        except Exception as exc:
            report["error"] = str(exc)
            failures.append(case + ": " + str(exc))
        finally:
            dump(directory / "清理报告.json", report)
    if failures:
        raise RuntimeError("本轮验收资源清理失败：" + "; ".join(failures))
    print("本轮创建的独立验收Release/tag已清理。", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    command = commands.add_parser("prepare")
    command.add_argument("--repository", required=True)
    command.add_argument("--revision", required=True)
    command.add_argument("--state", type=Path, required=True)
    command.add_argument("--case", choices=("missing", "corrupt"), required=True)
    command.add_argument("--run-id", default=os.environ.get("GITHUB_RUN_ID"))
    command.add_argument("--run-attempt", default=os.environ.get("GITHUB_RUN_ATTEMPT", "1"))
    command = commands.add_parser("client")
    command.add_argument("--state", type=Path, required=True)
    command.add_argument("--case", choices=("missing", "corrupt"), required=True)
    command.add_argument("--source-root", type=Path, required=True)
    command.add_argument("--cache", type=Path, required=True)
    command.add_argument("--jobs", type=int, default=4)
    command = commands.add_parser("cleanup")
    command.add_argument("--state", type=Path, required=True)
    command.add_argument("--run-id", default=os.environ.get("GITHUB_RUN_ID"))
    command.add_argument("--run-attempt", default=os.environ.get("GITHUB_RUN_ATTEMPT", "1"))
    args = parser.parse_args()
    if hasattr(args, "revision") and not common._REVISION.fullmatch(args.revision):
        raise RuntimeError("生产索引需要完整提交")
    globals()[args.command](args)


if __name__ == "__main__":
    main()
