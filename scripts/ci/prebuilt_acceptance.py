#!/usr/bin/env python3
"""在独立Release注入附件故障，用匿名客户端下载并真实本机编译验证恢复。"""
from __future__ import annotations

import argparse
import contextlib
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time
import urllib.error

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from scripts import neh_prebuilt as common
from scripts.module_installer import builds

VALIDATION_RELEASE = "neh-prebuilt-validation-v1"
PRODUCTION_RELEASE = "neh-prebuilt-v1"
PROGRAM = "nuwax-person-follow"
PLATFORM = "noble-x86_64"


def dump(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, sort_keys=True, indent=2) + "\n", encoding="utf-8")


def load_state(directory):
    payload = json.loads((directory / "验收状态.json").read_text(encoding="utf-8"))
    if payload.get("validation_release") != VALIDATION_RELEASE or payload.get("production_release") != PRODUCTION_RELEASE:
        raise RuntimeError("验收状态来源错误，拒绝操作Release")
    common._repository(payload["repository"])
    if not common._REVISION.fullmatch(payload["revision"]):
        raise RuntimeError("验收状态缺少完整提交")
    entry = payload["entry"]
    if not common._HEX.fullmatch(entry["fingerprint"]) or entry["asset"] != PROGRAM + "-" + entry["fingerprint"] + ".tar.zst" or not common._HEX.fullmatch(entry["sha256"]):
        raise RuntimeError("验收程序附件身份错误")
    return payload


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


def release(repository, *, create_revision=None):
    # 所有有副作用API都经此固定tag解析；不能接受命令行指定其它Release。
    endpoint = "repos/" + repository + "/releases/tags/" + VALIDATION_RELEASE
    payload = gh(["api", endpoint], capture=True, allow_missing=create_revision is not None)
    if payload is None:
        gh(["release", "create", VALIDATION_RELEASE, "--repo", repository, "--target", create_revision, "--prerelease", "--title", "NEH预编译故障验收专用", "--notes", "仅供隔离故障注入；程序附件每轮结束恢复原始SHA-256。生产预编译Release不受此验收修改。"])
        payload = gh(["api", endpoint], capture=True)
    if payload["tag_name"] != VALIDATION_RELEASE or payload.get("draft"):
        raise RuntimeError("验收Release身份不符")
    return payload


def assets(repository, release_id):
    pages = gh(["api", "--paginate", "--slurp", "repos/" + repository + "/releases/" + str(release_id) + "/assets?per_page=100"], capture=True)
    return [item for page in pages for item in page]


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


def wait_visibility(url, destination, *, size, sha256, report_path, missing=False, previous_sha256=()):
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
                if not ready and (observation["bytes"] != size or observation["sha256"] not in previous_sha256):
                    raise RuntimeError("验收对象既不是目标也不是已知旧版本，拒绝作为传播延迟重试")
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
    repository = common._repository(args.repository)
    index = common.fetch_index(repository, args.revision)
    target = index["platforms"][PLATFORM]
    entry = target["programs"][PROGRAM]
    if entry["asset"] != PROGRAM + "-" + entry["fingerprint"] + ".tar.zst":
        raise RuntimeError("生产程序附件命名不符合指纹")
    state = args.state.resolve()
    state.mkdir(parents=True, exist_ok=True)
    original = state / "原始附件" / entry["asset"]
    verification = anonymous_download(common.release_url(repository, entry["asset"]), original, size=entry["size"], sha256=entry["sha256"])
    # 恢复所需的原件和状态先落盘，再允许任何故障注入。
    payload = {"schema": 1, "repository": repository, "revision": args.revision, "production_release": PRODUCTION_RELEASE, "validation_release": VALIDATION_RELEASE, "entry": entry, "original_download": verification, "fault": None, "prepared": False}
    dump(state / "验收状态.json", payload)
    validated = release(repository, create_revision=args.revision)
    payload["release_id"] = validated["id"]
    dump(state / "验收状态.json", payload)
    # 验收副本只包含当前平台人物跟随，原输入声明与编译器身份完整保留。
    copied = {**index, "platforms": {PLATFORM: {**target, "programs": {PROGRAM: entry}}}, "validation_copy_of": PRODUCTION_RELEASE}
    copied_index = state / ("index-" + args.revision + ".json")
    dump(copied_index, copied)
    gh(["release", "upload", VALIDATION_RELEASE, str(original), str(copied_index), "--repo", repository, "--clobber"])
    common.RELEASE = VALIDATION_RELEASE
    verification = wait_visibility(common.release_url(repository, entry["asset"]), state / "验证下载临时文件", size=entry["size"], sha256=entry["sha256"], report_path=state / "准备附件可见性.json")
    wait_visibility(common.release_url(repository, copied_index.name), state / "验证下载临时文件", size=copied_index.stat().st_size, sha256=common.sha256(copied_index), report_path=state / "准备索引可见性.json")
    if common.fetch_index(repository, args.revision)["platforms"][PLATFORM]["programs"][PROGRAM] != entry:
        raise RuntimeError("验收索引副本与生产条目不符")
    payload.update(prepared=True, validation_original=verification)
    dump(state / "验收状态.json", payload)
    print("验收Release副本已就绪：" + VALIDATION_RELEASE, flush=True)


def inject(args):
    payload = load_state(args.state)
    if not payload.get("prepared"):
        raise RuntimeError("原件未完成验证，拒绝故障注入")
    validated = release(payload["repository"])
    if validated["id"] != payload["release_id"]:
        raise RuntimeError("验收Release被替换，拒绝注入")
    original = args.state / "原始附件" / payload["entry"]["asset"]
    if common.sha256(original) != payload["entry"]["sha256"]:
        raise RuntimeError("恢复原件损坏，拒绝故障注入")
    # 故障意图先记入状态，即使后续API失败always步骤仍可恢复原件。
    payload["fault"] = args.case
    payload["fault_ready"] = False
    dump(args.state / "验收状态.json", payload)
    if args.case == "missing":
        candidates = [item for item in assets(payload["repository"], validated["id"]) if item["name"] == payload["entry"]["asset"]]
        if len(candidates) != 1:
            raise RuntimeError("验收附件不存在或不唯一，不能执行删除场景")
        payload["fault_expected"] = {"http_status": 404}
        dump(args.state / "验收状态.json", payload)
        gh(["api", "--method", "DELETE", "repos/" + payload["repository"] + "/releases/assets/" + str(candidates[0]["id"])])
    else:
        damaged = args.state / "故障附件" / original.name
        damaged.parent.mkdir(exist_ok=True)
        shutil.copyfile(original, damaged)
        with damaged.open("r+b") as output:
            byte = output.read(1)
            if not byte:
                raise RuntimeError("原始程序归档为空")
            output.seek(0)
            output.write(bytes([byte[0] ^ 0xFF]))
        if damaged.stat().st_size != original.stat().st_size or common.sha256(damaged) == payload["entry"]["sha256"]:
            raise RuntimeError("故障附件未按计划改变摘要")
        payload["fault_expected"] = {"http_status": 200, "bytes": damaged.stat().st_size, "sha256": common.sha256(damaged)}
        dump(args.state / "验收状态.json", payload)
        gh(["release", "upload", VALIDATION_RELEASE, str(damaged), "--repo", payload["repository"], "--clobber"])
    common.RELEASE = VALIDATION_RELEASE
    observation = wait_visibility(common.release_url(payload["repository"], original.name), args.state / "故障核验临时文件", size=payload["entry"]["size"], sha256=payload["fault_expected"].get("sha256", payload["entry"]["sha256"]), missing=args.case == "missing", previous_sha256=(payload["entry"]["sha256"],), report_path=args.state / (args.case + "-附件可见性.json"))
    payload.update(fault_ready=True, fault_visibility=observation)
    dump(args.state / "验收状态.json", payload)
    print("已对验收Release注入附件故障：" + args.case, flush=True)


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
    payload = load_state(args.state)
    if not payload.get("prepared") or payload.get("fault") != args.case or not payload.get("fault_ready"):
        raise RuntimeError("验收状态与故障场景不符")
    root, cache = args.source_root.resolve(), args.cache.resolve()
    revision = subprocess.check_output(["git", "-C", str(root), "rev-parse", "HEAD"], text=True).strip()
    if revision != payload["revision"]:
        raise RuntimeError("真实本机构建源码与生产预编译提交不一致")
    if subprocess.check_output(["git", "-C", str(root), "status", "--porcelain", "--untracked-files=no"], text=True).strip():
        raise RuntimeError("验收源码有未提交变化")
    cache.mkdir(parents=True, exist_ok=True)
    owner = cache / "验收缓存拥有方.json"
    expected_owner = {"schema": 1, "purpose": VALIDATION_RELEASE}
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
    common.RELEASE = VALIDATION_RELEASE
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
    report = {"schema": 1, "case": args.case, "source_revision": revision, "harness_revision": os.environ.get("NEH_ACCEPTANCE_HARNESS_REVISION", ""), "validation_release": VALIDATION_RELEASE, "repository": payload["repository"], "expected_asset": payload["entry"], "fault_expected": payload["fault_expected"], "fault_visibility": payload["fault_visibility"], "success": False}
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
            report["fault_observation"] = wait_visibility(asset_url, args.state / "客户端故障核验临时文件", size=payload["entry"]["size"], sha256=expected.get("sha256", payload["entry"]["sha256"]), missing=args.case == "missing", previous_sha256=(payload["entry"]["sha256"],), report_path=args.state / (args.case + "-客户端附件可见性.json"))
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


def restore(args):
    if not (args.state / "验收状态.json").is_file():
        print("准备阶段尚未产生原件状态，无需恢复。", flush=True)
        return
    payload = load_state(args.state)
    original = args.state / "原始附件" / payload["entry"]["asset"]
    if not original.is_file() or common.sha256(original) != payload["entry"]["sha256"]:
        raise RuntimeError("无法恢复：验收原件缺失或摘要损坏")
    if not payload.get("release_id"):
        print("验收Release尚未创建，无需恢复。", flush=True)
        return
    validated = release(payload["repository"])
    if validated["id"] != payload["release_id"]:
        raise RuntimeError("验收Release身份变化，拒绝恢复到未知位置")
    gh(["release", "upload", VALIDATION_RELEASE, str(original), "--repo", payload["repository"], "--clobber"])
    common.RELEASE = VALIDATION_RELEASE
    previous = payload.get("fault_expected", {}).get("sha256")
    verification = wait_visibility(common.release_url(payload["repository"], original.name), args.state / "恢复核验临时文件", size=payload["entry"]["size"], sha256=payload["entry"]["sha256"], previous_sha256=(previous,) if previous else (), report_path=args.state / "恢复附件可见性.json")
    payload.update(fault=None, fault_ready=False, restored=True, restore_verification=verification)
    dump(args.state / "验收状态.json", payload)
    dump(args.state / "恢复报告.json", {"success": True, "validation_release": VALIDATION_RELEASE, "original_sha256": payload["entry"]["sha256"], "anonymous_verification": verification})
    # 原件仅为注入恢复服务；恢复核验完成后清除本地副本和故障副本。
    shutil.rmtree(args.state / "原始附件")
    fault_copy = args.state / "故障附件"
    if fault_copy.exists():
        shutil.rmtree(fault_copy)
    print("验收附件已恢复，匿名摘要与生产原件一致。", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    command = commands.add_parser("prepare")
    command.add_argument("--repository", required=True)
    command.add_argument("--revision", required=True)
    command.add_argument("--state", type=Path, required=True)
    command = commands.add_parser("inject")
    command.add_argument("--state", type=Path, required=True)
    command.add_argument("--case", choices=("missing", "corrupt"), required=True)
    command = commands.add_parser("client")
    command.add_argument("--state", type=Path, required=True)
    command.add_argument("--case", choices=("missing", "corrupt"), required=True)
    command.add_argument("--source-root", type=Path, required=True)
    command.add_argument("--cache", type=Path, required=True)
    command.add_argument("--jobs", type=int, default=4)
    command = commands.add_parser("restore")
    command.add_argument("--state", type=Path, required=True)
    args = parser.parse_args()
    if hasattr(args, "revision") and not common._REVISION.fullmatch(args.revision):
        raise RuntimeError("生产索引需要完整提交")
    globals()[args.command](args)


if __name__ == "__main__":
    main()
