"""完整本机命令行；与网页使用同一计划、任务和配置实现。"""
from __future__ import annotations

import argparse
import getpass
import json
import os
import sys
import time
from pathlib import Path

from .auth import Authentication
from .common import ManagerError
from .engine import ACTIONS, TERMINAL, Engine
from .server import serve


def output(value):
    print(json.dumps(value, ensure_ascii=False, indent=2))


def load_file(path):
    try:
        value = json.loads(Path(path).read_text())
        if not isinstance(value, dict):
            raise ValueError()
        return value
    except (OSError, ValueError) as exc:
        raise ManagerError("file_invalid", "配置文件必须是可读取的JSON对象。") from exc


def wait_job(engine, job_id):
    last = None
    while True:
        engine.recover_if_needed()
        job = engine.job(job_id)
        state = (job["stage"], job["progress"], job["message"])
        if state != last:
            print("[{}%] {}".format(job["progress"], job["message"]), file=sys.stderr, flush=True)
            last = state
        if job["status"] in TERMINAL:
            return job
        time.sleep(0.4)


def request_from_args(args):
    request = load_file(args.file) if getattr(args, "file", None) else {}
    request.setdefault("action", getattr(args, "action", None) or "install")
    for key in ("version", "robot", "source_path", "source_repository", "source_branch", "source_revision"):
        if getattr(args, key, None):
            request[key] = getattr(args, key)
    if getattr(args, "modules", None):
        request["modules"] = [name.strip() for name in args.modules.split(",") if name.strip()]
    if getattr(args, "stop_running", False):
        request["stop_running"] = True
    return request


def wizard(engine, catalog=None):
    if not sys.stdin.isatty():
        raise ManagerError("terminal_required", "交互向导需要终端；无人值守请使用apply --file 配置.json --wait。")
    hardware = engine.hardware()
    recommendation = hardware["recommendation"]
    print("NEH设备安装向导")
    print("系统：{} {}，{}".format(hardware["hardware"]["os_id"], hardware["hardware"]["version_id"], hardware["hardware"]["architecture"]))
    for reason in recommendation["reasons"] + recommendation["warnings"]:
        print("  " + reason)
    if not catalog and not engine.settings()["feed_url"]:
        url = input("请输入HTTPS版本目录地址：").strip()
        engine.save_settings({"feed_url": url})
    robot = input("机器人类型 [customrobot/go2，默认{}]：".format(recommendation["robot"])).strip() or recommendation["robot"]
    for module in engine.modules()["modules"]:
        print("  {} — {}：{}".format(module["key"], module["title"], module["description"]))
    default_modules = ",".join(recommendation["modules"])
    selected = input("模块（逗号分隔，默认{}）：".format(default_modules)).strip() or default_modules
    configuration = {}
    print("输入首次配置，每行KEY=value，留空继续；敏感值可稍后从离线设置入口保存。")
    while True:
        assignment = input("配置：").strip()
        if not assignment:
            break
        key, separator, value = assignment.partition("=")
        if not separator:
            print("请使用KEY=value格式。")
            continue
        configuration[key.strip()] = value
    current = engine.current()
    request = {"action": "update" if current else "install", "robot": robot, "modules": selected.split(","), "configuration": configuration}
    if engine.runtime.status()["running"]:
        request["stop_running"] = input("业务正在运行，允许安装期间停止当前任务？[y/N]：").strip().lower() == "y"
    plan = engine.plan(request, local_catalog=catalog, local_request=True)
    output(plan)
    if plan["blockers"]:
        raise ManagerError("plan_blocked", "请处理计划中的阻塞条件后重试。", 409, plan["blockers"])
    if input("执行以上计划？[y/N]：").strip().lower() != "y":
        return {"status": "not_submitted", "message": "计划已保存，尚未提交。"}
    job = engine.submit(plan["id"])
    return wait_job(engine, job["id"])


def parser():
    result = argparse.ArgumentParser(description="NEH端侧管理：安装、更新、运行、配置和恢复使用同一持久引擎。")
    result.add_argument("--home", default=os.environ.get("NEH_HOME", "~/.local/share/neh"), help="设备安装根目录")
    sub = result.add_subparsers(dest="command", required=True)
    server = sub.add_parser("serve", help="提供独立设备网页")
    server.add_argument("--host", default="127.0.0.1")
    server.add_argument("--port", type=int, default=8765)
    for name in ("status", "hardware", "modules", "jobs", "migration-sources"):
        sub.add_parser(name)
    releases = sub.add_parser("releases", help="查询或主动刷新版本目录")
    releases.add_argument("--refresh", action="store_true")
    releases.add_argument("--catalog", help="显式本机catalog.json路径，仅供离线安装和验收")
    for command in ("plan", "apply", "migrate"):
        item = sub.add_parser(command)
        if command == "plan":
            item.add_argument("action", choices=sorted(ACTIONS), nargs="?", default="install")
        elif command == "migrate":
            item.set_defaults(action="migrate")
            item.add_argument("source_path", help="原安装源码目录")
        else:
            item.add_argument("plan_id", nargs="?")
        item.add_argument("--file", help="预配置JSON（action/version/modules/robot/configuration/stop_running）")
        item.add_argument("--catalog", help="显式本机版本目录路径")
        item.add_argument("--version")
        item.add_argument("--modules", help="逗号分隔的模块")
        item.add_argument("--robot", choices=("customrobot", "go2"))
        item.add_argument("--source-repository", help="本次源码仓库；显式本机Git路径仅供设备本地验证")
        item.add_argument("--source-branch", help="源码分支，默认main")
        item.add_argument("--source-revision", help="固定完整源码提交")
        item.add_argument("--stop-running", action="store_true", help="明确允许通过原拥有方停止当前业务")
        if command != "plan":
            item.add_argument("--wait", action="store_true", help="等待结果；退出终端不会取消后台任务")
    for command in ("start", "stop", "restart", "rollback", "update-manager"):
        item = sub.add_parser(command)
        item.add_argument("--wait", action="store_true")
        item.add_argument("--stop-running", action="store_true")
    job = sub.add_parser("job")
    job.add_argument("job_id")
    job.add_argument("--wait", action="store_true")
    job.add_argument("--cancel", action="store_true")
    settings = sub.add_parser("settings")
    settings.add_argument("--feed-url")
    settings.add_argument("--source-repository", help="HTTPS源码仓库")
    settings.add_argument("--source-branch", help="源码分支，默认main")
    settings.add_argument("--source-mode", choices=("git", "catalog"))
    settings.add_argument("--allow-local-source", action="store_true", help="明确允许本机Git夹具；网页不能启用此选项")
    settings.add_argument("--gateway-port", type=int, help="首次安装前设定业务端口，默认8766")
    settings.add_argument("--channel", choices=("stable", "preview"))
    settings.add_argument("--auto-start", choices=("true", "false"))
    settings.add_argument("--token-stdin", action="store_true", help="从标准输入保存私有源码或旧目录访问令牌，查询时不返回")
    password = sub.add_parser("password")
    password.add_argument("--stdin", action="store_true", help="逐行读取当前密码和新密码；未设密码时只读新密码")
    password.add_argument("--disable", action="store_true")
    config = sub.add_parser("config")
    config.add_argument("--file", help="JSON对象，支持{values:{...},revision:...}")
    config.add_argument("--set", nargs=2, action="append", metavar=("KEY", "VALUE"))
    config.add_argument("--revision")
    interactive = sub.add_parser("wizard", help="交互安装向导；全部步骤仍走共享计划与任务")
    interactive.add_argument("--catalog")
    worker = sub.add_parser("_worker", help=argparse.SUPPRESS)
    worker.add_argument("job_id")
    return result


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        engine = Engine(args.home)
        command = args.command
        if command == "serve":
            serve(engine, args.host, args.port)
            return 0
        if command == "_worker":
            engine.run_job(args.job_id)
            return 0
        engine.recover_if_needed()
        if command in ("status", "hardware", "modules"):
            result = getattr(engine, command)()
        elif command == "migration-sources":
            result = engine.migration_sources()
        elif command == "jobs":
            result = {"jobs": engine.jobs()}
        elif command == "releases":
            result = engine.releases(args.refresh, args.catalog)
        elif command == "settings":
            values = {key: getattr(args, key) for key in ("feed_url", "source_repository", "source_branch", "source_mode", "channel", "gateway_port") if getattr(args, key) is not None}
            if args.auto_start is not None:
                values["auto_start"] = args.auto_start == "true"
            result = engine.save_settings(values, allow_local_source=args.allow_local_source) if values else engine.settings()
            if args.token_stdin:
                engine.set_token(sys.stdin.readline().rstrip("\r\n"))
                result = engine.settings()
        elif command == "password":
            auth = Authentication(engine)
            current = (sys.stdin.readline().rstrip("\r\n") if args.stdin else getpass.getpass("当前密码：")) if auth.enabled() else ""
            password = "" if args.disable else sys.stdin.readline().rstrip("\r\n") if args.stdin else getpass.getpass("新密码（至少8位）：")
            result = auth.change(password, current)
        elif command == "config":
            if args.file or args.set:
                request = load_file(args.file) if args.file else {"values": dict(args.set or [])}
                revision = request.get("revision", args.revision)
                if revision is None:
                    revision = engine.config().get("revision")
                result = engine.save_config(request.get("values", request), revision)
            else:
                result = engine.config()
        elif command == "wizard":
            result = wizard(engine, args.catalog)
        elif command == "job":
            result = engine.cancel(args.job_id) if args.cancel else wait_job(engine, args.job_id) if args.wait else engine.job(args.job_id)
        else:
            if command in ("start", "stop", "restart", "rollback", "update-manager"):
                request = {"action": command, "stop_running": args.stop_running}
                plan = engine.plan(request, local_request=True)
            elif command == "apply" and args.plan_id:
                plan = {"id": args.plan_id}
            else:
                plan = engine.plan(request_from_args(args), local_catalog=args.catalog, local_request=True)
            if command == "plan":
                result = plan
            else:
                if plan.get("blockers"):
                    raise ManagerError("plan_blocked", "计划存在阻塞条件。", 409, plan["blockers"])
                result = engine.submit(plan["id"])
                if args.wait:
                    result = wait_job(engine, result["id"])
        output(result)
        return 1 if result.get("status") in ("failed", "interrupted", "cancelled") or result.get("blockers") else 0
    except ManagerError as exc:
        output({"error": exc.as_dict()})
        return 1
    except KeyboardInterrupt:
        print("已退出交互；已提交任务继续运行。可使用jobs、job或job --cancel管理。", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
