"""面向使用者的短向导，以及可复用的模块增删命令。"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from dataclasses import replace
from pathlib import Path
from typing import Optional, Sequence

from .catalog import MODULES, remove_modules, replace_lio, resolve_modules
from .hardware import detect_hardware, recommend_modules
from .manager import InstallError, InstallRequest, apply, existing_config, make_plan, read_current
from .runtime import read_settings


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="./install.sh", description="Nuwax · 按需安装，随时增删",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""常用操作：
  ./install.sh                         自动检测，进入简洁向导
  ./install.sh install --yes            接受自动推荐并安装
  ./install.sh install --modules core   只安装基础网关
  ./install.sh add fastlio2             增加建图，自动补齐依赖
  ./install.sh add pointlio             将 FastLIO2 替换为 PointLIO
  ./install.sh remove perception        卸载视觉及依赖它的跟随模块
  ./install.sh list                     查看当前安装
  ./install.sh install --dry-run        只查看计划
  ./install.sh start                    启动当前安装（前台运行）

原有生产部署命令 plan/check/deploy/resume/verify/status/rollback/bundle 保持可用。
使用 ./install.sh deploy --help 查看严格生产部署选项。""",
    )
    parser.add_argument("command", nargs="?", default="manage", choices=("manage", "install", "add", "remove", "list", "doctor", "start", "stop"), help="安装管理操作")
    parser.add_argument("names", nargs="*", help="add/remove 的模块名称，可使用逗号分隔")
    parser.add_argument("--modules", help="完整目标模块列表；默认采用已有选择或自动推荐")
    parser.add_argument("--robot", choices=("auto", "customrobot", "go2"), default="auto", help="自动沿用配置，未知设备使用标准接入")
    parser.add_argument("--lio", choices=("fastlio2", "pointlio", "device"), help="切换定位算法；device 表示设备已有里程计")
    parser.add_argument("--install-dir", type=Path, help="专属安装目录，默认 runtime_data/installation")
    parser.add_argument("--python", default="", help="指定 Python（编程语言）3.8—3.12 解释器")
    parser.add_argument("--wheel-dir", type=Path, help="包含 Go2 设备安装包的目录；已有可用设备依赖时自动复用")
    parser.add_argument("--jobs", type=int, help="编译并行数，默认依据处理器和内存自动限制")
    parser.add_argument("--yes", "-y", action="store_true", help="接受显示的目标方案，不再交互确认")
    parser.add_argument("--dry-run", action="store_true", help="只查看模块、依赖和构建计划，不写入文件")
    parser.add_argument("--json", action="store_true", help="输出结构化计划或状态；不执行安装")
    parser.add_argument("--repair", action="store_true", help="重新安装相同模块，修复缺失或损坏的产物")
    return parser


def _names(values: Sequence[str]) -> tuple:
    return tuple(item.strip().lower() for value in values for item in value.split(",") if item.strip())


def _show_hardware(hardware, recommendation, *, show_reasons: bool = True) -> None:
    print("\nNuwax · 模块管理\n")
    print(f"{hardware.os_id} {hardware.version_id} · {hardware.architecture} · {hardware.cpu_count} 核 · {hardware.memory_bytes / 1024 ** 3:.1f} GiB（吉字节）内存 · {hardware.free_disk_bytes / 1024 ** 3:.1f} GiB 可用磁盘")
    print("图形设备：{}；编译并行：{}".format(
        "Jetson（嵌入式计算平台）" if hardware.is_jetson else "NVIDIA（英伟达）" if hardware.nvidia_gpu else "未检测到可用的 NVIDIA 图形设备",
        hardware.build_jobs,
    ))
    print("摄像头：" + ("检测到本地视频设备" if hardware.camera_detected else "未检测到本地视频设备"))
    for reason in (recommendation.reasons if show_reasons else ()) + recommendation.warnings:
        print("  " + reason)


def _show_modules(selected: Sequence[str]) -> None:
    print()
    for number, (key, spec) in enumerate(MODULES.items(), 1):
        mark = "✓" if key in selected else " "
        print(f"  {number} [{mark}] {key:<12} {spec.title}")


def _show_plan(plan: dict) -> None:
    print("\n目标模块：" + "、".join(MODULES[key].title for key in plan["modules"]))
    print("机器人接入：%s" % ("Go2（宇树四足机器人）" if plan["robot"] == "go2" else "标准输入（设备型号和传感器待接入）"))
    for label, key in (("新增", "add"), ("移除", "remove")):
        if plan[key]:
            print(label + "：" + "、".join(MODULES[name].title for name in plan[key]))
    print("依赖清单：" + "、".join(plan["requirements"]))
    print("系统依赖（只安装缺项）：" + ", ".join(plan["system_packages"]))
    for step in plan["builds"]:
        print("  " + step)
    if plan["models"]:
        print("模型：仅下载已选模块所需模型；已有模型继续保留。")
    if "navigation" in plan["modules"]:
        print("导航和跟随运行需要已配置的运动安全守卫与真实执行器；安装不会启动机器人。")
    print("安装位置：" + plan["install_dir"])


def _wizard(selected: tuple, recommended: tuple) -> Optional[tuple]:
    keys = tuple(MODULES)
    while True:
        _show_modules(selected)
        print("\n回车使用以上方案；输入编号增删（如 3 6）；r 恢复推荐；q 退出。")
        answer = input("选择 › ").strip().lower()
        if not answer:
            return selected
        if answer == "q":
            return None
        if answer == "r":
            selected = recommended
            continue
        try:
            for token in answer.replace(",", " ").split():
                number = int(token)
                if not 1 <= number <= len(keys):
                    raise ValueError("编号超出范围")
                key = keys[number - 1]
                if key in selected:
                    selected = remove_modules(selected, (key,), cascade=True)
                elif key in ("fastlio2", "pointlio"):
                    selected = replace_lio(selected, key)
                else:
                    selected = resolve_modules(selected + (key,))
        except ValueError as exc:
            print("  " + str(exc))


def _stop(current: dict, root: Path) -> int:
    prefix = Path(current["prefix"])
    values = read_settings(prefix / "settings.env")
    pid_file = Path(values.get("NUWAX_GATEWAY_PID_FILE") or root / "runtime_data/gateway/gateway.pid")
    try:
        pid = int(pid_file.read_text().strip())
        if pid <= 1:
            raise ValueError("无效进程编号")
        environment = (Path("/proc") / str(pid) / "environ").read_bytes().split(b"\0")
        identity = ("NUWAX_BOOTSTRAP_ENV_FILE=" + str(prefix / "settings.env")).encode()
        if identity not in environment:
            raise InstallError("进程不属于当前模块安装，未发送停止信号。")
        os.kill(pid, signal.SIGTERM)
        for _ in range(150):
            if not (Path("/proc") / str(pid)).exists():
                print("当前安装的网关已停止。")
                return 0
            time.sleep(0.2)
        raise InstallError("停止仍在收尾，请查看网关日志；安装环境已保留。")
    except (FileNotFoundError, ProcessLookupError):
        print("当前安装的网关没有运行。")
        return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    root = Path(__file__).resolve().parents[2]
    home = (args.install_dir or root / "runtime_data/installation").expanduser().resolve()
    try:
        if args.names and args.command not in ("add", "remove"):
            parser.error("模块名称只能跟在 add/remove 后；完整选择请用 --modules。")
        if args.command in ("add", "remove") and (not args.names or args.modules):
            parser.error("add/remove 需要模块名称，且不能与 --modules 同用。")
        current = read_current(home)
        if args.command == "list":
            if args.json:
                print(json.dumps(current, ensure_ascii=False, indent=2))
            elif current:
                print("当前已安装：")
                _show_modules(current["modules"])
                print("\n配置文件：{}/settings.env".format(current["prefix"]))
            else:
                print("尚未安装。运行 ./install.sh 自动检测并开始。")
            return 0
        if args.command in ("start", "stop"):
            if args.dry_run or args.json:
                print(json.dumps({"action": args.command, "prefix": current.get("prefix")}, ensure_ascii=False))
                return 0
            if not current:
                raise InstallError("尚未安装，请先运行 ./install.sh。")
            if args.command == "stop":
                return _stop(current, root)
            script = str(Path(current["prefix"]) / "start.sh")
            os.execv("/bin/bash", ["bash", script])
        hardware = detect_hardware(home)
        existing = existing_config(root, current)
        if args.robot != "auto":
            existing["NUWAX_ACTIVE_ROBOT"] = args.robot
        recommendation = recommend_modules(hardware, existing, current.get("modules"))
        selected = resolve_modules(_names([args.modules])) if args.modules is not None else recommendation.modules
        if args.command == "add":
            names = _names(args.names)
            if "fastlio2" in names and "pointlio" in names:
                raise InstallError("一次只能选择一种激光惯性定位算法。")
            selected = tuple(current.get("modules", ("core",)))
            for name in names:
                selected = replace_lio(selected, name) if name in ("fastlio2", "pointlio") else resolve_modules(selected + (name,))
        elif args.command == "remove":
            if not current:
                raise InstallError("尚无受管安装可移除；现有开发环境不会被当作安装器产物删除。")
            selected = remove_modules(current["modules"], _names(args.names), cascade=True)
        if args.lio:
            selected = replace_lio(selected, args.lio)
        robot = args.robot if args.robot != "auto" else current.get("robot", recommendation.robot)
        if robot not in ("customrobot", "go2"):
            robot = "customrobot"
        jobs = args.jobs if args.jobs is not None else hardware.build_jobs
        if jobs < 1:
            raise InstallError("--jobs 必须为正整数。")
        wheel_dir = args.wheel_dir
        if wheel_dir is None:
            cached = home / "cache/device-wheels"
            conventional = root / "third_party/wheels"
            wheel_dir = cached if cached.is_dir() else conventional if conventional.is_dir() else None
        request = InstallRequest(root, home, tuple(selected), robot, jobs,
                                 args.python or current.get("python", ""),
                                 wheel_dir.expanduser().resolve() if wheel_dir else None, args.repair)
        preview = args.dry_run or args.json or args.command == "doctor"
        if not args.json:
            _show_hardware(hardware, recommendation, show_reasons=args.modules is None and args.command in ("manage", "install", "doctor"))
        if args.command == "manage" and not preview and not args.yes:
            if not sys.stdin.isatty():
                print("\n非交互环境只展示计划；实际安装请使用 install --yes。")
                preview = True
            else:
                selected = _wizard(request.modules, recommend_modules(hardware, existing).modules)
                if selected is None:
                    return 0
                request = replace(request, modules=selected)
        plan = make_plan(request, hardware, current)
        if args.json:
            print(json.dumps({"hardware": hardware.to_dict(), "recommendation": recommendation.to_dict(), "plan": plan}, ensure_ascii=False, indent=2))
        else:
            _show_plan(plan)
        if preview:
            return 0
        if not args.yes:
            if not sys.stdin.isatty():
                raise InstallError("非交互安装请加 --yes；查看计划可用 --dry-run。")
            if input("\n确认应用以上安装方案？[Y/n] › ").strip().lower() not in ("", "y", "yes"):
                return 0
        def cancel_install(_signum, _frame):
            raise KeyboardInterrupt

        previous_handler = signal.signal(signal.SIGTERM, cancel_install)
        try:
            result = apply(request, hardware)
        finally:
            signal.signal(signal.SIGTERM, previous_handler)
        print("\n安装完成：" + "、".join(MODULES[key].title for key in result["modules"]))
        print("启动：./install.sh start" + (" --install-dir " + str(home) if args.install_dir else ""))
        print("以后运行 ./install.sh 可继续增删模块。")
        return 0
    except KeyboardInterrupt:
        print("\n已取消，上一有效安装保留。", file=sys.stderr)
        return 130
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
        print(f"\n无法完成：{exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
