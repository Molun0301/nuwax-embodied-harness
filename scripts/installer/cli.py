#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""installer CLI 入口：argparse + 5 子命令 + 父级 --phase-override。

设计要点（plan 4.1 L103 / 4.4 L162-178 / spec "单入口闭环"）：

- 5 个子命令：``check`` / ``wizard`` / ``apply`` / ``resume`` / ``profile``；
  其中 ``profile`` 是父命令，下含 ``show`` / ``edit`` / ``diff`` 三子命令。
- 父级选项（作用于前 4 个子命令）：
  * ``--phase-override <phase>=<exit-code>``（可重复，parse_phase_overrides 解析）。
  * ``--robot`` / ``--tier`` / ``--non-interactive`` / ``--dry-run`` / ``--offline``
  * ``--profile`` / ``--save-profile``
- 中文帮助（AC9）；退出码走 ``InstallerErrorCode`` 体系。
- 本文件是 task 1 骨架；具体子命令实装由 task 5（installer-ac-coverage）补全。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Optional, Sequence

from scripts.installer.errors import InstallerError, InstallerErrorCode
from scripts.installer.planner import (
    InstallPlanner,
    RuntimeOptions,
    coerce_bool_flag,
)
from scripts.installer.phases import DEFAULT_PHASE_ORDER, parse_phase_overrides


# ---------------------------------------------------------------------------
# 父级选项（plan 4.4 L162-178）
# ---------------------------------------------------------------------------

def _add_parent_flags(parent: argparse.ArgumentParser) -> None:
    """所有子命令共享的父级选项。

    注意：``--phase-override`` 用 ``action="append"``，允许命令行重复。
    实际解析在 :func:`parse_phase_overrides` 完成（plan 4.4 L180-194）。
    """
    parent.add_argument(
        "--phase-override",
        action="append",
        default=[],
        metavar="PHASE=EXIT_CODE",
        help=(
            "测试钩子：将指定 phase 的退出码覆盖为 EXIT_CODE（用于断点续跑与错误注入单测），"
            "可重复。格式: PHASE=exit-<int> 或 =skip。"
            "示例: --phase-override build_ws=exit-1 --phase-override models=skip"
        ),
    )
    parent.add_argument(
        "--robot",
        choices=("go2", "g1", "customrobot"),
        default=None,
        help="机器人类型；不指定时走 wizard / profile 默认。",
    )
    parent.add_argument(
        "--tier",
        choices=("full", "lite"),
        default=None,
        help="部署档位；full 默认开启 YOLO-World engine 导出，lite 仅下 .pt。",
    )
    parent.add_argument(
        "--non-interactive",
        action="store_true",
        help="非交互模式（跳过 wizard 提问，全部走默认值）。",
    )
    parent.add_argument(
        "--dry-run",
        action="store_true",
        help="dry-run 模式：所有副作用只打印命令不执行，且不写 state 文件。",
    )
    parent.add_argument(
        "--offline",
        action="store_true",
        help="离线模式：拒绝所有网络入口（apt / pip / docker / curl / wget）。",
    )
    parent.add_argument(
        "--profile",
        default="",
        metavar="PATH",
        help="profile JSON 路径（绝对路径覆盖默认 ~/.config/nuwax/installer/）。",
    )
    parent.add_argument(
        "--save-profile",
        default="",
        metavar="PATH",
        help="wizard 完成后把配置保存到指定 profile JSON 路径。",
    )
    # 15 个 bootstrap boolean flag（plan / bootstrap.py:1522-1536）的简化复刻；
    # 每个 phase 实现读取 ``planner.<flag>``，不直接读 argparse。
    _add_bootstrap_boolean_flags(parent)


def _add_bootstrap_boolean_flags(parent: argparse.ArgumentParser) -> None:
    """复刻 bootstrap.add_boolean_flag：每个 flag 同时支持 ``--xxx`` 与 ``--no-xxx``。"""
    flags = (
        "geometry",
        "data_plane",
        "systemd",
        "relay",
        "install_nuwaclaw_docker",
        "install_python",
        "prepare_memory",
        "prepare_yolo",
        "export_yolo_engine",
        "build_geometry",
        "build_go2slam",
        "install_geometry_deps",
        "repair_ros2_apt_source",
        "start_gateway",
        "healthcheck",
    )
    for name in flags:
        parent.add_argument(
            f"--{name.replace('_', '-')}",
            dest=name,
            action="store_true",
            default=None,
            help=f"打开 {name}（与 --no-{name.replace('_', '-')} 配对；None 表示走默认值）",
        )
        parent.add_argument(
            f"--no-{name.replace('_', '-')}",
            dest=name,
            action="store_false",
            help=f"关闭 {name}",
        )


# ---------------------------------------------------------------------------
# 子命令占位（task 5 替换为完整实装）
# ---------------------------------------------------------------------------


def _run_check(args: argparse.Namespace, runtime: RuntimeOptions) -> int:
    """``check`` 子命令：探测 + 打印，**不**改系统（plan "check" 语义）。"""
    planner = _build_planner_from_args(args, runtime)
    print("[cli: check] 环境探测（task 1 骨架，后续 task 5 实装）")
    print(planner.summary())
    return 0


def _run_wizard(args: argparse.Namespace, runtime: RuntimeOptions) -> int:
    """``wizard`` 子命令：交互式向导（task 1 骨架）。

    真实 questionary 集成由 task 2 (installer-ui-fallback) 实装；
    当前走 dataclass 默认值 + summary 打印。
    """
    planner = _build_planner_from_args(args, runtime)
    print("[cli: wizard] 交互式向导（task 1 骨架，后续 task 2 实装 questionary 集成）")
    print(planner.summary())
    # profile 保存（task 3 实装 schema 守门；当前 placeholder 不调用）
    if runtime.save_profile_path:
        print(f"[cli: wizard] （占位）将保存 profile 到 {runtime.save_profile_path}")
    return 0


def _run_apply(args: argparse.Namespace, runtime: RuntimeOptions) -> int:
    """``apply`` 子命令：按配置执行（task 1 骨架）。

    真实 phase 执行由 task 5 (installer-ac-coverage) 实装；
    当前仅做参数解析 + 打印 phase 列表。
    """
    planner = _build_planner_from_args(args, runtime)
    warnings = planner.validate_mutex()
    for warning in warnings:
        print(f"[cli: apply] 警告: {warning}")
    print(f"[cli: apply] 将按以下顺序执行 {len(DEFAULT_PHASE_ORDER)} 个 phase（task 1 骨架）:")
    for phase_name in DEFAULT_PHASE_ORDER:
        override = runtime.phase_overrides.get(phase_name)
        suffix = f"  (override={override})" if override is not None else ""
        print(f"  - {phase_name}{suffix}")
    if runtime.dry_run:
        print("[cli: apply] dry-run：不写 state 文件，不执行副作用")
    return 0


def _run_resume(args: argparse.Namespace, runtime: RuntimeOptions) -> int:
    """``resume`` 子命令：从 state 续跑（task 1 骨架）。"""
    planner = _build_planner_from_args(args, runtime)
    from scripts.installer.state import DEFAULT_STATE_FILE, load_state

    state = load_state(DEFAULT_STATE_FILE)
    success_names = {p.name for p in state.phases if p.status == "success"}
    print(
        f"[cli: resume] state.json: 已成功 {len(success_names)} 个 phase = {sorted(success_names)}"
    )
    print("[cli: resume] （task 1 骨架；task 5 实装实际 phase 重试）")
    return 0


def _run_profile_show(args: argparse.Namespace) -> int:
    """``profile show`` 子命令：打印 profile JSON（task 1 骨架）。"""
    profile_path = args.profile_path or ""
    if not profile_path:
        print("[cli: profile show] 缺少 --profile 参数", file=sys.stderr)
        return 64
    path = Path(profile_path).expanduser()
    if not path.exists():
        print(f"[cli: profile show] profile 文件不存在: {path}", file=sys.stderr)
        return 64
    print(path.read_text(encoding="utf-8"))
    return 0


def _run_profile_edit(args: argparse.Namespace) -> int:
    """``profile edit`` 子命令：编辑 profile（task 1 骨架）。"""
    print("[cli: profile edit] （task 1 骨架；task 3 实装 wizard 引导编辑）")
    return 0


def _run_profile_diff(args: argparse.Namespace) -> int:
    """``profile diff`` 子命令：diff 两个 profile（task 1 骨架）。"""
    print("[cli: profile diff] （task 1 骨架；task 3 实装完整 diff）")
    return 0


# ---------------------------------------------------------------------------
# Planner / Runtime 构造
# ---------------------------------------------------------------------------


def _build_runtime(args: argparse.Namespace) -> RuntimeOptions:
    """从 argparse Namespace 构造 RuntimeOptions（不进入 InstallPlanner）。"""
    overrides = parse_phase_overrides(getattr(args, "phase_override", []) or [])
    return RuntimeOptions(
        dry_run=bool(getattr(args, "dry_run", False)),
        offline=bool(getattr(args, "offline", False)),
        non_interactive=bool(getattr(args, "non_interactive", False)),
        profile_path=str(getattr(args, "profile", "") or ""),
        save_profile_path=str(getattr(args, "save_profile", "") or ""),
        phase_overrides=overrides,
    )


def _build_planner_from_args(args: argparse.Namespace, runtime: RuntimeOptions) -> InstallPlanner:
    """从 argparse Namespace 构造 InstallPlanner；触发 tier 默认值与互斥校验。"""
    planner = InstallPlanner()
    # robot / tier 来自 CLI
    robot = getattr(args, "robot", None) or planner.robot
    tier = getattr(args, "tier", None) or planner.tier
    planner.robot = robot
    planner.tier = tier
    # 15 个 boolean flag：coerce_bool_flag 归一化；None 时保留 InstallPlanner 默认
    flag_names = (
        "geometry", "data_plane", "systemd", "relay", "install_nuwaclaw_docker",
        "install_python", "prepare_memory", "prepare_yolo", "export_yolo_engine",
        "build_geometry", "build_go2slam", "install_geometry_deps",
        "repair_ros2_apt_source", "start_gateway", "healthcheck",
    )
    for name in flag_names:
        coerced = coerce_bool_flag(getattr(args, name, None), default=None)
        if coerced is not None:
            setattr(planner, name, coerced)
    # tier 联动默认值（plan AC10）
    planner.apply_tier_defaults()
    return planner


# ---------------------------------------------------------------------------
# 顶层入口
# ---------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    """构造 argparse 解析器；中文帮助。"""
    parent = argparse.ArgumentParser(add_help=False)
    _add_parent_flags(parent)

    parser = argparse.ArgumentParser(
        prog="python3 install.py",
        description="nuwax 一键部署 installer（plan: openspec/changes/installer-redesign）",
        parents=[parent],
    )
    subparsers = parser.add_subparsers(dest="command", required=True, metavar="<子命令>")

    subparsers.add_parser("check", parents=[parent], help="只做探测，不改系统（plan check 语义）")
    subparsers.add_parser(
        "wizard",
        parents=[parent],
        help="交互式向导；支持 --save-profile / --profile",
    )
    subparsers.add_parser(
        "apply",
        parents=[parent],
        help="按 profile 或参数执行安装；支持 --dry-run / --resume / --offline",
    )
    subparsers.add_parser(
        "resume",
        parents=[parent],
        help="从 runtime_data/installer_state.json 断点续跑",
    )

    # profile 父命令（自身不继承 parent flags：profile 操作不需要 phase-override）
    profile_parser = subparsers.add_parser(
        "profile",
        help="profile 子命令：show / edit / diff",
    )
    profile_subparsers = profile_parser.add_subparsers(
        dest="profile_command", required=True, metavar="<profile-子命令>"
    )
    show = profile_subparsers.add_parser("show", help="显示 profile JSON 内容")
    show.add_argument("profile_path", nargs="?", default="", help="profile JSON 路径")
    edit = profile_subparsers.add_parser("edit", help="编辑 profile（task 3 实装）")
    edit.add_argument("profile_path", nargs="?", default="", help="profile JSON 路径")
    diff = profile_subparsers.add_parser("diff", help="比较两个 profile（task 3 实装）")
    diff.add_argument("profile_a", nargs="?", default="", help="profile A 路径")
    diff.add_argument("profile_b", nargs="?", default="", help="profile B 路径")

    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI 顶层入口；返回 exit code（InstallerErrorCode 体系）。"""
    parser = build_parser()
    try:
        args = parser.parse_args(argv)
    except SystemExit as exc:
        # argparse 错误退出；统一为 EX_USAGE (64)
        return int(getattr(exc, "code", 64) or 64)

    try:
        if args.command == "profile":
            if args.profile_command == "show":
                return _run_profile_show(args)
            if args.profile_command == "edit":
                return _run_profile_edit(args)
            if args.profile_command == "diff":
                return _run_profile_diff(args)
            parser.error("未知 profile 子命令")
            return int(InstallerErrorCode.UNKNOWN)

        runtime = _build_runtime(args)
        if args.command == "check":
            return _run_check(args, runtime)
        if args.command == "wizard":
            return _run_wizard(args, runtime)
        if args.command == "apply":
            return _run_apply(args, runtime)
        if args.command == "resume":
            return _run_resume(args, runtime)
        parser.error("未知命令")
        return int(InstallerErrorCode.UNKNOWN)
    except InstallerError as exc:
        print(exc.to_human(), file=sys.stderr)
        return int(exc.code) if isinstance(exc.code, int) else int(InstallerErrorCode.UNKNOWN)


__all__ = ["build_parser", "main"]