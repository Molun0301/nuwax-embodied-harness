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
- 本文件是 task 5（installer-ac-coverage）集成的最终形态：check / wizard / apply /
  resume 5 个子命令全部接通 prompts / planner / profile / _legacy_bridge / _lock / state。
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

from scripts.installer.errors import InstallerError, InstallerErrorCode, PhaseError
from scripts.installer.planner import (
    InstallPlanner,
    RuntimeOptions,
    coerce_bool_flag,
    print_wizard_next_step_banner,
    save_profile as planner_save_profile,
)
from scripts.installer.phases import DEFAULT_PHASE_ORDER, default_registry, parse_phase_overrides

# 触发 phases/*.py 模块级 register()；必须在首次调用 default_registry.get 之前完成
import scripts.installer.phases.preflight  # noqa: F401, E402
import scripts.installer.phases.system  # noqa: F401, E402
import scripts.installer.phases.python_env  # noqa: F401, E402
import scripts.installer.phases.geometry_deps  # noqa: F401, E402
import scripts.installer.phases.config_env  # noqa: F401, E402
import scripts.installer.phases.models  # noqa: F401, E402
import scripts.installer.phases.systemd  # noqa: F401, E402
import scripts.installer.phases.mcp  # noqa: F401, E402
import scripts.installer.phases.nuwaclaw  # noqa: F401, E402
import scripts.installer.phases.health  # noqa: F401, E402


# ---------------------------------------------------------------------------
# PhaseContext：phase.run(planner, ctx) 的 ctx 参数。
#
# 与 task 5 之前的 phase 文件调用习惯（ctx.info / ctx.error）保持兼容。
# info/warn/error/success 走 ui/console（rich 优先，ANSI 降级）；
# print 走纯 stdout（与原 task 1 占位 print 调用兼容）。
# ---------------------------------------------------------------------------


@dataclass
class PhaseContext:
    """phase 执行上下文：dry_run / offline / 非交互标志 + UI 通道。"""

    dry_run: bool = False
    offline: bool = False
    non_interactive: bool = False
    # 离线模式阶段列表（NETWORK_HOOKS guard 触发阶段名）
    offline_strict_phases: tuple = ("system", "geometry_deps", "models", "nuwaclaw", "health")

    def _check_offline(self, phase_name: str) -> None:
        """离线模式下，被列入 strict_phases 的阶段拒绝执行（仅 dry_run 允许）。"""
        if self.offline and phase_name in self.offline_strict_phases and not self.dry_run:
            raise InstallerError(
                message=f"--offline 模式下拒绝执行网络相关阶段 {phase_name!r}",
                code=InstallerErrorCode.OFFLINE_REJECT_NETWORK,
                remediation="去掉 --offline 重跑，或确认该阶段无需网络（apt/pip/docker/curl）。",
            )

    def guard_network(self, phase_name: str) -> None:
        """NETWORK_HOOKS guard 入口：供 phase 顶层调用。"""
        self._check_offline(phase_name)

    # UI 通道（与 ui/console 一致；rich 不可用时降级 ANSI）
    def info(self, msg: str) -> None:
        from scripts.installer.ui import console as _console
        _console.info(msg)

    def warn(self, msg: str) -> None:
        from scripts.installer.ui import console as _console
        _console.warn(msg)

    def error(self, msg: str) -> None:
        from scripts.installer.ui import console as _console
        _console.error(msg)

    def success(self, msg: str) -> None:
        from scripts.installer.ui import console as _console
        _console.success(msg)


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
            "示例: --phase-override models=exit-1 --phase-override nuwaclaw=skip"
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
# 子命令实装（task 5 / installer-ac-coverage 集成）
# ---------------------------------------------------------------------------


def _build_ctx(runtime: RuntimeOptions) -> PhaseContext:
    """从 RuntimeOptions 构造 PhaseContext（供所有 phase.run/dry_run 调用）。"""
    return PhaseContext(
        dry_run=bool(runtime.dry_run),
        offline=bool(runtime.offline),
        non_interactive=bool(runtime.non_interactive),
    )


def _run_check(args: argparse.Namespace, runtime: RuntimeOptions) -> int:
    """``check`` 子命令：探测 + 打印，**不**改系统（plan "check" 语义）。

    实现：
    - 跑 preflight phase 的 dry_run 路径（不触发副作用）；
    - 打印 planner.summary()。
    """
    planner = _build_planner_from_args(args, runtime)
    planner.validate_mutex()
    ctx = _build_ctx(runtime)
    print("[cli: check] 环境探测（不修改系统）")
    print(planner.summary())
    print()
    # 跑 preflight phase 的 dry_run 路径（无副作用）
    cls = default_registry.get("preflight")
    phase = cls()
    cmds = phase.dry_run(planner, ctx)
    for line in cmds:
        print(f"+ {line}")
    # 补充探测：CPU 架构 / Python / 磁盘
    import platform
    import shutil
    print(f"+ OS={platform.system()} {platform.release()}")
    print(f"+ arch={platform.machine()}")
    print(f"+ python={platform.python_version()}")
    print(f"+ curl={'available' if shutil.which('curl') else 'missing'}")
    return 0


def _run_wizard(args: argparse.Namespace, runtime: RuntimeOptions) -> int:
    """``wizard`` 子命令：交互式向导（task 5 集成主入口）。

    - ``--non-interactive`` 跳过所有问题，用默认值；
    - ``--save-profile PATH`` 保存到指定路径；
    - 完成后打印 ``下一步: bash scripts/start_gateway.sh start 启动 gateway``。
    """
    from scripts.installer import prompts as _prompts

    planner = _build_planner_from_args(args, runtime)
    planner.validate_mutex()
    non_interactive = bool(runtime.non_interactive) or not sys.stdin.isatty()

    # 启动横幅
    try:
        from scripts.installer.ui.banner import print_banner
        print_banner()
    except Exception as exc:  # pragma: no cover - 容错
        print(f"[cli: wizard] （启动横幅失败: {exc}）")

    # 问题清单（plan F4 / spec 单入口闭环）
    print()
    print("[cli: wizard] 开始交互式配置（Ctrl-C 随时退出）")
    print()

    # 1. 机器人类型
    if not non_interactive:
        robot = _prompts.ask_select(
            "机器人类型",
            choices=("go2", "g1", "customrobot"),
            default=planner.robot,
        )
        planner.robot = robot
    else:
        print(f"机器人类型: {planner.robot}（non-interactive，使用默认）")

    # 2. 部署档位
    if not non_interactive:
        tier = _prompts.ask_select(
            "部署档位",
            choices=("full", "lite"),
            default=planner.tier,
        )
        planner.tier = tier
    else:
        print(f"部署档位: {planner.tier}（non-interactive，使用默认）")

    # tier 联动默认值（必须放在 wizard 显式选择之后）
    planner.apply_tier_defaults()

    is_full = planner.tier == "full"
    is_go2 = planner.robot == "go2"

    # 3. memory 模式
    if not non_interactive:
        planner.memory_mode = _prompts.ask_select(
            "memory 模式",
            choices=("cloud", "local", "disabled"),
            default="cloud" if is_full else "disabled",
        )
    else:
        print(f"memory 模式: {planner.memory_mode}（non-interactive，使用默认）")

    # 4. perception 模式
    if not non_interactive:
        planner.perception_mode = _prompts.ask_select(
            "perception 模式",
            choices=("local", "hybrid", "cloud-scene", "disabled"),
            default="hybrid" if is_full else "local",
        )
    else:
        print(f"perception 模式: {planner.perception_mode}（non-interactive，使用默认）")

    # 5. geometry 平台侧车
    if not non_interactive:
        planner.geometry = _prompts.ask_confirm(
            "启用 geometry 平台侧车（SLAM/Nav2）",
            default=is_full,
        )
    else:
        print(f"geometry: {planner.geometry}（non-interactive，使用默认）")

    # 6. Go2 数据面
    if is_go2:
        if not non_interactive:
            planner.data_plane = _prompts.ask_confirm(
                "启用 Go2 数据面（drivers/go2/data_plane.env）",
                default=is_full,
            )
        else:
            print(f"Go2 数据面: {planner.data_plane}（non-interactive，使用默认）")
    else:
        planner.data_plane = False

    # 7. YOLO-World TensorRT engine 导出（task 7 AC10）
    if not non_interactive:
        planner.export_yolo_engine = _prompts.ask_confirm(
            "导出 YOLO-World TensorRT engine（tier full 推荐）",
            default=is_full,
        )
    else:
        print(f"export_yolo_engine: {planner.export_yolo_engine}（non-interactive，使用默认）")

    # 8. MCP 注入模式
    if not non_interactive:
        planner.mcp_mode = _prompts.ask_select(
            "MCP 注入模式",
            choices=("auto", "config-only", "skip"),
            default="auto",
        )
    else:
        print(f"MCP 模式: {planner.mcp_mode}（non-interactive，使用默认）")

    # 9. 启用 relay
    if not non_interactive:
        planner.relay = _prompts.ask_confirm(
            "启用 relay",
            default=False,
        )
    else:
        print(f"relay: {planner.relay}（non-interactive，使用默认）")

    # 10. 生成 systemd 示例
    if not non_interactive:
        planner.systemd = _prompts.ask_confirm(
            "生成 systemd 示例（deploy/systemd/）",
            default=False,
        )
    else:
        print(f"systemd: {planner.systemd}（non-interactive，使用默认）")

    # 11. 安装 NuwaxClaw Docker 客户端
    if not non_interactive:
        planner.install_nuwaclaw_docker = _prompts.ask_confirm(
            "安装 NuwaxClaw Docker 客户端（默认关闭；启用需注意 host mount 风险）",
            default=False,
        )
    else:
        print(f"NuwaxClaw Docker: {planner.install_nuwaclaw_docker}（non-interactive，使用默认）")

    if planner.install_nuwaclaw_docker:
        if not non_interactive:
            planner.nuwaclaw_host_mount = _prompts.ask_select(
                "NuwaxClaw 宿主机挂载模式",
                choices=("none", "ro", "rw"),
                default="none",
            )
            if planner.nuwaclaw_host_mount == "rw":
                # 双重确认（RK6）
                confirmed = _prompts.ask_confirm(
                    "确认把宿主机根目录以 rw 模式挂载？此操作有宿主机文件被改风险",
                    default=False,
                )
                if not confirmed:
                    planner.nuwaclaw_host_mount = "ro"
                    print("[cli: wizard] 已自动降级为 ro")
            planner.nuwaclaw_saved_key = _prompts.ask_password(
                "请输入 NuwaxClaw 密钥（NUWAX_SAVED_KEY）"
            ) if planner.nuwaclaw_host_mount != "none" else ""
        else:
            print(f"NuwaxClaw host_mount: {planner.nuwaclaw_host_mount}（non-interactive）")

    # tier 联动默认值（最后一次，确保所有 wizard 显式选择覆盖生效）
    planner.apply_tier_defaults()
    warnings = planner.validate_mutex()
    for w in warnings:
        print(f"[cli: wizard] 警告: {w}")

    print()
    print("[cli: wizard] 配置摘要：")
    print(planner.summary())

    # profile 保存
    if runtime.save_profile_path:
        try:
            written = planner_save_profile(runtime.save_profile_path, planner)
            print()
            print(f"[cli: wizard] profile 已保存: {written}")
        except InstallerError as exc:
            print(f"[cli: wizard] profile 保存失败: {exc.to_human()}", file=sys.stderr)
            return int(exc.code) if isinstance(exc.code, int) else int(InstallerErrorCode.UNKNOWN)

    # 下一步 banner
    print()
    print_wizard_next_step_banner()
    return 0


def _should_skip_phase(phase_name: str, planner: InstallPlanner, runtime: RuntimeOptions) -> bool:
    """判断 phase 是否应跳过。

    规则（按顺序）：
    1. ``--phase-override <phase>=skip`` → 跳过
    2. planner.<对应 flag> == False → 跳过（如 planner.install_python=False 跳过 python_env）
    """
    if runtime.phase_overrides.get(phase_name) == "skip":
        return True
    # 每个 phase 对应的 planner flag（如未对应则视为不可跳）
    skip_flags: Dict[str, str] = {
        "python_env": "install_python",
        "geometry_deps": "install_geometry_deps",
        "systemd": "systemd",
        "nuwaclaw": "install_nuwaclaw_docker",
        "health": "healthcheck",
    }
    flag_name = skip_flags.get(phase_name)
    if flag_name and not bool(getattr(planner, flag_name, True)):
        return True
    return False


def _execute_phase(
    phase_name: str,
    planner: InstallPlanner,
    runtime: RuntimeOptions,
    ctx: PhaseContext,
) -> int:
    """执行单个 phase；处理 dry_run / override / 离线守卫。

    返回 phase 退出码（0=成功）。
    """
    # 离线守卫（仅 dry-run 时放行，便于用户审阅；非 dry-run 直接抛错）
    if runtime.offline and phase_name in ctx.offline_strict_phases:
        if not runtime.dry_run:
            raise InstallerError(
                message=f"--offline 模式下拒绝执行网络相关阶段 {phase_name!r}",
                code=InstallerErrorCode.OFFLINE_REJECT_NETWORK,
                remediation="去掉 --offline 重跑，或确认该阶段无需网络（apt/pip/docker/curl）。",
            )

    override = runtime.phase_overrides.get(phase_name)
    if override == "skip":
        print(f"[cli: apply] 跳过 phase: {phase_name}（override=skip）")
        return 0
    if isinstance(override, int):
        # 错误注入（测试钩子）：模拟失败
        print(f"[cli: apply] phase {phase_name} 被 override 强制退出码 {override}")
        return int(override)

    cls = default_registry.get(phase_name)
    phase = cls()

    if runtime.dry_run:
        cmds = phase.dry_run(planner, ctx) or []
        for line in cmds:
            print(f"+ {line}")
        return 0

    return int(phase.run(planner, ctx) or 0)


def _write_phase_record(
    state,
    name: str,
    order: int,
    status: str,
    started_at: float,
    finished_at: float,
    message: str,
) -> None:
    """写一条 PhaseRecord 到 state 并落盘。"""
    from scripts.installer.state import (
        DEFAULT_STATE_FILE,
        PhaseRecord,
        save_state,
        upsert_phase,
    )

    rec = PhaseRecord(
        name=name,
        order=order,
        status=status,  # type: ignore[arg-type]
        started_at=started_at,
        finished_at=finished_at,
        message=message,
    )
    upsert_phase(state, rec)
    save_state(state, DEFAULT_STATE_FILE)


def _run_apply(args: argparse.Namespace, runtime: RuntimeOptions) -> int:
    """``apply`` 子命令：按 profile 或参数执行所有 phase。

    - ``--dry-run`` 调每个 phase.dry_run(planner, ctx) 输出 ``+ <cmd>``；
    - 顺序遍历 ``DEFAULT_PHASE_ORDER``，每个 phase 前调离线守卫 + planner flag 跳过判断；
    - 失败抛 PhaseError（错误码 1）；最后释放 lock。
    """
    from scripts.installer._lock import acquire_lock, release_lock, LockBusy
    from scripts.installer.state import (
        DEFAULT_STATE_FILE,
        InstallerState,
        load_state,
        save_state,
    )

    planner = _build_planner_from_args(args, runtime)

    # profile 重放（--profile 给出时覆盖 planner 字段）
    if runtime.profile_path:
        from scripts.installer.planner import load_profile
        try:
            planner = load_profile(runtime.profile_path)
        except InstallerError as exc:
            print(f"[cli: apply] profile 加载失败: {exc.to_human()}", file=sys.stderr)
            return int(exc.code) if isinstance(exc.code, int) else int(InstallerErrorCode.UNKNOWN)

    warnings = planner.validate_mutex()
    for w in warnings:
        print(f"[cli: apply] 警告: {w}")

    # 打印启动横幅 + 配置摘要
    try:
        from scripts.installer.ui.banner import print_banner
        print_banner()
    except Exception:  # pragma: no cover
        pass
    print()
    print("[cli: apply] 配置摘要：")
    print(planner.summary())
    print()

    # 状态文件加载（dry_run 不写）
    state = load_state(DEFAULT_STATE_FILE) if not runtime.dry_run else InstallerState()
    if not runtime.dry_run:
        save_state(state, DEFAULT_STATE_FILE)

    ctx = _build_ctx(runtime)

    # 锁（dry_run 跳过锁：dry-run 不冲突）
    if not runtime.dry_run:
        try:
            acquire_lock(timeout=0)
        except LockBusy as exc:
            raise InstallerError(
                message=f"gateway lock 已被占用（{exc.lock_path}），无法启动 installer。",
                code=InstallerErrorCode.LOCK_HELD,
                remediation="先停掉持有 lock 的进程，或运行 'bash scripts/start_gateway.sh stop'。",
            ) from exc

    failed_phase: Optional[str] = None
    try:
        for idx, phase_name in enumerate(DEFAULT_PHASE_ORDER):
            print(f"\n[cli: apply] ({idx + 1}/{len(DEFAULT_PHASE_ORDER)}) phase: {phase_name}")

            if _should_skip_phase(phase_name, planner, runtime):
                print(f"[cli: apply] 跳过（planner flag 关闭 或 override=skip）: {phase_name}")
                continue

            started = time.time()
            try:
                rc = _execute_phase(phase_name, planner, runtime, ctx)
            except InstallerError:
                raise
            except Exception as exc:
                # phase 内未归一化的异常 → 包成 PhaseError
                raise PhaseError(
                    phase=phase_name,
                    message=str(exc),
                    file_path=f"scripts/installer/phases/{phase_name}.py",
                    code=InstallerErrorCode.PHASE_FAILED,
                    remediation=f"检查 phase {phase_name} 的依赖与权限后重跑。",
                ) from exc
            finished = time.time()

            if rc != 0:
                if not runtime.dry_run:
                    _write_phase_record(
                        state, phase_name, idx, "failed", started, finished,
                        f"exit={rc}",
                    )
                failed_phase = phase_name
                # 非 dry-run 立即停
                if not runtime.dry_run:
                    raise InstallerError(
                        message=f"phase {phase_name} 失败（exit={rc}）",
                        code=InstallerErrorCode.PHASE_FAILED,
                        file_path=f"scripts/installer/phases/{phase_name}.py",
                        remediation="修复后重跑 'python3 install.py apply'；失败阶段已写入 state.json，可用 'resume' 续跑。",
                    )
                print(f"[cli: apply] phase {phase_name} 在 dry-run 下返回非零（仅打印，不阻断）")

            if not runtime.dry_run:
                _write_phase_record(
                    state, phase_name, idx, "success", started, finished, "ok"
                )

        if runtime.dry_run:
            print("\n[cli: apply] dry-run 完成：未执行任何副作用，未写 state 文件。")
        else:
            state.last_status = "success" if not failed_phase else "failed"
            save_state(state, DEFAULT_STATE_FILE)
            print("\n[cli: apply] 所有 phase 执行完成。")
            print_wizard_next_step_banner()
    finally:
        if not runtime.dry_run:
            try:
                release_lock()
            except Exception:  # pragma: no cover - 释放锁失败不阻断主流程
                pass

    return 0


def _run_resume(args: argparse.Namespace, runtime: RuntimeOptions) -> int:
    """``resume`` 子命令：从 state.json 断点续跑。

    - 跳过 status=="success" 的 phase；
    - 重试 status in {"pending", "failed"} 的 phase；
    - 其他 status（如 "running" 中断遗留）按重试处理。
    """
    from scripts.installer._lock import acquire_lock, release_lock, LockBusy
    from scripts.installer.state import DEFAULT_STATE_FILE, load_state, save_state

    planner = _build_planner_from_args(args, runtime)
    if runtime.profile_path:
        from scripts.installer.planner import load_profile
        try:
            planner = load_profile(runtime.profile_path)
        except InstallerError as exc:
            print(f"[cli: resume] profile 加载失败: {exc.to_human()}", file=sys.stderr)
            return int(exc.code) if isinstance(exc.code, int) else int(InstallerErrorCode.UNKNOWN)

    state = load_state(DEFAULT_STATE_FILE)
    success_names = {p.name for p in state.phases if p.status == "success"}
    todo = [n for n in DEFAULT_PHASE_ORDER if n not in success_names]
    print(f"[cli: resume] state.json: 已成功 {len(success_names)} 个 phase，跳过: {sorted(success_names)}")
    print(f"[cli: resume] 将继续执行 {len(todo)} 个 phase: {todo}")

    ctx = _build_ctx(runtime)

    if not runtime.dry_run:
        try:
            acquire_lock(timeout=0)
        except LockBusy as exc:
            raise InstallerError(
                message=f"gateway lock 已被占用（{exc.lock_path}），无法 resume。",
                code=InstallerErrorCode.LOCK_HELD,
                remediation="先停掉持有 lock 的进程。",
            ) from exc

    try:
        for phase_name in todo:
            print(f"\n[cli: resume] phase: {phase_name}")
            if _should_skip_phase(phase_name, planner, runtime):
                print(f"[cli: resume] 跳过: {phase_name}")
                continue
            started = time.time()
            try:
                rc = _execute_phase(phase_name, planner, runtime, ctx)
            except Exception as exc:
                raise PhaseError(
                    phase=phase_name,
                    message=str(exc),
                    file_path=f"scripts/installer/phases/{phase_name}.py",
                    code=InstallerErrorCode.PHASE_FAILED,
                ) from exc
            finished = time.time()
            if rc != 0 and not runtime.dry_run:
                _write_phase_record(state, phase_name, 0, "failed", started, finished, f"exit={rc}")
                raise InstallerError(
                    message=f"resume 阶段 {phase_name} 失败（exit={rc}）",
                    code=InstallerErrorCode.PHASE_FAILED,
                    file_path=f"scripts/installer/phases/{phase_name}.py",
                )
            if not runtime.dry_run:
                _write_phase_record(state, phase_name, 0, "success", started, finished, "ok")

        if not runtime.dry_run:
            state.last_status = "success"
            save_state(state, DEFAULT_STATE_FILE)
        print("\n[cli: resume] 续跑完成。")
        print_wizard_next_step_banner()
    finally:
        if not runtime.dry_run:
            try:
                release_lock()
            except Exception:  # pragma: no cover
                pass

    return 0


def _run_profile_show(args: argparse.Namespace) -> int:
    """``profile show`` 子命令：打印 profile JSON。"""
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
    """``profile edit`` 子命令：wizard 引导编辑（task 5 最小实装）。"""
    profile_path = args.profile_path or ""
    if not profile_path:
        print("[cli: profile edit] 缺少 --profile 参数", file=sys.stderr)
        return 64
    path = Path(profile_path).expanduser()
    if not path.exists():
        print(f"[cli: profile edit] profile 文件不存在: {path}", file=sys.stderr)
        return 64

    from scripts.installer.planner import load_profile, save_profile

    try:
        planner = load_profile(path)
    except InstallerError as exc:
        print(f"[cli: profile edit] 加载失败: {exc.to_human()}", file=sys.stderr)
        return int(exc.code) if isinstance(exc.code, int) else int(InstallerErrorCode.UNKNOWN)

    print(f"[cli: profile edit] 当前 robot={planner.robot} tier={planner.tier} export_yolo_engine={planner.export_yolo_engine}")
    print("[cli: profile edit] （最小实装：仅打印摘要；后续可扩展为 wizard 引导）")
    print(planner.summary())
    return 0


def _run_profile_diff(args: argparse.Namespace) -> int:
    """``profile diff`` 子命令：diff 两个 profile。"""
    a_path = Path(args.profile_a or "").expanduser()
    b_path = Path(args.profile_b or "").expanduser()
    if not a_path.exists() or not b_path.exists():
        print(f"[cli: profile diff] 文件不存在: a={a_path} b={b_path}", file=sys.stderr)
        return 64
    try:
        a = json.loads(a_path.read_text(encoding="utf-8"))
        b = json.loads(b_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        print(f"[cli: profile diff] JSON 解析失败: {exc}", file=sys.stderr)
        return 1
    diff_keys = sorted(set(a.keys()) | set(b.keys()))
    print(f"[cli: profile diff] a={a_path} b={b_path}")
    for k in diff_keys:
        if k in ("_schema_version", "created_at", "installer_version"):
            continue
        if a.get(k) != b.get(k):
            print(f"  diff: {k}: {a.get(k)!r} -> {b.get(k)!r}")
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
    edit = profile_subparsers.add_parser("edit", help="编辑 profile（task 5 实装最小形态）")
    edit.add_argument("profile_path", nargs="?", default="", help="profile JSON 路径")
    diff = profile_subparsers.add_parser("diff", help="比较两个 profile")
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
    except PhaseError as exc:
        print(exc.to_human(), file=sys.stderr)
        return int(exc.code) if isinstance(exc.code, int) else int(InstallerErrorCode.UNKNOWN)
    except InstallerError as exc:
        print(exc.to_human(), file=sys.stderr)
        return int(exc.code) if isinstance(exc.code, int) else int(InstallerErrorCode.UNKNOWN)


__all__ = ["build_parser", "main", "PhaseContext"]