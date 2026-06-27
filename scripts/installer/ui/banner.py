"""installer.ui.banner：ASCII 启动横幅 + 版本号 + 探测结果展示。

设计目标：
- 启动 installer 时打印醒目横幅（含项目名 + 版本号）。
- 紧接一行汇总 OS / 架构 / Python 版本 / tty 状态 / 双模式门判定。
- rich 不可用时降级为纯文本横幅（仍对齐 + 含分隔符）。

依赖：
- console.get_console() 用于 rich 模式打印。
- ansi._supports_color() 用于降级模式颜色判定。
"""

from __future__ import annotations

import os
import platform
import sys
from typing import Optional

# 安装包版本：与 scripts/installer/__init__.py 的 __version__ 保持一致
try:
    from scripts.installer import __version__ as INSTALLER_VERSION
except ImportError:
    # 极端容错：包未正确初始化时使用硬编码 fallback
    INSTALLER_VERSION = "0.1.0"


# ---------- 横幅 ASCII ----------

# 多行 ASCII 横幅（共 5 行），使用项目缩写 NUWAX
BANNER_LINES = [
    " _   _ _    _          __        __         _     _            ",
    "| \\ | | |  | |         \\ \\      / /__  _ __| | __| | ___ _ __  ",
    "|  \\| | |  | |  _____   \\ \\ /\\ / / _ \\| '__| |/ _` |/ _ \\ '_ \\ ",
    "| |\\  | |__| | |_____|   \\ V  V / (_) | |  | | (_| |  __/ | | |",
    "|_| \\_|\\____/             \\_/\\_/ \\___/|_|  |_|\\__,_|\\___|_| |_|",
]


def _detect_os() -> str:
    """探测当前 OS（Linux 发行版 / macOS / Windows）。"""
    system = platform.system()
    if system == "Linux":
        try:
            # /etc/os-release 是 Linux 标准；macOS/Windows 没有
            with open("/etc/os-release", encoding="utf-8") as f:
                for line in f:
                    if line.startswith("PRETTY_NAME="):
                        return line.split("=", 1)[1].strip().strip('"')
        except (FileNotFoundError, PermissionError):
            pass
        return f"Linux {platform.release()}"
    if system == "Darwin":
        return f"macOS {platform.mac_ver()[0]}"
    if system == "Windows":
        return f"Windows {platform.release()}"
    return system or "Unknown"


def _detect_arch() -> str:
    """探测 CPU 架构（x86_64 / aarch64 / arm64 等）。"""
    return platform.machine() or "unknown"


def _detect_python() -> str:
    """返回 Python 版本字符串。"""
    return platform.python_version()


def _detect_tty() -> str:
    """探测 stdin/stdout 是否 tty。"""
    try:
        stdin_ok = bool(sys.stdin.isatty())
        stdout_ok = bool(sys.stdout.isatty())
    except (AttributeError, ValueError):
        stdin_ok = False
        stdout_ok = False
    return f"stdin={stdin_ok}, stdout={stdout_ok}"


def _detect_dual_mode() -> str:
    """探测双模式门状态。延迟 import 避免循环依赖（ansi.py 已无 rich 依赖）。"""
    from scripts.installer.ui.ansi import is_terminal_ui_available
    return "questionary/rich" if is_terminal_ui_available() else "ANSI 降级"


def _detect_terminal_width(default: int = 80) -> int:
    """探测终端列数（用于降级横幅居中）。失败时用 default。"""
    try:
        return max(40, os.get_terminal_size().columns)
    except (OSError, ValueError):
        return default


# ---------- 打印函数 ----------

def print_banner() -> None:
    """打印启动横幅（含 ASCII + 版本号 + 探测结果）。

    rich 模式：彩色 banner + 表格化探测结果
    降级模式：纯文本 banner + 列表式探测结果
    """
    from scripts.installer.ui import console as _console

    console = _console.get_console()
    if console is not None:
        # rich 模式：rich.console.print 支持 markup；用 cyan 主题
        for line in BANNER_LINES:
            console.print(f"[info]{line}[/info]")
        console.print(f"[bold]Nuwax 一键安装器 v{INSTALLER_VERSION}[/bold]")
        console.print("")
    else:
        # 降级模式：直接 print；颜色由 console 模块兜底
        for line in BANNER_LINES:
            _console.info(line)
        _console.success(f"Nuwax 一键安装器 v{INSTALLER_VERSION}")
        _console.muted("")

    # 探测结果汇总：6 行固定字段
    rows = [
        ("OS", _detect_os()),
        ("架构", _detect_arch()),
        ("Python", _detect_python()),
        ("终端", _detect_tty()),
        ("UI 模式", _detect_dual_mode()),
        ("工作目录", os.getcwd()),
    ]
    _print_probe_table(rows)


def _print_probe_table(rows: list[tuple[str, str]]) -> None:
    """打印 6 行探测结果。

    rich 模式：使用 rich.table.Table 对齐
    降级模式：左对齐 key（宽 10），后续 value
    """
    from scripts.installer.ui import console as _console

    console = _console.get_console()
    if console is not None:
        try:
            from rich.table import Table as _RichTable
            table = _RichTable(
                show_header=False,
                show_edge=False,
                box=None,
                pad_edge=False,
            )
            table.add_column(style="cyan", no_wrap=True)
            table.add_column()
            for k, v in rows:
                table.add_row(k, str(v))
            console.print(table)
        except ImportError:
            # 极小概率：rich.console 在但 rich.table 不在（拆分安装）
            _print_probe_fallback(rows)
    else:
        _print_probe_fallback(rows)


def _print_probe_fallback(rows: list[tuple[str, str]]) -> None:
    """降级模式：左对齐打印探测结果（无 rich 表格）。"""
    from scripts.installer.ui import console as _console
    width = _detect_terminal_width()
    key_w = 10
    for k, v in rows:
        line = f"  {k:<{key_w}} : {v}"
        # 仅截断 v 防止极端长字符串破坏终端（保留至少 20 字符）
        if len(line) > width and len(v) > max(20, width - key_w - 6):
            v_short = v[: max(20, width - key_w - 6)] + "..."
            line = f"  {k:<{key_w}} : {v_short}"
        _console.muted(line)