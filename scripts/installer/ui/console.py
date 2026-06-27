"""installer.ui.console：rich.Console 单例 + info/warn/error/success 主题。

设计目标：
- 提供进程级 Console 单例，避免多入口争抢 stdout。
- 提供 info/warn/error/success 四级打印方法，自动适配 rich 是否可用。
- 与 ansi.py 协同：rich 不可用（python<3.10 / ImportError）时降级到 ANSI 输出。

import 容错：
- try/except ImportError 包裹 `from rich.console import Console`，
  导入失败时回退到 ansi.info/warn/error/success（实机 python3.8.10 + rich 未装场景）。
"""

from __future__ import annotations

from typing import Optional

# 延迟导入 rich；导入失败不抛（仅记 _HAS_RICH = False）
try:
    from rich.console import Console as _RichConsole
    from rich.theme import Theme as _RichTheme
    _HAS_RICH = True
except ImportError:  # 实机 python3.8.10 + rich 未装（RK16 / memory `realmachine-python-env.md`）
    _HAS_RICH = False
    _RichConsole = None  # type: ignore[assignment]
    _RichTheme = None  # type: ignore[assignment]


# ---------- 主题定义 ----------

# 与 ANSI 降级路径颜色语义一致：
#   info    -> 青/亮蓝
#   warn    -> 黄
#   error   -> 红
#   success -> 绿
_INSTALLER_THEME_DICT = {
    "info": "cyan",
    "warning": "yellow",
    "error": "bold red",
    "success": "bold green",
    "muted": "grey50",
}


def _build_console() -> Optional[object]:
    """构造 rich Console 实例；不可用时返回 None。"""
    if not _HAS_RICH:
        return None
    theme = _RichTheme(_INSTALLER_THEME_DICT)
    # force_terminal=None：尊重 stdout 是否 tty；管道场景自动去色
    # soft_wrap=True：避免长行被截断
    return _RichConsole(theme=theme, force_terminal=None, soft_wrap=True)


# 进程级单例（首次访问时构造）
_CONSOLE_SINGLETON: Optional[object] = None


def get_console() -> Optional[object]:
    """返回进程级 Console 单例；rich 不可用时返回 None（调用方应判空）。"""
    global _CONSOLE_SINGLETON
    if _CONSOLE_SINGLETON is None and _HAS_RICH:
        _CONSOLE_SINGLETON = _build_console()
    return _CONSOLE_SINGLETON


def has_rich() -> bool:
    """诊断用：返回 rich 是否成功导入。"""
    return _HAS_RICH


# ---------- 四级打印方法（rich 优先，降级到 ansi） ----------

def info(message: str) -> None:
    """打印 info 级别消息。"""
    console = get_console()
    if console is not None:
        console.print(f"[info]{message}[/info]")
    else:
        # 降级到 ansi；避免循环 import（ansi 已无 rich 依赖）
        from scripts.installer.ui import ansi as _ansi
        _ansi.info(message)


def warn(message: str) -> None:
    """打印 warn 级别消息（stderr）。"""
    console = get_console()
    if console is not None:
        console.print(f"[warning]{message}[/warning]", stderr=True)
    else:
        from scripts.installer.ui import ansi as _ansi
        _ansi.warn(message)


def error(message: str) -> None:
    """打印 error 级别消息（stderr）。"""
    console = get_console()
    if console is not None:
        console.print(f"[error]{message}[/error]", stderr=True)
    else:
        from scripts.installer.ui import ansi as _ansi
        _ansi.error(message)


def success(message: str) -> None:
    """打印 success 级别消息。"""
    console = get_console()
    if console is not None:
        console.print(f"[success]{message}[/success]")
    else:
        from scripts.installer.ui import ansi as _ansi
        _ansi.success(message)


def muted(message: str) -> None:
    """打印 muted 级别消息（辅助说明，灰色）。"""
    console = get_console()
    if console is not None:
        console.print(f"[muted]{message}[/muted]")
    else:
        # 降级模式无 muted 概念，直接 print（不破坏输出）
        print(message)


def rule(title: str = "") -> None:
    """打印水平分割线（带可选标题）。rich 不可用时退化为 40 字符 '-'。"""
    console = get_console()
    if console is not None:
        console.rule(title)
    else:
        bar = "-" * 40
        if title:
            print(f"{bar} {title} {bar}")
        else:
            print(bar * 2)
