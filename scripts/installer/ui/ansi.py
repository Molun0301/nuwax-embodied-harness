"""installer.ui.ansi：终端 UI 双模式门的 ANSI 降级实现。

设计目标：
- 提供 `is_terminal_ui_available()`：必须**真正**检测 tty 双端 + python 版本。
- 当 questionary / rich 不可用或非 tty / python<3.10 时，降级到 `input()` + ANSI 转义码。
- 避免在 Go2 实机 python3.8.10 环境下因 questionary 导入失败导致 installer 崩溃
  （memory `realmachine-python-env.md`）。

ANSI 转义码降级说明：
- 仅在 `sys.stdout.isatty()` 为 True 时输出颜色码，否则强制纯文本（防止日志/管道污染）。
- 不依赖外部库（colorama 等）以减少首装成本。
"""

from __future__ import annotations

import platform
import sys
from typing import Optional


# ---------- ANSI 转义码常量（标准 SGR 子集） ----------

# 复位
ANSI_RESET = "\033[0m"
# 基础前景色
ANSI_GRAY = "\033[90m"
ANSI_RED = "\033[31m"
ANSI_GREEN = "\033[32m"
ANSI_YELLOW = "\033[33m"
ANSI_BLUE = "\033[34m"
ANSI_CYAN = "\033[36m"
# 高亮（粗体）+ 前景色
ANSI_BOLD_GREEN = "\033[1;32m"
ANSI_BOLD_YELLOW = "\033[1;33m"
ANSI_BOLD_RED = "\033[1;31m"
ANSI_BOLD_CYAN = "\033[1;36m"
# 反显（用于高亮默认值）
ANSI_REVERSE = "\033[7m"


# ---------- 双模式门 ----------

# 实机 python 最低支持 3.8（含 3.8.10，memory `realmachine-python-env.md`）；
# 双模式门阈值 3.10（questionary/rich typing PEP 604 依赖）。
_REQUIRED_PY = (3, 10)


def _parse_py_version() -> tuple[int, int]:
    """解析当前 Python 解释器版本号为 (major, minor) 元组。

    使用 platform.python_version() 而非 sys.version_info 是为了与 spec.md
    Scenario: 非 tty 或 python<3.10 降级 的描述字面对齐。
    """
    ver_str = platform.python_version()
    parts = ver_str.split(".")
    try:
        major = int(parts[0])
        minor = int(parts[1]) if len(parts) >= 2 else 0
    except (ValueError, IndexError):
        # 解析失败时保守返回 (0, 0)，强制走降级路径，避免误判高版本
        return (0, 0)
    return (major, minor)


def is_terminal_ui_available() -> bool:
    """判定 questionary/rich 高级 UI 是否可用。

    判定规则（与 spec.md `Requirement: CLI 交互双模式` 完全对齐）：
        sys.stdin.isatty() and sys.stdout.isatty() and python_version >= (3, 10)

    返回：
        True  -> questionary/rich 完整模式可用；prompts.py 走高级分支
        False -> 降级到 input() + ANSI；保证 Go2 实机 python3.8.10 不崩

    注意：本函数不实际 import questionary/rich。导入失败的二次降级在
    prompts.py 内部处理（try/except ImportError）。
    """
    # 双端 tty 必须同时满足（避免 `< /dev/null` 或 `| tee` 场景误判）
    try:
        stdin_tty = bool(sys.stdin.isatty())
        stdout_tty = bool(sys.stdout.isatty())
    except (AttributeError, ValueError):
        # 某些嵌入式环境 sys.stdin 是伪文件对象，无 isatty
        stdin_tty = False
        stdout_tty = False

    py_tuple = _parse_py_version()
    py_ok = py_tuple >= _REQUIRED_PY

    return stdin_tty and stdout_tty and py_ok


# ---------- 颜色输出辅助 ----------

def _supports_color() -> bool:
    """内部判定：当前 stdout 是否能接收 ANSI 转义码。

    规则：仅在 stdout 是 tty 时输出颜色；管道/重定向/日志文件强制纯文本。
    """
    try:
        return bool(sys.stdout.isatty())
    except (AttributeError, ValueError):
        return False


def _wrap(text: str, color: str) -> str:
    """如果 stdout 支持颜色，用 color 包裹 text；否则原样返回。"""
    if not _supports_color():
        return text
    return f"{color}{text}{ANSI_RESET}"


def info(msg: str) -> None:
    """打印 info 级别消息（青色加粗）。"""
    print(_wrap(msg, ANSI_BOLD_CYAN))


def warn(msg: str) -> None:
    """打印 warn 级别消息（黄色加粗）。"""
    print(_wrap(msg, ANSI_BOLD_YELLOW), file=sys.stderr)


def error(msg: str) -> None:
    """打印 error 级别消息（红色加粗，输出到 stderr）。"""
    print(_wrap(msg, ANSI_BOLD_RED), file=sys.stderr)


def success(msg: str) -> None:
    """打印 success 级别消息（绿色加粗）。"""
    print(_wrap(msg, ANSI_BOLD_GREEN))


def dim(msg: str) -> str:
    """返回灰色暗淡文本（用于辅助说明，不直接打印）。"""
    return _wrap(msg, ANSI_GRAY)


# ---------- 基础 prompt 包装（降级路径） ----------

def pip_color_input(prompt: str, default: str = "") -> str:
    """降级版 input：显示 prompt + 高亮默认值，等待用户输入。

    参数：
        prompt: 提示文本（不含冒号，函数自动追加 " [<default>]: "）
        default: 默认值（用户直接回车时返回；空字符串表示无默认）

    返回：用户输入的字符串（去除首尾空白）。Ctrl-C / EOF 抛 KeyboardInterrupt / EOFError。

    行为：
        - stdout 非 tty 时退回原生 input()（无颜色、无反显）。
        - stdout 是 tty 时用 ANSI 反显高亮默认值。
    """
    if not _supports_color():
        # 管道/重定向场景：直接调 input，不要混 ANSI 码
        suffix = f" [{default}]: " if default else ": "
        try:
            return input(prompt + suffix).strip()
        except (KeyboardInterrupt, EOFError):
            raise

    suffix = f" [{default}]: " if default else ": "
    # 反显（ANSI_REVERSE）让默认值在终端上明显高亮；用户输入即覆盖
    rendered_default = f"{ANSI_REVERSE}{default}{ANSI_RESET}" if default else ""
    full_prompt = f"{prompt}{suffix}{rendered_default}"
    try:
        return input(full_prompt).strip()
    except (KeyboardInterrupt, EOFError):
        raise


def confirm(prompt: str, default: bool = False) -> bool:
    """降级版 confirm：y/N 确认。

    参数：
        prompt: 提示文本
        default: 默认答案（True=y, False=N）

    返回：布尔。空输入返回 default。
    """
    default_letter = "Y/n" if default else "y/N"
    full_prompt = f"{prompt} [{default_letter}]"
    while True:
        try:
            raw = pip_color_input(full_prompt, default="").strip().lower()
        except (KeyboardInterrupt, EOFError):
            # Ctrl-C / EOF 视为取消（返回 False）
            return False
        if raw == "":
            return default
        if raw in ("y", "yes", "是"):
            return True
        if raw in ("n", "no", "否"):
            return False
        # 其他输入：循环重问（降级模式无 UI 重绘，仅 stderr 提示）
        warn(f"无法识别输入 '{raw}'，请输入 y 或 n")


def password(prompt: str = "请输入密码") -> str:
    """降级版 password 提示。

    注意：降级路径**无法**隐藏回显（需要 termios tty 模式控制），仅作为占位。
    高敏场景应强制走 questionary 模式或外部环境变量。

    返回：用户输入的字符串。
    """
    warn("降级模式不支持隐藏回显；密码将以明文显示，请注意环境安全")
    return pip_color_input(prompt, default="")


def select_one(prompt: str, choices: list[str], default: Optional[str] = None) -> str:
    """降级版单选：列出选项 + 默认值，等待用户输入序号或名称。

    参数：
        prompt: 提示文本
        choices: 选项字符串列表（至少 1 项）
        default: 默认选项（必须在 choices 中；None 表示无默认）

    返回：用户选中的字符串。输入序号（1-based）或精确名称均接受。
    """
    if not choices:
        raise ValueError("select_one: choices 必须非空")

    # 打印选项列表
    info(f"{prompt}")
    for idx, choice in enumerate(choices, start=1):
        marker = " *" if choice == default else "  "
        line = f"  {marker}{idx}) {choice}"
        if choice == default and _supports_color():
            line = f"{ANSI_REVERSE}  {idx}) {choice}  {ANSI_RESET}"
        print(line)

    default_hint = f" [{choices.index(default) + 1}]" if default in choices else ""
    while True:
        try:
            raw = pip_color_input("请选择（序号或名称）", default=str(
                choices.index(default) + 1) if default in choices else "").strip()
        except (KeyboardInterrupt, EOFError):
            raise
        if raw == "" and default in choices:
            return default
        # 序号匹配
        if raw.isdigit():
            n = int(raw)
            if 1 <= n <= len(choices):
                return choices[n - 1]
        # 名称精确匹配
        if raw in choices:
            return raw
        warn(f"无法识别 '{raw}'，请重新输入")


def select_many(prompt: str, choices: list[str], defaults: Optional[list[str]] = None) -> list[str]:
    """降级版多选：逗号分隔序号或名称。

    参数：
        prompt: 提示文本
        choices: 选项字符串列表（至少 1 项）
        defaults: 默认已选项列表（必须都在 choices 中）

    返回：选中的字符串列表（保持 choices 中的相对顺序）。空输入返回 defaults 拷贝。
    """
    if not choices:
        raise ValueError("select_many: choices 必须非空")

    defaults = defaults or []
    info(f"{prompt}（多选，逗号分隔；空输入使用默认）")
    for idx, choice in enumerate(choices, start=1):
        marker = "[x]" if choice in defaults else "[ ]"
        print(f"  {marker} {idx}) {choice}")

    default_hint = ",".join(
        str(choices.index(c) + 1) for c in defaults if c in choices
    )
    while True:
        try:
            raw = pip_color_input("请选择（逗号分隔）", default=default_hint).strip()
        except (KeyboardInterrupt, EOFError):
            raise
        if raw == "":
            return list(defaults)
        # 解析逗号分隔 token
        tokens = [t.strip() for t in raw.split(",") if t.strip()]
        selected: list[str] = []
        valid = True
        for tok in tokens:
            if tok.isdigit():
                n = int(tok)
                if 1 <= n <= len(choices):
                    if choices[n - 1] not in selected:
                        selected.append(choices[n - 1])
                    continue
            if tok in choices:
                if tok not in selected:
                    selected.append(tok)
                continue
            warn(f"无法识别 token '{tok}'，请重新输入")
            valid = False
            break
        if valid:
            return selected
