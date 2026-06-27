"""installer.prompts：questionary + ANSI 双模式 prompt 封装。

设计目标：
- 提供统一 API：text / password / confirm / select / checkbox。
- **强制**走 `is_terminal_ui_available()` 双模式门：
  - True  -> 走 questionary（方向键 / 多选 / 输入 / 确认 / 默认值高亮）
  - False -> 降级到 ui/ansi.py 的 input + ANSI
- questionary import 失败时**不抛**（try/except ImportError 包裹），
  强制降级到 ANSI，保证 Go2 实机 python3.8.10 首装不崩（memory `realmachine-python-env.md`）。

RK16 / spec.md `Requirement: CLI 交互双模式` 实现位置。
"""

from __future__ import annotations

from typing import Any, Optional, Sequence

# 延迟导入 questionary；失败时强制走 ANSI 分支
try:
    import questionary  # type: ignore[import-untyped]
    from questionary import Choice as _QChoice  # type: ignore[import-untyped]
    _HAS_QUESTIONARY = True
except ImportError:
    _HAS_QUESTIONARY = False
    questionary = None  # type: ignore[assignment]
    _QChoice = None  # type: ignore[assignment]


# ---------- 双模式门 ----------

def _use_advanced() -> bool:
    """判定是否走 questionary 高级 UI。

    双重门：
    1) is_terminal_ui_available() 为 True（tty 双端 + python≥3.10）
    2) questionary 实际 import 成功（避免假阳性）

    两者都满足才走 questionary；否则降级 ANSI。
    """
    if not _HAS_QUESTIONARY:
        return False
    # 延迟 import 避免循环依赖（ansi.py 不依赖 prompts）
    from scripts.installer.ui.ansi import is_terminal_ui_available
    return is_terminal_ui_available()


def _use_ansi() -> bool:
    """显式降级：当前是否在 ANSI 降级路径。"""
    return not _use_advanced()


# ---------- 诊断 ----------

def mode_name() -> str:
    """诊断用：返回当前 prompt 模式名称。"""
    return "questionary" if _use_advanced() else "ansi"


# ---------- 统一 API ----------

def text(
    message: str,
    default: str = "",
    validate: Optional[Any] = None,
) -> str:
    """单行文本输入。

    参数：
        message: 提示文本
        default: 默认值
        validate: 可选校验函数 (str) -> bool；失败时阻塞重问

    返回：用户输入（去除首尾空白）。
    """
    if _use_advanced():
        try:
            result = questionary.text(  # type: ignore[union-attr]
                message,
                default=default,
                validate=validate,
            ).ask()
        except (KeyboardInterrupt, EOFError):
            raise
        # questionary.ask() 在 Ctrl-C 时返回 None；转为 KeyboardInterrupt 上抛
        if result is None:
            raise KeyboardInterrupt("用户取消 (Ctrl-C)")
        return str(result).strip()
    # 降级
    from scripts.installer.ui import ansi as _ansi
    while True:
        try:
            raw = _ansi.pip_color_input(message, default=default)
        except (KeyboardInterrupt, EOFError):
            raise
        if validate is None:
            return raw
        try:
            ok = validate(raw)
        except Exception:  # validate 函数本身出错时按"通过"处理
            ok = True
        if ok:
            return raw
        _ansi.warn("输入校验失败，请重新输入")


def password(message: str = "请输入密码") -> str:
    """密码输入（高级模式隐藏回显，降级模式明文 + 警告）。"""
    if _use_advanced():
        try:
            result = questionary.password(message).ask()  # type: ignore[union-attr]
        except (KeyboardInterrupt, EOFError):
            raise
        if result is None:
            raise KeyboardInterrupt("用户取消 (Ctrl-C)")
        return str(result)
    from scripts.installer.ui import ansi as _ansi
    return _ansi.password(message)


def confirm(message: str, default: bool = False) -> bool:
    """y/N 确认。"""
    if _use_advanced():
        try:
            result = questionary.confirm(message, default=default).ask()  # type: ignore[union-attr]
        except (KeyboardInterrupt, EOFError):
            raise
        if result is None:
            raise KeyboardInterrupt("用户取消 (Ctrl-C)")
        return bool(result)
    from scripts.installer.ui import ansi as _ansi
    return _ansi.confirm(message, default=default)


def select(
    message: str,
    choices: Sequence[Any],
    default: Any = None,
) -> Any:
    """单选（方向键选择）。

    参数：
        message: 提示文本
        choices: 选项列表；元素可以是 str 或 dict {"name": ..., "value": ...}
        default: 默认选项（str 时按 name/value 精确匹配；dict 时按 value 匹配）

    返回：选中的 value（dict 选项返回 dict['value']）。
    """
    # 统一 choices 形式：questionary 接受 Choice 对象；ansi 接受 str
    if _use_advanced():
        # questionary 直接接受 Sequence[Choice | dict | str]；让 questionary 自己处理
        try:
            result = questionary.select(  # type: ignore[union-attr]
                message,
                choices=list(choices),
                default=default,
            ).ask()
        except (KeyboardInterrupt, EOFError):
            raise
        if result is None:
            raise KeyboardInterrupt("用户取消 (Ctrl-C)")
        return result

    # 降级：把 choices 统一为 str 列表（保留顺序）；如果是 dict 取 name
    str_choices: list[str] = []
    value_map: dict[str, Any] = {}
    for c in choices:
        if isinstance(c, dict):
            name = str(c.get("name", c.get("value", "")))
            value_map[name] = c.get("value", name)
            str_choices.append(name)
        else:
            name = str(c)
            value_map[name] = name
            str_choices.append(name)

    default_str = None
    if default is not None:
        if isinstance(default, dict):
            default_str = str(default.get("name", default.get("value")))
        else:
            default_str = str(default) if str(default) in str_choices else None

    from scripts.installer.ui import ansi as _ansi
    picked = _ansi.select_one(message, str_choices, default=default_str)
    return value_map.get(picked, picked)


def checkbox(
    message: str,
    choices: Sequence[Any],
    defaults: Optional[Sequence[Any]] = None,
) -> list[Any]:
    """多选（空格切换 / Enter 确认）。

    参数：
        message: 提示文本
        choices: 选项列表（str 或 dict）
        defaults: 默认已选项列表

    返回：选中的 value 列表（保持 choices 中的相对顺序）。
    """
    defaults = defaults or []
    if _use_advanced():
        try:
            result = questionary.checkbox(  # type: ignore[union-attr]
                message,
                choices=list(choices),
                default=list(defaults),
            ).ask()
        except (KeyboardInterrupt, EOFError):
            raise
        if result is None:
            raise KeyboardInterrupt("用户取消 (Ctrl-C)")
        return list(result)

    # 降级
    str_choices: list[str] = []
    value_map: dict[str, Any] = {}
    for c in choices:
        if isinstance(c, dict):
            name = str(c.get("name", c.get("value", "")))
            value_map[name] = c.get("value", name)
            str_choices.append(name)
        else:
            name = str(c)
            value_map[name] = name
            str_choices.append(name)

    default_strs: list[str] = []
    for d in defaults:
        if isinstance(d, dict):
            d_str = str(d.get("name", d.get("value")))
        else:
            d_str = str(d)
        if d_str in str_choices:
            default_strs.append(d_str)

    from scripts.installer.ui import ansi as _ansi
    picked = _ansi.select_many(message, str_choices, defaults=default_strs)
    return [value_map.get(p, p) for p in picked]


# ---------- 向后兼容别名（task 1 引用了 ask_text / ask_select 等） ----------

def ask_text(message: str, default: str = "") -> str:
    """text() 的兼容别名。"""
    return text(message, default=default)


def ask_confirm(message: str, default: bool = False) -> bool:
    """confirm() 的兼容别名。"""
    return confirm(message, default=default)


def ask_select(message: str, choices: Sequence[Any], default: Any = None) -> Any:
    """select() 的兼容别名。"""
    return select(message, choices, default=default)


def ask_checkbox(message: str, choices: Sequence[Any], defaults: Optional[Sequence[Any]] = None) -> list:
    """checkbox() 的兼容别名。"""
    return checkbox(message, choices, defaults=defaults)


def ask_password(message: str = "请输入密码") -> str:
    """password() 的兼容别名。"""
    return password(message)