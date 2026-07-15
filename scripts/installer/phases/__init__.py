#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""installer phases 包入口 + --phase-override 解析。

设计要点（plan 4.1 L107 / 4.4 L180-194 / spec "断点续跑"）：

- ``parse_phase_overrides()`` 把 ``['models=exit-1', 'nuwaclaw=skip']`` 解析为
  ``{'models': -1, 'nuwaclaw': 'skip'}``；解析失败抛 ``InstallerError(64)``。
- ``DEFAULT_PHASE_ORDER`` 是 10 个 phase 的标准顺序；
  ``geometry_runtime_deps`` 承载 nuwax-geometry-runtime 产物校验；后续
  phase 注册、``--resume`` 跳过已成功阶段、wizard 进度展示都依赖这个顺序。
- ``PhaseRegistry`` 是 phase 类的注册表，phases/*.py 在 import 时 ``register()``。
"""

from __future__ import annotations

import re
from typing import Any, Dict, Iterable, List, Type

from scripts.installer.errors import InstallerError, InstallerErrorCode


# --phase-override 格式：phase_name=exit[-]<int> | =skip
_PHASE_OVERRIDE_RE = re.compile(r"^(?P<phase>[a-z_]+)=(?P<code>exit-?\d+|skip)$")

# phase 顺序（plan 4.1 L110-120）：
# 每个 phase 文件名（不含 .py）。
DEFAULT_PHASE_ORDER: tuple[str, ...] = (
    "preflight",
    "system",
    "python_env",
    "geometry_runtime_deps",
    "config_env",
    "models",
    "systemd",
    "mcp",
    "nuwaclaw",
    "health",
)


def parse_phase_overrides(raw: Iterable[str]) -> Dict[str, Any]:
    """解析 ``--phase-override <phase>=<exit-code>`` 列表。

    接受以下格式（plan 4.4 L180-194）：
    - ``<phase_name>=exit-<n>``  →  ``{<phase>: -<n>}``（int）
    - ``<phase_name>=exit<n>``   →  ``{<phase>: <n>}``（int）
    - ``<phase_name>=skip``      →  ``{<phase>: "skip"}``（跳过该 phase）

    Args:
        raw: 来自 argparse ``action="append"`` 的列表（每项是 ``<phase>=<code>``）。

    Returns:
        ``{phase_name: int | "skip"}`` 字典。

    Raises:
        InstallerError(code=64, EX_USAGE): 格式错误或 phase 不在 ``DEFAULT_PHASE_ORDER``。
    """
    result: Dict[str, Any] = {}
    for entry in raw:
        if not isinstance(entry, str) or not entry.strip():
            raise InstallerError(
                message=f"--phase-override 条目不能为空: {entry!r}",
                code=InstallerErrorCode.UNKNOWN,
                remediation="格式: --phase-override <phase>=exit-<code> 或 =skip；示例: --phase-override models=exit-1",
            )
        match = _PHASE_OVERRIDE_RE.match(entry.strip())
        if not match:
            raise InstallerError(
                message=(
                    f"--phase-override 格式错误: {entry!r}。"
                    "格式必须是 <phase>=exit-<int>|exit<int>|skip"
                ),
                code=InstallerErrorCode.UNKNOWN,
                remediation=(
                    "正确格式示例: --phase-override models=exit-1 "
                    "--phase-override nuwaclaw=skip；phase 名称仅支持小写字母+下划线。"
                ),
            )
        phase = match.group("phase")
        code = match.group("code")
        if phase not in DEFAULT_PHASE_ORDER:
            allowed = ", ".join(DEFAULT_PHASE_ORDER)
            raise InstallerError(
                message=f"--phase-override phase 不合法: {phase!r}",
                code=InstallerErrorCode.UNKNOWN,
                remediation=f"允许的 phase: {allowed}",
            )
        if code == "skip":
            result[phase] = "skip"
        else:
            # "exit-1" / "exit100" / "exit0"
            number_part = code[4:]  # strip "exit"
            try:
                result[phase] = int(number_part)
            except ValueError as exc:  # pragma: no cover - regex 已保证是数字
                raise InstallerError(
                    message=f"--phase-override 退出码解析失败: {entry!r}",
                    code=InstallerErrorCode.UNKNOWN,
                    remediation="exit-<int> 中 <int> 必须是十进制整数。",
                ) from exc
    return result


class PhaseRegistry:
    """phase 类的注册表（runtime_data 之外的纯内存结构）。"""

    def __init__(self) -> None:
        self._registry: Dict[str, Type] = {}

    def register(self, name: str, cls: Type) -> None:
        if name in self._registry:
            raise ValueError(f"phase {name!r} 重复注册")
        self._registry[name] = cls

    def get(self, name: str) -> Type:
        if name not in self._registry:
            raise KeyError(f"phase {name!r} 未注册")
        return self._registry[name]

    def all(self) -> List[Type]:
        return [self._registry[name] for name in DEFAULT_PHASE_ORDER if name in self._registry]


# 全局单例；phases/*.py 在 import 时调用 ``default_registry.register(...)``
default_registry = PhaseRegistry()


__all__ = [
    "DEFAULT_PHASE_ORDER",
    "parse_phase_overrides",
    "PhaseRegistry",
    "default_registry",
]
