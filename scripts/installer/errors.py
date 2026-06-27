#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""installer 错误类型与错误码定义。

设计要点：
- ``InstallerError``：通用顶层错误，承载 ``code`` + ``remediation`` 文本。
- ``PhaseError``：阶段执行失败专用错误，额外带 ``phase`` 名称 + ``file_path:line_number``。
- ``InstallerErrorCode``：IntEnum，统一对外退出码与分类。

退出码契约（评审要点 4）：
- 0 = 成功
- 1 = Phase 失败
- 2 = env 已存在拒绝覆盖
- 3 = NuwaxClaw 风险拒绝
- 4 = lock 占用
- 5 = 离线模式拒绝网络
- 6 = profile schema 不兼容
- 64 = 参数解析 / ``--phase-override`` 格式错误（POSIX EX_USAGE）
- 70 = 内部错误（bootstrap.py 不可达等）
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
from typing import Optional


class InstallerErrorCode(IntEnum):
    """installer 退出码枚举。"""

    UNKNOWN = 0
    PHASE_FAILED = 1
    ENV_EXISTS_REFUSE_OVERWRITE = 2
    NUWACLAW_RISK_REFUSED = 3
    LOCK_HELD = 4
    OFFLINE_REJECT_NETWORK = 5
    PROFILE_SCHEMA_INCOMPATIBLE = 6
    INTERNAL = 70

    # 64 留给 POSIX EX_USAGE（argparse / phase-override 解析失败）


@dataclass
class InstallerError(RuntimeError):
    """通用 installer 错误。

    ``code`` 取自 :class:`InstallerErrorCode`；``remediation`` 是中文修复建议。
    """

    message: str
    code: InstallerErrorCode = InstallerErrorCode.UNKNOWN
    remediation: str = ""
    file_path: Optional[str] = None
    line_number: Optional[int] = None

    def __str__(self) -> str:  # pragma: no cover - 走 __post_str__
        return self.__post_str__()

    def __post_str__(self) -> str:
        parts: list[str] = [f"[installer 错误] {self.message}"]
        if self.file_path is not None:
            line = f":{self.line_number}" if self.line_number is not None else ""
            parts.append(f" 位置: {self.file_path}{line}")
        if self.remediation:
            parts.append(f" 建议: {self.remediation}")
        return "\n".join(parts)

    def to_human(self) -> str:
        """人类可读的多行中文错误输出。"""
        return self.__post_str__()


@dataclass
class PhaseError(InstallerError):
    """阶段执行失败错误。

    额外携带 ``phase`` 名称，用于按 plan/openspec 约定的输出格式
    ``[phase: <name>] 失败: <message> — <file>:<line> <ctx>。建议: <remediation>``。
    """

    phase: str = ""
    context: str = field(default="")

    def __post_str__(self) -> str:
        head = f"[phase: {self.phase}] 失败: {self.message}" if self.phase else f"失败: {self.message}"
        if self.context:
            head += f" — {self.context}"
        if self.file_path is not None:
            line = f":{self.line_number}" if self.line_number is not None else ""
            head += f" ({self.file_path}{line})"
        out = [head]
        if self.remediation:
            out.append(f"建议: {self.remediation}")
        return "\n".join(out)

    def to_human(self) -> str:
        return self.__post_str__()


def format_phase_error(err: PhaseError) -> str:
    """按 spec "错误定位输出契约" 格式化 phase 错误。"""
    return err.to_human()