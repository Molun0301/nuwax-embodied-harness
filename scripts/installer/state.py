#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""installer 状态文件读写。

设计要点（plan 4.1 / 4.4 L196-206 / spec "断点续跑与状态文件"）：

- ``runtime_data/installer_state.json``：唯一状态落点。
- ``PhaseRecord`` dataclass 字段：``name / order / status / started_at / finished_at / message``
  + plan 6.9 / spec Scenario "state JSON 合法" 隐含的``_schema_version`` 字段。
- 写入走 ``os.replace`` 原子写：先写临时文件 + fsync，再 ``os.replace``。
- 读取若文件不存在或 JSON 损坏，**不**抛错（首次运行场景）；返回空 state。
- 状态文件入 ``.gitignore``，不入仓（memory ``feedback_no_test_commits.md`` 同样适用于 state）。
"""

from __future__ import annotations

import json
import os
import tempfile
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Literal, Optional


# 状态文件路径（相对工作区根）；与 plan 4.2 / spec Scenario "断点续跑" 一致
DEFAULT_STATE_FILE = Path("runtime_data/installer_state.json")

# 状态文件 schema 版本（递增必须显式；plan RK17 schema 守门）
STATE_SCHEMA_VERSION = "1.0.0"

PhaseStatus = Literal["pending", "running", "success", "failed", "skipped"]


@dataclass
class PhaseRecord:
    """单个 phase 的执行记录。"""

    name: str
    order: int
    status: PhaseStatus = "pending"
    started_at: float = 0.0
    finished_at: float = 0.0
    message: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "PhaseRecord":
        return cls(
            name=str(data.get("name", "")),
            order=int(data.get("order", 0)),
            status=data.get("status", "pending"),
            started_at=float(data.get("started_at", 0.0)),
            finished_at=float(data.get("finished_at", 0.0)),
            message=str(data.get("message", "")),
        )


@dataclass
class InstallerState:
    """installer 全量状态（仅 PhaseRecord 列表 + 元数据）。

    注意：这是 plan 4.4 L196-206 草案 + spec "断点续跑与状态文件" 的最小实现。
    字段集合受 ``_schema_version`` 守门：未来加字段必须显式 bump STATE_SCHEMA_VERSION。
    """

    _schema_version: str = STATE_SCHEMA_VERSION
    created_at: float = field(default_factory=time.time)
    updated_at: float = field(default_factory=time.time)
    last_status: str = "pending"
    phases: List[PhaseRecord] = field(default_factory=list)
    # 启动过的 gateway PID 列表（task 4 / installer-shared-lock 写入，health phase 读取）
    started_pids: List[int] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "_schema_version": self._schema_version,
            "created_at": self.created_at,
            "updated_at": self.updated_at,
            "last_status": self.last_status,
            "phases": [p.to_dict() for p in self.phases],
            "started_pids": list(self.started_pids),
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "InstallerState":
        phases_raw = data.get("phases", [])
        phases = [PhaseRecord.from_dict(p) for p in phases_raw if isinstance(p, dict)]
        return cls(
            _schema_version=str(data.get("_schema_version", STATE_SCHEMA_VERSION)),
            created_at=float(data.get("created_at", time.time())),
            updated_at=float(data.get("updated_at", time.time())),
            last_status=str(data.get("last_status", "pending")),
            phases=phases,
            started_pids=[int(p) for p in data.get("started_pids", []) if str(p).isdigit()],
        )


def load_state(path: Path = DEFAULT_STATE_FILE) -> InstallerState:
    """读 state 文件；文件不存在或损坏时返回空 state（首次运行场景）。"""
    if not path.exists():
        return InstallerState()
    try:
        with path.open("r", encoding="utf-8") as fh:
            data = json.load(fh)
    except (json.JSONDecodeError, OSError):
        return InstallerState()
    if not isinstance(data, dict):
        return InstallerState()
    return InstallerState.from_dict(data)


def save_state(state: InstallerState, path: Path = DEFAULT_STATE_FILE) -> None:
    """原子写 state 文件：临时文件 + fsync + os.replace。

    失败时保留旧文件（不抛错给 wizard 中断；只打日志）。
    """
    state.updated_at = time.time()
    payload = json.dumps(state.to_dict(), ensure_ascii=False, indent=2, sort_keys=False)
    path.parent.mkdir(parents=True, exist_ok=True)
    # 同目录临时文件，确保 os.replace 是原子 rename
    fd, tmp_path = tempfile.mkstemp(prefix=".installer_state.", suffix=".tmp", dir=str(path.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            fh.write(payload)
            fh.flush()
            try:
                os.fsync(fh.fileno())
            except OSError:
                # fsync 在某些 fs（tmpfs / overlayfs）上可能失败；不阻断
                pass
        os.replace(tmp_path, path)
    except Exception:
        # 清理临时文件
        try:
            os.unlink(tmp_path)
        except OSError:
            pass
        raise


def upsert_phase(state: InstallerState, record: PhaseRecord) -> None:
    """按 ``name`` 替换或追加一条 PhaseRecord。"""
    for idx, existing in enumerate(state.phases):
        if existing.name == record.name:
            state.phases[idx] = record
            return
    state.phases.append(record)


def find_phase(state: InstallerState, name: str) -> Optional[PhaseRecord]:
    for p in state.phases:
        if p.name == name:
            return p
    return None


__all__ = [
    "DEFAULT_STATE_FILE",
    "STATE_SCHEMA_VERSION",
    "PhaseRecord",
    "InstallerState",
    "load_state",
    "save_state",
    "upsert_phase",
    "find_phase",
]