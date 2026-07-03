#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""薄包装层：把 scripts/bootstrap.py 的 9 个纯函数包成 installer 内部 API。

设计要点（plan 4.3 / 评审要点 3）：
- 禁止修改 scripts/bootstrap.py 任何代码（保持向后兼容 100%）。
- 禁止构造 argparse.Namespace 透传给 bootstrap.py（评审要点 3 强制约束）；
  本桥接层只 import 纯函数，再传入 ``BootstrapPlanner`` 实例。
- ``BootstrapError`` 一律转 ``PhaseError``，错误码 1（PHASE_FAILED）。
- 9 个被包装函数签名（scripts/bootstrap.py 实测行号）：
  * L983  ``write_root_env(planner) -> Path``
  * L990  ``write_robot_env(planner) -> Optional[Path]``
  * L1000 ``install_python_dependencies(planner) -> None``
  * L1020 ``install_geometry_system_dependencies(planner) -> None``
  * L1129 ``build_workspaces(planner) -> None``
  * L1151 ``prepare_assets(planner) -> None``
  * L1255 ``install_nuwaclaw_client(planner) -> None``
  * L1302 ``configure_mcp(planner) -> None``
  * L1336 ``write_systemd_example(planner) -> Path``
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import ModuleType
from typing import Optional

from scripts.installer.errors import InstallerErrorCode, PhaseError


PROJECT_ROOT = Path(__file__).resolve().parents[2]
BOOTSTRAP_PATH = PROJECT_ROOT / "scripts" / "bootstrap.py"


def _load_bootstrap() -> ModuleType:
    """动态加载 scripts/bootstrap.py 模块。

    bootstrap.py 顶层会做 ``sys.path.insert(0, PROJECT_ROOT)`` 并 import
    drivers.registry；这里复用其副作用，不再重复实现。
    """
    spec = importlib.util.spec_from_file_location("scripts.bootstrap_legacy", BOOTSTRAP_PATH)
    if spec is None or spec.loader is None:
        raise PhaseError(
            phase="bootstrap",
            message="无法定位 scripts/bootstrap.py 加载 spec",
            file_path=str(BOOTSTRAP_PATH),
            code=InstallerErrorCode.INTERNAL,
            remediation="确认 scripts/bootstrap.py 存在且可读；不要删除原 bootstrap.py。",
        )
    module = importlib.util.module_from_spec(spec)
    sys.modules.setdefault("scripts.bootstrap_legacy", module)
    spec.loader.exec_module(module)
    return module


# 单次加载即可；bootstrap.py 内部无副作用副作用
_BOOTSTRAP: Optional[ModuleType] = None


def _bootstrap() -> ModuleType:
    global _BOOTSTRAP
    if _BOOTSTRAP is None:
        _BOOTSTRAP = _load_bootstrap()
    return _BOOTSTRAP


def _wrap(phase: str, fn, planner, *, context: str = ""):
    """调用 bootstrap 纯函数并把 ``BootstrapError`` 翻译为 ``PhaseError``。"""
    try:
        return fn(planner)
    except _bootstrap().BootstrapError as exc:  # type: ignore[attr-defined]
        raise PhaseError(
            phase=phase,
            message=str(exc),
            file_path=str(BOOTSTRAP_PATH),
            code=InstallerErrorCode.PHASE_FAILED,
            context=context,
            remediation=(
                "确认 prerequisites（apt 源 / ROS2 安装 / Python 解释器）后重试；"
                "或运行 'python3 install.py check' 复现前置检查。"
            ),
        ) from exc


# --- 9 个公开包装函数 ------------------------------------------------------


def call_write_root_env(planner):
    """包装 bootstrap.write_root_env（L983）。"""
    return _wrap("config_env", _bootstrap().write_root_env, planner, context=".env 根配置生成")


def call_write_robot_env(planner):
    """包装 bootstrap.write_robot_env（L990）。"""
    return _wrap("config_env", _bootstrap().write_robot_env, planner, context="机器人专属 env 生成")


def call_install_python_dependencies(planner):
    """包装 bootstrap.install_python_dependencies（L1000）。"""
    return _wrap("python_env", _bootstrap().install_python_dependencies, planner, context="Python 依赖安装")


def call_install_geometry_system_dependencies(planner):
    """包装 bootstrap.install_geometry_system_dependencies（L1020）— 去 ROS2 化后保留为 no-op。"""
    # 2026-07-03 drop-ros2：ROS2 apt 依赖安装已迁移至 native runtime 系统包，
    # bootstrap.install_geometry_system_dependencies 自身成为 no-op；保留包装以稳定
    # ``phases/system.py`` 调用点。
    return 0


def call_prepare_assets(planner):
    """包装 bootstrap.prepare_assets（L1151）。"""
    return _wrap("models", _bootstrap().prepare_assets, planner, context="YOLO-World / memory 模型准备")


def call_install_nuwaclaw_client(planner):
    """包装 bootstrap.install_nuwaclaw_client（L1255）。"""
    return _wrap("nuwaclaw", _bootstrap().install_nuwaclaw_client, planner, context="NuwaxClaw Docker 客户端")


def call_configure_mcp(planner):
    """包装 bootstrap.configure_mcp（L1302）。"""
    return _wrap("mcp", _bootstrap().configure_mcp, planner, context="MCP 注入配置生成")


def call_write_systemd_example(planner):
    """包装 bootstrap.write_systemd_example（L1336）。"""
    return _wrap("systemd", _bootstrap().write_systemd_example, planner, context="systemd 示例渲染")


__all__ = [
    "BOOTSTRAP_PATH",
    "call_write_root_env",
    "call_write_robot_env",
    "call_install_python_dependencies",
    "call_install_geometry_system_dependencies",
    "call_prepare_assets",
    "call_install_nuwaclaw_client",
    "call_configure_mcp",
    "call_write_systemd_example",
]