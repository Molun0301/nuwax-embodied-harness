#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""Transactional deployment entry point used by ``install.sh``."""

from __future__ import annotations

import sys


def _bootstrap_path() -> None:
    """确保工作区根在 sys.path，便于 deploy installer 顶层导入。

    安装器包位于 scripts/deploy_installer/；将项目根加入 sys.path 后，薄入口
    可以在不安装项目 wheel 的 bootstrap 环境中导入该模块。
    """
    from pathlib import Path

    project_root = Path(__file__).resolve().parent
    project_root_str = str(project_root)
    if project_root_str not in sys.path:
        sys.path.insert(0, project_root_str)


def main() -> int:
    """Delegate to the stdlib-only deployment CLI and preserve its exit code."""
    _bootstrap_path()
    from scripts.deploy_installer.cli import main as installer_main

    return int(installer_main())


if __name__ == "__main__":
    raise SystemExit(main())
