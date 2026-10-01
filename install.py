#!/usr/bin/env python3
"""统一安装入口：默认模块管理，兼容现有生产部署命令。"""

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
    """默认进入模块向导，原有明确部署命令继续使用事务安装器。"""
    _bootstrap_path()
    if len(sys.argv) > 1 and sys.argv[1] == "update":
        from scripts.update_client import main as update_main
        return int(update_main(sys.argv[2:]))
    legacy_commands = {"plan", "check", "deploy", "resume", "verify", "status", "rollback", "bundle"}
    if len(sys.argv) > 1 and sys.argv[1] in legacy_commands:
        from scripts.deploy_installer.cli import main as installer_main
    else:
        from scripts.module_installer.cli import main as installer_main

    return int(installer_main())


if __name__ == "__main__":
    raise SystemExit(main())
