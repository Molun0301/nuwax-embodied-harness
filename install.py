#!/usr/bin/env python3
"""统一安装入口：本机模块、设备管理与现行软件更新。"""

from __future__ import annotations

import sys


def _bootstrap_path() -> None:
    """让源码检出目录中的保留安装模块可直接导入。"""
    from pathlib import Path

    project_root = Path(__file__).resolve().parent
    project_root_str = str(project_root)
    if project_root_str not in sys.path:
        sys.path.insert(0, project_root_str)


def main() -> int:
    """默认进入模块向导，显式子命令进入设备管理或软件更新。"""
    _bootstrap_path()
    if len(sys.argv) > 1 and sys.argv[1] == "update":
        from scripts.update_client import main as update_main
        return int(update_main(sys.argv[2:]))
    if len(sys.argv) > 1 and sys.argv[1] == "product":
        from scripts.neh_bootstrap import main as product_main
        return int(product_main(sys.argv[2:]))
    if len(sys.argv) > 1 and sys.argv[1] == "manage-device":
        from scripts.device_manager.cli import main as device_main
        return int(device_main(sys.argv[2:]))
    from scripts.module_installer.cli import main as installer_main

    return int(installer_main())


if __name__ == "__main__":
    raise SystemExit(main())
