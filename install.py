#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""installer 顶层 Python 入口（与 install.sh 等价）。

设计要点：
- 仅做 sys.argv 透传到 scripts.installer.__main__:main()，不实现任何业务逻辑。
- 退出码由 installer 包内部统一约定，详见 scripts/installer/errors.py。
- 必须可在工作区根直接执行：``python3 install.py --help``。
- 与 scripts/bootstrap.py 的三子命令（check / wizard / apply）并存，
  本入口不替换它们，只新增单入口闭环。
"""

from __future__ import annotations

import sys


def _bootstrap_path() -> None:
    """确保工作区根在 sys.path，便于 scripts.installer 顶层导入。

    installer 包位于 scripts/installer/，其 __main__ 在被 import 时需要
    解析 drivers.registry 等同级包；将项目根加入 sys.path 是最小侵入做法。
    """
    from pathlib import Path

    project_root = Path(__file__).resolve().parent
    project_root_str = str(project_root)
    if project_root_str not in sys.path:
        sys.path.insert(0, project_root_str)


def main() -> int:
    """透传调用并传播退出码。"""
    _bootstrap_path()
    from scripts.installer.__main__ import main as installer_main

    return int(installer_main())


if __name__ == "__main__":
    raise SystemExit(main())