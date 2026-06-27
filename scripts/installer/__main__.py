#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""``python3 -m scripts.installer`` 入口。

仅负责：
1. 构造 cli 解析器。
2. 调用 ``scripts.installer.cli.main`` 跑实际子命令。
3. 把退出码透传给操作系统。

本身不写任何文件、不发网络请求；副作用全部在 phases/ 内完成。
"""

from __future__ import annotations

import sys
from typing import Sequence


def main(argv: Sequence[str] | None = None) -> int:
    """入口；``argv`` 为 None 时用 ``sys.argv[1:]``。"""
    from scripts.installer import cli

    if argv is None:
        argv = sys.argv[1:]
    return int(cli.main(list(argv)))


if __name__ == "__main__":
    raise SystemExit(main())