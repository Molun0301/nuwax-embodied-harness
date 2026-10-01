#!/usr/bin/env python3
"""兼容旧文件名；所有机器人使用相同的NEH更新客户端。"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scripts.update_client import main

if __name__ == "__main__":
    raise SystemExit(main())
