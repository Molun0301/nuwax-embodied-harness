#!/usr/bin/env python3
"""开发机Go2增量构建、部署与验证入口。"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scripts.go2_update.cli import main

if __name__ == "__main__":
    raise SystemExit(main())
