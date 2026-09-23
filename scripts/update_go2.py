#!/usr/bin/env python3
"""在开发机执行 Go2 增量更新；帮助见 --help。"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scripts.go2_update.cli import main

if __name__ == "__main__":
    raise SystemExit(main())
