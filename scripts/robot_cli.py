#!/usr/bin/env python3
"""从项目目录运行轻量机器人客户端。"""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from clients.robot_client import main


if __name__ == "__main__":
    sys.exit(main())
