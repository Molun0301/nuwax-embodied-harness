import sys
from pathlib import Path

# 保证从任意 cwd 运行都能 import 顶层包（contracts/services/...）
ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
