"""NEH端侧管理器：独立于业务网关的标准库管理入口。"""
import json
from pathlib import Path

__version__ = "1.0.0"
# 导入时固定物理目录和版本，current切换后旧进程仍报告实际已加载代码。
PACKAGE_ROOT = Path(__file__).resolve().parents[2]
try:
    MANAGER_DEVELOPMENT = False
    MANAGER_VERSION = json.loads((PACKAGE_ROOT / "manager-manifest.json").read_text())["version"]
except (OSError, ValueError, KeyError):
    MANAGER_VERSION = "development"
    MANAGER_DEVELOPMENT = True
