"""Python 版本兼容层。"""

from enum import Enum

try:
    from enum import StrEnum
except ImportError:  # pragma: no cover - Python 3.11 以下走兼容实现
    class StrEnum(str, Enum):
        """兼容 Python 3.8 的 `StrEnum` 回退实现。"""

