"""PCD 资产点数门槛配置。"""
from __future__ import annotations

from core.config_values import runtime_value

import os


def default_min_points() -> int:
    """读取 NUWAX_PCD_MIN_POINTS（默认 500），用于 PCD 点数门槛判定。"""

    try:
        return max(1, int(runtime_value("NUWAX_PCD_MIN_POINTS", "500")))
    except ValueError:
        return 500
