"""建图、探索与导航的两条固定生产链契约。"""

import math
from typing import Optional


def navigation_support_distance(value: object) -> float:
    """校验三维机体至支撑面的实测正距离，不推断机器人尺寸。"""
    try:
        distance = float(str(value).strip())
    except (TypeError, ValueError):
        distance = float("nan")
    # 与原生规划器的单精度正常正数范围一致。
    if not math.isfinite(distance) or not 1.17549435e-38 <= distance <= 3.4028234e38:
        raise ValueError(
            "三维导航缺少有效的机体到支撑面距离。请在平台设置中填写"
            "“机体到支撑面距离（米）”，使用正常站立时机体坐标原点到地面的实测正距离，"
            "保存并重启网关、几何与导航服务。"
        )
    return distance


def resolve_navigation_profile(pipeline: Optional[str], configured_profile: Optional[str] = None) -> str:
    """从执行链确定导航；显式旧配置仅能确认同一配对。"""
    selected = (pipeline or "").strip() or "lio_3d"
    if pipeline and pipeline != pipeline.strip():
        raise ValueError("NUWAX_GEOMETRY_PIPELINE不能包含首尾空白")
    profiles = {"lio_3d": "pct-scan-3d", "slam_2d": "simple-2d"}
    if selected not in profiles:
        raise ValueError("NUWAX_GEOMETRY_PIPELINE必须为lio_3d或slam_2d")
    expected = profiles[selected]
    configured = (configured_profile or "").strip()
    if selected == "lio_3d" and configured == "mls-3d":
        configured = expected  # 旧三维方案随升级迁入当前实现。
    if configured and configured != expected:
        raise ValueError(
            f"执行链{selected}固定使用{expected}导航，"
            f"NUWAX_NAV_PROFILE={configured}与执行链冲突；请移除该覆盖或改为{expected}"
        )
    return expected
