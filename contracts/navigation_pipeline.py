"""建图、探索与导航的两条固定生产链契约。"""

from typing import Optional


def resolve_navigation_profile(pipeline: Optional[str], configured_profile: Optional[str] = None) -> str:
    """从执行链确定导航；显式旧配置仅能确认同一配对。"""
    selected = (pipeline or "").strip() or "lio_3d"
    if pipeline and pipeline != pipeline.strip():
        raise ValueError("NUWAX_GEOMETRY_PIPELINE不能包含首尾空白")
    profiles = {"lio_3d": "mls-3d", "slam_2d": "simple-2d"}
    if selected not in profiles:
        raise ValueError("NUWAX_GEOMETRY_PIPELINE必须为lio_3d或slam_2d")
    expected = profiles[selected]
    configured = (configured_profile or "").strip()
    if configured and configured != expected:
        raise ValueError(
            f"执行链{selected}固定使用{expected}导航，"
            f"NUWAX_NAV_PROFILE={configured}与执行链冲突；请移除该覆盖或改为{expected}"
        )
    return expected
