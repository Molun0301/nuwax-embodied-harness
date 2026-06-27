from __future__ import annotations

from contracts.naming import validate_frame_id
from typing import Tuple


_GLOBAL_FRAME_ROLES = {
    "map": {"map"},
    "odom": {"odom", "odom_combined"},
    "world": {"world"},
}

_BASE_FRAME_ALIASES = {
    "base",
    "base_link",
    "base_footprint",
    "body",
    "go2_base_link",
}

_BASE_FRAME_SUFFIXES = (
    "_base_link",
    "_base_footprint",
)


def _is_base_leaf(leaf: str) -> bool:
    if leaf in _BASE_FRAME_ALIASES:
        return True
    return any(leaf.endswith(suffix) for suffix in _BASE_FRAME_SUFFIXES)


def _normalized_tokens(frame_id: str) -> Tuple[str, ...]:
    return tuple(validate_frame_id(frame_id).split("/"))


def frame_leaf(frame_id: str) -> str:
    """返回坐标系名称最后一段。"""

    return _normalized_tokens(frame_id)[-1]


def infer_frame_role(frame_id: str) -> str:
    """按项目主线约定推断坐标系语义角色。"""

    tokens = _normalized_tokens(frame_id)
    leaf = tokens[-1]
    for role, aliases in _GLOBAL_FRAME_ROLES.items():
        if leaf in aliases:
            return role
    if _is_base_leaf(leaf):
        return "base"
    return "other"


def frame_ids_semantically_equal(left: str, right: str) -> bool:
    """判断两个坐标系名称在主线语义上是否等价。"""

    normalized_left = validate_frame_id(left)
    normalized_right = validate_frame_id(right)
    if normalized_left == normalized_right:
        return True

    left_role = infer_frame_role(normalized_left)
    right_role = infer_frame_role(normalized_right)
    if left_role != right_role:
        return False

    left_tokens = _normalized_tokens(normalized_left)
    right_tokens = _normalized_tokens(normalized_right)
    if len(left_tokens) > 1 and len(right_tokens) > 1 and left_tokens[:-1] != right_tokens[:-1]:
        return False

    if left_role in {"map", "odom", "world"}:
        return True

    if left_role == "base":
        return _is_base_leaf(frame_leaf(normalized_left)) and _is_base_leaf(frame_leaf(normalized_right))

    return False


def frame_ids_conflict(left: str, right: str) -> bool:
    """判断两个坐标系是否存在明确冲突。"""

    return not frame_ids_semantically_equal(left, right)


__all__ = [
    "frame_leaf",
    "frame_ids_conflict",
    "frame_ids_semantically_equal",
    "infer_frame_role",
]
