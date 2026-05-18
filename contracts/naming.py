from __future__ import annotations

from datetime import datetime, timezone
import re
import secrets
from typing import Optional

FRAME_ID_PATTERN = re.compile(r"^[A-Za-z][A-Za-z0-9_]*(/[A-Za-z][A-Za-z0-9_]*)*$")
TOPIC_NAME_PATTERN = re.compile(
    r"^(?:[a-z][a-z0-9_]*(?:\.[a-z][a-z0-9_]*)*|/[a-z][a-z0-9_]*(?:/[a-z][a-z0-9_]*)*)$"
)
CAPABILITY_NAME_PATTERN = re.compile(r"^[a-z][a-z0-9_]*$")
TASK_ID_PATTERN = re.compile(r"^task_[a-z0-9][a-z0-9_:-]{2,127}$")
EVENT_TYPE_PATTERN = re.compile(r"^[a-z][a-z0-9_]*(\.[a-z][a-z0-9_]*)+$")
ARTIFACT_ID_PATTERN = re.compile(r"^art_[a-z0-9_]+_[0-9]{8}T[0-9]{6}Z_[a-f0-9]{8}$")
OBSERVATION_ID_PATTERN = re.compile(r"^obs_[a-z0-9_]+_[0-9]{8}T[0-9]{6}Z_[a-f0-9]{8}$")
MEMORY_ID_PATTERN = re.compile(r"^mem_[a-z0-9_]+_[0-9]{8}T[0-9]{6}Z_[a-f0-9]{8}$")
LOCATION_ID_PATTERN = re.compile(r"^loc_[a-z0-9_]+_[0-9]{8}T[0-9]{6}Z_[a-f0-9]{8}$")
ANCHOR_ID_PATTERN = re.compile(r"^anc_[a-z0-9_]+_[0-9]{8}T[0-9]{6}Z_[a-f0-9]{8}$")
INSTANCE_ID_PATTERN = re.compile(r"^ins_[a-z0-9_]+_[0-9]{8}T[0-9]{6}Z_[a-f0-9]{8}$")
OBSERVATION_EVENT_ID_PATTERN = re.compile(r"^oev_[a-z0-9_]+_[0-9]{8}T[0-9]{6}Z_[a-f0-9]{8}$")
NAVIGATION_CANDIDATE_ID_PATTERN = re.compile(r"^navc_[a-z0-9_]+_[0-9]{8}T[0-9]{6}Z_[a-f0-9]{8}$")
SCENE_SPACE_ID_PATTERN = re.compile(r"^ssp_[a-z0-9_]+_[0-9]{8}T[0-9]{6}Z_[a-f0-9]{8}$")
NAVIGATION_TRAJECTORY_EVENT_ID_PATTERN = re.compile(r"^nte_[a-z0-9_]+_[a-f0-9]{12}$")


def _ensure_match(value: str, pattern: re.Pattern[str], field_name: str) -> str:
    value = value.strip()
    if not value:
        raise ValueError(f"{field_name} 不能为空。")
    if not pattern.fullmatch(value):
        raise ValueError(f"{field_name} 不符合命名规范：{value}")
    return value


def validate_frame_id(value: str) -> str:
    """校验坐标系名称。"""

    return _ensure_match(value, FRAME_ID_PATTERN, "frame_id")


def validate_topic_name(value: str) -> str:
    """校验主题名称。"""

    return _ensure_match(value, TOPIC_NAME_PATTERN, "topic_name")


def validate_capability_name(value: str) -> str:
    """校验能力名称。"""

    return _ensure_match(value, CAPABILITY_NAME_PATTERN, "capability_name")


def validate_task_id(value: str) -> str:
    """校验任务标识。"""

    return _ensure_match(value, TASK_ID_PATTERN, "task_id")


def validate_event_type(value: str) -> str:
    """校验事件类型。"""

    return _ensure_match(value, EVENT_TYPE_PATTERN, "event_type")


def validate_artifact_id(value: str) -> str:
    """校验制品标识。"""

    return _ensure_match(value, ARTIFACT_ID_PATTERN, "artifact_id")


def validate_observation_id(value: str) -> str:
    """校验观察标识。"""

    return _ensure_match(value, OBSERVATION_ID_PATTERN, "observation_id")


def validate_memory_id(value: str) -> str:
    """校验记忆标识。"""

    return _ensure_match(value, MEMORY_ID_PATTERN, "memory_id")


def validate_location_id(value: str) -> str:
    """校验地点标识。"""

    return _ensure_match(value, LOCATION_ID_PATTERN, "location_id")


def validate_anchor_id(value: str) -> str:
    """校验空间锚点标识。"""

    return _ensure_match(value, ANCHOR_ID_PATTERN, "anchor_id")


def validate_instance_id(value: str) -> str:
    """校验语义实例标识。"""

    return _ensure_match(value, INSTANCE_ID_PATTERN, "instance_id")


def validate_observation_event_id(value: str) -> str:
    """校验观察事件标识。"""

    return _ensure_match(value, OBSERVATION_EVENT_ID_PATTERN, "observation_event_id")


def validate_navigation_candidate_id(value: str) -> str:
    """校验导航候选标识。"""

    return _ensure_match(value, NAVIGATION_CANDIDATE_ID_PATTERN, "navigation_candidate_id")


def validate_scene_space_id(value: str) -> str:
    """校验场景空间标识。"""

    return _ensure_match(value, SCENE_SPACE_ID_PATTERN, "scene_space_id")


def validate_navigation_trajectory_event_id(value: str) -> str:
    """校验导航轨迹事件标识。"""

    return _ensure_match(value, NAVIGATION_TRAJECTORY_EVENT_ID_PATTERN, "navigation_trajectory_event_id")


def build_task_id(capability_name: str, timestamp: Optional[datetime] = None, suffix: Optional[str] = None) -> str:
    """按平台约定生成任务标识。"""

    capability_name = validate_capability_name(capability_name)
    current_time = (timestamp or datetime.now(timezone.utc)).astimezone(timezone.utc)
    if suffix is None:
        suffix = secrets.token_hex(4)
    suffix = suffix.lower()
    task_id = f"task_{capability_name}_{current_time.strftime('%Y%m%dt%H%M%Sz').lower()}_{suffix}"
    return validate_task_id(task_id)


def build_event_id(event_type: str, timestamp: Optional[datetime] = None, suffix: Optional[str] = None) -> str:
    """按平台约定生成事件标识。"""

    event_type = validate_event_type(event_type)
    current_time = (timestamp or datetime.now(timezone.utc)).astimezone(timezone.utc)
    if suffix is None:
        suffix = secrets.token_hex(4)
    suffix = suffix.lower()
    safe_event_type = event_type.replace(".", "_")
    return f"evt_{safe_event_type}_{current_time.strftime('%Y%m%dT%H%M%SZ')}_{suffix}"


def build_artifact_id(kind: str, timestamp: Optional[datetime] = None, suffix: Optional[str] = None) -> str:
    """按平台约定生成制品标识。"""

    kind = validate_capability_name(kind)
    current_time = (timestamp or datetime.now(timezone.utc)).astimezone(timezone.utc)
    if suffix is None:
        suffix = secrets.token_hex(4)
    suffix = suffix.lower()
    return f"art_{kind}_{current_time.strftime('%Y%m%dT%H%M%SZ')}_{suffix}"


def build_observation_id(source: str, timestamp: Optional[datetime] = None, suffix: Optional[str] = None) -> str:
    """按平台约定生成观察标识。"""

    source = validate_capability_name(source)
    current_time = (timestamp or datetime.now(timezone.utc)).astimezone(timezone.utc)
    if suffix is None:
        suffix = secrets.token_hex(4)
    suffix = suffix.lower()
    return f"obs_{source}_{current_time.strftime('%Y%m%dT%H%M%SZ')}_{suffix}"


def build_memory_id(kind: str, timestamp: Optional[datetime] = None, suffix: Optional[str] = None) -> str:
    """按平台约定生成记忆标识。"""

    kind = validate_capability_name(kind)
    current_time = (timestamp or datetime.now(timezone.utc)).astimezone(timezone.utc)
    if suffix is None:
        suffix = secrets.token_hex(4)
    suffix = suffix.lower()
    return validate_memory_id(f"mem_{kind}_{current_time.strftime('%Y%m%dT%H%M%SZ')}_{suffix}")


def build_location_id(source: str, timestamp: Optional[datetime] = None, suffix: Optional[str] = None) -> str:
    """按平台约定生成地点标识。"""

    source = validate_capability_name(source)
    current_time = (timestamp or datetime.now(timezone.utc)).astimezone(timezone.utc)
    if suffix is None:
        suffix = secrets.token_hex(4)
    suffix = suffix.lower()
    return validate_location_id(f"loc_{source}_{current_time.strftime('%Y%m%dT%H%M%SZ')}_{suffix}")


def build_anchor_id(kind: str, timestamp: Optional[datetime] = None, suffix: Optional[str] = None) -> str:
    """按平台约定生成空间锚点标识。"""

    kind = validate_capability_name(kind)
    current_time = (timestamp or datetime.now(timezone.utc)).astimezone(timezone.utc)
    if suffix is None:
        suffix = secrets.token_hex(4)
    suffix = suffix.lower()
    return validate_anchor_id(f"anc_{kind}_{current_time.strftime('%Y%m%dT%H%M%SZ')}_{suffix}")


def build_instance_id(kind: str, timestamp: Optional[datetime] = None, suffix: Optional[str] = None) -> str:
    """按平台约定生成语义实例标识。"""

    kind = validate_capability_name(kind)
    current_time = (timestamp or datetime.now(timezone.utc)).astimezone(timezone.utc)
    if suffix is None:
        suffix = secrets.token_hex(4)
    suffix = suffix.lower()
    return validate_instance_id(f"ins_{kind}_{current_time.strftime('%Y%m%dT%H%M%SZ')}_{suffix}")


def build_observation_event_id(
    source: str,
    timestamp: Optional[datetime] = None,
    suffix: Optional[str] = None,
) -> str:
    """按平台约定生成观察事件标识。"""

    source = validate_capability_name(source)
    current_time = (timestamp or datetime.now(timezone.utc)).astimezone(timezone.utc)
    if suffix is None:
        suffix = secrets.token_hex(4)
    suffix = suffix.lower()
    return validate_observation_event_id(
        f"oev_{source}_{current_time.strftime('%Y%m%dT%H%M%SZ')}_{suffix}"
    )


def build_navigation_candidate_id(
    source: str,
    timestamp: Optional[datetime] = None,
    suffix: Optional[str] = None,
) -> str:
    """按平台约定生成导航候选标识。"""

    source = validate_capability_name(source)
    current_time = (timestamp or datetime.now(timezone.utc)).astimezone(timezone.utc)
    if suffix is None:
        suffix = secrets.token_hex(4)
    suffix = suffix.lower()
    return validate_navigation_candidate_id(
        f"navc_{source}_{current_time.strftime('%Y%m%dT%H%M%SZ')}_{suffix}"
    )


def build_scene_space_id(
    source: str,
    timestamp: Optional[datetime] = None,
    suffix: Optional[str] = None,
) -> str:
    """按平台约定生成场景空间标识。"""

    source = validate_capability_name(source)
    current_time = (timestamp or datetime.now(timezone.utc)).astimezone(timezone.utc)
    if suffix is None:
        suffix = secrets.token_hex(4)
    suffix = suffix.lower()
    return validate_scene_space_id(
        f"ssp_{source}_{current_time.strftime('%Y%m%dT%H%M%SZ')}_{suffix}"
    )
