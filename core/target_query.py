"""找物、跟随与记忆候选共用的查询文本处理。"""
from typing import Any, List, Optional


def normalize_query(value: Optional[str]) -> str:
    normalized = "".join(
        char.lower() if char.isalnum() or "一" <= char <= "鿿" else " "
        for char in str(value or "")
    )
    return " ".join(normalized.split())


def target_label_matches_query(query: str, label: str) -> bool:
    normalized_query = normalize_query(query)
    normalized_label = normalize_query(label)
    return bool(normalized_query and normalized_label and (
        normalized_query in normalized_label or normalized_label in normalized_query
    ))


def coerce_optional_str_list(value: Any) -> List[str]:
    if value is None or isinstance(value, bool):
        return []
    if isinstance(value, str):
        normalized = value.strip()
        return [normalized] if normalized else []
    if isinstance(value, (list, tuple, set, frozenset)):
        return [str(item).strip() for item in value if str(item or "").strip()]
    return [str(value).strip()] if str(value).strip() else []
