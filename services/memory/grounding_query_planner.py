from __future__ import annotations

import re
from typing import Iterable, List, Optional

from contracts.memory import MemoryPayloadFilter, MemoryRecordKind
from contracts.spatial_memory import GroundingQueryPlan


class GroundingQueryPlanner:
    """把自然语言查询拆成 grounding 规划。"""

    _INTENT_PATTERNS = (
        ("navigate", ("去", "带我去", "导航到", "前往", "到达")),
        ("verify", ("看看", "确认", "是否", "还在", "是不是")),
        ("search", ("找", "寻找", "查询", "检索", "哪里")),
    )
    _SPATIAL_HINTS = ("左边", "右边", "前方", "后方", "门口", "窗边", "附近", "旁边", "角落", "桌上", "架子上")
    _TEMPORAL_HINTS = (
        ("recent", ("刚才", "刚刚", "最近", "现在", "当前")),
        ("today", ("今天", "白天", "早上", "下午", "晚上")),
        ("history", ("之前", "先前", "历史上", "曾经", "上次")),
    )
    _ATTRIBUTE_HINTS = ("红色", "蓝色", "绿色", "黄色", "黑色", "白色", "大", "小", "圆形", "方形", "靠窗", "充电", "补给")

    def plan(self, query: str, *, known_labels: Optional[Iterable[str]] = None) -> GroundingQueryPlan:
        raw_query = str(query or "").strip()
        normalized_query = self._normalize(raw_query)
        intent = self._resolve_intent(raw_query)
        spatial_hint = self._first_match(raw_query, [item for item in self._SPATIAL_HINTS])
        temporal_hint = self._resolve_temporal_hint(raw_query)
        attributes = self._extract_attributes(raw_query)
        target_class = self._resolve_target_class(raw_query, known_labels=known_labels)
        preferred_collections = self._resolve_collections(intent=intent, temporal_hint=temporal_hint)
        return GroundingQueryPlan(
            raw_query=raw_query,
            normalized_query=normalized_query,
            intent=intent,
            target_class=target_class,
            attributes=attributes,
            spatial_hint=spatial_hint,
            temporal_hint=temporal_hint,
            preferred_collections=preferred_collections,
            metadata={"known_label_count": len(list(known_labels or ()))},
        )

    def build_payload_filter(
        self,
        plan: GroundingQueryPlan,
        *,
        map_version_id: Optional[str],
        localization_session_id: Optional[str],
    ) -> MemoryPayloadFilter:
        filter_kwargs = {
            "map_version_id": map_version_id,
            "localization_session_id": localization_session_id,
            "record_kinds": [
                MemoryRecordKind.SCENE,
                MemoryRecordKind.NOTE,
                MemoryRecordKind.OBJECT_INSTANCE,
                MemoryRecordKind.OBSERVATION_EVENT,
            ],
        }
        if plan.temporal_hint == "recent":
            filter_kwargs["max_age_sec"] = 3600.0
            filter_kwargs["last_seen_after"] = None
        if plan.temporal_hint == "today":
            filter_kwargs["max_age_sec"] = 86400.0
        return MemoryPayloadFilter(**filter_kwargs)

    def _resolve_intent(self, query: str) -> str:
        for intent, keywords in self._INTENT_PATTERNS:
            if any(keyword in query for keyword in keywords):
                return intent
        return "search"

    def _resolve_temporal_hint(self, query: str) -> Optional[str]:
        for hint, keywords in self._TEMPORAL_HINTS:
            if any(keyword in query for keyword in keywords):
                return hint
        return None

    def _extract_attributes(self, query: str) -> List[str]:
        result: List[str] = []
        for keyword in self._ATTRIBUTE_HINTS:
            if keyword in query and keyword not in result:
                result.append(keyword)
        return result

    def _resolve_target_class(self, query: str, *, known_labels: Optional[Iterable[str]]) -> Optional[str]:
        normalized_query = self._normalize(query)
        known = []
        for item in known_labels or ():
            normalized = self._normalize(str(item))
            if normalized:
                known.append(normalized)
        for label in known:
            if label and label in normalized_query:
                return label
        condensed = normalized_query.replace(" ", "")
        if not condensed:
            return None
        tokens = re.findall(r"[a-z0-9]+|[\u4e00-\u9fff]{1,8}", condensed)
        if not tokens:
            return condensed
        return tokens[-1]

    def _resolve_collections(self, *, intent: str, temporal_hint: Optional[str]) -> List[str]:
        if intent == "navigate":
            return ["object_instances", "place_nodes", "episodic_observations"]
        if temporal_hint in {"recent", "history", "today"}:
            return ["episodic_observations", "object_instances", "place_nodes"]
        return ["object_instances", "episodic_observations", "place_nodes"]

    def _first_match(self, query: str, keywords: Iterable[str]) -> Optional[str]:
        for keyword in keywords:
            if keyword in query:
                return keyword
        return None

    def _normalize(self, text: str) -> str:
        lowered = str(text or "").strip().lower()
        lowered = re.sub(r"[^0-9a-z\u4e00-\u9fff]+", " ", lowered)
        return " ".join(item for item in lowered.split() if item)
