from __future__ import annotations

import re
from datetime import datetime
from typing import Any, Iterable, List, Optional

from contracts.memory import MemoryPayloadFilter, MemoryRecordKind
from contracts.spatial_memory import GroundingQueryPlan
from services.model_providers import resolve_grounding_provider_spec


class GroundingQueryPlanner:
    """把自然语言查询拆成 grounding 规划。"""

    _PLANNER_VERSION = "4.3-rule-based-v1"
    _INTENT_PATTERNS = (
        ("verify", ("看看", "确认", "是否", "还在", "是不是", "在不在", "check", "verify", "confirm", "still")),
        ("navigate", ("带我去", "导航到", "前往", "到达", "go to", "navigate", "take me", "去")),
        ("search", ("找", "寻找", "查询", "检索", "哪里", "where", "find", "search", "look for")),
    )
    _SPATIAL_HINTS = (
        "左边",
        "右边",
        "前方",
        "后方",
        "门口",
        "窗边",
        "附近",
        "旁边",
        "角落",
        "桌上",
        "架子上",
        "near",
        "beside",
        "next to",
        "left of",
        "right of",
        "in front of",
        "behind",
        "on",
        "inside",
        "at",
    )
    _TEMPORAL_HINTS = (
        ("recent", ("刚才", "刚刚", "最近", "现在", "当前", "recent", "latest", "just now")),
        ("today", ("今天", "白天", "早上", "下午", "晚上", "today", "this morning", "tonight")),
        ("history", ("之前", "先前", "历史上", "曾经", "上次", "before", "previous", "history", "last time")),
    )
    _ATTRIBUTE_GROUPS = {
        "color": ("红色", "红", "蓝色", "蓝", "绿色", "绿", "黄色", "黄", "黑色", "黑", "白色", "白", "red", "blue", "green", "yellow", "black", "white"),
        "size": ("大的", "小的", "大", "小", "large", "big", "small"),
        "shape": ("圆形", "方形", "圆", "方", "round", "square"),
        "state": ("打开", "关闭", "开着", "关着", "亮着", "熄灭", "open", "closed", "on", "off"),
        "usage": ("靠窗", "充电", "补给", "charging", "supply"),
    }
    _ATTRIBUTE_HINTS = tuple(dict.fromkeys(item for values in _ATTRIBUTE_GROUPS.values() for item in values))
    _PLACE_KEYWORDS = (
        "地点",
        "位置",
        "区域",
        "房间",
        "办公室",
        "门口",
        "窗边",
        "角落",
        "补给点",
        "充电位",
        "充电区",
        "工作站",
        "dock",
        "station",
        "room",
        "area",
        "zone",
        "where",
    )
    _OBJECT_KEYWORDS = (
        "看到",
        "看见",
        "这个",
        "那个",
        "物体",
        "目标",
        "人",
        "箱子",
        "盒子",
        "桌子",
        "架子",
        "补给架",
        "person",
        "charger",
        "charging dock",
        "charging_dock",
        "box",
        "table",
    )
    _TARGET_ALIASES = {
        "charging dock": ("charging dock", "charging_dock", "充电位", "充电区", "充电桩", "dock"),
        "charger": ("charger", "补给架", "充电器"),
        "supply point": ("补给点", "补给站", "supply point"),
        "station": ("station", "工作站"),
        "box": ("box", "箱子", "盒子"),
        "table": ("table", "桌子"),
        "doorway": ("doorway", "门口"),
    }
    _NEGATION_KEYWORDS = ("不是", "不要", "排除", "别找", "别去", "不要去", "not", "exclude", "except")
    _SPATIAL_PATTERNS = (
        ("left_of", "left", ("左边", "左侧", "left of", "on the left", "left")),
        ("right_of", "right", ("右边", "右侧", "right of", "on the right", "right")),
        ("front_of", "front", ("前方", "前面", "front of", "in front of", "front")),
        ("behind", "back", ("后方", "后面", "behind", "back")),
        ("near", None, ("附近", "旁边", "靠近", "边上", "near", "beside", "next to", "around")),
        ("on", "up", ("桌上", "架子上", "上面", "on")),
        ("inside", None, ("里面", "内部", "inside", "in")),
        ("at", None, ("门口", "窗边", "角落", "at")),
    )

    def plan(
        self,
        query: str,
        *,
        known_labels: Optional[Iterable[str]] = None,
        known_object_labels: Optional[Iterable[str]] = None,
    ) -> GroundingQueryPlan:
        raw_query = str(query or "").strip()
        normalized_query = self._normalize(raw_query)
        normalized_known_labels = self._normalize_known_labels(known_labels)
        normalized_known_object_labels = self._normalize_known_labels(known_object_labels)
        diagnostics = self._build_empty_diagnostics()
        intent, intent_confidence, intent_reason = self._resolve_intent_details(raw_query, diagnostics=diagnostics)
        temporal_hint, temporal_constraints = self._resolve_temporal_constraints(raw_query, diagnostics=diagnostics)
        spatial_constraints = self._resolve_spatial_constraints(raw_query, diagnostics=diagnostics)
        spatial_hint = spatial_constraints[0]["text"] if spatial_constraints else self._first_match(raw_query, [item for item in self._SPATIAL_HINTS])
        attribute_constraints = self._extract_attribute_constraints(raw_query)
        attributes = [item["name"] for item in attribute_constraints]
        target_candidates = self._resolve_target_candidates(
            raw_query,
            normalized_query=normalized_query,
            known_labels=normalized_known_labels,
            known_object_labels=normalized_known_object_labels,
            diagnostics=diagnostics,
        )
        negative_constraints = self._resolve_negative_constraints(raw_query, target_candidates=target_candidates)
        for candidate in target_candidates:
            candidate["is_negated"] = any(item["normalized"] and item["normalized"] in candidate["normalized"] for item in negative_constraints)
        resolved_target = self._resolve_best_target(target_candidates)
        target_class = resolved_target["normalized"] if resolved_target else self._resolve_target_class(raw_query, known_labels=normalized_known_labels)
        query_scope, scope_reason = self._resolve_query_scope_details(
            raw_query,
            normalized_query=normalized_query,
            target_class=target_class,
            target_candidates=target_candidates,
            known_labels=normalized_known_labels,
            known_object_labels=normalized_known_object_labels,
            diagnostics=diagnostics,
        )
        preferred_record_kinds = self._resolve_record_kinds(query_scope=query_scope)
        preferred_collections = self._resolve_collections(intent=intent, temporal_hint=temporal_hint, query_scope=query_scope)
        target_aliases = self._resolve_target_aliases(target_candidates)
        ambiguities = self._resolve_ambiguities(target_candidates, query_scope=query_scope)
        composite_constraints = self._resolve_composite_constraints(
            resolved_target=resolved_target,
            attribute_constraints=attribute_constraints,
            spatial_constraints=spatial_constraints,
        )
        filter_hints = self._build_filter_hints(
            resolved_target=resolved_target,
            temporal_constraints=temporal_constraints,
            query_scope=query_scope,
        )
        metadata = {
            "planner_version": self._PLANNER_VERSION,
            "known_label_count": len(normalized_known_labels),
            "query_scope": query_scope,
            "scope_reason": scope_reason,
            "preferred_record_kinds": [item.value for item in preferred_record_kinds],
            "preferred_collections_reason": self._preferred_collections_reason(intent=intent, temporal_hint=temporal_hint, query_scope=query_scope),
            "cross_map_protection": True,
            "grounding_provider": resolve_grounding_provider_spec().to_metadata(),
            "intent_confidence": intent_confidence,
            "intent_reason": intent_reason,
            "target_candidates": target_candidates,
            "resolved_target": resolved_target,
            "target_aliases": target_aliases,
            "attribute_constraints": attribute_constraints,
            "negative_constraints": negative_constraints,
            "temporal_constraints": temporal_constraints,
            "spatial_constraints": spatial_constraints,
            "composite_constraints": composite_constraints,
            "verification_intent": self._build_verification_intent(intent=intent, raw_query=raw_query, intent_reason=intent_reason),
            "candidate_ranking_hints": {
                "prefer_record_kinds": [item.value for item in preferred_record_kinds],
                "prefer_recent": temporal_hint == "recent",
                "prefer_exact_label": bool(resolved_target and resolved_target.get("source") in {"known_label", "known_object_label", "alias"}),
                "prefer_spatial_relation": bool(spatial_constraints),
                "prefer_verified_place": query_scope == "place",
                "query_scope": query_scope,
            },
            "filter_hints": filter_hints,
            "fallback_semantics": {
                "tagged_location_fallback": query_scope in {"place", "mixed"},
                "fallback_target_text": target_class,
                "allow_mixed_scope_expansion": query_scope == "mixed",
                "reason": "place_or_mixed_scope_allows_formal_query_location_fallback" if query_scope in {"place", "mixed"} else "object_scope_uses_object_first_retrieval",
            },
            "diagnostics": diagnostics,
            "ambiguities": ambiguities,
        }
        return GroundingQueryPlan(
            raw_query=raw_query,
            normalized_query=normalized_query,
            intent=intent,
            target_class=target_class,
            attributes=attributes,
            spatial_hint=spatial_hint,
            temporal_hint=temporal_hint,
            preferred_collections=preferred_collections,
            metadata=metadata,
        )

    def build_payload_filter(
        self,
        plan: GroundingQueryPlan,
        *,
        map_version_id: Optional[str],
        localization_session_id: Optional[str],
    ) -> MemoryPayloadFilter:
        metadata = dict(plan.metadata or {})
        preferred_record_kinds = self._record_kinds_from_metadata(metadata)
        filter_kwargs: dict[str, Any] = {
            "map_version_id": map_version_id,
            "localization_session_id": localization_session_id,
            "record_kinds": preferred_record_kinds,
        }
        filter_hints = metadata.get("filter_hints") if isinstance(metadata.get("filter_hints"), dict) else {}
        if plan.temporal_hint == "recent":
            filter_kwargs["max_age_sec"] = 3600.0
            filter_kwargs["last_seen_after"] = None
        elif plan.temporal_hint == "today":
            filter_kwargs["max_age_sec"] = 86400.0
        elif isinstance(filter_hints.get("max_age_sec"), (int, float)):
            filter_kwargs["max_age_sec"] = float(filter_hints["max_age_sec"])
        for field_name in ("created_after", "created_before", "last_seen_after", "last_seen_before"):
            parsed = self._parse_metadata_datetime(filter_hints.get(field_name))
            if parsed is not None:
                filter_kwargs[field_name] = parsed
        return MemoryPayloadFilter(**filter_kwargs)

    def _resolve_intent(self, query: str) -> str:
        intent, _, _ = self._resolve_intent_details(query, diagnostics=self._build_empty_diagnostics())
        return intent

    def _resolve_intent_details(self, query: str, *, diagnostics: dict[str, list[str]]) -> tuple[str, float, str]:
        normalized_query = self._normalize(query)
        matched: list[tuple[str, str]] = []
        for intent, keywords in self._INTENT_PATTERNS:
            for keyword in keywords:
                if self._keyword_matches(query, normalized_query, keyword):
                    matched.append((intent, keyword))
        for intent in ("verify", "navigate", "search"):
            intent_keywords = [keyword for matched_intent, keyword in matched if matched_intent == intent]
            if intent_keywords:
                diagnostics["matched_intent_keywords"].extend(self._deduplicate(intent_keywords))
                confidence = 0.92 if len({item[0] for item in matched}) == 1 else 0.82
                return intent, confidence, f"matched_{intent}_keywords"
        return "search", 0.55, "default_search_without_explicit_intent"

    def _resolve_temporal_hint(self, query: str) -> Optional[str]:
        hint, _ = self._resolve_temporal_constraints(query, diagnostics=self._build_empty_diagnostics())
        return hint

    def _resolve_temporal_constraints(self, query: str, *, diagnostics: dict[str, list[str]]) -> tuple[Optional[str], list[dict[str, Any]]]:
        normalized_query = self._normalize(query)
        for hint, keywords in self._TEMPORAL_HINTS:
            matched = [keyword for keyword in keywords if self._keyword_matches(query, normalized_query, keyword)]
            if matched:
                diagnostics["matched_temporal_keywords"].extend(self._deduplicate(matched))
                max_age_sec = 3600.0 if hint == "recent" else 86400.0 if hint == "today" else None
                return hint, [
                    {
                        "hint": hint,
                        "text": matched[0],
                        "max_age_sec": max_age_sec,
                        "created_after": None,
                        "created_before": None,
                        "last_seen_after": None,
                        "last_seen_before": None,
                        "confidence": 0.9,
                    }
                ]
        return None, []

    def _extract_attributes(self, query: str) -> List[str]:
        return [item["name"] for item in self._extract_attribute_constraints(query)]

    def _extract_attribute_constraints(self, query: str) -> list[dict[str, Any]]:
        normalized_query = self._normalize(query)
        result: list[dict[str, Any]] = []
        for attribute_type, keywords in self._ATTRIBUTE_GROUPS.items():
            for keyword in keywords:
                if not self._keyword_matches(query, normalized_query, keyword):
                    continue
                canonical = self._canonical_attribute(keyword, attribute_type=attribute_type)
                if any(item["normalized"] == self._normalize(canonical) for item in result):
                    continue
                result.append(
                    {
                        "name": canonical,
                        "normalized": self._normalize(canonical),
                        "type": attribute_type,
                        "source": "dictionary",
                        "confidence": 0.86,
                        "is_negated": self._is_near_negation(query, keyword),
                    }
                )
        return result

    def _resolve_target_class(self, query: str, *, known_labels: Optional[Iterable[str]]) -> Optional[str]:
        normalized_query = self._normalize(query)
        known = list(known_labels or ())
        for label in known:
            if label and label in normalized_query:
                return label
        alias_candidates = self._alias_candidates(normalized_query)
        if alias_candidates:
            return alias_candidates[0]["normalized"]
        condensed = normalized_query.replace(" ", "")
        if not condensed:
            return None
        tokens = re.findall(r"[a-z0-9]+|[一-鿿]{1,8}", condensed)
        if not tokens:
            return condensed
        return tokens[-1]

    def _resolve_query_scope(
        self,
        query: str,
        *,
        normalized_query: str,
        target_class: Optional[str],
        known_labels: List[str],
        known_object_labels: List[str],
    ) -> str:
        query_scope, _ = self._resolve_query_scope_details(
            query,
            normalized_query=normalized_query,
            target_class=target_class,
            target_candidates=[],
            known_labels=known_labels,
            known_object_labels=known_object_labels,
            diagnostics=self._build_empty_diagnostics(),
        )
        return query_scope

    def _resolve_query_scope_details(
        self,
        query: str,
        *,
        normalized_query: str,
        target_class: Optional[str],
        target_candidates: list[dict[str, Any]],
        known_labels: List[str],
        known_object_labels: List[str],
        diagnostics: dict[str, list[str]],
    ) -> tuple[str, str]:
        kinds = {candidate["kind"] for candidate in target_candidates if candidate.get("kind") in {"place", "object"}}
        if len(kinds) > 1:
            return "mixed", "target_candidates_include_place_and_object"
        if "object" in kinds:
            return "object", "resolved_target_kind_object"
        if "place" in kinds:
            return "place", "resolved_target_kind_place"
        normalized_object_keywords = self._normalize_known_labels(self._OBJECT_KEYWORDS)
        matched_object_keywords = [keyword for keyword in normalized_object_keywords if keyword and keyword in normalized_query]
        if matched_object_keywords:
            diagnostics["matched_scope_keywords"].extend(self._deduplicate(matched_object_keywords))
            return "object", "matched_object_scope_keyword"
        if target_class and target_class in known_object_labels:
            return "object", "target_class_in_known_object_labels"
        matched_place_keywords = [keyword for keyword in self._PLACE_KEYWORDS if self._keyword_matches(query, normalized_query, keyword)]
        if matched_place_keywords:
            diagnostics["matched_scope_keywords"].extend(self._deduplicate(matched_place_keywords))
            return "place", "matched_place_scope_keyword"
        if target_class and target_class in known_labels:
            return "object", "target_class_in_known_labels_without_place_signal"
        return "mixed", "no_decisive_scope_signal"

    def _resolve_record_kinds(self, *, query_scope: str) -> List[MemoryRecordKind]:
        if query_scope == "place":
            return [
                MemoryRecordKind.TAGGED_LOCATION,
                MemoryRecordKind.SCENE,
                MemoryRecordKind.NOTE,
                MemoryRecordKind.OBSERVATION_EVENT,
                MemoryRecordKind.OBJECT_INSTANCE,
            ]
        if query_scope == "object":
            return [
                MemoryRecordKind.OBJECT_INSTANCE,
                MemoryRecordKind.OBSERVATION_EVENT,
                MemoryRecordKind.SCENE,
                MemoryRecordKind.NOTE,
                MemoryRecordKind.TAGGED_LOCATION,
            ]
        return [
            MemoryRecordKind.OBJECT_INSTANCE,
            MemoryRecordKind.OBSERVATION_EVENT,
            MemoryRecordKind.TAGGED_LOCATION,
            MemoryRecordKind.SCENE,
            MemoryRecordKind.NOTE,
        ]

    def _resolve_collections(self, *, intent: str, temporal_hint: Optional[str], query_scope: str) -> List[str]:
        if query_scope == "place":
            return ["place_nodes", "episodic_observations", "object_instances"]
        if intent == "navigate":
            return ["object_instances", "place_nodes", "episodic_observations"]
        if temporal_hint in {"recent", "history", "today"}:
            return ["episodic_observations", "object_instances", "place_nodes"]
        return ["object_instances", "episodic_observations", "place_nodes"]

    def _record_kinds_from_metadata(self, metadata: dict) -> List[MemoryRecordKind]:
        preferred_values = metadata.get("preferred_record_kinds") or []
        result: List[MemoryRecordKind] = []
        for value in preferred_values:
            try:
                result.append(MemoryRecordKind(str(value)))
            except ValueError:
                continue
        if result:
            return result
        return [
            MemoryRecordKind.SCENE,
            MemoryRecordKind.NOTE,
            MemoryRecordKind.OBJECT_INSTANCE,
            MemoryRecordKind.OBSERVATION_EVENT,
        ]

    def _normalize_known_labels(self, known_labels: Optional[Iterable[str]]) -> List[str]:
        result: List[str] = []
        for item in known_labels or ():
            normalized = self._normalize(str(item))
            if normalized and normalized not in result:
                result.append(normalized)
        return result

    def _resolve_target_candidates(
        self,
        query: str,
        *,
        normalized_query: str,
        known_labels: list[str],
        known_object_labels: list[str],
        diagnostics: dict[str, list[str]],
    ) -> list[dict[str, Any]]:
        result: list[dict[str, Any]] = []
        known_object_set = set(known_object_labels)
        for label in known_labels:
            if not label or label not in normalized_query:
                continue
            self._append_target_candidate(
                result,
                text=label,
                normalized=label,
                kind="object" if label in known_object_set else "place",
                source="known_object_label" if label in known_object_set else "known_label",
                confidence=0.94,
                matched_label=label,
                aliases=[],
                is_negated=self._is_near_negation(query, label),
            )
        for candidate in self._alias_candidates(normalized_query):
            diagnostics["matched_aliases"].extend(candidate["aliases"])
            self._append_target_candidate(
                result,
                text=candidate["text"],
                normalized=candidate["normalized"],
                kind=candidate["kind"],
                source="alias",
                confidence=0.88,
                matched_label=candidate["normalized"],
                aliases=candidate["aliases"],
                is_negated=self._is_near_negation(query, candidate["text"]),
            )
        if result:
            return result
        fallback = self._fallback_target_text(normalized_query)
        if fallback:
            self._append_target_candidate(
                result,
                text=fallback,
                normalized=self._normalize(fallback),
                kind="unknown",
                source="fallback",
                confidence=0.48,
                matched_label=None,
                aliases=[],
                is_negated=self._is_near_negation(query, fallback),
            )
        return result

    def _append_target_candidate(
        self,
        result: list[dict[str, Any]],
        *,
        text: str,
        normalized: str,
        kind: str,
        source: str,
        confidence: float,
        matched_label: Optional[str],
        aliases: list[str],
        is_negated: bool,
    ) -> None:
        if any(item["normalized"] == normalized and item["kind"] == kind for item in result):
            return
        result.append(
            {
                "text": text,
                "normalized": normalized,
                "kind": kind,
                "source": source,
                "confidence": confidence,
                "matched_label": matched_label,
                "aliases": self._deduplicate([self._normalize(alias) for alias in aliases if alias]),
                "is_negated": is_negated,
            }
        )

    def _resolve_best_target(self, target_candidates: list[dict[str, Any]]) -> Optional[dict[str, Any]]:
        if not target_candidates:
            return None
        available = [candidate for candidate in target_candidates if not candidate.get("is_negated")] or target_candidates
        object_candidates = [candidate for candidate in available if candidate.get("kind") == "object"]
        ranked = object_candidates or available
        return sorted(ranked, key=lambda item: -float(item.get("confidence") or 0.0))[0]

    def _alias_candidates(self, normalized_query: str) -> list[dict[str, Any]]:
        result: list[dict[str, Any]] = []
        for canonical, aliases in self._TARGET_ALIASES.items():
            normalized_canonical = self._normalize(canonical)
            normalized_aliases = self._normalize_known_labels(aliases)
            matched_aliases = [alias for alias in normalized_aliases if alias and alias in normalized_query]
            if not matched_aliases:
                continue
            result.append(
                {
                    "text": matched_aliases[0],
                    "normalized": normalized_canonical,
                    "kind": self._alias_kind(canonical),
                    "aliases": normalized_aliases,
                }
            )
        return result

    def _alias_kind(self, canonical: str) -> str:
        if canonical in {"doorway", "supply point", "station"}:
            return "place"
        return "object"

    def _resolve_target_aliases(self, target_candidates: list[dict[str, Any]]) -> list[str]:
        aliases: list[str] = []
        for candidate in target_candidates:
            aliases.extend(str(alias) for alias in candidate.get("aliases") or [])
        return self._deduplicate(aliases)

    def _resolve_negative_constraints(self, query: str, *, target_candidates: list[dict[str, Any]]) -> list[dict[str, Any]]:
        normalized_query = self._normalize(query)
        result: list[dict[str, Any]] = []
        for keyword in self._NEGATION_KEYWORDS:
            normalized_keyword = self._normalize(keyword)
            index = normalized_query.find(normalized_keyword)
            if index < 0:
                continue
            tail = normalized_query[index + len(normalized_keyword):].strip()
            text = self._first_meaningful_token(tail) or keyword
            normalized = self._normalize(text)
            constraint_type = self._negative_constraint_type(normalized, target_candidates=target_candidates)
            result.append(
                {
                    "text": text,
                    "normalized": normalized,
                    "type": constraint_type,
                    "confidence": 0.78,
                    "applies_to": target_candidates[0]["normalized"] if target_candidates else None,
                    "hard_filter": False,
                }
            )
        return result

    def _negative_constraint_type(self, normalized: str, *, target_candidates: list[dict[str, Any]]) -> str:
        if any(candidate.get("normalized") == normalized for candidate in target_candidates):
            return "target"
        if any(self._normalize(keyword) in normalized for keyword in self._PLACE_KEYWORDS):
            return "place"
        if any(self._normalize(keyword) in normalized for keyword in self._SPATIAL_HINTS):
            return "spatial"
        if any(self._normalize(keyword) in normalized for keyword in self._ATTRIBUTE_HINTS):
            return "attribute"
        return "unknown"

    def _resolve_spatial_constraints(self, query: str, *, diagnostics: dict[str, list[str]]) -> list[dict[str, Any]]:
        normalized_query = self._normalize(query)
        result: list[dict[str, Any]] = []
        for relation, direction, keywords in self._SPATIAL_PATTERNS:
            matched = [keyword for keyword in keywords if self._keyword_matches(query, normalized_query, keyword)]
            if not matched:
                continue
            keyword = matched[0]
            diagnostics["matched_spatial_keywords"].append(keyword)
            anchor_text = self._resolve_spatial_anchor(query, keyword=keyword)
            result.append(
                {
                    "relation": relation,
                    "text": keyword,
                    "anchor_text": anchor_text,
                    "anchor_normalized": self._normalize(anchor_text) if anchor_text else None,
                    "anchor_kind": self._infer_anchor_kind(anchor_text) if anchor_text else None,
                    "direction": direction,
                    "distance_m": self._extract_distance_m(query),
                    "confidence": 0.82 if anchor_text else 0.68,
                }
            )
        return result[:3]

    def _resolve_spatial_anchor(self, query: str, *, keyword: str) -> Optional[str]:
        normalized_query = self._normalize(query)
        normalized_keyword = self._normalize(keyword)
        tokens = normalized_query.split()
        if not tokens:
            return None
        if normalized_keyword in normalized_query:
            keyword_tokens = normalized_keyword.split()
            for index in range(0, max(len(tokens) - len(keyword_tokens) + 1, 0)):
                if tokens[index : index + len(keyword_tokens)] != keyword_tokens:
                    continue
                before = tokens[max(0, index - 2) : index]
                after = tokens[index + len(keyword_tokens) : index + len(keyword_tokens) + 2]
                anchor_tokens = after or before
                anchor_tokens = [item for item in anchor_tokens if item not in {"的", "那个", "这个", "the", "a", "an"}]
                if anchor_tokens:
                    return " ".join(anchor_tokens)
        compact_query = normalized_query.replace(" ", "")
        compact_keyword = normalized_keyword.replace(" ", "")
        index = compact_query.find(compact_keyword)
        if index > 0:
            before = compact_query[max(0, index - 8) : index]
            chunks = re.findall(r"[a-z0-9]+|[一-鿿]{1,8}", before)
            if chunks:
                return chunks[-1]
        if index >= 0:
            after = compact_query[index + len(compact_keyword) : index + len(compact_keyword) + 8]
            chunks = re.findall(r"[a-z0-9]+|[一-鿿]{1,8}", after)
            if chunks:
                return chunks[0]
        return None

    def _infer_anchor_kind(self, anchor_text: Optional[str]) -> str:
        normalized = self._normalize(anchor_text or "")
        if any(self._normalize(keyword) in normalized for keyword in self._PLACE_KEYWORDS):
            return "place"
        if any(self._normalize(keyword) in normalized for keyword in self._OBJECT_KEYWORDS):
            return "object"
        return "unknown"

    def _extract_distance_m(self, query: str) -> Optional[float]:
        match = re.search(r"(\d+(?:\.\d+)?)\s*(?:米|m|meter|meters)", str(query or ""), flags=re.IGNORECASE)
        if not match:
            return None
        try:
            return float(match.group(1))
        except ValueError:
            return None

    def _resolve_composite_constraints(
        self,
        *,
        resolved_target: Optional[dict[str, Any]],
        attribute_constraints: list[dict[str, Any]],
        spatial_constraints: list[dict[str, Any]],
    ) -> list[dict[str, Any]]:
        if not resolved_target or not (attribute_constraints or spatial_constraints):
            return []
        modifiers = [item["normalized"] for item in attribute_constraints]
        modifiers.extend(item["anchor_normalized"] for item in spatial_constraints if item.get("anchor_normalized"))
        return [
            {
                "head": resolved_target["normalized"],
                "modifiers": self._deduplicate([item for item in modifiers if item]),
                "relations": self._deduplicate([item["relation"] for item in spatial_constraints]),
                "confidence": min(0.9, float(resolved_target.get("confidence") or 0.5) + 0.04),
            }
        ]

    def _build_filter_hints(
        self,
        *,
        resolved_target: Optional[dict[str, Any]],
        temporal_constraints: list[dict[str, Any]],
        query_scope: str,
    ) -> dict[str, Any]:
        semantic_labels: list[str] = []
        visual_labels: list[str] = []
        vision_tags: list[str] = []
        if resolved_target and float(resolved_target.get("confidence") or 0.0) >= 0.93 and not resolved_target.get("is_negated"):
            label = str(resolved_target.get("normalized") or "")
            if query_scope == "place":
                semantic_labels.append(label)
            elif query_scope == "object":
                visual_labels.append(label)
                vision_tags.append(label)
        temporal = temporal_constraints[0] if temporal_constraints else {}
        return {
            "semantic_labels_any": semantic_labels,
            "visual_labels_any": visual_labels,
            "vision_tags_any": vision_tags,
            "instance_types": [],
            "movabilities": [],
            "max_age_sec": temporal.get("max_age_sec"),
            "created_after": temporal.get("created_after"),
            "created_before": temporal.get("created_before"),
            "last_seen_after": temporal.get("last_seen_after"),
            "last_seen_before": temporal.get("last_seen_before"),
        }

    def _build_verification_intent(self, *, intent: str, raw_query: str, intent_reason: str) -> Optional[dict[str, Any]]:
        if intent != "verify":
            return None
        expected_presence = None
        normalized_query = self._normalize(raw_query)
        if any(keyword in normalized_query for keyword in ("还在", "在不在", "still")):
            expected_presence = True
        return {"requested": True, "query_text": raw_query, "expected_presence": expected_presence, "reason": intent_reason}

    def _resolve_ambiguities(self, target_candidates: list[dict[str, Any]], *, query_scope: str) -> list[dict[str, Any]]:
        result: list[dict[str, Any]] = []
        if len(target_candidates) > 1:
            candidates = [candidate["normalized"] for candidate in target_candidates]
            result.append({"type": "target", "message": "multiple_target_candidates", "candidates": candidates, "resolution": target_candidates[0]["normalized"]})
        kinds = sorted({candidate["kind"] for candidate in target_candidates if candidate.get("kind") != "unknown"})
        if len(kinds) > 1 or query_scope == "mixed":
            result.append({"type": "scope", "message": "scope_not_fully_decisive", "candidates": kinds, "resolution": query_scope})
        return result

    def _preferred_collections_reason(self, *, intent: str, temporal_hint: Optional[str], query_scope: str) -> str:
        if query_scope == "place":
            return "place_scope_prefers_place_nodes"
        if intent == "navigate":
            return "navigate_intent_prefers_object_instances"
        if temporal_hint in {"recent", "history", "today"}:
            return "temporal_query_prefers_episodic_observations"
        return "default_object_first_collections"

    def _fallback_target_text(self, normalized_query: str) -> Optional[str]:
        cleaned = normalized_query
        stopwords = self._normalize_known_labels(
            [
                "带我去",
                "导航到",
                "前往",
                "找",
                "寻找",
                "查询",
                "检索",
                "哪里",
                "看看",
                "确认",
                "是否",
                "go to",
                "navigate",
                "find",
                "search",
                "where",
                "the",
                "a",
                "an",
            ]
        )
        for stopword in stopwords:
            cleaned = cleaned.replace(stopword, " ")
        tokens = [item for item in cleaned.split() if item]
        if not tokens:
            compact = cleaned.replace(" ", "")
            chunks = re.findall(r"[a-z0-9]+|[一-鿿]{1,8}", compact)
            return chunks[-1] if chunks else None
        return tokens[-1]

    def _first_meaningful_token(self, text: str) -> Optional[str]:
        for token in text.split():
            if token and token not in {"的", "那个", "这个", "the", "a", "an"}:
                return token
        compact = text.replace(" ", "")
        chunks = re.findall(r"[a-z0-9]+|[一-鿿]{1,8}", compact)
        return chunks[0] if chunks else None

    def _is_near_negation(self, query: str, keyword: str) -> bool:
        normalized_query = self._normalize(query)
        normalized_keyword = self._normalize(keyword)
        index = normalized_query.find(normalized_keyword)
        if index < 0:
            return False
        prefix = normalized_query[max(0, index - 10) : index]
        return any(self._normalize(negation) in prefix for negation in self._NEGATION_KEYWORDS)

    def _canonical_attribute(self, keyword: str, *, attribute_type: str) -> str:
        normalized = self._normalize(keyword)
        color_map = {
            "红": "红色",
            "red": "红色",
            "蓝": "蓝色",
            "blue": "蓝色",
            "绿": "绿色",
            "green": "绿色",
            "黄": "黄色",
            "yellow": "黄色",
            "黑": "黑色",
            "black": "黑色",
            "白": "白色",
            "white": "白色",
        }
        if attribute_type == "color":
            return color_map.get(normalized, keyword)
        return keyword

    def _parse_metadata_datetime(self, value: Any) -> Optional[datetime]:
        if not isinstance(value, str) or not value.strip():
            return None
        try:
            return datetime.fromisoformat(value.replace("Z", "+00:00"))
        except ValueError:
            return None

    def _build_empty_diagnostics(self) -> dict[str, list[str]]:
        return {
            "matched_intent_keywords": [],
            "matched_scope_keywords": [],
            "matched_aliases": [],
            "matched_temporal_keywords": [],
            "matched_spatial_keywords": [],
            "normalization_notes": [],
            "warnings": [],
        }

    def _first_match(self, query: str, keywords: Iterable[str]) -> Optional[str]:
        normalized_query = self._normalize(query)
        for keyword in keywords:
            if self._keyword_matches(query, normalized_query, keyword):
                return keyword
        return None

    def _keyword_matches(self, raw_query: str, normalized_query: str, keyword: str) -> bool:
        normalized_keyword = self._normalize(keyword)
        if not normalized_keyword:
            return False
        return str(keyword or "") in str(raw_query or "") or normalized_keyword in normalized_query

    def _deduplicate(self, values: Iterable[str]) -> list[str]:
        result: list[str] = []
        for value in values:
            if value and value not in result:
                result.append(value)
        return result

    def _normalize(self, text: str) -> str:
        lowered = str(text or "").strip().lower()
        lowered = re.sub(r"[^0-9a-z一-鿿]+", " ", lowered)
        return " ".join(item for item in lowered.split() if item)
