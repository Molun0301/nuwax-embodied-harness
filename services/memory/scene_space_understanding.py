from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

from contracts.base import utc_now
from contracts.runtime_views import SceneObjectSummary, SceneSummary
from contracts.spatial_memory import SceneCandidateLabel, SceneModelSummary, SceneObservationSample
from services.model_providers import (
    normalize_scene_space_vlm_metadata,
    resolve_scene_space_rule_fallback_provider_metadata,
)


@dataclass(frozen=True)
class SceneSpaceUnderstandingResult:
    """场景空间理解归一化结果。"""

    model_scene_summary: SceneModelSummary
    candidate_labels: Tuple[SceneCandidateLabel, ...]
    evidence_sample_ids: Tuple[str, ...]
    confidence: float
    provider_metadata: Dict[str, object] = field(default_factory=dict)
    availability: str = "available"
    fallback_reason: str = ""


class SceneSpaceUnderstandingAdapter:
    """归一化场景空间 VLM 输出。"""

    _ADAPTER_VERSION = "scene-space-understanding-v1"
    _MAX_LABELS = 8
    _FALLBACK_CONFIDENCE = 0.28

    def normalize(
        self,
        model_output: Optional[object],
        *,
        observation_samples: Sequence[SceneObservationSample] = (),
        scene_summaries: Sequence[SceneSummary] = (),
        provider_metadata: Optional[Dict[str, object]] = None,
        availability: str = "available",
        fallback_reason: str = "",
    ) -> SceneSpaceUnderstandingResult:
        payload = self._coerce_payload(model_output)
        metadata = self._resolve_provider_metadata(
            payload=payload,
            provider_metadata=provider_metadata,
            availability=availability,
            fallback_reason=fallback_reason,
        )
        resolved_availability = str(metadata.get("availability") or availability or "available")
        resolved_fallback_reason = str(metadata.get("fallback_reason") or fallback_reason or "")
        if resolved_availability != "available" or resolved_fallback_reason:
            return self.build_rule_fallback(
                observation_samples=observation_samples,
                scene_summaries=scene_summaries,
                fallback_reason=resolved_fallback_reason or "provider_not_available",
                availability=resolved_availability if resolved_availability != "available" else "unavailable",
                provider_metadata=metadata,
            )

        summary_text = self._extract_summary_text(payload, scene_summaries=scene_summaries)
        evidence_sample_ids = self._extract_evidence_sample_ids(payload, observation_samples)
        confidence = self._coerce_confidence(payload.get("confidence"), default=self._summary_confidence(scene_summaries))
        candidate_labels = self._build_candidate_labels(
            payload.get("candidate_labels") or payload.get("labels") or payload.get("tags"),
            evidence_sample_ids=evidence_sample_ids,
            provider_metadata=metadata,
            default_confidence=confidence,
        )
        if not summary_text and not candidate_labels:
            return self.build_rule_fallback(
                observation_samples=observation_samples,
                scene_summaries=scene_summaries,
                fallback_reason="empty_model_output",
                availability="unavailable",
                provider_metadata=metadata,
            )
        if not candidate_labels:
            candidate_labels = self._fallback_labels_from_observations(
                observation_samples=observation_samples,
                scene_summaries=scene_summaries,
                evidence_sample_ids=evidence_sample_ids,
                provider_metadata=metadata,
                confidence=min(confidence, 0.5),
            )
        if not summary_text:
            summary_text = self._build_observation_summary(scene_summaries=scene_summaries, observation_samples=observation_samples)

        model_summary = SceneModelSummary(
            summary=summary_text,
            provider_name=self._metadata_text(metadata, "provider_name"),
            model_name=self._metadata_text(metadata, "model_name"),
            confidence=confidence,
            evidence_observation_sample_ids=list(evidence_sample_ids),
            generated_at=utc_now(),
            metadata={
                "adapter_version": self._ADAPTER_VERSION,
                "provider": metadata,
                "availability": resolved_availability,
                "fallback_reason": resolved_fallback_reason,
                "fallback_used": False,
            },
        )
        return SceneSpaceUnderstandingResult(
            model_scene_summary=model_summary,
            candidate_labels=candidate_labels,
            evidence_sample_ids=evidence_sample_ids,
            confidence=confidence,
            provider_metadata=metadata,
            availability=resolved_availability,
            fallback_reason=resolved_fallback_reason,
        )

    def build_rule_fallback(
        self,
        *,
        observation_samples: Sequence[SceneObservationSample] = (),
        scene_summaries: Sequence[SceneSummary] = (),
        fallback_reason: str = "provider_not_available",
        availability: str = "unavailable",
        provider_metadata: Optional[Dict[str, object]] = None,
    ) -> SceneSpaceUnderstandingResult:
        metadata = resolve_scene_space_rule_fallback_provider_metadata(
            fallback_reason=fallback_reason,
            availability=availability,
            requested_metadata=provider_metadata,
        )
        evidence_sample_ids = self._sample_ids(observation_samples)
        confidence = self._FALLBACK_CONFIDENCE if evidence_sample_ids or scene_summaries else 0.16
        candidate_labels = self._fallback_labels_from_observations(
            observation_samples=observation_samples,
            scene_summaries=scene_summaries,
            evidence_sample_ids=evidence_sample_ids,
            provider_metadata=metadata,
            confidence=confidence,
        )
        summary_text = self._build_observation_summary(scene_summaries=scene_summaries, observation_samples=observation_samples)
        if not summary_text:
            summary_text = "模型不可用，当前仅基于有限观测记录生成低置信度空间摘要。"
        else:
            summary_text = f"{summary_text}模型不可用，当前仅作为低置信度场景空间候选。"
        model_summary = SceneModelSummary(
            summary=summary_text,
            provider_name=self._metadata_text(metadata, "provider_name"),
            model_name=self._metadata_text(metadata, "model_name"),
            confidence=confidence,
            evidence_observation_sample_ids=list(evidence_sample_ids),
            generated_at=utc_now(),
            metadata={
                "adapter_version": self._ADAPTER_VERSION,
                "provider": metadata,
                "availability": str(metadata.get("availability") or availability),
                "fallback_reason": str(metadata.get("fallback_reason") or fallback_reason),
                "fallback_used": True,
            },
        )
        return SceneSpaceUnderstandingResult(
            model_scene_summary=model_summary,
            candidate_labels=candidate_labels,
            evidence_sample_ids=evidence_sample_ids,
            confidence=confidence,
            provider_metadata=metadata,
            availability=str(metadata.get("availability") or availability),
            fallback_reason=str(metadata.get("fallback_reason") or fallback_reason),
        )

    def _resolve_provider_metadata(
        self,
        *,
        payload: Dict[str, object],
        provider_metadata: Optional[Dict[str, object]],
        availability: str,
        fallback_reason: str,
    ) -> Dict[str, object]:
        payload_metadata = payload.get("provider_metadata") or payload.get("metadata") or {}
        if isinstance(payload_metadata, dict) and isinstance(payload_metadata.get("scene_provider"), dict):
            payload_metadata = payload_metadata.get("scene_provider") or {}
        merged = {
            **(dict(payload_metadata) if isinstance(payload_metadata, dict) else {}),
            **dict(provider_metadata or {}),
        }
        return normalize_scene_space_vlm_metadata(
            merged,
            availability=availability,
            fallback_reason=fallback_reason,
        )

    def _coerce_payload(self, model_output: Optional[object]) -> Dict[str, object]:
        if model_output is None:
            return {}
        if isinstance(model_output, SceneSummary):
            return {
                "summary": model_output.headline,
                "details": list(model_output.details),
                "candidate_labels": list(model_output.metadata.get("semantic_tags") or []) or [item.label for item in model_output.objects],
                "confidence": self._summary_confidence([model_output]),
                "metadata": dict(model_output.metadata),
            }
        if isinstance(model_output, dict):
            return dict(model_output)
        payload = getattr(model_output, "model_dump", None)
        if callable(payload):
            value = payload(mode="json")
            return dict(value) if isinstance(value, dict) else {}
        return {}

    def _extract_summary_text(self, payload: Dict[str, object], *, scene_summaries: Sequence[SceneSummary]) -> str:
        for key in ("model_scene_summary", "scene_summary", "summary", "headline"):
            value = payload.get(key)
            if isinstance(value, dict):
                value = value.get("summary") or value.get("text") or value.get("headline")
            text = str(value or "").strip()
            if text:
                return text
        return self._build_observation_summary(scene_summaries=scene_summaries, observation_samples=())

    def _build_candidate_labels(
        self,
        raw_labels: object,
        *,
        evidence_sample_ids: Tuple[str, ...],
        provider_metadata: Dict[str, object],
        default_confidence: float,
    ) -> Tuple[SceneCandidateLabel, ...]:
        labels: List[SceneCandidateLabel] = []
        seen = set()
        if not isinstance(raw_labels, list):
            return tuple()
        for raw_item in raw_labels:
            item = raw_item if isinstance(raw_item, dict) else {"label": raw_item}
            label = str(item.get("label") or item.get("name") or item.get("text") or "").strip()
            normalized_label = self._normalize_label(item.get("normalized_label") or label)
            if not label or normalized_label in seen:
                continue
            seen.add(normalized_label)
            item_evidence = self._dedupe_strings(
                item.get("evidence_observation_sample_ids") or item.get("evidence_sample_ids") or item.get("sample_ids") or evidence_sample_ids
            )
            labels.append(
                SceneCandidateLabel(
                    label=label,
                    normalized_label=normalized_label,
                    confidence=self._coerce_confidence(item.get("confidence"), default=default_confidence),
                    source_model=self._metadata_text(provider_metadata, "model_name") or self._metadata_text(provider_metadata, "provider_name"),
                    evidence_observation_sample_ids=item_evidence,
                    evidence_observation_event_ids=self._dedupe_strings(item.get("evidence_observation_event_ids") or []),
                    metadata={
                        "provider": provider_metadata,
                        "adapter_version": self._ADAPTER_VERSION,
                    },
                )
            )
            if len(labels) >= self._MAX_LABELS:
                break
        return tuple(labels)

    def _fallback_labels_from_observations(
        self,
        *,
        observation_samples: Sequence[SceneObservationSample],
        scene_summaries: Sequence[SceneSummary],
        evidence_sample_ids: Tuple[str, ...],
        provider_metadata: Dict[str, object],
        confidence: float,
    ) -> Tuple[SceneCandidateLabel, ...]:
        raw_labels: List[object] = []
        for summary in scene_summaries:
            raw_labels.extend(summary.metadata.get("semantic_tags") or [])
            raw_labels.extend(summary.metadata.get("visual_labels") or [])
            raw_labels.extend(item.label for item in summary.objects)
        for sample in observation_samples:
            raw_labels.extend(sample.vision_tags)
            raw_labels.extend(sample.visual_labels)
        labels = []
        for label in self._dedupe_strings(raw_labels):
            labels.append(
                {
                    "label": label,
                    "normalized_label": self._normalize_label(label),
                    "confidence": confidence,
                    "evidence_sample_ids": evidence_sample_ids,
                }
            )
        return self._build_candidate_labels(
            labels[: self._MAX_LABELS],
            evidence_sample_ids=evidence_sample_ids,
            provider_metadata=provider_metadata,
            default_confidence=confidence,
        )

    def _build_observation_summary(
        self,
        *,
        scene_summaries: Sequence[SceneSummary],
        observation_samples: Sequence[SceneObservationSample],
    ) -> str:
        headlines = self._dedupe_strings(summary.headline for summary in scene_summaries if summary.headline)
        object_labels = self._top_object_labels(scene_summaries)
        sample_labels = self._dedupe_strings(
            label for sample in observation_samples for label in [*sample.vision_tags, *sample.visual_labels]
        )
        labels = self._dedupe_strings([*object_labels, *sample_labels])[:5]
        if headlines:
            prefix = " ".join(headlines[:2])
            return prefix if prefix.endswith(("。", ".", "！", "!")) else prefix + "。"
        if labels:
            return "观测样本显示该空间可见 " + "、".join(labels) + "。"
        return ""

    def _top_object_labels(self, scene_summaries: Sequence[SceneSummary]) -> List[str]:
        objects: List[SceneObjectSummary] = []
        for summary in scene_summaries:
            objects.extend(summary.objects)
        objects.sort(key=lambda item: (-int(item.count), -float(item.max_score), item.label))
        return self._dedupe_strings(item.label for item in objects)

    def _extract_evidence_sample_ids(
        self,
        payload: Dict[str, object],
        observation_samples: Sequence[SceneObservationSample],
    ) -> Tuple[str, ...]:
        raw_ids = payload.get("evidence_observation_sample_ids") or payload.get("evidence_sample_ids") or payload.get("sample_ids")
        if isinstance(raw_ids, list):
            return tuple(self._dedupe_strings(raw_ids))
        return self._sample_ids(observation_samples)

    def _sample_ids(self, observation_samples: Sequence[SceneObservationSample]) -> Tuple[str, ...]:
        return tuple(self._dedupe_strings(sample.sample_id for sample in observation_samples if sample.sample_id))

    def _summary_confidence(self, scene_summaries: Sequence[SceneSummary]) -> float:
        scores = [float(obj.max_score) for summary in scene_summaries for obj in summary.objects]
        if not scores:
            return 0.5 if scene_summaries else 0.0
        return self._coerce_confidence(max(scores), default=0.5)

    def _coerce_confidence(self, value: object, *, default: float) -> float:
        try:
            confidence = float(value)
        except (TypeError, ValueError):
            confidence = float(default)
        return max(0.0, min(1.0, confidence))

    def _normalize_label(self, value: object) -> str:
        label = str(value or "").strip().lower()
        label = re.sub(r"[^0-9a-zA-Z\u4e00-\u9fff]+", "_", label)
        return label.strip("_") or "unknown"

    def _metadata_text(self, metadata: Dict[str, object], key: str) -> Optional[str]:
        value = str(metadata.get(key) or "").strip()
        return value or None

    def _dedupe_strings(self, values: Iterable[object]) -> List[str]:
        items: List[str] = []
        for raw_value in values:
            value = str(raw_value or "").strip()
            if value and value not in items:
                items.append(value)
        return items


def normalize_scene_space_understanding(
    model_output: Optional[object],
    *,
    observation_samples: Sequence[SceneObservationSample] = (),
    scene_summaries: Sequence[SceneSummary] = (),
    provider_metadata: Optional[Dict[str, object]] = None,
    availability: str = "available",
    fallback_reason: str = "",
) -> SceneSpaceUnderstandingResult:
    adapter = SceneSpaceUnderstandingAdapter()
    return adapter.normalize(
        model_output,
        observation_samples=observation_samples,
        scene_summaries=scene_summaries,
        provider_metadata=provider_metadata,
        availability=availability,
        fallback_reason=fallback_reason,
    )


def build_rule_based_scene_space_understanding(
    *,
    observation_samples: Sequence[SceneObservationSample] = (),
    scene_summaries: Sequence[SceneSummary] = (),
    fallback_reason: str = "provider_not_available",
    availability: str = "unavailable",
    provider_metadata: Optional[Dict[str, object]] = None,
) -> SceneSpaceUnderstandingResult:
    adapter = SceneSpaceUnderstandingAdapter()
    return adapter.build_rule_fallback(
        observation_samples=observation_samples,
        scene_summaries=scene_summaries,
        fallback_reason=fallback_reason,
        availability=availability,
        provider_metadata=provider_metadata,
    )
