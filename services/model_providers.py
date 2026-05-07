from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Optional


PROVIDER_KINDS = (
    "local",
    "cloud",
    "hybrid",
    "disabled",
    "passthrough",
    "bbox_only_fallback",
    "primary",
    "low_latency",
    "unsupported",
    "custom",
)
PROVIDER_CAPABILITIES = (
    "text_embedding",
    "vision_language_embedding",
    "vlm",
    "grounding",
    "segmentation",
    "detection",
)
PROVIDER_AVAILABILITIES = ("available", "disabled", "not_implemented", "unsupported", "unavailable")
_PROVIDER_CAPABILITY_KEYS = ("ground", "segment", "understand", "embedText", "embedImage", "detect")


def _capabilities(**values: bool) -> Dict[str, bool]:
    return {key: bool(values.get(key, False)) for key in _PROVIDER_CAPABILITY_KEYS}


def _normalized_capabilities(payload: object) -> Dict[str, bool]:
    if not isinstance(payload, dict):
        return _capabilities()
    return {key: bool(payload.get(key, False)) for key in _PROVIDER_CAPABILITY_KEYS}


def _default_availability(provider_kind: str) -> str:
    if provider_kind == "disabled":
        return "disabled"
    if provider_kind in {"unsupported", "custom"}:
        return "unsupported"
    return "available"


def _safe_availability(value: object, provider_kind: str) -> str:
    normalized = str(value or "").strip()
    if normalized in PROVIDER_AVAILABILITIES:
        return normalized
    return _default_availability(provider_kind)


@dataclass(frozen=True)
class ModelProviderSpec:
    """统一模型 provider 描述。"""

    capability: str
    provider_kind: str
    provider_name: str
    model_name: str = ""
    backend_name: str = ""
    metadata: Dict[str, object] = field(default_factory=dict)

    def is_enabled(self) -> bool:
        return self.provider_kind not in {"disabled", "off", "unsupported"}

    def to_metadata(self) -> Dict[str, object]:
        metadata = dict(self.metadata)
        fallback_reason = str(metadata.get("fallback_reason") or metadata.get("reason") or "")
        fallback_default = bool(fallback_reason) and self.provider_kind not in {"disabled", "unsupported", "custom"}
        metadata["capabilities"] = _normalized_capabilities(metadata.get("capabilities"))
        metadata["availability"] = _safe_availability(metadata.get("availability"), self.provider_kind)
        metadata["fallback_used"] = bool(metadata.get("fallback_used", fallback_default))
        metadata["fallback_reason"] = fallback_reason
        metadata.setdefault("requested_provider_name", self.provider_name)
        metadata.setdefault("resolved_provider_name", self.provider_name)
        metadata.setdefault("requested_backend", self.backend_name)
        metadata.setdefault("resolved_backend", self.backend_name)
        return {
            "capability": self.capability,
            "provider_kind": self.provider_kind,
            "provider_name": self.provider_name,
            "model_name": self.model_name,
            "backend_name": self.backend_name,
            **metadata,
        }


def resolve_text_embedding_provider_spec(model_name: str) -> ModelProviderSpec:
    normalized = str(model_name or "").strip()
    lowered = normalized.lower()
    if not normalized or lowered.startswith("hashing"):
        return ModelProviderSpec(
            capability="text_embedding",
            provider_kind="local",
            provider_name="hashing",
            model_name=normalized or "hashing-v1",
            metadata={"capabilities": _capabilities(embedText=True)},
        )
    if _is_dashscope_multimodal_model(normalized):
        return ModelProviderSpec(
            capability="text_embedding",
            provider_kind="cloud",
            provider_name="dashscope",
            model_name=normalized,
            metadata={
                "supports_fused_text_image": True,
                "capabilities": _capabilities(embedText=True, embedImage=True),
            },
        )
    if lowered.startswith("sentence-transformers:") or "bge" in lowered or _looks_like_local_model_reference(normalized):
        return ModelProviderSpec(
            capability="text_embedding",
            provider_kind="local",
            provider_name="sentence_transformers",
            model_name=normalized,
            metadata={"capabilities": _capabilities(embedText=True)},
        )
    return ModelProviderSpec(
        capability="text_embedding",
        provider_kind="custom",
        provider_name="custom_text_embedding",
        model_name=normalized,
        metadata={
            "availability": "unsupported",
            "fallback_reason": "unsupported_provider",
            "requested_provider_name": normalized,
            "resolved_provider_name": "custom_text_embedding",
            "capabilities": _capabilities(embedText=True),
        },
    )


def resolve_vision_language_provider_spec(model_name: str) -> ModelProviderSpec:
    normalized = str(model_name or "").strip()
    lowered = normalized.lower()
    if not normalized or lowered in {"disabled", "none", "off"}:
        return ModelProviderSpec(
            capability="vision_language_embedding",
            provider_kind="disabled",
            provider_name="disabled",
            model_name=normalized,
            metadata={
                "fallback_reason": "disabled_by_config" if lowered in {"none", "off"} else "",
                "requested_provider_name": normalized or "disabled",
                "resolved_provider_name": "disabled",
                "capabilities": _capabilities(),
            },
        )
    if _is_dashscope_multimodal_model(normalized):
        return ModelProviderSpec(
            capability="vision_language_embedding",
            provider_kind="cloud",
            provider_name="dashscope",
            model_name=normalized,
            metadata={
                "supports_text": True,
                "supports_image": True,
                "supports_fused_text_image": True,
                "capabilities": _capabilities(embedText=True, embedImage=True),
            },
        )
    if lowered.startswith("transformers-clip:") or "clip" in lowered or _looks_like_local_model_reference(normalized):
        return ModelProviderSpec(
            capability="vision_language_embedding",
            provider_kind="local",
            provider_name="clip",
            model_name=normalized,
            metadata={
                "supports_text": True,
                "supports_image": True,
                "capabilities": _capabilities(embedText=True, embedImage=True),
            },
        )
    return ModelProviderSpec(
        capability="vision_language_embedding",
        provider_kind="custom",
        provider_name="custom_vision_language",
        model_name=normalized,
        metadata={
            "availability": "unsupported",
            "fallback_reason": "unsupported_provider",
            "requested_provider_name": normalized,
            "resolved_provider_name": "custom_vision_language",
            "capabilities": _capabilities(embedText=True, embedImage=True),
        },
    )


def resolve_scene_semantic_provider_spec(
    default_backend_name: str,
    *,
    cloud_enabled: bool,
    cloud_model_name: str,
    local_backend_name: str = "simple_scene",
    cloud_backend_name: str = "openai_vision",
) -> ModelProviderSpec:
    normalized = str(default_backend_name or "").strip().lower()
    if normalized in {"simple_scene", "local_scene", "rule_scene"}:
        return ModelProviderSpec(
            capability="vlm",
            provider_kind="local",
            provider_name="rule_based_scene",
            backend_name=local_backend_name,
            metadata={"capabilities": _capabilities(understand=True)},
        )
    if normalized in {"openai_scene", "cloud_scene", "openai_vision", "cloud"}:
        if cloud_enabled:
            return ModelProviderSpec(
                capability="vlm",
                provider_kind="cloud",
                provider_name="openai_compatible",
                model_name=str(cloud_model_name or "").strip(),
                backend_name=cloud_backend_name,
                metadata={"capabilities": _capabilities(understand=True)},
            )
        return ModelProviderSpec(
            capability="vlm",
            provider_kind="local",
            provider_name="rule_based_scene",
            backend_name=local_backend_name,
            metadata={
                "requested_provider_name": normalized or "cloud_scene",
                "resolved_provider_name": "rule_based_scene",
                "requested_backend": normalized or "cloud_scene",
                "resolved_backend": local_backend_name,
                "fallback_used": True,
                "fallback_reason": "cloud_disabled",
                "capabilities": _capabilities(understand=True),
            },
        )
    if cloud_enabled:
        return ModelProviderSpec(
            capability="vlm",
            provider_kind="hybrid",
            provider_name="hybrid_scene",
            model_name=str(cloud_model_name or "").strip(),
            backend_name=normalized or "hybrid_scene",
            metadata={
                "local_backend": local_backend_name,
                "cloud_backend": cloud_backend_name,
                "capabilities": _capabilities(understand=True),
            },
        )
    return ModelProviderSpec(
        capability="vlm",
        provider_kind="local",
        provider_name="rule_based_scene",
        backend_name=local_backend_name,
        metadata={
            "requested_provider_name": normalized or "hybrid_scene",
            "resolved_provider_name": "rule_based_scene",
            "requested_backend": normalized or "hybrid_scene",
            "resolved_backend": local_backend_name,
            "fallback_used": True,
            "fallback_reason": "cloud_disabled",
            "capabilities": _capabilities(understand=True),
        },
    )


def resolve_grounding_provider_spec() -> ModelProviderSpec:
    return ModelProviderSpec(
        capability="grounding",
        provider_kind="local",
        provider_name="rule_based_query_planner",
        backend_name="grounding_query_planner",
        metadata={"capabilities": _capabilities(ground=True)},
    )


def resolve_detection_provider_spec(
    *,
    enabled: bool = True,
    backend_name: str = "",
    backend_kind: str = "metadata",
    model_name: str = "",
) -> ModelProviderSpec:
    requested_kind = str(backend_kind or "").strip().lower() or "metadata"
    requested_backend = str(backend_name or "").strip() or requested_kind
    normalized_model = str(model_name or "").strip()
    base_metadata: Dict[str, object] = {
        "requested_provider_name": requested_kind,
        "requested_backend": requested_backend,
        "requested_backend_kind": requested_kind,
        "capabilities": _capabilities(),
    }
    if not enabled or requested_kind in {"disabled", "off", "none"}:
        return ModelProviderSpec(
            capability="detection",
            provider_kind="disabled",
            provider_name="disabled",
            model_name=normalized_model,
            backend_name=requested_backend,
            metadata={
                **base_metadata,
                "availability": "disabled",
                "fallback_reason": "disabled_by_config",
                "resolved_provider_name": "disabled",
                "resolved_backend": requested_backend,
            },
        )
    if requested_kind in {"metadata", "metadata_detector"}:
        return ModelProviderSpec(
            capability="detection",
            provider_kind="local",
            provider_name="metadata_detector",
            model_name=normalized_model,
            backend_name=requested_backend,
            metadata={
                **base_metadata,
                "resolved_provider_name": "metadata_detector",
                "resolved_backend": requested_backend,
                "capabilities": _capabilities(detect=True),
            },
        )
    if "yolo_world" in requested_kind or requested_kind in {"yoloworld", "ultralytics_yolo_world"}:
        return ModelProviderSpec(
            capability="detection",
            provider_kind="local",
            provider_name="yolo_world",
            model_name=normalized_model,
            backend_name=requested_backend,
            metadata={
                **base_metadata,
                "resolved_provider_name": "yolo_world",
                "resolved_backend": requested_backend,
                "open_vocabulary": True,
                "capabilities": _capabilities(ground=True, detect=True),
            },
        )
    if "yolo" in requested_kind or requested_kind in {"ultralytics", "ultralytics_yolo_tensorrt", "ultralytics_yolo_pytorch"}:
        return ModelProviderSpec(
            capability="detection",
            provider_kind="local",
            provider_name="ultralytics_yolo",
            model_name=normalized_model,
            backend_name=requested_backend,
            metadata={
                **base_metadata,
                "resolved_provider_name": "ultralytics_yolo",
                "resolved_backend": requested_backend,
                "capabilities": _capabilities(detect=True),
            },
        )
    return ModelProviderSpec(
        capability="detection",
        provider_kind="unsupported",
        provider_name=requested_kind,
        model_name=normalized_model,
        backend_name=requested_backend,
        metadata={
            **base_metadata,
            "availability": "unsupported",
            "fallback_reason": "unsupported_provider",
            "resolved_provider_name": requested_kind,
            "resolved_backend": requested_backend,
        },
    )


def resolve_segmentation_provider_spec(
    *,
    enabled: bool,
    backend_name: str = "",
    backend_kind: str = "disabled_segmentation",
    route: str = "primary",
    default_backend_name: str = "segmentation_disabled",
    primary_backend_name: str = "groundingdino_sam2",
    low_latency_backend_name: str = "yolo_world_sam2",
    allow_bbox_only_fallback: bool = True,
    quality_thresholds: Optional[Dict[str, float]] = None,
    output_policy: Optional[Dict[str, object]] = None,
) -> ModelProviderSpec:
    resolved_route = str(route or "").strip().lower() or "primary"
    requested_kind = str(backend_kind or "").strip().lower() or "disabled_segmentation"
    requested_backend = str(backend_name or "").strip() or str(default_backend_name or "").strip() or "segmentation_disabled"
    thresholds = dict(quality_thresholds or {})
    policy = dict(output_policy or {})
    metadata: Dict[str, object] = {
        "optional": True,
        "route": resolved_route,
        "requested_provider_name": requested_kind,
        "route_provider_name": resolved_route,
        "requested_backend": requested_backend,
        "requested_backend_kind": requested_kind,
        "primary_backend_name": str(primary_backend_name or "groundingdino_sam2"),
        "low_latency_backend_name": str(low_latency_backend_name or "yolo_world_sam2"),
        "allow_bbox_only_fallback": bool(allow_bbox_only_fallback),
        "quality_thresholds": thresholds,
        "output_policy": policy,
        "capabilities": _capabilities(),
    }

    if not enabled or resolved_route in {"disabled", "off", "none"}:
        return ModelProviderSpec(
            capability="segmentation",
            provider_kind="disabled",
            provider_name="disabled",
            backend_name=requested_backend,
            metadata={
                **metadata,
                "availability": "disabled",
                "fallback_reason": "disabled_by_config",
                "reason": "disabled_by_config",
                "resolved_provider_name": "disabled",
                "resolved_backend": requested_backend,
            },
        )

    if requested_kind in {"detection_attributes", "detection_attributes_segmentation", "passthrough"} or resolved_route in {
        "passthrough",
        "detection_attributes",
        "detection_attributes_segmentation",
    }:
        return ModelProviderSpec(
            capability="segmentation",
            provider_kind="passthrough",
            provider_name="detection_attributes",
            backend_name=requested_backend,
            metadata={
                **metadata,
                "source": "detection_attributes",
                "fallback_reason": "reuse_detector_segmentation_attributes",
                "reason": "reuse_detector_segmentation_attributes",
                "resolved_provider_name": "detection_attributes",
                "resolved_backend": requested_backend,
                "capabilities": _capabilities(segment=True),
            },
        )

    if resolved_route in {"bbox_only", "bbox_only_fallback"}:
        return ModelProviderSpec(
            capability="segmentation",
            provider_kind="bbox_only_fallback",
            provider_name="bbox_only_fallback",
            backend_name=requested_backend,
            metadata={
                **metadata,
                "source": "bbox_only_fallback",
                "fallback_reason": "bbox_only_fallback_requested",
                "reason": "bbox_only_fallback_requested",
                "resolved_provider_name": "bbox_only_fallback",
                "resolved_backend": requested_backend,
                "segment_capability_available": False,
            },
        )

    if resolved_route in {"primary", "default", "main"}:
        provider_name = str(primary_backend_name or "groundingdino_sam2").strip() or "groundingdino_sam2"
        return ModelProviderSpec(
            capability="segmentation",
            provider_kind="primary",
            provider_name=provider_name,
            backend_name=requested_backend or provider_name,
            metadata={
                **metadata,
                "route": "primary",
                "grounding_provider": "groundingdino",
                "segmentation_provider": "sam2",
                "capabilities": _capabilities(ground=True, segment=True),
                "availability": "not_implemented",
                "fallback_reason": "provider_not_available",
                "reason": "not_implemented",
                "resolved_provider_name": provider_name,
                "resolved_backend": requested_backend or provider_name,
            },
        )

    if resolved_route in {"low_latency", "low-latency", "fast"}:
        provider_name = str(low_latency_backend_name or "yolo_world_sam2").strip() or "yolo_world_sam2"
        return ModelProviderSpec(
            capability="segmentation",
            provider_kind="low_latency",
            provider_name=provider_name,
            backend_name=requested_backend or provider_name,
            metadata={
                **metadata,
                "route": "low_latency",
                "grounding_provider": "yolo_world",
                "segmentation_provider": "sam2",
                "capabilities": _capabilities(ground=True, segment=True),
                "availability": "not_implemented",
                "fallback_reason": "provider_not_available",
                "reason": "not_implemented",
                "resolved_provider_name": provider_name,
                "resolved_backend": requested_backend or provider_name,
            },
        )

    return ModelProviderSpec(
        capability="segmentation",
        provider_kind="unsupported",
        provider_name=requested_kind or resolved_route,
        backend_name=requested_backend,
        metadata={
            **metadata,
            "availability": "unsupported",
            "fallback_reason": "unsupported_segmentation_route",
            "reason": "unsupported_segmentation_route",
            "resolved_provider_name": requested_kind or resolved_route,
            "resolved_backend": requested_backend,
        },
    )


def normalize_scene_space_vlm_metadata(
    metadata: Optional[Dict[str, object]] = None,
    *,
    availability: str = "available",
    fallback_reason: str = "",
) -> Dict[str, object]:
    payload = dict(metadata or {})
    nested_provider = payload.get("provider") or payload.get("scene_provider")
    if isinstance(nested_provider, dict):
        payload = {**dict(nested_provider), **payload}
        payload.pop("provider", None)
        payload.pop("scene_provider", None)

    requested_provider = str(
        payload.get("requested_provider_name")
        or payload.get("provider_name")
        or payload.get("provider")
        or "scene_space_vlm"
    ).strip()
    resolved_provider = str(
        payload.get("resolved_provider_name")
        or payload.get("provider_name")
        or requested_provider
        or "scene_space_vlm"
    ).strip()
    requested_backend = str(payload.get("requested_backend") or payload.get("backend_name") or "scene_space_vlm").strip()
    resolved_backend = str(payload.get("resolved_backend") or payload.get("backend_name") or requested_backend).strip()
    model_name = str(payload.get("model_name") or payload.get("model") or "").strip()
    provider_kind = str(payload.get("provider_kind") or "cloud").strip().lower()
    resolved_availability = _safe_availability(payload.get("availability") or availability, provider_kind)
    resolved_fallback_reason = str(
        fallback_reason
        or payload.get("fallback_reason")
        or payload.get("reason")
        or ""
    ).strip()
    fallback_used = bool(payload.get("fallback_used", bool(resolved_fallback_reason) or resolved_availability != "available"))
    capabilities = _normalized_capabilities(payload.get("capabilities"))
    capabilities["understand"] = True

    return {
        **payload,
        "capability": "vlm",
        "scene_space_capability": "scene_space_understanding",
        "provider_kind": provider_kind,
        "provider_name": resolved_provider,
        "model_name": model_name,
        "backend_name": resolved_backend,
        "requested_provider_name": requested_provider or resolved_provider,
        "resolved_provider_name": resolved_provider,
        "requested_backend": requested_backend or resolved_backend,
        "resolved_backend": resolved_backend,
        "availability": resolved_availability,
        "fallback_used": fallback_used,
        "fallback_reason": resolved_fallback_reason,
        "capabilities": capabilities,
    }


def resolve_scene_space_rule_fallback_provider_metadata(
    *,
    fallback_reason: str = "provider_not_available",
    availability: str = "unavailable",
    requested_metadata: Optional[Dict[str, object]] = None,
) -> Dict[str, object]:
    requested = dict(requested_metadata or {})
    requested_provider = str(
        requested.get("requested_provider_name")
        or requested.get("provider_name")
        or requested.get("resolved_provider_name")
        or "scene_space_vlm"
    ).strip()
    requested_backend = str(
        requested.get("requested_backend")
        or requested.get("backend_name")
        or requested.get("resolved_backend")
        or "scene_space_vlm"
    ).strip()
    metadata = {
        **requested,
        "capability": "vlm",
        "scene_space_capability": "scene_space_understanding",
        "provider_kind": "local",
        "provider_name": "rule_based_scene_space",
        "model_name": "rule-based-scene-space-v1",
        "backend_name": "rule_based_scene_space",
        "requested_provider_name": requested_provider,
        "resolved_provider_name": "rule_based_scene_space",
        "requested_backend": requested_backend,
        "resolved_backend": "rule_based_scene_space",
        "availability": _safe_availability(availability, "local"),
        "fallback_used": True,
        "fallback_reason": str(fallback_reason or "provider_not_available"),
        "capabilities": _capabilities(understand=True),
    }
    return metadata

def _looks_like_local_model_reference(model_name: str) -> bool:
    normalized = str(model_name or "").strip()
    if not normalized:
        return False
    candidate = Path(normalized).expanduser()
    if candidate.is_absolute():
        return True
    if normalized.startswith("./") or normalized.startswith("../"):
        return True
    return candidate.exists()


def _is_dashscope_multimodal_model(model_name: str) -> bool:
    normalized = str(model_name or "").strip().lower()
    if normalized.startswith("dashscope:"):
        normalized = normalized.split(":", 1)[1].strip()
    return normalized.startswith("tongyi-embedding-vision") or normalized.startswith("qwen3-vl-embedding") or normalized.startswith("qwen2.5-vl-embedding") or normalized.startswith("multimodal-embedding-v1")
