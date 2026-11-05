"""关键帧对既有场景空间的采样与理解；不改变语义建图开关。"""
from dataclasses import replace

from contracts.base import utc_now
from contracts.spatial_memory import SceneSpaceReviewStatus
from services.memory.semantic_map.scene_space_sampler import SceneSpaceSampler
from services.memory.semantic_map.scene_space_understanding import SceneSpaceUnderstandingAdapter


def provider_metadata(perception_context):
    summary = getattr(perception_context, "scene_summary", None)
    metadata = dict(getattr(summary, "metadata", {}) or {})
    provider = metadata.get("scene_provider") or metadata.get("provider")
    if isinstance(provider, dict):
        metadata = {**provider, **metadata}
    metadata.setdefault("backend_name", str(metadata.get("scene_backend") or metadata.get("resolved_backend") or "perception_scene_summary"))
    metadata.setdefault("provider_name", str(metadata.get("provider_name") or metadata.get("resolved_provider_name") or "perception_scene_summary"))
    model = str(metadata.get("model_name") or metadata.get("cloud_vision_model") or "").strip()
    if model:
        metadata["model_name"] = model
    return metadata


def understand_space(scene_space, *, model_output, observation_samples, scene_summaries, provider_metadata,
                     library_name, availability="available", fallback_reason=""):
    adapter = SceneSpaceUnderstandingAdapter()
    if model_output is None:
        understanding = adapter.build_rule_fallback(observation_samples=observation_samples, scene_summaries=scene_summaries,
            fallback_reason=fallback_reason or "provider_not_available",
            availability=availability if availability != "available" else "unavailable", provider_metadata=provider_metadata)
    else:
        understanding = adapter.normalize(model_output, observation_samples=observation_samples, scene_summaries=scene_summaries,
            provider_metadata=provider_metadata, availability=availability, fallback_reason=fallback_reason)
    locked = {SceneSpaceReviewStatus.REVIEWED, SceneSpaceReviewStatus.CORRECTED, SceneSpaceReviewStatus.REJECTED}
    labels = list(understanding.candidate_labels)
    primary_label, review_status = scene_space.primary_label, scene_space.review_status
    if review_status not in locked:
        primary_label = labels[0].label if labels else None
        review_status = SceneSpaceReviewStatus.AUTO_LABELED if labels else SceneSpaceReviewStatus.CANDIDATE
    attributes = {**scene_space.attributes, "scene_space_understanding": {
        "adapter_version": str(understanding.model_scene_summary.metadata.get("adapter_version") or ""),
        "evidence_sample_ids": list(understanding.evidence_sample_ids), "candidate_label_count": len(labels)}}
    metadata = {**scene_space.metadata, "memory_library_name": library_name,
        "availability": understanding.availability, "fallback_reason": understanding.fallback_reason,
        "scene_space_understanding": {"availability": understanding.availability, "fallback_reason": understanding.fallback_reason,
            "provider": dict(understanding.provider_metadata), "evidence_sample_ids": list(understanding.evidence_sample_ids),
            "confidence": round(float(understanding.confidence), 6)}}
    return scene_space.model_copy(update={"observation_sample_ids": list(understanding.evidence_sample_ids),
        "observation_samples": list(observation_samples), "model_summary": understanding.model_scene_summary,
        "candidate_labels": labels, "primary_label": primary_label, "review_status": review_status,
        "semantic_updated_at": utc_now(), "attributes": attributes, "metadata": metadata}, deep=False)


def refresh_scene_spaces(store, *, perception_context, observation_events, context):
    """由记忆写线程调用，只更新context中已有的业务区域。"""
    summary = getattr(perception_context, "scene_summary", None)
    if summary is None or not observation_events or not context.scene_spaces:
        return context
    events = store.model_map("observation").values()
    instances = store.model_map("instance").values()
    current = store.model_map("scene_space")
    new_event_ids = {event.event_id for event in observation_events}
    metadata = provider_metadata(perception_context)
    availability = str(metadata.get("availability") or "available")
    reason = str(metadata.get("fallback_reason") or metadata.get("reason") or "")
    sampler = SceneSpaceSampler()
    updated = {}
    for scene in context.scene_spaces:
        space = current.get(scene.scene_space_id)
        if space is None or space.map_name != store.name:
            continue
        sampling = sampler.sample(space, observation_events=events, semantic_instances=instances)
        if not sampling.samples or not any(sample.observation_event_id in new_event_ids
                                          for sample in sampling.samples if sample.observation_event_id):
            continue
        value = understand_space(sampling.scene_space, model_output=summary, observation_samples=sampling.samples,
            scene_summaries=(summary,), provider_metadata=metadata, availability=availability,
            fallback_reason=reason, library_name=store.name)
        if space.metadata.get("scene_mapping_managed"):
            value.metadata["scene_revision"] = int(space.metadata.get("scene_revision", 0)) + 1
        updated[value.scene_space_id] = store.put("scene_space", value)
    return replace(context, scene_spaces=tuple(updated.get(space.scene_space_id, space) for space in context.scene_spaces))
