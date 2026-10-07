#!/usr/bin/env python3
"""离线验证实例展示的只读生命周期投影，不调用外部模型、网络或硬件。"""
from __future__ import annotations

import ast
import copy
import math
import sys
import unittest
from contextlib import contextmanager
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Dict, List
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from contracts.geometry_runtime import DEFAULT_MAP_FRAME
from contracts.spatial_memory import (
    DeferredProjectionInput,
    InstanceDynamicState,
    InstanceLifecycleState,
    InstanceMovability,
    InstanceObservationWindowSample,
    SemanticInstance,
)
from scripts import check_memory_durable_integration as integration
from scripts.check_memory_storage_transactions import NOW, records
from services.memory.instance_association_service import InstanceAssociationService
from services.memory.service import MemoryService


# 以下两个原始实现逐字取自01763b67；保留为独立行为对照，不调用新增投影帮助函数。
def _original_mark_stale(self, instance: SemanticInstance, *, now: datetime) -> SemanticInstance:
    age_sec = max(0.0, (now - instance.last_seen_ts).total_seconds())
    lifecycle_state = InstanceLifecycleState.ACTIVE
    removed_after_sec = self._REMOVED_AFTER_SEC.get(instance.movability, 7 * 24 * 3600.0)
    stale_after_sec = self._STALE_AFTER_SEC.get(instance.movability, 24 * 3600.0)
    uncertain_after_sec = self._UNCERTAIN_AFTER_SEC.get(instance.movability, 3600.0)
    if age_sec >= removed_after_sec:
        lifecycle_state = InstanceLifecycleState.REMOVED
    elif age_sec >= stale_after_sec:
        lifecycle_state = InstanceLifecycleState.STALE
    elif age_sec >= uncertain_after_sec:
        lifecycle_state = InstanceLifecycleState.UNCERTAIN
    # 阶段0：待定位实例(坐标尚不可信)保持 UNCERTAIN，不因年龄年轻被提升为 ACTIVE。
    if (instance.metadata or {}).get("pending_localization") is True:  # noqa: SIM102 - 冻结原始对照实现
        if lifecycle_state == InstanceLifecycleState.ACTIVE:
            lifecycle_state = InstanceLifecycleState.UNCERTAIN
    if lifecycle_state == instance.lifecycle_state:
        return instance
    return instance.model_copy(
        update={
            "lifecycle_state": lifecycle_state,
            "metadata": {
                **dict(instance.metadata),
                "association_policy": instance.movability.value,
                "uncertain_after_sec": uncertain_after_sec,
                "stale_after_sec": stale_after_sec,
                "removed_after_sec": removed_after_sec,
            },
        },
        deep=True,
    )


def _original_semantic_instance_rows(self, instances, target_library, resolved_now, *, label=None):
    label_lower = str(label).lower().strip() if label else None
    lifecycle_order = {
        InstanceLifecycleState.ACTIVE: 0,
        InstanceLifecycleState.UNCERTAIN: 1,
        InstanceLifecycleState.STALE: 2,
        InstanceLifecycleState.REMOVED: 3,
    }
    result: List[Dict[str, object]] = []
    for inst in instances:
        if label_lower is not None and str(inst.label).lower().strip() != label_lower:
            continue
        pose = inst.pose
        frame_id = pose.frame_id if pose else ""
        pos = pose.position if pose else None
        valid_position = pos is not None and all(math.isfinite(value) for value in (pos.x, pos.y, pos.z))
        map_matches_library = str(inst.map_name or "").strip() == target_library
        metadata = dict(inst.metadata or {})
        projection_method = str(metadata.get("projection_method") or "").strip()
        observation_only = (
            self._coerce_optional_bool(metadata.get("observation_only")) is True
            or projection_method == "current_pose_fallback"
        )
        is_fallback = (
            observation_only or not valid_position or frame_id != DEFAULT_MAP_FRAME
            or projection_method.lower() in {"", "unknown", "none"} or not map_matches_library
        )
        projection_confidence = self._coerce_optional_float(metadata.get("projection_confidence"))
        if projection_confidence is not None and not math.isfinite(projection_confidence):
            projection_confidence = None
        dynamic_state = self._instance_dynamic_state(inst)
        lifecycle_source = inst
        if inst.last_seen_ts.tzinfo is None:
            lifecycle_source = inst.model_copy(
                update={"last_seen_ts": inst.last_seen_ts.replace(tzinfo=resolved_now.tzinfo)},
            )
        aged_instance = self._instance_association_service._mark_stale(lifecycle_source, now=resolved_now)
        # 展示时复用关联老化规则，只读派生；不能重新激活已明确失效的记录。
        lifecycle_state = max(
            (inst.lifecycle_state, aged_instance.lifecycle_state), key=lifecycle_order.__getitem__,
        ).value
        navigable = (
            self._coerce_optional_bool(metadata.get("navigable")) is True
            and not is_fallback
            and lifecycle_state == InstanceLifecycleState.ACTIVE.value
            and self._is_long_term_static_instance(inst)
        )
        non_navigable_reason = str(metadata.get("non_navigable_reason") or "")
        if not navigable and not non_navigable_reason:
            if not valid_position:
                non_navigable_reason = "invalid_object_position"
            elif not map_matches_library:
                non_navigable_reason = "map_binding_mismatch"
            elif observation_only:
                non_navigable_reason = "observation_only"
            elif is_fallback:
                non_navigable_reason = "object_position_unverified"
            elif lifecycle_state != InstanceLifecycleState.ACTIVE.value:
                non_navigable_reason = "instance_not_active"
            elif not self._is_long_term_static_instance(inst):
                non_navigable_reason = "not_long_term_static_target"
            else:
                non_navigable_reason = "navigation_unverified"
        result.append(
            {
                "instance_id": inst.instance_id,
                "label": inst.label,
                "display_name": inst.display_name or inst.label,
                "pose": {"x": pos.x, "y": pos.y, "z": pos.z} if valid_position else None,
                "frame_id": frame_id,
                "confidence": inst.confidence if math.isfinite(inst.confidence) else None,
                "instance_type": inst.instance_type,
                "last_seen_ts": lifecycle_source.last_seen_ts.isoformat(),
                "is_fallback": is_fallback,
                "map_name": inst.map_name,
                "library_name": target_library,
                "observation_only": observation_only,
                "navigable": navigable,
                "non_navigable_reason": non_navigable_reason if not navigable else "",
                "projection_method": projection_method,
                "projection_confidence": projection_confidence,
                "support_point_count": self._coerce_optional_int(metadata.get("support_point_count")) or 0,
                "observation_count": inst.observation_count,
                "lifecycle_state": lifecycle_state,
                "dynamic_state": dynamic_state.value,
            }
        )
    return result


@contextmanager
def original_rendering():
    with patch.object(InstanceAssociationService, "_mark_stale", _original_mark_stale), \
            patch.object(MemoryService, "_semantic_instance_rows", _original_semantic_instance_rows):
        yield


@contextmanager
def copy_counts():
    calls = []
    original = SemanticInstance.model_copy

    def capture(instance, *args, **kwargs):
        calls.append({"instance_id": instance.instance_id, "deep": kwargs.get("deep", False)})
        return original(instance, *args, **kwargs)

    with patch.object(SemanticInstance, "model_copy", capture):
        yield calls


class LifecycleProjectionChecks(unittest.TestCase):
    fixture = integration.IntegrationChecks.fixture

    def instance(self, *, index=0, age=0, state=InstanceLifecycleState.ACTIVE,
                 movability=InstanceMovability.MOVABLE, pending=None, naive=False):
        base = records()[2][3]
        timestamp = NOW - timedelta(seconds=age)
        if naive:
            timestamp = timestamp.replace(tzinfo=None)
        metadata = {"memory_library_name": "library", "nested": {"labels": ["original"]},
                    "projection_method": "depth", "navigable": True}
        if pending is not None:
            metadata["pending_localization"] = pending
        return base.model_copy(update={
            "instance_id": f"ins_lifecycle_20260101T000000Z_{index:08x}",
            "anchor_id": f"anc_lifecycle_20260101T000000Z_{index:08x}",
            "label": "cabinet" if index % 2 == 0 else "table",
            "map_name": "library", "last_seen_ts": timestamp, "lifecycle_state": state,
            "movability": movability, "dynamic_state": InstanceDynamicState.STATIC, "metadata": metadata,
            "attributes": {"nested": {"values": [1, 2, 3]}},
        }, deep=True)

    def policy(self):
        return InstanceAssociationService(text_embedder=None)

    def public_fixture(self):
        service = self.fixture()
        instances = [
            self.instance(index=0, age=0),
            self.instance(index=1, age=2 * 3600),
            self.instance(index=2, age=7 * 86400),
            self.instance(index=3, age=0, state=InstanceLifecycleState.STALE),
            self.instance(index=4, age=0, pending=True),
            self.instance(index=5, age=15 * 60, movability=InstanceMovability.TRANSIENT),
            self.instance(index=6, age=0, naive=True),
            self.instance(index=7, age=86400, naive=True),
        ]
        instances[0].last_seen_ts = NOW.astimezone(timezone(timedelta(hours=5, minutes=30)))
        for instance in instances[:-1]:
            service._repository.upsert_semantic_instance(instance)
        live = instances[-1]
        object.__setattr__(live, "_memory_live_only", True)
        service._semantic_instances_by_id[live.instance_id] = live
        return service

    def test_boundary_matrix_matches_original_full_model_and_projection(self):
        policy = self.policy()
        cases = 0
        for movability in InstanceMovability:
            thresholds = [policy._UNCERTAIN_AFTER_SEC[movability], policy._STALE_AFTER_SEC[movability],
                          policy._REMOVED_AFTER_SEC[movability]]
            ages = [-1, 0] + [boundary + delta for boundary in thresholds for delta in (-.000001, 0, .000001)]
            for state in InstanceLifecycleState:
                for pending in (None, False, True, 1, "true"):
                    for age in ages:
                        instance = self.instance(age=age, state=state, movability=movability, pending=pending)
                        before = copy.deepcopy(instance.model_dump())
                        expected = _original_mark_stale(policy, instance, now=NOW)
                        projection = policy._lifecycle_values(instance, now=NOW)
                        actual = policy._mark_stale(instance, now=NOW)
                        self.assertEqual(projection[0], expected.lifecycle_state)
                        self.assertEqual(projection[1:], tuple(thresholds))
                        self.assertEqual(actual.model_dump(), expected.model_dump())
                        self.assertEqual(actual is instance, expected is instance)
                        self.assertEqual(instance.model_dump(), before)
                        cases += 1
        self.assertEqual(cases, 660)

    def test_full_mark_stale_preserves_identity_and_nested_copy_relationships(self):
        policy = self.policy()
        for age in (0, 7 * 86400):
            instance = self.instance(age=age)
            expected = _original_mark_stale(policy, instance, now=NOW)
            actual = policy._mark_stale(instance, now=NOW)
            self.assertEqual(actual.model_dump(), expected.model_dump())
            for field in ("pose", "metadata", "attributes", "observation_window", "deferred_projection_cache"):
                self.assertEqual(getattr(actual, field) is getattr(instance, field),
                                 getattr(expected, field) is getattr(instance, field))
            self.assertEqual(actual.metadata["nested"] is instance.metadata["nested"],
                             expected.metadata["nested"] is instance.metadata["nested"])
            self.assertEqual(actual is instance, age == 0)

    def test_policy_lookup_order_defaults_and_captured_values_are_exact(self):
        instance = self.instance(age=10 * 86400, movability=InstanceMovability.STATIC)
        results = []
        for function in (_original_mark_stale, InstanceAssociationService._mark_stale):
            policy = self.policy()
            events = []

            class TrackedPolicy(dict):
                def __init__(self, label, value):
                    super().__init__(value)
                    self.label = label

                def get(self, key, default=None, events=events, policy=policy):
                    events.append((self.label, key, default))
                    # 首次读取后改变前一个映射，不得影响已捕获的移除阈值。
                    if self.label == "stale":
                        policy._REMOVED_AFTER_SEC[key] = 999999999
                    return super().get(key, default)

            for label, attr in (("removed", "_REMOVED_AFTER_SEC"), ("stale", "_STALE_AFTER_SEC"),
                                ("uncertain", "_UNCERTAIN_AFTER_SEC")):
                setattr(policy, attr, TrackedPolicy(label, {}))
            result = function(policy, instance, now=NOW)
            results.append((events, result.model_dump()))
            self.assertEqual([event[0] for event in events], ["removed", "stale", "uncertain"])
            self.assertEqual(result.metadata["removed_after_sec"], 7 * 86400.0)
            self.assertEqual(result.metadata["stale_after_sec"], 86400.0)
            self.assertEqual(result.metadata["uncertain_after_sec"], 3600.0)
        self.assertEqual(results[0], results[1])

    def test_pending_flag_is_boolean_identity_and_display_does_not_reactivate(self):
        service = self.fixture()
        rows = [self.instance(index=index, pending=pending) for index, pending in enumerate((None, False, True, 1, "true"))]
        for pending, row in zip((None, False, True, 1, "true"), rows):
            expected = InstanceLifecycleState.UNCERTAIN if pending is True else InstanceLifecycleState.ACTIVE
            self.assertEqual(service._instance_association_service._lifecycle_values(row, now=NOW)[0], expected)
        stored = self.instance(state=InstanceLifecycleState.REMOVED)
        self.assertEqual(service._instance_association_service._mark_stale(stored, now=NOW).lifecycle_state,
                         InstanceLifecycleState.ACTIVE)
        self.assertEqual(service._semantic_instance_rows([stored], "library", NOW)[0]["lifecycle_state"], "removed")

    def test_naive_and_offset_timezone_output_and_clock_sequence_match(self):
        service = self.public_fixture()
        operations = [service.list_semantic_instances, lambda: service.list_semantic_instances(label="CABINET"),
                      lambda: service.get_semantic_instances_snapshot(limit=5),
                      lambda: service.get_semantic_instances_snapshot(limit=1000)]
        for call in operations:
            outputs, clocks, copies = [], [], []
            for baseline in (True, False):
                events = []

                def now(events=events):
                    events.append("utc_now")
                    return NOW

                with patch("services.memory.service.utc_now", side_effect=now), copy_counts() as counts:
                    if baseline:
                        with original_rendering():
                            result = call()
                    else:
                        result = call()
                outputs.append(result)
                clocks.append(events)
                copies.append(counts)
            self.assertEqual(outputs[0], outputs[1])
            self.assertEqual(clocks[0], clocks[1])
            self.assertEqual(sum(not item["deep"] for item in copies[0]), sum(not item["deep"] for item in copies[1]))
            self.assertFalse(any(item["deep"] for item in copies[1]))
        rendered = service._semantic_instance_rows(list(service._semantic_instances_by_id.values()), "library", NOW)
        by_id = {row["instance_id"]: row for row in rendered}
        self.assertTrue(by_id["ins_lifecycle_20260101T000000Z_00000000"]["last_seen_ts"].endswith("+05:30"))
        self.assertTrue(by_id["ins_lifecycle_20260101T000000Z_00000006"]["last_seen_ts"].endswith("+00:00"))

    def test_direct_mark_stale_does_not_gain_timezone_normalization(self):
        policy = self.policy()
        instance = self.instance(naive=True)
        failures = []
        for function in (_original_mark_stale, InstanceAssociationService._mark_stale):
            with self.assertRaises(TypeError) as error:
                function(policy, instance, now=NOW)
            failures.append(str(error.exception))
        self.assertEqual(failures[0], failures[1])
        naive_now = NOW.replace(tzinfo=None)
        self.assertEqual(policy._mark_stale(instance, now=naive_now).model_dump(),
                         _original_mark_stale(policy, instance, now=naive_now).model_dump())

    def test_full_public_lists_and_snapshots_preserve_sources_filters_and_deadlines(self):
        service = self.public_fixture()
        sources = dict(service._semantic_instances_by_id)
        before = {key: copy.deepcopy(value.model_dump()) for key, value in sources.items()}
        operations = [lambda: service.list_semantic_instances(library_name="missing"),
                      lambda: service.list_semantic_instances(label="unmatched"),
                      lambda: service.get_semantic_instances_snapshot(limit=1),
                      lambda: service.get_semantic_instances_snapshot(limit=1000)]
        with patch("services.memory.service.utc_now", return_value=NOW):
            for call in operations:
                with original_rendering():
                    expected = call()
                actual = call()
                self.assertEqual(actual, expected)
            snapshot = service.get_semantic_instances_snapshot(limit=1000)
            self.assertEqual(snapshot["total"], 8)
            self.assertIsNotNone(snapshot["_next_update_at"])
            self.assertGreater(snapshot["_next_update_at"], NOW.timestamp())
        for key, instance in sources.items():
            self.assertIs(service._semantic_instances_by_id[key], instance)
            self.assertEqual(instance.model_dump(), before[key])

    def test_aged_public_fixture_removes_660_discarded_deepcopies(self):
        service = self.fixture()
        for index in range(32):
            service._repository.upsert_semantic_instance(self.instance(index=index, age=60 * 86400))
        live = self.instance(index=32, age=60 * 86400)
        object.__setattr__(live, "_memory_live_only", True)
        service._semantic_instances_by_id[live.instance_id] = live
        with patch("services.memory.service.utc_now", return_value=NOW):
            with original_rendering(), copy_counts() as baseline:
                expected = [service.list_semantic_instances() for _ in range(20)]
            with copy_counts() as candidate:
                actual = [service.list_semantic_instances() for _ in range(20)]
            self.assertEqual(actual, expected)
            self.assertEqual(len(baseline), 660)
            self.assertTrue(all(item["deep"] for item in baseline))
            self.assertEqual(candidate, [])
            with original_rendering(), copy_counts() as baseline_snapshot:
                expected_snapshot = service.get_semantic_instances_snapshot()
            with copy_counts() as candidate_snapshot:
                actual_snapshot = service.get_semantic_instances_snapshot()
            self.assertEqual(actual_snapshot, expected_snapshot)
            self.assertEqual(len(baseline_snapshot), 33)
            self.assertEqual(candidate_snapshot, [])

    def test_young_matching_state_avoids_copies_in_both_paths(self):
        service = self.fixture()
        instance = self.instance()
        service._semantic_instances_by_id[instance.instance_id] = instance
        with patch("services.memory.service.utc_now", return_value=NOW):
            with original_rendering(), copy_counts() as baseline:
                expected = service.get_semantic_instances_snapshot()
            with copy_counts() as candidate:
                actual = service.get_semantic_instances_snapshot()
        self.assertEqual(actual, expected)
        self.assertEqual(baseline, [])
        self.assertEqual(candidate, [])

    def test_large_nested_instance_display_has_no_deepcopy_or_mutation(self):
        service = self.fixture()
        instance = self.instance(age=60 * 86400)
        instance.metadata["large"] = {"rows": [[index, "evidence"] for index in range(1024)]}
        instance.observation_window = [InstanceObservationWindowSample(timestamp=NOW, pose=instance.pose) for _ in range(8)]
        instance.deferred_projection_cache = [DeferredProjectionInput(observer_pose=instance.pose, timestamp=NOW) for _ in range(32)]
        before = copy.deepcopy(instance.model_dump())
        with original_rendering():
            expected = service._semantic_instance_rows([instance], "library", NOW)
        with patch.object(SemanticInstance, "__deepcopy__", side_effect=AssertionError("display deep copy")):
            actual = service._semantic_instance_rows([instance], "library", NOW)
        self.assertEqual(actual, expected)
        self.assertEqual(instance.model_dump(), before)

    def test_association_still_materializes_full_aged_instances(self):
        policy = self.policy()
        instance = self.instance(age=60 * 86400)
        kwargs = {"existing_instances": {instance.instance_id: instance}, "map_name": "library", "now": NOW}
        with patch.object(InstanceAssociationService, "_mark_stale", _original_mark_stale), copy_counts() as baseline:
            expected, expected_outcomes = policy.associate([], **kwargs)
        with copy_counts() as candidate:
            actual, actual_outcomes = policy.associate([], **kwargs)
        self.assertEqual(actual_outcomes, expected_outcomes)
        self.assertEqual(actual[instance.instance_id].model_dump(), expected[instance.instance_id].model_dump())
        self.assertIsNot(actual[instance.instance_id], instance)
        self.assertEqual(candidate, baseline)
        self.assertEqual(candidate, [{"instance_id": instance.instance_id, "deep": True}])

    def test_python38_syntax(self):
        for path in (Path(__file__), Path("services/memory/service.py"), Path("services/memory/instance_association_service.py")):
            ast.parse(path.read_text(), filename=str(path), feature_version=(3, 8))


if __name__ == "__main__":
    unittest.main(verbosity=2)
