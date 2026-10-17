#!/usr/bin/env python3
"""导航服务状态消费回归：合成提供器与真实服务，不连接设备或执行运动。"""
from __future__ import annotations

import sys
import tempfile
from datetime import timedelta
import time
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from contracts.navigation import NavigationGoal, NavigationState, NavigationStatus
from contracts.runtime_views import LocalizationSession, LocalizationSessionStatus, MapSnapshot
from core import StateStore
from scripts.check_navigation_map_snapshot_identity import OfflineProvider
from services.navigation.forbidden_zones import ForbiddenZoneStore
from services.navigation.service import NavigationService


class LifecycleProvider(OfflineProvider):
    def __init__(self):
        super().__init__()
        self.generation = 0
        self.on_submit = None
        self.submit_status = NavigationStatus.ACCEPTED

    def set_goal(self, goal, **kwargs):
        if self.on_submit is not None:
            self.on_submit()
        self.generation += 1
        self.goals.append(goal)
        self.navigation_state = state(goal.goal_id, self.submit_status, self.generation)
        return True

    def get_navigation_state(self, **kwargs):
        return self.navigation_state


def state(goal_id, status, generation=1, **kwargs):
    return NavigationState(current_goal_id=goal_id, status=status,
        metadata={"producer_runtime_generation": 1, "goal_generation": generation}, **kwargs)


class NavigationServiceLifecycleChecks(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="neh-navigation-lifecycle-")
        self.addCleanup(temporary.cleanup)
        self.provider = LifecycleProvider()
        self.navigation = NavigationService(
            provider_owner=NS(providers=self.provider), state_store=StateStore(),
            mapping_service=NS(get_latest_snapshot=lambda: None),
            localization_service=NS(get_active_session=lambda: None),
            forbidden_zone_store=ForbiddenZoneStore(Path(temporary.name) / "zones.json"),
        )

    def goal(self, goal_id):
        return NavigationGoal(goal_id=goal_id, target_name="离线目标")

    def test_late_old_terminal_during_new_submission_cannot_fail_new_running_goal(self):
        first = self.goal("first")
        self.navigation.set_goal(first)
        stale = state("first", NavigationStatus.FAILED, message="旧目标失败")
        self.provider.on_submit = lambda: self.navigation._handle_provider_navigation_update(stale)
        self.provider.submit_status = NavigationStatus.RUNNING
        second = self.goal("second")
        context = self.navigation.set_goal(second)
        self.assertEqual(context.navigation_state.status, NavigationStatus.RUNNING)
        self.assertIs(context.current_goal, second)
        self.assertIs(self.navigation._current_goal, second)

    def test_first_running_snapshot_keeps_authoritative_backend_status(self):
        self.provider.navigation_state = state("already-running", NavigationStatus.RUNNING)
        context = self.navigation.refresh_navigation()
        self.assertEqual(context.navigation_state.status, NavigationStatus.RUNNING)
        self.assertEqual(context.metadata["navigation_state_machine"]["goal_id"], "already-running")

    def test_lightweight_terminal_then_full_running_cannot_disagree(self):
        goal = self.goal("retry")
        self.navigation.set_goal(goal)
        self.provider.navigation_state = state("retry", NavigationStatus.FAILED)
        failed = self.navigation.refresh_navigation(deadline_monotonic=time.monotonic() + 1)
        self.assertEqual(failed.navigation_state.status, NavigationStatus.FAILED)
        self.assertEqual(self.navigation._state_machine.current_status, NavigationStatus.FAILED)

    def test_running_to_planning_transition_is_preserved(self):
        goal = self.goal("replanning")
        self.navigation.set_goal(goal)
        self.provider.navigation_state = state(goal.goal_id, NavigationStatus.RUNNING)
        self.navigation.refresh_navigation()
        self.provider.navigation_state = state(goal.goal_id, NavigationStatus.PLANNING, message="等待可通行路径")
        context = self.navigation.refresh_navigation()
        self.assertEqual(context.navigation_state.status, NavigationStatus.PLANNING)
        self.assertEqual(context.navigation_state.message, "等待可通行路径")


    def test_older_goal_generation_never_replaces_new_goal(self):
        self.navigation.set_goal(self.goal("first"))
        self.navigation.set_goal(self.goal("second"))
        previous = self.navigation.get_latest_navigation_context()
        late = state("first", NavigationStatus.FAILED, 1)
        result = self.navigation._ingest_navigation_state(self.provider, late)
        self.assertIs(result, previous)
        self.assertEqual(result.navigation_state.current_goal_id, "second")

    def test_terminal_zero_proof_is_not_erased_by_same_generation_running(self):
        self.navigation.set_goal(self.goal("goal"))
        terminal = state("goal", NavigationStatus.CANCELLED)
        terminal.metadata.update(actuator_zeroed=True, zero_ack_id="offline-zero", zero_applied_monotonic_ns=1)
        self.navigation._ingest_navigation_state(self.provider, terminal)
        previous = self.navigation.get_latest_navigation_context()
        result = self.navigation._ingest_navigation_state(self.provider, state("goal", NavigationStatus.RUNNING))
        self.assertIs(result, previous)
        self.assertEqual(result.navigation_state.status, NavigationStatus.CANCELLED)

    def test_older_snapshot_does_not_erase_running_progress(self):
        self.navigation.set_goal(self.goal("goal"))
        running = state("goal", NavigationStatus.RUNNING)
        self.navigation._ingest_navigation_state(self.provider, running)
        previous = self.navigation.get_latest_navigation_context()
        old = state("goal", NavigationStatus.ACCEPTED, updated_at=running.updated_at-timedelta(seconds=1))
        self.assertIs(self.navigation._ingest_navigation_state(self.provider, old), previous)

    def test_same_goal_new_generation_can_run_after_terminal_zero_proof(self):
        self.navigation.set_goal(self.goal("goal"))
        terminal = state("goal", NavigationStatus.FAILED)
        terminal.metadata.update(actuator_zeroed=True, zero_ack_id="offline-zero", zero_applied_monotonic_ns=1)
        self.navigation._ingest_navigation_state(self.provider, terminal)
        result = self.navigation._ingest_navigation_state(self.provider, state("goal", NavigationStatus.RUNNING, 2))
        self.assertEqual(result.navigation_state.status, NavigationStatus.RUNNING)
        self.assertEqual(result.metadata["navigation_state_machine"]["status"], "running")

    def test_same_goal_retry_preserves_target_during_old_terminal_delivery(self):
        self.navigation.set_goal(self.goal("same"))
        stale = state("same", NavigationStatus.FAILED, message="旧尝试失败")
        first = self.navigation._current_goal
        retry = NavigationGoal(goal_id="same", target_name="重试目标")
        def late_terminal_updates():
            for lightweight in (False, True, False):
                old_context = self.navigation._ingest_navigation_state(self.provider, stale, lightweight=lightweight)
                self.assertIs(old_context.current_goal, first)
                self.assertIs(self.navigation._current_goal, retry)
        self.provider.on_submit = late_terminal_updates
        self.provider.submit_status = NavigationStatus.RUNNING
        result = self.navigation.set_goal(retry)
        self.assertEqual(result.navigation_state.status, NavigationStatus.RUNNING)
        self.assertIs(result.current_goal, retry)
        self.assertIs(self.navigation._current_goal, retry)

    def test_same_object_retry_retains_target_across_repeated_terminal_reads(self):
        goal = self.goal("same-object")
        self.navigation.set_goal(goal)
        stale = state(goal.goal_id, NavigationStatus.FAILED)
        self.provider.on_submit = lambda: [
            self.navigation._ingest_navigation_state(self.provider, stale, lightweight=lightweight)
            for lightweight in (False, True, False)
        ]
        self.provider.submit_status = NavigationStatus.RUNNING
        result = self.navigation.set_goal(goal)
        self.assertEqual(result.navigation_state.status, NavigationStatus.RUNNING)
        self.assertIs(result.current_goal, goal)
        self.assertIs(self.navigation._current_goal, goal)

    def test_old_attempt_on_previous_map_does_not_acquire_new_target(self):
        snapshot = [MapSnapshot(schema_version=2, map_name="map-A", source_name="offline",
                                source_epoch="a", update_sequence=1)]
        self.navigation._mapping_service.get_latest_snapshot = lambda: snapshot[0]
        self.navigation._localization_service.get_active_session = lambda: LocalizationSession(
            session_id="offline-session", map_name=snapshot[0].map_name, status=LocalizationSessionStatus.READY)
        first = NavigationGoal(goal_id="same", map_name="map-A", target_name="旧目标")
        self.navigation.set_goal(first)
        snapshot[0] = MapSnapshot(schema_version=2, map_name="map-B", source_name="offline",
                                  source_epoch="b", update_sequence=1)
        retry = NavigationGoal(goal_id="same", map_name="map-B", target_name="新目标")
        stale = state("same", NavigationStatus.FAILED)
        def old_attempt_updates():
            for lightweight in (False, True, False):
                result = self.navigation._ingest_navigation_state(self.provider, stale, lightweight=lightweight)
                self.assertIs(result.current_goal, first)
                self.assertEqual(result.current_goal.map_name, "map-A")
                self.assertIs(self.navigation._current_goal, retry)
        self.provider.on_submit = old_attempt_updates
        self.provider.submit_status = NavigationStatus.RUNNING
        result = self.navigation.set_goal(retry)
        self.assertIs(result.current_goal, retry)
        self.assertEqual(result.metadata["map_name"], "map-B")
        self.assertEqual(result.navigation_state.status, NavigationStatus.RUNNING)

    def test_terminal_without_goal_id_cannot_clear_the_submitted_goal(self):
        goal = self.goal("pending")
        self.navigation.set_goal(goal)
        result = self.navigation._ingest_navigation_state(self.provider, state(None, NavigationStatus.FAILED))
        self.assertIs(self.navigation._current_goal, goal)
        self.assertIsNone(result.current_goal)
        self.assertIsNone(result.navigation_state.current_goal_id)
        self.assertIsNone(result.metadata["navigation_state_machine"]["goal_id"])
        self.assertIsNone(result.metadata["diagnostics"]["goal_id"])

    def test_running_frame_is_not_promoted_to_success_by_arrival_hint(self):
        self.navigation.set_goal(self.goal("goal"))
        result = self.navigation._ingest_navigation_state(self.provider,
            state("goal", NavigationStatus.RUNNING, goal_reached=True))
        self.assertEqual(result.navigation_state.status, NavigationStatus.RUNNING)
        self.assertTrue(result.goal_reached)

    def test_replanning_diagnostics_match_same_frame(self):
        self.navigation.set_goal(self.goal("goal"))
        result = self.navigation._ingest_navigation_state(self.provider, state("goal", NavigationStatus.RUNNING))
        self.assertEqual(result.metadata["diagnostics"]["platform_status"], "running")
        self.assertEqual(result.metadata["navigation_state_machine"]["backend_state"], "running")

    def test_full_and_lightweight_share_each_lifecycle_transition(self):
        goal = self.goal("goal")
        self.navigation.set_goal(goal)
        for status in (NavigationStatus.RUNNING, NavigationStatus.PLANNING,
                       NavigationStatus.RUNNING, NavigationStatus.SUCCEEDED):
            with self.subTest(status=status):
                frame = state(goal.goal_id, status, goal_reached=status == NavigationStatus.SUCCEEDED)
                full = self.navigation._ingest_navigation_state(self.provider, frame)
                full_summary = self.navigation._state_machine.get_state_summary()
                light = self.navigation._ingest_navigation_state(self.provider, frame, lightweight=True)
                self.assertEqual(light.navigation_state.status, full.navigation_state.status)
                self.assertEqual(light.goal_reached, full.goal_reached)
                self.assertEqual(light.navigation_state.current_goal_id, full.navigation_state.current_goal_id)
                self.assertEqual(self.navigation._state_machine.current_status.value, full_summary["status"])
                self.assertEqual(self.navigation._state_machine.current_goal_id, goal.goal_id)

    def test_terminal_without_arrival_is_not_changed_to_success(self):
        self.navigation.set_goal(self.goal("goal"))
        result = self.navigation._ingest_navigation_state(self.provider, state("goal", NavigationStatus.FAILED))
        self.assertEqual(result.navigation_state.status, NavigationStatus.FAILED)
        self.assertFalse(result.goal_reached)


if __name__ == "__main__":
    unittest.main()
