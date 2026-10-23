#!/usr/bin/env python3
"""地图 v2 身份的真实导航/探索调用回归；仅使用离线提供器，不连接设备。"""
from __future__ import annotations

import sys
import tempfile
import unittest
from contextlib import ExitStack
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from contracts.geometry import Pose
from contracts.maps import SemanticMap, SemanticRegion
from contracts.navigation import (
    ExplorationState,
    ExplorationStatus,
    ExploreAreaRequest,
    NavigationGoal,
    NavigationState,
    NavigationStatus,
)
from contracts.runtime_views import LocalizationSession, LocalizationSessionStatus, MapSnapshot
from core import StateNamespace, StateStore
from core.errors import GatewayError
from providers import ExplorationProvider, NavigationProvider
from services.mapping.service import MappingService
from services.navigation.forbidden_zones import ForbiddenZoneStore
from services.navigation.service import NavigationService


class OfflineProvider(NavigationProvider, ExplorationProvider):
    """只记录调用并返回合成状态，所有控制接口都留在当前进程。"""

    provider_name = "offline-platform"
    provider_version = "test"

    def __init__(self):
        self.goals = []
        self.requests = []
        self.navigation_state = NavigationState()
        self.exploration_state = ExplorationState()

    def is_available(self):
        return True

    def set_goal(self, goal):
        self.goals.append(goal)
        self.navigation_state = NavigationState(current_goal_id=goal.goal_id, status=NavigationStatus.ACCEPTED)
        return True

    def get_navigation_state(self):
        return self.navigation_state

    def get_navigation_control_state(self, *, deadline_monotonic=None):
        return self.navigation_state

    def is_goal_reached(self):
        return self.navigation_state.goal_reached

    def navigate(self, goal, timeout_sec=None):
        self.set_goal(goal)
        return goal.goal_id

    def cancel_goal(self, request_id=None):
        raise AssertionError("此回归不应执行取消")

    def cancel(self, request_id=None):
        return self.cancel_goal(request_id)

    def get_state(self, request_id=None):
        return self.get_navigation_state()

    def start_exploration(self, request):
        self.requests.append(request)
        self.exploration_state = ExplorationState(
            current_request_id=request.request_id, status=ExplorationStatus.ACCEPTED,
        )
        return True

    def stop_exploration(self):
        raise AssertionError("此回归不应执行停止")

    def get_exploration_state(self):
        return self.exploration_state


class NavigationMapIdentityChecks(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="neh-navigation-map-identity-")
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.provider = OfflineProvider()
        self.owner = SimpleNamespace(providers=self.provider)
        self.state = StateStore()
        self.mapping = MappingService(provider_owner=self.owner, state_store=self.state)
        self.session = LocalizationSession(
            session_id="offline-session", map_name="离线地图", status=LocalizationSessionStatus.READY,
        )
        self.localization = SimpleNamespace(
            get_active_session=lambda: self.session,
            refresh_current_session=lambda **kwargs: self.session,
        )
        self.navigation = NavigationService(
            provider_owner=self.owner, mapping_service=self.mapping,
            localization_service=self.localization, state_store=self.state,
            forbidden_zone_store=ForbiddenZoneStore(self.root / "forbidden-zones.json"),
        )
        # 在应用路径内禁止新摘要计算，不影响标准库协议或临时文件初始化。
        guards = ExitStack()
        self.addCleanup(guards.close)
        for name in ("sha256", "sha1", "md5", "new"):
            guards.enter_context(patch("hashlib." + name, side_effect=AssertionError("不应计算内容摘要")))

    def publish(self, sequence=7, epoch="source-lifecycle-1"):
        return self.mapping.ingest_external_snapshot(
            map_name="离线地图", source_name="generic-map-source", source_epoch=epoch,
            update_sequence_override=sequence,
            semantic_map=SemanticMap(
                map_id="离线地图", frame_id="map",
                regions=[SemanticRegion(
                    region_id="region-1", label="入口", centroid=Pose(frame_id="map"),
                    attributes={"alias": "Entry", "aliases": ["Lobby"]},
                )],
            ),
        )

    @staticmethod
    def expected(sequence=7, epoch="source-lifecycle-1"):
        return {
            "schema_version": 2, "map_name": "离线地图", "source_name": "generic-map-source",
            "source_epoch": epoch, "frame_id": "map", "update_sequence": str(sequence),
        }

    def assert_context_identity(self, context, expected, kind="navigation"):
        self.assertEqual(context.metadata["map_snapshot_identity"], expected)
        state = context.navigation_state if kind == "navigation" else context.exploration_state
        self.assertEqual(state.metadata["map_snapshot_identity"], expected)
        self.assertEqual(self.state.read(StateNamespace.NAVIGATION, kind).value, context)
        self.assertNotIn("content_sha256", context.model_dump_json())

    def test_named_goal_resolution_and_submission_use_complete_v2_binding(self):
        snapshot = self.publish(9007199254740993)
        self.assertFalse(hasattr(snapshot, "content_sha256"))
        for name in ("region-1", "入口", " Entry ", "LOBBY"):
            with self.subTest(name=name):
                goal = self.navigation.resolve_named_goal(name)
                self.assertEqual(goal.metadata["map_snapshot_identity"], self.expected(9007199254740993))
                self.assertEqual(goal.metadata["resolved_region_id"], "region-1")
                context = self.navigation.set_goal(goal)
                self.assertIs(self.provider.goals[-1], goal)
                self.assert_context_identity(context, self.expected(9007199254740993))

    def test_pose_submission_and_refresh_follow_revision_and_epoch_without_changing_old_context(self):
        self.publish()
        goal = NavigationGoal(goal_id="pose-goal", map_name="离线地图", target_pose=Pose(frame_id="map"))
        first = self.navigation.set_goal(goal)
        self.assert_context_identity(first, self.expected())
        for sequence, epoch in ((8, "source-lifecycle-1"), (1, "source-lifecycle-2")):
            with self.subTest(sequence=sequence, epoch=epoch):
                self.publish(sequence, epoch)
                self.assert_context_identity(self.navigation.refresh_navigation(), self.expected(sequence, epoch))
                self.assertEqual(first.metadata["map_snapshot_identity"], self.expected())
                self.assertEqual(self.navigation.resolve_named_goal("入口").metadata["map_snapshot_identity"],
                                 self.expected(sequence, epoch))
        self.assertEqual(self.provider.goals, [goal])

    def test_exploration_start_and_refresh_share_current_v2_binding(self):
        self.publish()
        request = ExploreAreaRequest(request_id="explore-1", map_name="离线地图")
        first = self.navigation.start_exploration(request)
        self.assert_context_identity(first, self.expected(), "exploration")
        self.publish(8)
        self.assert_context_identity(self.navigation.refresh_exploration(), self.expected(8), "exploration")
        self.assertEqual(first.metadata["map_snapshot_identity"], self.expected())
        self.assertEqual(self.provider.requests, [request])

    def test_absent_snapshot_remains_absent_in_navigation_and_exploration(self):
        for context, kind in ((self.navigation.refresh_navigation(), "navigation"),
                              (self.navigation.refresh_exploration(), "exploration")):
            self.assert_context_identity(context, None, kind)

    def test_existing_map_mismatch_errors_are_preserved(self):
        self.publish()
        self.session = self.session.model_copy(update={"map_name": "其它地图"})
        for operation in (lambda: self.navigation.resolve_named_goal("入口"),
                          self.navigation.refresh_navigation, self.navigation.refresh_exploration):
            with self.subTest(operation=operation):
                with self.assertRaises(GatewayError) as caught:
                    operation()
                self.assertEqual(caught.exception.error_code, "map_snapshot_name_mismatch")
        self.assertEqual(self.provider.goals, [])

    def test_legacy_snapshot_conversion_is_consumable_without_reintroducing_digest(self):
        current = self.publish(19)
        legacy = current.model_dump(mode="json")
        legacy.pop("schema_version")
        legacy["content_sha256"] = "opaque historical field"
        restored = MapSnapshot.model_validate(legacy)
        with patch.object(self.mapping, "get_latest_snapshot", return_value=restored):
            goal = self.navigation.resolve_named_goal("入口")
            self.assertEqual(goal.metadata["map_snapshot_identity"], self.expected(19))
            self.assert_context_identity(self.navigation.set_goal(goal), self.expected(19))
        self.assertEqual(legacy["content_sha256"], "opaque historical field")



if __name__ == "__main__":
    unittest.main(verbosity=2)
