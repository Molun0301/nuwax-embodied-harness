#!/usr/bin/env python3
"""Real task/receipt projections with isolated navigation endpoints, no motion."""
import json
import sys
import tempfile
from unittest.mock import patch
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from contracts.navigation import NavigationGoal, NavigationState, NavigationStatus  # noqa: E402
from contracts.runtime_views import NavigationContext  # noqa: E402
from core.errors import GatewayError  # noqa: E402
from core.event_bus import EventBus  # noqa: E402
from core.resource_lock import ResourceLockManager  # noqa: E402
from core.task_manager import TaskManager  # noqa: E402
from gateways.runtime import GatewayRuntime  # noqa: E402
from gateways.feedback import build_mcp_tool_result  # noqa: E402


class NavigationFailureMessageTests(unittest.TestCase):
    def setUp(self):
        from tests.conftest import create_isolated_gateway
        directory = tempfile.TemporaryDirectory(prefix="nuwax-navigation-message-")
        self.addCleanup(directory.cleanup)
        self.runtime = create_isolated_gateway(Path(directory.name), register_default_capabilities=True)
        self.addCleanup(self.runtime.stop)
        self.bus, self.manager = self.runtime.event_bus, self.runtime.task_manager
        runtime = self.runtime
        runtime.event_bus, runtime.task_manager = self.bus, self.manager
        runtime._task_status_last_polled_at = {}
        runtime._reconcile_navigation_quarantine = lambda: {'pending': False}
        runtime._goal_with_motion_admission = lambda goal, admission: goal
        runtime._run_with_motion_admission = lambda admission, operation: operation()
        runtime._ensure_motion_runtime_for_backend_call = lambda *args, **kwargs: None
        runtime._capture_task_generation_snapshot = lambda **kwargs: {}
        runtime._record_task_generation_snapshot = lambda *args: None
        runtime._task_generation_rollovers = lambda *args: {}
        runtime._estimate_navigation_progress = lambda context: 0.1
        runtime._build_task_request_metadata = lambda **kwargs: {}
        runtime.memory_service.resolve_navigation_region = lambda goal: None
        runtime._navigation_terminal_proof = lambda context, **kwargs: {
            'goal_id': kwargs['goal_id'], 'parking_confirmed': True}
        runtime._cancel_navigation_with_safety_barrier = lambda *a, **k: self.fail('unexpected motion/cancel path')
        self.set_calls = []

    def state(self, message, status=NavigationStatus.FAILED, reached=False):
        return NavigationContext(backend_name='isolated', goal_reached=reached,
            navigation_state=NavigationState(current_goal_id='offline-goal', status=status,
                message=message, goal_reached=reached))

    def submit(self, states):
        remaining = iter(states)
        self.current = states[0]

        def refresh(**kwargs):
            self.current = next(remaining, self.current)
            return self.current

        def set_goal(goal, **kwargs):
            self.set_calls.append(goal.goal_id)
            return self.current

        self.runtime.navigation_service.set_goal = set_goal
        self.runtime.navigation_service.refresh_navigation = refresh
        return self.runtime._submit_navigation_task(capability_name='navigate_to_pose',
            goal=NavigationGoal(goal_id='offline-goal', map_name='map', target_name='fixture'),
            motion_admission=object(), poll_interval_sec=0.02, timeout_sec=2,
            requested_by='offline-test')

    def test_failed_budget_timeout_and_geometry_keep_cause_without_unreachable_claim(self):
        for cause in ('BudgetExhausted: 256 visits; clearance unknown',
                      'planner path timeout; previous=MissingSupport',
                      'InsufficientClearance: candidate rejected'):
            with self.subTest(cause=cause):
                task = self.submit([self.state(cause)])
                with self.assertRaises(GatewayError) as raised:
                    self.manager.wait_for_task(task.task_id, timeout=2)
                self.assertEqual(raised.exception.error_code, 'navigation_failed')
                snapshot = self.runtime.get_task_snapshot(task.task_id)
                self.assertEqual(snapshot['error_payload']['code'], 'navigation_failed')
                self.assertEqual(snapshot['error_payload']['message'], cause)
                self.assertIn(cause, snapshot['status'].message)
                self.assertIn(cause, snapshot['feedback']['message'])
                self.assertIn(cause, snapshot['events'][-1].message)
                mcp = self.runtime._handle_get_navigation_state({})
                self.assertEqual(mcp['navigation_state']['message'], cause)
                public = build_mcp_tool_result(snapshot)
                self.assertTrue(public['isError'])
                self.assertEqual(json.loads(public['content'][0]['text']), public['structuredContent'])
                self.assertEqual(public['structuredContent']['error_payload']['code'], 'navigation_failed')
                self.assertEqual(public['structuredContent']['error_payload']['message'], cause)
                self.assertIn(cause, public['structuredContent']['status']['message'])

    def test_success_without_arrival_retains_specific_error(self):
        task = self.submit([self.state('backend done without arrival', NavigationStatus.SUCCEEDED)])
        with self.assertRaises(GatewayError) as raised:
            self.manager.wait_for_task(task.task_id, timeout=2)
        self.assertEqual(raised.exception.error_code, 'arrival_not_confirmed')

    def test_stale_input_wait_keeps_the_same_task_and_goal(self):
        task = self.submit([self.state('位姿短时过期，保持零速等待', NavigationStatus.RUNNING),
                            self.state('arrived', NavigationStatus.SUCCEEDED, True)])
        self.manager.wait_for_task(task.task_id, timeout=2)
        snapshot = self.runtime.get_task_snapshot(task.task_id)
        self.assertEqual(snapshot['status'].state.value, 'succeeded')
        self.assertEqual(self.set_calls, ['offline-goal'])
        self.assertTrue(any(event.payload.get('stage') == 'navigation_running'
                            and event.message == '位姿短时过期，保持零速等待'
                            for event in snapshot['events']))


if __name__ == '__main__':
    unittest.main()
