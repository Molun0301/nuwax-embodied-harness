#!/usr/bin/env python3
"""Real task/receipt projections with isolated navigation endpoints, no motion."""
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from contracts.navigation import NavigationGoal, NavigationState, NavigationStatus  # noqa: E402
from contracts.runtime_views import NavigationContext  # noqa: E402
from core.errors import GatewayError  # noqa: E402
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
        runtime._goal_with_motion_admission = lambda goal, admission: goal
        runtime._ensure_motion_runtime_for_backend_call = lambda *args, **kwargs: None
        runtime._capture_task_generation_snapshot = lambda **kwargs: {}
        runtime._record_task_generation_snapshot = lambda *args: None
        runtime._task_generation_rollovers = lambda *args: {}
        runtime._estimate_navigation_progress = lambda context: 0.1
        runtime._build_task_request_metadata = lambda **kwargs: {}
        runtime.memory_service.resolve_navigation_region = lambda goal: None
        runtime._cancel_navigation_and_wait_terminal = lambda *a, **k: self.fail('unexpected motion/cancel path')
        self.set_calls = []

    def state(self, message, status=NavigationStatus.FAILED, reached=False, metadata=None):
        return NavigationContext(backend_name='isolated', goal_reached=reached,
            navigation_state=NavigationState(current_goal_id='offline-goal', status=status,
                message=message, goal_reached=reached,
                metadata={'observed_goal_id': 'offline-goal', **(metadata or {})}))

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
        self.runtime.navigation_service.get_latest_navigation_context = lambda: self.current
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

    def test_native_success_does_not_require_python_arrival_or_zero_proof(self):
        receipt = {'actuator_zeroed': False, 'zero_ack_id': '', 'zero_applied_monotonic_ns': 0}
        task = self.submit([self.state('原生已完成停车后到达复核', NavigationStatus.SUCCEEDED,
                                      metadata=receipt)])
        result = self.manager.wait_for_task(task.task_id, timeout=2)
        self.assertEqual(self.manager.get_task_status(task.task_id).state.value, 'succeeded')
        for key, value in receipt.items():
            self.assertEqual(result['navigation_context'].navigation_state.metadata[key], value)

    def test_ungranted_lease_keeps_original_failure_without_substitute_proof(self):
        task = self.submit([self.state('LEASE_NOT_GRANTED: STALE_STOP_GENERATION',
                                      metadata={'reason_code': 'LEASE_NOT_GRANTED'})])
        with self.assertRaises(GatewayError) as raised:
            self.manager.wait_for_task(task.task_id, timeout=2)
        self.assertEqual(raised.exception.error_code, 'navigation_failed')
        self.assertEqual(raised.exception.message, 'LEASE_NOT_GRANTED: STALE_STOP_GENERATION')

    def test_native_cancel_without_zero_receipt_cancels_task(self):
        task = self.submit([self.state('后端已取消', NavigationStatus.CANCELLED)])
        self.manager.wait_for_task(task.task_id, timeout=2)
        self.assertEqual(self.manager.get_task_status(task.task_id).state.value, 'cancelled')

    def test_public_cancel_active_state_without_goal_id_delegates_to_backend(self):
        active = NavigationContext(navigation_state=NavigationState(status=NavigationStatus.RUNNING))
        calls = []
        self.runtime.navigation_service.get_navigation_control_context = lambda: active
        def cancel_goal(*, request_id):
            calls.append(request_id)
            return self.state('后端已取消当前目标', NavigationStatus.CANCELLED)
        self.runtime.navigation_service.cancel_goal = cancel_goal
        result = self.runtime._handle_cancel_navigation({})
        self.assertEqual(calls, [None])
        self.assertTrue(result['cancellation_requested'])
        self.assertTrue(result['completed'])
        self.assertEqual(result['navigation_state']['current_goal_id'], 'offline-goal')

    def test_memory_candidate_navigation_consumes_native_success(self):
        terminal = self.state('原生到达完成', NavigationStatus.SUCCEEDED)
        self.runtime.navigation_service.set_goal = lambda *args, **kwargs: terminal
        self.runtime.navigation_service.refresh_navigation = lambda **kwargs: terminal
        result = self.runtime._navigate_until_complete_with_motion_admission(
            NavigationGoal(goal_id='offline-goal', target_name='fixture'),
            motion_admission=object(), timeout_sec=1)
        self.assertIs(result, terminal)

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
