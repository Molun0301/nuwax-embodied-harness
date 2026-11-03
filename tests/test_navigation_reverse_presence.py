"""生产请求三态与真实 protobuf 的代码级合成测试；不连接 RPC/机器人。"""
import importlib.util
from pathlib import Path
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from contracts.navigation import NavigationGoal, navigation_reverse_wire_fields

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('reverse_test_geometry_pb2', ROOT / 'services/geometry/proto_gen/geometry_runtime_pb2.py')
pb = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pb)


def wire_roundtrip(choice):
    original = NavigationGoal(goal_id='choice-test', target_name='target', **({} if choice is None else {'allow_reverse': choice}))
    replay = NavigationGoal.model_validate_json(original.model_dump_json())
    assert replay.allow_reverse is choice
    request = pb.NavigationGoal(**navigation_reverse_wire_fields(replay.allow_reverse))
    return pb.NavigationGoal.FromString(request.SerializeToString())


class NavigationReversePresenceTests(unittest.TestCase):
    def test_missing_choice_survives_contract_serialization_and_wire(self):
        result = wire_roundtrip(None)
        self.assertFalse(result.allow_reverse)
        self.assertTrue(result.use_profile_reverse_default)

    def test_explicit_false_survives_contract_serialization_and_wire(self):
        result = wire_roundtrip(False)
        self.assertFalse(result.allow_reverse)
        self.assertFalse(result.use_profile_reverse_default)

    def test_explicit_true_survives_contract_serialization_and_wire(self):
        result = wire_roundtrip(True)
        self.assertTrue(result.allow_reverse)
        self.assertFalse(result.use_profile_reverse_default)

    def test_legacy_wire_false_and_true_do_not_opt_into_new_defaults(self):
        for value in (False, True):
            result = pb.NavigationGoal.FromString(pb.NavigationGoal(allow_reverse=value).SerializeToString())
            self.assertEqual(result.allow_reverse, value)
            self.assertFalse(result.use_profile_reverse_default)

    def test_invalid_untyped_choice_is_not_coerced_into_permission(self):
        for value in ('false', 1, [], {}):
            with self.assertRaises(ValueError):
                navigation_reverse_wire_fields(value)

    def test_real_provider_preserves_choice_without_mutating_goal(self):
        from tests.unit.test_python_provider_set_goal_transaction import _configured_provider, _goal, _state
        for choice in (None, False, True):
            goal = _goal('provider-test').model_copy(update={'allow_reverse': choice})
            before = goal.model_dump_json()
            stub = SimpleNamespace(SetNavigationGoal=Mock(return_value=_state(
                goal.goal_id, status='ACCEPTED', goal_generation=1)))
            provider = _configured_provider(stub)
            self.assertTrue(provider.set_goal(goal))
            request = stub.SetNavigationGoal.call_args.args[0]
            result = pb.NavigationGoal.FromString(request.SerializeToString())
            self.assertEqual((result.allow_reverse, result.use_profile_reverse_default),
                             (choice is True, choice is None))
            self.assertEqual(goal.model_dump_json(), before)

    def test_gateway_preserves_absent_and_explicit_reverse_choice(self):
        from tests.test_navigation_goal_height_public_input import NavigationGoalHeightPublicInputTests
        fixture = NavigationGoalHeightPublicInputTests()
        self.addCleanup(fixture.doCleanups)
        gateway = fixture.configure_runner(fixture.gateway())
        for values, expected in (({}, None), ({'allow_reverse': False}, False), ({'allow_reverse': True}, True)):
            fixture.mcp_call(gateway, {'frame_id': 'map', 'x': 1, 'y': 2, **values})
            fixture.run_latest(gateway)
            self.assertIs(gateway.dispatched[-1].allow_reverse, expected)


if __name__ == '__main__':
    unittest.main()
