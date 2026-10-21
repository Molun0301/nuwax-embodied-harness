"""生产请求三态与真实 protobuf 的代码级合成测试；不连接 RPC/机器人。"""
import ast
import importlib.util
from pathlib import Path
import unittest

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

    def test_real_provider_keyword_preserves_choice_without_mutating_goal(self):
        # Evaluate the exact production kwargs expression, not a replacement adapter.
        tree = ast.parse((ROOT / 'services/geometry/geometry_runtime_provider.py').read_text())
        calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
                 and isinstance(node.func, ast.Attribute) and node.func.attr == 'NavigationGoal']
        kwargs = [kw.value for call in calls for kw in call.keywords if kw.arg is None
                  and isinstance(kw.value, ast.Call) and isinstance(kw.value.func, ast.Name)
                  and kw.value.func.id == 'navigation_reverse_wire_fields']
        self.assertEqual(len(kwargs), 1)
        for choice in (None, False, True):
            goal = NavigationGoal(goal_id='provider-test', target_name='target', allow_reverse=choice)
            before = goal.model_dump_json()
            fields = eval(compile(ast.Expression(kwargs[0]), '<actual-provider-keyword>', 'eval'),
                          {'goal': goal, 'navigation_reverse_wire_fields': navigation_reverse_wire_fields})
            result = pb.NavigationGoal.FromString(pb.NavigationGoal(**fields).SerializeToString())
            self.assertEqual((result.allow_reverse, result.use_profile_reverse_default),
                             (choice is True, choice is None))
            self.assertEqual(goal.model_dump_json(), before)

    def test_gateway_does_not_inject_a_false_choice_when_argument_is_absent(self):
        tree = ast.parse((ROOT / 'gateways/runtime.py').read_text())
        calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
                 and isinstance(node.func, ast.Name) and node.func.id == 'NavigationGoal']
        choices = [kw.value for call in calls for kw in call.keywords if kw.arg == 'allow_reverse']
        self.assertEqual(len(choices), 1)
        for arguments, expected in (({}, None), ({'allow_reverse': False}, False), ({'allow_reverse': True}, True)):
            self.assertIs(eval(compile(ast.Expression(choices[0]), '<actual-gateway-keyword>', 'eval'),
                               {'arguments': arguments}), expected)


if __name__ == '__main__':
    unittest.main()
