"""三态倒退选择通过当前真实客户端和protobuf保持存在性。"""
import pytest
from services.geometry.client import NavigationGoal
from services.geometry.proto_gen import geometry_runtime_pb2 as pb
from services.geometry.state import Pose
from tests.test_a2a_geometry_client_state import runtime


@pytest.mark.parametrize('choice', [None, False, True])
def test_optional_reverse_survives_real_wire_without_mutating_goal(runtime, choice):
    service, client = runtime
    goal = NavigationGoal(Pose(1,2,3,0,0,0,1,'map',1783908184123456789),7,allow_reverse=choice)
    assert client.set_goal(goal)=='goal-new'
    wire = pb.NavigationGoal.FromString(service.goal.SerializeToString())
    assert wire.HasField('allow_reverse') == (choice is not None)
    if choice is not None:
        assert wire.allow_reverse is choice
    assert goal.allow_reverse is choice
    assert wire.pose.timestamp_ns == 1783908184123456789
