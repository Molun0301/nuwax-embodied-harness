"""公开HTTP/MCP目标到真实Geometry协议的高度、朝向及可选参数。"""
import math
from types import SimpleNamespace

from fastapi import FastAPI
from fastapi.responses import JSONResponse
from fastapi.testclient import TestClient
import pytest

from core.errors import GatewayError
from core.settings import load_config
from core.stream_bus import StreamBus
from gateways.http.api import build_http_router
from gateways.mcp.protocol import build_mcp_router
from services.geometry.proto_gen import geometry_runtime_pb2 as pb
from tests.test_a2b_navigation import navigation, finished
from tests.test_a2a_geometry_client_state import eventually


@pytest.fixture
def public_navigation(navigation):
    platform,service=navigation
    platform.config=load_config()
    bus=StreamBus()
    platform.streams=SimpleNamespace(stream_bus=bus,event_bus=platform.events)
    app=FastAPI()
    @app.exception_handler(GatewayError)
    async def error_handler(_request,error):
        return JSONResponse({'error':error.to_payload()},status_code=error.http_status)
    app.include_router(build_http_router(platform,None))
    app.include_router(build_mcp_router(platform,None))
    try:
        with TestClient(app) as client:
            yield platform,service,client
    finally:
        bus.shutdown()


def dispatch(fixture,transport,args):
    p,service,client=fixture
    if transport=='http':
        response=client.post('/api/capabilities/navigate_to_pose/invoke',json={'arguments':args})
        assert response.status_code==200
        result=response.json()
    else:
        response=client.post('/mcp',json={'jsonrpc':'2.0','id':'height','method':'tools/call',
            'params':{'name':'navigate_to_pose','arguments':args}})
        assert response.status_code==200
        result=response.json()['result']['structuredContent']
    assert service.received.wait(2)
    return result['task']['task_id']


def complete(fixture,task_id):
    p,service,_=fixture
    service.transition(pb.GOAL_STATUS_SUCCEEDED)
    assert finished(p,task_id).state.value=='succeeded'


@pytest.mark.parametrize('transport',['http','mcp'])
@pytest.mark.parametrize('pipeline,height',[('lio_3d',1.7),('slam_2d',0.)])
def test_omitted_height_uses_actual_pipeline_and_source_pose(public_navigation,transport,pipeline,height):
    p,service,_=public_navigation
    service.state.info.pipeline=pipeline
    service.send('state',service.state)
    eventually(lambda:p.geometry_state.snapshot().info.pipeline==pipeline)
    task_id=dispatch(public_navigation,transport,{'x':1.,'y':2.})
    assert service.goal.pose.position.z==height
    assert (service.goal.pose.position.x,service.goal.pose.position.y)==(1.,2.)
    assert not service.goal.HasField('allow_reverse')
    assert service.goal.position_tolerance_m==.35
    assert service.goal.yaw_tolerance_rad==math.pi
    complete(public_navigation,task_id)


@pytest.mark.parametrize('transport',['http','mcp'])
@pytest.mark.parametrize('height',[0.,-2.3,4.2])
def test_explicit_height_including_zero_and_negative_is_not_replaced(public_navigation,transport,height):
    task_id=dispatch(public_navigation,transport,{'x':1.,'y':2.,'z':height})
    assert public_navigation[1].goal.pose.position.z==height
    complete(public_navigation,task_id)


@pytest.mark.parametrize('transport',['http','mcp'])
@pytest.mark.parametrize('yaw,reverse',[(0.,False),(.7,True),(-.4,None)])
def test_yaw_reverse_and_tolerances_cross_public_protocol_unchanged(public_navigation,transport,yaw,reverse):
    args={'x':1.,'y':2.,'z':0.,'yaw_rad':yaw,'tolerance_position_m':.3,'tolerance_yaw_rad':.15,'frame_id':'map'}
    if reverse is not None:args['allow_reverse']=reverse
    task_id=dispatch(public_navigation,transport,args)
    goal=public_navigation[1].goal
    assert goal.pose.orientation.z==math.sin(yaw/2)
    assert goal.pose.orientation.w==math.cos(yaw/2)
    assert goal.pose.frame_id=='map'
    assert goal.HasField('allow_reverse')==(reverse is not None)
    if reverse is not None:assert goal.allow_reverse is reverse
    assert goal.position_tolerance_m==pytest.approx(.3)
    assert goal.yaw_tolerance_rad==pytest.approx(.15)
    complete(public_navigation,task_id)


def test_removed_navigation_route_is_absent(public_navigation):
    assert public_navigation[2].post('/api/navigation/goal',json={'x':1,'y':2}).status_code==404
