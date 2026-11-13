"""回环HTTP与新任务执行器验证设备操作，不作为实机抓取验收。"""
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from types import SimpleNamespace

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from core.settings import ManipulationConfig
from core.tasks import TaskState
from core.errors import GatewayError
from gateways.invoke import invoke
from gateways.mcp.protocol import build_mcp_router
from services.manipulation.http_backend import DeviceError, HttpManipulationBackend
from services.manipulation.service import ManipulationService
from tests.conftest import isolated_platform_config

NAME = "execute_manipulation_task"

@pytest.fixture
def device():
    state=SimpleNamespace(session='session-a',epoch=2,mode='success',operations=list(HttpManipulationBackend.OPERATIONS),
        get_requests=[],requests=[],task_id='device-task-1',polls=0,started=threading.Event(),cancelled=threading.Event(),paused=False)
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def reply(self,value,status=200):
            body=json.dumps(value).encode();self.send_response(status);self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
        def do_GET(self):
            state.get_requests.append(self.path)
            if self.path.endswith('/capabilities'):self.reply({'operations':state.operations})
            else:self.reply({'state':'running','session':state.session,'epoch':state.epoch})
        def do_POST(self):
            req=json.loads(self.rfile.read(int(self.headers['Content-Length'])));state.requests.append(req)
            if req.get('session')!=state.session or req.get('epoch')!=state.epoch:
                self.reply({'error':'stale'},409);return
            op=req['op'];base={'session':state.session,'epoch':state.epoch,'task_id':state.task_id}
            if op=='manipulate':
                state.started.set()
                if state.mode=='start_lost':self.close_connection=True;return
                self.reply({**base,'accepted':True,'state':'running'});return
            if op=='cancel_manipulation':
                state.cancelled.set();state.paused=state.mode!='never_pause'
                self.reply({**base,'accepted':True,'state':'cancelled','physics_paused':state.paused});return
            state.polls+=1
            if state.mode=='wrong_task':base['task_id']='other-task'
            if state.mode=='wrong_epoch':base['epoch']+=1
            if state.cancelled.is_set():self.reply({**base,'state':'cancelled','physics_paused':state.paused});return
            terminal=state.mode in {'success','failed','false_success'} and state.polls>=2
            self.reply({**base,'state':('failed' if state.mode=='failed' else 'succeeded') if terminal else 'running',
                'steps':state.polls,'instruction':'把苹果放进盘子','error':'policy failure' if state.mode=='failed' else None,
                'result':{'physics_success':state.mode!='false_success','model_source':{'model_id':'test-only'}}})
    server=ThreadingHTTPServer(('127.0.0.1',0),Handler);thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    state.url='http://127.0.0.1:'+str(server.server_address[1])
    yield state
    server.shutdown();server.server_close();thread.join(2)


@pytest.fixture
def runtime(device, platform_factory, tmp_path):
    config = isolated_platform_config(tmp_path / "操作平台")
    config.manipulation = ManipulationConfig(enabled=True, base_url=device.url, http_timeout_sec=.2,
        stop_timeout_sec=.25, poll_interval_sec=.01)
    return platform_factory(config=config)


def submit(runtime, **kwargs):
    return invoke(runtime, NAME, {"instruction": "苹果入盘", "timeout_sec": 2, **kwargs}, "test")["task"]


def terminal(runtime, task):
    end = time.monotonic() + 4
    while time.monotonic() < end:
        status = runtime.tasks.status(task["task_id"])
        if status.finished_at is not None:
            return status
        threading.Event().wait(.005)
    raise AssertionError("设备操作未形成终态")


def test_discovery_reports_missing_operation_and_disabled_configuration(device, runtime):
    assert runtime.manipulation.availability()[0]
    device.operations.remove("cancel_manipulation")
    with pytest.raises(DeviceError):
        runtime.manipulation.bind()
    disabled = ManipulationService(ManipulationConfig())
    try:
        assert disabled.availability()[0] is False
        with pytest.raises(DeviceError): disabled.bind()
    finally:
        disabled.close()


def test_success_requires_real_device_result_and_preserves_task_binding(device, runtime):
    task = submit(runtime)
    status = terminal(runtime, task)
    assert status.state is TaskState.SUCCEEDED and status.motion is False
    assert status.result["device_task_id"] == "device-task-1"
    assert status.result["device_status"]["result"]["physics_success"] is True
    assert status.result["device_status"]["result"]["model_source"]["model_id"] == "test-only"
    assert not device.cancelled.is_set()


@pytest.mark.parametrize("mode", ["failed", "false_success"])
def test_failed_model_or_missing_physical_success_cannot_pass(mode, device, runtime):
    device.mode = mode
    status = terminal(runtime, submit(runtime))
    assert status.state is TaskState.FAILED and status.error_code == "manipulation_device_error"
    assert device.cancelled.is_set()


def test_cancel_sends_device_cancel_without_waiting_for_pause(device, runtime):
    device.mode = "never_pause"
    task = submit(runtime)
    assert device.started.wait(1)
    runtime.tasks.cancel(task["task_id"])
    status = terminal(runtime, task)
    assert status.state is TaskState.CANCELLED
    assert device.cancelled.is_set() and device.paused is False


def test_timeout_sends_cancel_and_keeps_task_timeout_state(device, runtime):
    device.mode = "running"
    task = submit(runtime, timeout_sec=1)
    status = terminal(runtime, task)
    assert status.state is TaskState.TIMED_OUT and device.cancelled.is_set()


@pytest.mark.parametrize("mode", ["wrong_task", "wrong_epoch", "start_lost"])
def test_mismatched_or_uncertain_device_result_is_failure(mode, device, runtime):
    device.mode = mode
    status = terminal(runtime, submit(runtime))
    assert status.state is TaskState.FAILED
    assert status.error_code == ("manipulation_device_error" if mode == "start_lost" else "manipulation_identity_mismatch")
    assert device.cancelled.is_set() is (mode != "start_lost")


def test_bound_old_device_generation_is_rejected_without_start(device, runtime):
    binding = runtime.manipulation.bind()
    device.epoch += 1
    from core.capability import Capability
    cap = Capability("设备绑定验证", "测试已绑定设备", "测试已绑定设备", {"type": "object"}, "task",
        lambda p, ctx, args: p.manipulation.run(ctx, binding,
            {"instruction": "苹果入盘", "timeout_sec": 1, "max_steps": 1440}), timeout_sec=2)
    runtime.capabilities.register(cap)
    task = invoke(runtime, cap.name, {}, "test")["task"]
    assert terminal(runtime, task).state is TaskState.FAILED
    assert not device.started.is_set() and not device.cancelled.is_set()


def test_real_mcp_catalog_and_execution_use_new_capability(device, runtime):
    app = FastAPI()
    app.include_router(build_mcp_router(runtime))
    with TestClient(app) as client:
        catalog = client.post("/mcp", json={"jsonrpc": "2.0", "id": 1, "method": "tools/list"}).json()
        tool = next(item for item in catalog["result"]["tools"] if item["name"] == NAME)
        assert tool["inputSchema"]["properties"]["timeout_sec"]["maximum"] == 1800
        response = client.post("/mcp", json={"jsonrpc": "2.0", "id": 2, "method": "tools/call",
            "params": {"name": NAME, "arguments": {"instruction": "苹果入盘"}}}).json()
        status = terminal(runtime, response["result"]["structuredContent"]["task"])
        assert status.state is TaskState.SUCCEEDED and status.motion is False


@pytest.mark.parametrize("args", [{"instruction": " "}, {"instruction": "x", "timeout_sec": 0},
    {"instruction": "x", "max_steps": 0}])
def test_invalid_instruction_and_limits_are_rejected_at_invoke(runtime, device, args):
    with pytest.raises(GatewayError) as error:
        invoke(runtime, NAME, args, "test")
    assert error.value.error_code == "invalid_params" and not device.started.is_set()
