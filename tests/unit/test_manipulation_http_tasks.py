"""可控回环HTTP验证外部设备协议与真实TaskManager；不是物理抓取验收。"""
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from types import SimpleNamespace

import pytest

from core.resource_lock import ResourceConflictError
from core.settings import ManipulationConfig, load_config
from core.task_manager import TaskSafetyBarrierError
from gateways.manipulation import NAME, availability, execute, register_capability, register_skill
from services.manipulation.http_backend import DeviceError, HttpManipulationBackend
from services.manipulation.service import ManipulationService


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
def runtime(device, gateway_runtime_factory, monkeypatch):
    runtime = gateway_runtime_factory()
    config = ManipulationConfig(enabled=True, base_url=device.url, http_timeout_sec=.2,
        stop_timeout_sec=.25, poll_interval_sec=.01)
    monkeypatch.setattr(runtime.config, "manipulation", config)
    register_capability(runtime)
    register_skill(runtime)
    yield runtime


def submit(runtime,**kwargs):
    return execute(runtime,{'instruction':'把苹果放进盘子','timeout_sec':2,**kwargs})


def terminal(runtime,task):
    end=time.monotonic()+4
    while time.monotonic()<end:
        value=runtime.task_manager.get_task_status(task.task_id)
        if value.state.value in {'succeeded','failed','cancelled','timeout'}:return value
        time.sleep(.005)
    raise AssertionError('任务未形成终态')


def test_discovery_and_default_disabled(device,runtime):
    assert availability(runtime).supported
    device.operations.remove('cancel_manipulation')
    assert availability(runtime).supported  # 目录不是后端健康结论。
    with pytest.raises(DeviceError): runtime.manipulation_service.bind()
    runtime.config.manipulation.enabled=False
    assert not availability(runtime).supported
    assert load_config().manipulation.enabled is False
    assert load_config().scene_mapping.auto_apply is True


def test_success_is_physical_terminal_not_acceptance(device,runtime):
    task=submit(runtime)
    assert terminal(runtime,task).state.value=='succeeded'
    result=runtime.task_manager.get_task_result(task.task_id)
    assert result['device_status']['result']['physics_success'] is True
    assert device.polls>=2 and not device.cancelled.is_set()
    assert all(r['session']=='session-a' and r['epoch']==2 for r in device.requests)
    assert runtime.resource_lock_manager.snapshot()==()


@pytest.mark.parametrize('mode',['failed','false_success'])
def test_failed_model_or_missing_physical_success_cannot_pass(mode,device,runtime):
    device.mode=mode;task=submit(runtime)
    assert terminal(runtime,task).state.value=='failed'
    assert device.cancelled.is_set() and device.paused


def test_cancel_waits_for_matching_pause_and_holds_motion_resource(device,runtime):
    device.mode='running';task=submit(runtime);assert device.started.wait(1)
    with pytest.raises(ResourceConflictError):runtime.resource_lock_manager.acquire('navigation',['base_motion'])
    runtime.task_manager.cancel_task(task.task_id)
    assert terminal(runtime,task).state.value=='cancelled'
    assert device.cancelled.is_set() and device.paused
    events=runtime.task_manager.get_task_events(task.task_id)
    proof=next(e.payload['interrupt_barrier_proof'] for e in events if e.event_type=='task.interrupt_barrier_acknowledged')
    assert proof['device_task_id']=='device-task-1' and proof['physics_paused'] is True
    assert 'joint' not in proof


def test_timeout_sends_cancel_and_confirms_terminal(device,runtime):
    device.mode='running';task=submit(runtime,timeout_sec=1)
    assert terminal(runtime,task).state.value=='timeout'
    assert device.cancelled.is_set() and device.paused


@pytest.mark.parametrize('mode',['wrong_task','wrong_epoch','never_pause','start_lost'])
def test_uncertain_or_mismatched_stop_fails_and_releases_task_resources(mode,device,runtime):
    device.mode=mode;task=submit(runtime,timeout_sec=1)
    assert terminal(runtime,task).state.value=='failed'
    with pytest.raises(TaskSafetyBarrierError):
        runtime.task_manager.wait_for_task(task.task_id, timeout=2)
    assert runtime.task_manager.get_task_error_payload(task.task_id)["code"] == "manipulation_stop_unconfirmed"
    assert not runtime.resource_lock_manager.held_resources(task.task_id)
    if mode=='start_lost':assert not device.cancelled.is_set()


def test_bound_old_generation_is_rejected_without_start(device,runtime):
    service=runtime.manipulation_service;binding=service.bind();device.epoch+=1
    task=runtime.task_manager.submit(NAME,lambda ctx:service.run(ctx,binding,service.parameters({'instruction':'苹果入盘'})),
        required_resources=['base_motion'],timeout_sec=1,deferred_interrupt=True)
    assert terminal(runtime,task).state.value=='failed'
    assert not device.started.is_set() and not device.cancelled.is_set()


def test_real_mcp_catalog_shows_configured_unverified_device(device,runtime):
    from contracts.capabilities import CapabilityMatrix
    from gateways.mcp.protocol import _build_collection_tools
    runtime.robot=SimpleNamespace(manifest=SimpleNamespace(robot_model='external-device'))
    def refresh(**kwargs):
        runtime.capability_registry.bind_robot_capability_matrix(CapabilityMatrix(robot_model='external-device',capabilities=[availability(runtime)]))
    runtime._refresh_dynamic_capability_matrix=refresh
    assert NAME in json.dumps(_build_collection_tools(runtime))
    device.operations=[]
    from gateways.mcp.protocol import _build_collection_catalogs
    catalog = json.dumps(_build_collection_catalogs(runtime), ensure_ascii=False)
    assert NAME in catalog and '目录不探测远端' in catalog
    assert device.get_requests == []
    runtime.config.manipulation.enabled = False
    assert NAME not in json.dumps(_build_collection_tools(runtime))


def test_shutdown_cancels_owned_device_before_transport_close(device,runtime):
    device.mode='running';task=submit(runtime);assert device.started.wait(1)
    runtime.task_manager.shutdown(wait=True)
    assert runtime.task_manager.get_task_status(task.task_id).state.value=='cancelled'
    assert device.cancelled.is_set() and device.paused


def build_gateway(device, tmp_path):
    from drivers.customrobot import create_customrobot_assembly
    from gateways.runtime import GatewayRuntime
    config=load_config();config.robot.active_robot='customrobot';config.geometry.enabled=False
    config.runtime_data.map_active_restore_on_start=False
    config.runtime_data.map_catalog_root=str(tmp_path/'maps');config.runtime_data.memory_db_path=str(tmp_path/'memory.db')
    config.runtime_data.memory_embedding_model='disabled';config.runtime_data.memory_embedding_dimension=128
    config.runtime_data.memory_image_embedding_model='disabled';config.gateway.artifact_dir=str(tmp_path/'artifacts')
    config.perception.yolo_world.enabled=config.perception.openai_vision.enabled=config.perception.segmentation.enabled=False
    config.perception.stream_runtime.enabled=config.perception.stream_runtime.auto_start=False
    config.scene_mapping.enabled=False
    config.manipulation=ManipulationConfig(enabled=True,base_url=device.url,http_timeout_sec=.2,stop_timeout_sec=.25,poll_interval_sec=.01)
    return GatewayRuntime(config=config,robot=create_customrobot_assembly(config))


def test_full_gateway_public_task_snapshot_uses_same_handler(device,tmp_path):
    gateway = build_gateway(device, tmp_path)
    try:
        captured = {"wire": "memory-service"}
        gateway.memory_service.capture_observation_spatial_context = lambda: captured
        assert gateway.perception_service._spatial_context_provider() is captured
        response=gateway.invoke_tool(NAME,{'instruction':'苹果入盘','timeout_sec':2},requested_by='isolated-test')
        task=SimpleNamespace(task_id=response['task_id'])
        assert terminal(gateway,task).state.value=='succeeded'
        snapshot=gateway.get_task_snapshot(task.task_id)
        assert snapshot['result']['device_status']['result']['physics_success'] is True
        assert snapshot['task'].capability_name==NAME
        assert device.get_requests == ['/api/v1/capabilities', '/api/v1/status']
    finally:gateway.stop()


@pytest.mark.parametrize('parameters', [{'timeout_sec': .5}, {'timeout_sec': 1801}, {'max_steps': 10001}])
def test_public_limits_match_device_protocol(parameters):
    from services.manipulation.service import ManipulationService
    with pytest.raises(ValueError):
        ManipulationService.parameters({'instruction': '苹果入盘', **parameters})


def test_close_timeout_keeps_http_alive_until_owned_task_exits(device, runtime):
    device.mode = 'running'
    task = submit(runtime)
    assert device.started.wait(1)
    service = runtime.manipulation_service
    assert service.close(timeout=0) is False
    assert not service.backend.client.is_closed
    with pytest.raises(DeviceError): service.bind()
    runtime.task_manager.shutdown(wait=True)
    assert device.cancelled.is_set() and device.paused
    assert service.close(timeout=.5) is True
    assert service.backend.client.is_closed
    assert service.close(timeout=0) is True


def test_gateway_stop_keeps_transport_until_task_cancel_proof(device, tmp_path):
    device.mode = 'running'
    gateway = build_gateway(device, tmp_path)
    service = gateway.manipulation_service
    original_close = service.backend.close
    observed = []
    def checked_close():
        assert device.cancelled.is_set() and device.paused
        assert not service._active_operations
        observed.append(True)
        return original_close()
    service.backend.close = checked_close
    try:
        response = gateway.invoke_tool(NAME, {'instruction': '苹果入盘', 'timeout_sec': 2})
        assert device.started.wait(1)
        gateway.stop()
        assert gateway.task_manager.get_task_status(response['task_id']).state.value == 'cancelled'
        assert observed == [True] and service.backend.client.is_closed
    finally:
        gateway.stop()


def test_gateway_retry_retains_cloud_dispatch_blockers(device, tmp_path, monkeypatch):
    gateway = build_gateway(device, tmp_path)
    service = gateway.manipulation_service
    original_close = service.close
    monkeypatch.setattr(service, 'close', lambda timeout=2: False)
    with pytest.raises(RuntimeError): gateway.stop()
    assert gateway._dispatch_shutdown_complete is False
    assert gateway._dispatch_shutdown_blocker == 'manipulation_in_flight'
    assert not service.backend.client.is_closed
    monkeypatch.setattr(service, 'close', original_close)
    original_artifacts = gateway._close_artifact_dependencies
    monkeypatch.setattr(gateway, '_close_artifact_dependencies', lambda timeout=2: False)
    with pytest.raises(RuntimeError): gateway.stop()
    assert gateway._dispatch_shutdown_blocker == 'artifact_owner_in_flight'
    monkeypatch.setattr(gateway, '_close_artifact_dependencies', original_artifacts)
    gateway.stop()
    assert gateway._dispatch_shutdown_complete is True
    assert gateway._dispatch_shutdown_blocker is None


def test_service_retries_incomplete_transport_close():
    calls = []
    def close():
        calls.append(True)
        return len(calls) > 1
    service = ManipulationService(ManipulationConfig(), backend=SimpleNamespace(close=close))
    assert service.close(timeout=0) is False
    assert service.close(timeout=0) is True
    assert service.close(timeout=0) is True
    assert len(calls) == 2


def test_operation_config_loads_explicit_values_without_losing_cloud_options(monkeypatch):
    values = {'NUWAX_MANIPULATION_ENABLED': 'true', 'NUWAX_MANIPULATION_BASE_URL': 'http://127.0.0.1:9',
              'NUWAX_MANIPULATION_HTTP_TIMEOUT_SEC': '4', 'NUWAX_MANIPULATION_STOP_TIMEOUT_SEC': '12',
              'NUWAX_MANIPULATION_POLL_INTERVAL_SEC': '.4', 'NUWAX_SCENE_MAPPING_AUTO_APPLY': 'false'}
    for key, value in values.items(): monkeypatch.setenv(key, value)
    config = load_config()
    assert config.manipulation.enabled and config.manipulation.base_url == values['NUWAX_MANIPULATION_BASE_URL']
    assert (config.manipulation.http_timeout_sec, config.manipulation.stop_timeout_sec,
            config.manipulation.poll_interval_sec) == (4, 12, .4)
    assert config.scene_mapping.auto_apply is False


@pytest.mark.parametrize('key,value', [
    ('NUWAX_MANIPULATION_HTTP_TIMEOUT_SEC', '0'), ('NUWAX_MANIPULATION_HTTP_TIMEOUT_SEC', 'nan'),
    ('NUWAX_MANIPULATION_HTTP_TIMEOUT_SEC', '31'), ('NUWAX_MANIPULATION_STOP_TIMEOUT_SEC', '121'),
    ('NUWAX_MANIPULATION_POLL_INTERVAL_SEC', '6'), ('NUWAX_MANIPULATION_POLL_INTERVAL_SEC', '-1')])
def test_invalid_operation_config_is_rejected_during_load(monkeypatch, key, value):
    monkeypatch.setenv(key, value)
    with pytest.raises(ValueError): load_config()


@pytest.mark.parametrize('backend_mode', ['offline', 'slow'])
def test_gateway_catalogs_do_no_remote_io_and_bind_checks_once(backend_mode, device, tmp_path, monkeypatch):
    from services.manipulation.http_backend import DeviceBinding
    from gateways.mcp.protocol import _build_collection_tools
    entered, release, done = threading.Event(), threading.Event(), threading.Event()
    calls, errors = [], []
    def discover(self):
        calls.append(True)
        entered.set()
        if backend_mode == 'offline':
            raise DeviceError('controlled offline backend')
        if not release.wait(2):
            raise DeviceError('controlled backend still waiting')
        return DeviceBinding('session-a', 2)
    monkeypatch.setattr(HttpManipulationBackend, 'discover', discover)
    gateway = build_gateway(device, tmp_path)
    worker = None
    try:
        gateway.start()
        assert calls == []
        def catalogs():
            caps = gateway.list_capabilities()
            tools = gateway.list_tools()
            mcp = _build_collection_tools(gateway)
            descriptor = next(item for item in caps if item.name == NAME)
            assert '执行受理前实时验证' in descriptor.description
            assert NAME in json.dumps(mcp, ensure_ascii=False)
            assert any(item.descriptor.name == NAME and '目录不探测远端' in item.descriptor.description for item in tools)
            item = gateway.capability_registry.get_availability(NAME, gateway.robot.manifest.robot_model)
            assert item.supported and item.metadata['backend_verification'] == 'not_checked'
        catalogs()
        assert calls == []
        if backend_mode == 'offline':
            with pytest.raises(Exception, match='controlled offline backend'):
                gateway.invoke_tool(NAME, {'instruction': '苹果入盘', 'timeout_sec': 2})
            assert calls == [True]
            assert not device.started.is_set()
        else:
            def invoke():
                try:
                    gateway.invoke_tool(NAME, {'instruction': '苹果入盘', 'timeout_sec': 2})
                except Exception as error:
                    errors.append(error)
                finally:
                    done.set()
            worker = threading.Thread(target=invoke)
            worker.start()
            assert entered.wait(1)
            assert not done.is_set()
            catalogs()  # 真实 bind 正在等待后端时，目录仍可同步完成。
            assert calls == [True] and not done.is_set()
            release.set()
            worker.join(2)
            assert done.is_set() and not errors
            assert device.started.wait(1)
            assert calls == [True]
        catalogs()
        assert calls == [True]
    finally:
        release.set()
        if worker is not None: worker.join(3)
        gateway.stop()
