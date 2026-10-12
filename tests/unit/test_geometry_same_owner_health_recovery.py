"""瞬时探测失败只能复用同一Rust启动代际，不能切断既有传感器桥。"""
import threading
from types import SimpleNamespace
import pytest
from services.geometry.geometry_runtime_client import GeometryRuntimeSubprocess
from services.geometry.runtime import PlatformGeometryRuntime
from services.geometry.runtime_input_bridge import RuntimeInputBridge


def attached(monkeypatch):
    client = GeometryRuntimeSubprocess(port=50123)
    source = {'pid':12345, 'generation':71, 'healthy':True}
    monkeypatch.setattr(client, '_is_port_open', lambda: True)
    def ready():
        if client._attached_ready_pid not in (None, source['pid']):return False
        client._attached_ready_pid = source['pid']
        return True
    def probe():
        client._last_probed_runtime_generation = source['generation'] if source['healthy'] else None
        return source['healthy']
    monkeypatch.setattr(client, '_verify_ready_locked', ready)
    monkeypatch.setattr(client, '_verify_attached_runtime_locked', probe)
    monkeypatch.setattr(client, '_warn_if_video_relay_missing', lambda: None)
    assert client.start()
    return client, source


def test_transient_probe_reuses_provider_bound_to_existing_input_bridge(monkeypatch):
    client, source = attached(monkeypatch)
    class Provider:
        is_connected = True
        _stopping = False
        def __init__(self):self.outputs = []
        def publish_point_cloud(self, stamp):
            assert not self._stopping
            self.outputs.append(stamp)
            return True
    provider = Provider()
    bridge = RuntimeInputBridge(runtime_provider=provider)
    runtime = object.__new__(PlatformGeometryRuntime)
    runtime._runtime_state_lock = threading.RLock()
    runtime._provider_attach_lock = threading.RLock()
    runtime._runtime_started = True
    runtime._provider_attach_stopping = False
    runtime._remote_provider = provider
    runtime._geometry_runtime_subprocess = client
    runtime._attach_geometry_runtime_provider = lambda *_:pytest.fail('同一代际不应创建第二provider')
    bridge.runtime_provider.publish_point_cloud(100)
    client._attached_health_check_at = 0
    probe = client._verify_attached_runtime_locked
    calls = []
    def one_failure():
        calls.append(True)
        source['healthy'] = len(calls) > 1
        return probe()
    monkeypatch.setattr(client, '_verify_attached_runtime_locked', one_failure)
    runtime._start_geometry_runtime_subprocess()
    assert len(calls) == 2
    assert runtime.get_geometry_runtime_provider() is bridge.runtime_provider is provider
    assert runtime._geometry_runtime_subprocess is client
    assert client.is_running
    bridge.runtime_provider.publish_point_cloud(200)
    assert provider.outputs == [100, 200]


@pytest.mark.parametrize('change', ['generation', 'pid', 'unavailable_generation'])
def test_recovery_requires_confirmed_same_owner_not_just_same_numeric_pid(monkeypatch, change):
    client, source = attached(monkeypatch)
    source['healthy'] = False
    client._attached_health_check_at = 0
    assert not client.is_running
    source['healthy'] = True
    if change == 'generation':source['generation'] += 1
    elif change == 'pid':source['pid'] += 1
    else:source['generation'] = None
    assert not client.recover_same_runtime()
    assert not client._attached


@pytest.mark.parametrize('operation', ['stop', 'disconnect'])
def test_explicit_retirement_cannot_be_recovered(monkeypatch, operation):
    client, _ = attached(monkeypatch)
    getattr(client, operation)()
    monkeypatch.setattr(client, '_verify_attached_runtime_locked', lambda:pytest.fail('退役身份不能再验活复用'))
    assert not client.recover_same_runtime()


def test_failed_probe_stays_unhealthy_without_replacing_identity(monkeypatch):
    client, source = attached(monkeypatch)
    source['healthy'] = False
    client._attached_health_check_at = 0
    assert not client.is_running
    assert not client.recover_same_runtime()
    assert not client._attached
    assert client._bound_runtime_generation == 71


def test_owner_change_during_recovery_probe_is_rejected(monkeypatch):
    client, source = attached(monkeypatch)
    source['healthy'] = False
    client._attached_health_check_at = 0
    assert not client.is_running
    def changed_during_rpc():
        client._last_probed_runtime_generation = 71
        source['pid'] += 1
        return True
    monkeypatch.setattr(client, '_verify_attached_runtime_locked', changed_during_rpc)
    assert not client.recover_same_runtime()


def test_rpc_identity_comes_from_runtime_health_response(monkeypatch):
    import grpc
    from services.geometry.proto_gen import geometry_runtime_pb2_grpc as rpc
    closed = []
    channel = SimpleNamespace(close=lambda:closed.append(True))
    monkeypatch.setattr(grpc, 'insecure_channel', lambda *_:channel)
    monkeypatch.setattr(rpc, 'GeometryRuntimeStub', lambda _:SimpleNamespace(
        GetStatus=lambda *a,**kw:SimpleNamespace(phase='running'),
        HealthCheck=lambda *a,**kw:SimpleNamespace(runtime_generation=987654321),
    ))
    client = GeometryRuntimeSubprocess()
    assert client._verify_attached_runtime_locked()
    assert client._last_probed_runtime_generation == 987654321
    assert closed == [True]
