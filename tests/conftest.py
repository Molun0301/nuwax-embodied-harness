"""新装配根、同协议几何服务及隔离安装配置的公共夹具。"""
import sys
import os
from pathlib import Path
import threading
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

# 收集阶段也只读取本轮测试安装根。
from tempfile import TemporaryDirectory
from copy import deepcopy
from core import config as configuration
_TEST_CONFIGURATION = TemporaryDirectory(prefix="d1-pytest-")
os.environ["NUWAX_INSTALL_HOME"] = _TEST_CONFIGURATION.name
_initial_store = configuration.ConfigStore(_TEST_CONFIGURATION.name)
_initial_store.create("测试")
_INITIAL_CONFIG = _initial_store.activate("测试")


def set_config(monkeypatch, key, value):
    section, name = key.split(".", 1)
    monkeypatch.setitem(configuration.runtime_config()[section], name, value)


def reset_config(monkeypatch, key):
    set_config(monkeypatch, key, deepcopy(configuration.specifications()[key]["default"]))


@pytest.fixture(autouse=True)
def configuration_snapshot(tmp_path_factory, monkeypatch):
    home = tmp_path_factory.mktemp("d1-config")
    store = configuration.ConfigStore(home)
    store.create("测试")
    values = store.activate("测试")
    monkeypatch.setenv("NUWAX_INSTALL_HOME", str(home))
    monkeypatch.setattr(configuration, "_RUNTIME", deepcopy(values))
    return store


def isolated_platform_config(directory):
    """隔离数据、模型与设备来源；不改变能力执行语义。"""
    from core.settings import load_config
    directory = Path(directory)
    config = load_config()
    config.base_dir = str(directory)
    config.robot.active_robot = "customrobot"
    config.relay.enabled = False
    config.runtime_data.map_catalog_root = str(directory / "maps")
    config.runtime_data.memory_embedding_model = "disabled"
    config.runtime_data.memory_image_embedding_model = "disabled"
    config.gateway.artifact_dir = str(directory / "artifacts")
    config.gateway.telemetry_enabled = False
    config.logging.log_dir = str(directory / "logs")
    config.logging.log_file = str(directory / "logs/gateway.log")
    config.perception.yolo_world.enabled = False
    config.perception.openai_vision.enabled = False
    config.perception.segmentation.enabled = False
    return config


@pytest.fixture
def platform_factory(tmp_path, monkeypatch):
    """真实构造与关闭平台；不恢复旧网关或任务管理接口。"""
    from gateways.platform import build_platform
    platforms = []
    def create(config=None, robot=None, start=False):
        config = config or isolated_platform_config(tmp_path / f"platform-{len(platforms)}")
        with monkeypatch.context() as patch:
            if robot is not None:
                patch.setattr("gateways.platform.create_active_robot_assembly", lambda _config: robot)
            platform = build_platform(config)
        platforms.append(platform)
        if start:
            platform.start()
        return platform
    yield create
    for platform in reversed(platforms):
        platform.stop()


@pytest.fixture
def geometry_service(tmp_path):
    """同一v3协议的进程内服务与真实客户端、订阅视图。"""
    from core.event_bus import EventBus
    from scripts.diagnostics.building_geometry import GeometrySimulator, clone
    from services.geometry.client import GeometryClient
    from services.geometry.maps import MapStore
    from services.geometry.state import GeometryStateView

    class Server(GeometrySimulator):
        def publish(self):
            with self.lock:
                self._publish()
        def publish_pose(self, pose):
            with self.lock:
                self.state.pose.CopyFrom(pose)
                self._publish()
        def publish_map(self, grid):
            with self.lock:
                for pending in self.subscribers["map"]:
                    self._offer(pending, clone(grid))

    events = EventBus()
    maps = MapStore(str(tmp_path / "geometry-maps"))
    with Server(maps) as server:
        client = GeometryClient(server.target)
        state = GeometryStateView(client, events)
        ready = threading.Event()
        unsubscribe = state.subscribe(lambda _value: ready.set())
        state.start()
        try:
            assert ready.wait(5), "假Geometry初始状态订阅未返回"
            yield SimpleNamespace(client=client, state=state, server=server, maps=maps)
        finally:
            unsubscribe()
            state.stop()
            client.close()
            events.shutdown()


@pytest.fixture
def perception_runtime(tmp_path):
    """探索夹具使用真实原因集合运行时，共享感知服务的模型隔离配置。"""
    from tests.test_a2c_perception_runtime import setup
    with setup(tmp_path / "探索感知") as value:
        value.source.current = None
        yield value.runtime


def prepare_business_release(prefix):
    """提供真实业务源码和当前测试解释器，不用空壳脚本替代探针。"""
    source = prefix / "source"
    source.mkdir(parents=True, exist_ok=True)
    for name in ("core", "contracts", "scripts", "drivers", "adapters"):
        target = source / name
        if not target.exists():
            target.symlink_to(ROOT / name, target_is_directory=True)
    virtualenv = prefix / "venv"
    if not virtualenv.exists():
        virtualenv.symlink_to(Path(sys.prefix), target_is_directory=True)
    return source
