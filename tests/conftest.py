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

def create_robot_profile(root, text="", *, robot="go2", name="默认"):
    """按两种安装共用的写法准备方案、选择器与活动链接。"""
    from scripts.module_installer.runtime import write_profile

    root = Path(root)
    directory = root / "drivers" / robot
    directory.mkdir(parents=True, exist_ok=True)
    for filename in ("manifest.py", "assembly.py"):
        (directory / filename).touch()
    prefix = root / "测试配置安装"
    prefix.mkdir(exist_ok=True)
    (prefix / "settings.env").write_text(text, encoding="utf-8")
    write_profile(prefix, directory / "configs", name, root / "runtime_data/settings")
    return directory / "configs" / (name + ".env")


@pytest.fixture
def navigation_startup_root(tmp_path, monkeypatch):
    """在已安装方案的临时源码根验证真实启动解析器。"""
    import shutil

    for key in tuple(os.environ):
        if key.startswith(("NUWAX_", "GO2_", "_NUWAX_")):
            monkeypatch.delenv(key)
    root = tmp_path / "启动配置"
    for relative in (
        "scripts/supervisor.py",
        "services/navigation/proto/motion_guard_socket_relative.txt",
        "scripts/lib/config_env.sh", "scripts/resolve_navigation_pipeline.py",
        "scripts/resolve_robot_geometry.py", "contracts/navigation_pipeline.py",
        "core/config_values.py", "core/config_catalog.json", "core/robot_geometry.py",
        "core/settings.py", "drivers/go2/settings.py", ".env.example",
    ):
        target = root / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT / relative, target)
    (root / "core/__init__.py").write_text("", encoding="utf-8")
    create_robot_profile(root, robot="customrobot")
    create_robot_profile(root, "NUWAX_ACTIVE_ROBOT=go2\n")
    return root



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
