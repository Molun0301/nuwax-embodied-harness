"""仿真入口复用正式应用生命周期；不启动服务器、不调用动作后端。"""
import asyncio
import sys
from pathlib import Path

from scripts import run_g1_sim_gateway


def test_sim_entry_reuses_gateway_lifespan_and_isolated_data(tmp_path, monkeypatch):
    import uvicorn
    from services.manipulation.http_backend import HttpManipulationBackend
    output = tmp_path / 'sim'
    calls = []
    def forbidden_discover(self):
        raise AssertionError('构造、启动与目录不应探测设备')
    monkeypatch.setattr(HttpManipulationBackend, 'discover', forbidden_discover)
    def run(app, *, host, port):
        calls.append((host, port))
        config = app.state.config
        runtime = app.state.runtime
        assert config.robot.active_robot == 'customrobot'
        assert runtime.robot.manifest.robot_name == 'g1_sim'
        assert not config.geometry.enabled
        assert not config.perception.yolo_world.enabled
        assert config.runtime_data.memory_embedding_model == 'disabled'
        assert Path(config.runtime_data.map_catalog_root) == output / 'maps'
        assert Path(config.runtime_data.memory_db_path) == output / 'memory.db'
        assert Path(config.gateway.artifact_dir) == output / 'artifacts'
        assert config.manipulation.enabled
        assert config.manipulation.base_url == 'http://127.0.0.1:9'
        async def lifecycle():
            async with app.router.lifespan_context(app):
                assert any(item.name == 'execute_manipulation_task' for item in runtime.list_capabilities())
                assert app.state.agent_tools.store.path.parent == output
        asyncio.run(lifecycle())
        assert runtime._dispatch_shutdown_complete
        assert runtime.manipulation_service.backend.client.is_closed
    monkeypatch.setattr(uvicorn, 'run', run)
    monkeypatch.setattr(sys, 'argv', ['run_g1_sim_gateway.py', '--output', str(output),
        '--port', '18912', '--device-url', 'http://127.0.0.1:9'])
    run_g1_sim_gateway.main()
    assert calls == [('127.0.0.1', 18912)]
