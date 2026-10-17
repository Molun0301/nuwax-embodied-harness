"""固定完整链的配置、保存事务和启动拒绝行为。"""

import os
from pathlib import Path
import subprocess
import sys

import pytest

from contracts.navigation_pipeline import resolve_navigation_profile
from core import config_io, config_registry
from core.config_values import read_env_values


ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize("pipeline,profile", [("lio_3d", "pct-scan-3d"), ("slam_2d", "simple-2d")])
def test_complete_pipeline_rejects_cross_dimension_and_experimental_profiles(pipeline, profile):
    assert resolve_navigation_profile(pipeline) == profile
    assert resolve_navigation_profile(pipeline, profile) == profile
    for other in {"simple-2d", "pct-scan-3d", "replanning-astar", "cmu-native", "exploration-native", "invalid"} - {profile}:
        with pytest.raises(ValueError, match="与执行链冲突"):
            resolve_navigation_profile(pipeline, other)


def test_console_exposes_one_complete_pipeline_choice():
    assert config_registry.get_field("NUWAX_NAV_PROFILE") is None
    field = config_registry.get_field("NUWAX_GEOMETRY_PIPELINE")
    assert set(field.choices) == {"lio_3d", "slam_2d"}
    assert all("探索" in label and "导航" in label for label in field.choice_labels)


@pytest.mark.parametrize("pipeline", [" slam_2d ", " ", "unknown"])
def test_invalid_pipeline_cannot_select_a_different_input_consumer(pipeline):
    with pytest.raises(ValueError):
        resolve_navigation_profile(pipeline)


@pytest.mark.parametrize("pipeline,profile", [("lio_3d", "pct-scan-3d"), ("slam_2d", "simple-2d")])
def test_switch_saves_pipeline_and_derived_navigation_in_one_atomic_file(tmp_path, monkeypatch, pipeline, profile):
    # 方案迁移后保存目标由机器人目录决定，测试必须隔离项目根，不能写实际活动方案。
    bootstrap = tmp_path / ".env"
    bootstrap.write_text("NUWAX_ACTIVE_ROBOT=go2\n")
    monkeypatch.setattr(config_io, "_CORE_ENV", bootstrap)
    monkeypatch.delenv("NUWAX_USER_CONFIG_ENV_FILE", raising=False)
    settings = tmp_path / "drivers/go2/configs/默认.env"
    settings.parent.mkdir(parents=True)
    (settings.parent.parent / "manifest.py").touch()
    (settings.parent.parent / "assembly.py").touch()
    settings.write_text("NUWAX_GEOMETRY_PIPELINE=lio_3d\nNUWAX_NAV_PROFILE=replanning-astar\n")
    writes = []
    original_write = config_io._atomic_write

    def capture_write(path, content):
        writes.append((path, content))
        return original_write(path, content)

    monkeypatch.setattr(config_io, "_atomic_write", capture_write)
    result = config_io.save_values({"NUWAX_GEOMETRY_PIPELINE": pipeline})
    assert not result.rejected
    assert len(writes) == 1
    assert set(result.written["robot_profile_env"]) == {"NUWAX_GEOMETRY_PIPELINE", "NUWAX_NAV_PROFILE"}
    assert read_env_values(settings)["NUWAX_NAV_PROFILE"] == profile
    assert read_env_values(settings)["NUWAX_GEOMETRY_PIPELINE"] == pipeline
    before = settings.read_bytes()
    rejected = config_io.save_values({"NUWAX_NAV_PROFILE": "replanning-astar"})
    assert "NUWAX_NAV_PROFILE" in rejected.rejected
    assert settings.read_bytes() == before


@pytest.mark.parametrize("pipeline,profile", [("lio_3d", "simple-2d"), ("slam_2d", "pct-scan-3d"), ("slam_2d", "replanning-astar")])
def test_launchers_reject_mismatch_before_creating_runtime(tmp_path, pipeline, profile):
    marker = tmp_path / "started"
    binary = tmp_path / "runtime"
    binary.write_text(f"#!/bin/sh\ntouch '{marker}'\n")
    binary.chmod(0o755)
    empty = tmp_path / "empty.env"
    empty.write_text("")
    env = {**os.environ, "NUWAX_GEOMETRY_PIPELINE": pipeline, "NUWAX_NAV_PROFILE": profile,
           "NUWAX_GEOMETRY_RUNTIME_BINARY": str(binary), "NUWAX_NAV_STACK_BINARY": str(binary),
           "NUWAX_BOOTSTRAP_ENV_FILE": str(empty), "NUWAX_USER_CONFIG_ENV_FILE": str(empty),
           "NUWAX_GEOMETRY_EXTRA_ENV_FILES": "", "NUWAX_GEOMETRY_WEB_CONFIG_ENV_FILE": str(empty),
           "NUWAX_GATEWAY_WEB_CONFIG_ENV_FILE": str(empty), "NUWAX_PYTHON_BIN": sys.executable}
    for launcher in ["start_nav_stack.sh", "start_geometry_runtime.sh"]:
        result = subprocess.run(["bash", str(ROOT / "scripts/systemd" / launcher)], env=env,
                                capture_output=True, text=True, timeout=10)
        assert result.returncode != 0
        assert "与执行链冲突" in result.stderr
        assert not marker.exists()


def test_pipeline_preflight_works_without_runtime_python_packages():
    env = {key: value for key, value in os.environ.items() if key not in {"NUWAX_NAV_PROFILE", "NUWAX_GEOMETRY_PIPELINE"}}
    result = subprocess.run(["/usr/bin/python3", "-S", str(ROOT / "scripts/resolve_navigation_pipeline.py")],
                            env=env, capture_output=True, text=True, check=True)
    assert result.stdout.strip() == "pct-scan-3d"


@pytest.mark.parametrize("modules,pipeline,profile", [
    (["core", "geometry", "navigation"], "lio_3d", "pct-scan-3d"),
    (["core", "geometry", "slam2d", "navigation"], "slam_2d", "simple-2d"),
])
def test_installer_derives_profile_from_selected_pipeline_without_pinning_legacy_value(tmp_path, modules, pipeline, profile):
    from scripts.module_installer.runtime import write_runtime

    prefix = tmp_path / "generation"
    prefix.mkdir()
    write_runtime(root=ROOT, prefix=prefix, modules=modules, robot="customrobot",
                  existing={"NUWAX_NAV_PROFILE": "replanning-astar"})
    saved = read_env_values(prefix / "settings.env")
    assert saved["NUWAX_GEOMETRY_PIPELINE"] == pipeline
    assert "NUWAX_NAV_PROFILE" not in saved
    assert resolve_navigation_profile(saved["NUWAX_GEOMETRY_PIPELINE"], saved.get("NUWAX_NAV_PROFILE")) == profile
    # A later pipeline switch must not remain pinned to the install-time profile.
    other_pipeline = "slam_2d" if pipeline == "lio_3d" else "lio_3d"
    other_profile = "simple-2d" if other_pipeline == "slam_2d" else "pct-scan-3d"
    assert resolve_navigation_profile(other_pipeline, saved.get("NUWAX_NAV_PROFILE")) == other_profile


def test_exploration_diagnostic_does_not_count_deadline_as_completion():
    from scripts.diagnostics.exploration_closed_loop import require_completed_exploration

    for limit in [None, 1]:
        with pytest.raises(AssertionError):
            require_completed_exploration("duration_exceeded", 1.0, limit)
    require_completed_exploration("max_goals_reached", 0.5, 1)
    require_completed_exploration("no_frontier_timeout", 0.99, None)
    with pytest.raises(AssertionError):
        require_completed_exploration("no_frontier_timeout", 0.7, None)


def test_navigation_startup_exports_the_existing_go2_speed_limit():
    """设备已有速度限制须传给导航，较低的现场覆盖同样生效。"""
    import os
    import subprocess
    import sys
    from pathlib import Path
    script = Path(__file__).resolve().parents[2] / 'scripts/resolve_navigation_pipeline.py'
    for value in ('0.35', '0.23'):
        result = subprocess.run([sys.executable, str(script), '--max-linear-speed'],
            env={**os.environ, 'NUWAX_ACTIVE_ROBOT': 'go2',
                 'GO2_OFFICIAL_MAX_LINEAR_VELOCITY_MPS': value},
            capture_output=True, text=True, check=True)
        assert float(result.stdout) == float(value)
    invalid = subprocess.run([sys.executable, str(script), '--max-linear-speed'],
        env={**os.environ, 'NUWAX_ACTIVE_ROBOT': 'go2',
             'GO2_OFFICIAL_MAX_LINEAR_VELOCITY_MPS': 'nan'}, capture_output=True, text=True)
    assert invalid.returncode != 0


def test_navigation_startup_exports_yaw_limit_separately_from_translation():
    """设备角速度直接使用自身设置，不能跟随线速度改变单位。"""
    import os,subprocess,sys
    from pathlib import Path
    script=Path(__file__).resolve().parents[2]/'scripts/resolve_navigation_pipeline.py'
    result=subprocess.run([sys.executable,str(script),'--max-yaw-rate'],env={**os.environ,
        'NUWAX_ACTIVE_ROBOT':'go2','GO2_OFFICIAL_MAX_LINEAR_VELOCITY_MPS':'0.05',
        'GO2_OFFICIAL_MAX_YAW_RATE_RPS':'0.8'},capture_output=True,text=True,check=True)
    assert float(result.stdout)==.8


def test_legacy_mls_profile_migrates_to_current_3d_navigation():
    assert resolve_navigation_profile("lio_3d", "mls-3d") == "pct-scan-3d"
    with pytest.raises(ValueError):
        resolve_navigation_profile("slam_2d", "mls-3d")
