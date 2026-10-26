"""启动解析必须只依赖标准库，输出沿用同一配置优先级。"""
import json
import os
from pathlib import Path
import subprocess
import sys
import shlex

import pytest

ROOT=Path(__file__).resolve().parents[2]


def invoke(values):
    return subprocess.run([sys.executable,'-S',str(ROOT/'scripts/resolve_navigation_pipeline.py'),'--runtime-env'],
        env={**os.environ,'NUWAX_ACTIVE_ROBOT':'go2','NUWAX_GEOMETRY_PIPELINE':'lio_3d','NUWAX_NAV_PROFILE':'mls-3d',
             'GO2_OFFICIAL_MAX_LINEAR_VELOCITY_MPS':'0.05','GO2_OFFICIAL_MAX_YAW_RATE_RPS':'0.8',**values},
        capture_output=True,text=True)


def test_startup_assembly_without_site_packages():
    result=invoke({})
    assert result.returncode==0,result.stderr
    assert dict(line.split('=',1) for line in result.stdout.splitlines())=={
        'NUWAX_NAV_PROFILE':'pct-scan-3d','_NUWAX_NAV_ACTUATOR_MAX_LINEAR_MPS':'0.05','_NUWAX_NAV_ACTUATOR_MAX_YAW_RPS':'0.8','_NUWAX_NAV_ACTUATOR_MOTION_MODEL':'omnidirectional'}


@pytest.mark.parametrize('values',[
    {'NUWAX_NAV_PROFILE':'simple-2d'},
    {'GO2_OFFICIAL_MAX_YAW_RATE_RPS':'nan'},
    {'GO2_OFFICIAL_MAX_LINEAR_VELOCITY_MPS':'0'},
])
def test_bad_assembly_never_exports_partial_values(values):
    result=invoke(values)
    assert result.returncode!=0
    assert result.stdout==''


def test_actual_source_script_exports_combined_device_limits():
    result=subprocess.run(['bash','-c',
        'source services/geometry/source_runtime_env.sh && printf "%s,%s,%s,%s" "$NUWAX_NAV_PROFILE" "$_NUWAX_NAV_ACTUATOR_MAX_LINEAR_MPS" "$_NUWAX_NAV_ACTUATOR_MAX_YAW_RPS" "$_NUWAX_NAV_ACTUATOR_MOTION_MODEL"'],
        cwd=ROOT,env={**os.environ,'NUWAX_ACTIVE_ROBOT':'go2','NUWAX_PYTHON_BIN':sys.executable,
        'NUWAX_GEOMETRY_PIPELINE':'lio_3d','NUWAX_NAV_PROFILE':'mls-3d',
        'GO2_OFFICIAL_MAX_LINEAR_VELOCITY_MPS':'0.05','GO2_OFFICIAL_MAX_YAW_RATE_RPS':'0.7'},
        capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    assert result.stdout=='pct-scan-3d,0.05,0.7,omnidirectional'


@pytest.mark.parametrize('values',[{'NUWAX_ACTIVE_ROBOT':'customrobot'}, {'NUWAX_GEOMETRY_PIPELINE':'slam_2d','NUWAX_NAV_PROFILE':'simple-2d'}])
def test_basic_omnidirectional_channels_are_not_inferred_for_other_profiles(values):
    result=invoke(values)
    assert result.returncode==0,result.stderr
    assert dict(line.split('=',1) for line in result.stdout.splitlines())['_NUWAX_NAV_ACTUATOR_MOTION_MODEL']==''


EXPLICIT_MOTION = {
    'NUWAX_ACTIVE_ROBOT': 'customrobot',
    'NUWAX_NAV_ACTUATOR_MOTION_MODEL': 'omnidirectional',
    'NUWAX_NAV_ACTUATOR_MAX_LINEAR_MPS': '0.3',
    'NUWAX_NAV_ACTUATOR_MAX_YAW_RPS': '0.3',
}


def test_generic_explicit_motion_is_consumed_without_go2_identity_or_braking():
    result = invoke(EXPLICIT_MOTION)
    assert result.returncode == 0, result.stderr
    values = dict(line.split('=', 1) for line in result.stdout.splitlines())
    assert values['_NUWAX_NAV_ACTUATOR_MAX_LINEAR_MPS'] == '0.3'
    assert values['_NUWAX_NAV_ACTUATOR_MAX_YAW_RPS'] == '0.3'
    assert values['_NUWAX_NAV_ACTUATOR_MOTION_MODEL'] == 'omnidirectional'
    assert not any('BRAK' in key or 'CAPABILITIES' in key for key in values)


@pytest.mark.parametrize('override', [
    {'NUWAX_NAV_ACTUATOR_MOTION_MODEL': ''},
    {'NUWAX_NAV_ACTUATOR_MAX_LINEAR_MPS': ''},
    {'NUWAX_NAV_ACTUATOR_MAX_YAW_RPS': 'nan'},
    {'NUWAX_NAV_ACTUATOR_MAX_LINEAR_MPS': '5.01'},
    {'NUWAX_NAV_ACTUATOR_MAX_YAW_RPS': '13'},
    {'NUWAX_NAV_ACTUATOR_MOTION_MODEL': 'ackermann'},
])
def test_incomplete_or_unsupported_generic_motion_exports_nothing(override):
    result = invoke({**EXPLICIT_MOTION, **override})
    assert result.returncode != 0
    assert result.stdout == ''


def test_actual_startup_exports_generic_declared_motion():
    result = subprocess.run(['bash', '-c',
        'source services/geometry/source_runtime_env.sh && printf "%s,%s,%s" '
        '"$_NUWAX_NAV_ACTUATOR_MAX_LINEAR_MPS" "$_NUWAX_NAV_ACTUATOR_MAX_YAW_RPS" '
        '"$_NUWAX_NAV_ACTUATOR_MOTION_MODEL"'], cwd=ROOT,
        env={**os.environ, **EXPLICIT_MOTION, 'NUWAX_PYTHON_BIN': sys.executable,
             'NUWAX_GEOMETRY_PIPELINE': 'lio_3d', 'NUWAX_NAV_PROFILE': 'pct-scan-3d'},
        capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert result.stdout == '0.3,0.3,omnidirectional'


_TERRAIN_SOURCE = "NUWAX_NAV_PCT_TERRAIN_SOURCE"


def _terrain_startup(tmp_path, file_value=None):
    """运行真实导航启动器，以取值后退出的子进程核对环境和参数。"""
    settings = tmp_path / "settings.env"
    settings.write_text("" if file_value is None else f"{_TERRAIN_SOURCE}={file_value}\n")
    output = tmp_path / "导航子进程.json"
    binary = tmp_path / "nav-runtime"
    binary.write_text(
        f"#!{sys.executable}\n"
        "import json, os, sys\n"
        "from pathlib import Path\n"
        "if sys.argv[1:] == ['--launcher-contract']:\n"
        "    print('nuwax-nav-stack-internal-ready-v1')\n"
        "    raise SystemExit(0)\n"
        f"Path({str(output)!r}).write_text(json.dumps({{'terrain': os.environ.get({_TERRAIN_SOURCE!r}), 'argv': sys.argv[1:]}}))\n"
        "raise SystemExit(23)\n")
    binary.chmod(0o755)
    env = {key: value for key, value in os.environ.items()
           if not key.startswith(("NUWAX_", "GO2_", "_NUWAX_"))}
    env.update({
        "NUWAX_ACTIVE_ROBOT": "go2", "NUWAX_GEOMETRY_PIPELINE": "lio_3d",
        "NUWAX_NAV_PROFILE": "pct-scan-3d", "NUWAX_PYTHON_BIN": sys.executable,
        "NUWAX_NAV_STACK_BINARY": str(binary),
        "NUWAX_NAV_STACK_READY_FILE": str(tmp_path / "nav.ready"),
        "NUWAX_BOOTSTRAP_ENV_FILE": str(settings), "NUWAX_USER_CONFIG_ENV_FILE": str(settings),
        "GO2_DATA_PLANE_ENV_FILE": str(settings),
        "NUWAX_GEOMETRY_WEB_CONFIG_ENV_FILE": str(settings),
        "NUWAX_GATEWAY_WEB_CONFIG_ENV_FILE": str(settings),
        "NUWAX_GEOMETRY_EXTRA_ENV_FILES": "",
    })
    return env, settings, output


def _run_terrain_launcher(env, output, command=None):
    result = subprocess.run(command or ["bash", str(ROOT / "scripts/systemd/start_nav_stack.sh")],
                            cwd=ROOT, env=env, capture_output=True, text=True, timeout=10)
    # 子进程取值完成后按约定退出；启动器必须保留此失败，不能伪造就绪。
    assert result.returncode != 0
    assert "runtime exited during ownership/health warmup with status 23" in result.stderr
    assert not Path(env["NUWAX_NAV_STACK_READY_FILE"]).exists()
    captured = json.loads(output.read_text())
    assert captured["argv"] == ["--profile=pct-scan-3d"]
    return captured["terrain"]


@pytest.mark.parametrize("file_value,environment_value,expected", [
    (None, None, "tomography"),
    ("''", None, "tomography"),
    ("mls", None, "mls"),
    ("tomography", "mls", "mls"),
    ("mls", " ", "tomography"),
    ("invalid", None, "invalid"),
])
def test_navigation_child_receives_pct_terrain_source(tmp_path, file_value, environment_value, expected):
    env, _settings, output = _terrain_startup(tmp_path, file_value)
    if environment_value is not None:
        env[_TERRAIN_SOURCE] = environment_value
    assert _run_terrain_launcher(env, output) == expected


def test_inherited_pct_terrain_default_does_not_hide_later_saved_value(tmp_path):
    env, settings, output = _terrain_startup(tmp_path)
    # 父启动器先补默认，文件随后修改；子启动器必须重读新值。
    command = ["bash", "-c",
               "source services/geometry/source_runtime_env.sh && "
               f"printf '%s\\n' '{_TERRAIN_SOURCE}=mls' > {shlex.quote(str(settings))} && "
               "exec bash scripts/systemd/start_nav_stack.sh"]
    assert _run_terrain_launcher(env, output, command) == "mls"
