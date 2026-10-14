"""启动解析必须只依赖标准库，输出沿用同一配置优先级。"""
import json
import os
from pathlib import Path
import subprocess
import sys

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
