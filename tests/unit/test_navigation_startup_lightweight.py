"""合成配置中的导航推导与监督者环境隔离。"""
from copy import deepcopy
import json
import pytest
from core.config import read_config
from scripts.supervisor import load_environment


@pytest.mark.parametrize('pipeline,profile', [('lio_3d','pct-scan-3d'),('slam_2d','simple-2d')])
def test_startup_derives_profile_and_go2_limits(configuration_snapshot, pipeline, profile):
    store = configuration_snapshot
    store.save({'robot.driver':'go2', 'geometry.pipeline':pipeline,
                'go2.official_max_linear_velocity_mps':.05, 'go2.official_max_yaw_rate_rps':.7})
    from core.config import CODE_ROOT
    environment = load_environment(CODE_ROOT)
    values = read_config(store.home)['derived']
    assert values['nav_profile'] == profile
    assert values['actuator_max_linear_mps'] == .05
    assert values['actuator_max_yaw_rps'] == .7
    assert values['actuator_motion_model'] == ('omnidirectional' if pipeline == 'lio_3d' else '')
    assert not any(key.startswith(('_NUWAX_', 'GO2_')) for key in environment)


def test_generic_declaration_does_not_infer_braking(configuration_snapshot):
    store = configuration_snapshot
    store.save({'robot.driver':'customrobot','robot.actuator_motion_model':'omnidirectional',
                'robot.actuator_max_linear_mps':.3,'robot.actuator_max_yaw_rps':.3})
    result = read_config(store.home)
    assert result['derived']['actuator_max_linear_mps'] == .3
    assert result['derived']['actuator_max_yaw_rps'] == .3
    assert result['navigation']['capabilities_json'] == ''


@pytest.mark.parametrize('updates', [
    {'robot.actuator_max_linear_mps':.3}, {'robot.actuator_max_yaw_rps':.3},
    {'robot.actuator_motion_model':'omnidirectional'},
    {'robot.actuator_max_linear_mps':0.,'robot.actuator_max_yaw_rps':.3,'robot.actuator_motion_model':'omnidirectional'},
    {'robot.actuator_max_linear_mps':.3,'robot.actuator_max_yaw_rps':float('nan'),'robot.actuator_motion_model':'omnidirectional'},
    {'robot.actuator_max_linear_mps':.3,'robot.actuator_max_yaw_rps':.3,'robot.actuator_motion_model':'differential'},
])
def test_invalid_motion_does_not_publish_partial_composed_file(configuration_snapshot, updates):
    store = configuration_snapshot
    before = (store.home / 'run/config.toml').read_bytes()
    profile_before = store.profile_path(store.active()).read_bytes()
    with pytest.raises(ValueError):store.save(updates)
    assert (store.home / 'run/config.toml').read_bytes() == before
    assert store.profile_path(store.active()).read_bytes() == profile_before
