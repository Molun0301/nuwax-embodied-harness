"""独立射线原点的配置边界与实机Go2默认标定回归。"""
import json

import numpy as np
import pytest

from adapters.sensors.go2_calibration import GO2_CALIBRATION_PROFILE, go2_calibration_profile_from_config
from core.config_io import _valid_rotation_calibration


@pytest.mark.parametrize("origin", [[0.31502, 0, -0.089145], [0, 0], None, [True, 0, 0], [float("nan"), 0, 0]])
def test_optional_ray_origin_is_validated_before_readiness(origin):
    config = dict(translation_m=[0, 0, 0], rotation_row_major=[1, 0, 0, 0, 1, 0, 0, 0, 1])
    assert _valid_rotation_calibration(json.dumps(config))
    config["ray_origin_in_input_m"] = origin
    assert _valid_rotation_calibration(json.dumps(config)) is (origin == [0.31502, 0, -0.089145])


def test_real_go2_default_still_applies_physical_mount_once():
    profile = go2_calibration_profile_from_config()
    assert profile == GO2_CALIBRATION_PROFILE
    np.testing.assert_allclose(profile.transform_lidar_xyz([[0, 0, 0]])[0], [.28945, 0, -.046825])
    np.testing.assert_allclose(profile.transform_lidar_xyz([[0, 0, 1]])[0],
        profile.lidar_rotation[:, 2] + [.28945, 0, -.046825])
