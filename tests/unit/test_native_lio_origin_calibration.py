"""独立射线原点的配置边界与实机Go2默认标定回归。"""
import json

import numpy as np
import pytest

from adapters.sensors.go2_calibration import GO2_CALIBRATION_PROFILE, go2_calibration_profile_from_config


def test_real_go2_default_still_applies_physical_mount_once():
    profile = go2_calibration_profile_from_config()
    assert profile == GO2_CALIBRATION_PROFILE
    np.testing.assert_allclose(profile.transform_lidar_xyz([[0, 0, 0]])[0], [.28945, 0, -.046825])
    np.testing.assert_allclose(profile.transform_lidar_xyz([[0, 0, 1]])[0],
        profile.lidar_rotation[:, 2] + [.28945, 0, -.046825])
