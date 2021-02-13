import numpy as np
import pytest

import multi_body.euclidean_transform as et


@pytest.mark.parametrize("v, x_t, y_t, z_t, exp_t", [
    ([0, 0, 0], 0, 0, 0, [0, 0, 0]),
    ([0, 0, 0], 1, 1, 1, [1, 1, 1]),
    (np.identity(3), 0, 0, 0, np.identity(3)),
])
def test_translation(v, x_t, y_t, z_t, exp_t):
    act_t = et.translate(v, x_t, y_t, z_t)
    np.testing.assert_array_almost_equal(exp_t, act_t)


@pytest.mark.parametrize("v, x_s, y_s, z_s, exp_s", [
    ([0, 0, 0], 0, 0, 0, [0, 0, 0]),
    ([1, 0, 1], 0.5, 1, -10, [0.5, 0, -10]),
    (np.identity(3), 1, 1, 1, np.identity(3)),
])
def test_scale(v, x_s, y_s, z_s, exp_s):
    act_s = et.scale(v, x_s, y_s, z_s)
    np.testing.assert_array_almost_equal(exp_s, act_s)


@pytest.mark.parametrize("v, x_rad, y_rad, z_rad, exp_r", [
    ([0, 0, 0], 0, 0, 0, [0, 0, 0]),
    ([1, -1, 1], np.pi, -np.pi, np.pi / 2, [-1, -1, 1]),
    (np.identity(3), 0, 0, 0, np.identity(3)),
])
def test_rotate(v, x_rad, y_rad, z_rad, exp_r):
    act_r = et.rotate(v, x_rad, y_rad, z_rad)
    np.testing.assert_array_almost_equal(exp_r, act_r)
