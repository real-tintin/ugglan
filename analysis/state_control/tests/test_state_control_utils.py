import numpy.testing as npt
import pytest

from state_control_utils import *
from state_space import get_state_space, State


def test_step_info():
    y = np.array([0, 1, 2, 3, 4, 3, 2, 3, 3])
    t = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8])

    si = step_info(t, y)

    npt.assert_almost_equal(si.rise_time, 3 * 0.8, decimal=2)  # 80 % of linear part (dy/dt = 1)
    npt.assert_almost_equal(si.peak, 4, decimal=2)
    npt.assert_almost_equal(si.overshoot, (4 / 3 - 1) * 1e2, decimal=2)


def test_find_ref_state_idx():
    assert find_ref_state_idx(np.array([0, 0, 1, 0])) == 2

    with pytest.raises(ValueError):
        find_ref_state_idx(np.array([]))
        find_ref_state_idx(np.array([0, 0, 0]))


def test_closed_loop_step_response():
    closed_loop_step_response(state_space=get_state_space(State.PHI),
                              L=np.array([[1, 2, 3]]),
                              x_0=np.zeros((3, 1)),
                              x_r=np.array([[0, 0, 1]]).transpose(),
                              t_end=1)
