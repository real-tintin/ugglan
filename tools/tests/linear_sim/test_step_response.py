import numpy as np
import numpy.testing as npt

import ugglan_tools.linear_sim.step_response as step_response


def test_step_info():
    y = np.array([0, 1, 2, 3, 4, 3, 2, 3, 3])
    t = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8])

    step_info = step_response.step_info(t, y)

    npt.assert_almost_equal(step_info.rise_time, 3 * 0.8, decimal=2)  # 80 % of linear part (dy/dt = 1)
    npt.assert_almost_equal(step_info.peak, 4, decimal=2)
    npt.assert_almost_equal(step_info.overshoot, (4 / 3 - 1) * 1e2, decimal=2)
