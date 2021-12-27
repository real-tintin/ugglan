import numpy as np
import pytest

import misc_scripts.motor_dynamics as md


@pytest.mark.parametrize("min_step_size, min_step_length_s, x, exp_indices",
                         [(2, 2, [0, 2, 2, -1, -1, -1, 0, -1, 0, -1], [(0, 2), (2, 5)]),
                          (8, 5, [5, 4, 3, 2, 10, 10, 10, 10, 10], [(3, 8)]),
                          (5, 1, [0, 5, 1, 1, 6, 0], [(0, 1), (3, 4)]),
                          (9, 9, [0, 2, 2, -1, -1, -1, 0, -1, 0, -1], [])])
def test_find_step_indices(min_step_size, min_step_length_s, x, exp_indices):
    t_s = np.arange(0, len(x), 1)
    x = np.array(x)

    indices = md._find_step_indices(t_s, x, min_step_size, min_step_length_s)
    assert set(indices) == set(exp_indices)
