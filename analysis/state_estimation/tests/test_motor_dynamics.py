import numpy as np
import pytest

from motor_dynamics import time_const_from_steps, find_step_indices


@pytest.mark.parametrize("min_step_size, min_step_length_s, x, exp_indices",
                         [(2, 2, [0, 2, 2, -1, -1, -1, 0, -1, 0, -1], [(0, 2), (2, 5)]),
                          (8, 5, [5, 4, 3, 2, 10, 10, 10, 10, 10], [(3, 8)]),
                          (5, 1, [0, 5, 1, 1, 6, 0], [(0, 1), (3, 4)]),
                          (9, 9, [0, 2, 2, -1, -1, -1, 0, -1, 0, -1], [])])
def test_find_step_indices(min_step_size, min_step_length_s, x, exp_indices):
    t_s = np.arange(0, len(x), 1)
    x = np.array(x)

    indices = find_step_indices(t_s, x, min_step_size, min_step_length_s)
    assert set(indices) == set(exp_indices)


def test_time_const_from_steps():
    pos_tau = 0.123
    neg_tau = 0.321
    step_length_s = 3
    dt_s = 0.1

    steps = [0, 1, -1, 20, 12, 0, 100]
    min_step_size = 0.5
    min_step_length_s = step_length_s / 2

    noise_mu = 0
    noise_sigma = 0.01

    u, y = [], []
    for i_step in range(0, len(steps) - 1):
        u_0 = steps[i_step]
        u_1 = steps[i_step + 1]

        tau = pos_tau if u_0 < u_1 else neg_tau

        t_step = np.arange(0, step_length_s, dt_s)
        u_step = [u_0] + [u_1] * int(step_length_s / dt_s - 1)
        y_step = u_1 + (u_0 - u_1) * np.exp(-t_step / tau)

        u = np.concatenate([u, u_step])
        y = np.concatenate([y, y_step])

    t = np.arange(0, step_length_s * (len(steps) - 1), dt_s)
    y = y + np.random.normal(noise_mu, noise_sigma, len(u))

    (est_pos_tau, est_neg_tau) = time_const_from_steps(t, u, y, min_step_size, min_step_length_s)

    np.testing.assert_almost_equal(est_pos_tau, pos_tau, decimal=2)
    np.testing.assert_almost_equal(est_neg_tau, neg_tau, decimal=2)
