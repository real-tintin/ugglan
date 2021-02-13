from dataclasses import dataclass

import numpy as np
from scipy.integrate import solve_ivp

from .physical_const import *
from .state_space import State, get_state_space, get_c


@dataclass
class StepInfo:
    rise_time: float
    peak: float
    overshoot: float


def step_info(t: np.array, y: np.array, upsample_dt: float = 0.01) -> StepInfo:
    """
    Returns step info, see StepInfo. Upsamples y
    to increase it's resolution.
    """
    ts = np.arange(t[0], t[-1], upsample_dt)
    ys = np.interp(ts, t, y)

    yn = ys / ys[-1]

    ind_10p = np.argwhere(yn <= 0.1)[-1, 0]
    ind_90p = np.argwhere(yn >= 0.9)[0, 0]
    rise_time = ts[ind_90p] - ts[ind_10p]

    peak = np.max(ys)
    overshoot = (peak / ys[-1] - 1) * 1e2

    return StepInfo(rise_time, peak, overshoot)


def _u_est_reduced_observer(x, u_0, c, alpha, L, is_intg_state) -> np.array:
    beta_4 = TAU + DT * (1 + TAU * alpha * c)
    beta_1 = TAU / beta_4
    beta_2 = -alpha * DT * (1 + alpha * TAU * c) / beta_4
    beta_3 = DT / beta_4

    _, n = x.shape
    u_est = np.zeros(n)
    u_est[0] = u_0

    if not is_intg_state:  # Add dummy states
        x = np.concatenate((np.zeros((1, n)), x), axis=0)
        L = np.concatenate([[[0]], L], axis=1)

    for k in range(1, n):
        u_est[k] = -(L[0, 0] * x[0, k] + L[0, 1] * x[1, k] + (L[0, 2] + L[0, 3] * (beta_2 + alpha)) * x[2, k] +
                     L[0, 3] * beta_1 * (u_est[k - 1] - alpha * x[2, k - 1])) / (1 + L[0, 3] * beta_3)

    return u_est


def closed_loop(state: State, L: np.array, alpha: float,
                x_0: np.array, x_r: np.array,
                t_end: float, add_intg_state: bool = False) -> (np.array, np.array, np.array, np.array, np.array):
    """
    Returns the step response of the closed-loop state feedback
    system i.e., u = -Lx.

    Also implements the reduced observer estimating u_est, see
    doc for details. Note, u_est is atm not used as feedback.

    Returns the tuple (t, x, y, u, u_est).
    """
    A, B, C, _ = get_state_space(state, add_intg_state)
    Ac = A - B.dot(L)  # Closed-loop system

    x_0_h = x_0 - x_r  # Change of variable to track reference
    sol = solve_ivp(lambda t, x: Ac.dot(x), t_span=[0, t_end], y0=x_0_h[:, 0],
                    first_step=DT, max_step=DT)  # To use a fixed step size.
    t = sol.t
    x = sol.y

    u = -L.dot(x)[0]  # Assume SIMO
    u_0 = u[0]  # Be kind to observer
    u_est = _u_est_reduced_observer(x, u_0, get_c(state), alpha, L, add_intg_state)

    x += x_r  # Change variable back
    y = C.dot(x)

    return t, x, y, u, u_est
