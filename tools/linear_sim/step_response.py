from dataclasses import dataclass

import numpy as np
from scipy.integrate import solve_ivp

from .state_space import State, get_state_space


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


def closed_loop(state: State, L: np.array,
                x_0: np.array, x_r: np.array,
                t_end: float, add_intg_state: bool = False,
                sample_time=0.02) -> (np.array, np.array, np.array, np.array, np.array):
    """
    Returns the step response of the closed-loop state feedback
    system i.e., u = -Lx.

    Returns the tuple (t, x, y, u, u_est).
    """
    A, B, C, _ = get_state_space(state, add_intg_state)
    Ac = A - B.dot(L)  # Closed-loop system

    x_0_h = x_0 - x_r  # Change of variable to track reference
    sol = solve_ivp(lambda t, x: Ac.dot(x), t_span=[0, t_end], y0=x_0_h[:, 0],
                    first_step=sample_time, max_step=sample_time)  # To use a fixed step size.
    t = sol.t
    x = sol.y

    u = -L.dot(x)[0]  # Assume SIMO
    x += x_r  # Change variable back
    y = C.dot(x)

    return t, x, y, u
