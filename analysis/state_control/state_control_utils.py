import numpy as np
from dataclasses import dataclass
from scipy.integrate import solve_ivp

from state_space import StateSpace, DT


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


def find_ref_state_idx(x_r):
    idx = np.argwhere(x_r != 0)

    if len(idx) != 1:
        raise ValueError('Expected to find only one reference state.')

    return idx[0, 0]


def closed_loop_step_response(state_space: StateSpace, L: np.array,
                              x_0: np.array, x_r: np.array, t_end: float) -> (np.array, np.array, np.array, np.array):
    """
    Returns the step response of the closed-loop state
    feedback system i.e., u = -Lx.

    Returns the tuple (t, x, u, y).
    """
    x_0_h = x_0 - x_r  # Change of variable to track reference
    Ac = state_space.A - state_space.B.dot(L)  # Closed-loop system

    sol = solve_ivp(lambda t, x: Ac.dot(x), t_span=[0, t_end], y0=x_0_h[:, 0],
                    first_step=DT, max_step=DT)  # To use a fixed step size.
    t_out = sol.t
    x_out = sol.y

    u_out = -L.dot(x_out)[0]  # Assume SIMO
    x_out += x_r  # Change variable back
    y_out = state_space.C.dot(x_out)

    return t_out, x_out, u_out, y_out
