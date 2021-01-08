import numpy as np

from state_space import get_c, State, TAU, DT


def red_observer(state: State, x_v: np.array, u: np.array,
                 alpha: float, z_0: float = 0) -> np.array:
    """
    Reduced observer estimating the last state
    of A i.e., force or torque. See doc for details.

    The state x_v here is the second to last state
    i.e., angular-rate or velocity.
    """
    c = get_c(state)

    z = np.zeros(len(u))
    z[0] = z_0

    for i in range(1, len(u)):
        z[i] = ((u[i] - alpha * (1 + TAU * alpha * c) * x_v[i]) * DT + z[i - 1] * TAU) / (
                TAU + DT * (1 + TAU * alpha * c))

    return z + alpha * x_v
