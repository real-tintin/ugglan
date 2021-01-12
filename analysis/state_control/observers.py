import numpy as np

from physical_const import *


def reduced_observer(x_v: np.array, u: np.array, c: float,
                     alpha: float, z_0: float = 0) -> np.array:
    """
    Reduced observer estimating the last state of A
    i.e., force or torque. See doc for details.

    The state x_v here is the second to last state
    i.e., angular-rate or velocity.
    """
    z = np.zeros(len(x_v))
    z[0] = z_0

    for i in range(1, len(z)):
        z[i] = ((u[i] - alpha * (1 + TAU * alpha * c) * x_v[i]) * DT + z[i - 1] * TAU) / (
                TAU + DT * (1 + TAU * alpha * c))

    return z + alpha * x_v
