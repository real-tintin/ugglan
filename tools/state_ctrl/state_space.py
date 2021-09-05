from enum import Enum

import numpy as np

from .physical_const import *


class State(Enum):
    X = 1  # translation in x
    Y = 2  # translation in y
    Z = 3  # translation in z

    PHI = 4  # rotation about x
    THETA = 5  # rotation about y
    PSI = 6  # rotation about z


_A = np.array([[0, 1, 0],
               [0, 0, 1],
               [0, 0, -1 / TAU_M]])

_B = lambda c: np.array([[0, 0, c / TAU_M]]).transpose()

_C = np.eye(3)

_D = 0

_A_INTG = np.array([[0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1],
                    [0, 0, 0, -1 / TAU_M]])

_B_INTG = lambda c: np.array([[0, 0, 0, c / TAU_M]]).transpose()

_C_INTG = np.eye(4)


def get_state_space(state: State, add_intg_state: bool = False) -> \
        (np.array, np.array, np.array, np.array):
    """
    Returns a StateSpace (A, B, C, D) of the open-loop
    drone's dynamics given a State.

    Use add_intg_state to add an integrating state.
    """
    c = _get_c(state)

    if add_intg_state:
        return _A_INTG, _B_INTG(c), _C_INTG, _D
    else:
        return _A, _B(c), _C, _D


def _get_c(state: State) -> float:
    if state == State.X or state == State.Y or state == State.Z:
        c = 1 / MASS
    elif state == State.PHI:
        c = 1 / I_XX
    elif state == State.THETA:
        c = 1 / I_YY
    elif state == State.PSI:
        c = 1 / I_ZZ
    else:
        raise ValueError

    return c
