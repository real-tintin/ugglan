from enum import Enum

import numpy as np
from dataclasses import dataclass

MASS = 1.038  # [kg]

I_XX = 0.014  # [kgm^2]
I_YY = 0.014  # [kgm^2]
I_ZZ = 0.026  # [kgm^2]

TAU = 0.08

DT = 0.02  # [s]


class State(Enum):
    X = 1  # translation in x
    Y = 2  # translation in y
    Z = 3  # translation in z

    PHI = 4  # rotation about x
    THETA = 5  # rotation about y
    PSI = 6  # rotation about z


@dataclass
class StateSpace:
    A: np.array
    B: np.array
    C: np.array
    D: np.array

    state: State


_A = lambda c: np.array([[0, 1, 0],
                         [0, 0, c],
                         [0, 0, -1 / TAU]])

_B = np.array([[0, 0, 1 / TAU]]).transpose()

_C = np.array([[1, 0, 0],
               [0, 1, 0]])

_D = 0

_A_INTG = lambda c: np.array([[0, 1, 0, 0],
                              [0, 0, 1, 0],
                              [0, 0, 0, c],
                              [0, 0, 0, -1 / TAU]])

_B_INTG = np.array([[0, 0, 0, 1 / TAU]]).transpose()

_C_INTG = np.array([[1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0]])


def get_c(state: State) -> float:
    """
    Returns c (see doc for details) given a StateSpace.
    """
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


def get_state_space(state: State, add_intg_state: bool = False) -> StateSpace:
    """
    Returns a StateSpace (A, B, C, D) of the open-loop
    drone's dynamics given a State.

    Use add_intg_state to add an integrating state.
    """
    c = get_c(state)

    if add_intg_state:
        return StateSpace(_A_INTG(c), _B_INTG, _C_INTG, _D, state)
    else:
        return StateSpace(_A(c), _B, _C, _D, state)
