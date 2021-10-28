from dataclasses import dataclass

import numpy as np


@dataclass
class BodyCtrl:
    fx: float
    fy: float
    fz: float

    mx: float
    my: float
    mz: float


@dataclass
class State:
    pos: float = 0
    vel: float = 0
    acc: float = 0


STATE_ZERO = State()


@dataclass
class States:
    x: State = STATE_ZERO
    y: State = STATE_ZERO
    z: State = STATE_ZERO

    phi: State = STATE_ZERO
    theta: State = STATE_ZERO
    psi: State = STATE_ZERO


STATES_ZERO = States()


class SixDofModel:
    """
    Implements the 6dof non-linear model of an rigid object derived in
    https://www.diva-portal.org/smash/get/diva2:857660/FULLTEXT01.pdf, see
    section 3.3 for details.
    """

    # TODO: Need to handle which frame we are in.
    # TODO: Also https://www.es.ele.tue.nl/education/5HC99/wiki/images/4/42/RigidBodyDynamics.pdf slide 12.
    # TODO: Create matrices.

    def __init__(self, mass: float, moment_of_inertia: np.ndarray, states_init: States = STATES_ZERO):
        self._states = states_init

        self._m = mass
        self._I = moment_of_inertia

    def step(self, body_ctrl: BodyCtrl):
        # Important to understand: The body dynamics v and a (r=0) are updated in the body frame, the inertial
        # frame dynamics are then given by the rotation matrices, hence r and eta (position) is integral
        # state in I-frame. Hence, we should be able to drive the I-frame dynamics from the body frame dynamics.
        pass

    def reset(self, states_reset: States = STATES_ZERO):
        self._states = states_reset

    def get_states(self) -> States:
        return self._states
