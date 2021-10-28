from dataclasses import dataclass

import numpy as np


# TODO: What should this contain, what are the interfaces? Could be:
# * Full state => 3x6 (6dof)
# * Either step and have controller etc outside, or include? The first approahc will be much cleaner. But, it will also
# move logic into something else.
# TODO: Should the motor force and torques Fm and Mm be the boundary?

@dataclass
class CtrlInput:
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


@dataclass
class DroneParams:
    m: float = 1.071  # mass [kg]
    I_body: np.ndarray = np.diag([0.014, 0.014, 0.026])  # moi of drone body [kg/m^3]
    I_motor_zz: float = 1e-4  # moi of drone motor & propeller [kg/m^3]

    Axz: float = 0.004  # area xz-plane [m^2]
    Ayz: float = 0.004  # area yz-plane [m^2]
    Axy: float = 0.05  # area xy-plane [m^2]

    cd: float = 1.0  # drag coefficient [-]

    lx: float = 0.24  # length x [m]
    ly: float = 0.24  # length y [m]
    lz: float = 0.24  # length z [m]


DEFAULT_DRONE_PARAMS = DroneParams()


@dataclass
class EnvParams:
    g: float = 9.82  # gravity of earth [m/s^2]
    rho: float = 1.225  # air density [kg/m^3]


DEFAULT_ENV_PARAMS = EnvParams()


class NonLinearModel:
    """
    Implements the 6dof non-linear model derived in www.diva-portal.org/smash/get/diva2:857660/FULLTEXT01.pdf,
    see section 3 for details.
    """

    # TODO: Where are all parameters given, during init?
    # TODO: Need to handle which frame we are in.
    # TODO: Also https://www.es.ele.tue.nl/education/5HC99/wiki/images/4/42/RigidBodyDynamics.pdf slide 12
    # TODO: Vibration's should be modelled here. Sensor noise should be modelled outside.

    def __init__(self,
                 states_init: States = STATES_ZERO,
                 drone_params: DroneParams = DEFAULT_DRONE_PARAMS,
                 env_params: EnvParams = DEFAULT_ENV_PARAMS
                 ):
        self._states = states_init

        self._drone_params = drone_params
        self._env_params = env_params

    def step(self, ctrl_input: CtrlInput):
        # Important to understand: The body dynamics v and a (r=0) are updated in the body frame, the inertial
        # frame dynamics are then given by the rotation matrices, hence r and eta (position) is integral
        # state in I-frame. Hence, we should be able to drive the I-frame dynamics from the body frame dynamics.
        pass

    def reset(self, states_reset: States = STATES_ZERO):
        self._states = states_reset

    def get_states(self) -> States:
        return self._states
