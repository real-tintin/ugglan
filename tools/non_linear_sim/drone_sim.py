from dataclasses import dataclass

import numpy as np


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

# TODO: This should make use of the 6dof model and implement the drone dynamics.
# TODO: Vibration's and sensor should be modelled here.
