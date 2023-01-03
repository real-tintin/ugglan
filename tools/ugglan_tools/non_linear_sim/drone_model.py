from dataclasses import dataclass

import numpy as np
from scipy.integrate import ode

from ugglan_tools.linear_sim.physical_const import MASS, TAU_M, I_XX, I_YY, I_ZZ
from ugglan_tools.multi_body_sim.euclidean_transform import rotation_matrix
from ugglan_tools.non_linear_sim.six_dof_model import SixDofModel, State, BodyInput, get_zero_initialized_state


@dataclass
class CtrlInput:
    f_z: float = 0.0
    m_x: float = 0.0
    m_y: float = 0.0
    m_z: float = 0.0


@dataclass
class MotorAngRates:
    w_0: float = 0.0
    w_1: float = 0.0
    w_2: float = 0.0
    w_3: float = 0.0


@dataclass
class DroneParams:
    m: float = MASS  # mass [kg]

    I_drone_xx: float = I_XX  # moi of drone body about x [kg/m^3]
    I_drone_yy: float = I_YY  # moi of drone body about y [kg/m^3]
    I_drone_zz: float = I_ZZ  # moi of drone body about z [kg/m^3]

    I_motor_zz: float = 1e-4  # moi of drone motor & propeller about z [kg/m^3]

    Axz: float = 0.1 * 0.5  # area xz-plane [m^2]
    Ayz: float = 0.1 * 0.5  # area yz-plane [m^2]
    Axy: float = 0.5 ** 2  # area xy-plane [m^2]

    C_d: float = 1.0  # drag coefficient [-]

    lx: float = 0.24  # length x [m]
    ly: float = 0.24  # length y [m]
    lz: float = 0.24  # length z [m]

    c_fz: float = 8.37e-6  # motor propeller force constant [Ns^2/rad^2]
    c_mz: float = 8.37e-6 / 50  # motor propeller torque constant [Nms^2/rad^2]

    tau_m: float = TAU_M  # motor time constant [s]

    sq_motor_cmd_min: float = 200 ** 2  # [rad^2/s^2]
    sq_motor_cmd_max: float = 750 ** 2  # [rad^2/s^2]


DEFAULT_DRONE_PARAMS = DroneParams()


@dataclass
class EnvParams:
    g: float = 9.82  # gravity of earth [m/s^2]
    rho_air: float = 1.225  # air density [kg/m^3]


DEFAULT_ENV_PARAMS = EnvParams()


class DroneModel:
    """
    Implements the 6dof drone model described in section 3.4 in
    http://www.diva-portal.org/smash/get/diva2:857660/FULLTEXT01.pdf.
    """

    def __init__(self, drone_params: DroneParams,
                 env_params: EnvParams,
                 dt: float,
                 init_state: State = get_zero_initialized_state(),
                 init_motors_with_fz_mg: bool = True):
        self._drone_params = drone_params
        self._env_params = env_params
        self._init_motors_with_fz_mg = init_motors_with_fz_mg
        self._init_H_and_H_inv()
        self._dt = dt

        self._6dof_model = SixDofModel(mass=drone_params.m,
                                       moment_of_inertia=np.diag([drone_params.I_drone_xx,
                                                                  drone_params.I_drone_yy,
                                                                  drone_params.I_drone_zz]),
                                       dt=dt)

        self.reset(state=init_state)

    def step(self, ctrl_input: CtrlInput):
        self._t += self._dt

        state_6dof = self._6dof_model.get_state()

        F_m, M_m, self._w_m = self._exec_motor_dyn(ctrl_input)

        F_d = self._compute_F_d(state_6dof.v_b)
        F_g = self._compute_F_g(state_6dof.n_i)

        M_d = self._compute_M_d(state_6dof.w_b)
        M_p = self._compute_M_p(state_6dof.w_b, self._w_m)

        F_b = F_m + F_d + F_g
        M_b = M_m + M_d + M_p

        self._6dof_model.step(BodyInput(
            fx=F_b[0], fy=F_b[1], fz=F_b[2],
            mx=M_b[0], my=M_b[1], mz=M_b[2])
        )

    def reset(self, state: State = State()):
        self._t = 0.0
        self._w_m = np.zeros(4)

        self._6dof_model.reset(state=state)
        self._init_motor_dyn()

    def get_6dof_state(self) -> State:
        return self._6dof_model.get_state()

    def get_motor_ang_rates(self) -> MotorAngRates:
        return MotorAngRates(*self._w_m)

    def get_t(self) -> float:
        return self._t

    def _init_motor_dyn(self):
        self._ode_motor_dyn = ode(self._f_motor_dyn).set_integrator('dopri5')

        if self._init_motors_with_fz_mg:
            self._ode_motor_dyn.set_initial_value(y=[-self._get_mg(), 0, 0, 0])
        else:
            self._ode_motor_dyn.set_initial_value(y=np.zeros(4))

    def _exec_motor_dyn(self, ctrl_input: CtrlInput) -> (np.ndarray, np.ndarray, np.ndarray):
        y_m = self._step_motor_dyn(ctrl_input)
        F_m, M_m, w_m = self._range_lim_motor_ctrl(y_m)

        return F_m, M_m, w_m

    def _range_lim_motor_ctrl(self, y_raw) -> (np.ndarray, np.ndarray, np.ndarray):
        w_m_lim_sq = self._H_inv @ y_raw

        w_m_lim_sq[w_m_lim_sq > self._drone_params.sq_motor_cmd_max] = self._drone_params.sq_motor_cmd_max
        w_m_lim_sq[w_m_lim_sq < self._drone_params.sq_motor_cmd_min] = 0.0

        y_m_lim = self._H @ w_m_lim_sq
        w_m_lim = np.sqrt(w_m_lim_sq)

        F_m_lim = np.array([0, 0, y_m_lim[0]])
        M_m_lim = y_m_lim[1:]

        return F_m_lim, M_m_lim, w_m_lim

    def _step_motor_dyn(self, ctrl_input: CtrlInput) -> np.ndarray:
        u = np.array([ctrl_input.f_z, ctrl_input.m_x, ctrl_input.m_y, ctrl_input.m_z])

        self._ode_motor_dyn.set_f_params(u)
        y = self._ode_motor_dyn.integrate(self._t)

        return y

    def _f_motor_dyn(self, t: float, y: np.ndarray, u: np.ndarray) -> np.ndarray:
        yp = 1 / self._drone_params.tau_m * (-y + u)

        return yp

    def _init_H_and_H_inv(self):
        lx = self._drone_params.lx
        c_fz = self._drone_params.c_fz
        c_mz = self._drone_params.c_mz

        self._H = np.array([
            [-c_fz, -c_fz, -c_fz, -c_fz],
            [-lx * c_fz, -lx * c_fz, lx * c_fz, lx * c_fz],
            [lx * c_fz, -lx * c_fz, -lx * c_fz, lx * c_fz],
            [-c_mz, c_mz, -c_mz, c_mz],
        ])

        self._H_inv = np.linalg.inv(self._H)

    def _compute_F_d(self, v_b: np.ndarray) -> np.ndarray:
        rho = self._env_params.rho_air
        C_d = self._drone_params.C_d

        Ayz = self._drone_params.Ayz
        Axz = self._drone_params.Axz
        Axy = self._drone_params.Axy

        return -0.5 * rho * C_d * np.array([
            Ayz * v_b[0] * np.abs(v_b[0]),
            Axz * v_b[1] * np.abs(v_b[1]),
            Axy * v_b[2] * np.abs(v_b[2])
        ])

    def _compute_F_g(self, n_i: np.ndarray) -> np.ndarray:
        R_i_to_b = rotation_matrix(*n_i).transpose()
        e_z = np.array([0, 0, 1])

        return self._get_mg() * R_i_to_b @ e_z

    def _compute_M_d(self, w_b: np.ndarray) -> np.ndarray:
        rho = self._env_params.rho_air
        C_d = self._drone_params.C_d

        Axy = self._drone_params.Axy
        Axz = self._drone_params.Axz

        lx = self._drone_params.lx
        ly = self._drone_params.ly
        lz = self._drone_params.lz

        return -0.5 * rho * C_d * np.array([
            Axy * w_b[0] * np.abs(w_b[0]) * lx,
            Axy * w_b[1] * np.abs(w_b[1]) * ly,
            8 * Axz * w_b[2] * np.abs(w_b[2]) * lz
        ])

    def _compute_M_p(self, w_b: np.ndarray, w_m: np.ndarray) -> np.ndarray:
        I_m = np.diag([0, 0, self._drone_params.I_motor_zz])
        e_z = np.array([0, 0, 1])
        M_p = np.zeros(3)

        for i_m in range(4):
            M_p += float(-1) ** (i_m + 1) * np.cross(w_b, I_m @ e_z * w_m[i_m])

        return M_p

    def _get_mg(self):
        return self._drone_params.m * self._env_params.g
