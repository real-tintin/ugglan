# TODO: This is basically a copy of the target implementation (pilot_controller.cpp). In would be beneficial
#  if the target implementation could be used instead (DRY), see https://github.com/real-tintin/ugglan/issues/13.

import numpy as np
from dataclasses import dataclass
from non_linear_sim.att_estimator import AttEstimate
from non_linear_sim.drone_model import CtrlInput


@dataclass
class RefInput:
    f_z: float
    roll: float
    pitch: float
    yaw_rate: float


@dataclass
class State:
    x_phi: np.ndarray = np.zeros(4)
    x_theta: np.ndarray = np.zeros(4)
    x_psi: np.ndarray = np.zeros(3)


@dataclass
class Params:
    L_phi_0: float = 0.40
    L_phi_1: float = 3.85
    L_phi_2: float = 0.3
    L_phi_3: float = 0.02

    L_theta_0: float = 0.40
    L_theta_1: float = 3.85
    L_theta_2: float = 0.3
    L_theta_3: float = 0.02

    L_psi_0: float = 0.02
    L_psi_1: float = 0.04
    L_psi_2: float = 0.00

    anti_windup_sat_phi: float = 0.3
    anti_windup_sat_theta: float = 0.3
    anti_windup_sat_psi: float = 2.4


DEFAULT_PILOT_CTRL_PARAMS = Params()


class PilotCtrl:

    def __init__(self, params: Params, dt: float):
        self._L_phi = np.array([params.L_phi_0, params.L_phi_1, params.L_phi_2, params.L_phi_3])
        self._L_theta = np.array([params.L_theta_0, params.L_theta_1, params.L_theta_2, params.L_theta_3])
        self._L_psi = np.array([params.L_psi_0, params.L_psi_1, params.L_psi_2])

        self._params = params
        self._dt = dt

        self.reset()

    def update(self, ref_input: RefInput, att_estimate: AttEstimate):
        self._extract_att_estimates(att_estimate)
        self._change_of_variable(ref_input)
        self._integrate_with_antiwindup()

        self._update_ctrl(ref_input.f_z)

    def reset(self):
        self._state = State()
        self._ctrl_input = CtrlInput()

    def get_ctrl_input(self) -> CtrlInput:
        return self._ctrl_input

    def get_state(self) -> State:
        return self._state

    def _extract_att_estimates(self, est: AttEstimate):
        self._state.x_phi[1] = est.roll.angle
        self._state.x_phi[2] = est.roll.rate
        self._state.x_phi[3] = est.roll.acc

        self._state.x_theta[1] = est.pitch.angle
        self._state.x_theta[2] = est.pitch.rate
        self._state.x_theta[3] = est.pitch.acc

        self._state.x_psi[1] = est.yaw.rate
        self._state.x_psi[2] = est.yaw.acc

    def _change_of_variable(self, ref_input: RefInput):
        self._state.x_phi[1] -= ref_input.roll
        self._state.x_theta[1] -= ref_input.pitch
        self._state.x_psi[1] -= ref_input.yaw_rate

    def _integrate_with_antiwindup(self):
        self._state.x_phi[0] += self._state.x_phi[1] * self._dt
        self._state.x_theta[0] += self._state.x_theta[1] * self._dt
        self._state.x_psi[0] += self._state.x_psi[1] * self._dt

        self._state.x_phi[0] = self._range_sat(self._state.x_phi[0], self._params.anti_windup_sat_phi)
        self._state.x_theta[0] = self._range_sat(self._state.x_theta[0], self._params.anti_windup_sat_theta)
        self._state.x_psi[0] = self._range_sat(self._state.x_psi[0], self._params.anti_windup_sat_psi)

    def _update_ctrl(self, ref_f_z):
        self._ctrl_input.f_z = ref_f_z

        self._ctrl_input.m_x = -np.dot(self._L_phi, self._state.x_phi)
        self._ctrl_input.m_y = -np.dot(self._L_theta, self._state.x_theta)
        self._ctrl_input.m_z = -np.dot(self._L_psi, self._state.x_psi)

    @staticmethod
    def _range_sat(x, v):
        return max(-v, min(v, x))
