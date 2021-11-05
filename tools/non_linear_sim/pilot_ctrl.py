# TODO: This is basically a copy of the target implementation (pilot_controller.cpp). In would be beneficial
#  if the target implementation could be used instead (DRY).

from dataclasses import dataclass

import numpy as np

from non_linear_sim.drone_model import CtrlInput
from state_est.att_estimators import State as AttEstState

ANTI_WINDUP_SAT_PHI = 0.3
ANTI_WINDUP_SAT_THETA = 0.3
ANTI_WINDUP_SAT_PSI = 2.4


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
    x_psi: np.ndarray = np.zeros(4)


class PilotCtrl:

    def __init__(self, L_phi: np.ndarray, L_theta: np.ndarray, L_psi: np.ndarray, dt: float):
        self._L_phi = L_phi
        self._L_theta = L_theta
        self._L_psi = L_psi

        self._dt = dt

        self._state = State()
        self._ctrl_input = CtrlInput()

    def update(self, ref_input: RefInput, att_est_state: AttEstState):
        self._extract_states(att_est_state)
        self._change_of_variable(ref_input)
        self._integrate_with_antiwindup()

        self._update_ctrl(ref_input)

    def reset(self):
        self._state = State()

    def get_ctrl_input(self) -> CtrlInput:
        return self._ctrl_input

    def get_state(self) -> State:
        return self._state

    def _extract_states(self, att_est_state: AttEstState):
        self._state.x_phi = np.array([att_est_state.phi, att_est_state.phi_p, att_est_state.phi_pp])
        self._state.x_theta = np.array([att_est_state.theta, att_est_state.theta_p, att_est_state.theta_pp])
        self._state.x_psi = np.array([att_est_state.psi, att_est_state.psi_p, att_est_state.psi_pp])

    def _change_of_variable(self, ref_input: RefInput):
        self._state.x_phi[1] -= ref_input.roll
        self._state.x_theta[1] -= ref_input.pitch
        self._state.x_psi[1] -= ref_input.yaw_rate

    def _integrate_with_antiwindup(self):
        self._state.x_phi[0] += self._state.x_phi[1] * self._dt
        self._state.x_theta[0] += self._state.x_theta[1] * self._dt
        self._state.x_psi[0] += self._state.x_psi[1] * self._dt

        self._range_sat(self._state.x_phi[0], ANTI_WINDUP_SAT_PHI)
        self._range_sat(self._state.x_theta[0], ANTI_WINDUP_SAT_THETA)
        self._range_sat(self._state.x_psi[0], ANTI_WINDUP_SAT_PSI)

    def _update_ctrl(self, ref_input: RefInput):
        self._ctrl_input.f_z = ref_input.f_z

        self._ctrl_input.m_x = -(self._L_phi @ self._state.x_phi)
        self._ctrl_input.m_y = -(self._L_theta @ self._state.x_theta)
        self._ctrl_input.m_z = -(self._L_psi @ self._state.x_psi)

    @staticmethod
    def _range_sat(x, v):
        x = np.min(v, np.max(v, x))
