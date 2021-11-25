from dataclasses import dataclass

import numpy as np
from scipy.integrate import ode

from multi_body_sim.euclidean_transform import rotation_matrix


@dataclass
class BodyInput:
    fx: float
    fy: float
    fz: float

    mx: float
    my: float
    mz: float


@dataclass
class State:
    r_i: np.ndarray = np.zeros(3)  # translational position inertial-frame
    v_i: np.ndarray = np.zeros(3)  # translational velocity inertial-frame
    a_i: np.ndarray = np.zeros(3)  # translational acceleration inertial-frame

    v_b: np.ndarray = np.zeros(3)  # translational velocity body-frame
    a_b: np.ndarray = np.zeros(3)  # translational acceleration body-frame

    n_i: np.ndarray = np.zeros(3)  # rotational position (euler angles) inertial-frame
    w_b: np.ndarray = np.zeros(3)  # rotational velocity body-frame
    wp_b: np.ndarray = np.zeros(3)  # rotational acceleration body-frame

    q: np.ndarray = np.zeros(4)  # rotation as an quaternion (yaw -> pitch -> roll)


STATE_ZERO = State()


class SixDofModel:
    """
    Implements the 6dof non-linear model of an rigid object derived in
    https://www.diva-portal.org/smash/get/diva2:857660/FULLTEXT01.pdf, see
    section 3.3 for details.
    """

    def __init__(self, mass: float, moment_of_inertia: np.ndarray,
                 dt: float, state: State = STATE_ZERO):
        self._m = mass
        self._I_b = moment_of_inertia
        self._I_b_inv = np.linalg.inv(self._I_b)

        self._state = state
        self._dt = dt
        self._t = 0.0

        self._init_odes()

    def step(self, body_input: BodyInput):
        self._t += self._dt

        v_b, a_b, w_b, wp_b = self._step_6dof(body_input)
        q = self._step_quat(w_b)

        n_i = self._quat_to_euler(q)
        R_b_to_i = rotation_matrix(*n_i)

        v_i = R_b_to_i @ v_b
        a_i = R_b_to_i @ a_b
        r_i = self._step_ri(v_i)

        self._state = State(
            r_i=r_i,
            v_i=v_i,
            a_i=a_i,

            v_b=v_b,
            a_b=a_b,

            n_i=n_i,
            w_b=w_b,
            wp_b=wp_b,

            q=q,
        )

    def reset(self, state: State = STATE_ZERO):
        self._state = state
        self._init_odes()

    def get_state(self) -> State:
        return self._state

    def _init_odes(self):
        self._ode_6dof = ode(self._f_6dof).set_integrator('dopri5')
        self._ode_6dof.set_initial_value(y=[*self._state.v_b, *self._state.w_b])

        self._ode_quat = ode(self._f_quat).set_integrator('dopri5')
        self._ode_quat.set_initial_value(y=self._euler_to_quat(self._state.n_i))

        self._ode_ri = ode(self._f_ri).set_integrator('dopri5')
        self._ode_ri.set_initial_value(y=self._state.r_i)

    def _step_6dof(self, body_input: BodyInput) -> (np.ndarray, np.ndarray, np.ndarray, np.ndarray):
        F_b = np.array([body_input.fx, body_input.fy, body_input.fz])
        M_b = np.array([body_input.mx, body_input.my, body_input.mz])

        self._ode_6dof.set_f_params(F_b, M_b)
        y = self._ode_6dof.integrate(self._t)
        yp = self._f_6dof(self._t, y, F_b, M_b)

        v_b = y[0:3]
        a_b = yp[0:3]
        w_b = y[3:6]
        wp_b = yp[3:6]

        return v_b, a_b, w_b, wp_b

    def _step_quat(self, w_b: np.ndarray) -> np.ndarray:
        self._ode_quat.set_f_params(w_b)
        q = self._ode_quat.integrate(self._t)

        return q

    def _step_ri(self, v_i: np.ndarray) -> np.ndarray:
        self._ode_ri.set_f_params(v_i)
        y = self._ode_ri.integrate(self._t)

        return y

    def _f_6dof(self, t: float, y: np.ndarray, F_b: np.ndarray, M_b: np.ndarray) -> np.ndarray:
        yp = np.zeros(6)

        v_b = y[0:3]
        w_b = y[3:6]

        yp[0:3] = 1 / self._m * F_b - np.cross(w_b, v_b)
        yp[3:6] = self._I_b_inv @ (M_b - np.cross(w_b, self._I_b @ w_b))

        return yp

    def _f_quat(self, t: float, q: np.ndarray, w_b: np.ndarray) -> np.ndarray:
        qp = 1 / 2 * self._quat_multi(q, np.array([0, *w_b]))

        return qp

    def _f_ri(self, t: float, y: np.ndarray, v_i: np.ndarray) -> np.ndarray:
        yp = v_i

        return yp

    def _euler_to_quat(self, n: np.ndarray) -> np.ndarray:
        q_phi = np.array([np.cos(n[0] / 2), np.sin(n[0] / 2), 0, 0])
        q_theta = np.array([np.cos(n[1] / 2), 0, np.sin(n[1] / 2), 0])
        q_psi = np.array([np.cos(n[2] / 2), 0, 0, np.sin(n[2] / 2)])

        return self._quat_multi(q_psi, self._quat_multi(q_theta, q_phi))

    @staticmethod
    def _quat_to_euler(q: np.ndarray) -> np.ndarray:
        phi = np.arctan2(2 * (q[0] * q[1] + q[2] * q[3]), q[0] ** 2 - q[1] ** 2 - q[2] ** 2 + q[3] ** 2)
        theta = np.arcsin(2 * (q[0] * q[2] - q[1] * q[3]))
        psi = np.arctan2(2 * (q[0] * q[3] + q[1] * q[2]), q[0] ** 2 + q[1] ** 2 - q[2] ** 2 - q[3] ** 2)

        return np.array([phi, theta, psi])

    @staticmethod
    def _quat_multi(q_0: np.ndarray, q_1: np.ndarray) -> np.ndarray:
        s_0 = q_0[0]
        s_1 = q_1[0]

        v_0 = q_0[1:]
        v_1 = q_1[1:]

        return np.array([s_0 * s_1 - np.dot(v_0, v_1), *(s_0 * v_1 + s_1 * v_0 + np.cross(v_0, v_1))])
