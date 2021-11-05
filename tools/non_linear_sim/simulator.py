import numpy as np

from multi_body.euclidean_transform import rotation_matrix
from non_linear_sim.att_est import AttEst, ImuOut
from non_linear_sim.att_est import Estimate as AttEstimate
from non_linear_sim.drone_model import DroneModel, DroneParams, EnvParams, CtrlInput
from non_linear_sim.pilot_ctrl import PilotCtrl, RefInput
from non_linear_sim.pilot_ctrl import State as PilotCtrlState
from non_linear_sim.six_dof_model import State as SixDofState


# TODO: Unit test this, some steps etc.
# TODO: Add IMU sensor noise.


class Simulator:

    def __init__(self,
                 att_est: AttEst,
                 pilot_ctrl: PilotCtrl,
                 six_dof_state: SixDofState,
                 drone_params: DroneParams,
                 env_params: EnvParams,
                 dt: float,
                 ):
        self._att_est = att_est
        self._pilot_ctrl = pilot_ctrl
        self._drone_model = DroneModel(state=six_dof_state,
                                       drone_params=drone_params,
                                       env_params=env_params,
                                       dt=dt)

    def step(self, ref_input: RefInput):
        self._att_est.update(imu_out=self._extract_imu_out(self._drone_model.get_6dof_state()))
        self._pilot_ctrl.update(ref_input=ref_input, att_est_state=self._att_est.get_state())
        self._drone_model.step(ctrl_input=self._pilot_ctrl.get_ctrl_input())

    def get_6dof_state(self):
        return self._drone_model.get_6dof_state()

    def get_att_estimate(self) -> AttEstimate:
        return self._att_est.get_estimate()

    def get_ctrl_input(self) -> CtrlInput:
        return self._pilot_ctrl.get_ctrl_input()

    def get_pilot_ctrl_state(self) -> PilotCtrlState:
        return self._pilot_ctrl.get_state()

    def get_t(self) -> float:
        return self._drone_model.get_t()

    def reset(self, six_dof_state: SixDofState):
        self._att_est.reset()
        self._pilot_ctrl.reset()
        self._drone_model.reset(six_dof_state)

    def _extract_imu_out(self, state: SixDofState) -> ImuOut:
        a_b = state.a_b
        w_b = state.w_b
        M = self._euler_to_imu_mag_for_yaw_est(state.n_i)

        return ImuOut(
            acc_x=a_b[0],
            acc_y=a_b[1],
            acc_z=a_b[2],

            ang_rate_x=w_b[0],
            ang_rate_y=w_b[1],
            ang_rate_z=w_b[2],

            mag_field_x=M[0],
            mag_field_y=M[1],
            mag_field_z=M[2],
        )

    @staticmethod
    def _euler_to_imu_mag_for_yaw_est(phi, theta, psi):
        """
        Use the euler angles to fabricate the magnetic field
        from a body mounted imu. Note, we let B_z = 0 i.e.,
        assume 90 deg inclination angle.
        """
        R_phi = rotation_matrix(x_rad=phi)
        R_theta = rotation_matrix(y_rad=theta)

        B = np.array([np.cos(psi), -np.sin(psi), 0])
        M = (R_theta @ R_phi).transpose() @ B

        return M
