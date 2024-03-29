from dataclasses import dataclass

import numpy as np

from ugglan_tools.multi_body_sim.euclidean_transform import rotation_matrix
from ugglan_tools.non_linear_sim.att_estimator import AttEstimator, ImuOut, AttEstimate
from ugglan_tools.non_linear_sim.att_estimator import Params as AttEstParams
from ugglan_tools.non_linear_sim.drone_model import DroneModel, DroneParams, EnvParams, CtrlInput, MotorAngRates
from ugglan_tools.non_linear_sim.pilot_ctrl import Params as PilotCtrlParams
from ugglan_tools.non_linear_sim.pilot_ctrl import RefInput, PilotCtrl
from ugglan_tools.non_linear_sim.pilot_ctrl import State as PilotCtrlState
from ugglan_tools.non_linear_sim.six_dof_model import State as SixDofState
from ugglan_tools.non_linear_sim.six_dof_model import get_zero_initialized_state


@dataclass
class ImuNoise:
    acc_std: float = 0.8
    gyro_std: float = 0.02
    mag_std: float = 0.002


class Simulator:

    def __init__(self,
                 att_est_params: AttEstParams,
                 pilot_ctrl_params: PilotCtrlParams,
                 drone_params: DroneParams,
                 env_params: EnvParams,
                 imu_noise: ImuNoise,
                 dt: float,
                 init_state: SixDofState = get_zero_initialized_state(),
                 init_motors_with_fz_mg: bool = True
                 ):
        self._att_estimator = AttEstimator(params=att_est_params, dt=dt)
        self._pilot_ctrl = PilotCtrl(params=pilot_ctrl_params, dt=dt)
        self._drone_model = DroneModel(drone_params=drone_params, env_params=env_params,
                                       dt=dt, init_motors_with_fz_mg=init_motors_with_fz_mg)

        self._imu_noise = imu_noise
        self._g = env_params.g

        self.reset(state=init_state)

    def step(self, ref_input: RefInput):
        self._imu_out = self._extract_imu_out(self._drone_model.get_6dof_state())

        self._att_estimator.update(imu_out=self._imu_out)
        self._pilot_ctrl.update(ref_input=ref_input, att_estimate=self._att_estimator.get_estimate())

        self._drone_model.step(ctrl_input=self._pilot_ctrl.get_ctrl_input())

    def get_6dof_state(self):
        return self._drone_model.get_6dof_state()

    def get_att_estimate(self) -> AttEstimate:
        return self._att_estimator.get_estimate()

    def get_ctrl_input(self) -> CtrlInput:
        return self._pilot_ctrl.get_ctrl_input()

    def get_pilot_ctrl_state(self) -> PilotCtrlState:
        return self._pilot_ctrl.get_state()

    def get_imu_out(self) -> ImuOut:
        return self._imu_out

    def get_motor_ang_rates(self) -> MotorAngRates:
        return self._drone_model.get_motor_ang_rates()

    def get_t(self) -> float:
        return self._drone_model.get_t()

    def reset(self, state: SixDofState):
        self._imu_out = ImuOut()

        self._att_estimator.reset()
        self._pilot_ctrl.reset()
        self._drone_model.reset(state=state)

    def _extract_imu_out(self, state: SixDofState) -> ImuOut:
        imu_acc = self._body_acc_to_imu_acc(state.a_b, state.n_i, self._g)
        imu_mag = self._euler_to_imu_mag_for_yaw_est(*state.n_i)

        return ImuOut(
            acc_x=imu_acc[0] + np.random.normal(loc=0, scale=self._imu_noise.acc_std),
            acc_y=imu_acc[1] + np.random.normal(loc=0, scale=self._imu_noise.acc_std),
            acc_z=imu_acc[2] + np.random.normal(loc=0, scale=self._imu_noise.acc_std),

            ang_rate_x=state.w_b[0] + np.random.normal(loc=0, scale=self._imu_noise.gyro_std),
            ang_rate_y=state.w_b[1] + np.random.normal(loc=0, scale=self._imu_noise.gyro_std),
            ang_rate_z=state.w_b[2] + np.random.normal(loc=0, scale=self._imu_noise.gyro_std),

            mag_field_x=imu_mag[0] + np.random.normal(loc=0, scale=self._imu_noise.mag_std),
            mag_field_y=imu_mag[1] + np.random.normal(loc=0, scale=self._imu_noise.mag_std),
            mag_field_z=imu_mag[2] + np.random.normal(loc=0, scale=self._imu_noise.mag_std),
        )

    @staticmethod
    def _body_acc_to_imu_acc(a_b, n_i, g):
        """
        Convert the body acceleration to imu (or proper)
        accelerations.
        """
        e_z = np.array([0, 0, 1])
        R_i_to_b = rotation_matrix(*n_i).transpose()
        a_proper = a_b - (R_i_to_b @ e_z * g)

        return a_proper

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
