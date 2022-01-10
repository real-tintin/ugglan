# TODO: This is basically a copy of the target implementation (attitude_estimation.cpp). In would be beneficial
#  if the target implementation could be used instead (DRY), see https://github.com/real-tintin/ugglan/issues/13.
#  Also, note that another python implementation exists under ./state_est/attitude_estimators (AttEstKalman).

from dataclasses import dataclass, field

import numpy as np

from non_linear_sim.real_time_filter import FilterConfig, RealTimeFilter, FilterName, FilterType

MODULO_ROLL = np.pi
MODULO_PITCH = np.pi / 2
MODULO_YAW = np.pi

R_ALMOST_ZERO = 1e-6


@dataclass
class ImuOut:
    acc_x: float = 0.0  # [m/s^2]
    acc_y: float = 0.0  # [m/s^2]
    acc_z: float = 0.0  # [m/s^2]

    ang_rate_x: float = 0.0  # [rad/s]
    ang_rate_y: float = 0.0  # [rad/s]
    ang_rate_z: float = 0.0  # [rad/s]

    mag_field_x: float = 0.0  # [gauss]
    mag_field_y: float = 0.0  # [gauss]
    mag_field_z: float = 0.0  # [gauss]


@dataclass
class AttState:
    angle: float = 0.0  # [rad]
    rate: float = 0.0  # [rad/s]
    acc: float = 0.0  # [rad/s^2]


@dataclass
class AttEstimate:
    roll: AttState = field(default_factory=AttState)
    pitch: AttState = field(default_factory=AttState)
    yaw: AttState = field(default_factory=AttState)


@dataclass
class KalmanState:
    x: np.ndarray = field(default_factory=lambda: np.zeros(3))
    z: np.ndarray = field(default_factory=lambda: np.zeros(2))
    P: np.ndarray = field(default_factory=lambda: np.zeros((3, 3)))
    R: np.ndarray = field(default_factory=lambda: np.zeros((2, 2)))


@dataclass
class ImuAngleEst:
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


@dataclass
class AttEstDebug:
    kalman_r_0_roll: float
    kalman_r_0_pitch: float
    kalman_r_0_yaw: float

    raw_acc_residual: float
    filtered_acc_residual: float

    filtered_acc_x: float
    filtered_acc_y: float
    filtered_acc_z: float


@dataclass
class Params:
    scale_Q: float = 1e2

    r_0_min: float = 10
    r_0_max: float = 1000

    r_0_acc_res_scale: float = 1000
    r_0_gyro_scale: float = 100

    r_1: float = 1


DEFAULT_ATT_EST_PARAMS = Params()


class AttEstimator:
    def __init__(self, params: Params, g: float, dt: float):
        self._params = params
        self._dt = dt
        self._g = g

        self._Q = params.scale_Q * np.array([
            [0.25 * dt ** 4, 0.5 * dt ** 3, 0.5 * dt ** 2],
            [0.5 * dt ** 3, dt ** 2, dt],
            [0.5 * dt ** 2, dt, 1],
        ])

        self._F = np.array([
            [1, dt, 0.5 * dt ** 2],
            [0, 1, dt],
            [0, 0, 1],
        ])
        self._F_t = self._F.transpose()

        self._H = np.array([
            [1, 0, 0],
            [0, 1, 0]
        ])
        self._H_t = self._H.transpose()

        self.reset()

    def update(self, imu_out: ImuOut):
        self._imu_out = imu_out

        self._lp_filter_acc()
        self._update_imu_angle_est()
        self._update_acc_residual()

        self._update_roll()
        self._update_pitch()
        self._update_yaw()

    def reset(self):
        self._att_est = AttEstimate()
        self._imu_ang_est = ImuAngleEst()

        self._kalman_roll = KalmanState()
        self._kalman_pitch = KalmanState()
        self._kalman_yaw = KalmanState()

        self._raw_acc_residual = 0.0
        self._filtered_acc_residual = 0.0

        self._filter_acc_res = RealTimeFilter(config=FilterConfig(name=FilterName.BUTTER,
                                                                  type=FilterType.LOW_PASS,
                                                                  n_order=3,
                                                                  low_cut_off=1.0,
                                                                  ), fs=int(1 / self._dt))

        acc_filter_config = FilterConfig(name=FilterName.BUTTER, type=FilterType.LOW_PASS, n_order=3, low_cut_off=1.0)

        self._lp_filter_acc_x = RealTimeFilter(config=acc_filter_config, fs=int(1 / self._dt))
        self._lp_filter_acc_y = RealTimeFilter(config=acc_filter_config, fs=int(1 / self._dt))
        self._lp_filter_acc_z = RealTimeFilter(config=acc_filter_config, fs=int(1 / self._dt))

    def get_estimate(self) -> AttEstimate:
        return self._att_est

    def get_debug(self) -> AttEstDebug:
        return AttEstDebug(
            raw_acc_residual=self._raw_acc_residual,
            filtered_acc_residual=self._filter_acc_res.get(),

            kalman_r_0_roll=self._kalman_roll.R[0, 0],
            kalman_r_0_pitch=self._kalman_pitch.R[0, 0],
            kalman_r_0_yaw=self._kalman_yaw.R[0, 0],

            filtered_acc_x=self._lp_filter_acc_x.get(),
            filtered_acc_y=self._lp_filter_acc_y.get(),
            filtered_acc_z=self._lp_filter_acc_z.get(),
        )

    def _lp_filter_acc(self):
        self._lp_filter_acc_x.update(self._imu_out.acc_x)
        self._lp_filter_acc_y.update(self._imu_out.acc_y)
        self._lp_filter_acc_z.update(self._imu_out.acc_z)

    def _update_acc_residual(self):
        acc_xyz_norm = np.linalg.norm([self._lp_filter_acc_x.get(),
                                       self._lp_filter_acc_y.get(),
                                       self._lp_filter_acc_z.get()])

        self._raw_acc_residual = np.abs(self._g - acc_xyz_norm)
        self._filter_acc_res.update(self._raw_acc_residual)

    def _update_imu_angle_est(self):
        self._imu_ang_est.roll = np.arctan2(-self._lp_filter_acc_y.get(),
                                            -self._lp_filter_acc_z.get())

        self._imu_ang_est.pitch = np.arctan2(self._lp_filter_acc_x.get(),
                                             np.sqrt(self._lp_filter_acc_y.get() ** 2 +
                                                     self._lp_filter_acc_z.get() ** 2))

        b_x = self._imu_out.mag_field_x * np.cos(self._att_est.pitch.angle) + \
              self._imu_out.mag_field_y * np.sin(self._att_est.roll.angle) * np.sin(self._att_est.pitch.angle) + \
              self._imu_out.mag_field_z * np.sin(self._att_est.pitch.angle) * np.cos(self._att_est.roll.angle)

        b_y = self._imu_out.mag_field_y * np.cos(self._att_est.roll.angle) - \
              self._imu_out.mag_field_z * np.sin(self._att_est.roll.angle)

        self._imu_ang_est.yaw = np.arctan2(-b_y, b_x)

    def _update_roll(self):
        self._update_est(
            self._imu_ang_est.roll,
            self._imu_out.ang_rate_x,
            self._kalman_roll,
            self._att_est.roll,
            MODULO_ROLL,
        )

    def _update_pitch(self):
        self._update_est(
            self._imu_ang_est.pitch,
            self._imu_out.ang_rate_y,
            self._kalman_pitch,
            self._att_est.pitch,
            MODULO_PITCH,
        )

    def _update_yaw(self):
        self._update_est(
            self._imu_ang_est.yaw,
            self._imu_out.ang_rate_z,
            self._kalman_yaw,
            self._att_est.yaw,
            MODULO_YAW,
        )

    def _update_est(self,
                    z_0: float, z_1: float,
                    kalman_state: KalmanState,
                    att_state: AttState,
                    modulo_lim: float):
        kalman_state.z[0] = z_0
        kalman_state.z[1] = z_1

        r_0 = self._params.r_0_gyro_scale * np.abs(z_1) + self._params.r_0_acc_res_scale * self._raw_acc_residual
        r_0 = min(max(r_0, self._params.r_0_min), self._params.r_0_max)

        kalman_state.R[0, 0] = r_0
        kalman_state.R[1, 1] = self._params.r_1

        self._update_kalman_state(kalman_state)
        self._kalman_state_to_att_(kalman_state, att_state)

        att_state.angle = self._modulo_angle(att_state.angle, modulo_lim)

    def _update_kalman_state(self, state: KalmanState):
        x_pri = self._F @ state.x
        P_pri = self._F @ state.P @ self._F_t + self._Q

        S = self._H @ P_pri @ self._H_t + state.R
        K = P_pri @ self._H_t @ np.linalg.inv(S)

        state.x = x_pri + K @ (state.z - self._H @ x_pri)
        state.P = (np.eye(3) - K @ self._H) @ P_pri

    @staticmethod
    def _kalman_state_to_att_(kalman: KalmanState, att: AttState):
        att.angle = kalman.x[0]
        att.rate = kalman.x[1]
        att.acc = kalman.x[2]

    @staticmethod
    def _modulo_angle(angle: float, limit: float) -> float:
        return np.mod(angle + limit, 2 * limit) - limit
