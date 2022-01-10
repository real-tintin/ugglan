# TODO: This is basically a copy of the target implementation (attitude_estimation.cpp). In would be beneficial
#  if the target implementation could be used instead (DRY), see https://github.com/real-tintin/ugglan/issues/13.
#  Also, note that another python implementation exists under ./state_est/attitude_estimators (AttEstKalman).

from collections import deque
from dataclasses import dataclass, field

import numpy as np
from scipy import signal

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
    x: np.ndarray = np.zeros(3)
    z: np.ndarray = np.zeros(2)
    P: np.ndarray = np.zeros((3, 3))
    R: np.ndarray = np.zeros((2, 2))


@dataclass
class ImuAngleEst:
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


@dataclass
class Params:
    scale_Q: float = 1e2

    r_0_init: float = 1

    r_0_min = 10
    r_0_max = 1000

    r_0_gyro_norm_T: float = 0.1
    r_0_gyro_norm_i: float = 2

    r_0_acc_norm_T: float = 0.1
    r_0_acc_norm_pi: float = 1
    r_0_acc_norm_ni: float = 1

    scale_R_1: float = 1

    rolling_var_window_size: int = 20


DEFAULT_ATT_EST_PARAMS = Params()


class AttEstimator:
    def __init__(self, params: Params, dt: float):
        self._params = params
        self._dt = dt

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

        self._update_imu_angle_est()
        self._update_rolling_var()
        self._update_r_0()

        if self._is_rolling_var_ready:
            self._update_roll()
            self._update_pitch()
            self._update_yaw()

    def reset(self):
        self._att_est = AttEstimate()
        self._imu_ang_est = ImuAngleEst()

        self._kalman_roll = KalmanState()
        self._kalman_pitch = KalmanState()
        self._kalman_yaw = KalmanState()

        self._is_rolling_var_ready = False
        self._rolling_var_samples_in_buf = 0

        self._v_filt = 0
        self._v_filt_old = 0
        self._v = 0
        self._v_sos = signal.butter(N=3, Wn=1, btype='low', fs=int(1 / self._dt), output='sos')
        self._v_z = np.zeros((2, 2))

        self._r_0 = self._params.r_0_init

        self._rolling_var_roll_angle = deque(maxlen=self._params.rolling_var_window_size)
        self._rolling_var_pitch_angle = deque(maxlen=self._params.rolling_var_window_size)
        self._rolling_var_yaw_angle = deque(maxlen=self._params.rolling_var_window_size)

        self._rolling_var_roll_rate = deque(maxlen=self._params.rolling_var_window_size)
        self._rolling_var_pitch_rate = deque(maxlen=self._params.rolling_var_window_size)
        self._rolling_var_yaw_rate = deque(maxlen=self._params.rolling_var_window_size)

    def get_estimate(self) -> AttEstimate:
        return self._att_est

    def is_calibrated(self) -> bool:
        return self._is_rolling_var_ready

    def _update_r_0(self):
        # TODO: Handle dip between. cumsum?
        acc_xyz_norm = np.linalg.norm([self._imu_out.acc_x, self._imu_out.acc_y, self._imu_out.acc_z])
        g = 9.82
        self._v = np.abs(g - acc_xyz_norm)

        v_filt, self._v_z = signal.sosfilt(self._v_sos, [self._v], zi=self._v_z)

        self._v_filt_old = self._v_filt
        self._v_filt = v_filt[0]

        gyro_xy_norm = np.linalg.norm([self._imu_out.ang_rate_x, self._imu_out.ang_rate_y])

        if gyro_xy_norm > self._params.r_0_gyro_norm_T:
            self._r_0 += self._params.r_0_gyro_norm_i

        elif (self._v_filt - self._v_filt_old) > 0:
            self._r_0 += self._params.r_0_acc_norm_pi

        else:
            self._r_0 -= self._params.r_0_acc_norm_ni

        self._r_0 = 100 * np.abs(self._imu_out.ang_rate_x) + 500 * self._v_filt

        self._r_0 = min(max(self._r_0, self._params.r_0_min), self._params.r_0_max)

    def get_r_0(self):
        return self._r_0

    def get_v(self):
        return self._v

    def get_v_filt(self):
        return self._v_filt

    def _update_imu_angle_est(self):
        self._imu_ang_est.roll = np.arctan2(-self._imu_out.acc_y,
                                            -self._imu_out.acc_z)

        self._imu_ang_est.pitch = np.arctan2(self._imu_out.acc_x,
                                             np.sqrt(self._imu_out.acc_y ** 2 + self._imu_out.acc_z ** 2))

        b_x = self._imu_out.mag_field_x * np.cos(self._att_est.pitch.angle) + \
              self._imu_out.mag_field_y * np.sin(self._att_est.roll.angle) * np.sin(self._att_est.pitch.angle) + \
              self._imu_out.mag_field_z * np.sin(self._att_est.pitch.angle) * np.cos(self._att_est.roll.angle)

        b_y = self._imu_out.mag_field_y * np.cos(self._att_est.roll.angle) - \
              self._imu_out.mag_field_z * np.sin(self._att_est.roll.angle)

        self._imu_ang_est.yaw = np.arctan2(-b_y, b_x)

    def _update_rolling_var(self):
        self._rolling_var_roll_angle.append(self._imu_ang_est.roll)
        self._rolling_var_pitch_angle.append(self._imu_ang_est.pitch)
        self._rolling_var_yaw_angle.append(self._imu_ang_est.yaw)

        self._rolling_var_roll_rate.append(self._imu_out.ang_rate_x)
        self._rolling_var_pitch_rate.append(self._imu_out.ang_rate_y)
        self._rolling_var_yaw_rate.append(self._imu_out.ang_rate_z)

        self._rolling_var_samples_in_buf += 1

        if self._rolling_var_samples_in_buf >= self._params.rolling_var_window_size:
            self._is_rolling_var_ready = True
        else:
            self._is_rolling_var_ready = False

    def _update_roll(self):
        self._update_est(
            self._imu_ang_est.roll,
            self._imu_out.ang_rate_x,
            np.var(self._rolling_var_roll_angle),
            np.var(self._rolling_var_roll_rate),
            self._kalman_roll,
            self._att_est.roll,
            MODULO_ROLL,
        )

    def _update_pitch(self):
        self._update_est(
            self._imu_ang_est.pitch,
            self._imu_out.ang_rate_y,
            np.var(self._rolling_var_pitch_angle),
            np.var(self._rolling_var_pitch_rate),
            self._kalman_pitch,
            self._att_est.pitch,
            MODULO_PITCH,
        )

    def _update_yaw(self):
        self._update_est(
            self._imu_ang_est.yaw,
            self._imu_out.ang_rate_z,
            np.var(self._rolling_var_yaw_angle),
            np.var(self._rolling_var_yaw_rate),
            self._kalman_yaw,
            self._att_est.yaw,
            MODULO_YAW,
        )

    def _update_est(self,
                    z_0: float, z_1: float,
                    r_0: float, r_1: float,
                    kalamn_state: KalmanState,
                    att_state: AttState,
                    modulo_lim: float):
        kalamn_state.z[0] = z_0
        kalamn_state.z[1] = z_1

        kalamn_state.R[0, 0] = 100 * np.abs(z_1) + 200 * self._v_filt
        kalamn_state.R[1, 1] = self._params.scale_R_1  # * max(r_1, R_ALMOST_ZERO)

        self._update_kalman_state(kalamn_state)
        self._kalman_state_to_att_(kalamn_state, att_state)

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
