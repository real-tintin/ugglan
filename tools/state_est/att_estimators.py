from abc import abstractmethod, ABC
from copy import deepcopy
from dataclasses import dataclass
from enum import Enum

import numpy as np
import pandas as pd

from data_log.io import Signals

IMU_SAMPLE_RATE_S = 0.01  # 100 Hz

STATIC_GYRO_OFFSET_COMP_SAMPLES = 100

DYNAMIC_GYRO_OFFSET_COMP_SAMPLES = 100
DYNAMIC_GYRO_OFFSET_COMP_ABS_ACC_LIM = 0.2  # [rad/s^2]

CF_CUT_OFF_FREQ = 0.2  # [Hz]
CF_TAU_PHI = 1 / (2 * np.pi * CF_CUT_OFF_FREQ)
CF_TAU_THETA = 1 / (2 * np.pi * CF_CUT_OFF_FREQ)
CF_TAU_PSI = 1 / (2 * np.pi * CF_CUT_OFF_FREQ)

GYRO_LP_CUT_OFF_FREQ = 15  # [Hz]

KALMAN_P_0 = np.zeros((3, 3))
KALMAN_Q_SCALE = 1e2
KALMAN_G = np.array([0.5 * IMU_SAMPLE_RATE_S ** 2, IMU_SAMPLE_RATE_S, 1])
KALMAN_Q = KALMAN_Q_SCALE * np.outer(KALMAN_G, KALMAN_G)
KALMAN_R_0_SCALE = 1
KALMAN_R_1_SCALE = 1
KALMAN_R_ALMOST_ZERO = 1e-9
KALMAN_R_WINDOW_SIZE = 20

HARD_IRON_OFFSET_X = 0.104
HARD_IRON_OFFSET_Y = 0.076
HARD_IRON_OFFSET_Z = 0.062


class Estimator(Enum):
    ACC_MAG = 'AccMag'
    GYRO = 'Gyro'
    GYRO_LP = 'GyroLp'
    CF = 'Cf'
    KALMAN = 'Kalman'
    TARGET = 'Target'

    def __str__(self):
        return self.value


@dataclass
class ImuOut:
    acc_x: np.ndarray
    acc_y: np.ndarray
    acc_z: np.ndarray

    ang_rate_x: np.ndarray
    ang_rate_y: np.ndarray
    ang_rate_z: np.ndarray

    mag_x: np.ndarray
    mag_y: np.ndarray
    mag_z: np.ndarray

    t_s: np.ndarray


def extract_imu_out(data: Signals) -> ImuOut:
    return ImuOut(
        acc_x=np.array(data.Imu.AccelerationX.val),
        acc_y=np.array(data.Imu.AccelerationY.val),
        acc_z=np.array(data.Imu.AccelerationZ.val),

        ang_rate_x=np.array(data.Imu.AngularRateX.val),
        ang_rate_y=np.array(data.Imu.AngularRateY.val),
        ang_rate_z=np.array(data.Imu.AngularRateZ.val),

        mag_x=np.array(data.Imu.MagneticFieldX.val),
        mag_y=np.array(data.Imu.MagneticFieldY.val),
        mag_z=np.array(data.Imu.MagneticFieldZ.val),

        t_s=np.array(data.Imu.AccelerationX.t_s)
    )


class AttEst(ABC):
    phi: np.ndarray
    theta: np.ndarray
    psi: np.ndarray

    phi_p: np.ndarray
    theta_p: np.ndarray
    psi_p: np.ndarray

    phi_pp: np.ndarray
    theta_pp: np.ndarray
    psi_pp: np.ndarray

    t_s: np.ndarray

    def __init__(self):
        self.phi = np.array([])
        self.theta = np.array([])
        self.psi = np.array([])

        self.phi_p = np.array([])
        self.theta_p = np.array([])
        self.psi_p = np.array([])

        self.phi_pp = np.array([])
        self.theta_pp = np.array([])
        self.psi_pp = np.array([])

        self.t_s = np.array([])

    def execute(self,
                imu_out: ImuOut,
                modulo_of_angles=True,
                static_gyro_offset_comp=False,
                dynamic_gyro_offset_comp=True,
                hard_iron_offset_comp=True):

        def pre_exec():
            self.t_s = imu_out.t_s

            _imu_out = deepcopy(imu_out)
            return self._imu_offset_comp(_imu_out,
                                         static_gyro_offset_comp,
                                         dynamic_gyro_offset_comp,
                                         hard_iron_offset_comp)

        def post_exec():
            if modulo_of_angles:
                self._modulo_of_angles()

        _imu_out = pre_exec()
        self._execute(_imu_out)
        post_exec()

    @abstractmethod
    def _execute(self, imu_out: ImuOut):
        pass

    def _to_phi(self, acc_y, acc_z):
        return np.arctan2(-acc_y, -acc_z)

    def _to_theta(self, acc_x, acc_y, acc_z):
        return np.arctan2(acc_x, np.sqrt(acc_y ** 2 + acc_z ** 2))

    def _to_psi(self, phi, theta, mag_x, mag_y, mag_z):
        b_x = mag_x * np.cos(theta) + \
              mag_y * np.sin(phi) * np.sin(theta) + \
              mag_z * np.sin(theta) * np.cos(phi)
        b_y = mag_y * np.cos(phi) - mag_z * np.sin(phi)

        return np.arctan2(-b_y, b_x)

    def _modulo_of_angles(self):
        self.phi = self._modulo_phi(self.phi)
        self.theta = self._modulo_phi(self.theta)
        self.psi = self._modulo_phi(self.psi)

    def _imu_offset_comp(self, imu_out: ImuOut,
                         static_gyro_offset_comp,
                         dynamic_gyro_offset_comp,
                         hard_iron_offset_comp) -> ImuOut:

        if static_gyro_offset_comp:
            imu_out.ang_rate_x = self._stat_gyro_offset_comp(imu_out.ang_rate_x)
            imu_out.ang_rate_y = self._stat_gyro_offset_comp(imu_out.ang_rate_y)
            imu_out.ang_rate_z = self._stat_gyro_offset_comp(imu_out.ang_rate_z)

        if dynamic_gyro_offset_comp:
            imu_out.ang_rate_x = self._dyn_gyro_offset_comp(imu_out.ang_rate_x)
            imu_out.ang_rate_y = self._dyn_gyro_offset_comp(imu_out.ang_rate_y)
            imu_out.ang_rate_z = self._dyn_gyro_offset_comp(imu_out.ang_rate_z)

        if hard_iron_offset_comp:
            imu_out.mag_x -= HARD_IRON_OFFSET_X
            imu_out.mag_y -= HARD_IRON_OFFSET_Y
            imu_out.mag_z -= HARD_IRON_OFFSET_Z

        return imu_out

    @staticmethod
    def _modulo_phi(x):
        return np.mod(x + np.pi, 2 * np.pi) - np.pi

    @staticmethod
    def _modulo_theta(x):
        return np.mod(x + np.pi / 2, np.pi) - np.pi / 2

    @staticmethod
    def _modulo_psi(x):
        return np.mod(x + np.pi, 2 * np.pi) - np.pi

    @staticmethod
    def _stat_gyro_offset_comp(v):
        return v - np.mean(v[0:STATIC_GYRO_OFFSET_COMP_SAMPLES])

    @staticmethod
    def _dyn_gyro_offset_comp(v):
        vp = np.diff(v, prepend=0) / IMU_SAMPLE_RATE_S
        indices = np.argwhere(np.abs(vp) < DYNAMIC_GYRO_OFFSET_COMP_ABS_ACC_LIM)
        offset = np.mean(v[indices[0:DYNAMIC_GYRO_OFFSET_COMP_SAMPLES]])

        return v - offset


class AttEstAccMag(AttEst):

    def _execute(self, imu_out: ImuOut):
        """
        Use the accelrometer and magnetometer to estimate angles (only).
        """
        self.phi = self._to_phi(imu_out.acc_y, imu_out.acc_z)
        self.theta = self._to_theta(imu_out.acc_x, imu_out.acc_y, imu_out.acc_z)
        self.psi = self._to_psi(self.phi, self.theta, imu_out.mag_x, imu_out.mag_y, imu_out.mag_z)


class AttEstGyro(AttEst):

    def _execute(self, imu_out: ImuOut):
        """
        Use the gyro for attitude estimation.
        """
        self._est_angles(imu_out)
        self._est_angular_rates(imu_out)
        self._est_angular_accelerations(imu_out)

    def _est_angles(self, imu_out: ImuOut):
        self.phi = np.cumsum(imu_out.ang_rate_x * IMU_SAMPLE_RATE_S)
        self.theta = np.cumsum(imu_out.ang_rate_y * IMU_SAMPLE_RATE_S)
        self.psi = np.cumsum(imu_out.ang_rate_z * IMU_SAMPLE_RATE_S)

    def _est_angular_rates(self, imu_out: ImuOut):
        self.phi_p = imu_out.ang_rate_x
        self.theta_p = imu_out.ang_rate_y
        self.psi_p = imu_out.ang_rate_z

    def _est_angular_accelerations(self, imu_out: ImuOut):
        self.phi_pp = np.diff(imu_out.ang_rate_x, prepend=0) / IMU_SAMPLE_RATE_S
        self.theta_pp = np.diff(imu_out.ang_rate_y, prepend=0) / IMU_SAMPLE_RATE_S
        self.psi_pp = np.diff(imu_out.ang_rate_z, prepend=0) / IMU_SAMPLE_RATE_S


class AttEstGyroLp(AttEst):

    def _execute(self, imu_out: ImuOut):
        """
        Use a LP-filtered (and derivative) of gyro for angular acceleration estimation.
        """
        phi_pp = np.diff(imu_out.ang_rate_x, prepend=0) / IMU_SAMPLE_RATE_S
        theta_pp = np.diff(imu_out.ang_rate_y, prepend=0) / IMU_SAMPLE_RATE_S
        psi_pp = np.diff(imu_out.ang_rate_z, prepend=0) / IMU_SAMPLE_RATE_S

        self.phi_pp = self._lp(phi_pp, fc=GYRO_LP_CUT_OFF_FREQ, dt=IMU_SAMPLE_RATE_S)
        self.theta_pp = self._lp(theta_pp, fc=GYRO_LP_CUT_OFF_FREQ, dt=IMU_SAMPLE_RATE_S)
        self.psi_pp = self._lp(psi_pp, fc=GYRO_LP_CUT_OFF_FREQ, dt=IMU_SAMPLE_RATE_S)

    def _lp(self, u, dt, fc):
        alpha = 1 / (1 + 1 / 2 / np.pi / dt / fc)

        y = np.zeros(len(u))
        y[0] = alpha * u[0]

        for k in range(1, len(u)):
            y[k] = alpha * u[k] + (1 - alpha) * y[k - 1]

        return y


class AttEstCf(AttEst):

    def _execute(self, imu_out: ImuOut):
        """
        Use a complementary filter to estimate angles (only).
        """
        phi_acc = self._to_phi(imu_out.acc_y, imu_out.acc_z)
        self.phi = self._complementary_filter(phi_acc, imu_out.ang_rate_x, CF_TAU_PHI)

        theta_acc = self._to_theta(imu_out.acc_x, imu_out.acc_y, imu_out.acc_z)
        self.theta = self._complementary_filter(theta_acc, imu_out.ang_rate_y, CF_TAU_THETA)

        psi_mag = self._to_psi(self.phi, self.theta, imu_out.mag_x, imu_out.mag_y, imu_out.mag_z)
        self.psi = self._complementary_filter(psi_mag, imu_out.ang_rate_z, CF_TAU_PSI)

    @staticmethod
    def _complementary_filter(u, up, tau):
        y = np.zeros(len(u))
        alpha = tau / (tau + IMU_SAMPLE_RATE_S)

        for k in range(1, len(u)):
            y[k] = alpha * (y[k - 1] + up[k] * IMU_SAMPLE_RATE_S) + (1 - alpha) * u[k]

        return y


class AttEstKalman(AttEst):
    P_0 = KALMAN_P_0
    Q = KALMAN_Q

    F = np.array([
        [1, IMU_SAMPLE_RATE_S, 0.5 * IMU_SAMPLE_RATE_S ** 2],
        [0, 1, IMU_SAMPLE_RATE_S],
        [0, 0, 1]
    ])
    H = np.array([
        [1, 0, 0],
        [0, 1, 0]
    ])

    def _execute(self, imu_out: ImuOut):
        """
        Use a Kalman filter for attitude estimation.
        """
        phi_acc = self._to_phi(imu_out.acc_y, imu_out.acc_z)
        self.phi, self.phi_p, self.phi_pp = self._kalman_filter(phi_acc, imu_out.ang_rate_x)

        theta_acc = self._to_theta(imu_out.acc_x, imu_out.acc_y, imu_out.acc_z)
        self.theta, self.theta_p, self.theta_pp = self._kalman_filter(theta_acc, imu_out.ang_rate_y)

        psi_mag = self._to_psi(self.phi, self.theta, imu_out.mag_x, imu_out.mag_y, imu_out.mag_z)
        self.psi, self.psi_p, self.psi_pp = self._kalman_filter(psi_mag, imu_out.ang_rate_z)

    def _kalman_filter(self, z, z_p):
        x = np.zeros((3, len(z)))
        P_pre = self.P_0

        z_var = pd.Series(z).rolling(window=KALMAN_R_WINDOW_SIZE).var()
        z_p_var = pd.Series(z_p).rolling(window=KALMAN_R_WINDOW_SIZE).var()

        for k in range(1, len(z)):
            x_pri = self.F @ x[:, k - 1]
            P_pri = self.F @ P_pre @ np.transpose(self.F) + self.Q

            R = self._get_R(z_var[k], z_p_var[k])

            S = self.H @ P_pri @ np.transpose(self.H) + R
            K = P_pri @ np.transpose(self.H) @ np.linalg.inv(S)

            x[:, k] = x_pri + K @ (np.array([z[k], z_p[k]]) - self.H @ x_pri)
            P_pre = (np.eye(3) - K @ self.H) @ P_pri

        return x[0, :], x[1, :], x[2, :]

    @staticmethod
    def _get_R(z_var, z_p_var):
        def _check_if_almost_zero_or_nan(x):
            if np.isnan(x) or x < KALMAN_R_ALMOST_ZERO:
                return KALMAN_R_ALMOST_ZERO
            else:
                return x

        return np.diag([KALMAN_R_0_SCALE * _check_if_almost_zero_or_nan(z_var),
                        KALMAN_R_1_SCALE * _check_if_almost_zero_or_nan(z_p_var)])


class AttEstTarget(AttEst):

    def _execute(self, imu_out: ImuOut):
        pass  # Note, extract_target_data should be used instead.

    def extract_target_data(self, data: Signals):
        """
        Extracts the target attitude estimation. Note, should be called
        after a possible execute to avoid alteration of states.
        """
        self.phi = np.array(data.StateEst.Roll.val)
        self.phi_p = np.array(data.StateEst.RollRate.val)
        self.phi_pp = np.array(data.StateEst.RollAcc.val)

        self.theta = np.array(data.StateEst.Pitch.val)
        self.theta_p = np.array(data.StateEst.PitchRate.val)
        self.theta_pp = np.array(data.StateEst.PitchAcc.val)

        self.psi = np.array(data.StateEst.Yaw.val)
        self.psi_p = np.array(data.StateEst.YawRate.val)
        self.psi_pp = np.array(data.StateEst.YawAcc.val)
