from abc import abstractmethod, ABC
from copy import deepcopy
from dataclasses import dataclass, field
from enum import Enum

import numpy as np

from ugglan_tools.data_log.io import Signals

IMU_SAMPLE_RATE_S = 0.01  # 100 Hz


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
    )


@dataclass
class State:
    phi: np.ndarray = field(default_factory=lambda: np.array([]))
    theta: np.ndarray = field(default_factory=lambda: np.array([]))
    psi: np.ndarray = field(default_factory=lambda: np.array([]))

    phi_p: np.ndarray = field(default_factory=lambda: np.array([]))
    theta_p: np.ndarray = field(default_factory=lambda: np.array([]))
    psi_p: np.ndarray = field(default_factory=lambda: np.array([]))

    phi_pp: np.ndarray = field(default_factory=lambda: np.array([]))
    theta_pp: np.ndarray = field(default_factory=lambda: np.array([]))
    psi_pp: np.ndarray = field(default_factory=lambda: np.array([]))


class AttEst(ABC):
    STATIC_GYRO_BIAS_COMP_SAMPLES = 100

    DYNAMIC_GYRO_BIAS_COMP_SAMPLES = 100
    DYNAMIC_GYRO_BIAS_COMP_ABS_ACC_LIM = 0.2  # [rad/s^2]

    TARGET_ACC_ERROR_S_X = 1.0121
    TARGET_ACC_ERROR_S_Y = 1.0073
    TARGET_ACC_ERROR_S_Z = 0.9958

    TARGET_ACC_ERROR_M_X_Y = -0.0153
    TARGET_ACC_ERROR_M_X_Z = 0.0165
    TARGET_ACC_ERROR_M_Y_X = 0.0141
    TARGET_ACC_ERROR_M_Y_Z = 0.0024
    TARGET_ACC_ERROR_M_Z_X = -0.0135
    TARGET_ACC_ERROR_M_Z_Y = -0.0039

    TARGET_ACC_ERROR_B_X = -0.1477
    TARGET_ACC_ERROR_B_Y = 0.0013
    TARGET_ACC_ERROR_B_Z = -0.4554

    TARGET_HARD_IRON_BIAS_X = 0.104
    TARGET_HARD_IRON_BIAS_Y = 0.076
    TARGET_HARD_IRON_BIAS_Z = 0.062

    def __init__(self, dt: float):
        self._dt = dt
        self._state = State()

    def execute(self,
                imu_out: ImuOut,
                modulo_of_angles: bool,
                static_acc_error_comp: bool,
                static_gyro_bias_comp: bool,
                dynamic_gyro_bias_comp: bool,
                hard_iron_bias_comp: bool):

        def pre_exec():
            _imu_out = deepcopy(imu_out)
            return self._imu_error_comp(_imu_out,
                                        static_acc_error_comp,
                                        static_gyro_bias_comp,
                                        dynamic_gyro_bias_comp,
                                        hard_iron_bias_comp)

        def post_exec():
            if modulo_of_angles:
                self._modulo_of_angles()

        _imu_out = pre_exec()
        self._execute(_imu_out)
        post_exec()

    def get_state(self):
        return self._state

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
        self._state.phi = self._modulo_phi(self._state.phi)
        self._state.theta = self._modulo_phi(self._state.theta)
        self._state.psi = self._modulo_phi(self._state.psi)

    def _imu_error_comp(self,
                        imu_out: ImuOut,
                        static_acc_error_comp: bool,
                        static_gyro_bias_comp: bool,
                        dynamic_gyro_bias_comp: bool,
                        hard_iron_bias_comp) -> ImuOut:

        if static_acc_error_comp:
            imu_out.acc_x = self.TARGET_ACC_ERROR_S_X * imu_out.acc_x + \
                            self.TARGET_ACC_ERROR_M_X_Y * imu_out.acc_y + \
                            self.TARGET_ACC_ERROR_M_X_Z * imu_out.acc_z + \
                            self.TARGET_ACC_ERROR_B_X

            imu_out.acc_y = self.TARGET_ACC_ERROR_S_Y * imu_out.acc_y + \
                            self.TARGET_ACC_ERROR_M_Y_X * imu_out.acc_x + \
                            self.TARGET_ACC_ERROR_M_Y_Z * imu_out.acc_z + \
                            self.TARGET_ACC_ERROR_B_Y

            imu_out.acc_z = self.TARGET_ACC_ERROR_S_Z * imu_out.acc_z + \
                            self.TARGET_ACC_ERROR_M_Z_X * imu_out.acc_x + \
                            self.TARGET_ACC_ERROR_M_Z_Y * imu_out.acc_y + \
                            self.TARGET_ACC_ERROR_B_Z

        if static_gyro_bias_comp:
            imu_out.ang_rate_x = self._stat_gyro_bias_comp(imu_out.ang_rate_x)
            imu_out.ang_rate_y = self._stat_gyro_bias_comp(imu_out.ang_rate_y)
            imu_out.ang_rate_z = self._stat_gyro_bias_comp(imu_out.ang_rate_z)

        if dynamic_gyro_bias_comp:
            imu_out.ang_rate_x = self._dyn_gyro_bias_comp(imu_out.ang_rate_x)
            imu_out.ang_rate_y = self._dyn_gyro_bias_comp(imu_out.ang_rate_y)
            imu_out.ang_rate_z = self._dyn_gyro_bias_comp(imu_out.ang_rate_z)

        if hard_iron_bias_comp:
            imu_out.mag_x -= self.TARGET_HARD_IRON_BIAS_X
            imu_out.mag_y -= self.TARGET_HARD_IRON_BIAS_Y
            imu_out.mag_z -= self.TARGET_HARD_IRON_BIAS_Z

        return imu_out

    def _stat_gyro_bias_comp(self, v):
        return v - np.mean(v[0:self.STATIC_GYRO_BIAS_COMP_SAMPLES])

    def _dyn_gyro_bias_comp(self, v):
        vp = np.diff(v, prepend=0) / self._dt
        indices = np.argwhere(np.abs(vp) < self.DYNAMIC_GYRO_BIAS_COMP_ABS_ACC_LIM)
        bias = np.mean(v[indices[0:self.DYNAMIC_GYRO_BIAS_COMP_SAMPLES]])

        return v - bias

    @staticmethod
    def _modulo_phi(x):
        return np.mod(x + np.pi, 2 * np.pi) - np.pi

    @staticmethod
    def _modulo_theta(x):
        return np.mod(x + np.pi / 2, np.pi) - np.pi / 2

    @staticmethod
    def _modulo_psi(x):
        return np.mod(x + np.pi, 2 * np.pi) - np.pi


class AttEstAccMag(AttEst):
    """
    Use the accelrometer and magnetometer to estimate angles (only).
    """

    def _execute(self, imu_out: ImuOut):
        self._state.phi = self._to_phi(imu_out.acc_y, imu_out.acc_z)
        self._state.theta = self._to_theta(imu_out.acc_x, imu_out.acc_y, imu_out.acc_z)
        self._state.psi = self._to_psi(self._state.phi, self._state.theta, imu_out.mag_x, imu_out.mag_y, imu_out.mag_z)


class AttEstGyro(AttEst):
    """
    Use the gyro for attitude estimation.
    """

    def _execute(self, imu_out: ImuOut):
        self._est_angles(imu_out)
        self._est_angular_rates(imu_out)
        self._est_angular_accelerations(imu_out)

    def _est_angles(self, imu_out: ImuOut):
        self._state.phi = np.cumsum(imu_out.ang_rate_x * self._dt)
        self._state.theta = np.cumsum(imu_out.ang_rate_y * self._dt)
        self._state.psi = np.cumsum(imu_out.ang_rate_z * self._dt)

    def _est_angular_rates(self, imu_out: ImuOut):
        self._state.phi_p = imu_out.ang_rate_x
        self._state.theta_p = imu_out.ang_rate_y
        self._state.psi_p = imu_out.ang_rate_z

    def _est_angular_accelerations(self, imu_out: ImuOut):
        self._state.phi_pp = np.diff(imu_out.ang_rate_x, prepend=0) / self._dt
        self._state.theta_pp = np.diff(imu_out.ang_rate_y, prepend=0) / self._dt
        self._state.psi_pp = np.diff(imu_out.ang_rate_z, prepend=0) / self._dt


class AttEstGyroLp(AttEst):
    """
    Use a LP-filtered (and derivative) of gyro for angular acceleration estimation.
    """
    LP_CUT_OFF_FREQ = 15  # [Hz]

    def _execute(self, imu_out: ImuOut):
        phi_pp = np.diff(imu_out.ang_rate_x, prepend=0) / self._dt
        theta_pp = np.diff(imu_out.ang_rate_y, prepend=0) / self._dt
        psi_pp = np.diff(imu_out.ang_rate_z, prepend=0) / self._dt

        self._state.phi_pp = self._lp(phi_pp, fc=self.LP_CUT_OFF_FREQ, dt=self._dt)
        self._state.theta_pp = self._lp(theta_pp, fc=self.LP_CUT_OFF_FREQ, dt=self._dt)
        self._state.psi_pp = self._lp(psi_pp, fc=self.LP_CUT_OFF_FREQ, dt=self._dt)

    def _lp(self, u, dt, fc):
        alpha = 1 / (1 + 1 / 2 / np.pi / dt / fc)

        y = np.zeros(len(u))
        y[0] = alpha * u[0]

        for k in range(1, len(u)):
            y[k] = alpha * u[k] + (1 - alpha) * y[k - 1]

        return y


class AttEstCf(AttEst):
    """
    Use a complementary filter to estimate angles (only).
    """
    CUT_OFF_FREQ_PHI = 0.2  # [Hz]
    CUT_OFF_FREQ_THETA = 0.2  # [Hz]
    CUT_OFF_FREQ_PSI = 0.2  # [Hz]

    def _execute(self, imu_out: ImuOut):
        phi_acc = self._to_phi(imu_out.acc_y, imu_out.acc_z)
        self._state.phi = self._complementary_filter(phi_acc, imu_out.ang_rate_x, self.CUT_OFF_FREQ_PHI)

        theta_acc = self._to_theta(imu_out.acc_x, imu_out.acc_y, imu_out.acc_z)
        self._state.theta = self._complementary_filter(theta_acc, imu_out.ang_rate_y, self.CUT_OFF_FREQ_THETA)

        psi_mag = self._to_psi(self._state.phi, self._state.theta, imu_out.mag_x, imu_out.mag_y, imu_out.mag_z)
        self._state.psi = self._complementary_filter(psi_mag, imu_out.ang_rate_z, self.CUT_OFF_FREQ_PSI)

    def _complementary_filter(self, u, up, freq):
        y = np.zeros(len(u))
        tau = 1 / (2 * np.pi * freq)
        alpha = tau / (tau + self._dt)

        for k in range(1, len(u)):
            y[k] = alpha * (y[k - 1] + up[k] * self._dt) + (1 - alpha) * u[k]

        return y


class AttEstKalman(AttEst):
    """
    Use a Kalman filter for attitude estimation.
    """
    Q_SCALE = 1e2

    R_0 = 1
    R_1 = 0.1

    def __init__(self, dt: float):
        super().__init__(dt)

        self.P_0 = np.identity(3)

        G = np.array([0.5 * dt ** 2, dt, 1])
        self.Q = self.Q_SCALE * np.outer(G, G)

        self.R = np.diag([self.R_0, self.R_1])

        self.F = np.array([
            [1, dt, 0.5 * dt ** 2],
            [0, 1, dt],
            [0, 0, 1]
        ])

        self.H = np.array([
            [1, 0, 0],
            [0, 1, 0]
        ])

    def _execute(self, imu_out: ImuOut):
        phi_acc = self._to_phi(imu_out.acc_y, imu_out.acc_z)
        self._state.phi, self._state.phi_p, self._state.phi_pp = self._kalman_filter(phi_acc, imu_out.ang_rate_x)

        theta_acc = self._to_theta(imu_out.acc_x, imu_out.acc_y, imu_out.acc_z)
        self._state.theta, self._state.theta_p, self._state.theta_pp = self._kalman_filter(theta_acc,
                                                                                           imu_out.ang_rate_y)

        psi_mag = self._to_psi(self._state.phi, self._state.theta, imu_out.mag_x, imu_out.mag_y, imu_out.mag_z)
        self._state.psi, self._state.psi_p, self._state.psi_pp = self._kalman_filter(psi_mag, imu_out.ang_rate_z)

    def _kalman_filter(self, z, z_p):
        x = np.zeros((3, len(z)))
        P = self.P_0

        for k in range(1, len(z)):
            x_pri = self.F @ x[:, k - 1]
            P_pri = self.F @ P @ np.transpose(self.F) + self.Q

            S = self.H @ P_pri @ np.transpose(self.H) + self.R
            K = P_pri @ np.transpose(self.H) @ np.linalg.inv(S)

            x[:, k] = x_pri + K @ (np.array([z[k], z_p[k]]) - self.H @ x_pri)
            P = (np.eye(3) - K @ self.H) @ P_pri

        return x[0, :], x[1, :], x[2, :]


class AttEstTarget(AttEst):
    """
    Extracts the target attitude estimation.
    """

    def _execute(self, imu_out: ImuOut):
        pass  # Note, extract_target_data should be used instead.

    def extract_target_data(self, data: Signals):
        """
        Note, should be called after a possible execute to avoid alteration of states.
        """
        self._state.phi = np.array(data.StateEst.Roll.val)
        self._state.phi_p = np.array(data.StateEst.RollRate.val)
        self._state.phi_pp = np.array(data.StateEst.RollAcc.val)

        self._state.theta = np.array(data.StateEst.Pitch.val)
        self._state.theta_p = np.array(data.StateEst.PitchRate.val)
        self._state.theta_pp = np.array(data.StateEst.PitchAcc.val)

        self._state.psi = np.array(data.StateEst.Yaw.val)
        self._state.psi_p = np.array(data.StateEst.YawRate.val)
        self._state.psi_pp = np.array(data.StateEst.YawAcc.val)
