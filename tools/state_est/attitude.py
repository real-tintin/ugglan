import argparse
from abc import abstractmethod, ABC
from enum import Enum
from pathlib import Path

import data_log.io as data_log_io
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from dataclasses import dataclass

mpl.rcParams['lines.linewidth'] = 0.5

IMU_SAMPLE_RATE_S = 0.02  # 50 Hz

N_SAMPLES_GYRO_OFFSET_COMP = 10

CF_CUT_OFF_FREQ = 0.2  # [Hz]
CF_TAU_PHI = 1 / (2 * np.pi * CF_CUT_OFF_FREQ)
CF_TAU_THETA = 1 / (2 * np.pi * CF_CUT_OFF_FREQ)
CF_TAU_PSI = 1 / (2 * np.pi * CF_CUT_OFF_FREQ)

GYRO_LP_CUT_OFF_FREQ = 15  # [Hz]

KALMAN_P_0 = np.zeros((3, 3))
KALMAN_Q_VARIANCE = 1e1
KALMAN_G = np.array([0.5 * IMU_SAMPLE_RATE_S ** 2, IMU_SAMPLE_RATE_S, 1])
KALMAN_Q = KALMAN_Q_VARIANCE * np.outer(KALMAN_G, KALMAN_G)
KALMAN_R_VARIANCE = 1e-3
KALMAN_R = KALMAN_R_VARIANCE * np.diag([1, 1])

HARD_IRON_OFFSET_X = 0.131
HARD_IRON_OFFSET_Y = 0.143
HARD_IRON_OFFSET_Z = -0.144


class Estimator(Enum):
    ACC_MAG = 'AccMag'
    GYRO = 'Gyro'
    GYRO_LP = 'GyroLp'
    CF = 'Cf'
    KALMAN = 'Kalman'

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

    def __init__(self, imu_out: ImuOut):
        self.phi = np.array([])
        self.theta = np.array([])
        self.psi = np.array([])

        self.phi_p = np.array([])
        self.theta_p = np.array([])
        self.psi_p = np.array([])

        self.phi_pp = np.array([])
        self.theta_pp = np.array([])
        self.psi_pp = np.array([])

        self.t_s = imu_out.t_s

        self.execute(imu_out)
        self._modulo_of_angles()

    @abstractmethod
    def execute(self, imu_out: ImuOut):
        pass

    def to_phi(self, acc_y, acc_z):
        return np.arctan2(-acc_y, -acc_z)

    def to_theta(self, acc_x, acc_y, acc_z):
        return np.arctan2(acc_x, np.sqrt(acc_y ** 2 + acc_z ** 2))

    def to_psi(self, phi, theta, mag_x, mag_y, mag_z):
        b_x = mag_x * np.cos(theta) + \
              mag_y * np.sin(phi) * np.sin(theta) + \
              mag_z * np.sin(theta) * np.cos(phi)
        b_y = mag_y * np.cos(phi) - mag_z * np.sin(phi)

        return np.arctan2(-b_y, b_x)

    def _modulo_of_angles(self):
        self.phi = self._modulo_phi(self.phi)
        self.theta = self._modulo_phi(self.theta)
        self.psi = self._modulo_phi(self.psi)

    @staticmethod
    def _modulo_phi(x):
        return np.mod(x + np.pi, 2 * np.pi) - np.pi

    @staticmethod
    def _modulo_theta(x):
        return np.mod(x + np.pi / 2, np.pi) - np.pi / 2

    @staticmethod
    def _modulo_psi(x):
        return np.mod(x + np.pi, 2 * np.pi) - np.pi


def _extract_imu_out(data) -> ImuOut:
    t_s = np.array(data.Imu.AccelerationX.t_s)

    acc_x = np.array(data.Imu.AccelerationX.val)
    acc_y = np.array(data.Imu.AccelerationY.val)
    acc_z = np.array(data.Imu.AccelerationZ.val)

    ang_rate_x = np.array(data.Imu.AngularRateX.val)
    ang_rate_y = np.array(data.Imu.AngularRateY.val)
    ang_rate_z = np.array(data.Imu.AngularRateZ.val)

    mag_x = np.array(data.Imu.MagneticFieldX.val) - HARD_IRON_OFFSET_X
    mag_y = np.array(data.Imu.MagneticFieldY.val) - HARD_IRON_OFFSET_Y
    mag_z = np.array(data.Imu.MagneticFieldZ.val) - HARD_IRON_OFFSET_Z

    ang_rate_x -= np.mean(ang_rate_x[0:N_SAMPLES_GYRO_OFFSET_COMP])
    ang_rate_y -= np.mean(ang_rate_y[0:N_SAMPLES_GYRO_OFFSET_COMP])
    ang_rate_z -= np.mean(ang_rate_z[0:N_SAMPLES_GYRO_OFFSET_COMP])

    return ImuOut(
        acc_x=acc_x,
        acc_y=acc_y,
        acc_z=acc_z,

        ang_rate_x=ang_rate_x,
        ang_rate_y=ang_rate_y,
        ang_rate_z=ang_rate_z,

        mag_x=mag_x,
        mag_y=mag_y,
        mag_z=mag_z,

        t_s=t_s,
    )


class AttEstAccMag(AttEst):

    def execute(self, imu_out: ImuOut):
        """
        Use the accelrometer and magnetometer to estimate angles (only).
        """
        self.phi = self.to_phi(imu_out.acc_y, imu_out.acc_z)
        self.theta = self.to_theta(imu_out.acc_x, imu_out.acc_y, imu_out.acc_z)
        self.psi = self.to_psi(self.phi, self.theta, imu_out.mag_x, imu_out.mag_y, imu_out.mag_z)


class AttEstGyro(AttEst):

    def execute(self, imu_out: ImuOut):
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

    def execute(self, imu_out: ImuOut):
        """
        Use a LP gyro for angular acceleration estimation.
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

    def execute(self, imu_out: ImuOut):
        """
        Use a complementary filter to estimate angles (only).
        """
        phi_acc = self.to_phi(imu_out.acc_y, imu_out.acc_z)
        self.phi = self._complementary_filter(phi_acc, imu_out.ang_rate_x, CF_TAU_PHI)

        theta_acc = self.to_theta(imu_out.acc_x, imu_out.acc_y, imu_out.acc_z)
        self.theta = self._complementary_filter(theta_acc, imu_out.ang_rate_y, CF_TAU_THETA)

        psi_mag = self.to_psi(self.phi, self.theta, imu_out.mag_x, imu_out.mag_y, imu_out.mag_z)
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
    R = KALMAN_R

    F = np.array([
        [1, IMU_SAMPLE_RATE_S, 0.5 * IMU_SAMPLE_RATE_S ** 2],
        [0, 1, IMU_SAMPLE_RATE_S],
        [0, 0, 1]
    ])
    H = np.array([
        [1, 0, 0],
        [0, 1, 0]
    ])

    def execute(self, imu_out: ImuOut):
        """
        Use a Kalman filter for attitude estimation.
        """
        phi_acc = self.to_phi(imu_out.acc_y, imu_out.acc_z)
        self.phi, self.phi_p, self.phi_pp = self._kalman_filter(phi_acc, imu_out.ang_rate_x)

        theta_acc = self.to_theta(imu_out.acc_x, imu_out.acc_y, imu_out.acc_z)
        self.theta, self.theta_p, self.theta_pp = self._kalman_filter(theta_acc, imu_out.ang_rate_y)

        psi_mag = self.to_psi(self.phi, self.theta, imu_out.mag_x, imu_out.mag_y, imu_out.mag_z)
        self.psi, self.psi_p, self.psi_pp = self._kalman_filter(psi_mag, imu_out.ang_rate_z)

    def _kalman_filter(self, z, z_p):
        x = np.zeros((3, len(z)))
        P_pre = self.P_0

        for k in range(1, len(z)):
            x_pri = self.F @ x[:, k - 1]
            P_pri = self.F @ P_pre @ np.transpose(self.F) + self.Q

            S = self.H @ P_pri @ np.transpose(self.H) + self.R
            K = P_pri @ np.transpose(self.H) @ np.linalg.inv(S)

            x[:, k] = x_pri + K @ (np.array([z[k], z_p[k]]) - self.H @ x_pri)
            P_pre = (np.eye(3) - K @ self.H) @ P_pri

        return x[0, :], x[1, :], x[2, :]


class PlotAttEst:

    def __init__(self):
        self._fig, self._axs = plt.subplots(3, 1)
        self._axs[0].set(title='Angles', ylabel='Roll [rad]')
        self._axs[1].set(ylabel='Pitch [rad]')
        self._axs[2].set(xlabel='Time [s]', ylabel='Yaw [rad]')

        self._fig_p, self._axs_p = plt.subplots(3, 1)
        self._axs_p[0].set(title='Angular-rates', ylabel='Roll-rate [rad/s]')
        self._axs_p[1].set(ylabel='Pitch-rate [rad/s]')
        self._axs_p[2].set(xlabel='Time [s]', ylabel='Yaw-rate [rad/s]')

        self._fig_pp, self._axs_pp = plt.subplots(3, 1)
        self._axs_pp[0].set(title='Angular-accelerations', ylabel='Roll-acc [rad/s^2]')
        self._axs_pp[1].set(ylabel='Pitch-acc [rad/s^2]')
        self._axs_pp[2].set(xlabel='Time [s]', ylabel='Yaw-acc [rad/s^2]')

    def add(self, att_est: AttEst, name: str, color):
        plot = lambda axs, v, label: self._plot(axs, t_s=att_est.t_s, v=v, color=color, label=label)

        plot(self._axs[0], att_est.phi, r'$\phi_{{{}}}$'.format(name))
        plot(self._axs[1], att_est.theta, r'$\theta_{{{}}}$'.format(name))
        plot(self._axs[2], att_est.psi, r'$\psi_{{{}}}$'.format(name))

        plot(self._axs_p[0], att_est.phi_p, r'$\dot{{\phi}}_{{{}}}$'.format(name))
        plot(self._axs_p[1], att_est.theta_p, r'$\dot{{\theta}}_{{{}}}$'.format(name))
        plot(self._axs_p[2], att_est.psi_p, r'$\dot{{\psi}}_{{{}}}$'.format(name))

        plot(self._axs_pp[0], att_est.phi_pp, r'$\ddot{{\phi}}_{{{}}}$'.format(name))
        plot(self._axs_pp[1], att_est.theta_pp, r'$\ddot{{\theta}}_{{{}}}$'.format(name))
        plot(self._axs_pp[2], att_est.psi_pp, r'$\ddot{{\psi}}_{{{}}}$'.format(name))

    def show(self):
        for ax in [*self._axs, *self._axs_p, *self._axs_pp]:
            h_legend, _ = ax.get_legend_handles_labels()
            if len(h_legend) > 0:
                ax.legend(loc='upper right')
            ax.grid()

        plt.show()

    @staticmethod
    def _plot(axs, t_s, v, label, color):
        if v.size > 0:
            axs.plot(t_s, v, color=color, label=label)


def main():
    parser = argparse.ArgumentParser(description='Attitude estimation of data log file.')
    parser.add_argument('path', type=Path, help='Path to data log file')
    parser.add_argument('--estimator', type=Estimator, nargs='+', choices=list(Estimator),
                        default=list(Estimator), help='Selected estimator(s)')
    args = parser.parse_args()

    data = data_log_io.read(args.path, resample_to_fixed_rate_s=IMU_SAMPLE_RATE_S)
    imu_out = _extract_imu_out(data)
    plot_att_est = PlotAttEst()

    if Estimator.ACC_MAG in args.estimator:
        plot_att_est.add(AttEstAccMag(imu_out), name=Estimator.ACC_MAG, color='c')

    if Estimator.GYRO in args.estimator:
        plot_att_est.add(AttEstGyro(imu_out), name=Estimator.GYRO, color='r')

    if Estimator.GYRO_LP in args.estimator:
        plot_att_est.add(AttEstGyroLp(imu_out), name=Estimator.GYRO_LP, color='m')

    if Estimator.CF in args.estimator:
        plot_att_est.add(AttEstCf(imu_out), name=Estimator.CF, color='b')

    if Estimator.KALMAN in args.estimator:
        plot_att_est.add(AttEstKalman(imu_out), name=Estimator.KALMAN, color='g')

    plot_att_est.show()


if __name__ == "__main__":
    main()
