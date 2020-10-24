import argparse
from pathlib import Path
from typing import List

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from cycler import cycler
from dataclasses import dataclass

from read_data_log import read_data_log, Signals, Signal

mpl.rcParams['axes.prop_cycle'] = cycler(color='bgrcmyk')
mpl.rcParams['lines.linewidth'] = 0.5

IMU_SAMPLE_RATE_S = 0.02  # 50 Hz

N_SAMPLES_FOR_OFFSET_COMP = 20

TAU_PHI = 0.08
TAU_THETA = 0.08
TAU_PSI = 0.04


@dataclass
class Attitude:
    phi: List[float]
    theta: List[float]
    psi: List[float]

    phi_acc: List[float]
    theta_acc: List[float]
    psi_mag: List[float]

    phi_gyro: List[float]
    theta_gyro: List[float]
    psi_gyro: List[float]

    t_s: List[float]


def _estimate_attitude(data: Signals) -> Attitude:
    """ Sync signals """
    t_s = np.arange(0, data.Imu.AccelerationX.t_s[-1], IMU_SAMPLE_RATE_S)

    acc_x = _sync_signal_in_time(data.Imu.AccelerationX, t_s)
    acc_y = _sync_signal_in_time(data.Imu.AccelerationY, t_s)
    acc_z = _sync_signal_in_time(data.Imu.AccelerationZ, t_s)

    ang_rate_x = _sync_signal_in_time(data.Imu.AngularRateX, t_s)
    ang_rate_y = _sync_signal_in_time(data.Imu.AngularRateY, t_s)
    ang_rate_z = _sync_signal_in_time(data.Imu.AngularRateZ, t_s)

    mag_x = _sync_signal_in_time(data.Imu.MagneticFieldX, t_s)
    mag_y = _sync_signal_in_time(data.Imu.MagneticFieldY, t_s)
    mag_z = _sync_signal_in_time(data.Imu.MagneticFieldZ, t_s)

    """ Offset compensate angular rates (gyro) """
    ang_rate_x -= np.mean(ang_rate_x[0:N_SAMPLES_FOR_OFFSET_COMP])
    ang_rate_y -= np.mean(ang_rate_y[0:N_SAMPLES_FOR_OFFSET_COMP])
    ang_rate_z -= np.mean(ang_rate_z[0:N_SAMPLES_FOR_OFFSET_COMP])

    """ Estimate phi, theta and psi using gyro """
    phi_gyro = np.cumsum(ang_rate_x * IMU_SAMPLE_RATE_S)
    theta_gyro = np.cumsum(ang_rate_y * IMU_SAMPLE_RATE_S)
    psi_gyro = np.cumsum(ang_rate_z * IMU_SAMPLE_RATE_S)

    """ Estimate phi and theta using acceleration and cf """
    phi_acc = np.arctan2(-acc_y, -acc_z)
    theta_acc = np.arctan2(acc_x, np.sqrt(acc_y ** 2 + acc_z ** 2))

    phi = _complementary_filter(phi_acc, ang_rate_x, TAU_PHI)
    theta = _complementary_filter(theta_acc, ang_rate_y, TAU_THETA)

    """ Estimate psi using magnetometer and cf"""
    b_fx = mag_x * np.cos(theta) + mag_y * np.sin(phi) * np.sin(theta) + mag_z * np.sin(theta) * np.cos(phi)
    b_fy = mag_y * np.cos(phi) - mag_z * np.sin(phi)
    psi_mag = np.arctan2(-b_fy, b_fx)

    psi = _complementary_filter(psi_mag, ang_rate_z, TAU_PSI)

    return Attitude(phi=_modulo_phi(phi),
                    theta=_modulo_theta(theta),
                    psi=_modulo_psi(psi),
                    phi_acc=_modulo_phi(phi_acc),
                    theta_acc=_modulo_theta(theta_acc),
                    psi_mag=_modulo_psi(psi_mag),
                    phi_gyro=_modulo_phi(phi_gyro),
                    theta_gyro=_modulo_theta(theta_gyro),
                    psi_gyro=_modulo_psi(psi_gyro),
                    t_s=t_s)


def _sync_signal_in_time(s: Signal, t_s):
    return np.interp(t_s, s.t_s, s.val)


def _complementary_filter(u, up, tau):
    y = np.zeros(len(u))
    alpha = tau / (tau + IMU_SAMPLE_RATE_S)

    for k in range(len(u) - 1):
        y[k + 1] = alpha * (y[k] + up[k] * IMU_SAMPLE_RATE_S) + (1 - alpha) * u[k]

    return y


def _modulo_phi(x):
    return np.mod(x + np.pi, 2 * np.pi) - np.pi


def _modulo_theta(x):
    return np.mod(x + np.pi / 2, np.pi) - np.pi / 2


def _modulo_psi(x):
    return np.mod(x + np.pi, 2 * np.pi) - np.pi


def _plot_attitude(att: Attitude):
    fig, axs = plt.subplots(3, 1)

    axs[0].plot(att.t_s, att.phi, label=r'$\phi_{cf}$')
    axs[0].plot(att.t_s, att.phi_acc, label=r'$\phi_{acc}$')
    axs[0].plot(att.t_s, att.phi_gyro, label=r'$\phi_{gyro}$')
    axs[0].set(title='Attitude Estimation', ylabel='Roll [rad]')

    axs[1].plot(att.t_s, att.theta, label=r'$\theta_{cf}$')
    axs[1].plot(att.t_s, att.theta_acc, label=r'$\theta_{acc}$')
    axs[1].plot(att.t_s, att.theta_gyro, label=r'$\theta_{gyro}$')
    axs[1].set(ylabel='Pitch [rad]')

    axs[2].plot(att.t_s, att.psi, label=r'$\psi_{cf}$')
    axs[2].plot(att.t_s, att.psi_mag, label=r'$\psi_{mag}$')
    axs[2].plot(att.t_s, att.psi_gyro, label=r'$\psi_{gyro}$')
    axs[2].set(xlabel='Time [s]', ylabel='Yaw [rad]')

    for ax in axs:
        ax.legend(loc='upper right')
        ax.grid()


def main():
    parser = argparse.ArgumentParser(description='Attitude estimation of data log file.')
    parser.add_argument('path', type=Path, help='Path to data log file')
    args = parser.parse_args()

    data = read_data_log(args.path)
    att = _estimate_attitude(data)
    _plot_attitude(att)

    plt.show()


if __name__ == "__main__":
    main()
