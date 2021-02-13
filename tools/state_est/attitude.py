import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import List

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from cycler import cycler

import data_log.io as data_log_io

mpl.rcParams['axes.prop_cycle'] = cycler(color='bgrcmyk')
mpl.rcParams['lines.linewidth'] = 0.5

IMU_SAMPLE_RATE_S = 0.02  # 50 Hz

N_SAMPLES_FOR_OFFSET_COMP = 10

CUT_OFF_FREQ = 0.2  # [Hz]

TAU_PHI = 1 / (2 * np.pi * CUT_OFF_FREQ)
TAU_THETA = 1 / (2 * np.pi * CUT_OFF_FREQ)
TAU_PSI = 1 / (2 * np.pi * CUT_OFF_FREQ)

HARD_IRON_OFFSET_X = 0.131
HARD_IRON_OFFSET_Y = 0.143
HARD_IRON_OFFSET_Z = -0.144


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


def _estimate_attitude(data) -> Attitude:
    """ Get signals """
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
    b_x = mag_x * np.cos(theta) + mag_y * np.sin(phi) * np.sin(theta) + mag_z * np.sin(theta) * np.cos(phi)
    b_y = mag_y * np.cos(phi) - mag_z * np.sin(phi)
    psi_mag = np.arctan2(-b_y, b_x)

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


def _complementary_filter(u, up, tau):
    y = np.zeros(len(u))
    alpha = tau / (tau + IMU_SAMPLE_RATE_S)

    for k in range(2, len(u)):
        y[k] = alpha * (y[k - 1] + up[k] * IMU_SAMPLE_RATE_S) + (1 - alpha) * u[k]

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

    data = data_log_io.read(args.path, resample_to_fixed_rate_s=IMU_SAMPLE_RATE_S)
    att = _estimate_attitude(data)
    _plot_attitude(att)

    plt.show()


if __name__ == "__main__":
    main()
