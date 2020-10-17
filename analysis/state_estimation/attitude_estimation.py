import argparse
from pathlib import Path
from typing import List

import matplotlib as mpl
import matplotlib.pyplot as plt
from dataclasses import dataclass

from read_data_log import read_data_log, Signals

mpl.rcParams['lines.linewidth'] = 0.5


@dataclass
class Attitude:
    phi: List[float]
    theta: List[float]
    psi: List[float]

    phi_p: List[float]
    theta_p: List[float]
    psi_p: List[float]

    phi_pp: List[float]
    theta_pp: List[float]
    psi_pp: List[float]

    t_s: List[float]


def _estimate_attitude(data: Signals) -> Attitude:
    att = Attitude(phi=[0, 1], theta=[0, 1], psi=[0, 1],
                   phi_p=[0, 1], theta_p=[0, 1], psi_p=[0, 1],
                   phi_pp=[0, 1], theta_pp=[0, 1], psi_pp=[0, 1],
                   t_s=[0, 1])

    return att


def _plot_attitude(att: Attitude):
    fig, axs = plt.subplots(3, 1)

    axs[0].plot(att.t_s, att.phi, label=r'$\phi$')
    axs[0].plot(att.t_s, att.theta, label=r'$\theta$')
    axs[0].plot(att.t_s, att.psi, label=r'$\psi$')
    axs[0].set(title='Attitude Estimation', ylabel='Angles [rad]')

    axs[1].plot(att.t_s, att.phi_p, label=r'$\dot{\phi}$')
    axs[1].plot(att.t_s, att.theta_p, label=r'$\dot{\theta}$')
    axs[1].plot(att.t_s, att.psi_p, label=r'$\dot{\psi}$')
    axs[1].set(ylabel='Rates [rad/s]')

    axs[2].plot(att.t_s, att.phi_pp, label=r'$\ddot{\phi}$')
    axs[2].plot(att.t_s, att.theta_pp, label=r'$\ddot{\theta}$')
    axs[2].plot(att.t_s, att.psi_pp, label=r'$\ddot{\psi}$')
    axs[2].set(xlabel='Time [s]', ylabel='Accelerations [rad/s^2]')

    for ax in axs:
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
