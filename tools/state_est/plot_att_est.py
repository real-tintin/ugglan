import argparse
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt

import data_log.io as data_log_io
from state_est.att_estimators import *

mpl.rcParams['lines.linewidth'] = 0.5

TARGET_IMU_SAMPLE_RATE_S = 0.02


class PlotAttEst:

    def __init__(self, t_s: np.ndarray):
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

        self.t_s = t_s

    def add(self, att_est: AttEst, name: str, color):
        plot = lambda axs, v, label: self._plot(axs, t_s=self.t_s, v=v, color=color, label=label)
        state = att_est.get_state()

        plot(self._axs[0], state.phi, r'$\phi_{{{}}}$'.format(name))
        plot(self._axs[1], state.theta, r'$\theta_{{{}}}$'.format(name))
        plot(self._axs[2], state.psi, r'$\psi_{{{}}}$'.format(name))

        plot(self._axs_p[0], state.phi_p, r'$\dot{{\phi}}_{{{}}}$'.format(name))
        plot(self._axs_p[1], state.theta_p, r'$\dot{{\theta}}_{{{}}}$'.format(name))
        plot(self._axs_p[2], state.psi_p, r'$\dot{{\psi}}_{{{}}}$'.format(name))

        plot(self._axs_pp[0], state.phi_pp, r'$\ddot{{\phi}}_{{{}}}$'.format(name))
        plot(self._axs_pp[1], state.theta_pp, r'$\ddot{{\theta}}_{{{}}}$'.format(name))
        plot(self._axs_pp[2], state.psi_pp, r'$\ddot{{\psi}}_{{{}}}$'.format(name))

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


def _create_and_exec_att_est(att_est_type: AttEst, imu_out: ImuOut):
    att_est = att_est_type(dt=TARGET_IMU_SAMPLE_RATE_S)
    att_est.execute(imu_out)

    return att_est


def _get_imu_t_s(data: Signals):
    return data.Imu.AccelerationX.t_s


def main():
    parser = argparse.ArgumentParser(description='Attitude estimation of data log file.')
    parser.add_argument('path', type=Path, help='Path to data log file')
    parser.add_argument('--estimator', type=Estimator, nargs='+', choices=list(Estimator),
                        default=list(Estimator), help='Selected estimator(s)')
    args = parser.parse_args()

    data = data_log_io.read(args.path, resample_to_fixed_rate_s=TARGET_IMU_SAMPLE_RATE_S)
    imu_out = extract_imu_out(data)
    plot_att_est = PlotAttEst(t_s=_get_imu_t_s(data))

    if Estimator.ACC_MAG in args.estimator:
        plot_att_est.add(_create_and_exec_att_est(AttEstAccMag, imu_out), name=Estimator.ACC_MAG, color='c')

    if Estimator.GYRO in args.estimator:
        plot_att_est.add(_create_and_exec_att_est(AttEstGyro, imu_out), name=Estimator.GYRO, color='r')

    if Estimator.GYRO_LP in args.estimator:
        plot_att_est.add(_create_and_exec_att_est(AttEstGyroLp, imu_out), name=Estimator.GYRO_LP, color='m')

    if Estimator.CF in args.estimator:
        plot_att_est.add(_create_and_exec_att_est(AttEstCf, imu_out), name=Estimator.CF, color='b')

    if Estimator.KALMAN in args.estimator:
        plot_att_est.add(_create_and_exec_att_est(AttEstKalman, imu_out), name=Estimator.KALMAN, color='g')

    if Estimator.TARGET in args.estimator:
        att_est_target = _create_and_exec_att_est(AttEstTarget, imu_out)
        att_est_target.extract_target_data(data)

        plot_att_est.add(att_est_target, name=Estimator.TARGET, color='k')

    plot_att_est.show()


if __name__ == "__main__":
    main()
