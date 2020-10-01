import argparse
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

from read_data_log import read_data_log, Signals

mpl.rcParams['lines.linewidth'] = 0.5

ROOT_PATH = Path(__file__).parent.absolute()

N_ESC = 4


def _finish_subplots(fig):
    for ax in fig.get_axes():
        ax.set(xlabel='Time [s]')
        ax.grid()
        if all(ax.get_legend_handles_labels()):
            ax.legend()


def _plot_sample_rate(axs, t_s, sensor):
    dt_s = np.diff(t_s)
    freq = np.divide(1, dt_s)
    mean_freq = np.mean(freq)

    label = "{}: {} Hz".format(sensor, np.array2string(mean_freq, precision=2))
    axs.plot(t_s[1:], freq, label=label)
    axs.set(ylabel='Sample rate [Hz]')


def _plot_imu(data):
    fig, axs = plt.subplots(4, 2)

    axs[0, 0].plot(data.Imu.AccelerationX.t_s, data.Imu.AccelerationX.val, label='X')
    axs[0, 0].plot(data.Imu.AccelerationY.t_s, data.Imu.AccelerationY.val, label='Y')
    axs[0, 0].plot(data.Imu.AccelerationZ.t_s, data.Imu.AccelerationZ.val, label='Z')
    axs[0, 0].set(ylabel='Acceleration [m/s]')

    axs[0, 1].plot(data.Imu.AngularRateX.t_s, data.Imu.AngularRateX.val, label='X')
    axs[0, 1].plot(data.Imu.AngularRateY.t_s, data.Imu.AngularRateY.val, label='Y')
    axs[0, 1].plot(data.Imu.AngularRateZ.t_s, data.Imu.AngularRateZ.val, label='Z')
    axs[0, 1].set(ylabel='Angular rate [rad/s]')

    axs[1, 0].plot(data.Imu.MagneticFieldX.t_s, data.Imu.MagneticFieldX.val, label='X')
    axs[1, 0].plot(data.Imu.MagneticFieldY.t_s, data.Imu.MagneticFieldY.val, label='Y')
    axs[1, 0].plot(data.Imu.MagneticFieldZ.t_s, data.Imu.MagneticFieldZ.val, label='Z')
    axs[1, 0].set(ylabel='Magnetic field [gauss]')

    axs[1, 1].plot(data.Imu.Pressure.t_s, data.Imu.Pressure.val)
    axs[1, 1].set(ylabel='Pressure [Pa]')

    axs[2, 0].plot(data.Imu.Temperature.t_s, data.Imu.Temperature.val)
    axs[2, 0].set(ylabel='Ambient temperature [C]')

    axs[2, 1].plot(data.Imu.AccMagStatus.t_s, data.Imu.AccMagStatus.val, label='AccMag')
    axs[2, 1].plot(data.Imu.GyroStatus.t_s, data.Imu.GyroStatus.val, label='Gyro')
    axs[2, 1].plot(data.Imu.BarometerStatus.t_s, data.Imu.BarometerStatus.val, label='Barometer')
    axs[2, 1].set(ylabel='Status [-]')

    _plot_sample_rate(axs[3, 0], data.Imu.AccelerationX.t_s, 'AccMag')
    _plot_sample_rate(axs[3, 0], data.Imu.MagneticFieldX.t_s, 'Gyro')
    _plot_sample_rate(axs[3, 0], data.Imu.Pressure.t_s, 'Barometer')

    _finish_subplots(fig)


def _plot_esc(data: Signals):
    fig, axs = plt.subplots(4, 2)

    _add_esc_features(axs[0, 0], data, 'AngularRate')
    axs[0, 0].set(ylabel='Angular rate [rpm]')

    _add_esc_features(axs[0, 1], data, 'Voltage')
    axs[0, 1].set(ylabel='Voltage [V]')

    _add_esc_features(axs[1, 0], data, 'Current')
    axs[1, 0].set(ylabel='Current [A]')

    _add_esc_features(axs[1, 1], data, 'Temperature')
    axs[1, 1].set(ylabel='Temperature [C]')

    _add_esc_features(axs[2, 0], data, 'IsAlive')
    axs[2, 0].set(ylabel='IsAlive [-]')

    _add_esc_features(axs[2, 1], data, 'Status')
    axs[2, 1].set(ylabel='Status [-]')

    _plot_sample_rate(axs[3, 0], data.Esc.Status0.t_s, 'Esc')

    _finish_subplots(fig)


def _add_esc_features(ax, data, feature):
    for i_esc in range(N_ESC):
        i_feature = feature + str(i_esc)
        signal = getattr(data.Esc, i_feature)

        ax.plot(signal.t_s, signal.val, label=i_feature)


def _plot_rc(data: Signals):
    fig, axs = plt.subplots(3, 2)

    axs[0, 0].plot(data.Rc.GimbalLeftX.t_s, data.Rc.GimbalLeftX.val, label='X')
    axs[0, 0].plot(data.Rc.GimbalLeftY.t_s, data.Rc.GimbalLeftY.val, label='Y')
    axs[0, 0].set(ylabel='Gimbal left [-]')

    axs[0, 1].plot(data.Rc.GimbalRightX.t_s, data.Rc.GimbalRightX.val, label='Z')
    axs[0, 1].plot(data.Rc.GimbalRightY.t_s, data.Rc.GimbalRightY.val, label='Z')
    axs[0, 1].set(ylabel='Gimbal right [-]')

    axs[1, 0].plot(data.Rc.SwitchLeft.t_s, data.Rc.SwitchLeft.val, label='Left')
    axs[1, 0].plot(data.Rc.SwitchRight.t_s, data.Rc.SwitchRight.val, label='Right')
    axs[1, 0].plot(data.Rc.SwitchMiddle.t_s, data.Rc.SwitchMiddle.val, label='Middle')
    axs[1, 0].set(ylabel='Switches [-]')

    axs[1, 1].plot(data.Rc.Knob.t_s, data.Rc.Knob.val)
    axs[1, 1].set(ylabel='Knob [-]')

    axs[2, 0].plot(data.Rc.Status.t_s, data.Rc.Status.val)
    axs[2, 0].set(ylabel='Status [-]')

    _plot_sample_rate(axs[2, 1], data.Rc.GimbalLeftX.t_s, 'rc')

    _finish_subplots(fig)


def main():
    parser = argparse.ArgumentParser(description='Analyze data log file.')
    parser.add_argument('path', type=Path, help='Path to data log file')
    args = parser.parse_args()

    data = read_data_log(args.path)

    _plot_imu(data)
    _plot_esc(data)
    _plot_rc(data)

    plt.show()


if __name__ == "__main__":
    main()
