import argparse
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from cycler import cycler

from read_data_log import read_data_log, Signals
from task_ids import TaskId

mpl.rcParams['axes.prop_cycle'] = cycler(color='bgrcmyk')
mpl.rcParams['lines.linewidth'] = 0.5

N_ESC = 4


def _finish_subplots(fig):
    for ax in fig.get_axes():
        ax.set(xlabel='Time [s]')
        ax.grid()
        if all(ax.get_legend_handles_labels()):
            ax.legend()


def _plot_task_sample_rate(axs, signal, task_id, task_name):
    val = np.array(signal.val)
    t_s = np.array(signal.t_s)
    use_idx = val == task_id.value
    t_s = t_s[use_idx]

    dt_s = np.diff(t_s)
    dt_s[dt_s == 0] = np.NaN
    freq = np.divide(1, dt_s)
    mean_freq = np.nanmean(freq)

    label = "{}: {} Hz".format(task_name, np.array2string(mean_freq, precision=2))
    axs.plot(t_s[1:], freq, label=label)
    axs.set(ylabel='Sample rate [Hz]')


def _plot_imu(data):
    fig, axs = plt.subplots(3, 2)

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

    _finish_subplots(fig)


def _plot_esc(data: Signals):
    fig, axs = plt.subplots(3, 2)

    _add_esc_features(axs[0, 0], data, 'AngularRate')
    axs[0, 0].set(ylabel='Angular rate [rad/s]')

    _add_esc_features(axs[0, 1], data, 'Voltage')
    axs[0, 1].set(ylabel='Voltage [V]')

    _add_esc_features(axs[1, 0], data, 'Current')
    axs[1, 0].set(ylabel='Current [A]')

    _add_esc_features(axs[1, 1], data, 'Temperature')
    axs[1, 1].set(ylabel='Temperature [C]')

    _add_esc_features(axs[2, 0], data, 'MotorCmd')
    axs[2, 0].set(ylabel='MotorCmd [-]')

    _add_esc_features(axs[2, 1], data, 'IsAlive')
    _add_esc_features(axs[2, 1], data, 'Status')
    axs[2, 1].set(ylabel='Flags [-]')

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

    axs[0, 1].plot(data.Rc.GimbalRightX.t_s, data.Rc.GimbalRightX.val, label='X')
    axs[0, 1].plot(data.Rc.GimbalRightY.t_s, data.Rc.GimbalRightY.val, label='Y')
    axs[0, 1].set(ylabel='Gimbal right [-]')

    axs[1, 0].plot(data.Rc.SwitchLeft.t_s, data.Rc.SwitchLeft.val, label='Left')
    axs[1, 0].plot(data.Rc.SwitchRight.t_s, data.Rc.SwitchRight.val, label='Right')
    axs[1, 0].plot(data.Rc.SwitchMiddle.t_s, data.Rc.SwitchMiddle.val, label='Middle')
    axs[1, 0].set(ylabel='Switches [-]')

    axs[1, 1].plot(data.Rc.Knob.t_s, data.Rc.Knob.val)
    axs[1, 1].set(ylabel='Knob [-]')

    axs[2, 0].plot(data.Rc.Status.t_s, data.Rc.Status.val)
    axs[2, 0].set(ylabel='Status [-]')

    _finish_subplots(fig)


def _plot_state_est(data: Signals):
    fig, axs = plt.subplots(2, 2)

    axs[0, 0].plot(data.StateEst.Roll.t_s, data.StateEst.Roll.val, label=r'$\phi$')
    axs[0, 0].plot(data.StateEst.Pitch.t_s, data.StateEst.Pitch.val, label=r'$\theta$')
    axs[0, 0].plot(data.StateEst.Yaw.t_s, data.StateEst.Yaw.val, label=r'$\psi$')
    axs[0, 0].set(ylabel='Angle [rad]')

    axs[0, 1].plot(data.StateEst.RollRate.t_s, data.StateEst.RollRate.val, label=r'$\dot{\phi}$')
    axs[0, 1].plot(data.StateEst.PitchRate.t_s, data.StateEst.PitchRate.val, label=r'$\dot{\theta}$')
    axs[0, 1].plot(data.StateEst.YawRate.t_s, data.StateEst.YawRate.val, label=r'$\dot{\psi}$')
    axs[0, 1].set(ylabel='Angular-rate [rad/s]')

    axs[1, 0].plot(data.StateEst.AttIsCalib.t_s, data.StateEst.AttIsCalib.val, label='AttIsCalibrated')
    axs[1, 0].set(ylabel='Status [-]')

    _finish_subplots(fig)


def _plot_state_ctrl(data: Signals):
    fig, axs = plt.subplots(2, 2)

    axs[0, 0].plot(data.StateEst.Roll.t_s, data.StateEst.Roll.val, label=r'$\phi$ [rad]')
    axs[0, 0].plot(data.StateCtrl.RollRef.t_s, data.StateCtrl.RollRef.val, label=r'$\phi_r$ [rad]')
    axs[0, 0].plot(data.StateCtrl.Mx.t_s, data.StateCtrl.Mx.val, label=r'$u_{Mx}$ [Nm]')

    axs[0, 1].plot(data.StateEst.Pitch.t_s, data.StateEst.Pitch.val, label=r'$\theta$ [rad]')
    axs[0, 1].plot(data.StateCtrl.PitchRef.t_s, data.StateCtrl.PitchRef.val, label=r'$\theta_r$ [rad]')
    axs[0, 1].plot(data.StateCtrl.My.t_s, data.StateCtrl.My.val, label=r'$u_{My}$ [Nm]')

    axs[1, 0].plot(data.StateEst.YawRate.t_s, data.StateEst.YawRate.val, label=r'$\dot{\psi}$ [rad/s]')
    axs[1, 0].plot(data.StateCtrl.YawRateRef.t_s, data.StateCtrl.YawRateRef.val, label=r'$\dot{\psi}_r$ [rad/s]')
    axs[1, 0].plot(data.StateCtrl.Mz.t_s, data.StateCtrl.Mz.val, label=r'$u_{Mz}$ [Nm]')

    axs[1, 1].plot(data.StateCtrl.FzRef.t_s, data.StateCtrl.FzRef.val, label=r'$F_{zr}$ [N]')
    axs[1, 1].plot(data.StateCtrl.Fz.t_s, data.StateCtrl.Fz.val, label=r'$u_{Fz}$ [N]')

    _finish_subplots(fig)


def _plot_tasks(data: Signals):
    fig, axs = plt.subplots(3, 2)

    _plot_task_sample_rate(axs[0, 0], data.Task.Execute, TaskId.AccMag, 'AccMag')
    _plot_task_sample_rate(axs[0, 0], data.Task.Execute, TaskId.Gyro, 'Gyro')
    _plot_task_sample_rate(axs[0, 0], data.Task.Execute, TaskId.Barometer, 'Barometer')

    _plot_task_sample_rate(axs[0, 1], data.Task.Execute, TaskId.EscRead0, 'EscRead0')
    _plot_task_sample_rate(axs[0, 1], data.Task.Execute, TaskId.EscRead1, 'EscRead1')
    _plot_task_sample_rate(axs[0, 1], data.Task.Execute, TaskId.EscRead2, 'EscRead2')
    _plot_task_sample_rate(axs[0, 1], data.Task.Execute, TaskId.EscRead3, 'EscRead3')

    _plot_task_sample_rate(axs[1, 0], data.Task.Execute, TaskId.EscWrite0, 'EscWrite0')
    _plot_task_sample_rate(axs[1, 0], data.Task.Execute, TaskId.EscWrite1, 'EscWrite1')
    _plot_task_sample_rate(axs[1, 0], data.Task.Execute, TaskId.EscWrite2, 'EscWrite2')
    _plot_task_sample_rate(axs[1, 0], data.Task.Execute, TaskId.EscWrite3, 'EscWrite3')

    _plot_task_sample_rate(axs[1, 1], data.Task.Execute, TaskId.RcReceiver, 'RcReceiver')

    _plot_task_sample_rate(axs[2, 0], data.Task.Execute, TaskId.StateEstAndCtrl, 'StateEstAndCtrl')

    _plot_task_sample_rate(axs[2, 1], data.Task.Execute, TaskId.DataLogger, 'DataLogger')

    _finish_subplots(fig)


def main():
    parser = argparse.ArgumentParser(description='Analyze data log file.')
    parser.add_argument('path', type=Path, help='Path to data log file')
    args = parser.parse_args()

    data = read_data_log(args.path)

    _plot_imu(data)
    _plot_esc(data)
    _plot_rc(data)
    _plot_state_est(data)
    _plot_state_ctrl(data)
    _plot_tasks(data)

    plt.show()


if __name__ == "__main__":
    main()
