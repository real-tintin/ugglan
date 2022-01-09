import argparse
from enum import Enum
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from cycler import cycler

import data_log.io as data_log_io
from .task_ids import TaskId

mpl.rcParams['axes.prop_cycle'] = cycler(color='bgrcmyk')
mpl.rcParams['lines.linewidth'] = 0.5

N_ESC = 4


class Figure(Enum):
    IMU = 'imu'
    ESC = 'esc'
    RC = 'rc'
    STATE_EST = 'state_est'
    STATE_CTRL = 'state_ctrl'
    TASKS = 'tasks'

    def __str__(self):
        return self.value


def _finish_subplots(fig):
    for ax in fig.get_axes():
        ax.xaxis.set_tick_params(labelbottom=True)
        ax.set(xlabel='Time [s]')
        ax.grid()

        if all(ax.get_legend_handles_labels()):
            ax.legend(loc='upper right', fontsize=7)


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
    fig, axs = plt.subplots(3, 2, sharex=True)
    fig.suptitle('IMU')

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


def _plot_esc(data):
    fig, axs = plt.subplots(3, 2, sharex=True)
    fig.suptitle('ESC')

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


def _plot_rc(data):
    fig, axs = plt.subplots(3, 2, sharex=True)
    fig.suptitle('RC')

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


def _plot_state_est(data):
    fig, axs = plt.subplots(2, 2, sharex=True)
    fig.suptitle('State Est')

    axs[0, 0].plot(data.StateEst.Roll.t_s, data.StateEst.Roll.val, label=r'$\phi$')
    axs[0, 0].plot(data.StateEst.Pitch.t_s, data.StateEst.Pitch.val, label=r'$\theta$')
    axs[0, 0].plot(data.StateEst.Yaw.t_s, data.StateEst.Yaw.val, label=r'$\psi$')
    axs[0, 0].set(ylabel='Angle [rad]')

    axs[0, 1].plot(data.StateEst.RollRate.t_s, data.StateEst.RollRate.val, label=r'$\dot{\phi}$')
    axs[0, 1].plot(data.StateEst.PitchRate.t_s, data.StateEst.PitchRate.val, label=r'$\dot{\theta}$')
    axs[0, 1].plot(data.StateEst.YawRate.t_s, data.StateEst.YawRate.val, label=r'$\dot{\psi}$')
    axs[0, 1].set(ylabel='Angular-rate [rad/s]')

    axs[1, 0].plot(data.StateEst.RollAcc.t_s, data.StateEst.RollAcc.val, label=r'$\ddot{\phi}$')
    axs[1, 0].plot(data.StateEst.PitchAcc.t_s, data.StateEst.PitchAcc.val, label=r'$\ddot{\theta}$')
    axs[1, 0].plot(data.StateEst.YawAcc.t_s, data.StateEst.YawAcc.val, label=r'$\ddot{\psi}$')
    axs[1, 0].set(ylabel='Angular-acceleration [rad/s^2]')

    axs[1, 1].plot(data.StateEst.AttIsCalib.t_s, data.StateEst.AttIsCalib.val, label='AttIsCalibrated')
    axs[1, 1].plot(data.StateEst.AttIsStandstill.t_s, data.StateEst.AttIsStandstill.val, label='AttIsStandstill')
    axs[1, 1].set(ylabel='Flags [-]')

    _finish_subplots(fig)


def _plot_state_ctrl(data):
    _plot_state_ctrl_phi(data)
    _plot_state_ctrl_theta(data)
    _plot_state_ctrl_psi(data)
    _plot_state_ctrl_fz(data)
    _plot_state_ctrl_flags(data)


def _plot_state_ctrl_phi(data):
    fig, axs = plt.subplots(2, 2, sharex=True)
    fig.suptitle(r'State Ctrl: $\phi$')

    axs[0, 0].plot(data.StateEst.Roll.t_s, data.StateEst.Roll.val, label=r'$\phi$ [rad]')
    axs[0, 0].plot(data.StateCtrl.RollRef.t_s, data.StateCtrl.RollRef.val, label=r'$\phi_r$ [rad]')

    axs[0, 1].plot(data.StateCtrl.Phi0.t_s, data.StateCtrl.Phi0.val, label=r'$\int\tilde{\phi}$ [rads]')
    axs[0, 1].plot(data.StateCtrl.Phi1.t_s, data.StateCtrl.Phi1.val, label=r'$\tilde{\phi}$ [rad]')
    axs[0, 1].plot(data.StateCtrl.Phi2.t_s, data.StateCtrl.Phi2.val, label=r'$\tilde{\dot{\phi}}$ [rad/s]')
    axs[0, 1].plot(data.StateCtrl.Phi3.t_s, data.StateCtrl.Phi3.val, label=r'$\tilde{\ddot{\phi}}$ [rad/s^2]')

    axs[1, 0].plot(data.StateCtrl.Mx.t_s, data.StateCtrl.Mx.val, label=r'$u_{Mx}$ [Nm]')

    _finish_subplots(fig)


def _plot_state_ctrl_theta(data):
    fig, axs = plt.subplots(2, 2, sharex=True)
    fig.suptitle(r'State Ctrl: $\theta$')

    axs[0, 0].plot(data.StateEst.Pitch.t_s, data.StateEst.Pitch.val, label=r'$\theta$ [rad]')
    axs[0, 0].plot(data.StateCtrl.PitchRef.t_s, data.StateCtrl.PitchRef.val, label=r'$\theta_r$ [rad]')

    axs[0, 1].plot(data.StateCtrl.Theta0.t_s, data.StateCtrl.Theta0.val, label=r'$\int\tilde{\theta}$ [rads]')
    axs[0, 1].plot(data.StateCtrl.Theta1.t_s, data.StateCtrl.Theta1.val, label=r'$\tilde{\theta}$ [rad]')
    axs[0, 1].plot(data.StateCtrl.Theta2.t_s, data.StateCtrl.Theta2.val, label=r'$\tilde{\dot{\theta}}$ [rad/s]')
    axs[0, 1].plot(data.StateCtrl.Theta3.t_s, data.StateCtrl.Theta3.val, label=r'$\tilde{\ddot{\theta}}$ [rad/s^2]')

    axs[1, 0].plot(data.StateCtrl.My.t_s, data.StateCtrl.My.val, label=r'$u_{My}$ [Nm]')

    _finish_subplots(fig)


def _plot_state_ctrl_psi(data):
    fig, axs = plt.subplots(2, 2, sharex=True)
    fig.suptitle(r'State Ctrl: $\psi$')

    axs[0, 0].plot(data.StateEst.YawRate.t_s, data.StateEst.YawRate.val, label=r'$\dot{\psi}$ [rad/s]')
    axs[0, 0].plot(data.StateCtrl.YawRateRef.t_s, data.StateCtrl.YawRateRef.val, label=r'$\dot{\psi}_r$ [rad/s]')

    axs[0, 1].plot(data.StateCtrl.Psi0.t_s, data.StateCtrl.Psi0.val, label=r'$\int\tilde{\dot{\psi}}$ [rad]')
    axs[0, 1].plot(data.StateCtrl.Psi1.t_s, data.StateCtrl.Psi1.val, label=r'$\tilde{\dot{\psi}}$ [rad/s]')
    axs[0, 1].plot(data.StateCtrl.Psi2.t_s, data.StateCtrl.Psi2.val, label=r'$\tilde{\dot{\psi}}$ [rad/s^2]')

    axs[1, 0].plot(data.StateCtrl.Mz.t_s, data.StateCtrl.Mz.val, label=r'$u_{Mz}$ [Nm]')

    _finish_subplots(fig)


def _plot_state_ctrl_fz(data):
    fig, axs = plt.subplots(1, 1, sharex=True)
    fig.suptitle(r'State Ctrl: $F_z$')

    axs.plot(data.StateCtrl.FzRef.t_s, data.StateCtrl.FzRef.val, label=r'$F_{zr}$ [N]')
    axs.plot(data.StateCtrl.Fz.t_s, data.StateCtrl.Fz.val, label=r'$u_{Fz}$ [N]')

    _finish_subplots(fig)


def _plot_state_ctrl_flags(data):
    fig, axs = plt.subplots(1, 1, sharex=True)
    fig.suptitle('State Ctrl: Flags')

    axs.plot(data.StateCtrl.Reset.t_s, data.StateCtrl.Reset.val, label='Reset')

    _finish_subplots(fig)


def _plot_tasks(data):
    fig, axs = plt.subplots(3, 2, sharex=True)
    fig.suptitle('Task State')

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

    _plot_task_sample_rate(axs[2, 0], data.Task.Execute, TaskId.StateEst, 'StateEst')
    _plot_task_sample_rate(axs[2, 0], data.Task.Execute, TaskId.StateCtrl, 'StateCtrl')

    _plot_task_sample_rate(axs[2, 1], data.Task.Execute, TaskId.DataLogger, 'DataLogger')

    _finish_subplots(fig)


def main():
    parser = argparse.ArgumentParser(description='Plot and analyze data log file.')
    parser.add_argument('path', type=Path, help='Path to data log file')
    parser.add_argument('--figure', type=Figure, choices=list(Figure), nargs='+', default=list(Figure),
                        help='Selected figure(s)')
    args = parser.parse_args()

    data = data_log_io.read(args.path)

    if Figure.IMU in args.figure:
        _plot_imu(data)
    if Figure.ESC in args.figure:
        _plot_esc(data)
    if Figure.RC in args.figure:
        _plot_rc(data)
    if Figure.STATE_EST in args.figure:
        _plot_state_est(data)
    if Figure.STATE_CTRL in args.figure:
        _plot_state_ctrl(data)
    if Figure.TASKS in args.figure:
        _plot_tasks(data)

    plt.show()


if __name__ == "__main__":
    main()
