import argparse
from pathlib import Path
from typing import List

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from cycler import cycler
from matplotlib.patches import Rectangle

import data_log.io as data_log_io
from data_log.io import Signals

MIN_ANG_RATE = 0.1  # [rad/s]
MAX_ABS_ACC_RATE = 0.05  # [rad/s^2]
MIN_STEP_LENGTH = 2  # [s]

RESAMPLE_RATE_S = 0.01  # 100 Hz

N_ESC = 4

mpl.rcParams['axes.prop_cycle'] = cycler(color='bgrcmyk')
mpl.rcParams['lines.linewidth'] = 0.5

ChunkedIndices = List[List]


def _split_in_chunks(indices: np.ndarray):
    return np.split(indices, np.where(np.diff(indices) != 1)[0] + 1)


def _find_zero_dx_index_chunks(x: np.ndarray,
                               min_x: float,
                               max_abs_dx: float,
                               min_size: float) -> List[List]:
    chunks = []
    dx = np.diff(x, prepend=0)

    possible_indices = np.argwhere((x >= min_x) & (np.abs(dx) <= max_abs_dx)).flatten()
    possible_chunks = _split_in_chunks(possible_indices)

    for chunk in possible_chunks:
        if len(chunk) >= min_size:
            chunks.append(chunk)

    return chunks


def _merge_chunks(old_chunks: ChunkedIndices, new_chunks: ChunkedIndices) -> ChunkedIndices:
    merged_chunks = []

    if len(old_chunks) != len(new_chunks):
        raise ValueError('Chunks need to have same length during merge.')

    for old_chunk, new_chunk in zip(old_chunks, new_chunks):
        start = min(old_chunk[0], new_chunk[0])
        end = max(old_chunk[-1], new_chunk[-1])

        merged_chunks.append(list(range(start, end + 1)))

    return merged_chunks


def _get_chunked_motor_ang_rate_indices(data: Signals) -> (ChunkedIndices, np.ndarray):
    chunks = []
    is_motor_active = [False, False, False, False]

    for i_motor in range(N_ESC):
        ang_rate_val = np.array(getattr(data.Esc, 'AngularRate' + str(i_motor)).val)

        if np.any(ang_rate_val):
            is_motor_active[i_motor] = True
            new_chunks = _find_zero_dx_index_chunks(x=ang_rate_val,
                                                    min_x=MIN_ANG_RATE,
                                                    max_abs_dx=MAX_ABS_ACC_RATE,
                                                    min_size=int(MIN_STEP_LENGTH / RESAMPLE_RATE_S))

            if chunks:
                chunks = _merge_chunks(chunks, new_chunks)
            else:
                chunks = new_chunks

    if not chunks:
        raise RuntimeError('No chunks found.')

    return chunks, is_motor_active


def _compute_var_in_chunks(x: np.ndarray, chunks: ChunkedIndices) -> np.ndarray:
    var = np.zeros(len(chunks))

    for i_chunk, chunk in enumerate(chunks):
        var[i_chunk] = np.var(x[chunk])

    return var


def _plot_vibrations(data: Signals):
    fig, axs = plt.subplots(2, 1)
    fig.suptitle('Motor induces vibration analysis')

    ax_top_left = axs[0]
    ax_top_right = axs[0].twinx()
    ax_bottom = axs[1]

    chunks, is_motor_active = _get_chunked_motor_ang_rate_indices(data)

    def plot_imu_acc(ax):
        ax.plot(data.Imu.AccelerationX.t_s, data.Imu.AccelerationX.val, label=r'$a_x$')
        ax.plot(data.Imu.AccelerationY.t_s, data.Imu.AccelerationY.val, label=r'$a_y$')
        ax.plot(data.Imu.AccelerationZ.t_s, data.Imu.AccelerationZ.val, label=r'$a_z$')

        ax.set(ylabel='Acceleration [m/s^2]', xlabel='Time [s]')
        ax.grid()
        ax.legend(loc='upper left', fontsize=7)

    def add_chunks_to_ax(ax, x):
        add_label = True
        ylim = ax.get_ylim()

        for chunk in chunks:
            label = 'found chunks' if add_label else None
            add_label = False

            ax.add_patch(Rectangle((x[chunk[0]], ylim[0]), x[chunk[-1]] - x[chunk[0]], ylim[1],
                                   facecolor='blue', alpha=0.1, edgecolor=None, label=label))

    def plot_motor_ang_rates(ax):
        for i_motor in range(N_ESC):
            ang_rate = getattr(data.Esc, 'AngularRate' + str(i_motor))

            active_str = 'active' if is_motor_active[i_motor] else 'inactive'
            label = r'motor$_{}$ ({})'.format(i_motor, active_str)

            ax.plot(ang_rate.t_s, ang_rate.val, label=label, linestyle='--')

        add_chunks_to_ax(ax, x=ang_rate.t_s)

        ax.set(ylabel='Angular-rate [rad/s]')
        ax.legend(loc='upper right', fontsize=7)

    def plot_vibrations_bar(ax):
        width = 0.2
        x_bar = np.arange(0, len(chunks))

        var_acc_x = _compute_var_in_chunks(np.array(data.Imu.AccelerationX.val), chunks)
        var_acc_y = _compute_var_in_chunks(np.array(data.Imu.AccelerationY.val), chunks)
        var_acc_z = _compute_var_in_chunks(np.array(data.Imu.AccelerationZ.val), chunks)
        var_acc_norm = np.linalg.norm((var_acc_x, var_acc_y, var_acc_z), axis=0)

        ax.bar(x_bar - width * 1.5, var_acc_x, width, label=r'$\sigma^2_{a_x}$')
        ax.bar(x_bar - width * 0.5, var_acc_y, width, label=r'$\sigma^2_{a_y}$')
        ax.bar(x_bar + width * 0.5, var_acc_z, width, label=r'$\sigma^2_{a_z}$')
        ax.bar(x_bar + width * 1.5, var_acc_norm, width, label=r'$\sigma^2_{|a|}$')

        ax.set(xlabel='Chunk n []', ylabel='Var[acceleration] [m/s^2]')
        ax.set_xticks(x_bar)

        ax.legend(loc='upper right', fontsize=7)
        ax.grid()

    plot_imu_acc(ax_top_left)
    plot_motor_ang_rates(ax_top_right)
    plot_vibrations_bar(ax_bottom)


def main():
    parser = argparse.ArgumentParser(description='Plot motor induced vibrations using the accelerometer. '
                                                 'Note, assumes data with step response of motor(s).')
    parser.add_argument('path', type=Path, help='Path to data log file')
    args = parser.parse_args()

    data = data_log_io.read(args.path, resample_to_fixed_rate_s=RESAMPLE_RATE_S)
    _plot_vibrations(data)
    plt.show()


if __name__ == "__main__":
    main()
