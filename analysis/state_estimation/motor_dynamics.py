import argparse
from pathlib import Path
from typing import List, Tuple

import matplotlib.pyplot as plt
import numpy as np

from read_data_log import read_data_log, Signals

RC_SAMPLE_RATE_S = 0.02  # 50 Hz

MOTOR_CMD_TO_ANG_RATE_SCALE = 1 / 57  # From the thrust estimation.
MOTOR_CMD_TO_ANG_RATE_OFFSET = 9675 / 57  # From the thrust estimation.

RPM_TO_RAD = 2 * np.pi / 60


def find_step_indices(t_s: List, x: List,
                      min_step_size: float,
                      min_step_length_s: float) -> List[Tuple[int, int]]:
    indices = []
    start = None

    for i in range(1, len(x)):

        if abs(x[i] - x[i - 1]) > 0:
            possible_end = i - 1
        elif i == (len(x) - 1):
            possible_end = i
        else:
            possible_end = None

        if start is not None and possible_end is not None:
            if t_s[possible_end] - t_s[start] >= min_step_length_s:
                indices.append((start, possible_end))
            start = None

        if abs(x[i] - x[i - 1]) >= min_step_size:
            start = i - 1

    return indices


def time_const_from_steps(t: List, u: List, y: List,
                          min_step_size=100,
                          min_step_length_s=1,
                          yr_lim_upper=0.9) -> (float, float):
    """
    Estimates the positive and negative time constants of
    the systems Y(s) = G(s) * U(s) step response(s). Where
    G(s) = 1 / (tau * s + 1).

    Use yr_lim_upper to set a relative upper limit for the
    inclusion of data in the MLSE calculation e.g.,
    0.9 => y reached 90 % of step the response.
    """
    a_pos, b_pos, a_neg, b_neg = [], [], [], []

    for start, end in find_step_indices(t, u, min_step_size, min_step_length_s):

        yr = (np.array(y[start:end]) - u[start]) / (u[end] - u[start])
        end_lim = start + np.argwhere(yr < yr_lim_upper)[-1][0]

        a_step = np.array(t[start:end_lim]) - t[start]
        b_step = np.array(np.log((u[end_lim] - u[start]) / (u[end_lim] - y[start:end_lim])))

        if (u[end] - u[start]) > 0:
            a_pos += a_step.tolist()
            b_pos += b_step.tolist()
        else:
            a_neg += a_step.tolist()
            b_neg += b_step.tolist()

    a_pos = np.array([a_pos]).T
    b_pos = np.array([b_pos]).T
    a_neg = np.array([a_neg]).T
    b_neg = np.array([b_neg]).T

    use_pos = np.isfinite(a_pos) & np.isfinite(b_pos)
    use_neg = np.isfinite(a_neg) & np.isfinite(b_neg)

    if len(a_pos) > 0:
        mlse_pos = np.linalg.lstsq(a_pos[use_pos[:, 0], :], b_pos[use_pos[:, 0], :])
        pos_tau = 1 / float(mlse_pos[0])
    else:
        pos_tau = None

    if len(a_neg) > 0:
        mlse_neg = np.linalg.lstsq(a_neg[use_neg[:, 0], :], b_neg[use_neg[:, 0], :])
        neg_tau = 1 / float(mlse_neg[0])
    else:
        neg_tau = None

    return pos_tau, neg_tau


def _motor_cmd_to_ang_rate(motor_cmd):
    ang_rate = []

    for cmd in motor_cmd:
        if cmd == 0:
            ang_rate.append(0)
        else:
            ang_rate.append(MOTOR_CMD_TO_ANG_RATE_SCALE * cmd + MOTOR_CMD_TO_ANG_RATE_OFFSET)

    return ang_rate


def _plot_motor_dynamics(data: Signals, motor_i: int):
    """ Compute time constant """
    ang_rate = getattr(data.Esc, 'AngularRate' + str(motor_i))
    motor_cmd = getattr(data.Esc, 'MotorCmd' + str(motor_i))

    t_s = ang_rate.t_s
    meas_ang_rate = ang_rate.val
    req_ang_rate = _motor_cmd_to_ang_rate(motor_cmd.val)

    pos_tau, neg_tau = time_const_from_steps(t_s, req_ang_rate, meas_ang_rate,
                                             min_step_size=1e3,
                                             min_step_length_s=3,
                                             )

    """ Plot """
    plt.plot(t_s, meas_ang_rate, label=r'$\omega_{measured}$')
    plt.plot(t_s, req_ang_rate, label=r'$\omega_{requested}$')

    plt.title(r'Step response motor {}: $\tau_+ = {}$, $\tau_- = {}$'.format(motor_i, pos_tau, neg_tau))
    plt.xlabel('Time [s]')
    plt.ylabel('Angular-rate [rpm]')
    plt.legend(loc='upper right')
    plt.grid()


def main():
    parser = argparse.ArgumentParser(description='Plot motor step response and estimate its time constant.')
    parser.add_argument('path', type=Path, help='Path to data log file')
    parser.add_argument('--motor_i', type=int, help='Select which motor (0,1,2,3)', default=0)
    args = parser.parse_args()

    data = read_data_log(args.path, resample_to_fixed_rate_s=0.02)
    _plot_motor_dynamics(data, args.motor_i)
    plt.show()


if __name__ == "__main__":
    main()
