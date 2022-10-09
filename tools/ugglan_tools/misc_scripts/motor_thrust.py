import argparse
from dataclasses import dataclass, field
from enum import Enum
from typing import List

import matplotlib.pyplot as plt
import numpy as np
from sympy import Matrix, symbols, pprint

FRAME_ARM_LENGTH = 0.23  # [m]
THRUST_2_TORQUE = 1 / 50  # Based on empirical relationship from master thesis.

GRAVITY = 9.82
RPM_2_RAD = 2 * np.pi / 60

UNITS = {
    'command': '-', 'ang_rate': 'rad/s', 'thrust': 'N',
    'command_sq': '-', 'ang_rate_sq': 'rad^2/s', 'thrust_sq': 'N^2'
}

MATH_SYMBOLS = {
    'command': 'u', 'ang_rate': '\\omega', 'thrust': 'F',
    'command_sq': 'u^2', 'ang_rate_sq': '\\omega^2', 'thrust_sq': 'F^2'
}

LABEL_STRING = '${}$ [{}]'


class Figure(Enum):
    THRUST_COMMAND = 'thrust_command'
    ANG_RATE_THRUST = 'ang_rate_thrust'
    ANG_RATE_COMMAND = 'ang_rate_command'

    def __str__(self):
        return self.value


@dataclass
class Measurements:
    command: List[int] = field(default_factory=list)
    command_sq: List[int] = field(default_factory=list)

    ang_rate: List[int] = field(default_factory=list)
    ang_rate_sq: List[int] = field(default_factory=list)

    thrust: List[int] = field(default_factory=list)
    thrust_sq: List[int] = field(default_factory=list)


# command, rpm, thrust_g (z pointing down)
raw_pos_meas = [
    [0, 0, 0],
    [1, 1272, -13],
    [10, 1277, -13],
    [100, 1296, -13],
    [1000, 1576, -19],
    [2000, 1847, -27],
    [4000, 2334, -44],
    [8000, 3099, -84],
    [12000, 3712, -120],
    [16000, 4209, -153],
    [20000, 4920, -223],
    [24000, 5616, -289],
    [28000, 6312, -383],
    [32000, 7019, -467]
]

raw_neg_meas = [
    [0, 0, 0],
    [-1, -1268, 10],
    [-10, -1260, 9],
    [-100, -1290, 10],
    [-1000, -1556, 15],
    [-2000, -1833, 20],
    [-4000, -2323, 28],
    [-8000, -3054, 48],
    [-12000, -3666, 66],
    [-16000, -4178, 87],
    [-20000, -4866, 119],
    [-24000, -5479, 159],
    [-28000, -6140, 211],
    [-32000, -6835, 253],
]


def _parse_raw_measurements(raw_measurements) -> Measurements:
    measurements = Measurements()

    for raw_m in raw_measurements:
        command = raw_m[0]
        ang_rate = raw_m[1] * RPM_2_RAD
        thrust = raw_m[2] * GRAVITY * 0.001

        measurements.command.append(command)
        measurements.command_sq.append(command ** 2)

        measurements.ang_rate.append(ang_rate)
        measurements.ang_rate_sq.append(ang_rate ** 2)

        measurements.thrust.append(thrust)
        measurements.thrust_sq.append(thrust ** 2)

    return measurements


def _plot_with_polyfit(x, y, used_coefs, skip_first_n, label, color):
    n_eval = 100
    deg = len(used_coefs) - 1
    n_coefs = np.sum(used_coefs)

    xn = x[skip_first_n:]
    yn = y[skip_first_n:]
    Xn = np.zeros((len(xn), n_coefs))

    xp = np.linspace(min(xn), max(xn), n_eval)
    Xp = np.zeros((n_eval, n_coefs))

    for i, used_coef in enumerate(used_coefs):
        if used_coef:
            Xn[:, i] = np.array(xn) ** (deg - i)
            Xp[:, i] = xp ** (deg - i)

    coefs, _, _, _ = np.linalg.lstsq(Xn, yn, rcond=None)
    yp = Xp @ coefs

    coef_str = np.array2string(coefs, precision=2)

    plt.plot(x, y, 'o', label=label + ' raw', color=color)
    plt.plot(xp, yp, '-', label=label + ' polyfit: ' + coef_str, color=color)


def _plot_correlation(pos_meas, neg_meas, label_x, label_y, used_coefs, skip_first_n=0):
    x_pos = getattr(pos_meas, label_x)
    y_pos = getattr(pos_meas, label_y)

    x_neg = getattr(neg_meas, label_x)
    y_neg = getattr(neg_meas, label_y)

    plt.figure()

    _plot_with_polyfit(x_pos, y_pos, used_coefs, skip_first_n, 'Positive rotation', 'C0')
    _plot_with_polyfit(x_neg, y_neg, used_coefs, skip_first_n, 'Negative rotation', 'C1')

    plt.xlabel(LABEL_STRING.format(MATH_SYMBOLS[label_x], UNITS[label_x]))
    plt.ylabel(LABEL_STRING.format(MATH_SYMBOLS[label_y], UNITS[label_y]))
    plt.grid()
    plt.legend()


def _print_h_inverse():
    cf, cm, lx = symbols('cf cm, lx')
    H = Matrix([[-cf, -cf, -cf, -cf],
                [-lx * cf, -lx * cf, lx * cf, lx * cf],
                [lx * cf, -lx * cf, -lx * cf, lx * cf],
                [-cm, cm, -cm, cm]])

    print("H^-1 = ")
    pprint(H.inv())


def main():
    parser = argparse.ArgumentParser(description='Plot motor thrust curves.')
    parser.add_argument('--figure', type=Figure, choices=list(Figure), nargs='+', default=list(Figure),
                        help='Selected figure(s)')

    args = parser.parse_args()

    pos_meas = _parse_raw_measurements(raw_pos_meas)
    neg_meas = _parse_raw_measurements(raw_neg_meas)

    _print_h_inverse()

    if Figure.THRUST_COMMAND in args.figure:
        _plot_correlation(pos_meas, neg_meas, 'thrust', 'command', used_coefs=[1, 1, 1])
        _plot_correlation(pos_meas, neg_meas, 'thrust', 'command_sq', used_coefs=[1, 1])

    if Figure.ANG_RATE_THRUST in args.figure:
        _plot_correlation(pos_meas, neg_meas, 'ang_rate', 'thrust', used_coefs=[1, 1, 1])
        _plot_correlation(pos_meas, neg_meas, 'ang_rate_sq', 'thrust', used_coefs=[1, 0])

    if Figure.ANG_RATE_COMMAND in args.figure:
        _plot_correlation(pos_meas, neg_meas, 'ang_rate', 'command', used_coefs=[1, 1], skip_first_n=5)
        _plot_correlation(pos_meas, neg_meas, 'ang_rate_sq', 'command', used_coefs=[1, 1, 1], skip_first_n=5)

    plt.show()


if __name__ == "__main__":
    main()
