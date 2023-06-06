import argparse
import json
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import List, Tuple

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
    ang_rate: List[int] = field(default_factory=list)
    thrust: List[int] = field(default_factory=list)

    def append(self, command: int, ang_rate: int, thrust: int):
        self.command.append(command)
        self.ang_rate.append(ang_rate)
        self.thrust.append(thrust)

    @property
    def command_sq(self):
        return [x ** 2 for x in self.command]

    @property
    def ang_rate_sq(self):
        return [x ** 2 for x in self.ang_rate]

    @property
    def thrust_sq(self):
        return [x ** 2 for x in self.thrust]


def _parse_raw_measurements(path: Path) -> Tuple[Measurements, Measurements, str]:
    positive = Measurements()
    negative = Measurements()

    with open(file=path, mode="r") as file:
        data = json.load(file)

        name = data["name"]

        for measurement in data["measurements"]:
            command = measurement["cmd"]
            ang_rate = measurement["rpm"] * RPM_2_RAD
            thrust = measurement["thrust_g"] * GRAVITY * 0.001

            if command == 0:
                positive.append(command=command, ang_rate=ang_rate, thrust=thrust)
                negative.append(command=command, ang_rate=ang_rate, thrust=thrust)
            elif command > 0:
                positive.append(command=command, ang_rate=ang_rate, thrust=thrust)
            elif command < 0:
                negative.append(command=command, ang_rate=ang_rate, thrust=thrust)
            else:
                RuntimeError("Unknown command")

    return positive, negative, name


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


def _plot_correlation(pos_meas, neg_meas, title, attr_x, attr_y, used_coefs, skip_first_n=0):
    x_pos = getattr(pos_meas, attr_x)
    y_pos = getattr(pos_meas, attr_y)

    x_neg = getattr(neg_meas, attr_x)
    y_neg = getattr(neg_meas, attr_y)

    plt.figure()

    _plot_with_polyfit(x_pos, y_pos, used_coefs, skip_first_n, 'Positive rotation', 'C0')
    _plot_with_polyfit(x_neg, y_neg, used_coefs, skip_first_n, 'Negative rotation', 'C1')

    plt.title(title)
    plt.xlabel(LABEL_STRING.format(MATH_SYMBOLS[attr_x], UNITS[attr_x]))
    plt.ylabel(LABEL_STRING.format(MATH_SYMBOLS[attr_y], UNITS[attr_y]))
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
    parser.add_argument('path', type=Path, help='Path to raw measurements json file')
    parser.add_argument('--figure', type=Figure, choices=list(Figure), nargs='+', default=list(Figure),
                        help='Selected figure(s)')

    args = parser.parse_args()

    pos_meas, neg_meas, name = _parse_raw_measurements(args.path)

    _print_h_inverse()

    if Figure.THRUST_COMMAND in args.figure:
        _plot_correlation(pos_meas, neg_meas, title=name, attr_x='thrust', attr_y='command', used_coefs=[1, 1, 1])
        _plot_correlation(pos_meas, neg_meas, title=name, attr_x='thrust', attr_y='command_sq', used_coefs=[1, 1])

    if Figure.ANG_RATE_THRUST in args.figure:
        _plot_correlation(pos_meas, neg_meas, title=name, attr_x='ang_rate', attr_y='thrust', used_coefs=[1, 1, 1])
        _plot_correlation(pos_meas, neg_meas, title=name, attr_x='ang_rate_sq', attr_y='thrust', used_coefs=[1, 0])

    if Figure.ANG_RATE_COMMAND in args.figure:
        _plot_correlation(pos_meas, neg_meas, title=name, attr_x='ang_rate', attr_y='command', used_coefs=[1, 1],
                          skip_first_n=5)
        _plot_correlation(pos_meas, neg_meas, title=name, attr_x='ang_rate_sq', attr_y='command', used_coefs=[1, 1, 1],
                          skip_first_n=5)

    plt.show()


if __name__ == "__main__":
    main()
