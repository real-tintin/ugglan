from typing import List

import matplotlib.pyplot as plt
import numpy as np
from dataclasses import dataclass, field

FRAME_ARM_LENGTH = 0.23  # [m]
THRUST_2_TORQUE = 1 / 50  # Based on empirical relationship from master thesis.

GRAVITY = 9.82
RPM_2_RAD = 2 * np.pi / 60

UNITS = {
    'command': '-', 'ang_rate': 'rad/s', 'thrust': 'N',
    'command_sq': '-', 'ang_rate_sq': 'rad^2/s', 'thrust_sq': 'N^2'
}

MATH_SYMBOLS = {
    'command': 'u', 'ang_rate': '\\omega', 'thrust': 'f',
    'command_sq': 'u^2', 'ang_rate_sq': '\\omega^2', 'thrust_sq': 'f^2'
}

LABEL_STRING = '${}$ [{}]'


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
    [0, 0, -0],
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


def _plot_with_polyfit(x, y, deg, label, color):
    p = np.polyfit(x, y, deg)
    xp = np.linspace(min(x), max(x), 100)
    yp = np.polyval(p, xp)

    poly_coef_str = np.array2string(p, precision=2)

    plt.plot(x, y, 'o', label=label + ' raw', color=color)
    plt.plot(xp, yp, '-', label=label + ' polyfit: ' + poly_coef_str, color=color)


def _plot_correlation(pos_meas, neg_meas, label_x, label_y, deg, square_x=False):
    x_pos = getattr(pos_meas, label_x)
    y_pos = getattr(pos_meas, label_y)

    x_neg = getattr(neg_meas, label_x)
    y_neg = getattr(neg_meas, label_y)

    if square_x:
        x_pos = np.array(x_pos) ** 2
        x_neg = np.array(x_neg) ** 2

    plt.figure()

    _plot_with_polyfit(x_pos, y_pos, deg, label='Positive rotation', color='C0')
    _plot_with_polyfit(x_neg, y_neg, deg, label='Negative rotation', color='C1')

    plt.xlabel(LABEL_STRING.format(MATH_SYMBOLS[label_x], UNITS[label_x]))
    plt.ylabel(LABEL_STRING.format(MATH_SYMBOLS[label_y], UNITS[label_y]))
    plt.grid()
    plt.legend()


def main():
    pos_meas = _parse_raw_measurements(raw_pos_meas)
    neg_meas = _parse_raw_measurements(raw_neg_meas)

    _plot_correlation(pos_meas, neg_meas, 'thrust', 'command', deg=2)
    _plot_correlation(pos_meas, neg_meas, 'thrust', 'command_sq', deg=1)

    _plot_correlation(pos_meas, neg_meas, 'ang_rate', 'thrust', deg=2)
    _plot_correlation(pos_meas, neg_meas, 'ang_rate_sq', 'thrust', deg=1)

    _plot_correlation(pos_meas, neg_meas, 'command', 'ang_rate', deg=1)

    plt.show()


if __name__ == "__main__":
    main()
