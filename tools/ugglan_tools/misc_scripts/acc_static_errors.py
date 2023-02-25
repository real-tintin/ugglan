import argparse
from dataclasses import dataclass
from enum import Enum
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

import ugglan_tools.data_log.io as data_log_io

IMU_SAMPLE_RATE_S = 0.01  # 100 Hz

GRAVITY_EARTH = 9.82  # Sweden, m/s^2

G_IN_POS_X_FILE_NAME = "g_in_pos_x.dat"
G_IN_NEG_X_FILE_NAME = "g_in_neg_x.dat"
G_IN_POS_Y_FILE_NAME = "g_in_pos_y.dat"
G_IN_NEG_Y_FILE_NAME = "g_in_neg_y.dat"
G_IN_POS_Z_FILE_NAME = "g_in_pos_z.dat"
G_IN_NEG_Z_FILE_NAME = "g_in_neg_z.dat"

EXPECTED_FILE_NAMES = [
    G_IN_POS_X_FILE_NAME,
    G_IN_NEG_X_FILE_NAME,
    G_IN_POS_Y_FILE_NAME,
    G_IN_NEG_Y_FILE_NAME,
    G_IN_POS_Z_FILE_NAME,
    G_IN_NEG_Z_FILE_NAME
]


class Axis(Enum):
    x = 0,
    y = 1,
    z = 2


@dataclass
class RegData:
    X: np.ndarray
    Y: np.ndarray


@dataclass
class Error:
    S: np.ndarray
    M: np.ndarray
    b: np.ndarray
    residuals: np.ndarray
    norm_residual: float


def _append_regression_data(data, path: Path, Y_x: float = 0.0, Y_y: float = 0.0, Y_z: float = 0.0) -> None:
    signals = data_log_io.read(path=path, resample_to_fixed_rate_s=IMU_SAMPLE_RATE_S)

    acc_x = np.array(signals.Imu.AccelerationX.val)
    acc_y = np.array(signals.Imu.AccelerationY.val)
    acc_z = np.array(signals.Imu.AccelerationZ.val)

    n_samples = len(acc_x)

    X = np.zeros((n_samples * 3, 12))
    Y = np.zeros((n_samples * 3, 1))

    for i, axis in enumerate(list(Axis)):
        weight_x, weight_y, weight_z = 0.0, 0.0, 0.0

        if axis == Axis.x:
            weight_x = 1.0
            Y_i = Y_x
        elif axis == Axis.y:
            weight_y = 1.0
            Y_i = Y_y
        elif axis == Axis.z:
            weight_z = 1.0
            Y_i = Y_z
        else:
            raise NotImplementedError

        X[(n_samples * i):(n_samples * (i + 1)), :] = np.array([weight_x * acc_x,
                                                                weight_y * acc_y,
                                                                weight_z * acc_z,
                                                                weight_x * acc_y,
                                                                weight_x * acc_z,
                                                                weight_y * acc_x,
                                                                weight_y * acc_z,
                                                                weight_z * acc_x,
                                                                weight_z * acc_y,
                                                                weight_x * np.ones(n_samples),
                                                                weight_y * np.ones(n_samples),
                                                                weight_z * np.ones(n_samples)]).transpose()
        Y[(n_samples * i):(n_samples * (i + 1)), :] = Y_i

    if len(data.X) > 0:
        data.X = np.vstack((data.X, X))
        data.Y = np.vstack((data.Y, Y))
    else:
        data.X = X
        data.Y = Y


def _load_regression_data(root: Path) -> RegData:
    data = RegData(X=np.array([]), Y=np.array([]))

    _append_regression_data(data, path=root / Path(G_IN_POS_X_FILE_NAME), Y_x=-GRAVITY_EARTH)
    _append_regression_data(data, path=root / Path(G_IN_NEG_X_FILE_NAME), Y_x=GRAVITY_EARTH)
    _append_regression_data(data, path=root / Path(G_IN_POS_Y_FILE_NAME), Y_y=-GRAVITY_EARTH)
    _append_regression_data(data, path=root / Path(G_IN_NEG_Y_FILE_NAME), Y_y=GRAVITY_EARTH)
    _append_regression_data(data, path=root / Path(G_IN_POS_Z_FILE_NAME), Y_z=-GRAVITY_EARTH)
    _append_regression_data(data, path=root / Path(G_IN_NEG_Z_FILE_NAME), Y_z=GRAVITY_EARTH)

    return data


def _estimate_error_using_regression(data: RegData) -> Error:
    exp_rank = data.X.shape[1]
    A, residual, act_rank, _ = np.linalg.lstsq(data.X, data.Y, rcond=None)

    assert act_rank == exp_rank, "Not full rank, better conditioning needed"

    return Error(
        S=np.diag(A[0:3, 0]),
        M=np.array([[1, A[3, 0], A[4, 0]],
                    [A[5, 0], 1, A[6, 0]],
                    [A[7, 0], A[8, 0], 1]]),
        b=A[9:12],
        residuals=data.X @ A - data.Y,
        norm_residual=residual[0] / len(data.Y)
    )


def _print_error(error: Error) -> None:
    print(f"S = {error.S}")
    print(f"M = {error.M}")
    print(f"b = {error.b}")
    print(f"norm_residual = {error.norm_residual}")


def _plot_residuals(data: RegData, error: Error) -> None:
    plt.scatter(np.arange(0, len(error.residuals)), error.residuals, marker='.', label='Xb-Y')

    plt.title("MLSE residuals of accelerometer statical error estimation")
    plt.ylabel('residual [m/s^2]')
    plt.xlabel('sample [-]')
    plt.grid()
    plt.legend()

    plt.show()


def main():
    parser = argparse.ArgumentParser(description='Estimates the accelerometer static errors.')
    parser.add_argument('path', type=Path, help=f'Path to folder with data log files: {", ".join(EXPECTED_FILE_NAMES)}')
    args = parser.parse_args()

    data = _load_regression_data(root=args.path)
    error = _estimate_error_using_regression(data=data)

    _print_error(error)
    _plot_residuals(data, error)


if __name__ == "__main__":
    main()
