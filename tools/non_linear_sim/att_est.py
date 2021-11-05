# TODO: This is basically a copy of the target implementation (attitude_estimation.cpp). In would be beneficial
#  if the target implementation could be used instead (DRY). Also, note that another python implementation exists 
#  under ./state_est/attitude_estimators (AttEstKalman).
from dataclasses import dataclass

import numpy as np


@dataclass
class ImuOut:
    acc_x: float  # [m/s^2]
    acc_y: float  # [m/s^2]
    acc_z: float  # [m/s^2]

    ang_rate_x: float  # [rad/s]
    ang_rate_y: float  # [rad/s]
    ang_rate_z: float  # [rad/s]

    mag_field_x: float  # [gauss]
    mag_field_y: float  # [gauss]
    mag_field_z: float  # [gauss]


@dataclass
class State:
    angle: float = 0.0  # [rad]
    rate: float = 0.0  # [rad/s]
    acc: float = 0.0  # [rad/s^2]


@dataclass
class Estimate:
    roll: State = State()
    pitch: State = State()
    yaw: State = State()


@dataclass
class KalmanState:
    x: np.ndarray = np.zeros(3)
    z: np.ndarray = np.zeros(2)
    P: np.ndarray = np.zeros((3, 3))
    R: np.ndarray = np.zeros((2, 2))


class AttEst:
    def __init__(self):
        self._est = Estimate()

        self._kalman_roll = KalmanState()
        self._kalman_pitch = KalmanState()
        self._kalman_yaw = KalmanState()

    def update(self, imu_out: ImuOut):
        # TODO: _update_kalman_state
        # TODO: _modulo_angle
        # TODO
        pass

    def reset(self):
        self._est = State()

        self._kalman_roll = KalmanState()
        self._kalman_pitch = KalmanState()
        self._kalman_yaw = KalmanState()

    def get_estimate(self):
        return self._est
