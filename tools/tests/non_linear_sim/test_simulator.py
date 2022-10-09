import numpy as np
import pytest

from ugglan_tools.non_linear_sim.att_estimator import DEFAULT_ATT_EST_PARAMS
from ugglan_tools.non_linear_sim.drone_model import DEFAULT_ENV_PARAMS, DEFAULT_DRONE_PARAMS
from ugglan_tools.non_linear_sim.pilot_ctrl import DEFAULT_PILOT_CTRL_PARAMS, RefInput
from ugglan_tools.non_linear_sim.simulator import Simulator, DEFAULT_IMU_NOISE

TEST_DT = 0.01
MG = DEFAULT_DRONE_PARAMS.m * DEFAULT_ENV_PARAMS.g
G = DEFAULT_ENV_PARAMS.g


@pytest.fixture
def default_sim():
    yield Simulator(
        att_est_params=DEFAULT_ATT_EST_PARAMS,
        pilot_ctrl_params=DEFAULT_PILOT_CTRL_PARAMS,
        drone_params=DEFAULT_DRONE_PARAMS,
        env_params=DEFAULT_ENV_PARAMS,
        imu_noise=DEFAULT_IMU_NOISE,
        dt=TEST_DT,
    )


class TestSimulator:

    @staticmethod
    def test_step_for_one_s(default_sim):
        n_samples_one_s = int(1 / TEST_DT)

        for i in range(n_samples_one_s):
            default_sim.step(ref_input=RefInput(f_z=-MG, roll=np.pi / 2, pitch=-np.pi / 7, yaw_rate=np.pi / 3))

        assert np.isclose(default_sim.get_t(), 1.0, 1e-9)

    @staticmethod
    @pytest.mark.parametrize("a_b, n_i, exp_imu_a", [
        (np.array([0, 0, G]), np.array([0, 0, 0]), np.array([0, 0, 0])),
        (np.array([0, 0, 0]), np.array([0, 0, 0]), np.array([0, 0, -G])),
        (np.array([0, 0, -G]), np.array([0, 0, 0]), np.array([0, 0, -2 * G])),
        (np.array([0, 0, 0]), np.array([np.pi / 2, 0, 0]), np.array([0, -G, 0])),
        (np.array([0, 0, 0]), np.array([0, np.pi / 2, 0]), np.array([G, 0, 0])),
        (np.array([0, 0, 0]), np.array([0, 0, np.pi]), np.array([0, 0, - G])),
    ])
    def test_body_acc_to_imu_acc(default_sim, a_b, n_i, exp_imu_a):
        act_imu_a = default_sim._body_acc_to_imu_acc(a_b, n_i, G)

        assert np.all(np.isclose(exp_imu_a, act_imu_a, 1e-9))

    @staticmethod
    @pytest.mark.parametrize("phi, theta, psi", [
        (0, 0, 0),
        (np.pi / 3, np.pi / 16, -np.pi / 4),
        (np.pi / 4, np.pi / 4, np.pi / 8),
        (np.pi / 4, np.pi / 9, -np.pi / 7),
        (np.pi / 4, -np.pi / 2, np.pi / 8),
        (0, 0, np.pi - 0.01),
        (np.pi / 4, 0, -np.pi / 2),
        (np.pi / 2, -np.pi / 2, -np.pi / 2),
        (np.pi / 3, -np.pi / 2, -np.pi + 0.01),
    ])
    def test_euler_to_imu_mag_for_yaw_est(default_sim, phi, theta, psi):
        mag = default_sim._euler_to_imu_mag_for_yaw_est(phi, theta, psi)

        def _mag_to_psi():
            mag_x = mag[0]
            mag_y = mag[1]
            mag_z = mag[2]

            b_x = mag_x * np.cos(theta) + \
                  mag_y * np.sin(phi) * np.sin(theta) + \
                  mag_z * np.sin(theta) * np.cos(phi)
            b_y = mag_y * np.cos(phi) - mag_z * np.sin(phi)

            return np.arctan2(-b_y, b_x)

        assert np.isclose(psi, _mag_to_psi(), 1e-9)
