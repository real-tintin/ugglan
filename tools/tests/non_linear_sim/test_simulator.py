import numpy as np
import pytest

from non_linear_sim.att_est import AttEst
from non_linear_sim.drone_model import DEFAULT_ENV_PARAMS, DEFAULT_DRONE_PARAMS
from non_linear_sim.pilot_ctrl import PilotCtrl
from non_linear_sim.simulator import Simulator
from non_linear_sim.six_dof_model import STATE_ZERO

TEST_DT = 0.1


@pytest.fixture
def test_simulator():
    yield Simulator(
        att_est=AttEst(),
        pilot_ctrl=PilotCtrl(L_phi=np.ones(4), L_theta=np.ones(4), L_psi=np.ones(43), dt=TEST_DT),
        six_dof_state=STATE_ZERO,
        drone_params=DEFAULT_DRONE_PARAMS,
        env_params=DEFAULT_ENV_PARAMS,
        dt=TEST_DT,
    )


class TestSimulator:

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
    def test_euler_to_imu_mag_for_yaw_est(test_simulator, phi, theta, psi):
        mag = test_simulator._euler_to_imu_mag_for_yaw_est(phi, theta, psi)

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
