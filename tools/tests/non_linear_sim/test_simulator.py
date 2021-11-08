import numpy as np
import pytest

from non_linear_sim.att_estimator import AttEstimator, DEFAULT_ATT_EST_PARAMS
from non_linear_sim.drone_model import DEFAULT_ENV_PARAMS, DEFAULT_DRONE_PARAMS
from non_linear_sim.pilot_ctrl import PilotCtrl, DEFAULT_PILOT_CTRL_PARAMS, RefInput
from non_linear_sim.simulator import Simulator
from non_linear_sim.six_dof_model import STATE_ZERO

TEST_DT = 0.02
MG = DEFAULT_DRONE_PARAMS.m * DEFAULT_ENV_PARAMS.g


@pytest.fixture
def default_sim():
    yield Simulator(
        att_estimator=AttEstimator(params=DEFAULT_ATT_EST_PARAMS, dt=TEST_DT),
        pilot_ctrl=PilotCtrl(params=DEFAULT_PILOT_CTRL_PARAMS, dt=TEST_DT),
        six_dof_state=STATE_ZERO,
        drone_params=DEFAULT_DRONE_PARAMS,
        env_params=DEFAULT_ENV_PARAMS,
        dt=TEST_DT,
    )


class TestSimulator:

    @staticmethod
    def test_step_response(default_sim):
        # TODO: Different states and refs.
        import matplotlib.pyplot as plt

        roll_ref = np.pi / 8
        n_samples_ten_s = int(1 / TEST_DT * 10)

        roll = np.zeros(n_samples_ten_s)

        for i in range(n_samples_ten_s):
            default_sim.step(ref_input=RefInput(f_z=-MG, roll=roll_ref, pitch=0, yaw_rate=0))
            roll[i] = default_sim.get_6dof_state().n_i[0]

        plt.plot(roll)
        plt.show()

        act_roll = default_sim.get_6dof_state().n_i[0]
        assert abs(act_roll - roll_ref) < 1e-3

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
