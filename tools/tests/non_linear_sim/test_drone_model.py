import numpy as np
import pytest

from ugglan_tools.non_linear_sim.drone_model import DroneModel, CtrlInput, DEFAULT_ENV_PARAMS, DEFAULT_DRONE_PARAMS

TEST_DT = 0.1

MG = DEFAULT_DRONE_PARAMS.m * DEFAULT_ENV_PARAMS.g


@pytest.fixture
def drone_model():
    yield DroneModel(drone_params=DEFAULT_DRONE_PARAMS,
                     env_params=DEFAULT_ENV_PARAMS,
                     dt=TEST_DT)


class TestDroneModel:

    def test_hover(self, drone_model):
        self.step_to_t_end(drone_model, CtrlInput(f_z=-MG, m_x=0, m_y=0, m_z=0), 100)
        state = drone_model.get_6dof_state()

        assert np.all(np.abs(state.a_b) < 1e-3)

    def test_free_fall_equilibrium_with_drag(self, drone_model):
        self.step_to_t_end(drone_model, CtrlInput(f_z=0, m_x=0, m_y=0, m_z=0), 10)
        state = drone_model.get_6dof_state()

        assert np.all(np.abs(state.a_b) < 1e-3)

    @pytest.mark.parametrize("ctrl_input", [
        (CtrlInput(f_z=-MG, m_x=0, m_y=0, m_z=1)),
        (CtrlInput(f_z=-MG, m_x=0, m_y=0, m_z=-1)),
    ])
    def test_yaw_torque_equilibrium_with_drag(self, drone_model, ctrl_input):
        self.step_to_t_end(drone_model, ctrl_input, 10)
        state = drone_model.get_6dof_state()

        assert np.all(np.isclose(state.wp_b, 1e-9))

    @staticmethod
    def step_to_t_end(model: DroneModel, ctrl_input: CtrlInput, t_end: float):
        for _ in range(int(t_end // TEST_DT)):
            model.step(ctrl_input)
