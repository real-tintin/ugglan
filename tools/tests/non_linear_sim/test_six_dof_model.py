import numpy as np
import pytest

from ugglan_tools.non_linear_sim.six_dof_model import State, SixDofModel, BodyInput, get_zero_initialized_state

STATE_NON_ZERO = State(
    r_i=np.random.rand(3),
    v_i=np.random.rand(3),
    a_i=np.random.rand(3),

    v_b=np.random.rand(3),
    a_b=np.random.rand(3),

    n_i=np.random.rand(3),
    w_b=np.random.rand(3),
)

TEST_DT = 0.01
STATE_ZERO = get_zero_initialized_state()


class UnitSixDofModel(SixDofModel):
    def __init__(self):
        super().__init__(mass=1.0, moment_of_inertia=np.eye(3), dt=TEST_DT)


@pytest.fixture
def unit_model_zero_init():
    yield UnitSixDofModel()


def assert_states_eq(exp, act):
    for member in State.__annotations__:
        np.array_equal(getattr(exp, member), getattr(act, member))


class TestSixDofModel:

    def test_standstill(self, unit_model_zero_init):
        self.step_n_times(unit_model_zero_init, BodyInput(fx=0, fy=0, fz=0, mx=0, my=0, mz=0), n=100)

        assert_states_eq(unit_model_zero_init.get_state(), STATE_ZERO)

    @pytest.mark.parametrize("moving_axis, body_input", [
        (0, BodyInput(fx=1.0, fy=0, fz=0, mx=0, my=0, mz=0)),
        (1, BodyInput(fx=0, fy=-0.5, fz=0, mx=0, my=0, mz=0)),
        (2, BodyInput(fx=0, fy=0, fz=0.3, mx=0, my=0, mz=0)),
        (3, BodyInput(fx=0, fy=0, fz=0, mx=-0.2, my=0, mz=0)),
        (4, BodyInput(fx=0, fy=0, fz=0, mx=0, my=1.0, mz=0)),
        (5, BodyInput(fx=0, fy=0, fz=0, mx=0, my=0, mz=-1.3)),
    ])
    def test_constant_force_or_torque(self, unit_model_zero_init, moving_axis, body_input):
        standstill_axis = list(set(range(6)) - {moving_axis})

        self.step_n_times(unit_model_zero_init, body_input, n=10)
        state = unit_model_zero_init.get_state()
        position = np.array([*state.r_i, *state.n_i])

        assert position[moving_axis] != 0
        assert not np.any(position[standstill_axis])

    def test_accelerate_and_stop(self, unit_model_zero_init):
        state = unit_model_zero_init.get_state()
        assert not np.any(state.v_b)
        assert not np.any(state.w_b)

        self.step_n_times(unit_model_zero_init, BodyInput(fx=1, fy=-1, fz=1, mx=-1, my=1, mz=-1), n=20)
        state = unit_model_zero_init.get_state()
        assert np.all(state.v_b)
        assert np.all(state.w_b)

        self.step_n_times(unit_model_zero_init, BodyInput(fx=-1, fy=1, fz=-1, mx=1, my=-1, mz=1), n=20)
        state = unit_model_zero_init.get_state()
        assert np.all(np.isclose(state.v_b, 1e-9))
        assert np.all(np.isclose(state.w_b, 1e-9))

    @pytest.mark.parametrize("init_n_i, expression", [
        (np.array([np.pi / 4, 0, 0]), "state.r_i[1] < 0"),
        (np.array([-np.pi / 8, 0, 0]), "state.r_i[1] > 0"),
        (np.array([0, -np.pi / 4, 0]), "state.r_i[0] > 0"),
        (np.array([0, np.pi / 8, 0]), "state.r_i[0] < 0"),
        (np.array([0, 0, np.pi]), "np.all(np.isclose(state.r_i))"),
    ])
    def test_translation_using_rotation_and_fz(self, unit_model_zero_init, init_n_i, expression):
        unit_model_zero_init.reset(state=State(n_i=init_n_i))

        self.step_n_times(unit_model_zero_init, BodyInput(fx=0, fy=0, fz=-1, mx=0, my=0, mz=0), n=20)
        state = unit_model_zero_init.get_state()

        assert expression

    @staticmethod
    def step_n_times(model: SixDofModel, body_input: BodyInput, n: int):
        for _ in range(n):
            model.step(body_input)
