import pytest

from non_linear_sim.six_dof_model import *

STATE_NON_ZERO = State(pos=1, vel=0.1, acc=0.01)

STATES_NON_ZERO = States(
    x=STATE_NON_ZERO,
    y=STATE_NON_ZERO,
    z=STATE_NON_ZERO,
    phi=STATE_NON_ZERO,
    theta=STATE_NON_ZERO,
    psi=STATE_NON_ZERO,
)


@pytest.fixture
def model_zero_init():
    yield SixDofModel()


class TestSixDofModel:

    def test_init_without_arg(self):
        model = SixDofModel()
        assert model.get_states() == STATES_ZERO

    @pytest.mark.parametrize("states", [STATES_ZERO, STATES_NON_ZERO])
    def test_init_with_arg(self, states):
        model = SixDofModel(states)
        assert model.get_states() == states

    def test_step(self):
        # TODO: Test step with various scenarios (zero input, const acc, step acc, back and fourth, ...)
        pass

    def test_reset_without_arg(self, model_zero_init):
        model_zero_init.reset()
        assert model_zero_init.get_states() == STATES_ZERO

    @pytest.mark.parametrize("states", [STATES_ZERO, STATES_NON_ZERO])
    def test_reset_with_arg(self, model_zero_init, states):
        model_zero_init.reset(states)
        assert model_zero_init.get_states() == states
