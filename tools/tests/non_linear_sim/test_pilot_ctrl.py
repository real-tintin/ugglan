from copy import copy
from typing import Set

import numpy as np
import pytest

from ugglan_tools.non_linear_sim.att_estimator import AttEstimate
from ugglan_tools.non_linear_sim.pilot_ctrl import PilotCtrl, Params, RefInput, CtrlInput

TEST_DT = 0.01

ZERO_ATT_ESTIMATE = AttEstimate()


@pytest.fixture
def default_pilot_ctrl():
    yield PilotCtrl(
        params=Params(),
        dt=TEST_DT,
    )


class TestPilotCtrl:

    @pytest.mark.parametrize("ref, exp_ctrl_pos, exp_ctrl_neg", [
        (RefInput(f_z=1, roll=0, pitch=0, yaw_rate=0), "f_z", None),
        (RefInput(f_z=-1, roll=0, pitch=0, yaw_rate=0), None, "f_z"),

        (RefInput(f_z=0, roll=np.pi, pitch=0, yaw_rate=0), "m_x", None),
        (RefInput(f_z=0, roll=-np.pi, pitch=0, yaw_rate=0), None, "m_x"),

        (RefInput(f_z=0, roll=0, pitch=np.pi, yaw_rate=0), "m_y", None),
        (RefInput(f_z=0, roll=0, pitch=-np.pi, yaw_rate=0), None, "m_y"),

        (RefInput(f_z=0, roll=0, pitch=0, yaw_rate=np.pi), "m_z", None),
        (RefInput(f_z=0, roll=0, pitch=0, yaw_rate=-np.pi), None, "m_z"),
    ])
    def test_sign_of_ctrl(self, default_pilot_ctrl, ref, exp_ctrl_pos, exp_ctrl_neg):
        ctrl_members = set(CtrlInput.__annotations__.keys())

        default_pilot_ctrl.update(ref_input=ref, att_estimate=ZERO_ATT_ESTIMATE)
        act_ctrl = default_pilot_ctrl.get_ctrl_input()

        if exp_ctrl_pos:
            exp_members_zero = ctrl_members - set([exp_ctrl_pos])

            self.assert_ctrl_members_zero(act_ctrl, exp_members_zero)
            assert getattr(act_ctrl, exp_ctrl_pos) > 0

        elif exp_ctrl_neg:
            exp_members_zero = ctrl_members - set([exp_ctrl_neg])

            self.assert_ctrl_members_zero(act_ctrl, exp_members_zero)
            assert getattr(act_ctrl, exp_ctrl_neg) < 0

    def test_anti_windup(self, default_pilot_ctrl):
        n_until_saturated = 1000
        ref = RefInput(f_z=0, roll=np.pi / 2, pitch=np.pi / 4, yaw_rate=np.pi)

        self.update_n_times(pilot_ctrl=default_pilot_ctrl, ref=ref, est=ZERO_ATT_ESTIMATE, n=n_until_saturated)
        ctrl_input_saturated = copy(default_pilot_ctrl.get_ctrl_input())

        self.update_n_times(pilot_ctrl=default_pilot_ctrl, ref=ref, est=ZERO_ATT_ESTIMATE, n=n_until_saturated)
        assert ctrl_input_saturated == default_pilot_ctrl.get_ctrl_input()

    @staticmethod
    def update_n_times(pilot_ctrl: PilotCtrl, est: AttEstimate, ref: RefInput, n: int):
        for _ in range(n):
            pilot_ctrl.update(ref_input=ref, att_estimate=est)

    @staticmethod
    def assert_ctrl_members_zero(ctrl: CtrlInput, members: Set[str]):
        for member in members:
            assert getattr(ctrl, member) == 0
