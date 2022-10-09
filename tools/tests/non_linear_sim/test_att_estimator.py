import numpy as np
import pytest

from ugglan_tools.non_linear_sim.att_estimator import AttEstimator, DEFAULT_ATT_EST_PARAMS, ImuOut

TEST_DT = 0.01

SAMPLES_UNTIL_CONVERGENCE = 1000


@pytest.fixture
def default_estimator():
    yield AttEstimator(
        params=DEFAULT_ATT_EST_PARAMS,
        dt=TEST_DT,
    )


class TestAttEstimator:

    @pytest.mark.parametrize("imu_out, rotation, exp_angle", [
        (ImuOut(acc_y=-np.sin(np.pi / 2), acc_z=-np.cos(np.pi / 2)), "roll", np.pi / 2),
        (ImuOut(acc_x=np.sin(-np.pi / 6), acc_z=-np.cos(-np.pi / 6)), "pitch", -np.pi / 6),
        (ImuOut(mag_field_x=1.0, mag_field_y=1.0), "yaw", np.pi / 4),
    ])
    def test_angle(self, default_estimator, imu_out, rotation, exp_angle):
        self.update_n_times(default_estimator, imu_out, SAMPLES_UNTIL_CONVERGENCE)
        act_angle = getattr(default_estimator.get_estimate(), rotation).angle

        assert np.isclose(act_angle, exp_angle, 1e-3)

    @staticmethod
    def update_n_times(estimator: AttEstimator, imu_out: ImuOut, n: int):
        for _ in range(n):
            estimator.update(imu_out=imu_out)
