from pathlib import Path
from typing import List, Union

import data_log.io as data_log_io
import numpy as np

ROOT_PATH = Path(__file__).parent.absolute()
DATA_LOG_PATH = ROOT_PATH / 'resources' / '20200904193108.dat'

EXP_LEN_IMU_ACC_X = 5
EXP_VAL_IMU_ACC_X = 0.3

EXP_LEN_ESC_STATUS_0 = 10
EXP_VAL_ESC_STATUS_0 = 0x02


def _list_all_equal_to(l: List, v: Union[int, float]):
    return set(l) == set([v])


def test_read():
    data = data_log_io.read(DATA_LOG_PATH)

    assert len(data.IMU.AccelerationX.val) == EXP_LEN_IMU_ACC_X
    assert len(data.IMU.AccelerationX.t_s) == EXP_LEN_IMU_ACC_X
    assert _list_all_equal_to(data.IMU.AccelerationX.val, EXP_VAL_IMU_ACC_X)

    assert len(data.ESC.Status0.val) == EXP_LEN_ESC_STATUS_0
    assert len(data.ESC.Status0.t_s) == EXP_LEN_ESC_STATUS_0
    assert _list_all_equal_to(data.ESC.Status0.val, EXP_VAL_ESC_STATUS_0)


def test_read_resample():
    sample_rate = 0.01
    data = data_log_io.read(DATA_LOG_PATH, resample_to_fixed_rate_s=sample_rate)

    assert np.all(np.isclose(np.diff(data.IMU.AccelerationX.t_s), sample_rate))
