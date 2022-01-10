import numpy as np
from scipy import signal

# noinspection PyUnresolvedReferences
from filter_design.filters import get_sos_filter_from_config, FilterConfig, FilterName, FilterType


class RealTimeFilter:

    def __init__(self, config: FilterConfig, fs: int):
        self._sos = get_sos_filter_from_config(config=config, fs=fs)

        self._z = np.zeros((self._sos.shape[0], 2))
        self._xf = 0.0

    def update(self, x: float):
        xf, self._z = signal.sosfilt(self._sos, [x], zi=self._z)
        self._xf = xf[0]

    def get(self) -> float:
        return self._xf
