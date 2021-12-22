import copy
from dataclasses import dataclass
from enum import Enum

import matplotlib.pyplot as plt
import numpy as np
from scipy import signal


class FilterType(Enum):
    LOW_PASS = 'lowpass'
    HIGH_PASS = 'highpass'
    BAND_PASS = 'bandpass'
    BAND_STOP = 'bandstop'


class FilterName(Enum):
    BUTTER = 'butter'
    BESSEL = 'bessel'
    CHEBY2 = 'cheby2'
    ELLIPTIC = 'elliptic'


DEFAULT_FILTER_TYPE = FilterType.LOW_PASS

DEFAULT_LOW_CUT_OFF = 10  # Hz
DEFAULT_HIGH_CUT_OFF = 20  # Hz

DEFAULT_N_ORDER = 5

DEFAULT_RIPPLE_PASS = 0.1  # db
DEFAULT_RIPPLE_STOP = 40  # db


@dataclass
class FilterConfig:
    name: FilterName
    type: FilterType = DEFAULT_FILTER_TYPE

    low_cut_off: float = DEFAULT_LOW_CUT_OFF  # Hz
    high_cut_off: float = DEFAULT_HIGH_CUT_OFF  # Hz

    n_order: int = DEFAULT_N_ORDER

    ripple_pass: float = DEFAULT_RIPPLE_PASS  # db
    ripple_stop: float = DEFAULT_RIPPLE_STOP  # db

    has_ripple_pass: bool = False
    has_ripple_stop: bool = False


DEFAULT_FILTER_CONFIG_MAP = {
    FilterName.BUTTER: FilterConfig(name=FilterName.BUTTER),
    FilterName.BESSEL: FilterConfig(name=FilterName.BESSEL),
    FilterName.CHEBY2: FilterConfig(name=FilterName.CHEBY2, has_ripple_stop=True),
    FilterName.ELLIPTIC: FilterConfig(name=FilterName.ELLIPTIC, has_ripple_pass=True, has_ripple_stop=True),
}


def get_default_filter_config_map():
    return copy.deepcopy(DEFAULT_FILTER_CONFIG_MAP)


def set_filter_config_map_params(config_map: {}, **kwargs):
    for filter_config in config_map.values():
        for param, value in kwargs.items():
            setattr(filter_config, param, value)


def get_sos_filter_from_config(config: FilterConfig, fs: int) -> np.ndarray:
    N = config.n_order
    Wn = get_wn_from_config(config)

    btype = config.type.value

    rs = config.ripple_stop
    rp = config.ripple_pass

    if config.name == FilterName.BUTTER:
        return signal.butter(N=N, Wn=Wn, btype=btype, fs=fs, output='sos')

    elif config.name == FilterName.BESSEL:
        return signal.bessel(N=N, Wn=Wn, btype=btype, norm='phase', fs=fs, output='sos')

    elif config.name == FilterName.CHEBY2:
        return signal.cheby2(N=N, Wn=Wn, rs=rs, btype=btype, fs=fs, output='sos')

    elif config.name == FilterName.ELLIPTIC:
        return signal.ellip(N=N, Wn=Wn, rp=rp, rs=rs, btype=btype, fs=fs, output='sos')

    else:
        raise ValueError


def get_wn_from_config(config: FilterConfig):
    if config.type is FilterType.LOW_PASS:
        return config.low_cut_off

    elif config.type is FilterType.HIGH_PASS:
        return config.high_cut_off

    elif config.type in [FilterType.BAND_PASS, FilterType.BAND_STOP]:
        return [config.low_cut_off, config.high_cut_off]

    else:
        raise ValueError


def add_wn_to_lines(config: FilterConfig, line_low: plt.Line2D, line_high: plt.Line2D):
    if config.type is FilterType.LOW_PASS:
        line_low.set_xdata(config.low_cut_off)
        line_high.set_xdata(np.nan)

        line_low.set_label(r'$f_{low}$')

    elif config.type is FilterType.HIGH_PASS:
        line_low.set_xdata(np.nan)
        line_high.set_xdata(config.high_cut_off)

        line_high.set_label(r'$f_{high}$')

    elif config.type in [FilterType.BAND_PASS, FilterType.BAND_STOP]:
        line_low.set_xdata(config.low_cut_off)
        line_high.set_xdata(config.high_cut_off)

        line_low.set_label(r'$f_{low}$')
        line_high.set_label(r'$f_{high}$')

    else:
        raise ValueError
