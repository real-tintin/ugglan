import copy

import matplotlib as mpl
import matplotlib.pyplot as plt
from cycler import cycler
from scipy import signal

from attitude_est.estimators import *

mpl.rcParams['axes.prop_cycle'] = cycler(color='bgrcmyk')
mpl.rcParams['lines.linewidth'] = 0.5

FS = 100
DT = 1 / FS

N = 2
FC = 5.0

T_0 = 0.0
T_END = 10.0


class Biquad:

    def __init__(self, a: np.ndarray, b: np.ndarray, dt=None):
        self._a = a
        self._b = b

        self._dt = dt

        self._x = np.zeros(3)
        self._y = np.zeros(3)

    def update(self, new_sample):
        self._x[0:2] = self._x[1:3]
        self._y[0:2] = self._y[1:3]

        self._x[2] = new_sample

        if self._dt is None:
            bp0 = self._b[0] / self._a[0]
            bp1 = self._b[1] / self._a[0]
            bp2 = self._b[2] / self._a[0]

            ap1 = self._a[1] / self._a[0]
            ap2 = self._a[2] / self._a[0]

        else:
            w0 = 2 * np.pi * FC
            K = 2 / DT * np.tan(w0 * DT / 2)  # Correct?
            c = self._a[0] * K ** 2 + self._a[1] * K + self._a[2]

            bp0 = (self._b[0] * K ** 2 + self._b[1] * K + self._b[2]) / c
            bp1 = 2 * (self._b[2] - self._b[0] * K ** 2) / c
            bp2 = (self._b[0] * K ** 2 - self._b[1] * K + self._b[2]) / c

            ap1 = 2 * (self._a[2] - self._a[0] * K ** 2) / c
            ap2 = (self._a[0] * K ** 2 - self._a[1] * K + self._a[2]) / c

        self._y[2] = (bp0 * self._x[2] + \
                      bp1 * self._x[1] + \
                      bp2 * self._x[0] - \
                      ap1 * self._y[1] - \
                      ap2 * self._y[0])

        return self._y[2]


def using_scipy_sosfilt(sos_filter, x):
    return signal.sosfilt(sos_filter, x)


def reformat_sos_coef(c):
    """
    This to handle lower orders of coefficients i.e., when order < len(c).
    """
    n_order = len(c) - 1
    cr = list(copy.copy(c))

    while len(cr) > 0:
        if cr[-1] == 0:
            cr.pop(-1)
            n_order -= 1
        else:
            break

    return np.concatenate([np.zeros(len(c) - 1 - n_order), np.array(cr)])


def using_trans_biquad(sos_filter, x, dt=None):
    """
    If fs is defined, the sos_filter is expected to be
    analogs i.e., in the continuous time domain.
    """
    n_samples = len(x)
    n_biquads = sos_filter.shape[0]

    y = np.zeros(n_samples)

    biquads = []
    for i_sos in range(0, n_biquads):
        a = reformat_sos_coef(sos_filter[i_sos][3:6])
        b = reformat_sos_coef(sos_filter[i_sos][0:3])

        biquads.append(Biquad(a=a, b=b, dt=dt))

    for i_sample in range(0, n_samples):
        y_i = x[i_sample]

        for biquad in biquads:
            y_i = biquad.update(y_i)

        y[i_sample] = y_i

    return y


def main():
    t = np.arange(T_0, T_END, DT)
    x = np.sin(2 * np.pi * 1 * t) + 0.1 * np.cos(2 * np.pi * 20 * t) + np.random.rand(len(t))

    sos_filter_a = signal.butter(N=N, Wn=FC, btype='high', analog=True, output='sos')
    sos_filter_d = signal.butter(N=N, Wn=FC, btype='high', fs=FS, output='sos')

    x_sp_d = using_scipy_sosfilt(sos_filter_d, x)
    x_tb_d = using_trans_biquad(sos_filter_d, x)
    x_tb_a = using_trans_biquad(sos_filter_a, x, dt=DT)

    fig, ax = plt.subplots(1, 1)

    ax.plot(t, x, label='$x_{raw}$')
    ax.plot(t, x_sp_d, label='$x_{scipy}^d$')
    ax.plot(t, x_tb_d, label='$x_{biquad}^d$')
    #ax.plot(t, x_tb_a, label='$x_{biquad}^a$')

    ax.set(xlabel='time [s]')
    ax.grid()
    ax.legend(loc='upper right')

    plt.show()


if __name__ == '__main__':
    main()
