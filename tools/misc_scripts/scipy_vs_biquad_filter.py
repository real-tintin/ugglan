import matplotlib as mpl
import matplotlib.pyplot as plt
from cycler import cycler
from scipy import signal

from attitude_est.estimators import *

mpl.rcParams['axes.prop_cycle'] = cycler(color='bgrcmyk')
mpl.rcParams['lines.linewidth'] = 0.5

FS = 100
DT = 1 / FS

N = 3
Wn = 5.0

T_0 = 0.0
T_END = 10.0


class Biquad:

    def __init__(self, a: np.ndarray, b: np.ndarray):
        self._a = a
        self._b = b

        self._x = np.zeros(3)
        self._y = np.zeros(3)

    def update(self, new_sample):
        self._x[0:2] = self._x[1:3]
        self._y[0:2] = self._y[1:3]

        self._x[2] = new_sample

        self._y[2] = 1 / self._a[0] * (self._b[0] * self._x[2] + \
                                       self._b[1] * self._x[1] + \
                                       self._b[2] * self._x[0] - \
                                       self._a[1] * self._y[1] - \
                                       self._a[2] * self._y[0])

        return self._y[2]


def using_scipy_sosfilt(sos_filter, x):
    return signal.sosfilt(sos_filter, x)


def using_trans_biquad(sos_filter, x, fs=None):
    """
    If fs is defined, the sos_filter is expected to be
    analogs i.e., in the continuous time domain.
    """
    # TODO: Handle fs
    n_samples = len(x)
    n_biquads = sos_filter.shape[0]

    y = np.zeros(n_samples)

    biquads = []
    for i_sos in range(0, n_biquads):
        biquads.append(Biquad(a=sos_filter[i_sos][3:6], b=sos_filter[i_sos][0:3]))

    for i_sample in range(0, n_samples):
        y_i = x[i_sample]

        for biquad in biquads:
            y_i = biquad.update(y_i)

        y[i_sample] = y_i

    return y


def main():
    t = np.arange(T_0, T_END, DT)
    x = np.sin(2 * np.pi * 1 * t) + 0.1 * np.cos(2 * np.pi * 50 * t) + np.random.rand(len(t))

    sos_filter_a = signal.butter(N=N, Wn=Wn, btype='low', analog=True, output='sos')
    sos_filter_d = signal.butter(N=N, Wn=Wn, btype='low', fs=FS, output='sos')

    x_sp = using_scipy_sosfilt(sos_filter_d, x)
    x_tb = using_trans_biquad(sos_filter_d, x)

    fig, ax = plt.subplots(1, 1)

    ax.plot(t, x, label='$x_{raw}$')
    ax.plot(t, x_sp, label='$x_{scipy}$')
    ax.plot(t, x_tb, label='$x_{biquad}$')

    ax.set(xlabel='time [s]')
    ax.grid()
    ax.legend(loc='upper right')

    plt.show()


if __name__ == '__main__':
    main()
