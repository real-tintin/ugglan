import argparse
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from cycler import cycler
from mpl_toolkits.mplot3d import Axes3D

import ugglan_tools.data_log.io as data_log_io

mpl.rcParams['axes.prop_cycle'] = cycler(color='bgrcmyk')
mpl.rcParams['lines.linewidth'] = 0.5

IMU_SAMPLE_RATE_S = 0.01  # 100 Hz

TITLE_STR = r'Hard iron offset: $V_x$ = {:.3f}, $V_y$ = {:.3f}, $V_z$ = {:.3f}'


def _hard_iron_offset(data):
    """ Get signals """
    t_s = np.array(data.Imu.MagneticFieldX.t_s)

    mag_x = np.array(data.Imu.MagneticFieldX.val)
    mag_y = np.array(data.Imu.MagneticFieldY.val)
    mag_z = np.array(data.Imu.MagneticFieldZ.val)

    """ Estimate offset using MLSE (Bp-V)'*(Bp-V) = B^2 """
    x = np.array([mag_x, mag_y, mag_z, np.ones(len(t_s))])
    y = mag_x ** 2 + mag_y ** 2 + mag_z ** 2
    b, *_ = np.linalg.lstsq(x.transpose(), y.transpose(), rcond=None)
    v_x = b[0] / 2
    v_y = b[1] / 2
    v_z = b[2] / 2
    radius = np.sqrt(b[3] + v_x ** 2 + v_y ** 2 + v_z ** 2)

    """ Correct for hard iron offset """
    mag_cor_x = mag_x - v_x
    mag_cor_y = mag_y - v_y
    mag_cor_z = mag_z - v_z

    """ Plot in 3d using spheres """
    ax = _create_3d_fig()

    _plot_sphere(ax, radius, v_x, v_y, v_z, 'r')
    _plot_sphere(ax, radius, 0, 0, 0, 'b')

    ax.scatter(mag_x, mag_y, mag_z, label='raw', color='r')
    ax.scatter(mag_cor_x, mag_cor_y, mag_cor_z, label='corrected', color='b')

    ax.set_title(TITLE_STR.format(v_x, v_y, v_z))
    ax.set_xlabel('x [gauss]')
    ax.set_ylabel('y [gauss]')
    ax.set_zlabel('z [gauss]')
    ax.legend()
    ax.grid()


def _create_3d_fig() -> Axes3D:
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    return ax


def _plot_sphere(ax, radius, offset_x, offset_y, offset_z, color):
    u, v = np.mgrid[0:2 * np.pi:20j, 0:np.pi:10j]

    x = np.cos(u) * np.sin(v) * radius + offset_x
    y = np.sin(u) * np.sin(v) * radius + offset_y
    z = np.cos(v) * radius + offset_z

    ax.plot_wireframe(x, y, z, color=color)


def main():
    parser = argparse.ArgumentParser(description='Hard iron offset estimation of data log file.')
    parser.add_argument('path', type=Path, help='Path to data log file')
    args = parser.parse_args()

    data = data_log_io.read(args.path, resample_to_fixed_rate_s=IMU_SAMPLE_RATE_S)
    _hard_iron_offset(data)
    plt.show()


if __name__ == "__main__":
    main()
