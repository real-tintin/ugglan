from typing import List

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from .multi_body import MultiBody
from .plot_shape import plot_shape


def plot_multi_body(m_body: MultiBody, ax_lim: List[float]) -> Axes3D:
    """
    Creates a 3d plot of a multi body
    object.
    """
    fig = plt.figure()
    plot3_ax = fig.gca(projection='3d')

    for body in m_body.bodies:
        plot_shape(plot3_ax,
                   body.shape_m,
                   body.rotation_rad,
                   body.translation_m,
                   body.color)

    plot3_ax.set_xlim(ax_lim)
    plot3_ax.set_ylim(ax_lim)
    plot3_ax.set_zlim(ax_lim)

    plot3_ax.set_xlabel("x [m]")
    plot3_ax.set_ylabel("y [m]")
    plot3_ax.set_zlabel("z [m]")

    return plot3_ax
