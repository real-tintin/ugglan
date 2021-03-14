import matplotlib.pyplot as plt
import numpy as np
import pytest
# noinspection PyUnresolvedReferences
from mpl_toolkits.mplot3d import Axes3D

from multi_body.plot_shape import plot_shape
from multi_body.shapes import *


@pytest.fixture
def plot3_ax():
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    yield ax

    plt.close(fig)


@pytest.mark.parametrize("shape", [Cuboid(1, 1, 1), Cuboid(4, 2, 1),
                                   Cylinder(1, 1), Cylinder(0.5, 2),
                                   Sphere(1), Sphere(3.14)])
@pytest.mark.parametrize("rot_b", [[0, 0, 0], [0, np.pi / 2, 0]])
@pytest.mark.parametrize("rot_i", [[0, 0, 0], [0, 0, np.pi / 2]])
@pytest.mark.parametrize("tran_i", [[0, 0, 0], [1, -1, -3]])
@pytest.mark.parametrize("color", [None, [1, 1, 1], 'green'])
def test_plot_3d_shape(plot3_ax, shape, rot_b, tran_i, rot_i, color):
    plot_shape(plot3_ax,
               shape,
               rot_b,
               tran_i,
               rot_i,
               color)
