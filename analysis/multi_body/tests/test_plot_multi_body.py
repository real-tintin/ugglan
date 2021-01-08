from multi_body_drone import drone
from plot_multi_body import plot_multi_body


def test_plot_multi_body():
    plot_multi_body(drone, ax_lim=[-1, 1])
