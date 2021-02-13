from multi_body.mb_drone import drone
from multi_body.plot_multi_body import plot_multi_body


def test_plot_multi_body():
    plot_multi_body(drone, ax_lim=[-1, 1])
