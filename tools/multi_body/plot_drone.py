import matplotlib.pyplot as plt
import numpy as np

from .mb_drone import drone
from .multi_body import MultiBody
from .plot_multi_body import plot_multi_body
from .utils import update_inertia


def main():
    update_inertia(drone)
    plot_multi_body(drone, ax_lim=[-0.3, 0.3])

    _print_mb_inertia(drone)
    plt.show()


def _print_mb_inertia(mb: MultiBody):
    print("*** Drone multi body inertia ***")
    print("    m  = %.3f [kg]" % mb.mass_kg)
    print("    CM = %s [m]" % np.array2string(mb.com_m, precision=3, suppress_small=True))
    print("    I  = \n %s [kgm^2]" % np.array2string(mb.moi_kg2, precision=3, suppress_small=True))


if __name__ == "__main__":
    main()
