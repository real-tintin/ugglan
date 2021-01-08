import matplotlib.pyplot as plt
import numpy as np

from multi_body import MultiBody
from multi_body_drone import drone
from multi_body_utils import update_inertia
from plot_multi_body import plot_multi_body


def main():
    """
    Analyze the drone.
    """
    update_inertia(drone)
    plot_multi_body(drone, ax_lim=[-0.3, 0.3])

    _print_mb_inertia(drone)
    plt.show()


def _print_mb_inertia(mb: MultiBody):
    print("*** Drone multi body inertia ***")
    print("    m  = %.3f [kg]" % mb.mass_kg)
    print("    CM = %s [m]" % np.array2string(mb.center_of_mass, precision=3, suppress_small=True))
    print("    I  = \n %s [kgm^2]" % np.array2string(mb.mom_of_inertia, precision=3, suppress_small=True))


if __name__ == "__main__":
    main()
