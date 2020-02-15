import matplotlib.pyplot as plt
import numpy as np

import multi_body_utils
from multi_body import MultiBody
from multi_body_drone import drone


def main():
    """
    Analyze the drone.
    """
    multi_body_utils.inertia(drone)
    multi_body_utils.plot(drone, ax_lim=[-0.3, 0.3])

    _print_mb_inertia(drone)
    plt.show()


def _print_mb_inertia(mb: MultiBody):
    print("*** Drone multi body inertia ***")
    print("    m  = %.3f [kg]" % mb.mass_kg)
    print("    CM = %s [m]" % np.array2string(mb.center_of_mass, precision=3, suppress_small=True))
    print("    I  = \n %s [kgm^2]" % np.array2string(mb.mom_of_inertia, precision=3, suppress_small=True))


if __name__ == "__main__":
    main()
