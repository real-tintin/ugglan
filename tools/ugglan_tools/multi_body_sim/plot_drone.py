import argparse
import sys

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from PyQt5.QtGui import QFont
from pyqtgraph.Qt import QtGui

from ugglan_tools.multi_body_sim.mb_drone import drone
from ugglan_tools.multi_body_sim.meshed_multi_body import MeshedMultiBody
from ugglan_tools.multi_body_sim.multi_body import MultiBody
from ugglan_tools.multi_body_sim.utils import update_inertia


def main():
    parser = argparse.ArgumentParser(description='Plot the drone multi body.')
    parser.add_argument('--print-inertia', action='store_true', help='Use to print the estimated inertia')
    args = parser.parse_args()

    update_inertia(drone)

    _plot_multi_body(drone)

    if args.print_inertia:
        _print_mb_inertia(drone)


def _plot_multi_body(mb: MultiBody):
    app = pg.mkQApp(mb.name)

    view = gl.GLViewWidget()
    view.setBackgroundColor([255, 255, 255])

    grid = gl.GLGridItem(size=QtGui.QVector3D(10, 10, 10), color=[0, 0, 0])
    grid.setSpacing(x=0.5, y=0.5, z=0.5)

    view.addItem(grid)
    view.addItem(gl.GLAxisItem(QtGui.QVector3D(0.2, 0.2, 0.2)))

    view.addItem(gl.GLTextItem(pos=[0.2, 0, 0], text='x (m)', font=QFont('Helvetica', 8), color=[0, 0, 0]))
    view.addItem(gl.GLTextItem(pos=[0, 0.2, 0], text='y (m)', font=QFont('Helvetica', 8), color=[0, 0, 0]))
    view.addItem(gl.GLTextItem(pos=[0, 0, 0.2], text='z (m)', font=QFont('Helvetica', 8), color=[0, 0, 0]))

    meshed_drone = MeshedMultiBody(multi_body=drone)

    for mesh_item in meshed_drone.iter_mesh_items():
        view.addItem(mesh_item)

    view.setCameraPosition(distance=1)
    view.show()

    sys.exit(app.exec_())


def _print_mb_inertia(mb: MultiBody):
    print("*** Drone multi body inertia ***")
    print("    m  = %.3f [kg]" % mb.mass_kg)
    print("    CM = %s [m]" % np.array2string(mb.com_m, precision=3, suppress_small=True))
    print("    I  = \n %s [kgm^2]" % np.array2string(mb.moi_kg2, precision=3, suppress_small=True))


if __name__ == "__main__":
    main()
