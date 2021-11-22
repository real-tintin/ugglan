from abc import ABC, abstractmethod
from typing import Callable

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from PyQt5.QtGui import QMatrix4x4, QQuaternion, QFont
from pyqtgraph.Qt import QtGui, QtCore


class SubplotWidget(ABC):

    def __init__(self, data_cb: Callable, *args, **kwargs):
        self._data_cb = data_cb
        self._base_widget = None

        self._post_init(*args, **kwargs)

    def _post_init(self, *args, **kwargs):
        self._ini_base_widget(*args, **kwargs)
        self._set_default_size_hint_and_policy()

    def update(self):
        self._update(**self._data_cb())

    def get_base_widget(self):
        return self._base_widget

    @abstractmethod
    def _ini_base_widget(self):
        pass

    @abstractmethod
    def _update(self, *args, **kwargs):
        pass

    def _set_default_size_hint_and_policy(self):
        """
        Set same size policy between widgets. Otherwise different widgets might not be
        seen in a shared grid, see https://groups.google.com/g/pyqtgraph/c/mTlUfT0ozT8.

        Note, for some reason the size policy of PlotWidget doesn't work as well as QLabel.
        """
        self._base_widget.sizeHint = lambda: QtGui.QLabel().sizeHint()
        self._base_widget.setSizePolicy(QtGui.QLabel().sizePolicy())


class SixDofWidget(SubplotWidget):
    def __init__(self, data_cb: Callable):
        super().__init__(data_cb)

    def _ini_base_widget(self):
        self._mesh = gl.GLMeshItem(meshdata=self.get_meshed_drone(), smooth=True, drawEdges=True, shader='balloon')

        self._base_widget = gl.GLViewWidget()
        self._base_widget.addItem(self._mesh)

        #  Note, as the axis in the 6dof model is rotated with pi about x, we set -y and -z.
        self._base_widget.addItem(gl.GLGridItem(size=QtGui.QVector3D(10, 10, 10)))
        self._base_widget.addItem(gl.GLAxisItem(QtGui.QVector3D(1, -1, -1)))

        self._base_widget.addItem(gl.GLTextItem(pos=[1, 0, 0], text='x (m)', font=QFont('Helvetica', 7)))
        self._base_widget.addItem(gl.GLTextItem(pos=[0, -1, 0], text='y (m)', font=QFont('Helvetica', 7)))
        self._base_widget.addItem(gl.GLTextItem(pos=[0, 0, -1], text='z (m)', font=QFont('Helvetica', 7)))

    def _update(self, r_i: np.ndarray, q: np.ndarray):
        transform = QMatrix4x4()

        transform.translate(r_i[0], -r_i[1], -r_i[2])
        transform.rotate(QQuaternion(*q))

        self._mesh.setTransform(transform)

    @staticmethod
    def get_meshed_drone():
        return gl.MeshData.cylinder(rows=10, cols=20, radius=[0.1, 0.2], length=0.05)  # TODO: Replace by actual drone.


class AttRefWidget(SubplotWidget):
    def __init__(self, data_cb: Callable, title: str, y_label: str, y_unit: str):
        super().__init__(data_cb, title, y_label, y_unit)

    def _ini_base_widget(self, title: str, y_label: str, y_unit: str):
        self._base_widget = pg.PlotWidget(title=title)
        self._base_widget.setLabel('bottom', 'Time', units='s')
        self._base_widget.setLabel('left', y_label, units=y_unit)
        self._base_widget.addLegend()
        self._base_widget.showGrid(x=True, y=True)

        self._plot_att_act = pg.PlotDataItem(pen=pg.mkPen(color="b", style=QtCore.Qt.SolidLine), name='act')
        self._plot_att_est = pg.PlotDataItem(pen=pg.mkPen(color="r", style=QtCore.Qt.SolidLine), name='est')
        self._plot_att_ref = pg.PlotDataItem(pen=pg.mkPen(color="g", style=QtCore.Qt.DashLine), name='ref')

        self._base_widget.addItem(self._plot_att_act)
        self._base_widget.addItem(self._plot_att_est)
        self._base_widget.addItem(self._plot_att_ref)

    def _update(self,
                t_s: np.ndarray,
                att_act: np.ndarray,
                att_est: np.ndarray,
                att_ref: np.ndarray):
        self._plot_att_act.setData(x=t_s, y=att_act)
        self._plot_att_est.setData(x=t_s, y=att_est)
        self._plot_att_ref.setData(x=t_s, y=att_ref)
