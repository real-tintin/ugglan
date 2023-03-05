from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Callable, List

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from PyQt6.QtGui import QMatrix4x4, QFont, QColor
from PyQt6.QtWidgets import QLabel
from pyqtgraph.Qt import QtGui, QtCore

from ugglan_tools.multi_body_sim.mb_drone import drone
from ugglan_tools.multi_body_sim.meshed_multi_body import MeshedMultiBody

DEFAULT_COLOR_ORDER = [QColor(31, 119, 180),
                       QColor(214, 39, 40),
                       QColor(255, 127, 14),
                       QColor(127, 127, 127),
                       QColor(23, 190, 207)]  # Matplotlib v2.0 color palette


@dataclass
class SixDofConfig:
    lock_origin: bool = False


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
        self._base_widget.sizeHint = lambda: QLabel().sizeHint()
        self._base_widget.setSizePolicy(QLabel().sizePolicy())


class SixDofWidget(SubplotWidget):
    """
    Note, as the axis in the 6dof model is rotated with pi about x, we set -y and -z.
    """

    def __init__(self, data_cb: Callable, config: SixDofConfig = SixDofConfig()):
        super().__init__(data_cb)

        self._config = config

    def _ini_base_widget(self):
        self._base_widget = gl.GLViewWidget()
        self._meshed_drone = self.get_meshed_drone()

        for mesh_item in self._meshed_drone.iter_mesh_items():
            self._base_widget.addItem(mesh_item)

        self._text_label = QLabel(self._base_widget)
        self._text_label.setStyleSheet("QLabel { color : white; }")
        self._text_label.setText(self._format_text_label(np.zeros(3), np.zeros(3), np.zeros(3)))
        self._text_label.setFont(QFont("Consolas", 8))

        self._base_widget.addItem(gl.GLGridItem(size=QtGui.QVector3D(10, 10, 10)))
        self._base_widget.addItem(gl.GLAxisItem(QtGui.QVector3D(1, -1, -1)))

        self._base_widget.addItem(gl.GLTextItem(pos=[1, 0, 0], text='x (m)', font=QFont('Helvetica', 7)))
        self._base_widget.addItem(gl.GLTextItem(pos=[0, -1, 0], text='y (m)', font=QFont('Helvetica', 7)))
        self._base_widget.addItem(gl.GLTextItem(pos=[0, 0, -1], text='z (m)', font=QFont('Helvetica', 7)))

    def _update(self, r_i: np.ndarray, v_i: np.ndarray, a_i: np.ndarray, n_i: np.ndarray):
        if self._config.lock_origin:
            camera_pos = QtGui.QVector3D(0, 0, 0)
        else:
            camera_pos = QtGui.QVector3D(r_i[0], -r_i[1], -r_i[2])

        self._base_widget.setCameraPosition(pos=camera_pos)

        transform = QMatrix4x4()

        transform.translate(r_i[0], -r_i[1], -r_i[2])

        transform.rotate(np.rad2deg(n_i[0]), 1, 0, 0)
        transform.rotate(np.rad2deg(n_i[1]), 0, -1, 0)
        transform.rotate(np.rad2deg(n_i[2]), 0, 0, -1)

        self._meshed_drone.set_transform(transform)

        self._text_label.setText(self._format_text_label(r_i, v_i, a_i))

    @staticmethod
    def _format_text_label(r_i: np.ndarray, v_i: np.ndarray, a_i: np.ndarray):
        def arr2str(x):
            return np.array2string(x, precision=2, floatmode='fixed', sign=' ', suppress_small=True)

        return " r_i = {} m\n" \
               " v_i = {} m/s\n" \
               " a_i = {} m/s^2".format(arr2str(r_i), arr2str(v_i), arr2str(a_i))

    @staticmethod
    def get_meshed_drone():
        return MeshedMultiBody(multi_body=drone)


class LinePlotWidget(SubplotWidget):
    def __init__(self, data_cb: Callable,
                 labels: [],
                 y_label: str,
                 y_unit: str,
                 color_order: List = None,
                 line_styles: List = None,
                 width: float = 1.0):
        super().__init__(data_cb, labels, y_label, y_unit, color_order, line_styles, width)

    def _ini_base_widget(self, labels, y_label, y_unit, color_order, styles, width):
        self._base_widget = pg.PlotWidget()

        self._base_widget.setLabel('bottom', 'Time', units='s')
        self._base_widget.setLabel('left', y_label, units=y_unit)

        self._base_widget.addLegend()
        self._base_widget.showGrid(x=True, y=True)

        n_lines = len(labels)
        self._line_plots = []

        if color_order is None:
            color_order = DEFAULT_COLOR_ORDER

        if styles is None:
            styles = [QtCore.Qt.PenStyle.SolidLine for _ in range(n_lines)]

        for i_line in range(n_lines):
            line_plot = pg.PlotDataItem(pen=pg.mkPen(color=color_order[i_line],
                                                     style=styles[i_line],
                                                     width=width),
                                        name=labels[i_line])

            self._line_plots.append(line_plot)
            self._base_widget.addItem(line_plot)

    def _update(self, t_s: List, y: List[List]):
        for i_line, line_plot in enumerate(self._line_plots):
            line_plot.setData(x=t_s, y=y[i_line])
