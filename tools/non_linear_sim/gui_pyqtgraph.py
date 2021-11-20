import sys
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import Union, Callable

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from PyQt5.QtGui import QMatrix4x4, QQuaternion, QFont
from pyqtgraph.Qt import QtGui, QtWidgets, QtCore

from non_linear_sim.att_estimator import DEFAULT_ATT_EST_PARAMS
from non_linear_sim.drone_model import DEFAULT_DRONE_PARAMS, DEFAULT_ENV_PARAMS
from non_linear_sim.pilot_ctrl import DEFAULT_PILOT_CTRL_PARAMS, RefInput
from non_linear_sim.rolling_sim_data import RollingSimData
from non_linear_sim.simulator import DEFAULT_IMU_NOISE
from non_linear_sim.simulator import Simulator
from non_linear_sim.six_dof_model import STATE_ZERO as SIX_DOF_STATE_ZERO
from non_linear_sim.threaded_task import ThreadedTask


# TODO: Add menus for update of all conf and params.
# TODO: Gamepad logic class and refactor with ref step.
# TODO: Subplots to add: Pilotctrl, att_est and motor_ang_rate.


class Subplot(Enum):
    SIX_DOF = 'six_dof'

    ROLL = 'roll'
    PITCH = 'pitch'
    YAW_RATE = 'yaw_rate'

    PILOT_CTRL = 'pilot_ctrl'
    ATT_EST = 'att_estimation'

    MOTOR_ANG_RATES = 'motor_ang_rates'


class GridPos(Enum):
    TOP_LEFT = 0
    TOP_RIGHT = 1
    BOTTOM_LEFT = 2
    BOTTOM_RIGHT = 3


class GuiState(Enum):
    INIT = 0
    RUNNING = 1
    STOPPED = 2


@dataclass
class ConfSim:
    dt_s: float = 0.01
    standstill_calib_att_est: bool = True


DEFAULT_CONF_SIM = ConfSim()


@dataclass
class ConfGui:
    refresh_rate_s: float = 1 / 30  # 30 Hz
    t_window_size_s: float = 10.0


DEFAULT_CONF_GUI = ConfGui()


@dataclass
class ConfStepResponse:
    ref_input: RefInput


@dataclass
class ConfGamepad:
    ref_input_scale: RefInput


class SubplotWidget(ABC):
    def __init__(self, data_cb: Callable):
        self._data_cb = data_cb
        self._base_widget = None

    def update(self):
        self._update(**self._data_cb())

    def get_base_widget(self):
        return self._base_widget

    @abstractmethod
    def _update(self, *args, **kwargs):
        pass


class SixDofWidget(SubplotWidget):
    def __init__(self, data_cb: Callable):
        """
        Note, as the axis in the 6dof model is rotated with
        pi about x, we set -y and -z.
        """
        super().__init__(data_cb)

        self._mesh = gl.GLMeshItem(meshdata=self.get_meshed_drone(), smooth=True, drawEdges=True, shader='balloon')

        self._base_widget = gl.GLViewWidget()
        self._base_widget.addItem(self._mesh)

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
        super().__init__(data_cb)

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


class Gui(QtGui.QMainWindow):
    def __init__(self, parent=None):
        super(Gui, self).__init__(parent)

        self._att_est_params = DEFAULT_ATT_EST_PARAMS
        self._pilot_ctrl_params = DEFAULT_PILOT_CTRL_PARAMS

        self._env_params = DEFAULT_ENV_PARAMS
        self._drone_params = DEFAULT_DRONE_PARAMS

        self._imu_noise = DEFAULT_IMU_NOISE
        self._conf_sim = DEFAULT_CONF_SIM
        self._conf_gui = DEFAULT_CONF_GUI

        self._conf_step_response = self._get_default_conf_step_response()
        self._conf_gamepad = self._get_default_conf_gamepad()

        self._setup_gui()

    def _setup_gui(self):
        def _conf_main_window():
            self.setWindowTitle('Non-linear 6dof simulator')
            self.setGeometry(100, 100, 1200, 800)

        def _create_grid():
            self._central_widget = QtWidgets.QWidget()
            self.setCentralWidget(self._central_widget)
            self._grid = QtWidgets.QGridLayout(self.centralWidget())

        def _init_default_subplot_widget_map():
            self._subplot_widget_map = {
                GridPos.TOP_LEFT: self._init_6dof_widget(),
                GridPos.TOP_RIGHT: self._init_roll_ref_widget(),
                GridPos.BOTTOM_RIGHT: self._init_pitch_ref_widget(),
                GridPos.BOTTOM_LEFT: self._init_yaw_rate_ref_widget(),
            }

        def _setup_and_place_widgets_in_grid():
            for grid_pos, widget in self._subplot_widget_map.items():
                self._place_widget_in_grid(widget=widget.get_base_widget(), grid_pos=grid_pos)

        def _setup_menu_and_actions():
            menuBar = self.menuBar()

            self._run_action = menuBar.addAction("&Run")
            self._run_action.triggered.connect(self._run)

            self._stop_action = menuBar.addAction("&Stop")
            self._stop_action.triggered.connect(self._stop)

            # Creating menus using a QMenu object
            # fileMenu = QMenu("&File", self)
            # menuBar.addMenu(fileMenu)
            # Creating menus using a title
            # editMenu = menuBar.addMenu("&Edit")

        _conf_main_window()
        _create_grid()
        _init_default_subplot_widget_map()
        _setup_and_place_widgets_in_grid()
        _setup_menu_and_actions()

        self._gui_state = GuiState.INIT

    def _get_default_conf_step_response(self):
        mg = self._drone_params.m * self._env_params.g

        return ConfStepResponse(ref_input=RefInput(f_z=-mg, roll=0.0, pitch=0.0, yaw_rate=0.0))

    def _get_default_conf_gamepad(self):
        mg = self._drone_params.m * self._env_params.g

        return ConfGamepad(ref_input_scale=RefInput(f_z=-2 * mg, roll=0.0, pitch=0.0, yaw_rate=0.0))

    def _run(self):
        if self._gui_state != GuiState.RUNNING:
            self._init_rolling_sim_data()
            self._init_simulator()
            self._run_sim_in_thread()
            self._run_gui_loop()

        self._gui_state = GuiState.RUNNING

    def _stop(self):
        if self._gui_state == GuiState.RUNNING:
            self._stop_sim_in_thread()
            self._stop_gui_loop()

        self._gui_state = GuiState.STOPPED

    def _run_gui_loop(self):
        self._gui_timer = QtCore.QTimer()
        self._gui_timer.timeout.connect(self._update_gui)
        self._gui_timer.start(int(self._conf_gui.refresh_rate_s * 1e3))

    def _stop_gui_loop(self):
        self._gui_timer.stop()

    def _place_widget_in_grid(self, widget: Union[pg.PlotWidget, gl.GLViewWidget], grid_pos: GridPos):
        if grid_pos == GridPos.TOP_LEFT:
            self._grid.addWidget(widget, 0, 0, 1, 1)

        elif grid_pos == GridPos.TOP_RIGHT:
            self._grid.addWidget(widget, 0, 1, 1, 1)

        elif grid_pos == GridPos.BOTTOM_LEFT:
            self._grid.addWidget(widget, 1, 0, 1, 1)

        elif grid_pos == GridPos.BOTTOM_RIGHT:
            self._grid.addWidget(widget, 1, 1, 1, 1)

        else:
            raise ValueError

    def _init_rolling_sim_data(self):
        self._rolling_sim_data = RollingSimData(
            n_samples=int(self._conf_gui.t_window_size_s / self._conf_gui.refresh_rate_s / 2))  # TODO: Why / 2?

    def _init_simulator(self):
        self._simulator = Simulator(
            att_est_params=self._att_est_params,
            pilot_ctrl_params=self._pilot_ctrl_params,
            six_dof_state=SIX_DOF_STATE_ZERO,
            drone_params=self._drone_params,
            env_params=self._env_params,
            imu_noise=self._imu_noise,
            dt=self._conf_sim.dt_s,
        )

    def _run_sim_in_thread(self):
        """
        Note, this approach is fine (w.r.t race conditions) as long as we
        only read from the simulator i.e., non-modifiable access.
        """
        self._threaded_task_exec_sim = ThreadedTask(cb=self._exec_simulator, exec_period_s=self._conf_sim.dt_s)
        self._threaded_task_exec_sim.launch()

    def _stop_sim_in_thread(self):
        self._threaded_task_exec_sim.teardown()

    def _exec_simulator(self):
        self._simulator.step(ref_input=self._conf_step_response.ref_input)

    def _update_gui(self):
        self._update_rolling_sim_data()
        self._update_subplots()

    def _update_rolling_sim_data(self):
        self._rolling_sim_data.update(
            t_s=self._simulator.get_t(),

            roll_ang=self._simulator.get_6dof_state().n_i[0],
            roll_rate=self._simulator.get_6dof_state().w_b[0],
            roll_acc=self._simulator.get_6dof_state().wp_b[0],

            pitch_ang=self._simulator.get_6dof_state().n_i[1],
            pitch_rate=self._simulator.get_6dof_state().w_b[1],
            pitch_acc=self._simulator.get_6dof_state().wp_b[0],

            yaw_ang=self._simulator.get_6dof_state().n_i[2],
            yaw_rate=self._simulator.get_6dof_state().w_b[2],
            yaw_acc=self._simulator.get_6dof_state().wp_b[0],

            est_roll_ang=self._simulator.get_att_estimate().roll.angle,
            est_pitch_ang=self._simulator.get_att_estimate().pitch.angle,
            est_yaw_ang=self._simulator.get_att_estimate().yaw.angle,

            est_roll_rate=self._simulator.get_att_estimate().roll.rate,
            est_pitch_rate=self._simulator.get_att_estimate().pitch.rate,
            est_yaw_rate=self._simulator.get_att_estimate().yaw.rate,

            est_roll_acc=self._simulator.get_att_estimate().roll.acc,
            est_pitch_acc=self._simulator.get_att_estimate().pitch.acc,
            est_yaw_acc=self._simulator.get_att_estimate().yaw.acc,

            ref_fz=self._conf_step_response.ref_input.f_z,
            ref_roll=self._conf_step_response.ref_input.roll,
            ref_pitch=self._conf_step_response.ref_input.pitch,
            ref_yaw_rate=self._conf_step_response.ref_input.yaw_rate,

            ctrl_fz=self._simulator.get_ctrl_input().f_z,
            ctrl_mx=self._simulator.get_ctrl_input().m_x,
            ctrl_my=self._simulator.get_ctrl_input().m_y,
            ctrl_mz=self._simulator.get_ctrl_input().m_z,
        )

    def _update_subplots(self):
        for widget in self._subplot_widget_map.values():
            widget.update()

    def _init_6dof_widget(self):
        return SixDofWidget(data_cb=self._cb_6dof_widget)

    def _init_roll_ref_widget(self):
        return AttRefWidget(data_cb=self._cb_roll_ref_widget,
                            title="Roll", y_label="Angle", y_unit="rad")

    def _init_pitch_ref_widget(self):
        return AttRefWidget(data_cb=self._cb_pitch_ref_widget,
                            title="Pitch", y_label="Angle", y_unit="rad")

    def _init_yaw_rate_ref_widget(self):
        return AttRefWidget(data_cb=self._cb_yaw_rate_ref_widget,
                            title="Yaw-rate", y_label="Angular-rate", y_unit="rad/s")

    def _cb_6dof_widget(self):
        return {"r_i": self._simulator.get_6dof_state().v_i,
                "q": self._simulator.get_6dof_state().q}

    def _cb_roll_ref_widget(self):
        return {"t_s": self._rolling_sim_data.t_s,
                "att_act": self._rolling_sim_data.roll_ang,
                "att_est": self._rolling_sim_data.est_roll_ang,
                "att_ref": self._rolling_sim_data.ref_roll}

    def _cb_pitch_ref_widget(self):
        return {"t_s": self._rolling_sim_data.t_s,
                "att_act": self._rolling_sim_data.pitch_ang,
                "att_est": self._rolling_sim_data.est_pitch_ang,
                "att_ref": self._rolling_sim_data.ref_pitch}

    def _cb_yaw_rate_ref_widget(self):
        return {"t_s": self._rolling_sim_data.t_s,
                "att_act": self._rolling_sim_data.yaw_rate,
                "att_est": self._rolling_sim_data.est_yaw_rate,
                "att_ref": self._rolling_sim_data.ref_yaw_rate}

    def closeEvent(self, event):
        self._stop()


def main():
    qt_app = QtGui.QApplication(sys.argv)
    gui = Gui()
    gui.show()
    sys.exit(qt_app.exec_())


if __name__ == '__main__':
    main()
