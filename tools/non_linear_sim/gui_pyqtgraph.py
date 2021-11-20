import sys
from abc import ABC, abstractmethod
from copy import copy
from dataclasses import dataclass
from enum import Enum
from typing import Union, Callable

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtGui, QtWidgets, QtCore

from non_linear_sim.att_estimator import DEFAULT_ATT_EST_PARAMS
from non_linear_sim.drone_model import DEFAULT_DRONE_PARAMS, DEFAULT_ENV_PARAMS
from non_linear_sim.pilot_ctrl import DEFAULT_PILOT_CTRL_PARAMS, RefInput
from non_linear_sim.rolling_sim_data import RollingSimData
from non_linear_sim.simulator import DEFAULT_IMU_NOISE
from non_linear_sim.simulator import Simulator
from non_linear_sim.six_dof_model import STATE_ZERO as SIX_DOF_STATE_ZERO
from non_linear_sim.threaded_task import ThreadedTask

# TODO: Some method to update simulation model, menu, buttons etc.
# TODO: Select subplot with breakout?
# TODO: Use MG for -fz as default value.
# TODO: Gamepad logic class and refactor with ref step.

GUI_REFRESH_RATE_S = 1 / 30  # 30 Hz
N_SUBPLOTS = 3

DEFAULT_TIME_WINDOW_SIZE_S = 10.0


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


@dataclass
class SimParams:
    dt_s: float = 0.01
    standstill_calib_att_est: bool = True


DEFAULT_SIM_PARAMS = SimParams()


@dataclass
class ConfigStepResponse:
    t_end_s: float = 10.0
    ref_input: RefInput = RefInput(f_z=-11.0, roll=0.0, pitch=0.0, yaw_rate=0.0)


DEFAULT_CONFIG_STEP_RESPONSE = ConfigStepResponse()


@dataclass
class ConfigGamepad:
    t_window_s: float = 10.0
    ref_input_scale: RefInput = RefInput(f_z=-12.0, roll=np.pi / 4, pitch=np.pi / 4, yaw_rate=np.pi)


DEFAULT_CONFIG_GAMEPAD = ConfigGamepad()


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
        super().__init__(data_cb)

        # TODO: Should be a 3d-drone object.
        self._scatter = gl.GLScatterPlotItem(pos=np.zeros(3))
        self._grid = gl.GLGridItem(size=QtGui.QVector3D(20, 20, 10))
        self._axis = gl.GLAxisItem()

        self._base_widget = gl.GLViewWidget()
        self._base_widget.addItem(self._scatter)
        self._base_widget.addItem(self._grid)
        self._base_widget.addItem(self._axis)

    def _update(self, pos: np.ndarray, angle: np.ndarray):
        self._scatter.setData(pos=self._flip_z_axis(pos))

    @staticmethod
    def _flip_z_axis(pos):
        pos_negative_z = copy(pos)
        pos_negative_z[2] *= -1  # Can it be done via the GLGridItem instead?

        return pos_negative_z


class AttRefWidget(SubplotWidget):
    def __init__(self, data_cb: Callable, y_label: str, y_unit: str):
        super().__init__(data_cb)

        self._base_widget = pg.PlotWidget()
        self._base_widget.setLabel('bottom', 'Time', units='s')
        self._base_widget.setLabel('left', y_label, units=y_unit)
        self._base_widget.addLegend()

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
        self._sim_params = DEFAULT_SIM_PARAMS

        self._config_step_response = DEFAULT_CONFIG_STEP_RESPONSE
        self._config_gamepad = DEFAULT_CONFIG_GAMEPAD

        self._rolling_sim_data = RollingSimData(
            n_samples=int(DEFAULT_TIME_WINDOW_SIZE_S / GUI_REFRESH_RATE_S / 2))  # TODO: Why / 2?

        self._init_simulator()
        self._setup_gui()
        self._launch_sim_in_thread()
        self._start_gui()

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

        _conf_main_window()
        _create_grid()
        _init_default_subplot_widget_map()
        _setup_and_place_widgets_in_grid()

    def _start_gui(self):
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._update_gui)
        self.timer.start(int(GUI_REFRESH_RATE_S * 1e3))

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

    def _init_simulator(self):
        self._simulator = Simulator(
            att_est_params=self._att_est_params,
            pilot_ctrl_params=self._pilot_ctrl_params,
            six_dof_state=SIX_DOF_STATE_ZERO,
            drone_params=self._drone_params,
            env_params=self._env_params,
            imu_noise=self._imu_noise,
            dt=self._sim_params.dt_s,
        )

    def _launch_sim_in_thread(self):
        """
        Note, this approach is fine (w.r.t race conditions) as long as we
        only read from the simulator i.e., non-modifiable access.
        """
        self._threaded_task_exec_sim = ThreadedTask(cb=self._exec_simulator, exec_period_s=self._sim_params.dt_s)
        self._threaded_task_exec_sim.launch()

    def _exec_simulator(self):
        self._simulator.step(ref_input=self._config_step_response.ref_input)

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

            ref_fz=self._config_step_response.ref_input.f_z,
            ref_roll=self._config_step_response.ref_input.roll,
            ref_pitch=self._config_step_response.ref_input.pitch,
            ref_yaw_rate=self._config_step_response.ref_input.yaw_rate,

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
                            y_label="roll", y_unit="rad")

    def _init_pitch_ref_widget(self):
        return AttRefWidget(data_cb=self._cb_pitch_ref_widget,
                            y_label="pitch", y_unit="rad")

    def _init_yaw_rate_ref_widget(self):
        return AttRefWidget(data_cb=self._cb_yaw_rate_ref_widget,
                            y_label="yaw-rate", y_unit="rad/s")

    def _cb_6dof_widget(self):
        return {"pos": self._simulator.get_6dof_state().r_i,
                "angle": self._simulator.get_6dof_state().n_i}

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
        self._threaded_task_exec_sim.teardown()


def main():
    qt_app = QtGui.QApplication(sys.argv)
    gui = Gui()
    gui.show()
    sys.exit(qt_app.exec_())


if __name__ == '__main__':
    main()
