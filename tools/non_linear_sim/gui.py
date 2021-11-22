import sys
from dataclasses import dataclass, is_dataclass
from enum import Enum
from functools import partial
from typing import Union

import pyqtgraph as pg
import pyqtgraph.opengl as gl
from PyQt5.QtWidgets import QInputDialog
from pyqtgraph.Qt import QtGui, QtCore
from pyqtgraph.Qt import QtWidgets

from non_linear_sim.att_estimator import DEFAULT_ATT_EST_PARAMS
from non_linear_sim.drone_model import DEFAULT_DRONE_PARAMS, DEFAULT_ENV_PARAMS
from non_linear_sim.pilot_ctrl import DEFAULT_PILOT_CTRL_PARAMS, RefInput
from non_linear_sim.rolling_buffer import RollingBuffer
from non_linear_sim.simulator import DEFAULT_IMU_NOISE
from non_linear_sim.simulator import Simulator
from non_linear_sim.six_dof_model import STATE_ZERO as SIX_DOF_STATE_ZERO
from non_linear_sim.subplot_widgets import SixDofWidget, AttRefWidget, BodyCtrlWidget, MotorCtrlWidget
from non_linear_sim.threaded_task import ThreadedTask

# TODO: Add menus for selection of input.
# TODO: Gamepad logic class and refactor with ref step.
# TODO: Subplots to add: imu_out, pos and velocity.
# TODO: Due to performance reasons we need a configureable supblot menu.... Let it be a 2x2 grid for performance. We
#  need to update all the plots in the background but not show.

FORMAT_MENU_LABEL = "{key}: {val}"


class SubplotId(Enum):
    SIX_DOF = 'six_dof'

    ROLL_REF = 'roll_ref'
    PITCH_REF = 'pitch_ref'
    YAW_RATE_REF = 'yaw_rate_ref'

    BODY_CTRL = 'body_ctrl'
    MOTOR_CTRL = 'motor_ctrl'

    IMU_ACC = 'imu_acc'
    IMU_GYRO = 'imu_gyro'
    IMU_MAG = 'imu_mag'


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
                GridPos.BOTTOM_LEFT: self._init_pitch_ref_widget(),
                GridPos.BOTTOM_RIGHT: self._init_yaw_rate_ref_widget(),
            }

        def _setup_and_place_widgets_in_grid():
            for grid_pos, subplot in self._subplot_widget_map.items():
                self._place_widget_in_grid(grid_pos, subplot.get_base_widget())

        def _setup_menu_and_actions():
            self._menu = self.menuBar()

            _setup_menu_conf()
            _setup_action_run()
            _setup_action_stop()

        def _setup_menu_conf():
            self._menu_conf = self._menu.addMenu("Config")
            self._menu_conf_model = self._menu_conf.addMenu("Model")
            self._menu_conf_input = self._menu_conf.addMenu("Input")

            _setup_menu_config_model()
            _setup_menu_conf_att_est()
            _setup_menu_config_pilot_ctrl()
            _setup_menu_config_imu_noise()
            _setup_menu_config_gui()
            _setup_menu_config_sim()
            _setup_menu_config_input()

        def _setup_menu_config_model():
            self._add_menu_from_dataclass(self._menu_conf_model, self._env_params, label="Env")
            self._add_menu_from_dataclass(self._menu_conf_model, self._drone_params, label="Drone")

        def _setup_menu_conf_att_est():
            self._add_menu_from_dataclass(self._menu_conf, self._att_est_params, label="Attitude estimator")

        def _setup_menu_config_pilot_ctrl():
            self._add_menu_from_dataclass(self._menu_conf, self._pilot_ctrl_params, label="Pilot controller")

        def _setup_menu_config_imu_noise():
            self._add_menu_from_dataclass(self._menu_conf, self._imu_noise, label="Imu noise")

        def _setup_menu_config_sim():
            self._add_menu_from_dataclass(self._menu_conf, self._conf_sim, label="Simulation")

        def _setup_menu_config_gui():
            self._add_menu_from_dataclass(self._menu_conf, self._conf_gui, label="Gui")

        def _setup_menu_config_input():
            self._add_menu_from_dataclass(self._menu_conf_input, self._conf_step_response, label="Step response")
            self._add_menu_from_dataclass(self._menu_conf_input, self._conf_gamepad, label="Gamepad")

        def _setup_action_run():
            self._action_run = self._menu.addAction("&Run")
            self._action_run.triggered.connect(self._run)

        def _setup_action_stop():
            self._action_stop = self._menu.addAction("&Stop")
            self._action_stop.triggered.connect(self._stop)

        _conf_main_window()
        _create_grid()
        _init_default_subplot_widget_map()
        _setup_and_place_widgets_in_grid()
        _setup_menu_and_actions()

        self._gui_state = GuiState.INIT

    def _add_menu_from_dataclass(self, menu_parent, data_class, label):
        menu_dataclass = menu_parent.addMenu(label)

        for idx, member in enumerate(data_class.__annotations__):
            val = getattr(data_class, member)

            if is_dataclass(val):
                self._add_menu_from_dataclass(menu_dataclass, val, member)
            else:
                action = menu_dataclass.addAction(self._format_menu_label(key=member, val=val))
                action.triggered.connect(partial(self._action_dataclass_cb, action, data_class, member))

    def _get_default_conf_step_response(self):
        mg = self._drone_params.m * self._env_params.g

        return ConfStepResponse(ref_input=RefInput(f_z=-mg, roll=0.0, pitch=0.0, yaw_rate=0.0))

    def _get_default_conf_gamepad(self):
        mg = self._drone_params.m * self._env_params.g

        return ConfGamepad(ref_input_scale=RefInput(f_z=-2 * mg, roll=0.0, pitch=0.0, yaw_rate=0.0))

    def _run(self):
        if self._gui_state != GuiState.RUNNING:
            self._init_simulator()
            self._init_rolling_sim_buf()
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

    def _place_widget_in_grid(self, grid_pos: GridPos, widget: Union[pg.PlotWidget, gl.GLViewWidget]):
        if grid_pos == GridPos.TOP_LEFT:
            self._grid.addWidget(widget, 0, 0, 1, 1)

        if grid_pos == GridPos.TOP_RIGHT:
            self._grid.addWidget(widget, 0, 1, 1, 1)

        if grid_pos == GridPos.BOTTOM_LEFT:
            self._grid.addWidget(widget, 1, 0, 1, 1)

        if grid_pos == GridPos.BOTTOM_RIGHT:
            self._grid.addWidget(widget, 1, 1, 1, 1)

    def _init_rolling_sim_buf(self):
        self._rolling_sim_buf = RollingBuffer(
            member_cb_map={
                "t_s": self._simulator.get_t,

                "roll_ang": lambda: self._simulator.get_6dof_state().n_i[0],
                "roll_rate": lambda: self._simulator.get_6dof_state().w_b[0],
                "roll_acc": lambda: self._simulator.get_6dof_state().wp_b[0],

                "pitch_ang": lambda: self._simulator.get_6dof_state().n_i[1],
                "pitch_rate": lambda: self._simulator.get_6dof_state().w_b[1],
                "pitch_acc": lambda: self._simulator.get_6dof_state().wp_b[0],

                "yaw_ang": lambda: self._simulator.get_6dof_state().n_i[2],
                "yaw_rate": lambda: self._simulator.get_6dof_state().w_b[2],
                "yaw_acc": lambda: self._simulator.get_6dof_state().wp_b[0],

                "ref_input": lambda: self._conf_step_response.ref_input,
                "ctrl_input": self._simulator.get_ctrl_input,

                "att_est": self._simulator.get_att_estimate,
                "imu_out": self._simulator.get_imu_out,

                "motor_ang_rates": self._simulator.get_motor_ang_rates,
            },
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
        self._rolling_sim_buf.update()
        self._update_subplots()

    def _update_subplots(self):
        for widget in self._subplot_widget_map.values():
            widget.update()

    def _init_6dof_widget(self):
        return SixDofWidget(data_cb=self._cb_6dof_widget)

    def _init_roll_ref_widget(self):
        return AttRefWidget(data_cb=self._cb_roll_ref_widget, y_label="Roll", y_unit="rad")

    def _init_pitch_ref_widget(self):
        return AttRefWidget(data_cb=self._cb_pitch_ref_widget, y_label="Pitch", y_unit="rad")

    def _init_yaw_rate_ref_widget(self):
        return AttRefWidget(data_cb=self._cb_yaw_rate_ref_widget, y_label="Yaw-rate", y_unit="rad/s")

    def _init_body_ctrl_widget(self):
        return BodyCtrlWidget(data_cb=self._cb_body_ctrl_widget)

    def _init_motor_ctrl_widget(self):
        return MotorCtrlWidget(data_cb=self._cb_motor_ctrl_widget)

    def _cb_6dof_widget(self):
        return {"r_i": self._simulator.get_6dof_state().r_i,
                "q": self._simulator.get_6dof_state().q}

    def _cb_roll_ref_widget(self):
        return {"t_s": self._rolling_sim_buf.t_s,
                "att_act": self._rolling_sim_buf.roll_ang,
                "att_est": self._rolling_sim_buf.att_est.roll.angle,
                "att_ref": self._rolling_sim_buf.ref_input.roll}

    def _cb_pitch_ref_widget(self):
        return {"t_s": self._rolling_sim_buf.t_s,
                "att_act": self._rolling_sim_buf.pitch_ang,
                "att_est": self._rolling_sim_buf.att_est.pitch.angle,
                "att_ref": self._rolling_sim_buf.ref_input.pitch}

    def _cb_yaw_rate_ref_widget(self):
        return {"t_s": self._rolling_sim_buf.t_s,
                "att_act": self._rolling_sim_buf.yaw_rate,
                "att_est": self._rolling_sim_buf.att_est.yaw.rate,
                "att_ref": self._rolling_sim_buf.ref_input.yaw_rate}

    def _cb_body_ctrl_widget(self):
        return {"t_s": self._rolling_sim_buf.t_s,
                "ctrl_mx": self._rolling_sim_buf.ctrl_input.m_x,
                "ctrl_my": self._rolling_sim_buf.ctrl_input.m_y,
                "ctrl_mz": self._rolling_sim_buf.ctrl_input.m_z}

    def _cb_motor_ctrl_widget(self):
        return {"t_s": self._rolling_sim_buf.t_s,
                "w_0": self._rolling_sim_buf.motor_ang_rates.w_0,
                "w_1": self._rolling_sim_buf.motor_ang_rates.w_1,
                "w_2": self._rolling_sim_buf.motor_ang_rates.w_2,
                "w_3": self._rolling_sim_buf.motor_ang_rates.w_3}

    def closeEvent(self, event):
        self._stop()

    def _action_dataclass_cb(self, action, data_class, member):
        old_val = getattr(data_class, member)

        if isinstance(old_val, int):
            new_val, is_updated = QInputDialog.getInt(self, member, 'Update value:', value=old_val)

        elif isinstance(old_val, float):
            new_val, is_updated = QInputDialog.getDouble(self, member, 'Update value:', value=old_val, decimals=5)

        else:
            raise NotImplementedError

        if is_updated:
            setattr(data_class, member, new_val)
            action.setText(self._format_menu_label(key=member, val=new_val))

    @staticmethod
    def _format_menu_label(key: str, val=Union[bool, int, float]) -> str:
        if isinstance(val, int):
            return FORMAT_MENU_LABEL.format(key=key, val=int(val))

        elif isinstance(val, float):
            return FORMAT_MENU_LABEL.format(key=key, val=float(val))

        else:
            raise NotImplementedError


def main():
    qt_app = QtGui.QApplication(sys.argv)
    gui = Gui()
    gui.show()
    sys.exit(qt_app.exec_())


if __name__ == '__main__':
    main()
