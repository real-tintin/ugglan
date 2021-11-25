import sys
from dataclasses import dataclass, is_dataclass
from enum import Enum
from functools import partial

from PyQt5.QtWidgets import QInputDialog, QStackedWidget
from pyqtgraph.Qt import QtGui, QtCore
from pyqtgraph.Qt import QtWidgets

from non_linear_sim.att_estimator import DEFAULT_ATT_EST_PARAMS
from non_linear_sim.drone_model import DEFAULT_DRONE_PARAMS, DEFAULT_ENV_PARAMS
from non_linear_sim.pilot_ctrl import DEFAULT_PILOT_CTRL_PARAMS, RefInput
from non_linear_sim.rolling_buffer import RollingBuffer
from non_linear_sim.simulator import DEFAULT_IMU_NOISE
from non_linear_sim.simulator import Simulator
from non_linear_sim.six_dof_model import STATE_ZERO as SIX_DOF_STATE_ZERO
from non_linear_sim.subplot_widgets import SixDofWidget, LinePlotWidget
from non_linear_sim.threaded_task import ThreadedTask

# TODO: Gamepad logic class and use ref in sim.


FORMAT_MENU_LABEL = "{key}: {val}"


class SubplotId(Enum):
    SIX_DOF = '6dof'

    ROLL_REF = 'Roll ref'
    PITCH_REF = 'Pitch ref'
    YAW_RATE_REF = 'Yaw-rate ref'

    BODY_CTRL = 'Body ctrl'
    MOTOR_CTRL = 'Motor ctrl'

    IMU_ACC = 'Imu acc'
    IMU_GYRO = 'Imu gyro'
    IMU_MAG = 'Imu mag'


class GridPos(Enum):
    TOP_LEFT = 'Top left'
    TOP_RIGHT = 'Top right'
    BOTTOM_LEFT = 'Bottom left'
    BOTTOM_RIGHT = 'Bottom right'


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
        def conf_main_window():
            self.setWindowTitle('Non-linear 6dof simulator')
            self.setGeometry(100, 100, 1200, 800)

        def create_grid():
            self._central_widget = QtWidgets.QWidget()
            self.setCentralWidget(self._central_widget)

            self._grid = QtWidgets.QGridLayout(self.centralWidget())
            self._grid_stack_map = {
                GridPos.TOP_LEFT: QStackedWidget(),
                GridPos.TOP_RIGHT: QStackedWidget(),
                GridPos.BOTTOM_LEFT: QStackedWidget(),
                GridPos.BOTTOM_RIGHT: QStackedWidget(),
            }

            self._grid.addWidget(self._grid_stack_map[GridPos.TOP_LEFT], 0, 0, 1, 1)
            self._grid.addWidget(self._grid_stack_map[GridPos.TOP_RIGHT], 0, 1, 1, 1)
            self._grid.addWidget(self._grid_stack_map[GridPos.BOTTOM_LEFT], 1, 0, 1, 1)
            self._grid.addWidget(self._grid_stack_map[GridPos.BOTTOM_RIGHT], 1, 1, 1, 1)

        def init_subplot_widget_map():
            self._subplot_widget_map = {
                SubplotId.SIX_DOF: self._init_6dof_widget(),
                SubplotId.ROLL_REF: self._init_roll_ref_widget(),
                SubplotId.PITCH_REF: self._init_pitch_ref_widget(),
                SubplotId.YAW_RATE_REF: self._init_yaw_rate_ref_widget(),
                SubplotId.BODY_CTRL: self._init_body_ctrl_widget(),
                SubplotId.MOTOR_CTRL: self._init_motor_ctrl_widget(),
                SubplotId.IMU_ACC: self._init_imu_acc_widget(),
                SubplotId.IMU_GYRO: self._init_imu_gyro_widget(),
                SubplotId.IMU_MAG: self._init_imu_mag_widget(),
            }

        def init_default_grid_subplot_map():
            self._grid_subplot_map = {
                GridPos.TOP_LEFT: SubplotId.SIX_DOF,
                GridPos.TOP_RIGHT: SubplotId.ROLL_REF,
                GridPos.BOTTOM_LEFT: SubplotId.PITCH_REF,
                GridPos.BOTTOM_RIGHT: SubplotId.YAW_RATE_REF,
            }

        def setup_menu_and_actions():
            self._menu = self.menuBar()

            setup_menu_conf()
            setup_menu_plots()
            setup_menu_input()
            setup_action_run()
            setup_action_stop()

        def setup_menu_conf():
            self._menu_conf = self._menu.addMenu("Config")
            self._menu_conf_model = self._menu_conf.addMenu("Model")
            self._menu_conf_input = self._menu_conf.addMenu("Input")

            setup_menu_config_model()
            setup_menu_conf_att_est()
            setup_menu_config_pilot_ctrl()
            setup_menu_config_imu_noise()
            setup_menu_config_gui()
            setup_menu_config_sim()
            setup_menu_config_input()

        def setup_menu_config_model():
            self._add_menu_from_dataclass(self._menu_conf_model, self._env_params, label="Env")
            self._add_menu_from_dataclass(self._menu_conf_model, self._drone_params, label="Drone")

        def setup_menu_conf_att_est():
            self._add_menu_from_dataclass(self._menu_conf, self._att_est_params, label="Attitude estimator")

        def setup_menu_config_pilot_ctrl():
            self._add_menu_from_dataclass(self._menu_conf, self._pilot_ctrl_params, label="Pilot controller")

        def setup_menu_config_imu_noise():
            self._add_menu_from_dataclass(self._menu_conf, self._imu_noise, label="Imu noise")

        def setup_menu_config_sim():
            self._add_menu_from_dataclass(self._menu_conf, self._conf_sim, label="Simulation")

        def setup_menu_config_gui():
            self._add_menu_from_dataclass(self._menu_conf, self._conf_gui, label="Gui")

        def setup_menu_config_input():
            self._add_menu_from_dataclass(self._menu_conf_input, self._conf_step_response, label="Step response")
            self._add_menu_from_dataclass(self._menu_conf_input, self._conf_gamepad, label="Gamepad")

        def setup_menu_plots():
            self._menu_plots = self._menu.addMenu("Plots")
            self._grid_subplot_action_map = {}

            for grid_pos in GridPos:
                menu_grid = self._menu_plots.addMenu(grid_pos.value)
                subplot_action_map = {}

                for subplot_id in SubplotId:
                    action_subplot = menu_grid.addAction(subplot_id.value)
                    action_subplot.setCheckable(True)

                    if subplot_id == self._grid_subplot_map[grid_pos]:
                        action_subplot.setChecked(True)

                    action_subplot.triggered.connect(partial(self._cb_action_grid_subplot, grid_pos, subplot_id))
                    subplot_action_map.update({subplot_id: action_subplot})

                self._grid_subplot_action_map.update({grid_pos: subplot_action_map})

        def setup_menu_input():
            self._menu_input = self._menu.addMenu("Input")

            self._action_input_step = self._menu_input.addAction("Step response")
            self._action_input_step.setCheckable(True)
            self._action_input_step.setChecked(True)
            self._action_input_step.triggered.connect(self._cb_input_step_checked)

            self._action_input_gamepad = self._menu_input.addAction("Gamepad")
            self._action_input_gamepad.setCheckable(True)
            self._action_input_gamepad.setChecked(False)
            self._action_input_gamepad.triggered.connect(self._cb_input_gamepad_checked)

        def setup_action_run():
            self._action_run = self._menu.addAction("Run")
            self._action_run.triggered.connect(self._run)

        def setup_action_stop():
            self._action_stop = self._menu.addAction("Stop")
            self._action_stop.triggered.connect(self._stop)

        conf_main_window()
        create_grid()
        init_subplot_widget_map()
        init_default_grid_subplot_map()
        setup_menu_and_actions()
        self._update_subplots_in_grid()

        self._gui_state = GuiState.INIT

    def _update_subplots_in_grid(self):
        for grid_pos, subplot_id in self._grid_subplot_map.items():
            subplot_widget = self._subplot_widget_map[subplot_id].get_base_widget()

            if self._grid_stack_map[grid_pos].indexOf(subplot_widget) < 0:
                self._grid_stack_map[grid_pos].addWidget(subplot_widget)

            self._grid_stack_map[grid_pos].setCurrentWidget(subplot_widget)

    def _cb_action_grid_subplot(self, grid_pos, new_subplot_id):
        old_subplot_id = self._set_new_subplot_id_and_get_old(grid_pos, new_subplot_id)
        self._if_new_subplot_in_other_grid_swap_with_old_subplot(grid_pos, new_subplot_id, old_subplot_id)
        self._check_and_uncheck_all_subplot_actions()
        self._update_subplots_in_grid()

    def _set_new_subplot_id_and_get_old(self, grid_pos, new_subplot_id):
        subplot_id_action_map = self._grid_subplot_action_map[grid_pos]

        for subplot_id, action in subplot_id_action_map.items():
            if action.isChecked() and subplot_id == new_subplot_id:
                old_subplot_id = self._grid_subplot_map[grid_pos]
                self._grid_subplot_map[grid_pos] = new_subplot_id

        return old_subplot_id

    def _if_new_subplot_in_other_grid_swap_with_old_subplot(self, grid_pos, new_subplot_id, old_subplot_id):
        for other_grid_pos in (set(GridPos) - set([grid_pos])):
            for subplot_id, action in self._grid_subplot_action_map[other_grid_pos].items():
                if action.isChecked() and subplot_id == new_subplot_id:
                    self._grid_subplot_map[other_grid_pos] = old_subplot_id

    def _check_and_uncheck_all_subplot_actions(self):
        for grid_pos in list(GridPos):
            for subplot_id, action in self._grid_subplot_action_map[grid_pos].items():
                if subplot_id == self._grid_subplot_map[grid_pos]:
                    action.setChecked(True)
                else:
                    action.setChecked(False)

    def _add_menu_from_dataclass(self, menu_parent, data_class, label):
        menu_dataclass = menu_parent.addMenu(label)

        for idx, member in enumerate(data_class.__annotations__):
            val = getattr(data_class, member)

            if is_dataclass(val):
                self._add_menu_from_dataclass(menu_dataclass, val, member)
            else:
                action = menu_dataclass.addAction(FORMAT_MENU_LABEL.format(key=member, val=val))
                action.triggered.connect(partial(self._action_dataclass_cb, action, data_class, member))

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
            action.setText(FORMAT_MENU_LABEL.format(key=member, val=new_val))

    def _cb_input_step_checked(self):
        self._action_input_step.setChecked(True)
        self._action_input_gamepad.setChecked(False)

    def _cb_input_gamepad_checked(self):
        self._action_input_step.setChecked(False)
        self._action_input_gamepad.setChecked(True)

    def _is_input_step_selected(self):
        return self._action_input_step.isChecked()

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

    def _run_sim_in_thread(self):
        """
        Note, this approach is fine (w.r.t race conditions) as long as we
        only read from the simulator i.e., non-modifiable access.
        """
        self._threaded_task_exec_sim = ThreadedTask(cb=self._exec_simulator, exec_period_s=self._conf_sim.dt_s)
        self._threaded_task_exec_sim.launch()

    def _exec_simulator(self):
        if self._is_input_step_selected():
            self._simulator.step(ref_input=self._conf_step_response.ref_input)
        else:
            raise NotImplementedError

    def _run_gui_loop(self):
        self._gui_timer = QtCore.QTimer()
        self._gui_timer.timeout.connect(self._update_gui)
        self._gui_timer.start(int(self._conf_gui.refresh_rate_s * 1e3))

    def _update_gui(self):
        self._rolling_sim_buf.update()
        self._update_subplots()

    def _update_subplots(self):
        for subplot in self._subplot_widget_map.values():
            subplot.update()

    def _stop(self):
        if self._gui_state == GuiState.RUNNING:
            self._stop_sim_in_thread()
            self._stop_gui_loop()

        self._gui_state = GuiState.STOPPED

    def _stop_sim_in_thread(self):
        self._threaded_task_exec_sim.teardown()

    def _stop_gui_loop(self):
        self._gui_timer.stop()

    def _init_6dof_widget(self):
        return SixDofWidget(data_cb=self._cb_6dof_widget)

    def _init_roll_ref_widget(self):
        return LinePlotWidget(data_cb=self._cb_roll_ref_widget,
                              labels=["act", "est", "ref"],
                              line_styles=[QtCore.Qt.SolidLine, QtCore.Qt.SolidLine, QtCore.Qt.DashLine],
                              y_label="Roll", y_unit="rad")

    def _init_pitch_ref_widget(self):
        return LinePlotWidget(data_cb=self._cb_pitch_ref_widget,
                              labels=["act", "est", "ref"],
                              line_styles=[QtCore.Qt.SolidLine, QtCore.Qt.SolidLine, QtCore.Qt.DashLine],
                              y_label="Pitch", y_unit="rad")

    def _init_yaw_rate_ref_widget(self):
        return LinePlotWidget(data_cb=self._cb_yaw_rate_ref_widget,
                              labels=["act", "est", "ref"],
                              line_styles=[QtCore.Qt.SolidLine, QtCore.Qt.SolidLine, QtCore.Qt.DashLine],
                              y_label="Yaw-rate", y_unit="rad/s")

    def _init_body_ctrl_widget(self):
        return LinePlotWidget(data_cb=self._cb_body_ctrl_widget,
                              labels=["m<sub>x</sub>", "m<sub>y</sub>", "m<sub>z</sub>"],
                              y_label="Torque", y_unit="Nm")

    def _init_motor_ctrl_widget(self):
        return LinePlotWidget(data_cb=self._cb_motor_ctrl_widget,
                              labels=["\u03C9<sub>0</sub>", "\u03C9<sub>1</sub>",
                                      "\u03C9<sub>2</sub>", "\u03C9<sub>3</sub>"],
                              y_label="Angular-rate", y_unit="rad/s")

    def _init_imu_acc_widget(self):
        return LinePlotWidget(data_cb=self._cb_imu_acc_widget,
                              labels=["x", "y", "z"],
                              y_label="Imu accelerometer", y_unit="m/s<sub>2</sub>")

    def _init_imu_gyro_widget(self):
        return LinePlotWidget(data_cb=self._cb_imu_gyro_widget,
                              labels=["x", "y", "z"],
                              y_label="Imu gyro", y_unit="rad/s")

    def _init_imu_mag_widget(self):
        return LinePlotWidget(data_cb=self._cb_imu_mag_widget,
                              labels=["x", "y", "z"],
                              y_label="Imu magnetometer", y_unit="gauss")

    def _cb_6dof_widget(self):
        return {"r_i": self._simulator.get_6dof_state().r_i,
                "v_i": self._simulator.get_6dof_state().v_i,
                "a_i": self._simulator.get_6dof_state().a_i,
                "q": self._simulator.get_6dof_state().q}

    def _cb_roll_ref_widget(self):
        return {"t_s": self._rolling_sim_buf.t_s,
                "y": [self._rolling_sim_buf.roll_ang,
                      self._rolling_sim_buf.att_est.roll.angle,
                      self._rolling_sim_buf.ref_input.roll]}

    def _cb_pitch_ref_widget(self):
        return {"t_s": self._rolling_sim_buf.t_s,
                "y": [self._rolling_sim_buf.pitch_ang,
                      self._rolling_sim_buf.att_est.pitch.angle,
                      self._rolling_sim_buf.ref_input.pitch]}

    def _cb_yaw_rate_ref_widget(self):
        return {"t_s": self._rolling_sim_buf.t_s,
                "y": [self._rolling_sim_buf.yaw_rate,
                      self._rolling_sim_buf.att_est.yaw.rate,
                      self._rolling_sim_buf.ref_input.yaw_rate]}

    def _cb_body_ctrl_widget(self):
        return {"t_s": self._rolling_sim_buf.t_s,
                "y": [self._rolling_sim_buf.ctrl_input.m_x,
                      self._rolling_sim_buf.ctrl_input.m_y,
                      self._rolling_sim_buf.ctrl_input.m_z]}

    def _cb_motor_ctrl_widget(self):
        return {"t_s": self._rolling_sim_buf.t_s,
                "y": [self._rolling_sim_buf.motor_ang_rates.w_0,
                      self._rolling_sim_buf.motor_ang_rates.w_1,
                      self._rolling_sim_buf.motor_ang_rates.w_2,
                      self._rolling_sim_buf.motor_ang_rates.w_3]}

    def _cb_imu_acc_widget(self):
        return {"t_s": self._rolling_sim_buf.t_s,
                "y": [self._rolling_sim_buf.imu_out.acc_x,
                      self._rolling_sim_buf.imu_out.acc_y,
                      self._rolling_sim_buf.imu_out.acc_z]}

    def _cb_imu_gyro_widget(self):
        return {"t_s": self._rolling_sim_buf.t_s,
                "y": [self._rolling_sim_buf.imu_out.ang_rate_x,
                      self._rolling_sim_buf.imu_out.ang_rate_y,
                      self._rolling_sim_buf.imu_out.ang_rate_z]}

    def _cb_imu_mag_widget(self):
        return {"t_s": self._rolling_sim_buf.t_s,
                "y": [self._rolling_sim_buf.imu_out.mag_field_x,
                      self._rolling_sim_buf.imu_out.mag_field_y,
                      self._rolling_sim_buf.imu_out.mag_field_z]}

    def closeEvent(self, event):
        self._stop()


def main():
    qt_app = QtGui.QApplication(sys.argv)
    gui = Gui()
    gui.show()
    sys.exit(qt_app.exec_())


if __name__ == '__main__':
    main()
