import sys
from dataclasses import dataclass
from enum import Enum

from pyqtgraph.Qt import QtGui, QtCore
from pyqtgraph.Qt import QtWidgets

from non_linear_sim.att_estimator import DEFAULT_ATT_EST_PARAMS
from non_linear_sim.drone_model import DEFAULT_DRONE_PARAMS, DEFAULT_ENV_PARAMS
from non_linear_sim.pilot_ctrl import DEFAULT_PILOT_CTRL_PARAMS, RefInput
from non_linear_sim.rolling_buffer import RollingBuffer
from non_linear_sim.simulator import DEFAULT_IMU_NOISE
from non_linear_sim.simulator import Simulator
from non_linear_sim.six_dof_model import STATE_ZERO as SIX_DOF_STATE_ZERO
from non_linear_sim.subplot_widgets import SubplotWidget, SixDofWidget, AttRefWidget
from non_linear_sim.threaded_task import ThreadedTask


# TODO: Add menus for update of all conf and params.
# TODO: Gamepad logic class and refactor with ref step.
# TODO: Subplots to add: Pilotctrl, att_est, motor_ang_rate, imu_out. How many subplots would i need?
# 6dof, 3 x att_ctrl, pilot_ctrl, 3 x imu_out, motor_ang, (3 x att_est?), Maybe combine 3 x att_est: re?
# Maybe have different sizes? The imu is not sooo important all the time? Maybe split into

class SubplotId(Enum):
    SIX_DOF = 'six_dof'

    ROLL_REF = 'roll_ref'
    PITCH_REF = 'pitch_ref'
    YAW_RATE_REF = 'yaw_rate_ref'

    PILOT_CTRL = 'pilot_ctrl'
    ATT_EST = 'att_estimation'

    MOTOR_ANG_RATES = 'motor_ang_rates'
    IMU_OUT = 'imu_out'


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
                SubplotId.SIX_DOF: self._init_6dof_widget(),
                SubplotId.ROLL_REF: self._init_roll_ref_widget(),
                SubplotId.PITCH_REF: self._init_pitch_ref_widget(),
                SubplotId.YAW_RATE_REF: self._init_yaw_rate_ref_widget(),
            }

        def _setup_and_place_widgets_in_grid():
            for subplot_id, subplot_widget in self._subplot_widget_map.items():
                self._place_subplot_in_grid(subplot_id, subplot_widget)

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

    def _place_subplot_in_grid(self, subplot_id=SubplotId, widget=SubplotWidget):
        if subplot_id == SubplotId.SIX_DOF:
            self._grid.addWidget(widget.get_base_widget(), 0, 0, 3, 3)

        if subplot_id == SubplotId.ROLL_REF:
            self._grid.addWidget(widget.get_base_widget(), 0, 3, 1, 3)

        if subplot_id == SubplotId.PITCH_REF:
            self._grid.addWidget(widget.get_base_widget(), 1, 3, 1, 3)

        if subplot_id == SubplotId.YAW_RATE_REF:
            self._grid.addWidget(widget.get_base_widget(), 2, 3, 1, 3)

        self._grid.addWidget(QtGui.QLabel("TODO"), 3, 0, 3, 6)

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
                "imu_out": self._simulator.get_imu_out

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

    def closeEvent(self, event):
        self._stop()


def main():
    qt_app = QtGui.QApplication(sys.argv)
    gui = Gui()
    gui.show()
    sys.exit(qt_app.exec_())


if __name__ == '__main__':
    main()
