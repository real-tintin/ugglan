import tkinter as tk
from functools import partial
from tkinter import Menu, simpledialog, messagebox
from typing import Union

import matplotlib.pyplot as plt
import numpy as np
from dataclasses import dataclass, is_dataclass
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg)
from non_linear_sim.att_estimator import DEFAULT_ATT_EST_PARAMS
from non_linear_sim.drone_model import DEFAULT_DRONE_PARAMS, DEFAULT_ENV_PARAMS
from non_linear_sim.pilot_ctrl import DEFAULT_PILOT_CTRL_PARAMS, RefInput
from non_linear_sim.simulator import DEFAULT_IMU_NOISE, Simulator
from non_linear_sim.six_dof_model import STATE_ZERO as SIX_DOF_STATE_ZERO

FORMAT_MENU_LABEL = "{key}: {val}"


# TODO: Support config of 6dof init state.
# TODO: Use dynamics updated mg for f_z_ref.
# TODO: The timing during real-time sim will be an issue on Windows.

@dataclass
class SimParams:
    dt_s: float = 0.01
    standstill_calib_att_est: bool = True


DEFAULT_SIM_PARAMS = SimParams()


@dataclass
class ConfigStepResponse:
    t_end_s: float = 10.0
    ref_input: RefInput = RefInput(f_z=-15.0, roll=np.pi / 8, pitch=0.0, yaw_rate=0.0)


DEFAULT_CONFIG_STEP_RESPONSE = ConfigStepResponse()


@dataclass
class ConfigGamepad:
    t_window_s: float = 10.0
    ref_input_scale: RefInput = RefInput(f_z=-10, roll=np.pi / 4, pitch=np.pi / 4, yaw_rate=np.pi)


DEFAULT_CONFIG_GAMEPAD = ConfigGamepad()


class Gui(tk.Tk):

    def __init__(self):
        super().__init__()

        self.title("Non-linear 6dof simulator")

        self._att_est_params = DEFAULT_ATT_EST_PARAMS
        self._pilot_ctrl_params = DEFAULT_PILOT_CTRL_PARAMS

        self._env_params = DEFAULT_ENV_PARAMS
        self._drone_params = DEFAULT_DRONE_PARAMS

        self._imu_noise = DEFAULT_IMU_NOISE
        self._sim_params = DEFAULT_SIM_PARAMS

        self._config_step_response = DEFAULT_CONFIG_STEP_RESPONSE
        self._config_gamepad = DEFAULT_CONFIG_GAMEPAD

        self._setup_menu()
        self._setup_plot()

        self._reset()

    def _setup_menu(self):
        menu_root = Menu(self)
        menu_config = Menu(menu_root, tearoff=False)
        menu_config_model = Menu(menu_config, tearoff=False)
        menu_config_input = Menu(menu_config, tearoff=False)
        menu_input = Menu(menu_config, tearoff=False)

        def _setup_menu_config():
            _setup_menu_config_model()
            _setup_menu_config_att_est()
            _setup_menu_config_pilot_ctrl()
            _setup_menu_config_imu_noise()
            _setup_menu_config_sim()
            _setup_menu_config_input()

            menu_root.add_cascade(label="Config", menu=menu_config)

        def _setup_menu_config_model():
            self._add_cascade_from_dataclass(menu_config_model, self._env_params, label="Env")
            self._add_cascade_from_dataclass(menu_config_model, self._drone_params, label="Drone")

            menu_config.add_cascade(label="Model", menu=menu_config_model)

        def _setup_menu_config_att_est():
            self._add_cascade_from_dataclass(menu_config, self._att_est_params, label="Attitude estimator")

        def _setup_menu_config_pilot_ctrl():
            self._add_cascade_from_dataclass(menu_config, self._pilot_ctrl_params, label="Pilot controller")

        def _setup_menu_config_imu_noise():
            self._add_cascade_from_dataclass(menu_config, self._imu_noise, label="Imu noise")

        def _setup_menu_config_sim():
            self._add_cascade_from_dataclass(menu_config, self._sim_params, label="Simulation")

        def _setup_menu_config_input():
            self._add_cascade_from_dataclass(menu_config_input, self._config_step_response, label="Step response")
            self._add_cascade_from_dataclass(menu_config_input, self._config_gamepad, label="Gamepad")

            menu_config.add_cascade(label="Input", menu=menu_config_input)

        def _setup_menu_input():
            menu_root.add_cascade(label="Input", menu=menu_input)

            self._is_input_step = tk.BooleanVar(self, True)
            self._is_input_gamepad = tk.BooleanVar(self, False)

            menu_input.add_checkbutton(label="Step response", variable=self._is_input_step, command=self._input_step_cb)
            menu_input.add_checkbutton(label="Gamepad", variable=self._is_input_gamepad, command=self._input_gamepad_cb)

        def _setup_menu_reset():
            menu_root.add_command(label="Reset", command=self._reset)

        def _setup_menu_start():
            menu_root.add_command(label="Start", command=self._start)

        _setup_menu_config()
        _setup_menu_input()
        _setup_menu_reset()
        _setup_menu_start()

        self.config(menu=menu_root)

    def _add_cascade_from_dataclass(self, menu_parent, data_class, label):
        menu_dataclass = Menu(menu_parent, tearoff=False)

        for idx, member in enumerate(data_class.__annotations__):
            val = getattr(data_class, member)
            if is_dataclass(val):
                self._add_cascade_from_dataclass(menu_dataclass, val, member)
            else:
                menu_dataclass_cb = partial(self._menu_dataclass_cb, menu_dataclass, idx, data_class, member)
                menu_dataclass.add_command(label=self._format_menu_label(key=member, val=val),
                                           command=menu_dataclass_cb)

        menu_parent.add_cascade(label=label, menu=menu_dataclass)

    def _menu_dataclass_cb(self, menu, entry_idx, data_class, member):
        old_val = getattr(data_class, member)
        new_val_as_str = simpledialog.askstring(title=member, prompt="Update value:", initialvalue=old_val)

        if new_val_as_str is None:
            return

        def try_casting_str(str_val, cast_type):
            try:
                val = cast_type(str_val)
                return val, True
            except:
                messagebox.showerror("Invalid format", "Can't cast string to {}, check format.".format(cast_type))
                return None, False

        if isinstance(old_val, int):
            new_val, cast_successful = try_casting_str(new_val_as_str, int)
        elif isinstance(old_val, float):
            new_val, cast_successful = try_casting_str(new_val_as_str, float)
        else:
            raise NotImplementedError

        if cast_successful:
            setattr(data_class, member, new_val)
            menu.entryconfigure(entry_idx, label=self._format_menu_label(key=member, val=new_val))

    def _input_step_cb(self):
        self._is_input_step.set(True)
        self._is_input_gamepad.set(False)

    def _input_gamepad_cb(self):
        self._is_input_step.set(False)
        self._is_input_gamepad.set(True)

    def _reset(self):
        self._simulator = Simulator(
            att_est_params=self._att_est_params,
            pilot_ctrl_params=self._pilot_ctrl_params,
            six_dof_state=SIX_DOF_STATE_ZERO,
            drone_params=self._drone_params,
            env_params=self._env_params,
            imu_noise=self._imu_noise,
            dt=self._sim_params.dt_s,
        )

    def _start(self):
        if self._is_input_step.get():
            # TODO: Do this as fast as possible, not figure parallell shit. Store output in arrays and plot!

            # TODO: Why don't we use anim as the driver? for each frame cb, we step the sim?
            self._plot_xdata, self._plot_ydata = [], []
            self._plot_ln, = self._plot_axs[0, 0].plot([], [], 'r--')

            self.after(ms=int(self._sim_params.dt_s * 1e3), func=self._step_simulator)
            # self._launch_plot_animation()
        else:
            raise NotImplementedError

    def _step_simulator(self):
        self._simulator.step(ref_input=self._config_step_response.ref_input)

        # TODO: Update plot here. What's the best logic?
        self._plot_xdata.append(self._simulator.get_t())
        self._plot_ydata.append(self._simulator.get_6dof_state().n_i[0])
        self._plot_ln.set_data(self._plot_xdata, self._plot_ydata)

        self._plot_canvas.draw()

        self.after(ms=int(self._sim_params.dt_s * 1e3), func=self._step_simulator)

    def _setup_plot(self):
        self._plot_fig, self._plot_axs = plt.subplots(2, 2)

        for i_ax, ax in enumerate(self._plot_axs.flatten()):
            ax.set_xlim(0, self._config_step_response.t_end_s)
            ax.set_ylim(0, 3)
            ax.grid()

        self._plot_canvas = FigureCanvasTkAgg(self._plot_fig, master=self)
        self._plot_canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

    def _launch_plot_animation(self):
        self._plot_xdata, self._plot_ydata = [], []
        self._plot_ln, = self._plot_axs[0, 0].plot([], [], 'r--')
        self._t = 0.0

        def init_plot():
            # self._plot_axs[0, 0].set_xlim(0, self._config_step_response.t_end_s)
            # self._plot_axs[0, 0].set_ylim(-1, 1)

            return self._plot_ln,

        def iter_frames():
            # if self._t >= self._config_step_response.t_end_s:
            #    self._plot_ani.pause()
            self._t += self._sim_params.dt_s
            yield self._t

        def _update_frame(t):
            self._plot_xdata.append(t)
            self._plot_ydata.append(self._simulator.get_6dof_state().n_i[0])

            # self._plot_axs[0, 0].set_xlim(0, self._config_step_response.t_end_s)
            # self._plot_axs[0, 0].set_ylim(min(self._plot_ydata), max(self._plot_ydata))

            self._plot_ln.set_data(self._plot_xdata, self._plot_ydata)

            return self._plot_ln,

        self._plot_ani = FuncAnimation(self._plot_fig, _update_frame, frames=iter_frames,
                                       init_func=init_plot, blit=True, interval=int(self._sim_params.dt_s * 1e3))

    @staticmethod
    def _format_menu_label(key: str, val=Union[bool, int, float]) -> str:
        if isinstance(val, int):
            return FORMAT_MENU_LABEL.format(key=key, val=int(val))

        elif isinstance(val, float):
            return FORMAT_MENU_LABEL.format(key=key, val=float(val))

        else:
            raise NotImplementedError


def main():
    """
    Launch and run the non-linear 6dof simulator gui.
    """
    gui = Gui()
    gui.mainloop()


if __name__ == "__main__":
    main()
