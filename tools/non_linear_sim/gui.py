import tkinter as tk
from functools import partial
from tkinter import Menu, simpledialog, messagebox
from typing import Union

import numpy as np
from dataclasses import dataclass
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
from matplotlib.figure import Figure
from non_linear_sim.att_estimator import DEFAULT_ATT_EST_PARAMS
from non_linear_sim.drone_model import DEFAULT_DRONE_PARAMS, DEFAULT_ENV_PARAMS
from non_linear_sim.pilot_ctrl import DEFAULT_PILOT_CTRL_PARAMS
from non_linear_sim.simulator import DEFAULT_IMU_NOISE

# Are tabs good? It would be nice to se a change in parameter right away. Maybe some parameters can be changed
# quickly (e.g., sliders for pilot ctrl) and some "more hidden".
# * Settings menu with EnvParams, DroneParams, DT and initial state. Makes it easy to add new members.
# * Would be cool with one subplot which is changeable e.g., checkboxes/marked list for figures to show.

# TODO: Tab/box for  drone params, env params, att_est tuning, noise (imu + vibrations), default_pilot_ctrl tuning.
# TODO: Feature to select input type (step response or game-pad).
# TODO: 3d plot of octocopter.
# TODO: time analysis plot where different signals can be selected.

# I want some results now! Control everthing with menu now and setup some initial subplots. Yes.

# READ: https://python-textbok.readthedocs.io/en/1.0/Introduction_to_GUI_Programming.html

FORMAT_MENU_LABEL = "{key}: {val}"


# TODO: Make use of dataclass stuf

# TODO: Support update of 6dof init state.

@dataclass
class SimParams:
    dt_s: float = 0.01
    t_end_s: float = 1.0
    standstill_calib_att_est: bool = True


DEFAULT_SIM_PARAMS = SimParams()


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

        self._setup_menu()
        self._setup_plot()

    def _setup_menu(self):
        menu_root = Menu(self)
        menu_config = Menu(menu_root, tearoff=False)
        menu_model = Menu(menu_config, tearoff=False)

        def _setup_menu_config():
            _setup_menu_config_model()
            _setup_menu_config_att_est()
            _setup_menu_config_pilot_ctrl()
            _setup_menu_config_imu_noise()
            _setup_menu_config_sim()

            menu_root.add_cascade(label="Config", menu=menu_config)

        def _setup_menu_config_model():
            menu_env_params = self._setup_interactive_menu_from_dataclass(menu_model, self._env_params)
            menu_drone_params = self._setup_interactive_menu_from_dataclass(menu_model, self._drone_params)

            menu_model.add_cascade(label="Env", menu=menu_env_params)
            menu_model.add_cascade(label="Drone", menu=menu_drone_params)

            menu_config.add_cascade(label="Model", menu=menu_model)

        def _setup_menu_config_att_est():
            menu_att_est = self._setup_interactive_menu_from_dataclass(menu_model, self._att_est_params)
            menu_config.add_cascade(label="AttEst", menu=menu_att_est)

        def _setup_menu_config_pilot_ctrl():
            menu_pilot_ctrl = self._setup_interactive_menu_from_dataclass(menu_model, self._pilot_ctrl_params)
            menu_config.add_cascade(label="PilotCtrl", menu=menu_pilot_ctrl)

        def _setup_menu_config_imu_noise():
            menu_imu_noise = self._setup_interactive_menu_from_dataclass(menu_model, self._imu_noise)
            menu_config.add_cascade(label="ImuNoise", menu=menu_imu_noise)

        def _setup_menu_config_sim():
            menu_sim_params = self._setup_interactive_menu_from_dataclass(menu_model, self._sim_params)
            menu_config.add_cascade(label="Simulator", menu=menu_sim_params)

        def _setup_menu_reset():
            pass

        def _setup_menu_start():
            pass

        _setup_menu_config()
        _setup_menu_reset()
        _setup_menu_start()

        self.config(menu=menu_root)

    def _setup_interactive_menu_from_dataclass(self, menu_root, data_class):
        menu_dataclass = Menu(menu_root, tearoff=False)

        for idx, member in enumerate(data_class.__annotations__):
            menu_dataclass_cb = partial(self._menu_dataclass_update, menu_dataclass, idx, data_class, member)
            menu_dataclass.add_command(label=self._format_menu_label(key=member, val=getattr(data_class, member)),
                                       command=menu_dataclass_cb)

        return menu_dataclass

    def _menu_dataclass_update(self, menu, entry_idx, data_class, member):
        old_val = getattr(data_class, member)
        new_val_as_str = simpledialog.askstring(title=self._format_menu_label(key=member, val=old_val),
                                                prompt="Update value:",
                                                initialvalue=old_val)

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

    def _reset(self):
        # TODO: Re-construct simulator
        pass

    def _start(self):
        # TODO: Step unit t_end, here we will need cb to update plots
        pass

    def _setup_plot(self):
        fig = Figure(figsize=(5, 4), dpi=100)

        self._sine_freq = 1
        t = np.arange(0, 3, .01)

        self._plot, = fig.add_subplot(111).plot(t, np.sin(self._sine_freq * np.pi * t))

        self._plot_canvas = FigureCanvasTkAgg(fig, master=self)
        self._plot_canvas.draw()

        self._plot_canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
        self._plot_canvas.get_tk_widget().after(int(self._sim_params.dt_s * 1e3), self._update_plot)

        toolbar = NavigationToolbar2Tk(self._plot_canvas, self)
        toolbar.update()

        self._plot_canvas.get_tk_widget().pack()

    def _update_plot(self):
        t = self._plot.get_xdata()

        self._sine_freq = np.mod(self._sine_freq + 0.01, 10)
        self._plot.set_ydata(np.sin(self._sine_freq * np.pi * t))

        self._plot_canvas.draw()
        self._plot_canvas.get_tk_widget().after(int(self._sim_params.dt_s * 1e3), self._update_plot)

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
