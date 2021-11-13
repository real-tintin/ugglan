import tkinter as tk
from functools import partial
from tkinter import ttk, Menu, simpledialog, messagebox

import numpy as np
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
from matplotlib.figure import Figure
from non_linear_sim.simulator import EnvParams, DroneParams

# Are tabs good? It would be nice to se a change in parameter right away. Maybe some parameters can be changed
# quickly (e.g., sliders for pilot ctrl) and some "more hidden".
# * Settings menu with EnvParams, DroneParams, DT and initial state. Makes it easy to add new members.
# * Would be cool with one subplot which is changeable e.g., checkboxes/marked list for figures to show.

# TODO: Tab/box for  drone params, env params, att_est tuning, noise (imu + vibrations), default_pilot_ctrl tuning.
# TODO: Feature to select input type (step response or game-pad).
# TODO: 3d plot of octocopter.
# TODO: time analysis plot where different signals can be selected.

# READ: https://python-textbok.readthedocs.io/en/1.0/Introduction_to_GUI_Programming.html

FORMAT_MENU_LABEL = "{key}: {val}"


class Gui(tk.Tk):
    PLOT_RATE_MS = 10

    def __init__(self):
        super().__init__()

        self.title("Non-linear 6dof simulator")

        # self._setup_tabs()
        self._env_params = EnvParams()
        self._drone_params = DroneParams()

        self._setup_menu()
        self._setup_plot()

    def _setup_menu(self):
        menu_root = Menu(self)
        menu_config = Menu(menu_root, tearoff=False)

        menu_env_params = self._setup_interactive_menu_from_dataclass(menu_root, self._env_params)
        menu_drone_params = self._setup_interactive_menu_from_dataclass(menu_root, self._drone_params)

        menu_config.add_cascade(label="EnvParams", menu=menu_env_params)
        menu_config.add_cascade(label="DroneParams", menu=menu_drone_params)

        menu_root.add_cascade(label="Config", menu=menu_config)
        self.config(menu=menu_root)

    def _setup_interactive_menu_from_dataclass(self, menu_root, data_class):
        menu_dataclass = Menu(menu_root, tearoff=False)

        for idx, member in enumerate(data_class.__annotations__):
            menu_dataclass_cb = partial(self._menu_dataclass_update, menu_dataclass, idx, data_class, member)
            menu_dataclass.add_command(label=FORMAT_MENU_LABEL.format(key=member, val=getattr(data_class, member)),
                                       command=menu_dataclass_cb)

        return menu_dataclass

    def _setup_tabs(self):
        self._tab_ctrl = ttk.Notebook(self)

        self._tab_0 = ttk.Frame(self._tab_ctrl)
        self._tab_1 = ttk.Frame(self._tab_ctrl)

        self._tab_ctrl.add(self._tab_0, text='Tab 1')
        self._tab_ctrl.add(self._tab_1, text='Tab 2')

        self._tab_ctrl.pack(expand=1, fill="both")

        ttk.Label(self._tab_0,
                  text="Welcome to \
                   GeeksForGeeks").grid(column=0,
                                        row=0,
                                        padx=30,
                                        pady=30)

    def _setup_plot(self):
        fig = Figure(figsize=(5, 4), dpi=100)

        self._sine_freq = 1
        t = np.arange(0, 3, .01)

        self._plot, = fig.add_subplot(111).plot(t, np.sin(self._sine_freq * np.pi * t))

        self._plot_canvas = FigureCanvasTkAgg(fig, master=self)
        self._plot_canvas.draw()

        self._plot_canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
        self._plot_canvas.get_tk_widget().after(10, self._update_plot)

        toolbar = NavigationToolbar2Tk(self._plot_canvas, self)
        toolbar.update()

        self._plot_canvas.get_tk_widget().pack()

    def _update_plot(self):
        t = self._plot.get_xdata()

        self._sine_freq = np.mod(self._sine_freq + 0.01, 10)
        self._plot.set_ydata(np.sin(self._sine_freq * np.pi * t))

        self._plot_canvas.draw()
        self._plot_canvas.get_tk_widget().after(self.PLOT_RATE_MS, self._update_plot)

    @staticmethod
    def _menu_dataclass_update(menu, entry_idx, data_class, member):
        cast_successful = False

        old_val = getattr(data_class, member)
        new_val_as_str = simpledialog.askstring(title=FORMAT_MENU_LABEL.format(key=member, val=old_val),
                                                prompt="Update value:",
                                                initialvalue=str(old_val))

        if new_val_as_str is None:
            return

        if isinstance(old_val, int):
            try:
                new_val = int(new_val_as_str)
                cast_successful = True
            except:
                messagebox.showerror("Invalid format", "Can't cast string to int, check format.")

        elif isinstance(old_val, float):
            try:
                new_val = float(new_val_as_str)
                cast_successful = True
            except:
                messagebox.showerror("Invalid format", "Can't cast string to float, check format.")

        elif isinstance(old_val, np.ndarray):
            try:
                new_val = np.array(new_val_as_str)
                cast_successful = True
            except:
                messagebox.showerror("Invalid format", "Can't cast string to ndarray, check format.")

        else:
            raise NotImplementedError

        if cast_successful:
            setattr(data_class, member, new_val)
            menu.entryconfigure(entry_idx, label=FORMAT_MENU_LABEL.format(key=member, val=new_val))


def main():
    """
    Launch and run the non-linear 6dof simulator gui.
    """
    gui = Gui()
    gui.mainloop()


if __name__ == "__main__":
    main()
