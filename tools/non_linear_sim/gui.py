import tkinter as tk
from tkinter import ttk

import numpy as np
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
from matplotlib.figure import Figure


# TODO: Draw first by hand. Refactor in the end!

# TODO: Tab/box for  drone params, env params, att_est tuning, noise (imu + vibrations), pilot_ctrl tuning.
# TODO: Feature to select input type (step response or game-pad).
# TODO: 3d plot of octocopter.
# TODO: time analysis plot where different signals can be selected.

# READ: https://python-textbok.readthedocs.io/en/1.0/Introduction_to_GUI_Programming.html
# Use class design pattern e.g., https://www.pythontutorial.net/tkinter/tkinter-after/: Create an app to call.
# Make use of menu?

class Gui(tk.Tk):
    PLOT_RATE_MS = 10

    def __init__(self):
        super().__init__()

        self.title("Non-linear 6dof simulator")

        self._setup_tabs()
        self._setup_plot()

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

        self._plot_canvas = FigureCanvasTkAgg(fig, master=self._tab_1)  # A tk.DrawingArea.
        self._plot_canvas.draw()

        self._plot_canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
        self._plot_canvas.get_tk_widget().after(10, self._update_plot)

        toolbar = NavigationToolbar2Tk(self._plot_canvas, self._tab_1)
        toolbar.update()

        self._plot_canvas.get_tk_widget().pack()

    def _update_plot(self):
        t = self._plot.get_xdata()

        self._sine_freq = np.mod(self._sine_freq + 0.01, 10)
        self._plot.set_ydata(np.sin(self._sine_freq * np.pi * t))

        self._plot_canvas.draw()
        self._plot_canvas.get_tk_widget().after(self.PLOT_RATE_MS, self._update_plot)


def main():
    """
    Launch and run the non-linear 6dof simulator gui.
    """
    gui = Gui()
    gui.mainloop()


if __name__ == "__main__":
    main()
