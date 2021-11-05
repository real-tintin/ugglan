import tkinter as tk
from tkinter import ttk

import numpy as np
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
from matplotlib.figure import Figure

FREQ_START = 1
# TODO: Draw first by hand. Refactor in the end!

# TODO: Tab/box for  drone params, env params, att_est tuning, noise (imu + vibrations), pilot_ctrl tuning.
# TODO: Feature to select input type (step response or game-pad).
# TODO: 3d plot of octocopter.
# TODO: time analysis plot where different signals can be selected.

# READ: https://python-textbok.readthedocs.io/en/1.0/Introduction_to_GUI_Programming.html
# Use class design pattern e.g., https://www.pythontutorial.net/tkinter/tkinter-after/: Create an app to call.
# Make use of menu?

root = tk.Tk()
root.title("Non-linear 6dof simulator")
tabControl = ttk.Notebook(root)

tab1 = ttk.Frame(tabControl)
tab2 = ttk.Frame(tabControl)

tabControl.add(tab1, text='Tab 1')
tabControl.add(tab2, text='Tab 2')
tabControl.pack(expand=1, fill="both")

ttk.Label(tab1,
          text="Welcome to \
          GeeksForGeeks").grid(column=0,
                               row=0,
                               padx=30,
                               pady=30)

fig = Figure(figsize=(5, 4), dpi=100)
t = np.arange(0, 3, .01)
sine, = fig.add_subplot(111).plot(t, np.sin(FREQ_START * np.pi * t))




def _update_plot():
    _update_plot.freq = np.mod(_update_plot.freq + 0.01, 10)
    sine.set_ydata(np.sin(_update_plot.freq * np.pi * t))
    canvas.draw()
    canvas.get_tk_widget().after(10, _update_plot)

_update_plot.freq = FREQ_START

canvas = FigureCanvasTkAgg(fig, master=tab2)  # A tk.DrawingArea.
canvas.draw()
canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
canvas.get_tk_widget().after(10, _update_plot)

toolbar = NavigationToolbar2Tk(canvas, tab2)
toolbar.update()

# placing the toolbar on the Tkinter window
canvas.get_tk_widget().pack()

ttk.Frame()


def main():
    """
    Launch and run the non-linear 6dof simulator gui.
    """
    root.mainloop()
    # freq = FREQ_START
    # while True:
    #     root.update_idletasks()
    #     root.update()
    #     _update_plot(freq)
    #     freq = np.mod(freq + 0.01, 10)
    #     time.sleep(0.01)


if __name__ == "__main__":
    main()
