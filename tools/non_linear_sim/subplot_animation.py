from collections import deque
from copy import copy
from typing import Callable, Tuple, Deque, List

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from dataclasses import dataclass
from matplotlib.axes import Axes
from matplotlib.lines import Line2D

UPDATE_Y_LIM_INTERVAL_MIN_S = 0.2  # s

DataCbType = Callable[[], float]


@dataclass
class Line:
    line_2d: Line2D
    cb: DataCbType
    y_data: Deque


@dataclass
class Subplot:
    lines: List[Line]
    ax: Axes
    use_dynamic_ylim: bool = False


class SubplotAnimation(animation.TimedAnimation):

    def __init__(self, n_rows: int, n_cols: int,
                 dt_s: float,
                 t_window_size_s: float):
        """
        Creates a timed subplot animation. It is assumed that x-axis is
        time in seconds and fixed length (controlled by t_window_size_s).
        """

        # TODO: Some setting to controlled if dynamic x_lim
        # TODO: Set x_lim, handle dynamic y_lim, rolling x_axis if needed.

        self._t_s = 0.0
        self._dt_s = dt_s
        self._t_window_size_s = t_window_size_s

        self._n_samples = int(t_window_size_s / dt_s)
        self._x_data = np.arange(-t_window_size_s, 0, dt_s)[::-1]  # TODO: Refactor

        self._prev_y_lim_update_s = 0.0
        self._update_y_lim_interval_s = max(UPDATE_Y_LIM_INTERVAL_MIN_S, dt_s)

        self._fig = plt.figure()
        self._subplots = []

        for i_row in range(n_rows):
            for i_col in range(n_cols):
                i_subplot = n_cols * i_row + i_col + 1

                ax = self._fig.add_subplot(n_rows, n_cols, i_subplot)

                ax.set_xlim(-t_window_size_s, 0)
                ax.grid()

                self._subplots.append(Subplot(lines=[], ax=ax))

        self._i_frame = 0
        self._is_running = False

    def run(self):
        animation.TimedAnimation.__init__(self,
                                          fig=self._fig,
                                          interval=int(self._dt_s * 1e3),
                                          blit=True,
                                          repeat=False)
        self._is_running = True

    def stop(self):
        self.pause()
        self._is_running = False

    def style_axis(self, i_subplot: int,
                   xlabel: str = "", ylabel: str = "",
                   init_ylim: Tuple = (0, 1),
                   use_dynamic_ylim: bool = False):
        self._subplots[i_subplot].ax.set_xlabel(xlabel)
        self._subplots[i_subplot].ax.set_ylabel(ylabel)

        self._subplots[i_subplot].ax.set_ylim(*init_ylim)
        self._subplots[i_subplot].use_dynamic_ylim = use_dynamic_ylim

    def add_line(self, i_subplot: int, data_cb: DataCbType, **kwargs):
        line_2d = Line2D([], [], **kwargs)

        subplot = self._subplots[i_subplot]
        subplot.ax.add_line(line_2d)

        subplot.lines.append(Line(line_2d=line_2d,
                                  cb=data_cb,
                                  y_data=deque(maxlen=self._n_samples)))

    def new_frame_seq(self):
        while self._is_running:
            self._t_s += self._dt_s
            yield self._t_s

    def _draw_frame(self, t_s):
        self._drawn_artists = []  # Use by the blit to clear the frame.

        for subplot in self._subplots:
            old_y_lim = subplot.ax.get_ylim()
            new_y_lim = copy(old_y_lim)

            for line in subplot.lines:
                line.y_data.append(line.cb())

                if subplot.use_dynamic_ylim:
                    new_y_lim = (min(new_y_lim[0], min(line.y_data)),
                                 max(new_y_lim[1], max(line.y_data)))

                line.line_2d.set_data(self._x_data[0:len(line.y_data)], line.y_data)
                self._drawn_artists.append(line.line_2d)

            if subplot.use_dynamic_ylim and \
                    (new_y_lim[0] < old_y_lim[0] or new_y_lim[1] > old_y_lim[1]) and \
                    (t_s - self._prev_y_lim_update_s) > self._update_y_lim_interval_s:
                subplot.ax.set_ylim(new_y_lim)
                self._fig.canvas.resize_event()
                self._prev_y_lim_update_s = t_s


def main():
    def data_cb():
        data_cb.t += 0.01
        return np.sin(data_cb.t)

    data_cb.t = 0

    subplot_ani = SubplotAnimation(2, 2, dt_s=0.01, t_window_size_s=10.0)

    for i_subplot in range(4):
        subplot_ani.style_axis(i_subplot=i_subplot, xlabel="t [s]", ylabel="sin(t)", use_dynamic_ylim=True)
        subplot_ani.add_line(i_subplot=i_subplot, data_cb=data_cb, color="black", linestyle=":")

    subplot_ani.run()

    plt.show()


if __name__ == "__main__":
    main()
