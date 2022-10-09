import argparse
import tkinter as tk
from functools import partial
from pathlib import Path

import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
from scipy import signal

import ugglan_tools.data_log.io as data_log_io
from ugglan_tools.filter_design.filters import FilterName, get_default_filter_config_map, DEFAULT_N_ORDER, \
    DEFAULT_RIPPLE_PASS, DEFAULT_RIPPLE_STOP, DEFAULT_HIGH_CUT_OFF, \
    DEFAULT_LOW_CUT_OFF, FilterType, get_sos_filter_from_config, add_wn_to_lines, set_filter_config_map_params

np.seterr(divide='ignore')

FORMAT_SIGNAL_LABEL = r'${signal_name}^{{{filter_name}}}$'

BACKGROUND_COLOR = 'white'

DEFAULT_FILTER = FilterName.BUTTER
DEFAULT_DT = 0.01


class FilterDesign(tk.Frame):
    def __init__(self, master):
        tk.Frame.__init__(self, master)
        self.master.configure(background=BACKGROUND_COLOR)

        self._data_path = Path()
        self._data_log = None
        self._data_signals = [['N', 'A']]
        self._sel_data_signal_idx = 0

        self._sel_filter_name = DEFAULT_FILTER
        self._dt = DEFAULT_DT

        self._filter_config_map = get_default_filter_config_map()

        self._create_frames()
        self._init_plot_widget()
        self._init_config_widget()

    def _create_frames(self):
        self._left_frame = tk.Frame(self.master, bg=BACKGROUND_COLOR)
        self._left_frame.pack(side=tk.LEFT, fill="both", expand=True)

        self._right_frame = tk.Frame(self.master, bg=BACKGROUND_COLOR)
        self._right_frame.pack(side=tk.RIGHT, fill="both", padx=15)

    def _init_config_widget(self):
        def setup_data_config_frame():
            config_frame = tk.LabelFrame(self._right_frame, text='Data config', bg=BACKGROUND_COLOR, padx=5, pady=5)
            config_frame.pack(side=tk.TOP, pady=100)

            def setup_button_file():
                button_file = tk.Button(config_frame, text="Select file", command=self._cb_file_button)
                button_file.grid(row=0, column=0, sticky=tk.E)

            def setup_label_file():
                label_file_name = tk.Label(config_frame, text='File name:', bg=BACKGROUND_COLOR)
                label_file_name.grid(row=1, column=0, sticky=tk.E)

                self._label_file_name = tk.Label(config_frame, text='', bg=BACKGROUND_COLOR)
                self._label_file_name.grid(row=1, column=1, sticky=tk.W)

            def setup_label_signal():
                label_signal = tk.Label(config_frame, text='Data signal:', bg=BACKGROUND_COLOR)
                label_signal.grid(row=2, column=0, sticky=tk.E)

            def setup_menu_signal():
                self._menu_signal_val = tk.StringVar(self.master, value=self._get_sel_data_signal_name())
                self._menu_signal = tk.OptionMenu(config_frame, self._menu_signal_val, *self._get_data_signal_names(),
                                                  command=self._cb_menu_signal)
                self._menu_signal.config(highlightthickness=0)
                self._menu_signal.grid(row=2, column=1, sticky=tk.E)

            def setup_label_dt():
                label_dt = tk.Label(config_frame, text='Data signal:', bg=BACKGROUND_COLOR)
                label_dt.grid(row=2, column=0, sticky=tk.E)

            def setup_spin_box_dt():
                label_dt = tk.Label(config_frame, text='Resample-rate [Hz]:', bg=BACKGROUND_COLOR)
                label_dt.grid(row=3, column=0, sticky=tk.E)

                self._spin_box_dt = tk.Spinbox(config_frame, from_=0.01, to=1.0, increment=0.01,
                                               textvariable=tk.StringVar(value=str(self._dt)),
                                               command=self._cb_spin_box_dt)
                self._spin_box_dt.grid(row=3, column=1, sticky=tk.W)

            setup_button_file()
            setup_label_file()

            setup_label_signal()
            setup_menu_signal()

            setup_label_dt()
            setup_spin_box_dt()

        def setup_filter_config_frame():
            config_frame = tk.LabelFrame(self._right_frame, text='Filter config', bg=BACKGROUND_COLOR, padx=5, pady=5)
            config_frame.pack(side=tk.TOP)

            def setup_label_filter_name():
                label_filter_name = tk.Label(config_frame, text='Filter:', bg=BACKGROUND_COLOR)
                label_filter_name.grid(row=0, column=0, sticky=tk.E)

            def setup_menu_filter_name():
                self._menu_filter_name = tk.OptionMenu(config_frame,
                                                       tk.StringVar(value=self._get_sel_filter_config().name.value),
                                                       *self._get_filter_names(), command=self._cb_menu_filter_name)
                self._menu_filter_name.config(highlightthickness=0)
                self._menu_filter_name.grid(row=0, column=1, sticky=tk.E)

            def setup_label_filter_type():
                label_filter_type = tk.Label(config_frame, text='Type:', bg=BACKGROUND_COLOR)
                label_filter_type.grid(row=1, column=0, sticky=tk.E)

            def setup_menu_filter_type():
                self._menu_filter_type = tk.OptionMenu(config_frame,
                                                       tk.StringVar(value=self._get_sel_filter_config().type.value),
                                                       *self._get_filter_types(), command=self._cb_menu_filter_type)
                self._menu_filter_type.config(highlightthickness=0)
                self._menu_filter_type.grid(row=1, column=1, sticky=tk.E)

            def setup_label_low_freq():
                label_low_freq = tk.Label(config_frame, text='Low-freq [Hz]:', bg=BACKGROUND_COLOR)
                label_low_freq.grid(row=2, column=0, sticky=tk.E)

            def setup_spin_box_low_freq():
                self._spin_box_low_freq = tk.Spinbox(config_frame, from_=0.1, to=100, increment=0.1,
                                                     textvariable=tk.StringVar(value=DEFAULT_LOW_CUT_OFF),
                                                     command=self._cb_spin_box_low_freq)
                self._spin_box_low_freq.grid(row=2, column=1, sticky=tk.W)

            def setup_label_high_freq():
                label_high_freq = tk.Label(config_frame, text='High-freq [Hz]:', bg=BACKGROUND_COLOR)
                label_high_freq.grid(row=3, column=0, sticky=tk.E)

            def setup_spin_box_high_freq():
                self._spin_box_high_freq = tk.Spinbox(config_frame, from_=0.1, to=100, increment=0.1,
                                                      textvariable=tk.StringVar(value=DEFAULT_HIGH_CUT_OFF),
                                                      command=self._cb_spin_box_high_freq)
                self._spin_box_high_freq.grid(row=3, column=1, sticky=tk.W)

            def setup_label_ripple_pass():
                label_ripple_pass = tk.Label(config_frame, text='Ripple-pass [db]:', bg=BACKGROUND_COLOR)
                label_ripple_pass.grid(row=4, column=0, sticky=tk.E)

            def setup_spin_box_ripple_pass():
                self._spin_box_ripple_pass = tk.Spinbox(config_frame, from_=0.1, to=100, increment=0.1,
                                                        textvariable=tk.StringVar(value=DEFAULT_RIPPLE_PASS),
                                                        command=self._cb_spin_box_ripple_pass)
                self._spin_box_ripple_pass.grid(row=4, column=1, sticky=tk.W)

            def setup_label_ripple_stop():
                label_ripple_stop = tk.Label(config_frame, text='Ripple-stop [db]:', bg=BACKGROUND_COLOR)
                label_ripple_stop.grid(row=5, column=0, sticky=tk.E)

            def setup_spin_box_ripple_stop():
                self._spin_box_ripple_stop = tk.Spinbox(config_frame, from_=0.1, to=100, increment=0.1,
                                                        textvariable=tk.StringVar(value=DEFAULT_RIPPLE_STOP),
                                                        command=self._cb_spin_box_ripple_stop)
                self._spin_box_ripple_stop.grid(row=5, column=1, sticky=tk.W)

            def setup_label_n_order():
                label_n_order = tk.Label(config_frame, text='N-order:', bg=BACKGROUND_COLOR)
                label_n_order.grid(row=6, column=0, sticky=tk.E)

            def setup_spin_box_n_order():
                self._spin_box_n_order = tk.Spinbox(config_frame, from_=1, to=30,
                                                    textvariable=tk.StringVar(value=DEFAULT_N_ORDER),
                                                    command=self._cb_spin_box_n_order)
                self._spin_box_n_order.grid(row=6, column=1, sticky=tk.W)

            setup_label_filter_name()
            setup_menu_filter_name()

            setup_label_filter_type()
            setup_menu_filter_type()

            setup_label_low_freq()
            setup_spin_box_low_freq()

            setup_label_high_freq()
            setup_spin_box_high_freq()

            setup_label_ripple_stop()
            setup_spin_box_ripple_stop()

            setup_label_ripple_pass()
            setup_spin_box_ripple_pass()

            setup_label_n_order()
            setup_spin_box_n_order()

        setup_data_config_frame()
        setup_filter_config_frame()

        self._update_spin_box_states()

    def _cb_file_button(self):
        self._data_path = Path(tk.filedialog.askopenfilename())
        self._label_file_name['text'] = self._data_path.name

        self._reload_data_log()
        self._reload_data_signals()

        self._update_menu_signal()
        self._update_plot_widget()

    def _cb_spin_box_dt(self):
        self._dt = float(self._spin_box_dt.get())

        self._reload_data_log()
        self._update_plot_widget()

    def _cb_menu_signal(self, signal_name):
        self._sel_data_signal_idx = self._get_data_signal_names().index(signal_name)
        self._menu_signal_val.set(self._get_sel_data_signal_name())

        self._update_plot_widget(force_autoscale=True)

    def _cb_menu_filter_name(self, filter_name):
        self._sel_filter_name = FilterName(filter_name)

        self._update_spin_box_states()
        self._update_plot_widget()

    def _cb_menu_filter_type(self, filter_type):
        self._set_config_map_params(type=FilterType(filter_type))

        self._update_spin_box_states()
        self._update_plot_widget()

    def _cb_spin_box_low_freq(self):
        self._set_config_map_params(low_cut_off=float(self._spin_box_low_freq.get()))
        self._update_plot_widget()

    def _cb_spin_box_high_freq(self):
        self._set_config_map_params(high_cut_off=float(self._spin_box_high_freq.get()))
        self._update_plot_widget()

    def _cb_spin_box_ripple_stop(self):
        self._set_config_map_params(ripple_stop=float(self._spin_box_ripple_stop.get()))
        self._update_plot_widget()

    def _cb_spin_box_ripple_pass(self):
        self._set_config_map_params(ripple_pass=float(self._spin_box_ripple_pass.get()))
        self._update_plot_widget()

    def _cb_spin_box_n_order(self):
        self._set_config_map_params(n_order=int(self._spin_box_n_order.get()))
        self._update_plot_widget()

    def _reload_data_log(self):
        self._data_log = data_log_io.read(path=self._data_path, resample_to_fixed_rate_s=self._dt)

    def _update_spin_box_states(self):
        filter_config = self._get_sel_filter_config()

        def update_freq_state():
            if filter_config.type is FilterType.LOW_PASS:
                self._spin_box_low_freq.configure(state='normal')
                self._spin_box_high_freq.configure(state='disable')

            elif filter_config.type is FilterType.HIGH_PASS:
                self._spin_box_low_freq.configure(state='disable')
                self._spin_box_high_freq.configure(state='normal')

            else:
                self._spin_box_low_freq.configure(state='normal')
                self._spin_box_high_freq.configure(state='normal')

        def update_ripple_state():
            if filter_config.has_ripple_stop:
                self._spin_box_ripple_stop.configure(state='normal')
            else:
                self._spin_box_ripple_stop.configure(state='disable')

            if filter_config.has_ripple_pass:
                self._spin_box_ripple_pass.configure(state='normal')
            else:
                self._spin_box_ripple_pass.configure(state='disable')

        update_freq_state()
        update_ripple_state()

    def _init_plot_widget(self):
        def setup_fig():
            self._fig = plt.figure()
            gs = gridspec.GridSpec(2, 2)

            self._ax_time = plt.subplot(gs[0, :])
            self._ax_bode_amp = plt.subplot(gs[1, 0])
            self._ax_bode_phase = plt.subplot(gs[1, 1])

            self._ax_time.plot([], [], linewidth=1.0)
            self._ax_time.plot([], [], linewidth=1.0)
            self._ax_time.set(xlabel='time [s]')
            self._ax_time.grid()

            self._ax_bode_amp.semilogx([], [])
            self._ax_bode_amp.axvline(np.nan, color='green')
            self._ax_bode_amp.axvline(np.nan, color='red')
            self._ax_bode_amp.set(xlabel='Frequency [Hz]', ylabel='Amplitude [dB]')
            self._ax_bode_amp.grid(which='both', axis='both')

            self._ax_bode_phase.semilogx([], [])
            self._ax_bode_phase.axvline(np.nan, color='green')
            self._ax_bode_phase.axvline(np.nan, color='red')
            self._ax_bode_phase.set(xlabel='Frequency [Hz]', ylabel='Phase [deg]')
            self._ax_bode_phase.grid(which='both', axis='both')

        def add_fig_to_canvas():
            self._fig_canvas = FigureCanvasTkAgg(self._fig, self._left_frame)
            self._fig_canvas.draw()
            self._fig_canvas.get_tk_widget().pack(side='top', fill="both", expand=True)

        def add_toolbar_to_canvas():
            toolbar = NavigationToolbar2Tk(self._fig_canvas, self._left_frame)
            toolbar.update()

        setup_fig()
        add_fig_to_canvas()
        add_toolbar_to_canvas()

    def _update_plot_widget(self, force_autoscale=False):
        if not self._is_data_log_loaded():
            return

        filter_config = self._get_sel_filter_config()
        data_signal = self._get_sel_data_signal()
        sos_filter = get_sos_filter_from_config(filter_config, fs=self._get_fs())

        data_signal_filtered = signal.sosfilt(sos_filter, data_signal.val)
        w, h = signal.sosfreqz(sos_filter, fs=self._get_fs())

        self._ax_time.lines[0].set_data(data_signal.t_s, data_signal.val)
        self._ax_time.lines[0].set_label(FORMAT_SIGNAL_LABEL.format(signal_name=self._get_sel_data_signal_name(),
                                                                    filter_name='raw'))

        self._ax_time.lines[1].set_data(data_signal.t_s, data_signal_filtered)
        self._ax_time.lines[1].set_label(FORMAT_SIGNAL_LABEL.format(signal_name=self._get_sel_data_signal_name(),
                                                                    filter_name=filter_config.name.value))

        self._ax_bode_amp.lines[0].set_data(w, 20 * np.log10(np.abs(h)))
        self._ax_bode_amp.lines[0].set_label(filter_config.name.value)
        add_wn_to_lines(filter_config, self._ax_bode_amp.lines[1], self._ax_bode_amp.lines[2])

        self._ax_bode_phase.lines[0].set_data(w, np.unwrap(np.angle(h, deg=True)))
        self._ax_bode_phase.lines[0].set_label(filter_config.name.value)
        add_wn_to_lines(filter_config, self._ax_bode_phase.lines[1], self._ax_bode_phase.lines[2])

        for ax in [self._ax_time, self._ax_bode_amp, self._ax_bode_phase]:
            ax.relim()
            if force_autoscale:
                ax.autoscale()
            else:
                ax.autoscale_view()
            ax.legend(loc='upper right', fontsize=8)

        self._fig_canvas.draw()

    def _update_menu_signal(self):
        menu = self._menu_signal["menu"]
        menu.delete(0, "end")

        for signal_name in self._get_data_signal_names():
            menu.add_command(label=signal_name, command=partial(self._cb_menu_signal, signal_name))

        self._menu_signal_val.set(self._get_sel_data_signal_name())

    def _reload_data_signals(self):
        self._data_signals = []

        for group_name in sorted(self._data_log.__dict__.keys()):
            group = getattr(self._data_log, group_name)

            for signal_name in sorted(group.__dict__.keys()):
                self._data_signals.append([group_name, signal_name])

    def _get_sel_data_signal(self):
        group_name, signal_name = self._data_signals[self._sel_data_signal_idx]
        data_signal = getattr(getattr(self._data_log, group_name), signal_name)

        return data_signal

    def _get_sel_data_signal_name(self):
        return ''.join(self._data_signals[self._sel_data_signal_idx])

    def _get_data_signal_names(self):
        return [''.join(group_signal) for group_signal in self._data_signals]

    def _set_config_map_params(self, **kwargs):
        set_filter_config_map_params(self._filter_config_map, **kwargs)

    def _get_sel_filter_config(self):
        return self._filter_config_map[self._sel_filter_name]

    def _get_fs(self):
        return int(1 / self._dt)

    def _is_data_log_loaded(self):
        return self._data_log is not None

    @staticmethod
    def _get_filter_names():
        return [filter_name.value for filter_name in FilterName]

    @staticmethod
    def _get_filter_types():
        return [filter_type.value for filter_type in FilterType]


def main():
    parser = argparse.ArgumentParser(description='Launches a gui for filter design.')
    parser.parse_args()

    root = tk.Tk()
    FilterDesign(root)
    root.mainloop()


if __name__ == "__main__":
    main()
