import argparse
from enum import Enum

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider

from ugglan_tools.linear_sim import step_response
from ugglan_tools.linear_sim.state_space import State

STEP_INFO_STR = r"""
rise time: {:.2f} s
peak: {:.2f}
overshoot: {:.1f}
$\max(|\int x|)$: {:.3f}
"""

SLIDER_INIT_POS = np.array([0.20, 0.18, 0.60, 0.01])
SLIDER_DELTA_POS = np.array([0, -0.03, 0, 0])

VEL_STATE_IDX = -2
TRQ_STATE_IDX = -1


class PlotState(Enum):
    PHI_THETA = 'PhiTheta'
    PSI = 'Psi'

    def __str__(self):
        return self.value


def main():
    parser = argparse.ArgumentParser(
        description="Launches interactive plot's for the analysis and tuning of the state feedback controller."
    )
    parser.add_argument('--plot-state', type=PlotState, nargs='+', choices=list(PlotState),
                        default=list(PlotState), help='Selected state(s) to plot')
    args = parser.parse_args()

    if PlotState.PHI_THETA in args.plot_state:
        _analyze_step(state=State.PHI,  # Same as for THETA (pitch)
                      add_intg_state=True,
                      x_0=np.zeros((4, 1)),
                      x_r=np.array([[0, np.pi / 8, 0, 0]]).transpose(),
                      L_0=np.array([[0.4, 3.85, 0.55, 0.02]]),
                      t_end=1,
                      title=r'Step in roll ($\phi$) & pitch ($\theta$)',
                      ref_state_ylabel='Angle [rad]',
                      slider_step_L=0.01,
                      slider_max_L=5)

    if PlotState.PSI in args.plot_state:
        _analyze_step(state=State.PSI,
                      add_intg_state=False,
                      x_0=np.zeros((3, 1)),
                      x_r=np.array([[0, np.pi, 0]]).transpose(),
                      L_0=np.array([[0.02, 0.04, 0.0]]),
                      t_end=10,
                      title=r'Step in yaw-rate ($\dot\psi$)',
                      ref_state_ylabel='Angular-rate [rad/s]',
                      slider_step_L=0.001,
                      slider_max_L=0.1)


def _analyze_step(state: State,
                  add_intg_state: bool,
                  x_0: np.array,
                  x_r: np.array,
                  L_0: np.array,
                  t_end: float,
                  title: str,
                  ref_state_ylabel: str,
                  slider_step_L: float = 0.1,
                  slider_max_L: float = 10):
    """
    Analysis tool for tuning and selecting the state feedback
    controller u = -Lx.

    Note, uses matplotlibs interactive sliders and will launch
    blocking figures.
    """
    ref_state_idx = _find_ref_state_idx(x_r)
    ref_state_val = x_r[ref_state_idx]

    # Setup plot
    fig, axs = plt.subplots(2, 1)
    plt.subplots_adjust(bottom=0.25)

    axs[0].set_title(title)
    axs[0].set_ylabel(ref_state_ylabel)

    axs[1].set_xlabel('Time [s]')
    axs[1].set_ylabel(_get_u_y_label(state))

    tb_step_info = axs[0].text(1.01, 1.02, STEP_INFO_STR, transform=axs[0].transAxes,
                               fontsize=8, verticalalignment='top')

    axs[0].plot([0, t_end], [ref_state_val, ref_state_val], label='$x_r$', color='tab:red', linestyle='--')

    line_x, = axs[0].plot([], [], label='$x$', color='tab:blue')
    line_u, = axs[1].plot([], [], label=r'$u$', color='tab:green')

    for ax in axs:
        ax.legend()
        ax.grid()

    # Add sliders to interactively tune L
    sliders_L = []
    for i_slider in range(L_0.size):
        sliders_L.append(Slider(
            plt.axes(SLIDER_INIT_POS + SLIDER_DELTA_POS * i_slider),
            'L_' + str(i_slider + 1), 0, slider_max_L, valinit=L_0[0, i_slider], valstep=slider_step_L
        ))

    def _update_plot(val=None):
        # Get parameters to tune controller & observer
        L = np.array([[sl.val for sl in sliders_L]])

        # Step response of closed loop
        t, x, y, u = step_response.closed_loop(state, L, x_0, x_r, t_end, add_intg_state)
        si = step_response.step_info(t, x[ref_state_idx])

        # Update and re-draw plot
        x_intg_abs_max = np.max(np.abs(x[0]))
        tb_step_info.set_text(STEP_INFO_STR.format(si.rise_time, si.peak, si.overshoot, x_intg_abs_max))

        line_x.set_data(t, x[ref_state_idx])
        line_u.set_data(t, u)

        for ax in axs:
            ax.relim()
            ax.autoscale_view()

    # Add slider cb's and draw plot
    for sl in sliders_L:
        sl.on_changed(_update_plot)

    _update_plot()
    plt.show()


def _get_u_y_label(state: State):
    if state == State.PHI or state == State.THETA or state == State.PSI:
        return 'Ctrl input [Nm]'
    else:
        return 'Ctrl input [N]'


def _find_ref_state_idx(x_r):
    idx = np.argwhere(x_r != 0)

    if len(idx) != 1:
        raise ValueError('Expected to find only one reference state.')

    return idx[0, 0]


if __name__ == '__main__':
    main()
