import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider

import state_control_utils
from observers import reduced_observer
from state_space import get_state_space, State, StateSpace, get_c

STEP_INFO_STR = 'rise time: {:.2f} s\n' \
                'peak: {:.2f}\n' \
                'overshoot: {:.1f} %'

SLIDER_STEP_SIZE = 0.1

SLIDER_INIT_POS = np.array([0.20, 0.18, 0.60, 0.01])
SLIDER_DELTA_POS = np.array([0, -0.03, 0, 0])

VEL_STATE_IDX = -2
TRQ_STATE_IDX = -1


def analyze_step(state_space: StateSpace,
                 x_0: np.array,
                 x_r: np.array,
                 L_0: np.array,
                 alpha_0: float,
                 t_end: float,
                 title: str,
                 ref_state_ylabel: str):
    """
    Analysis tool for tuning and selecting the state feedback
    controller u = -Lx.

    Note, uses matplotlibs interactive sliders and will launch
    blocking figures.
    """
    ref_state_idx = state_control_utils.find_ref_state_idx(x_r)
    ref_state_val = x_r[ref_state_idx]

    # Setup plot
    fig, axs = plt.subplots(2, 1)
    plt.subplots_adjust(bottom=0.25)

    axs[0].set_title(title)
    axs[0].set_ylabel(ref_state_ylabel)

    axs[1].set_xlabel('Time [s]')
    axs[1].set_ylabel('Torque [Nm]')

    tb_step_info = axs[0].text(1.01, 0.95, STEP_INFO_STR, transform=axs[0].transAxes,
                               fontsize=8, verticalalignment='top')

    axs[0].plot([0, t_end], [ref_state_val, ref_state_val], label='$x_r$', color='tab:red', linestyle='--')

    line_state, = axs[0].plot([], [], label='$x$', color='tab:blue')
    line_trq, = axs[1].plot([], [], label=r'$M$', color='tab:blue')
    line_trq_est, = axs[1].plot([], [], label=r'$\tilde{M}$', color='tab:red', linestyle='--')
    line_u, = axs[1].plot([], [], label='u', color='tab:green')

    for ax in axs:
        ax.legend()
        ax.grid()

    # Add sliders to interactively tune L and alpha
    sliders_L = []
    for i_slider in range(len(L_0)):
        sliders_L.append(Slider(
            plt.axes(SLIDER_INIT_POS + SLIDER_DELTA_POS * i_slider),
            'L_' + str(i_slider + 1), 0, 10, valinit=L_0[i_slider], valstep=SLIDER_STEP_SIZE
        ))

    slider_alpha = Slider(plt.axes(SLIDER_INIT_POS + SLIDER_DELTA_POS * len(L_0)),
                          r'$\alpha$', 0, 10, valinit=alpha_0, valstep=SLIDER_STEP_SIZE)

    def _update_plot(val=None):
        # Get parameters to tune controller & observer
        alpha = slider_alpha.val
        L = np.array([[sl.val for sl in sliders_L]])

        # Step response of closed loop and observer
        t, x, u, y = state_control_utils.closed_loop_step_response(state_space, L, x_0, x_r, t_end)
        si = state_control_utils.step_info(t, x[ref_state_idx])
        x_est = reduced_observer(x_v=x[VEL_STATE_IDX], u=u,
                                 c=get_c(state_space.state), alpha=alpha,
                                 z_0=u[0] / 2 - alpha * x[VEL_STATE_IDX, 0])

        # Update and re-draw plot
        tb_step_info.set_text(STEP_INFO_STR.format(si.rise_time, si.peak, si.overshoot))

        line_state.set_data(t, x[ref_state_idx])
        line_trq.set_data(t, x[TRQ_STATE_IDX])
        line_trq_est.set_data(t, x_est)
        line_u.set_data(t, u)

        for ax in axs:
            ax.relim()
            ax.autoscale_view()

    # Add slider cb's and draw plot
    for sl in sliders_L:
        sl.on_changed(_update_plot)
    slider_alpha.on_changed(_update_plot)

    _update_plot()
    plt.show()


def main():
    """
    Launches interactive plot's for the analysis and
    tuning of the state feedback controller.
    """

    analyze_step(state_space=get_state_space(State.PHI, add_intg_state=True),  # Same as for THETA (pitch)
                 x_0=np.zeros((4, 1)),
                 x_r=np.array([[0, np.pi / 8, 0, 0]]).transpose(),
                 L_0=np.array([0.5, 4.0, 0.6, 1.5]),
                 alpha_0=2.0,
                 t_end=1,
                 title=r'Step in roll ($\phi$) & pitch ($\theta$)',
                 ref_state_ylabel='Angle [rad]')

    analyze_step(state_space=get_state_space(State.PSI, add_intg_state=False),
                 x_0=np.zeros((3, 1)),
                 x_r=np.array([[0, np.pi, 0]]).transpose(),
                 L_0=np.array([0.1, 0.2, 0.3]),
                 alpha_0=2.0,
                 t_end=2,
                 title=r'Step in yaw-rate ($\dot\psi$)',
                 ref_state_ylabel='Angular-rate [rad/s]')


if __name__ == '__main__':
    main()
