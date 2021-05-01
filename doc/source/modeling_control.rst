Modeling & Control
*******************
.. _drone_multi_body:
.. figure:: figures/drone_multi_body.svg
    :width: 100%

    Multi body analysis of the drone.

Before moving on, note that some in-depth parts of the modeling, state estimation
and Control are not covered in this document, for details see the arthur's master
thesis `Design-Modeling-and-Control-of-an-Octocopter`_.

.. _Design-Modeling-and-Control-of-an-Octocopter: http://www.diva-portal.org/smash/get/diva2:857660/FULLTEXT01.pdf

Nomenclature
===================
:math:`_{I}` Inertial reference frame

:math:`_{B}` Body reference frame

:math:`\tau` Time-constant

:math:`M` Torque

:math:`F` Force

Inertia
===================

Multi Body Modeling
---------------------
Using multi body analysis of the drone and its components, the total drone
(mass and moments) inertia is estimated, see :numref:`drone_multi_body`.
This results in the following inertia estimates

.. math::

    I_{est} =
    \begin{bmatrix}
        0.014 & 0 & 0 \\
        0 & 0.014 & 0 \\
        0 & 0 & 0.026
    \end{bmatrix} \text{kgm}^2

and :math:`m_{est}=1.071` kg. For comparison, the measured weight is :math:`m_{meas}=1.096` kg.
Also its center of mass w.r.t the top frame is located at :math:`CM_{est}=[0, -0.001, -0.009]` m.
Hence, a rather good weight distribution.

The Pendulum Experiment
------------------------
In `Design-Modeling-and-Control-of-an-Octocopter`_ (Section 2.2) an experiment is proposed for
estimation the rotational moment of inertia. This by measuring the period time of a rigid body
rotating about an axis suspended by two strings i.e., bifilar pendulum.

Without going into the details, :math:`I_{zz}` was estimated by suspending the drone by two
strings apart :math:`D=0.46` m with length :math:`L=2.16` m, see :numref:`pendulum_experiment`.

.. _pendulum_experiment:
.. figure:: figures/pendulum_experiment.jpg
    :width: 75%

    Ugglan suspended by two strings - help?!

In total 10 measurements where made of the number of periods and the total time, listed in
:numref:`table_pendulum`.

.. _table_pendulum:

.. table:: :math:`N` number of periods and :math:`t` total time.

    =======  =======
    N        t [s]
    =======  =======
    10       19.3
    10       19.4
    15       29.2
    15       29.1
    20       39.0
    20       38.8
    25       48.2
    25       48.2
    30       58.1
    30       57.8
    =======  =======

Computing the mean period yields :math:`T=1.94` s. This in terms results in
:math:`I_{zz}=0.025 \text{ kgm}^2`. This can be seen as an validation of the estimation
from the multi body modeling.

Motor Dynamics
================
The motor dynamics, :math:`u_{M_i}\rightarrow\omega_{M_iz}` are modelled as a simple first order
LTL system (low-pass filter)

.. math::

    \Omega_{M_iz}(s) = \frac{1}{\tau_M s + 1}U_{M_i}(s)

where :math:`\tau_M` is the motor time constant. The time constant can be estimated by analyzing
the systems step response. By assuming :math:`\omega_{M_i}(0) = u_0` and
:math:`\omega_{M_i}(\infty) = u_\infty`, before and after the step response as settled, one gets

.. math::

    \omega_{M_i}(t) = u_\infty + (u_0 - u_\infty) e^{\frac{-t}{\tau_M}}

which can be re-written as

.. math::

    \ln\left(\frac{u_\infty - u_0}{u_\infty - \omega_{M_i}(t)}\right) = \tau_M^{-1} t

assuming :math:`u_\infty > u_0`. By then measuring :math:`\omega_{M_i}` and :math:`t` one can
estimate :math:`\tau_M` by using least squares regression. Note the data points for the regression
should be selected such that the problem is well conditioned i.e.,
:math:`\omega_{M_i} \lesssim u_\infty`.

In :numref:`tau_motor_dynamics` the time constant is estimated. One can see that it does vary
and decrease with an increasing :math:`u_0`, introducing a non-linearity. Moreover,
:math:`\tau_M` varies for positive and negative steps.

The seen effects have to be neglected to keep linearity. Therefore, the resulting time constant
is given by the mean of the steps responses where :math:`\omega_M \in [400, 800]` i.e., within
normal operating conditions. This gives :math:`\tau_M = 0.08`.

.. _tau_motor_dynamics:
.. figure:: figures/tau_motor_dynamics.svg
    :width: 100%

    Positive and negative motor step responses (with mounted propeller). The time constant
    is estimated for each step using MLSE.

On a further note. The approximation of using a first order low pass filter is fine. But from the
figure it becomes clear that better approximations of higher order exist. An improvement would
be to empirical derive such a system - by using system identification.

Linearized State Space
=======================
The same linearized (SIMO) state-space representation as derived in
`Design-Modeling-and-Control-of-an-Octocopter`_ (Section 3.7) is used i.e.,

.. math::

    \mathbf{\dot{x}_i} &= \mathbf{A_i x_i} + \mathbf{B_i} u_i \\
    \mathbf{y_i} &= \mathbf{C_i x_i} + \mathbf{D_i} u_i, i = 1,2,...,6

where :math:`\mathbf{x_i}\in\mathbb{R}^3`, :math:`\mathbf{y_i}\in\mathbb{R}^2` and

.. math::
    :label: state_space_matrices

    \mathbf{A_i}=
    \begin{bmatrix}
        0 & 1 & 0 \\
        0 & 0 & c_i \\
        0 & 0 & -\tfrac{1}{\tau_M}
    \end{bmatrix},
    \mathbf{B_i}=
    \begin{bmatrix}
        0 \\
        0 \\
        \tfrac{1}{\tau_M}
    \end{bmatrix},
    \mathbf{C_i}=
    \begin{bmatrix}
        1 & 0 & 0 \\
        0 & 1 & 0
    \end{bmatrix},
    \mathbf{D_i}= 0.

Here :math:`\mathbf{c} = [\tfrac{1}{m}, \tfrac{1}{m}, \tfrac{1}{m}, \tfrac{1}{I_{xx}},
\tfrac{1}{I_{yy}}, \tfrac{1}{I_{zz}}]`. For instance :math:`i = 4` corresponds to the
state-space representation in roll

.. math::

    \mathbf{x_4}=
    \begin{bmatrix}
        \phi \\
        \dot\phi \\
        M_{\phi}
    \end{bmatrix},
    u_4=u_\phi,
    c_4=\tfrac{1}{I_{xx}}.

Note that the final state e.g., :math:`M_{\phi} (= I_{xx}\ddot\phi)` is observed using a reduced observer,
see :ref:`force-torque-estimation`.

State Estimation
=================

Attitude Estimation
--------------------
For attitude control, the Euler angles :math:`\eta = [\phi, \theta, \psi]` and their respective
time derivatives (angular rates) :math:`\dot{\eta} = \omega` (in the linearized model) are
needed.

Here :math:`\omega` is simply given by the IMU's gyro, but :math:`\eta` has to be estimated. This
can be accomplished using all sensors (accelerometer, gyro and magnetometer) and is a common
problem. Without going into detail - geometrical relationships yield

.. math::

    \phi_{acc} &= \text{atan2}(-a_{By}, -a_{Bz}) \\
    \theta_{acc} &= \text{atan2}(a_{Bx}, \sqrt{a_{By}^2 + a_{Bz}^2}) \\
    \psi_{mag} &= \text{atan2}(-B_{Iy}, B_{Ix})

where

.. math::

    B_{Ix} &= B_{Bx}\cos(\theta) + B_{By}\sin(\phi)\sin(\theta) + B_{Bz}\sin(\theta)\cos(\phi) \\
    B_{Iy} &= B_{By}\cos(\phi) - B_{Bz}\sin(\phi)

and :math:`a_B` is the acceleration and :math:`B_B` is the earths magnetic field supplied by the
IMU in the body frame. Where :math:`B_{Ix}` and :math:`B_{Iy}` are the magnetic fields in the
inertial frame.

These estimates can be improved by using the gyro and a simple first order complementary
filter (see `IMU Data Fusing <http://www.olliw.eu/2013/imu-data-fusing/>`_)

.. math::

    \tilde{\phi}^{k} &= \text{cf}(\phi_{acc}^k, \dot{\phi}_{gyro}^k, \tilde{\phi}^{k-1}, \tau_{\phi}) \\
    \tilde{\theta}^{k} &= \text{cf}(\theta_{acc}^k, \dot{\theta}_{gyro}^k, \tilde{\theta}^{k-1}, \tau_{\theta}) \\
    \tilde{\psi}^{k} &= \text{cf}(\psi_{mag}^k, \dot{\psi}_{gyro}^k, \tilde{\psi}^{k-1}, \tau_{\psi})

where

.. math::

    y^{k} &= \text{cf}(u^k, \dot{u}^k, y^{k-1}, \tau) \\
            &= \alpha(y^{k-1} + \dot{u}^k\Delta t) + (1-\alpha)u^k

where :math:`\alpha = \tfrac{\tau}{\tau + \Delta t}` and :math:`\tau` is the cut-off frequency
(:math:`\tau = \tfrac{1}{2 \pi f_c}`). Note that the estimates also need range limiting
(module of angles) and offset compensation (gyro and hard iron).

In :numref:`attitude_estimation` the result of the attitude estimation is shown. Note the large
drift of the gyro.

.. _attitude_estimation:
.. figure:: figures/attitude_estimation.svg
    :width: 100%

    Attitude estimation of roll (:math:`\phi`), pitch (:math:`\theta`) and
    yaw (:math:`\psi`). Both unfiltered and complementary filter estimates.
    Here :math:`\tau_{\phi}=\tau_{\theta}=\tau_{\psi}` s.t. :math:`f_c=20` Hz.

Hard Iron Offset
^^^^^^^^^^^^^^^^^
Magnetic fields affecting the magnetometer other than earth's need to be compensated
for. One of those is hard iron (the other being soft iron) effects. These are static
magnetic fields e.g., components on the PCB.

These offsets can easily be estimated using least squares (offset of a sphere)

.. math::

    \underset{V}{\text{minimize}} (B-V)^T (B-V) = B^2.

To get a good estimate, the magnetometer should be rotated in space to excite all
directions, see :numref:`hard_iron_offset`.

.. _hard_iron_offset:
.. figure:: figures/hard_iron_offset.svg
    :width: 100%

    Hard iron offset estimation and correction.

.. _force-torque-estimation:

Force and Torque Estimation
----------------------------
The final state of :eq:`state_space_matrices` (force/torque), here denoted
:math:`\tilde{x}_a` is not measured. It is instead estimated using a reduced
observer, see `Design-Modeling-and-Control-of-an-Octocopter`_ Section (4.2.3)
for details.

A first order backwards time-difference of the observer gives

.. math::

    \tilde{x}^k_a = \beta_1 (\tilde{x}^{k-1}_a -\alpha x^{k-1}_v) +
    \beta_2 x^{k}_v + \beta_3 u^k, \tilde{x}^0_a = 0

where

.. math::

    \beta_1 &= \frac{\tau_M}{\beta_4} \\
    \beta_2 &= -\frac{\alpha\Delta t (1+ \alpha \tau_M c_i)}{\beta_4} \\
    \beta_3 &= \frac{\Delta t}{\beta_4} \\
    \beta_4 &= \tau_M + \Delta t (1 + \alpha \tau_M c_i)

and :math:`x_v` is the (translational/rotational) velocity state of each
state-space, :math:`\alpha` a tuning parameter. Note that
:math:`\text{Re}(\alpha) > -\tfrac{1}{\tau_M c_i}` for stability.

Also note that when :math:`\alpha\rightarrow\infty` the observer will result
in a backwards time-difference of :math:`x_v` i.e., its derivative
(:math:`\dot{x}_v = x_a`).

State Control
=================
The drone's dynamics are stabilized using a full state feedback controller

.. math::
    :label: cont_state_feedback

    u = -\mathbf{Lx}.

Which allows for arbitrary pole placement, see `Design-Modeling-and-Control-of-an-Octocopter`_
for in depth details.

Discretized Feedback
---------------------
In :ref:`force-torque-estimation` the reduced observer is presented, including the control
input. Hence, by using :eq:`cont_state_feedback` and solving for :math:`u_k` one derives at the
final time-discretized feedback controller

.. math::
    :label: disc_state_feedback

    u^k = -\frac{l_1 x_1^k + (l_2 + l_3(\beta_2 + \alpha)) x_2^k + l_3\beta_1(u^{k-1}
    - \alpha x_2^{k-1})}{1 + l_3\beta_3}, u^0 = 0.

Pilot Control
---------------------
Typically, the pilot controller (using the handheld controller), seeks for stability
of roll, pitch and yaw-rate.

To tune :math:`\mathbf{L}` and :math:`\alpha` (reduced observer), the step response of
the closed loop is analyzed, see :numref:`tune_ctrl_roll_pitch`-:numref:`tune_ctrl_yaw_rate`.

Note, an integrated state has been added for roll and pitch control (already in place
for yaw-rate) i.e., :math:`\mathbf{x}\in\mathbb{R}^4`.

.. _tune_ctrl_roll_pitch:
.. figure:: figures/tune_ctrl_roll_pitch.svg
    :width: 100%

    Tuning and parameter selection for roll and pitch control.

.. _tune_ctrl_yaw_rate:
.. figure:: figures/tune_ctrl_yaw_rate.svg
    :width: 100%

    Tuning and parameter selection for yaw-rate control.

In the above figures :math:`u` corresponds to :eq:`cont_state_feedback` and
:math:`\tilde{u}` to :eq:`disc_state_feedback`. The abs-max value of the integrated
state (reference error) is also shown to determine a suitable value for anti-windup
i.e., to handle a large overshoot/instability caused by the integration.

In :numref:`rc_gimbal_def_pilot` the RC-gimbal definitions for the pilot controller
are given.

.. _rc_gimbal_def_pilot:
.. figure:: figures/rc_gimbal_def_pilot.svg
    :width: 75%

    RC-gimbal definitions for the pilot controller. The arrow indicates
    positive direction. Note that thrust implies negative :math:`F_z` i.e., lift.

Motor Control
=================
The body force and torque control inputs :math:`u_{Bz}`, :math:`u_{B\phi}`,
:math:`u_{B\theta}` and :math:`u_{B\psi}` have to be converted to individual
motor control inputs :math:`u_{M_i}`.

.. _motor_rotations:
.. figure:: figures/motor_rotations.svg
    :width: 75%

    The drone motor rotational directions.

From :numref:`motor_rotations` one can derive the drone body forces and
torques generated by the motors

.. math::

    F_{Bx} &= 0 \\
    F_{By} &= 0 \\
    F_{Bz} &= - F_{M_1z} - F_{M_2} - F_{M_3} - F_{M_4z} \\
    M_{Bx} &= - l_xF_{M_1z} - l_xF_{M_2z} + l_xF_{M_3z} + l_xF_{M_4z} \\
    M_{By} &=   l_xF_{M_1z} - l_xF_{M_2z} - l_xF_{M_3z} + l_xF_{M_4z} \\
    M_{Bz} &= - M_{M_1z} + M_{M_2z} - M_{M_3z} + M_{M_4z}

where :math:`F_{M_ix} = F_{M_iy} = M_{M_ix} = M_{M_iy} = 0` and
:math:`l_x = 0.23` m (distance between body center of mass and motor).

In order to solve for the motor inputs one can use the fact that
:math:`F, M \propto \omega^2`, where :math:`\omega` is the angular rate of a
motor/propeller, see :numref:`ang_rate_sq_vs_thrust`.

.. _ang_rate_sq_vs_thrust:
.. figure:: figures/ang_rate_sq_vs_thrust.svg
    :width: 100%

    Motor angular rate and its corresponding generated thrust. Positive
    rotation corresponds to the "intended" propeller rotation i.e., not
    driven in reverse. Fitted a 1st-order polynomial :math:`y = X[p_1]^\intercal`.

Hence, the generated body forces and torques can be described as following

.. math::

    \begin{bmatrix}
        F_{Bz} \\
        M_{Bx} \\
        M_{By} \\
        M_{Bz}
    \end{bmatrix} =
    \underbrace{
        \begin{bmatrix}
            -c_{Fz} & -c_{Fz} & -c_{Fz} & -c_{Fz} \\
            -l_xc_{Fz} & -l_xc_{Fz} & l_xc_{Fz} & l_xc_{Fz} \\
            l_xc_{Fz} & -l_xc_{Fz} & -l_xc_{Fz} & l_xc_{Fz} \\
            -c_{Mz} & c_{Mz} & -c_{Mz} & c_{Mz}
        \end{bmatrix}
    }_H
    \begin{bmatrix}
        \omega_{M_1z}^2 \\
        \omega_{M_2z}^2 \\
        \omega_{M_3z}^2 \\
        \omega_{M_4z}^2
    \end{bmatrix}.


By computing :math:`H^{-1}` one gets

.. math::

    [\omega_{M_1z}^2, \omega_{M_2z}^2, \omega_{M_3z}^2, \omega_{M_4z}^2]^\intercal
    = H^{-1} [F_{Bz}, M_{Bx}, M_{By}, M_{Bz}]^\intercal

where

.. math::
    H^{-1} = \frac{1}{4}
        \begin{bmatrix}
            -\tfrac{1}{c_{Fz}} & -\tfrac{1}{l_xc_{Fz}} & \tfrac{1}{l_xc_{Fz}} & -\tfrac{1}{c_{Mz}} \\
            -\tfrac{1}{c_{Fz}} & -\tfrac{1}{l_xc_{Fz}} & -\tfrac{1}{l_xc_{Fz}} & \tfrac{1}{c_{Mz}} \\
            -\tfrac{1}{c_{Fz}} & \tfrac{1}{l_xc_{Fz}} & -\tfrac{1}{l_xc_{Fz}} & -\tfrac{1}{c_{Mz}} \\
            -\tfrac{1}{c_{Fz}} & \tfrac{1}{l_xc_{Fz}} & \tfrac{1}{l_xc_{Fz}} & \tfrac{1}{c_{Mz}}
        \end{bmatrix}.

From :numref:`ang_rate_sq_vs_thrust` it can be seen that :math:`c_{Fz}` is smaller (about half)
when the motor is reversing (negative rotation). This is probably due to the non-symmetrical
shape of the propeller. Hence, a non-linearity arises and :math:`H^{-1}` can't solely be used.
Therefore reversing will for now not be used, maybe in the future.

Anyhow, :numref:`ang_rate_sq_vs_thrust` also gives :math:`c_{Fz} = -8.37\times 10^{-6}` (positive rotation
from now on only). The torque constant is given by :math:`c_{Mz} = \tfrac{1}{50} c_{Fz}` - empirical
relation from the master thesis.

In :numref:`ang_rate_vs_command` the empirical relation between the raw motor
control inputs and the angular rates is given.

.. _ang_rate_vs_command:
.. figure:: figures/ang_rate_vs_command.svg
    :width: 100%

    The motor angular rates and raw control inputs. Fitted a 1nd-order polynomial
    :math:`y = X[p_1, p_0]^\intercal`. Note, first 5 values are not included in the
    regression for a better fit - not a commonly used interval. Also note the
    symmetry about :math:`u`.

Hence, the final conversion is given by

.. math::
    u_{M_i} =
    \begin{cases}
        57\omega_{M_iz} - 9675 & \text{if } \omega_{M_iz} \geq 200 \\
        0 & \text{otherwise}
    \end{cases}.

Note, :math:`u_{M_i}` should also be range limited since is it a ``int16`` and reversing
is not used.
