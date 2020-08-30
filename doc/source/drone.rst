Drone
*****************
The drone a.k.a "Ugglan" (Swedish for owl) is a DIY build
quadcopter, using the Raspberry Pi Zero.

Some in-depth parts of the Modeling, State Estimation and Control
are not covered in this document, for details see the arthur's master thesis
`Design, Modeling and Control of an Octocopter <http://www.diva-portal.org/smash/get/diva2:857660/FULLTEXT01.pdf>`_.

Software
=================

Tasks
---------------
The drone's top level software design can be seen as a task manager, (asynchronous
and synchronous) solving the producer and consumer problem. Where the producers are
the sensors (IMU, ESC and RC). And where the control loop, data logging etc. are
the consumers, see :numref:`drone_sw_design`

.. _drone_sw_design:
.. mermaid::
    :caption: Overview of the drone software design. Tasks are stadium-shaped nodes.

    graph LR
        ImuAccMag([ImuAccMag])
        ImuGyro([ImuGyro])
        ImuPres([ImuPres])

        EscRead_i([EscRead_i])
        RcReceiver([RcReceiver])

        Inputs((Inputs))
        InputHolder[InputHolder]
        DataLogQueue[(DataLogQueue)]

        StateControl([StateControl])
        DataLogger([DataLogger])

        ImuAccMag -- 50 Hz --> Inputs
        ImuGyro -- 50 Hz --> Inputs
        ImuPres -- 10 Hz --> Inputs
        EscRead_i -- 10 Hz --> Inputs
        RcReceiver -- 30 Hz --> Inputs

        Inputs --> InputHolder
        Inputs --> DataLogQueue

        InputHolder -- 50 Hz--> StateControl
        DataLogQueue -- 100 Hz --> DataLogger

        subgraph Producers
        ImuAccMag
        ImuGyro
        ImuPres
        EscRead_i
        RcReceiver
        end

        subgraph Consumers
        StateControl
        DataLogger
        end

As one can see, some consumers will fetch data from the ``InputHolder``. This is
a thread safe data structure which holds the latest input samples e.g., used by the
``StateControl`` which runs at a constant execution/sample rate i.e. to simplify
the signal processing. Whereas the data logger only will store a sample once and uses
the thread safe queue ``DataLogQueue``. Note that other tasks may also populate this
queue.

Note, some signals such as the ones from the pressure sensor will only be sampled
at 10 Hz. This has to be handled by the state controller.

Data Logging
-----------------
The ``DataLogger`` handles the data serialization of signals e.g., the IMU acceleration
which is continuously written to disk. It consists of a **HEADER** section and a **DATA**
section. The **HEADER** is a json-file describing the **DATA** section. See
:numref:`data_log_protocol` for an illustration of the data protocol.

.. _data_log_protocol:
.. figure:: figures/data_log_protocol.svg
    :width: 100%

    The data logging protocol. The **SIGNAL ID** is an unique identifer for each signal
    and of type ``uint16``. The **REL TIMESTAMP** is the relative timestamp in ms
    between each **PAYLOAD** and of type ``uint8``.

The json-file is compressed using gzip (and base64 encoded) to save space. See example
header below.

.. code-block:: json

    {
        "start_time": "1990-08-30T22:52:50Z",
        "types": {
            "0": "UINT8",
            "1": "UINT16",
            "2": "UINT32",
            "3": "SINT8",
            "4": "SINT16",
            "5": "SINT32",
            "6": "FLOAT",
            "7": "DOUBLE"
        },
        "groups": {
            "0": "IMU",
            "1": "ESC"
        },
        "signals": {
            "0": {
                "name": "AccelerationX",
                "group": 0,
                "type": 7
            },
            "1": {
                "name": "Status0",
                "group": 1,
                "type": 0
            }
        }
    }

Hardware
=================
.. _ugglan_in_person:
.. figure:: figures/ugglan_in_person.jpg
    :width: 50%

    Ugglan in person.

The drone hardware components are is listed below

* Raspberry Pi Zero
* Diatone Q450 with PCB
* Pololu AltIMU-10 v4
* Afro ESC 20 A
* Turnigy Evolution Digital AFHDS 2A RC transmitter & controller
* TGY-iA6C RC receiver
* ZIPPY Compact 3300mAh 3S (or similar)
* DC-DC step down voltage regulator 5V
* Turnigy 2830 900KV L2215J-900 Brushless Motor

In addition, miscellaneous self manufactured components such as a cut plexiglas
are used for mounting, see :numref:`ugglan_in_person`.

Devices & Busses
-----------------
.. _connected_busses:
.. mermaid::
    :caption: Overview of the hardware devices connected to the Pi Zero and their respective protocols.

    graph TD
        Esc_i -- i2c read --> Raspi
        Raspi -- i2c write --> Esc_i
        Imu_i -- i2c read --> Raspi
        RcReceiver -- uart read --> Raspi

Modeling
===============
TODO: Summary of master thesis work.

Moment of Inertia
------------------
TODO: Simulation & Empirical.

Motor Dynamics
------------------
TODO: Empirical Studies.

State Estimation
=================

Attitude Estimation
--------------------
For attitude control, the Euler angles :math:`\eta = [\phi, \theta, \psi]` and their respective
time derivatives (angular rates) :math:`\dot{\eta} = \omega` have to be estimated.

By using the IMU (accelerometer, gyro and magnetometer), :math:`\eta` and :math:`\dot{\eta}` can
easily be estimated. This is common problem and without going into detail - geometrical
relationships yield

.. math::

    \phi_{acc} &= \text{atan2}(-a_y, -a_z) \\
    \theta_{acc} &= \text{atan2}(a_x, \sqrt{a_y^2 + a_z^2}) \\
    \psi_{mag} &= \text{atan2}(-B_{fy}, B_{fx})

where

.. math::

    B_{fx} &= m_x\cos(\theta) + m_y\sin(\phi)\sin(\theta) + m_z\sin(\theta)\cos(\phi) \\
    B_{fy} &= m_y\cos(\phi) - m_z\sin(\phi)

and :math:`a` is the acceleration and :math:`m` is the earths magnetic field supplied by the
IMU. These estimates can be improved by using the gyro and a simple first order complementary
filter

.. math::

    \tilde{\phi}^{k+1} &= \text{cf}(\phi_{acc}^k, \dot{\phi}_{gyro}^k, \tilde{\phi}^k, \tau_{\phi}) \\
    \tilde{\theta}^{k+1} &= \text{cf}(\theta_{acc}^k, \dot{\theta}_{gyro}^k, \tilde{\theta}^k, \tau_{\theta}) \\
    \tilde{\psi}^{k+1} &= \text{cf}(\psi_{mag}^k, \dot{\psi}_{gyro}^k, \tilde{\psi}^k, \tau_{\psi})

where

.. math::

    y^{k+1} &= \text{cf}(u^k, \dot{u}^k, y^k, \tau) \\
            &= \alpha(y^k + \dot{u}^k\Delta t) + (1-\alpha)u^k

where :math:`\alpha = \tfrac{\tau}{\tau + \Delta t}` and :math:`\tau` is the cut-off frequency.
Note the estimates also need range limiting (module of angles) and offset compensation.

Motor Torque Estimation
------------------------
TODO: Reduced observer. Summary of master thesis work.

Control
=================
TODO: Summary of master thesis work. With flow chart.

State Control
-----------------
TODO: Summary of master thesis work.

Motor Control
------------------
The body force and torque control inputs :math:`u_z^{body}`, :math:`u_\phi^{body}`,
:math:`u_\theta^{body}` and :math:`u_\psi^{body}` have to be converted to individual
motor control inputs :math:`u^{motor_i}`. From Figure X one can derive the drone body
forces and torques generated by the motors

.. math::

    f_x^{body} &= 0 \\
    f_y^{body} &= 0 \\
    f_z^{body} &= - f_z^{motor_1} - f_z^{motor_2} - f_z^{motor_3} - f_z^{motor_4} \\
    m_x^{body} &= - l_xf_z^{motor_1} - l_xf_z^{motor_2} + l_xf_z^{motor_3} + l_xf_z^{motor_4} \\
    m_y^{body} &=   l_xf_z^{motor_1} - l_xf_z^{motor_2} - l_xf_z^{motor_3} + l_xf_z^{motor_4} \\
    m_z^{body} &= - m_z^{motor_1} + m_z^{motor_2} - m_z^{motor_3} + m_z^{motor_4}

where :math:`f_x^{motor_i} = f_y^{motor_i} = m_x^{motor_i} = m_y^{motor_i} = 0` and
:math:`l_x = 0.23` [m] (distance between body center of mass and motor).

In order to solve for the motor inputs one can use the fact that
:math:`f, m \propto \omega^2`, where :math:`\omega` is the angular rate of a
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
        f_z^{body} \\
        m_x^{body} \\
        m_y^{body} \\
        m_z^{body}
    \end{bmatrix} =
    \underbrace{
        \begin{bmatrix}
            -c_{fz} & -c_{fz} & -c_{fz} & -c_{fz} \\
            -l_xc_{fz} & -l_xc_{fz} & l_xc_{fz} & l_xc_{fz} \\
            l_xc_{fz} & -l_xc_{fz} & -l_xc_{fz} & l_xc_{fz} \\
            -c_{mz} & c_{mz} & -c_{mz} & c_{mz}
        \end{bmatrix}
    }_H
    \begin{bmatrix}
        {\omega_z^{motor_1}}^2 \\
        {\omega_z^{motor_2}}^2 \\
        {\omega_z^{motor_3}}^2 \\
        {\omega_z^{motor_4}}^2
    \end{bmatrix}.


By computing :math:`H^{-1}` one gets

.. math::

    [{\omega_z^{motor_1}}^2, {\omega_z^{motor_2}}^2, {\omega_z^{motor_3}}^2, {\omega_z^{motor_4}}^2]^\intercal
    = H^{-1} [f_z^{body}, m_x^{body}, m_y^{body}, m_z^{body}]^\intercal

where

.. math::
    H^{-1} = \frac{1}{4}
        \begin{bmatrix}
            -\tfrac{1}{c_{fz}} & -\tfrac{1}{l_xc_{fz}} & \tfrac{1}{l_xc_{fz}} & -\tfrac{1}{c_{mz}} \\
            -\tfrac{1}{c_{fz}} & -\tfrac{1}{l_xc_{fz}} & -\tfrac{1}{l_xc_{fz}} & \tfrac{1}{c_{mz}} \\
            -\tfrac{1}{c_{fz}} & \tfrac{1}{l_xc_{fz}} & -\tfrac{1}{l_xc_{fz}} & -\tfrac{1}{c_{mz}} \\
            -\tfrac{1}{c_{fz}} & \tfrac{1}{l_xc_{fz}} & \tfrac{1}{l_xc_{fz}} & \tfrac{1}{c_{mz}}
        \end{bmatrix}.

From :numref:`ang_rate_sq_vs_thrust` it can be seen that :math:`c_{fz}` is smaller (about half)
when the motor is reversing (negative rotation). This is probably due to the non-symmetrical
shape of the propeller. Hence, a non-linearity arises and :math:`H^{-1}` can't solely be used.
Therefore reversing will for now not be used, maybe in the future.

Anyhow, :numref:`ang_rate_sq_vs_thrust` also gives :math:`c_{fz} = -8.37\times 10^{-6}` (positive rotation
from now on only). The torque constant is given by :math:`c_{mz} = \tfrac{1}{50} c_{fz}` - empirical
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
    u^{motor_i} =
    \begin{cases}
        57\omega_z^{motor_i} - 9675 & \text{if } {\omega_z^{motor_i}} > 0 \\
        0 & \text{otherwise}
    \end{cases}.

Note, :math:`u_i` should also be range limited since is it a ``int16`` and reversing
is not used.
