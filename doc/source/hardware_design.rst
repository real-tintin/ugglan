Hardware Design
*****************
.. _ugglan_in_person:
.. figure:: figures/ugglan_in_person.jpg
    :width: 75%

    Ugglan in person.

Components
==============
The drone hardware components are is listed below

* Raspberry Pi Zero 2 W
* Diatone Q450 with PCB
* Pololu AltIMU-10 v4
* Afro ESC 20 A
* Turnigy Evolution Digital AFHDS 2A RC transmitter & controller
* TGY-iA6C RC receiver
* ZIPPY Compact 3300mAh 3S (or similar)
* DC-DC step down voltage regulator 5V
* T-MOTOR MN2212-V2.0 Brushless Motor (KV920)
* T-MOTOR T9545-A Propeller 9.5x4.5

In addition, miscellaneous self manufactured components such as a cut plexiglas
are used for mounting, see :numref:`ugglan_in_person`.

.. _devices_and_busses:

Devices & Busses
==================
The IMU's and ESC's are communicating with the Pi over i2c. The IMU can run at 400 kHz (fast mode)
and is using the built-in HW. But, the ESC's only run stable at 100 kHz (normal mode) and are
therefore using a SW implementation (i2c-gpio overlay, bit-banging over GPIO 23-24). The RC receiver
is communicating over UART, a serial connection. See overview in :numref:`connected_busses`.

.. _connected_busses:
.. mermaid::
    :caption: Overview of the hardware devices connected to the Pi Zero 2 W and their respective protocols.

    graph TD
        Esc_i -- i2c read 100 kHz --> Raspi
        Raspi -- i2c write 100 kHz --> Esc_i
        Imu_i -- i2c read 400 kHz --> Raspi
        RcReceiver -- uart read 115200 bps --> Raspi

Wiring
==================
.. _wiring_diagram:
.. figure:: figures/wiring_diagram.svg
    :width: 100%

    Wiring diagram.

Single core vs. Multi core
============================
During the project, the Raspberry Pi Zero single core processor was upgraded to a 2 W multi core
processor. As the application is utilizing threading, it was interesting to compare the
application performance between the two.

In :numref:`task_exec_rate_single_core` and :numref:`task_exec_rate_multi_core` the task execution
rates in time is shown. As one can see, the execution rates on the multi core processor are much smoother.

.. _task_exec_rate_single_core:
.. figure:: figures/task_exec_rate_single_core.svg
    :width: 100%

    Task execution rates on the single core Raspberry Pi Zero.

.. _task_exec_rate_multi_core:
.. figure:: figures/task_exec_rate_multi_core.svg
    :width: 100%

    Task execution rates on the multi core Raspberry Pi Zero 2 W.

Vibrations & Attenuation
=========================
The rotating motor with propeller is a source for mechanical vibrations. Un-balanced motors and propellers
will induce vibrations and can cause issue during state estimation e.g., the vibration will propagate thru
the frame to the IMU and cause un-wanted noise.

Motor & Propeller Balancing
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
To reduce vibrations the motor and propeller need to be balanced. To analyze and balance them the IMU acceleration
can be studied. In :numref:`vib_analysis_prop_balancing` the IMU acceleration in time, its variance and psd (power
spectral density) is shown for different runs where a pice of tape is positioned at different locations on a propeller.

.. _vib_analysis_prop_balancing:
.. figure:: figures/vib_analysis_prop_balancing.svg
    :width: 100%

    Each chunk corresponds to a separate run of the motors where the pice of tape is relocated. Note,
    chunk 0 is without any tape, chunk 1-3 is tape on one blade and chunk 4-6 on the other blade.

It can be seen that applying tape on a certain location can significantly reduce vibrations i.e., becoming
"better" balanced.

Also note the frequency components of the psd. The content at ~20 Hz could correspond to the frequency of
the motors but folded (due to aliasing as the sample rate 100/2 Hz < 81 Hz, hence at folded at 19 Hz).

Vibration Dampers
^^^^^^^^^^^^^^^^^^
The vibrations experienced by the IMU can be further reduced by installing vibrations dampers between
the frame and the custom mounting plate, see :numref:`ugglan_in_person`.

Different rubber dampers were tested (varying stiffness, mass and size), see :numref:`table_vibration_dampers`
for results.

.. _table_vibration_dampers:

.. table:: * See `data sheet <https://www.tme.eu/Document/3783353bba387d7e09757c85b47027b3/DVA1-3-EN.pdf>`_ for details.

    ====================    ================    ========    ===================     ===================
    Name*                   Stiffness [N/mm]    Mass [g]    Eigenfrequency [Hz]     Variance [m^2/s^4]
    ====================    ================    ========    ===================     ===================
    DVA.1-15-15-M4-10-55    39                  10          314                     1.12
    DVA.1-15-15-M4-10-70    118                 10          537                     0.85
    DVA.1-15-20-M4-10-40    20                  12          206                     2.40
    DVA.1-15-20-M4-10-55    25                  12          230                     1.30
    ====================    ================    ========    ===================     ===================

The above results are rather concise - increasing eigenfrequency will result in less experienced vibrations
by the IMU. This is most likely due the fact that the plate is vibrating about the motor rotational frequency
(~160 Hz). Hence, it would be interesting to test a damper with an eigenfrequency << 160 Hz.

Note, the eigenfrequency is estimated by assuming a classic second order mass-spring system i.e.,
:math:`f_n = \tfrac{1}{2\pi}\sqrt{\tfrac{k}{m}}` (SI-units).
