Hardware Design
*****************
.. _ugglan_in_person:
.. figure:: figures/ugglan_in_person.jpg
    :width: 75%

    Ugglan in person.

Components
==============
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

.. _devices_and_busses:

Devices & Busses
==================
The IMU's and ESC's are communicating with the Pi over i2c. The IMU can run at 400 kHz (fast mode)
and is using the built-in HW. But, the ESC's only run stable at 100 kHz (normal mode) and are
therefore using a SW implementation (i2c-gpio overlay, bit-banging over GPIO 23-24). The RC receiver
is communicating over UART, a serial connection. See overview in :numref:`connected_busses`.

.. _connected_busses:
.. mermaid::
    :caption: Overview of the hardware devices connected to the Pi Zero and their respective protocols.

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
