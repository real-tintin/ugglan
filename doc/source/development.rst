Development
*****************
Test driven development is emphasized in this project.

.. _py_unit_testing:

Python Unit Testing
====================
The unit test framework ``pytest`` is used for unit testing of the Python
code (mainly used for tools). To execute all tests (and build pkg)::

    make -C ./tools -f Makefile

C++ Unit Testing
=================
The drone core is written in C++. To test and verify its functionality,
the test framework `Catch 2 <https://github.com/catchorg/Catch2>`_ is used.

To run all unit tests in docker::

    make -C ./raspi -f Makefile_tests

ESC Compile & Flash
=====================
The drone uses the Afro ESC (Electronic Speed Controllers) to control the
brushless motors. They allow for communication via i2c and motor feedback such
as its angular-rate, temperature and voltage.

In order to enable the i2c feedback and update its address, the onboard Atmel AVR
Atmega8 have to be re-flashed. For flashing, the `Afro ESC USB programming tool <https://hobbyking.com/en_us/afro-esc-usb-programming-tool.html>`_
as well as the `USBasp programer for Atmel AVR controllers <https://www.fischl.de/usbasp/>`_
can be used. Note, the Afro USB programmer only works if the ESC has been pre-flashed
with the bootloader for PWM flashing.

To compile setup ``avra`` (https://github.com/Ro5bert/avra) and for flashing
``avrdude`` (http://savannah.nongnu.org/projects/avrdude). BlueRobotics have
manipulated the ESC source code and enabled for i2c feedback. Their Git repo has
been forked e.g., use Git submodules::

    git submodule add https://github.com/real-tintin/tgy

For simplified compiling and flashing, use the included `makefile` or the
`KKMulitCopter GUI <https://lazyzero.de/en/modellbau/kkmulticopterflashtool>`_.

Note, to change the default rotation direction, modify ``MOTOR_REVERSE`` in *tgy.asm*.

Setup Env on Target
====================
Todo's before deploying on target (Raspbian).

External devices
----------------
In :ref:`devices_and_busses` the external devices and busses are discussed. To enabled and
configure the needed pins on the Pi, add the following to the end of ``/boot/config.txt``::

    # Enable HW i2c in fast mode (400 kHz)
    dtparam=i2c=on
    dtparam=i2c_baudrate=400000

    # Enable SW i2c in normal mode (100 kHz)
    dtoverlay=i2c-gpio,bus=4,i2c_gpio_delay_us=2,i2c_gpio_sda=23,i2c_gpio_scl=24

    # Enable UART
    enable_uart=1

Install GCC 9.1
----------------
The C++ application is compiled using gcc 9.1 (for C++17) and needs to be installed on the Pi
(dependent libs). See ``./raspi/Dockerfile_raspi`` for details.

Remote access
--------------
Setup wpa supplicant and interfaces for remote access e.g., connect to a hotspot.

Build for & Deploy on Target
=============================
To build for target, the source is cross compiled in a Raspbian Docker container::

    make -C ./raspi -f Makefile_dpkg

This will also build a Debian package (``ugglan.deb``) which can easily be deployed on
target by using::

    dpkg --install path/to/ugglan.deb

Tools
======
For development various tools are made available e.g., tuning of the state
controller. These are written in Python and bundled in a package. To install
see :ref:`py_unit_testing`.

Beside the possibility to importing and use the modules, some useful cli's
(console scripts) are made available e.g., plotting and analysis of a data
log file::

    plot-data-log path/to/file.dat

Python Requirements
====================
When new Python packages are needed and installed, the ``requirements.txt`` shall be
updated accordingly. Under python virtual env::

    py -m pip freeze > requirements.txt
