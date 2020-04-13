Development
*****************
Test driven development is emphasized in this project.

C++ Unit Testing
=================
The drone core is written in C++. To test and verify its functionality,
the test framework `Catch 2 <https://github.com/catchorg/Catch2>`_ is used.

To run all unit tests::

    ./drone/tests/run_catch_tests.sh


ESC Compile & Flash
=====================
The drone uses the Afro ESC (Electronic Speed Controllers) to control the
brushless motors. They allow for communication via i2c and motor feedback such
as rpm, temperature and voltage.

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
