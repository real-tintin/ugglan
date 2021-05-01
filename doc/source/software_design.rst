Software Design
*****************

Tasks
=================
The drone's top level software design can be seen as a task manager, (asynchronous
and synchronous) solving the producer and consumer problem. Where the producers are
the sensors (IMU, ESC etc.). And where the control loop, data logging etc. are
the consumers, see :numref:`drone_sw_design`.

.. _drone_sw_design:
.. mermaid::
    :caption: Overview of the drone software design. Tasks are stadium-shaped nodes.

    graph LR
        ImuAccMag([ImuAccMag])
        ImuGyro([ImuGyro])
        ImuPres([ImuPres])

        EscRead([EscRead])
        RcReceiver([RcReceiver])

        DataLogQueue[(DataLogQueue)]

        StateControl([StateControl])
        EscWrite([EscWrite])
        DataLogger([DataLogger])

        ImuAccMag -- push 50 Hz --> DataLogQueue
        ImuGyro -- push 50 Hz --> DataLogQueue
        ImuPres -- push 12.5 Hz --> DataLogQueue
        EscRead -- push 5 Hz --> DataLogQueue
        RcReceiver -- push 50 Hz --> DataLogQueue

        DataLogQueue -- last value 50 Hz --> StateControl
        DataLogQueue -- last value 50 Hz --> EscWrite
        DataLogQueue -- pop 100 Hz --> DataLogger

        subgraph Producers
        ImuAccMag
        ImuGyro
        ImuPres
        EscRead
        RcReceiver
        end

        subgraph Consumers
        StateControl
        EscWrite
        DataLogger
        end

As one can see, the ``DataLogQueue`` maintains a thread safe queue for the producers to
push to and the ``DataLogger`` to pop from. But it also stores the last (pushed) sample
for consumers to use e.g., the ``StateControl`` which runs at a constant execution/sample
rate i.e., to simplify the signal processing. Note, other tasks than producers may populate
the queue e.g., estimated states which are useful for offline tuning of control laws.

Note, some signals such as the ones from the pressure sensor will only be sampled
at 12.5 Hz. This has to be handled by the state controller.

Data Logging
=================
The ``DataLogger`` handles the data serialization of signals e.g., the IMU acceleration
which is continuously written to disk. It consists of a **HEADER** section - a json string,
describing the content of the second **DATA PACKAGES** section. See :numref:`data_log_protocol`
for an illustration of the data-log protocol.

.. _data_log_protocol:
.. figure:: figures/data_log_protocol.svg
    :width: 100%

    The data logging protocol. The **SIGNAL ID** is an unique identifer for each signal/package
    and of type ``uint16``. The **REL TIMESTAMP** is the relative timestamp in ms
    between each **PACKAGE** and of type ``uint8``.

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

State Machine
=================

User Operator
---------------

.. mermaid::
    :caption: User operation of ESC's. LS: Left Switch. MS: Middle Switch.

    stateDiagram

        [*] --> Sound
        Sound --> Disarmed
        Disarmed --> Armed: LS Mid
        Armed --> Disarmed: LS Hi
        Armed --> Alive: LS Lo
        Alive --> Armed: LS Mid
        Disarmed --> [*]: MS Lo
