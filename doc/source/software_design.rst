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
        EscWrite([EscWrite])

        RcReceiver([RcReceiver])

        StateEst([StateEst])
        StateCtrl([StateCtrl])

        DataLogger([DataLogger])

        DataLogQueue[(DataLogQueue)]

        ImuAccMag -- push 100 Hz --> DataLogQueue
        ImuGyro -- push 100 Hz --> DataLogQueue
        ImuPres -- push 10 Hz --> DataLogQueue
        EscRead -- push 1 Hz --> DataLogQueue
        RcReceiver -- push 100 Hz --> DataLogQueue

        DataLogQueue -- last value 100 Hz --> StateEst
        DataLogQueue -- last value 100 Hz --> StateCtrl
        DataLogQueue -- last value 100 Hz --> EscWrite
        DataLogQueue -- last value 100 Hz --> StreamerServer
        DataLogQueue -- pack 10 Hz --> DataLogger

        subgraph Producers
        ImuAccMag
        ImuGyro
        ImuPres
        EscRead
        RcReceiver
        end

        subgraph Consumers
        StateEst
        StateCtrl
        EscWrite
        DataLogger
        StreamerServer
        end

As one can see, the ``DataLogQueue`` maintains a thread safe queue for the producers to
push to and the ``DataLogger`` to pack (pop until empty) from. But it also stores the last
(pushed) sample for consumers to use e.g., the ``StateEst`` which runs at a constant sample
rate i.e., to simplify the signal processing. Note, other tasks than producers may populate
the queue e.g., estimated states which are used by the ``StateCtrl`` or for offline
tuning of control laws.

.. _data_logging:

Data Logging
=================
The ``DataLogger`` handles the data serialization of signals e.g., the IMU acceleration
which is continuously written to disk. It consists of a **HEADER** section - a json string,
describing the content of the second **DATA PACKAGES** section. See :numref:`data_log_protocol`
for an illustration of the data-log protocol.

.. _data_log_protocol:
.. figure:: figures/data_log_protocol.svg
    :width: 100%

    The data logging protocol. The **SIGNAL ID** is an unique identifier for each signal/package
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

Wireless Streaming
===================
There exists support for wireless data streaming from the drone. The streaming is build
on top of the open-source message library `ZeroMQ <https://zeromq.org>`_. In
:numref:`wireless_streaming_seq_diagram` the communication sequence between the client
(e.g., Laptop) and the server (drone) is illustrated.

.. _wireless_streaming_seq_diagram:
.. mermaid::
    :caption: Communication sequence between the client and the server. Note, only a subset of all available requests are shown.

    sequenceDiagram
        Client->>Server: Request[method: Get_DataLogMetadata<0>, data:{}]
        Server-->>Client: Response[code: Ok<0>, data:<DataLogMetadata as JSON>]
        Client->>Server: Request[method: Set_SelectedDataLogSignals<4>, data:[0,1,2,3]]
        Server-->>Client: Response[code: Ok<0>, data:{}]
        Client->>Server: Request[method: Set_StartStream<2>, data:{}]
        Server-->>Client: Response[code: Ok<0>, data:{}]
        loop
            Server-->>Client: Pushing[Selected data log signal package]
        end
        Client->>Server: Request[method: Set_StopStream<3>, data:{}]
        Server-->>Client: Response[code: Ok, data:{}]

The requests/responses are sent on a socket using the ``REQUEST-REPLY`` pattern, while
the actual selected data log signal bytes are sent on separate socket using the ``PUSH-PULL``
pattern. This to improve performance.

The data sent back and forth via requests/responses are JSON, while the data log signal
package is packed as following::

    <ABS_TIMESTAMP><DATA_LOG_SIGNALS>

and the data log signals are packed as::

    <ID_0><BYTES_0>...<ID_N><BYTES_N>

Note, they are packed somewhat differently compared to the data log signals in
:ref:`data_logging`. This since we can't really unsure that relative timestamps will be
continuous between sent packages.
