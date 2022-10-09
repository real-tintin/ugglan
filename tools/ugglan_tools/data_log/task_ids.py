from enum import Enum


class AutoNumber(Enum):
    def __new__(cls):
        value = len(cls.__members__)  # note no + 1
        obj = object.__new__(cls)
        obj._value_ = value
        return obj


class TaskId(AutoNumber):
    AccMag = ()
    Gyro = ()
    Barometer = ()
    EscRead0 = ()
    EscRead1 = ()
    EscRead2 = ()
    EscRead3 = ()
    EscWrite0 = ()
    EscWrite1 = ()
    EscWrite2 = ()
    EscWrite3 = ()
    RcReceiver = ()
    StateEst = ()
    StateCtrl = ()
    DataLogger = ()
