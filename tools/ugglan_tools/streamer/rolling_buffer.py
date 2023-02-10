from collections import deque
from typing import List

from ugglan_tools.data_log.unpack_bytes import Value
from ugglan_tools.streamer.client import Package

MS_IN_S = 1000


class RollingBuffer:
    _ABS_TIME_KEY = "abs_time_s"

    def __init__(self, size: int):
        self._size = size

        self._buffers = {self._ABS_TIME_KEY: self._create_buffer()}
        self._abs_time_offset_s = None

    def push_package(self, package: Package):
        for signal_id, signal_value in package.signal_id_value_map.items():
            if signal_id not in self._buffers:
                self._buffers[signal_id] = self._create_buffer()

            self._buffers[signal_id].append(signal_value)

        abs_timestamp_s = package.abs_timestamp_ms / MS_IN_S

        if len(self._buffers[self._ABS_TIME_KEY]) == 0:
            self._abs_time_offset_s = abs_timestamp_s

        self._buffers[self._ABS_TIME_KEY].append(abs_timestamp_s - self._abs_time_offset_s)

    def clear(self):
        for buffer in self._buffers.values():
            buffer.clear()

    def get_values(self, signal_id: int) -> List[Value]:
        if signal_id in self._buffers:
            return list(self._buffers[signal_id])
        else:
            return []

    def get_abs_time_s(self) -> List[float]:
        return list(self._buffers[self._ABS_TIME_KEY])

    def _create_buffer(self) -> object:
        return deque(maxlen=self._size)
