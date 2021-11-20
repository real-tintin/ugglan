import ctypes
import os
import time
from threading import Thread
from time import perf_counter
from typing import Callable


class ThreadedTask:
    def __init__(self, cb: Callable, exec_period_s: float):
        self._cb = cb
        self._exec_period_s = exec_period_s

        self._thread = Thread(target=self._exec_task)
        self._run = False

        if self._is_windows():
            self._set_windows_high_accuracy_timer()

    def launch(self):
        self._run = True
        self._thread.start()

    def teardown(self):
        self._run = False
        self._thread.join()

    def _exec_task(self):
        while self._run:
            start_t_s = perf_counter()
            self._cb()
            exec_t_s = perf_counter() - start_t_s

            sleep_t_s = max(0, self._exec_period_s - exec_t_s)
            time.sleep(sleep_t_s)

    @staticmethod
    def _set_windows_high_accuracy_timer():
        """
        See https://stackoverflow.com/questions/11657734/sleep-for-exact-time-in-python/11658115.
        """
        winmm = ctypes.WinDLL('winmm')
        winmm.timeBeginPeriod(1)

    @staticmethod
    def _is_windows():
        return os.name == 'nt'
