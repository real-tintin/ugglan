from collections import deque
from copy import deepcopy
from dataclasses import dataclass, is_dataclass
from operator import attrgetter
from typing import Callable


class RollingBuffer:
    def __init__(self, member_cb_map: dict, n_samples: int):
        """
        Stores data in a rolling buffer (of size n_samples)
        given the member and data callback map.

        Note, the data can be an int, float or a nested dataclass.
        """
        self._n_samples = n_samples
        self._buf_cbs = []

        for member, cb in member_cb_map.items():
            data_type = deepcopy(cb())
            setattr(self, member, self._create_nested_deque(parent=data_type, cb=cb))

    def update(self):
        for buf_cb in self._buf_cbs:
            buf_cb()

    def _create_nested_deque(self, parent: dataclass, cb: Callable, ancestors_without_me: [str] = None) -> dataclass:
        if is_dataclass(parent):
            for child_name in parent.__annotations__:
                child = getattr(parent, child_name)

                if ancestors_without_me is None:
                    ancestors_with_me = [child_name]
                else:
                    ancestors_with_me = deepcopy(ancestors_without_me)
                    ancestors_with_me.append(child_name)

                setattr(parent, child_name, self._create_nested_deque(child, cb, ancestors_with_me))

        elif isinstance(parent, float) or isinstance(parent, int):
            buf = deque(maxlen=self._n_samples)

            if ancestors_without_me is None:
                self._buf_cbs.append(lambda: buf.append(cb()))
            else:
                self._buf_cbs.append(lambda: buf.append(attrgetter('.'.join(ancestors_without_me))(cb())))

            return buf

        else:
            raise ValueError

        return parent
