import base64
import gzip
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Union, List, Tuple

import numpy as np

from ugglan_tools.data_log import unpack_bytes
from ugglan_tools.data_log.unpack_bytes import PackedBytes, TypeName

S_IN_MS = 0.001


@dataclass
class Signals:
    pass


@dataclass
class Group:
    pass


@dataclass
class Signal:
    val: Union[List[int], List[float]]
    t_s: List[float]


def read(path: Path,
         resample_to_fixed_rate_s=False) -> Signals:
    with open(path, mode='rb') as f:
        header_bytes = f.readline()
        packages = PackedBytes(buf=f.read())

    header = _unpack_header(header_bytes)
    data, last_timestamp_s = _unpack_packages(header, packages)

    if resample_to_fixed_rate_s:
        _resample_to_fixed_rate(data, resample_to_fixed_rate_s, last_timestamp_s)

    return data


def _unpack_header(h_bytes: bytes) -> Dict:
    h_decoded = base64.decodebytes(h_bytes)
    h_uncmp = gzip.decompress(h_decoded)
    h_dict = json.loads(h_uncmp)

    return h_dict


def _unpack_packages(header: Dict, packages: PackedBytes) -> Tuple[Signals, float]:
    """
    Assumes little endian, Raspberry Pi (ARM) and Intel.
    """
    data = Signals()
    abs_timestamp_s = 0

    while packages.offset < len(packages.buf):
        signal_id = unpack_bytes.signal_id(packages)

        signal_name = header['signals'][str(signal_id)]['name']
        type_id = header['signals'][str(signal_id)]['type']
        type_name = header['types'][str(type_id)]
        group_id = header['signals'][str(signal_id)]['group']
        group_name = header['groups'][str(group_id)]

        value = unpack_bytes.signal_value(packed=packages, type_name=TypeName(type_name))
        rel_timestamp_ms = unpack_bytes.rel_timestamp_ms(packed=packages)
        abs_timestamp_s += rel_timestamp_ms * S_IN_MS

        if not hasattr(data, group_name):
            group = Group()
            setattr(data, group_name, group)
        else:
            group = getattr(data, group_name)

        if not hasattr(group, signal_name):
            signal = Signal([], [])
            setattr(group, signal_name, signal)
        else:
            signal = getattr(group, signal_name)

        signal.val.append(value)
        signal.t_s.append(abs_timestamp_s)

    return data, abs_timestamp_s


def _resample_to_fixed_rate(data, sample_rate_s, last_timestamp_s):
    t_s = np.arange(0, last_timestamp_s, sample_rate_s)

    for group in data.__dict__:
        for signal in getattr(data, group).__dict__:
            s = getattr(getattr(data, group), signal)
            s.val = list(np.interp(t_s, s.t_s, s.val))
            s.t_s = list(t_s)
