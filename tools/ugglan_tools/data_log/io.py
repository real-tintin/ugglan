import base64
import gzip
import json
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Union, List, Tuple

import numpy as np

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
        packages_bytes = f.read()

    header = _unpack_header(header_bytes)
    data, last_timestamp_s = _unpack_packages(header, packages_bytes)

    if resample_to_fixed_rate_s:
        _resample_to_fixed_rate(data, resample_to_fixed_rate_s, last_timestamp_s)

    return data


def _unpack_header(h_bytes: bytes) -> Dict:
    h_decoded = base64.decodebytes(h_bytes)
    h_uncmp = gzip.decompress(h_decoded)
    h_dict = json.loads(h_uncmp)

    return h_dict


def _unpack_packages(header: Dict, packages: bytes) -> Tuple[Signals, float]:
    """
    Assumes little endian, Raspberry Pi (ARM) and Intel.
    """
    data = Signals()
    abs_timestamp_s = 0
    offset = 0

    while offset < len(packages):
        offset, signal_id = _get_signal_id(packages, offset)

        signal_name = header['signals'][str(signal_id)]['name']
        type_id = header['signals'][str(signal_id)]['type']
        type_name = header['types'][str(type_id)]
        group_id = header['signals'][str(signal_id)]['group']
        group_name = header['groups'][str(group_id)]

        offset, value = _get_signal_value(packages, offset, type_name)
        offset, rel_timestamp_ms = _get_rel_timestamp_ms(packages, offset)
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


def _get_signal_id(packages, offset):
    signal_id = struct.unpack_from('<H', packages, offset)[0]
    offset += 2  # uint16

    return offset, signal_id


def _get_signal_value(packages, offset, type_name):
    if type_name == 'BOOL':
        value = struct.unpack_from('<B', packages, offset)[0]
        offset += 1
    elif type_name == 'UINT8':
        value = struct.unpack_from('<B', packages, offset)[0]
        offset += 1
    elif type_name == 'UINT16':
        value = struct.unpack_from('<H', packages, offset)[0]
        offset += 2
    elif type_name == 'UINT32':
        value = struct.unpack_from('<I', packages, offset)[0]
        offset += 4
    elif type_name == 'SINT8':
        value = struct.unpack_from('<b', packages, offset)[0]
        offset += 1
    elif type_name == 'SINT16':
        value = struct.unpack_from('<h', packages, offset)[0]
        offset += 2
    elif type_name == 'SINT32':
        value = struct.unpack_from('<i', packages, offset)[0]
        offset += 4
    elif type_name == 'FLOAT':
        value = struct.unpack_from('<f', packages, offset)[0]
        offset += 4
    elif type_name == 'DOUBLE':
        value = struct.unpack_from('<d', packages, offset)[0]
        offset += 8
    else:
        raise ValueError

    return offset, value


def _get_rel_timestamp_ms(packages, offset):
    rel_timestamp_ms = struct.unpack_from('<B', packages, offset)[0]
    offset += 1  # uint8

    return offset, rel_timestamp_ms


def _resample_to_fixed_rate(data, sample_rate_s, last_timestamp_s):
    t_s = np.arange(0, last_timestamp_s, sample_rate_s)

    for group in data.__dict__:
        for signal in getattr(data, group).__dict__:
            s = getattr(getattr(data, group), signal)
            s.val = list(np.interp(t_s, s.t_s, s.val))
            s.t_s = list(t_s)
