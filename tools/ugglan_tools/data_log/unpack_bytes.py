import struct
from dataclasses import dataclass
from enum import Enum
from typing import Union


class TypeName(Enum):
    BOOL = "BOOL"
    UINT8 = "UINT8"
    UINT16 = "UINT16"
    UINT32 = "UINT32"
    SINT8 = "SINT8"
    SINT16 = "SINT16"
    SINT32 = "SINT32"
    FLOAT = "FLOAT"
    DOUBLE = "DOUBLE"


Value = Union[bool, int, float]


@dataclass
class PackedBytes:
    buf: bytes
    offset: int = 0


def signal_id(packed: PackedBytes):
    signal_id = struct.unpack_from('<H', packed.buf, packed.offset)[0]
    packed.offset += 2  # uint16

    return signal_id


def signal_value(packed: PackedBytes, type_name: TypeName) -> Value:
    if type_name == TypeName.BOOL:
        value = struct.unpack_from('<B', packed.buf, packed.offset)[0]
        packed.offset += 1
    elif type_name == TypeName.UINT8:
        value = struct.unpack_from('<B', packed.buf, packed.offset)[0]
        packed.offset += 1
    elif type_name == TypeName.UINT16:
        value = struct.unpack_from('<H', packed.buf, packed.offset)[0]
        packed.offset += 2
    elif type_name == TypeName.UINT32:
        value = struct.unpack_from('<I', packed.buf, packed.offset)[0]
        packed.offset += 4
    elif type_name == TypeName.SINT8:
        value = struct.unpack_from('<b', packed.buf, packed.offset)[0]
        packed.offset += 1
    elif type_name == TypeName.SINT16:
        value = struct.unpack_from('<h', packed.buf, packed.offset)[0]
        packed.offset += 2
    elif type_name == TypeName.SINT32:
        value = struct.unpack_from('<i', packed.buf, packed.offset)[0]
        packed.offset += 4
    elif type_name == TypeName.FLOAT:
        value = struct.unpack_from('<f', packed.buf, packed.offset)[0]
        packed.offset += 4
    elif type_name == TypeName.DOUBLE:
        value = struct.unpack_from('<d', packed.buf, packed.offset)[0]
        packed.offset += 8
    else:
        raise ValueError

    return value


def abs_timestamp_ms(packed: PackedBytes) -> int:
    abs_timestamp_ms = struct.unpack_from('<I', packed.buf, packed.offset)[0]
    packed.offset += 4  # uint32

    return abs_timestamp_ms


def rel_timestamp_ms(packed: PackedBytes) -> int:
    rel_timestamp_ms = struct.unpack_from('<B', packed.buf, packed.offset)[0]
    packed.offset += 1  # uint8

    return rel_timestamp_ms
