import base64
import gzip
import json
import logging

from abc import abstractclassmethod, ABC
from enum import IntEnum
from typing import Union, List, Dict, Optional

logger = logging.getLogger(__name__)


class Method(IntEnum):
    Get_DataLogMetadata = 0
    Get_SelectedDataLogSignals = 1

    Set_StartStream = 2
    Set_StopStream = 3
    Set_SelectedDataLogSignals = 4


class Code(IntEnum):
    Ok = 0
    Error = 1


Data = Union[List, Dict]


class Base(ABC):

    def __init__(self,
                 metadata: Optional[IntEnum] = None,
                 data: Optional[Data] = None,
                 packed_bytes: Optional[bytes] = None):
        self._metadata = metadata
        self._data = data if data is not None else {}
        self._valid = True if metadata is not None else False

        if packed_bytes is not None:
            self.from_packed_bytes(packed_bytes)

    def __str__(self) -> str:
        return self._as_dict().__str__()

    def data(self) -> Data:
        return self._data

    def valid(self) -> bool:
        return self._valid

    def to_packed_bytes(self) -> Optional[bytes]:
        if self._valid:
            unpacked_as_dict = self._as_dict()
            unpacked_as_str = json.dumps(unpacked_as_dict)
            compressed_as_bytes = gzip.compress(unpacked_as_str.encode())
            encoded_as_bytes = base64.encodebytes(compressed_as_bytes)

            return encoded_as_bytes
        else:
            logger.error("Message is invalid, can't be packed")
            return None

    def from_packed_bytes(self, packed_bytes: bytes):
        try:
            decoded = base64.decodebytes(packed_bytes)
            decompressed = gzip.decompress(decoded)
            unpacked = json.loads(decompressed)

            self._metadata = unpacked[self._get_metadata_key()]
            self._data = unpacked[self._get_data_key()]
            self._valid = True
        except:
            logger.error("Unable to unpack_bytes.py bytes")
            self._valid = False

    def _as_dict(self) -> Dict:
        return {
            self._get_metadata_key(): self._metadata,
            self._get_data_key(): self._data
        }

    @staticmethod
    @abstractclassmethod
    def _get_metadata_key() -> str:
        pass

    @staticmethod
    def _get_data_key() -> str:
        return "data"


class Request(Base):

    def __init__(self,
                 method: Optional[Method] = None,
                 data: Optional[Data] = None,
                 packed_bytes: Optional[bytes] = None):
        super().__init__(metadata=method, data=data, packed_bytes=packed_bytes)

    def method(self) -> Method:
        return Method(self._metadata)

    @staticmethod
    def _get_metadata_key() -> str:
        return "method"


class Response(Base):

    def __init__(self,
                 code: Optional[Code] = None,
                 data: Optional[Data] = None,
                 packed_bytes: Optional[bytes] = None):
        super().__init__(metadata=code, data=data, packed_bytes=packed_bytes)

    def code(self) -> Code:
        return Code(self._metadata)

    @staticmethod
    def _get_metadata_key() -> str:
        return "code"
