from dataclasses import dataclass
from threading import Thread
from typing import Optional, List, Callable, Dict, Any

import zmq

from ugglan_tools.data_log import unpack_bytes
from ugglan_tools.data_log.unpack_bytes import Value, TypeName, PackedBytes
from ugglan_tools.streamer.msg import Request, Response, Code, Method

DEFAULT_ADDR_REQUEST = "tcp://raspberrypi:55555"
DEFAULT_ADDR_STREAM = "tcp://raspberrypi:55556"

RECV_TIMEOUT_MS = 1000


class ResponseError(Exception):
    pass


@dataclass
class Group:
    name: str
    id: int


@dataclass
class Signal:
    name: str
    id: int
    type: Type
    group: Group


@dataclass
class Package:
    signal_id_value_map: Dict[int, Value]
    abs_timestamp_ms: int


class Client:

    def __init__(self,
                 addr_request: str = DEFAULT_ADDR_REQUEST,
                 addr_stream: str = DEFAULT_ADDR_STREAM,
                 recv_on_stream_cb: Optional[Callable[[Package], None]] = None):
        self._addr_request = addr_request
        self._addr_stream = addr_stream

        self._context = zmq.Context()

        self._socket_request = self._context.socket(zmq.REQ)
        self._socket_stream = self._context.socket(zmq.PULL)

        self._socket_request.setsockopt(zmq.RCVTIMEO, RECV_TIMEOUT_MS)
        self._socket_stream.setsockopt(zmq.RCVTIMEO, RECV_TIMEOUT_MS)

        self._is_recv_on_stream = False
        self._recv_on_stream_cb = recv_on_stream_cb
        self._recv_stream_thread = Thread(target=self._recv_on_stream)

        self._group_id_to_name_map = None
        self._type_id_to_name_map = None
        self._signal_id_to_info_map = None

        self._selected_signal_ids = []

    def __del__(self):
        self._context.term()

    def __enter__(self) -> object:
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()

    def connect(self):
        self._socket_request.connect(addr=self._addr_request)
        self._socket_stream.connect(addr=self._addr_stream)

        self._update_metadata_maps()

    def disconnect(self):
        self._socket_request.close()
        self._socket_stream.close()

    def get_available_signals(self) -> List[Signal]:
        signals = []

        for signal_id in self._signal_id_to_info_map.keys():
            signals.append(self._create_signal_from_id(signal_id))

        return signals

    def get_selected_signals(self) -> List[Signal]:
        signals = []

        for signal_id in self._selected_signal_ids:
            signals.append(self._create_signal_from_id(signal_id))

        return signals

    def set_selected_signals(self, signals: Optional[List[Signal]]):
        self._selected_signal_ids = [signal.id for signal in signals]

        self.send_on_request(req=Request(method=Method.Set_SelectedDataLogSignals,
                                         data=self._selected_signal_ids))

        res = self.send_on_request(req=Request(method=Method.Get_SelectedDataLogSignals))

        if self._selected_signal_ids != res.data():
            raise RuntimeError("Missmatch between set and get selected signals")

    def start_recv_on_stream(self):
        self.send_on_request(req=Request(method=Method.Set_StartStream))
        self._start_recv_stream_in_thread()

    def stop_recv_on_stream(self):
        self._stop_recv_stream_in_thread()
        self.send_on_request(req=Request(method=Method.Set_StopStream))
        self._flush_stream_socket_recv()

    def send_on_request(self, req: Request) -> Response:
        self._socket_request.send(req.to_packed_bytes())

        packed_bytes = self._socket_request.recv()
        res = Response(packed_bytes=packed_bytes)

        if res.code() == Code.Error:
            raise ResponseError("Is the request valid?")

        return res

    def _update_metadata_maps(self):
        req = self.send_on_request(req=Request(method=Method.Get_DataLogMetadata))
        metadata = req.data()

        self._group_id_to_name_map = self._dict_keys_str_to_int(metadata["groups"])
        self._type_id_to_name_map = self._dict_keys_str_to_int(metadata["types"])
        self._signal_id_to_info_map = self._dict_keys_str_to_int(metadata["signals"])

    def _start_recv_stream_in_thread(self):
        self._is_recv_on_stream = True
        self._recv_stream_thread.start()

    def _stop_recv_stream_in_thread(self):
        self._is_recv_on_stream = False
        self._recv_stream_thread.join()

    def _recv_on_stream(self):
        while self._is_recv_on_stream:
            signal_id_value_map = {}
            package = PackedBytes(buf=self._socket_stream.recv())
            abs_timestamp_ms = unpack_bytes.abs_timestamp_ms(packed=package)

            while package.offset < len(package.buf):
                signal_id = unpack_bytes.signal_id(packed=package)
                type_id = self._signal_id_to_info_map[signal_id]["type"]
                type_name = self._type_id_to_name_map['types'][type_id]

                value = unpack_bytes.signal_value(packed=package, type_name=TypeName(type_name))
                signal_id_value_map.update({signal_id: value})

            package = Package(signal_id_value_map=signal_id_value_map,
                              abs_timestamp_ms=abs_timestamp_ms)

            self._recv_on_stream_cb(package)

    def _flush_stream_socket_recv(self):
        try:
            self._socket_stream.recv()
        except zmq.error.Again:
            pass

    def _create_signal_from_id(self, signal_id: int) -> Signal:
        signal_info = self._signal_id_to_info_map[signal_id]

        signal_name = signal_info["name"]
        signal_type_id = signal_info["type"]

        group_id = signal_info["group"]
        group_name = self._group_id_to_name_map[group_id]

        return Signal(
            name=signal_name,
            id=signal_id,
            type=Type(signal_type_id),
            group=Group(name=group_name, id=group_id)
        )

    @staticmethod
    def _dict_keys_str_to_int(d: Dict[str, Any]) -> Dict[int, Any]:
        return {int(key): d[key] for key in d.keys()}
