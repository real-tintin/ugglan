from typing import Optional

import zmq

from ugglan_tools.streamer.msg import Request, Response

DEFAULT_ADDR_REQUEST = "tcp://raspberrypi:55555"
DEFAULT_ADDR_STREAM = "tcp://raspberrypi:55556"


class Client:

    def __init__(self,
                 addr_request: str = DEFAULT_ADDR_REQUEST,
                 addr_stream: str = DEFAULT_ADDR_STREAM):
        self._addr_request = addr_request
        self._addr_stream = addr_stream

    def __enter__(self) -> object:
        self.connect()

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()

    def connect(self):
        self._context = zmq.Context()

        self._socket_request = self._context.socket(zmq.REQ)
        self._socket_request.connect(addr=self._addr_request)

        self._socket_stream = self._context.socket(zmq.PULL)
        self._socket_stream.connect(addr=self._addr_stream)

    def send_on_request(self, req: Request) -> Response:
        self._socket_request.send(req.to_packed_bytes())

        packed_bytes = self._socket_request.recv()
        res = Response(packed_bytes=packed_bytes)

        return res

    def recv_on_stream(self) -> Optional[bytes]:
        return self._socket_stream.recv()

    def disconnect(self):
        self._socket_request.close()
        self._socket_stream.close()

        self._context.term()
