from enum import IntEnum
from typing import Optional

from ugglan_tools.streamer.msg import Base, Data


class Status(IntEnum):
    Received = 0
    Lost = 1


class Mail(Base):

    def __init__(self,
                 status: Optional[Status] = None,
                 data: Optional[Data] = None):
        super().__init__(metadata=status, data=data)

    def status(self) -> Status:
        return Status(self._metadata)

    @staticmethod
    def _get_metadata_key() -> str:
        return "status"


def test_to_packed_bytes_and_back():
    exp_status = Status.Received
    exp_data = {"data": "my love..."}

    exp_mail = Mail(status=exp_status, data=exp_data)
    act_mail = Mail()

    packed_bytes = exp_mail.to_packed_bytes()
    act_mail.from_packed_bytes(packed_bytes)

    assert exp_status == act_mail.status()
    assert exp_data == act_mail.data()
