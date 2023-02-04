import struct
import time

from ugglan_tools.streamer.client import Client
from ugglan_tools.streamer.msg import Request, Method


def main():
    with Client() as client:
        req = Request(method=Method.Get_DataLogMetadata)
        print(f"Sending request: {req}")
        res = client.send_on_request(req)
        print(f"Received response: {res}")

        req = Request(method=Method.Get_SelectedDataLogSignals)
        print(f"Sending request: {req}")
        res = client.send_on_request(req)
        print(f"Received response: {res}")

        req = Request(method=Method.Set_SelectedDataLogSignals, data=[0, 1, 2, 3])
        print(f"Sending request: {req}")
        res = client.send_on_request(req)
        print(f"Received response: {res}")

        req = Request(method=Method.Get_SelectedDataLogSignals)
        print(f"Sending request: {req}")
        res = client.send_on_request(req)
        print(f"Received response: {res}")

        req = Request(method=Method.Set_StartStream)
        print(f"Sending request: {req}")
        res = client.send_on_request(req)
        print(f"Received response: {res}")

        n_calls = 1000
        n_bytes = 0
        start = time.time()

        for _ in range(n_calls):
            packed_bytes = client.recv_on_stream()

            offset = 0
            abs_timestamp_ms = struct.unpack_from('<I', packed_bytes, offset)[0]
            offset += 4
            signal_id = struct.unpack_from('<H', packed_bytes, offset)[0]
            offset += 2
            imu_acceleration_x = struct.unpack_from('<d', packed_bytes, offset)[0]

            print(f"{abs_timestamp_ms} [ms]: {signal_id}: {imu_acceleration_x} [m/s]")

            n_bytes += len(packed_bytes)

        dt = time.time() - start
        print(f"Took: {dt} s")
        print(f"Client received {n_bytes} bytes, at {n_bytes / dt} bytes/s")

        req = Request(method=Method.Set_StopStream)
        print(f"Sending request: {req}")
        res = client.send_on_request(req)
        print(f"Received response: {res}")


if __name__ == "__main__":
    main()
