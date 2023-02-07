import time

from ugglan_tools.streamer.client import Client, Package

SELECTED_SIGNAL_IDS = [0, 1, 2]


def recv_on_stream_cb(package: Package) -> None:
    acc_x = package.signal_id_value_map[0]
    acc_y = package.signal_id_value_map[1]
    acc_z = package.signal_id_value_map[2]

    print(f"{package.abs_timestamp_ms} [ms]: IMU acceleration: ({acc_x}, {acc_y}, {acc_z}) m/s")


def main():
    with Client(recv_on_stream_cb=recv_on_stream_cb) as client:
        available_signals = client.get_available_signals()
        print(f"Available signals: {available_signals}")

        selected_signals = [signal for signal in available_signals if signal.id in SELECTED_SIGNAL_IDS]
        print(f"Selected signals: {SELECTED_SIGNAL_IDS}")
        client.set_selected_signals(selected_signals)

        client.start_recv_on_stream()
        print(f"Started recv on stream")

        time.sleep(10)

        client.stop_recv_on_stream()
        print(f"Stopped recv on stream")


if __name__ == "__main__":
    main()
