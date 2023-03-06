import argparse
from dataclasses import dataclass, field
from typing import List

import plotly.graph_objects as go
from dash import Dash
from dash import dcc, html
from dash.dependencies import Input, Output

from ugglan_tools.streamer.client import Client as StreamerClient
from ugglan_tools.streamer.client import Signal
from ugglan_tools.streamer.rolling_buffer import RollingBuffer

GUI_REFRESH_RATE_MS = int(1000 / 30)  # 30 Hz
SERVER_STREAM_RATE_MS = 10  # 100 Hz

TIME_WINDOW_SIZE_S = 5
TIME_WINDOW_SIZE_SAMPLES = int(TIME_WINDOW_SIZE_S / (SERVER_STREAM_RATE_MS / 1000))


@dataclass
class Storage:
    available_signals: List[Signal] = field(default_factory=list)
    selected_signals: List[Signal] = field(default_factory=list)


gui = Dash(__name__)
rolling_buffer = RollingBuffer(size=TIME_WINDOW_SIZE_SAMPLES)
streamer_client = StreamerClient(recv_on_stream_cb=rolling_buffer.push_package)
storage = Storage()

gui.layout = html.Div([
    html.H4(),
    html.Div([
        html.Div(id='container-button'),
        html.Button('Connect', id='button-connect', n_clicks=0),
        html.Button('Start stream', id='button-stream', n_clicks=0, disabled=True)
    ]),
    dcc.Dropdown(
        id="dropdown",
        options=[],
        value=[],
        placeholder="Select signals...",
        searchable=True,
        multi=True,
    ),
    dcc.Graph(
        id="graph",
        style={'height': '90vh'}
    ),
    dcc.Interval(
        id='interval-component',
        interval=GUI_REFRESH_RATE_MS,
        n_intervals=0
    ),
    dcc.ConfirmDialog(
        id='confirm-dialog-connect',
        message='',
    ),
    dcc.ConfirmDialog(
        id='confirm-dialog-stream',
        message='',
    ),
])


def _get_error_str(e: Exception) -> str:
    return f"{e.__class__.__module__}.{e.__class__.__name__}: {e.__str__()}"


def _get_available_signal_from_group_signal_name(name: str) -> Signal:
    group_name, signal_name = name.split("_")

    for signal in storage.available_signals:
        if signal.group.name == group_name and signal.name == signal_name:
            return signal

    raise ValueError("Signal name not found")


def _get_group_signal_names_from_signals(signals: List[Signal]) -> List[str]:
    return [f"{signal.group.name}_{signal.name}"
            for signal in signals]


@gui.callback(
    Output('button-connect', 'children'),
    Output('button-stream', 'disabled'),
    Output('confirm-dialog-connect', 'displayed'),
    Output('confirm-dialog-connect', 'message'),
    Output('dropdown', 'options'),
    Input('button-connect', 'n_clicks'),
)
def update_button_connect(n_clicks):
    try:
        if n_clicks == 0:
            return "Connect", True, False, "", []

        elif streamer_client.is_connected():
            streamer_client.disconnect()

            return "Connect", True, False, "", []

        else:
            streamer_client.connect()
            storage.available_signals = streamer_client.get_available_signals()
            available_group_signal_names = _get_group_signal_names_from_signals(storage.available_signals)

            return "Disconnect", False, False, "", available_group_signal_names

    except Exception as e:
        return "Connect", True, True, _get_error_str(e), []


@gui.callback(
    Output('button-stream', 'children'),
    Output('confirm-dialog-stream', 'displayed'),
    Output('confirm-dialog-stream', 'message'),
    Input('button-stream', 'n_clicks'),
)
def update_button_stream(n_clicks):
    try:
        if n_clicks == 0:
            return "Start stream", False, ""

        elif streamer_client.is_recv_on_stream():
            streamer_client.stop_recv_on_stream()

            return "Start stream", False, ""

        else:
            rolling_buffer.clear()

            streamer_client.set_selected_signals(storage.selected_signals)
            streamer_client.start_recv_on_stream()

            return "Stop stream", False, ""

    except Exception as e:
        return "Start stream", True, _get_error_str(e)


@gui.callback(
    Output("dropdown", "value"),
    Input("dropdown", "value")
)
def update_dropdown(selected_group_signal_names):
    storage.selected_signals = [_get_available_signal_from_group_signal_name(name)
                                for name in selected_group_signal_names]

    if streamer_client.is_recv_on_stream():
        rolling_buffer.clear()

        streamer_client.stop_recv_on_stream()
        streamer_client.set_selected_signals(storage.selected_signals)
        streamer_client.start_recv_on_stream()

    return selected_group_signal_names


@gui.callback(Output('graph', 'figure'),
              Input('interval-component', 'n_intervals'))
def update_graph_live(_):
    fig = go.Figure()

    fig.update_xaxes(title="time [s]", showgrid=True)
    fig.update_yaxes(showgrid=True)

    fig.update_layout(
        legend=dict(
            yanchor="top",
            y=1.0,
            xanchor="left",
            x=1.0
        ))

    abs_time_s = rolling_buffer.get_abs_time_s()

    for signal in storage.selected_signals:
        values = rolling_buffer.get_values(signal_id=signal.id)

        fig.add_trace(go.Scatter(
            x=abs_time_s,
            y=values,
            name=signal.name,
            mode='lines'))

    return fig


def main():
    parser = argparse.ArgumentParser(description='Launches gui for live streaming of data.')
    parser.parse_args()

    gui.run_server(debug=True)


if __name__ == "__main__":
    main()
