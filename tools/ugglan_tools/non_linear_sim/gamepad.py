from inputs import InputEvent, UnpluggedError, WIN, devices

from ugglan_tools.non_linear_sim.pilot_ctrl import RefInput

INIT_REF_INPUT = RefInput(f_z=0.0, roll=0.0, pitch=0.0, yaw_rate=0.0)

SCALE_AXIS = 2 ** 15 - 1


class Gamepad:
    """
    Makes use of the inputs lib (https://inputs.readthedocs.io/).

    Note, as the inputs function get_gamepad is blocking, the core
    inputs features are used and 're-packed' to make it non-blocking.

    Note, tested on Windows using an Xbox 360 controller.
    """

    def __init__(self, ref_scale: RefInput):
        self._ref_scale = ref_scale
        self._ref_input = INIT_REF_INPUT

        try:
            self._gamepad = devices.gamepads[0]
        except IndexError:
            raise UnpluggedError("No gamepad found.")

    def _get_events_non_blocking(self):
        """
        Logic copied from GamePad.__iter__ (ugly I know). This to
        achieve non-blocking.
        """
        if WIN:
            self._gamepad._GamePad__check_state()

        events = []
        event = self._gamepad._do_iter()

        while event is not None:
            events.append(*event)
            event = self._gamepad._do_iter()

        return events

    def update(self):
        for event in self._get_events_non_blocking():
            if event.code == 'ABS_X':
                self._ref_input.yaw_rate = self._ref_scale.yaw_rate * self._scale_axis_event(event)

            if event.code == 'ABS_Y':
                self._ref_input.f_z = -self._ref_scale.f_z * 0.5 * (self._scale_axis_event(event) + 1)

            if event.code == 'ABS_RX':
                self._ref_input.roll = self._ref_scale.roll * self._scale_axis_event(event)

            if event.code == 'ABS_RY':
                self._ref_input.pitch = -self._ref_scale.pitch * self._scale_axis_event(event)

    def get_ref_input(self) -> RefInput:
        return self._ref_input

    @staticmethod
    def _scale_axis_event(event: InputEvent) -> float:
        return float(event.state) / SCALE_AXIS
