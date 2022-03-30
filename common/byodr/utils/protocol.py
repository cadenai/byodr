from __future__ import absolute_import

from byodr.utils import timestamp


class MessageStreamProtocol(object):
    """
        Safety:
        Protocol uses 2 timestamps, remote and local, and does not require the clocks to be synced.
        Local means receiver side so incoming messages.
        Because the clocks are not synced remote and local timestamps are not directly comparable.
        Timestamps:
        1. remote as reported by the sender
        2. local as recorded by the receiver
        -
        The protocol can be validated or invalidated.
        There is a warm-up period with invalidated protocol, after system reboot.
        -
        The incoming stream needs to be continuous (or uninterrupted) and recent (timely).
        Continuity violation
        Age violation
    """

    def __init__(self, max_age_ms=200, max_delay_ms=250):
        self._max_age_micro = max_age_ms * 1000.
        self._max_delay_micro = max_delay_ms * 1000.
        # There is currently no distinction in violation types.
        self._n_violations = 0
        self._last_message_time = 0
        self._last_protocol_time = 0

    def _violation(self):
        self._n_violations = 1 if self._n_violations < 1 else min(1e4, self._n_violations + 1)

    def _success(self):
        self._n_violations = 0 if self._n_violations > 0 else max(-1e4, self._n_violations - 1)

    def reset(self):
        self._n_violations = 0
        self._last_message_time = 0
        self._last_protocol_time = 0

    def on_message(self, message_timestamp_micro):
        # This is our time in microseconds.
        local_time = timestamp()
        if local_time - self._last_protocol_time > self._max_delay_micro:
            self._violation()
        elif message_timestamp_micro - self._last_message_time > self._max_age_micro:
            self._violation()
        else:
            self._success()
        self._last_message_time = message_timestamp_micro
        self._last_protocol_time = local_time

    def check(self):
        if timestamp() - self._last_protocol_time > self._max_delay_micro:
            self._violation()
        return self._n_violations
