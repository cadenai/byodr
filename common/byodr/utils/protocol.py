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
        There is a warmup period with invalidated protocol, after system reboot.
        -
        The incoming stream needs to be continuous (or uninterrupted) and recent (timely).
        Continuity violation
        Age violation
        Violations are counted during a time window using a ttl cache.
        A threshold invalidates the protocol.
    """

    def __init__(self, max_age_ms=200, max_delay_ms=250):
        # There is no distinction in violation types for now.
        self._n_violations = 0L
        self._max_age_micro = max_age_ms * 1000.
        self._max_delay_micro = max_delay_ms * 1000.
        self._last_message_time = None
        self._last_protocol_time = None
        self._started = False

    def _violation(self):
        self._n_violations = 1 if self._n_violations <= 0 else self._n_violations + 1

    def _success(self):
        self._n_violations -= 1

    def is_started(self):
        return self._started

    def reset(self):
        self._n_violations = 0L
        self._started = False

    def on_message(self, message_timestamp_micro):
        # This is our time in microseconds.
        local_time = timestamp()
        if self.is_started() and local_time - self._last_protocol_time > self._max_delay_micro:
            self._violation()
        elif self.is_started() and message_timestamp_micro - self._last_message_time > self._max_age_micro:
            self._violation()
        else:
            self._success()
        self._last_message_time = message_timestamp_micro
        self._last_protocol_time = local_time
        self._started = True

    def check(self):
        if self.is_started() and timestamp() - self._last_protocol_time > self._max_delay_micro:
            self._violation()
        return self._n_violations
