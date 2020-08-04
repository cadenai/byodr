import cachetools

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

    def __init__(self, time_window_ms=1000, max_age_ms=200, max_delay_ms=250):
        # TTL is in seconds.
        # There is no distinction in violation types for now.
        self._violation_cache = cachetools.TTLCache(maxsize=1, ttl=(time_window_ms / 1000.))
        self._violations_key = '__n_violations__'
        self._max_age_micro = max_age_ms * 1000.
        self._max_delay_micro = max_delay_ms * 1000.
        self._last_message_time = None
        self._last_protocol_time = None
        self._started = False

    def _get_violations(self):
        amount = self._violation_cache.get(self._violations_key)
        return 0 if amount is None else amount

    def _increment_violations(self):
        _key = self._violations_key
        amount = self._violation_cache.get(_key)
        amount = 0 if amount is None else amount
        self._violation_cache[_key] = amount + 1

    def is_started(self):
        return self._started

    def reset(self):
        self._violation_cache.clear()
        self._started = False

    def on_message(self, message_timestamp_micro):
        # This is our time in microseconds.
        local_time = timestamp()
        if self.is_started() and local_time - self._last_protocol_time > self._max_delay_micro:
            self._increment_violations()
        elif self.is_started() and message_timestamp_micro - self._last_message_time > self._max_age_micro:
            self._increment_violations()
        self._last_message_time = message_timestamp_micro
        self._last_protocol_time = local_time
        self._started = True
        return self._get_violations()

    def check(self):
        if self.is_started() and timestamp() - self._last_protocol_time > self._max_delay_micro:
            self._increment_violations()
        return self._get_violations()
