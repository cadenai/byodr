import time


def timestamp(value=None):
    """
    Timestamp as integer to retain precision e.g. when serializing to string.
    """
    ts = time.time() if value is None else value
    return int(ts * 1e6)
