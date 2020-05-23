import time

import numpy as np


def timestamp(value=None):
    """
    Timestamp as integer to retain precision e.g. when serializing to string.
    """
    ts = time.time() if value is None else value
    return int(ts * 1e6)


def entropy(x, eps=1e-20):
    return abs(-np.sum(x * np.log(np.clip(x, eps, 1.))))
