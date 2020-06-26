import multiprocessing
import time
from abc import ABCMeta, abstractmethod

import numpy as np

from byodr.utils.option import hash_dict


def timestamp(value=None):
    """
    Timestamp as integer to retain precision e.g. when serializing to string.
    """
    ts = time.time() if value is None else value
    return int(ts * 1e6)


def entropy(x, eps=1e-20):
    return abs(-np.sum(x * np.log(np.clip(x, eps, 1.))))


class Configurable(object):
    __metaclass__ = ABCMeta

    def __init__(self):
        self._lock = multiprocessing.Lock()
        self._errors = []
        self._hash = -1
        self._num_starts = 0

    # noinspection PyUnusedLocal
    @abstractmethod
    def internal_start(self, **kwargs):
        return []

    @abstractmethod
    def internal_quit(self):
        pass

    def get_errors(self):
        return self._errors

    def get_num_starts(self):
        return self._num_starts

    def is_reconfigured(self, **kwargs):
        return self._hash != hash_dict(**kwargs)

    def start(self, **kwargs):
        with self._lock:
            self._errors = self.internal_start(**kwargs)
            self._hash = hash_dict(**kwargs)
            self._num_starts += 1

    def quit(self):
        with self._lock:
            self.internal_quit()

    def join(self):
        self.quit()

    def restart(self, force=False, **kwargs):
        if force or self.is_reconfigured(**kwargs):
            self.quit()
            self.start(**kwargs)
