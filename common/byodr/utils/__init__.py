import collections
import logging
import multiprocessing
import signal
import time
import traceback
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
    def internal_quit(self, restarting=False):
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

    def quit(self, restarting=False):
        with self._lock:
            self.internal_quit(restarting)

    def join(self):
        self.quit()

    def restart(self, **kwargs):
        _reconfigured = self.is_reconfigured(**kwargs)
        if _reconfigured:
            if self._num_starts > 0:
                self.quit(restarting=True)
            self.start(**kwargs)
        return _reconfigured


class Application(object):
    def __init__(self, run_hz=10, quit_event=None):
        self.logger = logging.getLogger(__name__)
        self._hz = run_hz
        self._sleep = .100
        self.set_hz(run_hz)
        if quit_event is None:
            self.quit_event = multiprocessing.Event()
            signal.signal(signal.SIGINT, lambda sig, frame: self._interrupt())
            signal.signal(signal.SIGTERM, lambda sig, frame: self._interrupt())
        else:
            self.quit_event = quit_event
        # Recent window to calculate the actual processing frequency.
        self._rt_queue = collections.deque(maxlen=50)

    def _interrupt(self):
        self.logger.info("Received interrupt, quitting.")
        self.quit()

    @staticmethod
    def _latest_or_none(receiver, patience):
        candidate = receiver()
        _time = 0 if candidate is None else candidate.get('time')
        _on_time = (timestamp() - _time) < patience
        return candidate if _on_time else None

    def get_hz(self):
        return self._hz

    def get_actual_hz(self):
        return (1. / np.mean(self._rt_queue)) if self._rt_queue else 0

    def set_hz(self, hz):
        self._hz = hz
        self._sleep = 1. / hz

    def active(self):
        return not self.quit_event.is_set()

    def quit(self):
        self.quit_event.set()

    def setup(self):
        pass

    def step(self):
        pass

    def finish(self):
        pass

    def run(self):
        try:
            self.setup()
            while self.active():
                _start = time.time()
                self.step()
                _duration = (time.time() - _start)
                self._rt_queue.append(_duration)
                time.sleep(max(0., self._sleep - _duration))
        except Exception as e:
            self.logger.error("{}".format(traceback.format_exc(e)))
            self.quit()
        except KeyboardInterrupt:
            self.quit()
        finally:
            self.finish()
