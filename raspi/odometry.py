import logging
import multiprocessing
import signal
import time

from gpiozero import DigitalInputDevice

from byodr.utils import timestamp
from byodr.utils.ipc import JSONPublisher

logger = logging.getLogger(__name__)
log_format = '%(levelname)s: %(filename)s %(funcName)s %(message)s'

quit_event = multiprocessing.Event()

signal.signal(signal.SIGINT, lambda sig, frame: _interrupt())
signal.signal(signal.SIGTERM, lambda sig, frame: _interrupt())


def _interrupt():
    logger.info("Received interrupt, quitting.")
    quit_event.set()


class HallRps(object):
    def __init__(self, pin=22, moment=0.10):
        self._moment = moment
        self._detect_time, self._up, self._rps = 0, 0, 0
        self._sensor = DigitalInputDevice(pin=pin, pull_up=True)
        self._sensor.when_activated = self._detect

    def tick(self):
        # Drop to zero when stopped.
        if timestamp() - self._detect_time > 1e5:
            self._rps = (1. - self._moment) * self._rps
            self._rps = self._rps if self._rps > 1e-4 else 0

    def rps(self):
        return self._rps

    def debug(self):
        return '{} {} {}'.format(self._sensor.value, self._sensor.is_active, self._rps)

    def _detect(self):
        # self._up += 1
        # if self._up >= 2:
        h_val = 1e6 / (timestamp() - self._detect_time)
        self._rps = self._moment * h_val + (1. - self._moment) * self._rps
        self._detect_time = timestamp()
        self._up = 0


def main():
    p_odo = JSONPublisher(url='tcp://0.0.0.0:5556', topic='ras/sensor/odometer')
    s_hall = HallRps()

    rate = 0.02  # 50 Hz.
    while not quit_event.is_set():
        p_odo.publish(data=dict(rps=s_hall.rps()))
        s_hall.tick()
        time.sleep(rate)


if __name__ == "__main__":
    logging.basicConfig(format=log_format)
    logging.getLogger().setLevel(logging.INFO)
    main()
