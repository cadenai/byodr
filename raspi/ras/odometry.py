import argparse
import logging
import multiprocessing
import os
import signal
import time
from configparser import ConfigParser as SafeConfigParser

from gpiozero import DigitalInputDevice

from byodr.utils import timestamp, Application
from byodr.utils.ipc import JSONPublisher
from byodr.utils.option import parse_option

logger = logging.getLogger(__name__)
log_format = '%(levelname)s: %(filename)s %(funcName)s %(message)s'

signal.signal(signal.SIGINT, lambda sig, frame: _interrupt())
signal.signal(signal.SIGTERM, lambda sig, frame: _interrupt())

quit_event = multiprocessing.Event()


def _interrupt():
    logger.info("Received interrupt, quitting.")
    quit_event.set()


class HallRps(object):
    def __init__(self, pin=16, moment=0.05, debug=False):
        self._moment = moment
        self._debug = debug
        self._rps = 0
        self._detect_time = 0
        self._detections = 0
        self._sensor = DigitalInputDevice(pin=pin, pull_up=True)
        self._sensor.when_activated = self._detect

    def tick(self):
        # Drop to zero when stopped.
        if timestamp() - self._detect_time > 1e5:
            self._rps = (1. - self._moment) * self._rps
            self._rps = self._rps if self._rps > 1e-4 else 0

    def rps(self):
        return self._rps

    def detections(self):
        return self._detections

    def _detect(self):
        h_val = 1e6 / (timestamp() - self._detect_time)
        self._rps = self._moment * h_val + (1. - self._moment) * self._rps
        self._detect_time = timestamp()
        if self._debug:
            self._detections += 1


class HallApplication(Application):
    def __init__(self, event, hz=50, **kwargs):
        super(HallApplication, self).__init__(run_hz=hz, quit_event=event)
        self._cm_per_revolution = parse_option('odometer.distance.cm_per_revolution', float, 5.0, **kwargs)
        self._debug = parse_option('odometer.debug', int, 0, **kwargs) == 1
        self._alpha = parse_option('odometer.moment.alpha', float, 0.10, **kwargs)
        self._hall = HallRps(moment=self._alpha, debug=self._debug)
        self.publisher = JSONPublisher(url='tcp://0.0.0.0:5560', topic='ras/sensor/odometer')

    def setup(self):
        logger.info("Created hall with hz={} cm/rev={:2.2f} alpha={:2.2f} and debug={}.".format(
            self.get_hz(), self._cm_per_revolution, self._alpha, self._debug
        ))

    def step(self):
        _velocity = self._hall.rps() * self._cm_per_revolution * 1e-2  # Convert to meters per second.
        self.publisher.publish(data=dict(velocity=_velocity))
        self._hall.tick()
        if self._debug:
            logger.info("{:2.2f} n={}".format(self._hall.rps(), self._hall.detections()))


def main():
    parser = argparse.ArgumentParser(description='Steering and throttle driver.')
    parser.add_argument('--config', type=str, default='/config/driver.ini', help='Configuration file.')
    args = parser.parse_args()
    config_file = args.config

    _run_hall = os.path.exists(config_file) and os.path.isfile(config_file)
    _config = {}
    if _run_hall:
        parser = SafeConfigParser()
        parser.read(config_file)
        _config = dict(parser.items('driver'))
        _type = parse_option('drive.type', str, **_config)
        _run_hall = _type == 'gpio_with_hall'
        if _run_hall:
            _config = dict(parser.items('odometer'))

    if _run_hall:
        _clock_hz = parse_option('clock.hz', float, 100.0, **_config)
        application = HallApplication(event=quit_event, hz=_clock_hz, **_config)
        application.run()
    else:
        while not quit_event.is_set():
            time.sleep(1)


if __name__ == "__main__":
    logging.basicConfig(format=log_format)
    logging.getLogger().setLevel(logging.INFO)
    main()
