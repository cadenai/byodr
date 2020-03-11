import argparse
import collections
import logging
import multiprocessing
import os
import signal
import threading
import time
import traceback
from ConfigParser import SafeConfigParser

import cv2
import numpy as np

from byodr.utils.ipc import ReceiverThread, CameraThread, JSONPublisher
from recorder import get_or_create_recorder
from store import Event

logger = logging.getLogger(__name__)
quit_event = multiprocessing.Event()

signal.signal(signal.SIGINT, lambda sig, frame: _interrupt())
signal.signal(signal.SIGTERM, lambda sig, frame: _interrupt())


def _interrupt():
    logger.info("Received interrupt, quitting.")
    quit_event.set()


def to_event(blob, vehicle, image):
    event = Event(
        timestamp=blob.get('time'),
        image=image,
        steer_src=blob.get('steering_driver'),
        speed_src=blob.get('speed_driver'),
        command_src=blob.get('steering_driver'),
        steering=blob.get('steering'),
        desired_speed=blob.get('desired_speed'),
        actual_speed=vehicle.get('velocity'),
        heading=vehicle.get('heading'),
        throttle=blob.get('throttle'),
        command=blob.get('instruction'),
        x_coordinate=vehicle.get('x_coordinate'),
        y_coordinate=vehicle.get('y_coordinate')
    )
    event.save_event = blob.get('save_event')
    return event


class EventHandler(threading.Thread):
    def __init__(self, directory, event, **kwargs):
        super(EventHandler, self).__init__()
        self._directory = directory
        self._quit_event = event
        self._vehicle = kwargs.get('constant.vehicle.type')
        self._config = kwargs.get('constant.vehicle.config')
        _im_height, _im_width = kwargs.get('image.persist.scale').split('x')
        self._im_height = int(_im_height)
        self._im_width = int(_im_width)
        self._active = False
        self._driver = None
        self._recorder = get_or_create_recorder(directory=directory, mode=None)
        self._recent = collections.deque(maxlen=100)
        self._buffer = collections.deque(maxlen=10000)

    def _instance(self, driver):
        mode = None
        if driver in ('driver_mode.teleop.cruise', 'driver_mode.automatic.backend'):
            mode = 'record.mode.driving'
        elif driver == 'driver_mode.inference.dnn':
            mode = 'record.mode.interventions'
        return get_or_create_recorder(mode=mode,
                                      directory=self._directory,
                                      vehicle_type=self._vehicle,
                                      vehicle_config=self._config)

    def state(self):
        return dict(active=self._active, mode=self._recorder.get_mode())

    def record(self, blob, vehicle, image):
        # Switch on automatically and then off only when the driver changes.
        _driver = blob.get('driver')
        _start_recorder_dnn = not self._active and _driver == 'driver_mode.inference.dnn'
        _start_recorder_cruise = not self._active and _driver == 'driver_mode.teleop.cruise' and blob.get('cruise_speed') > 1e-3
        if _start_recorder_dnn or _start_recorder_cruise:
            self._recent.clear()
            self._recorder.start()
            self._active = True
        if self._active and self._driver != _driver:
            self._driver = _driver
            self._recorder.stop()
            self._active = False
            self._recorder = self._instance(_driver)
            self._recent.clear()
            self._recorder.start()
        if self._active:
            # Do not process the same event more than once.
            key_time = blob.get('time')
            if key_time not in self._recent:
                self._recent.append(key_time)
                self._buffer.append(to_event(blob, vehicle, np.copy(image)))

    def run(self):
        while not self._quit_event.is_set():
            try:
                event = self._buffer.pop()
                if event.image.shape != (self._im_height, self._im_width, 3):
                    event.image = cv2.resize(event.image, (self._im_width, self._im_height))
                self._recorder.do_record(event)
                # Allow the main thread access to the cpu.
                time.sleep(2e-3)
            except IndexError:
                pass


def main():
    parser = argparse.ArgumentParser(description='Recorder.')
    parser.add_argument('--sessions', type=str, required=True, help='Sessions directory.')
    parser.add_argument('--config', type=str, required=True, help='Config file location.')
    args = parser.parse_args()

    sessions_dir = os.path.expanduser(args.sessions)
    assert os.path.exists(sessions_dir), "Cannot use sessions directory '{}'".format(sessions_dir)

    parser = SafeConfigParser()
    [parser.read(_f) for _f in args.config.split(',')]
    cfg = dict(parser.items('recorder'))
    for key in sorted(cfg):
        logger.info("{} = {}".format(key, cfg[key]))

    _process_frequency = int(cfg.get('clock.hz'))
    _publish_frequency = int(cfg.get('publish.hz'))
    logger.info("Processing at {} Hz publishing at {} Hz.".format(_process_frequency, _publish_frequency))
    max_process_duration = 1. / _process_frequency
    max_publish_duration = 1. / _publish_frequency

    state_publisher = JSONPublisher(url='ipc:///byodr/recorder.sock', topic='aav/recorder/state')

    threads = []
    camera = CameraThread(url='ipc:///byodr/camera.sock', topic=b'aav/camera/0', event=quit_event)
    pilot = ReceiverThread(url='ipc:///byodr/pilot.sock', topic=b'aav/pilot/output', event=quit_event)
    vehicle = ReceiverThread(url='ipc:///byodr/vehicle.sock', topic=b'aav/vehicle/state', event=quit_event)
    handler = EventHandler(directory=sessions_dir, event=quit_event, **cfg)
    threads.append(camera)
    threads.append(pilot)
    threads.append(vehicle)
    threads.append(handler)
    [t.start() for t in threads]

    try:
        _last_publish = time.time()
        while not quit_event.is_set():
            proc_start = time.time()
            blob = pilot.get_latest()
            image_md, image = camera.capture()
            if None not in (blob, image_md):
                handler.record(blob, vehicle.get_latest(), image)
            if time.time() - _last_publish > max_publish_duration:
                state_publisher.publish(handler.state())
                _last_publish = time.time()
            _proc_sleep = max_process_duration - (time.time() - proc_start)
            if _proc_sleep < 0:
                logger.warning("Cannot maintain {} Hz.".format(_process_frequency))
            time.sleep(max(0., _proc_sleep))
    except KeyboardInterrupt:
        quit_event.set()
    except StandardError as e:
        logger.error("{}".format(traceback.format_exc(e)))
        quit_event.set()

    logger.info("Waiting on threads to stop.")
    [t.join() for t in threads]


if __name__ == "__main__":
    logging.basicConfig(format='%(levelname)s: %(filename)s %(funcName)s %(message)s')
    logging.getLogger().setLevel(logging.DEBUG)
    main()
