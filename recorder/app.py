import argparse
import collections
import copy
import glob
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


def to_event(ts, blob, vehicle, image):
    event = Event(
        timestamp=ts,
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


def get_ts(val):
    return val.get('time')


class ImageEventLog(object):
    """
    Select the closest data to the image by timestamp.
    """

    def __init__(self, buffer_size=int(2e3), window_ms=30):
        self._micro = window_ms * 1e3
        self._observed = None
        self._events = collections.deque(maxlen=buffer_size)

    def clear(self):
        self._events.clear()

    def append(self, pilot, vehicle, image_meta, image):
        # The image timestamp decides the event timestamp.
        img_ts = get_ts(image_meta)
        # Skip duplicate processing.
        if self._observed != img_ts:
            self._observed = img_ts
            d_pilot = abs(img_ts - get_ts(pilot))
            if d_pilot <= self._micro:
                self._events.append(to_event(img_ts, copy.deepcopy(pilot), copy.deepcopy(vehicle), np.copy(image)))
            else:
                d_vehicle = abs(img_ts - get_ts(vehicle))
                logger.info("Data window violation of pilot {} ms and vehicle {} ms - skipping image {}.".format(
                    d_pilot * 1e-3, d_vehicle * 1e-3, img_ts)
                )

    def pop(self):
        return self._events.pop() if self._events else None


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
        self._tracker = ImageEventLog()
        self._recorder = get_or_create_recorder(directory=directory, mode=None)
        self._recent = collections.deque(maxlen=100)

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

    def record(self, blob, vehicle, image_meta, image):
        # Switch on automatically and then off only when the driver changes.
        _driver = blob.get('driver')
        _recorder_dnn = _driver == 'driver_mode.inference.dnn'
        _recorder_cruise = _driver == 'driver_mode.teleop.cruise' and blob.get('cruise_speed') > 1e-3
        if not self._active and (_recorder_dnn or _recorder_cruise):
            self._tracker.clear()
            self._recorder = self._instance(_driver)
            self._recorder.start()
            self._active = True
        elif self._active and not (_recorder_dnn or _recorder_cruise):
            self._recorder.stop()
            self._recorder = self._instance(_driver)
            self._active = False
        if self._active:
            self._tracker.append(blob, vehicle, image_meta, image)

    def run(self):
        while not self._quit_event.is_set():
            try:
                event = self._tracker.pop()
                if event is not None:
                    if event.image.shape != (self._im_height, self._im_width, 3):
                        event.image = cv2.resize(event.image, (self._im_width, self._im_height))
                    self._recorder.do_record(event)
                # Allow other threads access to cpu.
                time.sleep(2e-3)
            except IndexError:
                pass


def main():
    parser = argparse.ArgumentParser(description='Recorder.')
    parser.add_argument('--sessions', type=str, default='/sessions', help='Sessions directory.')
    parser.add_argument('--config', type=str, default='/config', help='Config directory path.')
    args = parser.parse_args()

    sessions_dir = os.path.expanduser(args.sessions)
    assert os.path.exists(sessions_dir), "Cannot use sessions directory '{}'".format(sessions_dir)

    parser = SafeConfigParser()
    [parser.read(_f) for _f in ['config.ini'] + glob.glob(os.path.join(args.config, '*.ini'))]
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
            m_pilot = pilot.get_latest()
            m_vehicle = vehicle.get_latest()
            image_md, image = camera.capture()
            if None not in (m_pilot, m_vehicle, image_md):
                handler.record(m_pilot, m_vehicle, image_md, image)
            if time.time() - _last_publish > max_publish_duration:
                state_publisher.publish(handler.state())
                _last_publish = time.time()
            _proc_sleep = max_process_duration - (time.time() - proc_start)
            # if _proc_sleep < 0: logger.warning("Cannot maintain {} Hz.".format(_process_frequency))
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
    logging.getLogger().setLevel(logging.INFO)
    main()
