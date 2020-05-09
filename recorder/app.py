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

from byodr.utils.ipc import ReceiverThread, CameraThread, JSONPublisher, LocalIPCServer
from byodr.utils.option import parse_option, hash_dict
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
        # Maintain fifo ordering.
        return self._events.popleft() if self._events else None


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

    @staticmethod
    def _cruising(mode):
        return mode in ('driver_mode.teleop.cruise', 'driver_mode.automatic.backend')

    def _instance(self, driver):
        mode = None
        if self._cruising(driver):
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
        _recorder_cruise = self._cruising(_driver)
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


class IPCServer(LocalIPCServer):
    def __init__(self, url, event, receive_timeout_ms=50):
        super(IPCServer, self).__init__('recorder', url, event, receive_timeout_ms)

    def serve_local(self, message):
        return {}


class LocalHandler(object):
    def __init__(self, sessions_dir, **kwargs):
        self._hash = -1
        self._errors = []
        self._process_frequency = 1
        self._publish_frequency = 1
        self._event_handler = EventHandler(directory=sessions_dir, event=quit_event, **kwargs)
        self.reload(**kwargs)

    def get_errors(self):
        return self._errors

    def get_process_frequency(self):
        return self._process_frequency

    def get_publish_frequency(self):
        return self._publish_frequency

    def is_reconfigured(self, **kwargs):
        return self._hash != hash_dict(**kwargs)

    def start(self):
        self._event_handler.start()

    def reload(self, **kwargs):
        _errors = []
        self._hash = hash_dict(**kwargs)
        self._process_frequency = parse_option('clock.hz', int, 10, _errors, **kwargs)
        self._publish_frequency = parse_option('publish.hz', int, 1, _errors, **kwargs)
        self._errors = _errors

    def state(self):
        return self._event_handler.state()

    def record(self, blob, vehicle, image_meta, image):
        self._event_handler.record(blob, vehicle, image_meta, image)

    def join(self):
        self._event_handler.join()


def create_handler(ipc_server, config_dir, sessions_dir, previous=None):
    parser = SafeConfigParser()
    [parser.read(_f) for _f in ['config.ini'] + glob.glob(os.path.join(config_dir, '*.ini'))]
    cfg = dict(parser.items('recorder'))
    _configured = False
    if previous is None:
        previous = LocalHandler(sessions_dir, **cfg)
        _configured = True
    elif previous.is_reconfigured(**cfg):
        previous.reload(**cfg)
        _configured = True
    if _configured:
        ipc_server.register_start(previous.get_errors())
        logger.info("Processing at {} Hz publishing at {} Hz.".format(previous.get_process_frequency(), previous.get_publish_frequency()))
    return previous


def main():
    parser = argparse.ArgumentParser(description='Recorder.')
    parser.add_argument('--sessions', type=str, default='/sessions', help='Sessions directory.')
    parser.add_argument('--config', type=str, default='/config', help='Config directory path.')
    args = parser.parse_args()

    sessions_dir = os.path.expanduser(args.sessions)
    assert os.path.exists(sessions_dir), "Cannot use sessions directory '{}'".format(sessions_dir)

    camera = CameraThread(url='ipc:///byodr/camera.sock', topic=b'aav/camera/0', event=quit_event)
    pilot = ReceiverThread(url='ipc:///byodr/pilot.sock', topic=b'aav/pilot/output', event=quit_event)
    vehicle = ReceiverThread(url='ipc:///byodr/vehicle.sock', topic=b'aav/vehicle/state', event=quit_event)
    ipc_chatter = ReceiverThread(url='ipc:///byodr/teleop.sock', topic=b'aav/teleop/chatter', event=quit_event)
    ipc_server = IPCServer(url='ipc:///byodr/recorder_c.sock', event=quit_event)
    handler = create_handler(ipc_server, args.config, sessions_dir)
    threads = [camera, pilot, vehicle, handler, ipc_chatter, ipc_server]
    if quit_event.is_set():
        return 0

    state_publisher = JSONPublisher(url='ipc:///byodr/recorder.sock', topic='aav/recorder/state')
    [t.start() for t in threads]
    max_process_duration = 1. / handler.get_process_frequency()
    max_publish_duration = 1. / handler.get_publish_frequency()
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
            chat = ipc_chatter.pop_latest()
            if chat and chat.get('command') == 'restart':
                handler = create_handler(ipc_server, args.config, sessions_dir, previous=handler)
                max_process_duration = 1. / handler.get_process_frequency()
                max_publish_duration = 1. / handler.get_publish_frequency()
            else:
                _proc_sleep = max_process_duration - (time.time() - proc_start)
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
