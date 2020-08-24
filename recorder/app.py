import argparse
import collections
import copy
import glob
import logging
import os
import threading
import time
from ConfigParser import SafeConfigParser

import cv2
import numpy as np

from byodr.utils import Application
from byodr.utils.ipc import ReceiverThread, CameraThread, JSONPublisher, LocalIPCServer
from byodr.utils.option import parse_option, hash_dict
from recorder import get_or_create_recorder
from store import Event

logger = logging.getLogger(__name__)


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
    def __init__(self, directory, **kwargs):
        super(EventHandler, self).__init__()
        _errors = []
        self._hash = hash_dict(**kwargs)
        self._directory = directory
        self._quit_event = threading.Event()
        self._process_frequency = parse_option('clock.hz', int, 10, _errors, **kwargs)
        self._publish_frequency = parse_option('publish.hz', int, 1, _errors, **kwargs)
        self._vehicle = parse_option('constant.vehicle.type', str, 'none', _errors, **kwargs)
        self._config = parse_option('constant.vehicle.config', str, 'none', _errors, **kwargs)
        _im_height, _im_width = parse_option('image.persist.scale', str, '240x320', _errors, **kwargs).split('x')
        self._im_height = int(_im_height)
        self._im_width = int(_im_width)
        self._active = False
        self._tracker = ImageEventLog()
        self._recorder = get_or_create_recorder(directory=directory, mode=None)
        self._recent = collections.deque(maxlen=100)
        self._errors = _errors

    @staticmethod
    def _cruising(mode):
        return mode == 'driver_mode.teleop.cruise'

    @staticmethod
    def _automatic(mode):
        return mode in ('driver_mode.inference.dnn', 'driver_mode.automatic.backend')

    def _instance(self, driver):
        mode = None
        if self._cruising(driver):
            mode = 'record.mode.driving'
        elif self._automatic(driver):
            mode = 'record.mode.interventions'
        return get_or_create_recorder(mode=mode,
                                      directory=self._directory,
                                      vehicle_type=self._vehicle,
                                      vehicle_config=self._config)

    def get_errors(self):
        return self._errors

    def get_process_frequency(self):
        return self._process_frequency

    def get_publish_frequency(self):
        return self._publish_frequency

    def is_reconfigured(self, **kwargs):
        return self._hash != hash_dict(**kwargs)

    def state(self):
        return dict(active=self._active, mode=self._recorder.get_mode())

    def record(self, blob, vehicle, image_meta, image):
        # Switch on automatically and then off only when the driver changes.
        _driver = blob.get('driver')
        _recorder_auto = self._automatic(_driver)
        _recorder_cruise = self._cruising(_driver)
        if not self._active and (_recorder_auto or _recorder_cruise):
            self._tracker.clear()
            self._recorder = self._instance(_driver)
            self._recorder.start()
            self._active = True
        elif self._active and not (_recorder_auto or _recorder_cruise):
            self._recorder.stop()
            self._recorder = self._instance(_driver)
            self._active = False
        if self._active:
            self._tracker.append(blob, vehicle, image_meta, image)

    def quit(self):
        self._quit_event.set()

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


class RecorderApplication(Application):
    def __init__(self, config_dir=os.getcwd(), sessions_dir=os.getcwd()):
        super(RecorderApplication, self).__init__()
        self._config_dir = config_dir
        self._sessions_dir = sessions_dir
        self._handler = None
        self.publisher = None
        self.ipc_server = None
        self.ipc_chatter = None
        self.camera = None
        self.pilot = None
        self.vehicle = None
        self._last_publish = time.time()
        self._publish_duration = 0

    def _config(self):
        parser = SafeConfigParser()
        [parser.read(_f) for _f in ['config.ini'] + glob.glob(os.path.join(self._config_dir, '*.ini'))]
        return dict(parser.items('recorder'))

    def get_process_frequency(self):
        return 0 if self._handler is None else self._handler.get_process_frequency()

    def setup(self):
        if self._handler is not None:
            self._handler.quit()
        if self.active():
            self._handler = EventHandler(self._sessions_dir, **self._config())
            self.ipc_server.register_start(self._handler.get_errors())
            _process_hz = self._handler.get_process_frequency()
            _publish_hz = self._handler.get_publish_frequency()
            self.set_hz(_process_hz)
            self._publish_duration = 1. / _publish_hz
            self.logger.info("Processing at {} Hz publishing at {} Hz.".format(_process_hz, _publish_hz))

    def finish(self):
        self._handler.quit()

    def step(self):
        m_pilot = self.pilot.get_latest()
        m_vehicle = self.vehicle.get_latest()
        image_md, image = self.camera.capture()
        if None not in (m_pilot, m_vehicle, image_md):
            self._handler.record(m_pilot, m_vehicle, image_md, image)
        if time.time() - self._last_publish > self._publish_duration:
            self.publisher.publish(self._handler.state())
            self._last_publish = time.time()
        chat = self.ipc_chatter.pop_latest()
        if chat and chat.get('command') == 'restart':
            self.setup()


def main():
    parser = argparse.ArgumentParser(description='Recorder.')
    parser.add_argument('--sessions', type=str, default='/sessions', help='Sessions directory.')
    parser.add_argument('--config', type=str, default='/config', help='Config directory path.')
    args = parser.parse_args()

    sessions_dir = os.path.expanduser(args.sessions)
    assert os.path.exists(sessions_dir), "Cannot use sessions directory '{}'".format(sessions_dir)

    application = RecorderApplication(config_dir=args.config, sessions_dir=sessions_dir)
    quit_event = application.quit_event

    application.publisher = JSONPublisher(url='ipc:///byodr/recorder.sock', topic='aav/recorder/state')
    application.camera = CameraThread(url='ipc:///byodr/camera.sock', topic=b'aav/camera/0', event=quit_event)
    application.pilot = ReceiverThread(url='ipc:///byodr/pilot.sock', topic=b'aav/pilot/output', event=quit_event)
    application.vehicle = ReceiverThread(url='ipc:///byodr/vehicle.sock', topic=b'aav/vehicle/state', event=quit_event)
    application.ipc_chatter = ReceiverThread(url='ipc:///byodr/teleop.sock', topic=b'aav/teleop/chatter', event=quit_event)
    application.ipc_server = LocalIPCServer(url='ipc:///byodr/recorder_c.sock', name='recorder', event=quit_event)

    threads = [application.camera, application.pilot, application.vehicle, application.ipc_chatter, application.ipc_server]
    if quit_event.is_set():
        return 0

    [t.start() for t in threads]
    application.run()

    logger.info("Waiting on threads to stop.")
    [t.join() for t in threads]


if __name__ == "__main__":
    logging.basicConfig(format='%(levelname)s: %(filename)s %(funcName)s %(message)s')
    logging.getLogger().setLevel(logging.INFO)
    main()
