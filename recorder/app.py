from __future__ import absolute_import
import argparse
import collections
import copy
import glob
import logging
import os
import threading
import time
from datetime import datetime

import cv2
import numpy as np
from six.moves.configparser import SafeConfigParser
from store import Event

from byodr.utils import Application
from byodr.utils.ipc import CameraThread, JSONPublisher, LocalIPCServer, json_collector
from byodr.utils.option import parse_option, hash_dict
from recorder import get_or_create_recorder
from io import open

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
        x_coordinate=vehicle.get('latitude_geo'),
        y_coordinate=vehicle.get('longitude_geo'),
        inference_brake=blob.get('inference_brake')
    )
    event.save_event = blob.get('save_event')
    return event


def get_ts(val):
    return val.get('time')


def _get_directory(directory, mode=0o775):
    if not os.path.exists(directory):
        _mask = os.umask(000)
        os.makedirs(directory, mode=mode)
        os.umask(_mask)
    return directory


class ImageEventLog(object):
    """
    Select the closest data to the image by timestamp.
    """

    def __init__(self, buffer_size=int(1e4), window_ms=30):
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
            d_vehicle = abs(img_ts - get_ts(vehicle))
            if d_pilot <= self._micro and d_vehicle <= self._micro:
                self._events.append(to_event(img_ts, copy.deepcopy(pilot), copy.deepcopy(vehicle), np.copy(image)))
            else:
                logger.info("Data window violation of pilot {} ms and vehicle {} ms - skipping image {}.".format(
                    d_pilot * 1e-3, d_vehicle * 1e-3, img_ts)
                )

    def pop(self):
        # Maintain fifo ordering.
        return self._events.popleft() if self._events else None


class EventHandler(threading.Thread):
    def __init__(self, sessions, photos, **kwargs):
        super(EventHandler, self).__init__()
        self._hash = hash_dict(**kwargs)
        self._record_dir = sessions
        self._photo_dir = photos
        self._quit_event = threading.Event()
        _errors = []
        self._process_frequency = parse_option('clock.hz', int, 100, _errors, **kwargs)
        self._publish_frequency = parse_option('publish.hz', int, 2, _errors, **kwargs)
        self._vehicle = parse_option('constant.vehicle.type', str, 'vehicle.byodr.2020', _errors, **kwargs)
        self._config = parse_option('constant.vehicle.config', str, 'latest', _errors, **kwargs)
        _im_width, _im_height = parse_option('image.persist.scale', str, '320x240', _errors, **kwargs).split('x')
        self._im_height = int(_im_height)
        self._im_width = int(_im_width)
        self._session_active = False
        self._session_log = ImageEventLog()
        self._photo_log = ImageEventLog()
        self._recorder = get_or_create_recorder(mode='record.mode.interventions',
                                                directory=self._record_dir,
                                                vehicle_type=self._vehicle,
                                                vehicle_config=self._config)
        self._errors = _errors

    @staticmethod
    def _cruising(mode):
        return mode == 'driver_mode.teleop.cruise'

    @staticmethod
    def _automatic(mode):
        return mode in ('driver_mode.inference.dnn', 'driver_mode.automatic.backend')

    def get_errors(self):
        return self._errors

    def get_application_frequency(self):
        # The application process frequency, not of this handler.
        return self._process_frequency

    def get_publish_frequency(self):
        return self._publish_frequency

    def is_reconfigured(self, **kwargs):
        return self._hash != hash_dict(**kwargs)

    def state(self):
        return dict(active=self._session_active, mode=self._recorder.get_mode())

    def record(self, blob, vehicle, image_meta, image):
        if blob.get('button_right', 0) == 1:
            self._photo_log.append(blob, vehicle, image_meta, image)
        else:
            # Switch on automatically and then off only when the driver changes.
            _driver = blob.get('driver')
            _recorder_auto = self._automatic(_driver)
            _recorder_cruise = self._cruising(_driver)
            if not self._session_active and (_recorder_auto or _recorder_cruise):
                self._session_active = True
            elif self._session_active and not (_recorder_auto or _recorder_cruise):
                self._session_active = False
            if self._session_active:
                self._session_log.append(blob, vehicle, image_meta, image)

    def quit(self):
        self._quit_event.set()

    def _pop_from(self, tracker):
        event = tracker.pop()
        if event is not None:
            # The resize operation can be considered costly - also do not transfer excessive bytes until the end of the line.
            if event.image.shape != (self._im_height, self._im_width, 3):
                logger.warning("Image size must be finalized as soon as possible.")
                event.image = cv2.resize(event.image, (self._im_width, self._im_height))
        return event

    def _save_photo(self, event):
        _now = datetime.today()
        _directory = os.path.join(self._photo_dir, _now.strftime('%Y%B'))
        if not os.path.exists(_directory):
            _mask = os.umask(000)
            os.makedirs(_directory, mode=0o775)
            os.umask(_mask)
        latitude = event.x_coordinate
        longitude = event.y_coordinate
        fname = "{}_lat{}_long{}.jpg".format(_now.strftime('%Y%b%dT%H%M%S'),
                                             str(latitude)[:8].replace('.', '_'),
                                             str(longitude)[:8].replace('.', '_'))
        cv2.imwrite(os.path.join(_directory, fname), event.image)
        with open(os.path.join(_directory, 'photo.log'), 'a+') as f:
            f.write("{} {} latitude {} longitude {}\r\n".format(event.timestamp, fname, latitude, longitude))

    def run(self):
        while not self._quit_event.is_set():
            try:
                event = self._pop_from(self._session_log)
                if event is not None:
                    self._recorder.do_record(event)
                elif not self._session_active:
                    self._recorder.flush()
                event = self._pop_from(self._photo_log)
                if event is not None:
                    self._save_photo(event)
                # Allow other threads access to cpu.
                else:
                    time.sleep(1e-3)
            except IndexError:
                pass


class RecorderApplication(Application):
    def __init__(self, config_dir=os.getcwd(), sessions_dir=os.getcwd()):
        super(RecorderApplication, self).__init__()
        self._config_dir = config_dir
        self._recorder_dir = _get_directory(os.path.join(sessions_dir, 'autopilot'))
        self._photo_dir = _get_directory(os.path.join(sessions_dir, 'photos', 'cam0'))
        self._handler = None
        self.publisher = None
        self.ipc_server = None
        self.camera = None
        self.pilot = None
        self.vehicle = None
        self.ipc_chatter = None
        self._last_publish = time.time()
        self._publish_duration = 0
        self._config_hash = -1

    def _config(self):
        parser = SafeConfigParser()
        [parser.read(_f) for _f in glob.glob(os.path.join(self._config_dir, '*.ini'))]
        cfg = dict(parser.items('recorder')) if parser.has_section('recorder') else {}
        self.logger.info(cfg)
        return cfg

    def get_process_frequency(self):
        return 0 if self._handler is None else self._handler.get_application_frequency()

    def setup(self):
        if self.active():
            _config = self._config()
            _hash = hash_dict(**_config)
            if _hash != self._config_hash:
                self._config_hash = _hash
                if self._handler is not None:
                    self._handler.quit()
                self._handler = EventHandler(sessions=self._recorder_dir, photos=self._photo_dir, **_config)
                self._handler.start()
                self.ipc_server.register_start(self._handler.get_errors())
                _process_hz = self._handler.get_application_frequency()
                _publish_hz = self._handler.get_publish_frequency()
                self.set_hz(_process_hz)
                self._publish_duration = 1. / _publish_hz
                self.logger.info("Processing at {} Hz publishing at {} Hz.".format(_process_hz, _publish_hz))

    def finish(self):
        if self._handler is not None:
            self._handler.quit()

    def step(self):
        m_pilot = self.pilot()
        m_vehicle = self.vehicle()
        image_md, image = self.camera.capture()
        if None not in (m_pilot, m_vehicle, image_md):
            self._handler.record(m_pilot, m_vehicle, image_md, image)
        if time.time() - self._last_publish > self._publish_duration:
            self.publisher.publish(self._handler.state())
            self._last_publish = time.time()
        chat = self.ipc_chatter()
        if chat and chat.get('command') == 'restart':
            self.setup()


def main():
    parser = argparse.ArgumentParser(description='Recorder.')
    parser.add_argument('--name', type=str, default='none', help='Process name.')
    parser.add_argument('--sessions', type=str, default='/sessions', help='Sessions directory.')
    parser.add_argument('--config', type=str, default='/config', help='Config directory path.')
    args = parser.parse_args()

    sessions_dir = os.path.expanduser(args.sessions)
    assert os.path.exists(sessions_dir), "Cannot use sessions directory '{}'".format(sessions_dir)

    application = RecorderApplication(config_dir=args.config, sessions_dir=sessions_dir)
    quit_event = application.quit_event

    pilot = json_collector(url='ipc:///byodr/pilot.sock', topic=b'aav/pilot/output', event=quit_event)
    vehicle = json_collector(url='ipc:///byodr/vehicle.sock', topic=b'aav/vehicle/state', event=quit_event)
    ipc_chatter = json_collector(url='ipc:///byodr/teleop_c.sock', topic=b'aav/teleop/chatter', pop=True, event=quit_event)

    application.publisher = JSONPublisher(url='ipc:///byodr/recorder.sock', topic='aav/recorder/state')
    application.camera = CameraThread(url='ipc:///byodr/camera_0.sock', topic=b'aav/camera/0', event=quit_event)
    application.ipc_server = LocalIPCServer(url='ipc:///byodr/recorder_c.sock', name='recorder', event=quit_event)
    application.pilot = lambda: pilot.get()
    application.vehicle = lambda: vehicle.get()
    application.ipc_chatter = lambda: ipc_chatter.get()

    threads = [pilot, vehicle, ipc_chatter, application.camera, application.ipc_server]
    if quit_event.is_set():
        return 0

    [t.start() for t in threads]
    application.run()

    logger.info("Waiting on threads to stop.")
    [t.join() for t in threads]


if __name__ == "__main__":
    logging.basicConfig(format='%(levelname)s: %(asctime)s %(filename)s %(funcName)s %(message)s', datefmt='%Y%m%d:%H:%M:%S %p %Z')
    logging.getLogger().setLevel(logging.INFO)
    main()
