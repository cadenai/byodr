from __future__ import absolute_import

import argparse
import asyncio
import collections
import logging
import multiprocessing
import os
import signal
import sys
import threading
import traceback

import cv2
import numpy as np
from bson.binary import Binary
from pymongo import MongoClient
from tornado import web, ioloop
from tornado.httpserver import HTTPServer
from tornado.platform.asyncio import AnyThreadEventLoopPolicy

from byodr.utils import Application, timestamp
from byodr.utils.ipc import CameraThread, json_collector
from byodr.utils.option import parse_option
from . import jpeg_encode
from .store import Event, create_data_source
from .web import DataTableRequestHandler, JPEGImageRequestHandler

if sys.version_info > (3,):
    pass
else:
    pass

logger = logging.getLogger(__name__)

TRIGGER_SERVICE_START = 2 ** 0
TRIGGER_SERVICE_END = 2 ** 1
TRIGGER_SERVICE_STEP = 2 ** 2

signal.signal(signal.SIGINT, lambda sig, frame: _interrupt())
signal.signal(signal.SIGTERM, lambda sig, frame: _interrupt())

quit_event = multiprocessing.Event()


def _interrupt():
    logger.info("Received interrupt, quitting.")
    quit_event.set()


def _str(c, attr):
    return 'null' if (c is None or c.get(attr) is None) else str(c.get(attr))


def _bool(c, attr):
    return 1 if (c is not None and bool(c.get(attr))) else 0


def _get_ts(c, default=-1):
    return default if c is None else c.get('time')


def _driver_mode(pil):
    dr = None if pil is None else pil.get('driver')
    if dr == 'driver_mode.teleop.direct':
        return 1
    elif dr == 'driver_mode.inference.dnn':
        return 2
    else:
        return 0


def _driver_sub_type(pil, attr):
    x = _str(pil, attr)
    if x == 'src.console':
        return 1
    elif x == 'src.dnn.pre-intervention':
        return 2
    elif x == 'src.dnn':
        return 4
    else:
        return 0


class SharedMem(object):
    def __init__(self):
        self._user_busy = collections.deque(maxlen=1)

    def touch(self):
        self._user_busy.append(timestamp())

    def get(self):
        return self._user_busy[0] if len(self._user_busy) > 0 else 0

    def is_busy(self, wait_ms=3e5):
        return timestamp() - self.get() < (wait_ms * 1e3)


class LogApplication(Application):
    def __init__(self, mongo, mem, event, hz=10, config_dir=os.getcwd()):
        super(LogApplication, self).__init__(quit_event=event, run_hz=hz)
        self._database = mongo.logbox
        self._mem = mem
        self._config_dir = config_dir
        self._patience = 1.5 * (1e6 / self.get_hz())
        self.camera = None
        self.pilot = None
        self.vehicle = None
        self.inference = None

    def _states(self, stamp):
        # Ignore outdated data with nil.
        image_md, image = self.camera.capture()
        pil, veh, inf = self.pilot(), self.vehicle(), self.inference()
        _li = list([None if x is None or abs(stamp - _get_ts(x)) > self._patience else x for x in (pil, veh, inf, image_md)])
        pil, veh, inf, image_md = _li
        return pil, veh, inf, image_md, (None if image_md is None else image)

    @staticmethod
    def _image_fields(image, persist=True):
        _shape, _num_bytes, _buffer = -1, -1, -1
        if persist and image is not None:
            if image.shape != (240, 320, 3):
                image = cv2.resize(image, (320, 240))
            _shape = image.shape
            _img_bytes = jpeg_encode(image).tobytes()
            _num_bytes = len(_img_bytes)
            _buffer = Binary(_img_bytes)
        return _shape, _num_bytes, _buffer

    def _insert(self, trigger, force=False):
        _time = timestamp()
        _busy = False
        # The pilot message is nil when no party is driving.
        # The pilot message save_event attribute is set to true by the autopilot driver to indicate the image needs recording.
        pil, veh, inf, image_md, image = self._states(_time)
        _pil_ok = pil is not None and pil.get('driver') is not None
        _pil_ok = _pil_ok and (bool(pil.get('forced_steering', 0)) or bool(pil.get('forced_throttle', 0)))
        _busy = _pil_ok
        if force or _pil_ok:
            _save_img = pil is not None and bool(pil.get('save_event', 0))
            _busy = _busy or _save_img
            _img_fields = self._image_fields(image, persist=(force or _save_img))
            self._database.events.insert_one({
                'time': _time,
                'trigger': trigger,
                'pil_time': _get_ts(pil),
                'pil_cruise_speed': _str(pil, 'cruise_speed'),
                'pil_desired_speed': _str(pil, 'desired_speed'),
                'pil_driver_mode': _driver_mode(pil),
                'pil_steering_driver': _driver_sub_type(pil, 'steering_driver'),
                'pil_speed_driver': _driver_sub_type(pil, 'speed_driver'),
                'pil_is_steering_intervention': _bool(pil, 'forced_steering'),
                'pil_is_throttle_intervention': _bool(pil, 'forced_throttle'),
                'pil_is_save_event': _bool(pil, 'save_event'),
                'pil_steering': _str(pil, 'steering'),
                'pil_throttle': _str(pil, 'throttle'),
                'veh_time': _get_ts(veh),
                'veh_heading': _str(veh, 'heading'),
                'veh_gps_latitude': _str(veh, 'latitude_geo'),
                'veh_gps_longitude': _str(veh, 'longitude_geo'),
                'veh_velocity': _str(veh, 'velocity'),
                'veh_is_velocity_trusted': _bool(veh, 'trust_velocity'),
                'inf_time': _get_ts(inf),
                'inf_steer_action': _str(inf, 'action'),
                'inf_obstruction': _str(inf, 'obstacle'),
                'inf_steer_penalty': _str(inf, 'steer_penalty'),
                'inf_obstruction_penalty': _str(inf, 'brake_penalty'),
                'inf_running_penalty': _str(inf, 'total_penalty'),
                'img_time': _get_ts(image_md),
                'img_shape': _img_fields[0],
                'img_num_bytes': _img_fields[1],
                'img_buffer': _img_fields[2],
                'lb_zip_packaged': 0
            })
        if _busy:
            self._mem.touch()

    def setup(self):
        self._insert(TRIGGER_SERVICE_START, force=True)

    def finish(self):
        self._insert(TRIGGER_SERVICE_END, force=True)

    def step(self):
        self._insert(TRIGGER_SERVICE_STEP)


def _get_directory(directory, mode=0o775):
    if not os.path.exists(directory):
        _mask = os.umask(000)
        os.makedirs(directory, mode=mode)
        os.umask(_mask)
    return directory


class PackageApplication(Application):
    def __init__(self, mongo, mem, event, hz=1e-2, sessions_dir=os.getcwd()):
        super(PackageApplication, self).__init__(quit_event=event, run_hz=hz)
        self._database = mongo.logbox
        self._mem = mem
        self._recorder_dir = _get_directory(os.path.join(sessions_dir, 'autopilot'))
        self._vehicle = parse_option('constant.vehicle.type', str, 'vehicle.byodr.2020', [], **{})
        self._config = parse_option('constant.vehicle.config', str, 'latest', [], **{})

    def setup(self):
        pass

    def finish(self):
        pass

    def _mark(self, item):
        self._database.events.update_one({'_id': item.get('_id')}, {'$set': {"lb_zip_packaged": 1}})

    def _event(self, row):
        event = Event(
            timestamp=row.get('time'),
            image_shape=row.get('img_shape'),
            jpeg_image=np.frombuffer(memoryview(row.get('img_buffer')), dtype=np.uint8),
            steer_src=row.get('pil_steering_driver'),
            speed_src=row.get('pil_speed_driver'),
            command_src=row.get('pil_steering_driver'),
            steering=row.get('pil_steering'),
            desired_speed=row.get('pil_desired_speed'),
            actual_speed=row.get('veh_velocity'),
            heading=row.get('veh_heading'),
            throttle=row.get('pil_throttle'),
            command=None,
            x_coordinate=row.get('veh_gps_latitude'),
            y_coordinate=row.get('veh_gps_longitude'),
            inference_brake=row.get('inf_obstruction')
        )
        event.vehicle = self._vehicle
        event.vehicle_config = self._config
        return event

    def _zip(self):
        if self._mem.is_busy():
            return False

        _filter = {'pil_is_save_event': 1, 'lb_zip_packaged': 0, 'img_num_bytes': {'$gt': 0}}
        cursor = self._database.events.find(filter=_filter, sort=[('time', -1)], batch_size=1000, limit=1000)
        items = list(cursor)
        if len(items) < 1:
            return False

        _timestamp = items[-1].get('time')
        _archive = create_data_source(_timestamp, self._recorder_dir)
        try:
            _archive.open()
            assert _archive.is_open(), "Could not create a new archive."
            # Write the zip in time ascending order.
            for row in reversed(items):
                _archive.create_event(self._event(row))
            _archive.close()
            list([self._mark(x) for x in items])
        except Exception as e:
            logger.warning(e)
            logger.error("Recorder#do_record: {}".format(traceback.format_exc()))
            return False
        return not self._mem.is_busy()

    def step(self):
        _package = True
        while _package:
            _package = self._zip()


def main():
    parser = argparse.ArgumentParser(description='Logger black box service.')
    parser.add_argument('--name', type=str, default='none', help='Process name.')
    parser.add_argument('--sessions', type=str, default='/sessions', help='Sessions directory.')
    parser.add_argument('--config', type=str, default='/config', help='Config directory path.')
    args = parser.parse_args()

    # The mongo client is thread-safe and provides for transparent connection pooling.
    _mongo = MongoClient()

    shared_mem = SharedMem()
    log_application = LogApplication(_mongo, shared_mem, quit_event, hz=20, config_dir=args.config)
    package_application = PackageApplication(_mongo, shared_mem, quit_event, hz=0.01666666667, sessions_dir=args.sessions)

    pilot = json_collector(url='ipc:///byodr/pilot.sock', topic=b'aav/pilot/output', event=quit_event)
    vehicle = json_collector(url='ipc:///byodr/vehicle.sock', topic=b'aav/vehicle/state', event=quit_event)
    inference = json_collector(url='ipc:///byodr/inference.sock', topic=b'aav/inference/state', event=quit_event)

    log_application.camera = CameraThread(url='ipc:///byodr/camera_0.sock', topic=b'aav/camera/0', event=quit_event)
    log_application.pilot = lambda: pilot.get()
    log_application.vehicle = lambda: vehicle.get()
    log_application.inference = lambda: inference.get()

    log_thread = threading.Thread(target=log_application.run)
    package_thread = threading.Thread(target=package_application.run)
    threads = [log_application.camera, pilot, vehicle, inference, log_thread, package_thread]
    if quit_event.is_set():
        return 0

    [t.start() for t in threads]

    asyncio.set_event_loop_policy(AnyThreadEventLoopPolicy())
    asyncio.set_event_loop(asyncio.new_event_loop())

    try:
        io_loop = ioloop.IOLoop.instance()
        main_app = web.Application([
            (r"/api/datalog/event/v10/table", DataTableRequestHandler, dict(mongo_client=_mongo)),
            (r"/api/datalog/event/v10/image", JPEGImageRequestHandler, dict(mongo_client=_mongo))
        ])
        http_server = HTTPServer(main_app, xheaders=True)
        http_server.bind(8085)
        http_server.start()
        io_loop.start()
        logger.info("Data table web services started on port 8085.")
    except KeyboardInterrupt:
        quit_event.set()
    finally:
        _mongo.close()

    logger.info("Waiting on threads to stop.")
    [t.join() for t in threads]


if __name__ == "__main__":
    logging.basicConfig(format='%(levelname)s: %(asctime)s %(filename)s %(funcName)s %(message)s', datefmt='%Y%m%d:%H:%M:%S %p %Z')
    logging.getLogger().setLevel(logging.INFO)
    main()
