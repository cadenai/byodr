from __future__ import absolute_import

import argparse
import asyncio
import logging
import multiprocessing
import os
import signal
import sys
import threading

import cv2
from bson.binary import Binary
from pymongo import MongoClient
from tornado import web, ioloop
from tornado.httpserver import HTTPServer
from tornado.platform.asyncio import AnyThreadEventLoopPolicy

from byodr.utils import Application, timestamp
from byodr.utils.ipc import CameraThread, json_collector
from . import jpeg_encode
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
    return '1' if (c is not None and bool(c.get(attr))) else '0'


def _get_ts(c, default=-1):
    return default if c is None else c.get('time')


def _driver_type(pil):
    dr = None if pil is None else pil.get('driver')
    # driver_mode.teleop.direct or driver_mode.inference.dnn
    if dr == 'driver_mode.teleop.direct':
        return 1
    elif dr == 'driver_mode.inference.dnn':
        return 2
    else:
        return 0


class LogApplication(Application):
    def __init__(self, mongo, event, hz=10, config_dir=os.getcwd()):
        super(LogApplication, self).__init__(quit_event=event, run_hz=hz)
        self._config_dir = config_dir
        self._patience = 1.5 * (1e6 / self.get_hz())
        self._database = mongo.logbox
        self.camera = None
        self.pilot = None
        self.vehicle = None
        self.inference = None

    def _states(self, stamp):
        # Ignore outdated data with nil.
        image_md, image = self.camera.capture()
        pil, veh, inf = self.pilot(), self.vehicle(), self.inference()
        _li = list(map(lambda x: None if x is None or abs(stamp - _get_ts(x)) > self._patience else x, (pil, veh, inf, image_md)))
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
        # The pilot message is nil when no party is driving.
        # The pilot message save_event attribute is set to true by the autopilot driver to indicate the image needs recording.
        pil, veh, inf, image_md, image = self._states(_time)
        _pil_ok = pil is not None and pil.get('driver') is not None
        _pil_ok = _pil_ok and (bool(pil.get('forced_steering', 0)) or bool(pil.get('forced_throttle', 0)))
        if force or _pil_ok:
            _save_img = pil is not None and bool(pil.get('save_event', 0))
            _img_fields = self._image_fields(image, persist=(force or _save_img))
            self._database.events.insert_one({
                'time': _time,
                'trigger': trigger,
                'pil_time': _get_ts(pil),
                'pil_cruise_speed': _str(pil, 'cruise_speed'),
                'pil_desired_speed': _str(pil, 'desired_speed'),
                'pil_driver': _driver_type(pil),
                'pil_is_steering_intervention': _bool(pil, 'forced_steering'),
                'pil_is_throttle_intervention': _bool(pil, 'forced_throttle'),
                'pil_is_save_event': _bool(pil, 'save_event'),
                'pil_steering': _str(pil, 'steering'),
                'pil_throttle': _str(pil, 'throttle'),
                'veh_time': _get_ts(veh),
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
                'img_buffer': _img_fields[2]
            })

    def setup(self):
        self._insert(TRIGGER_SERVICE_START, force=True)

    def finish(self):
        self._insert(TRIGGER_SERVICE_END, force=True)

    def step(self):
        self._insert(TRIGGER_SERVICE_STEP)


def main():
    parser = argparse.ArgumentParser(description='Logger black box service.')
    parser.add_argument('--name', type=str, default='none', help='Process name.')
    parser.add_argument('--config', type=str, default='/config', help='Config directory path.')
    args = parser.parse_args()

    # The mongo client is thread-safe and provides for transparent connection pooling.
    _mongo = MongoClient()

    application = LogApplication(_mongo, quit_event, hz=20, config_dir=args.config)

    pilot = json_collector(url='ipc:///byodr/pilot.sock', topic=b'aav/pilot/output', event=quit_event)
    vehicle = json_collector(url='ipc:///byodr/vehicle.sock', topic=b'aav/vehicle/state', event=quit_event)
    inference = json_collector(url='ipc:///byodr/inference.sock', topic=b'aav/inference/state', event=quit_event)

    application.camera = CameraThread(url='ipc:///byodr/camera_0.sock', topic=b'aav/camera/0', event=quit_event)
    application.pilot = lambda: pilot.get()
    application.vehicle = lambda: vehicle.get()
    application.inference = lambda: inference.get()

    threads = [application.camera, pilot, vehicle, inference, threading.Thread(target=application.run)]
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
