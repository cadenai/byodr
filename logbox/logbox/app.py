from __future__ import absolute_import

import argparse
import asyncio
import logging
import multiprocessing
import signal
import sys
import threading
import traceback
from datetime import datetime

from pymongo import MongoClient
from tornado import web, ioloop
from tornado.httpserver import HTTPServer
from tornado.platform.asyncio import AnyThreadEventLoopPolicy

from byodr.utils import Application
from byodr.utils.ipc import CameraThread, json_collector
from byodr.utils.option import parse_option
from .core import *
from .store import Event, create_data_source
from .web import DataTableRequestHandler, JPEGImageRequestHandler

if sys.version_info > (3,):
    pass
else:
    pass

logger = logging.getLogger(__name__)

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


class PackageApplication(Application):
    def __init__(self, mongo, user, event, hz=1e-2, sessions_dir=os.getcwd()):
        super(PackageApplication, self).__init__(quit_event=event, run_hz=hz)
        self._mongo = mongo
        self._user = user
        self._recorder_dir = get_or_create_directory(os.path.join(sessions_dir, 'autopilot'))
        self._photo_dir = get_or_create_directory(os.path.join(sessions_dir, 'photos', 'cam0'))
        self._vehicle = parse_option('constant.vehicle.type', str, 'vehicle.byodr.2020', [], **{})
        self._config = parse_option('constant.vehicle.config', str, 'latest', [], **{})

    def setup(self):
        pass

    def finish(self):
        pass

    def _mark(self, item):
        self._mongo.update_event({'_id': item.get('_id')}, {'$set': {"lb_is_packaged": 1}})

    def _write_out_photos(self):
        items = self._mongo.list_all_non_packaged_photo_events()
        if len(items) > 0 and not self._user.is_busy():
            _directory = os.path.join(self._photo_dir, datetime.fromtimestamp(items[0].get('time') * 1e-6).strftime('%Y%B'))
            _directory = get_or_create_directory(_directory)
            with open(os.path.join(_directory, 'photo.log'), 'a+') as f:
                for item in items:
                    self._mark(item)
                    _timestamp = item.get('time')
                    _dts = datetime.fromtimestamp(_timestamp * 1e-6).strftime('%Y%b%dT%H%M%S')
                    latitude = item.get('veh_gps_latitude')
                    longitude = item.get('veh_gps_longitude')
                    fname = "{}_lat{}_long{}.jpg".format(_dts, str(latitude)[:8].replace('.', '_'), str(longitude)[:8].replace('.', '_'))
                    cv2.imwrite(os.path.join(_directory, fname), cv2_image_from_bytes(item.get('img_buffer')))
                    f.write("{} {} latitude {} longitude {}\r\n".format(_timestamp, fname, latitude, longitude))

    def _event(self, row, lenience_ms=30):
        _timestamp = row.get('time')
        event = Event(
            timestamp=_timestamp,
            image_shape=row.get('img_shape'),
            jpeg_buffer=row.get('img_buffer'),
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
        lenience = lenience_ms * 1e3
        pil_valid = abs(_timestamp - row.get('pil_time')) < lenience
        veh_valid = abs(_timestamp - row.get('veh_time')) < lenience
        inf_valid = abs(_timestamp - row.get('inf_time')) < lenience
        img_valid = abs(_timestamp - row.get('img_time')) < lenience
        event.valid = pil_valid and veh_valid and inf_valid and img_valid
        return event

    def _package_next(self):
        if self._user.is_busy():
            return False
        # Start by saving the photo snapshots.
        self._write_out_photos()
        # Proceed unless new user activity.
        if self._user.is_busy():
            return False
        # The save events not previously processed are packaged together in a zip.
        items = self._mongo.list_next_batch_of_non_packaged_save_events()
        if len(items) < 1:
            return False
        # Mark regardless of zip write success.
        list([self._mark(x) for x in items])
        # Do the zip write in time ascending order - filter out invalid saves.
        events = list(filter(lambda x: x.valid, map(self._event, reversed(items))))
        if len(events) < 1:
            return False
        logger.info("Packaging {} valid events out of {} total items.".format(len(events), len(items)))
        _archive = create_data_source(events[0].timestamp, self._recorder_dir)
        try:
            _archive.open()
            assert _archive.is_open(), "Could not create a new archive."
            list(map(lambda x: _archive.create_event(x), events))
            _archive.close()
        except Exception as e:
            logger.warning(e)
            logger.error("Packager#next: {}".format(traceback.format_exc()))
            return False
        return not self._user.is_busy()

    def step(self):
        _package = True
        while _package:
            _package = self._package_next()


class PhotoLog(object):
    """Ignore photos taken too fast."""

    def __init__(self, buffer_size=100, window_ms=500):
        self._micro = window_ms * 1e3
        self._events = collections.deque(maxlen=buffer_size)
        self._observed = None

    def clear(self):
        self._events.clear()

    def append(self, content):
        _timestamp = content[0]
        # Skip duplicate photos.
        if self._observed is None or (_timestamp - self._observed) > self._micro:
            self._observed = _timestamp
            self._events.append(content)

    def pop_all(self):
        _l = list(self._events) if len(self._events) > 0 else []
        self.clear()
        return _l


class LogApplication(Application):
    def __init__(self, mongo, user, state, event, config_dir=os.getcwd()):
        super(LogApplication, self).__init__(quit_event=event, run_hz=state.get_hz())
        self._mongo = mongo
        self._user = user
        self._state = state
        self._config_dir = config_dir
        self._photos = PhotoLog()

    def _insert(self, trigger, content, save_image=False):
        _time, pil, veh, inf, image_md, image = content
        _img_fields = prepare_image_persist(image, persist=save_image)
        self._mongo.insert_event({
            'time': _time,
            'trigger': trigger,
            'pil_time': get_timestamp(pil),
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
            'veh_time': get_timestamp(veh),
            'veh_heading': _str(veh, 'heading'),
            'veh_gps_latitude': _str(veh, 'latitude_geo'),
            'veh_gps_longitude': _str(veh, 'longitude_geo'),
            'veh_velocity': _str(veh, 'velocity'),
            'veh_is_velocity_trusted': _bool(veh, 'trust_velocity'),
            'inf_time': get_timestamp(inf),
            'inf_steer_action': _str(inf, 'action'),
            'inf_obstruction': _str(inf, 'obstacle'),
            'inf_steer_penalty': _str(inf, 'steer_penalty'),
            'inf_obstruction_penalty': _str(inf, 'brake_penalty'),
            'inf_running_penalty': _str(inf, 'total_penalty'),
            'img_time': get_timestamp(image_md),
            'img_shape': _img_fields[0],
            'img_num_bytes': _img_fields[1],
            'img_buffer': _img_fields[2],
            'lb_is_packaged': 0
        })

    def setup(self):
        self._insert(TRIGGER_SERVICE_START, content=self._state.pull()[:-1])

    def finish(self):
        pass

    def step(self):
        # The pilot message is nil when no party is driving.
        _time, pil, veh, inf, image_md, image, pilot_all = self._state.pull()
        # The pilot message save_event attribute is set to true by the autopilot driver to indicate the image needs recording.
        _drive = pil is not None and pil.get('driver') is not None
        _drive = _drive and (bool(pil.get('forced_steering', 0)) or bool(pil.get('forced_throttle', 0)))
        _train = pil is not None and bool(pil.get('save_event', 0))
        _contents = (_time, pil, veh, inf, image_md, image)
        # Log events.
        if _drive or _train:
            self._user.touch()
            self._insert(TRIGGER_SERVICE_STEP, content=_contents, save_image=_train)
        # Scan the pilot commands for photo requests.
        if any([cmd.get('button_right', 0) == 1 for cmd in ([] if pilot_all is None else pilot_all)]):
            self._photos.append(_contents)
        list(map(lambda x: self._insert(TRIGGER_PHOTO_SNAPSHOT, content=x, save_image=True), self._photos.pop_all()))


def main():
    parser = argparse.ArgumentParser(description='Logger black box service.')
    parser.add_argument('--name', type=str, default='none', help='Process name.')
    parser.add_argument('--sessions', type=str, default='/sessions', help='Sessions directory.')
    parser.add_argument('--config', type=str, default='/config', help='Config directory path.')
    args = parser.parse_args()

    # The mongo client is thread-safe and provides for transparent connection pooling.
    _mongo = MongoLogBox(MongoClient())

    # Never miss out on pilot commands - pop a large buffer.
    _camera = CameraThread(url='ipc:///byodr/camera_0.sock', topic=b'aav/camera/0', event=quit_event)
    pilot = json_collector(url='ipc:///byodr/pilot.sock', topic=b'aav/pilot/output', event=quit_event, pop=True, hwm=20)
    vehicle = json_collector(url='ipc:///byodr/vehicle.sock', topic=b'aav/vehicle/state', event=quit_event)
    inference = json_collector(url='ipc:///byodr/inference.sock', topic=b'aav/inference/state', event=quit_event)

    _user = SharedUser()
    _state = SharedState(channels=(_camera, (lambda: pilot.get()), (lambda: vehicle.get()), (lambda: inference.get())), hz=20)

    log_application = LogApplication(_mongo, _user, _state, quit_event, config_dir=args.config)
    package_application = PackageApplication(_mongo, _user, quit_event, hz=0.0333, sessions_dir=args.sessions)

    application_thread = threading.Thread(target=log_application.run)
    package_thread = threading.Thread(target=package_application.run)
    threads = [_camera, pilot, vehicle, inference, application_thread, package_thread]
    if quit_event.is_set():
        return 0

    [t.start() for t in threads]

    asyncio.set_event_loop_policy(AnyThreadEventLoopPolicy())
    asyncio.set_event_loop(asyncio.new_event_loop())

    try:
        io_loop = ioloop.IOLoop.instance()
        main_app = web.Application([
            (r"/api/datalog/event/v10/table", DataTableRequestHandler, dict(mongo_box=_mongo)),
            (r"/api/datalog/event/v10/image", JPEGImageRequestHandler, dict(mongo_box=_mongo))
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
