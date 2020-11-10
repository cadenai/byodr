#!/usr/bin/env python

import collections
import json
import logging
import os
import time
import traceback
from ConfigParser import SafeConfigParser

import cv2
import numpy as np
import tornado
from tornado import web, websocket

from byodr.utils import timestamp

logger = logging.getLogger(__name__)


class ControlServerSocket(websocket.WebSocketHandler):
    # There can be only one operator in control at any time.
    # Do not use a single variable since class attributes mutate to be instance attributes.
    operators = set()
    operator_access_control = collections.deque(maxlen=1)
    viewers = set()

    # noinspection PyAttributeOutsideInit
    def initialize(self, **kwargs):
        self._fn_control = kwargs.get('fn_control')
        self._operator_timeout_micro = 10 * 1e6  # 10 seconds.
        self._operator_throttle_micro = 10 * 1e3  # 10 milliseconds.

    def check_origin(self, origin):
        return True

    def data_received(self, chunk):
        pass

    def open(self, *args, **kwargs):
        # Perhaps the operator has timed-out but the control connection was not closed somehow.
        if self.operators:
            _last_msg_micro = self.operator_access_control[-1] if self.operator_access_control else 0
            if (timestamp() - _last_msg_micro > self._operator_timeout_micro) or next(iter(self.operators)).ws_connection is None:
                logger.info("Operator server-side timeout.")
                self.operators.clear()
        # Proceed.
        if self.operators:
            self.viewers.add(self)
            logger.info("Viewer {} connected.".format(self.request.remote_ip))
        else:
            self.operators.add(self)
            logger.info("Operator {} connected.".format(self.request.remote_ip))

    def on_close(self):
        if self in self.operators:
            self.operators.clear()
            logger.info("Operator {} disconnected.".format(self.request.remote_ip))
        else:
            try:
                self.viewers.remove(self)
            except KeyError:
                pass
            logger.info("Viewer {} disconnected.".format(self.request.remote_ip))

    def on_message(self, json_message):
        _response = json.dumps(dict(control='viewer'))
        if self in self.operators:
            _micro_time = timestamp()
            # Throttle very fast operator connections.
            _last_msg_micro = self.operator_access_control[-1] if self.operator_access_control else 0
            time.sleep(max(0, self._operator_throttle_micro - (_micro_time - _last_msg_micro)) * 1e-6)
            msg = json.loads(json_message)
            msg['time'] = _micro_time
            self.operator_access_control.append(_micro_time)
            self._fn_control(msg)
            _response = json.dumps(dict(control='operator'))
        try:
            self.write_message(_response)
        except websocket.WebSocketClosedError:
            pass


class MessageServerSocket(websocket.WebSocketHandler):
    # noinspection PyAttributeOutsideInit
    def initialize(self, **kwargs):
        self._fn_state = kwargs.get('fn_state')
        _fn_speed_scale = kwargs.get('fn_speed_scale')
        self._speed_scale = _fn_speed_scale()

    def check_origin(self, origin):
        return True

    def data_received(self, chunk):
        pass

    def open(self, *args, **kwargs):
        pass

    def on_close(self):
        pass

    @staticmethod
    def _translate_driver(pilot, inference):
        if None in (pilot, inference):
            return 0
        ctl = pilot.get('driver')
        if ctl is None:
            return 0
        elif ctl == 'driver_mode.teleop.direct':
            return 2
        elif ctl == 'driver_mode.teleop.cruise':
            return 3
        elif ctl == 'driver_mode.inference.dnn':
            return 7 if inference.get('dagger', 0) == 1 else 5

    @staticmethod
    def _translate_recorder(recorder):
        if recorder is not None:
            if recorder.get('mode') == 'record.mode.driving':
                return 551
            elif recorder.get('mode') == 'record.mode.interventions':
                return 594
        return -999

    def on_message(self, *args):
        try:
            state = self._fn_state()
            pilot = None if state is None else state[0]
            vehicle = None if state is None else state[1]
            inference = None if state is None else state[2]
            recorder = None if state is None else state[3]
            _speed_scale = self._speed_scale
            response = {
                'ctl': self._translate_driver(pilot, inference),
                'debug1': 0 if inference is None else inference.get('corridor'),
                'debug2': 0 if inference is None else inference.get('obstacle'),
                'debug3': 0 if inference is None else inference.get('penalty'),
                'debug4': 0 if inference is None else inference.get('surprise_out'),
                'debug5': 0 if inference is None else inference.get('critic_out'),
                'debug6': 0 if inference is None else inference.get('fallback'),
                'debug7': 0 if inference is None else inference.get('_fps'),
                'rec_act': False if recorder is None else recorder.get('active'),
                'rec_mod': self._translate_recorder(recorder),
                'ste': 0 if pilot is None else pilot.get('steering'),
                'thr': 0 if pilot is None else pilot.get('throttle'),
                'rev': 0,
                'vel_y': 0 if vehicle is None else vehicle.get('velocity') * _speed_scale,
                'x': 0 if vehicle is None else vehicle.get('x_coordinate'),
                'y': 0 if vehicle is None else vehicle.get('y_coordinate'),
                'speed': 0 if pilot is None else pilot.get('desired_speed') * _speed_scale,
                'max_speed': 0 if pilot is None else pilot.get('cruise_speed') * _speed_scale,
                'head': 0 if vehicle is None else vehicle.get('heading'),
                'nav_image': 'none' if inference is None else inference.get('navigation_image'),
                'nav_point': 'none' if pilot is None else pilot.get('navigation_point'),
                'nav_distance': 1 if inference is None else inference.get('navigation_distance'),
                'turn': None if pilot is None else pilot.get('instruction')
            }
            self.write_message(json.dumps(response))
        except Exception:
            logger.error("MessageServerSocket:on_message:{}".format(traceback.format_exc()))
            raise


def jpeg_encode(image, quality=95):
    """Higher quality leads to slightly more cpu load. Default cv jpeg quality is 95. """
    return cv2.imencode('.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), quality])[1]


class CameraMJPegSocket(websocket.WebSocketHandler):
    _display_resolutions = collections.OrderedDict()
    _display_resolutions['CGA'] = (320, 200)
    _display_resolutions['QVGA'] = (320, 240)
    _display_resolutions['HVGA'] = (480, 320)
    _display_resolutions['VGA'] = (640, 480)
    _display_resolutions['SVGA'] = (800, 600)
    _display_resolutions['XGA'] = (1024, 768)

    # noinspection PyAttributeOutsideInit
    def initialize(self, **kwargs):
        self._capture_front = kwargs.get('capture_front')
        self._capture_rear = kwargs.get('capture_rear')
        self._black_img = np.zeros(shape=(1, 1, 3), dtype=np.uint8)

    def check_origin(self, origin):
        return True

    def data_received(self, chunk):
        pass

    def open(self, *args, **kwargs):
        pass

    def on_close(self):
        pass

    def on_message(self, message):
        try:
            request = json.loads(message)
            quality = request.get('quality', 90)
            camera = request.get('camera', 'front').strip().lower()
            display = request.get('display', 'HVGA').strip().upper()
            img = self._capture_front() if camera == 'front' else self._capture_rear()
            if img is None:
                # Always send something to the client is able to resume polling.
                img = self._black_img
            resolutions = CameraMJPegSocket._display_resolutions
            if display in resolutions.keys():
                _width, _height = resolutions[display]
                if np.prod(img.shape[:2]) > (_width * _height):
                    img = cv2.resize(img, (_width, _height))
            self.write_message(jpeg_encode(img, quality).tobytes(), binary=True)
        except Exception as e:
            logger.error("Camera socket@on_message: {} {}".format(e, traceback.format_exc(e)))
            logger.error("JSON message:---\n{}\n---".format(message))


class NavImageHandler(web.RequestHandler):

    # noinspection PyAttributeOutsideInit
    def initialize(self, **kwargs):
        self._fn_get_image = kwargs.get('fn_get_image')
        self._black_img = np.zeros(shape=(1, 1, 3), dtype=np.uint8)
        self._jpeg_quality = 90

    def data_received(self, chunk):
        pass

    # noinspection PyUnresolvedReferences
    @tornado.web.asynchronous
    @tornado.gen.coroutine
    def get(self):
        image = self._fn_get_image()
        image = self._black_img if image is None else image
        chunk = jpeg_encode(image, quality=self._jpeg_quality)
        self.set_header('Content-Type', 'image/jpeg')
        self.set_header('Content-Length', len(chunk))
        self.write(chunk.tobytes())
        yield tornado.gen.Task(self.flush)


class UserOptions(object):
    def __init__(self, fname):
        self._fname = fname
        self._parser = SafeConfigParser()
        self.reload()

    def list_sections(self):
        return self._parser.sections()

    def get_options(self, section):
        return dict(self._parser.items(section))

    def get_option(self, section, name):
        return self._parser.get(section, name)

    def set_option(self, section, name, value):
        self._parser.set(section, name, value)

    def reload(self):
        if os.path.exists(self._fname):
            self._parser.read(self._fname)

    def save(self):
        # The options could have been updated on disk.
        parser = SafeConfigParser()
        parser.read(self._fname)
        for section in parser.sections():
            for option in parser.options(section):
                if not self._parser.has_option(section, option):
                    self.set_option(section, option, parser.get(section, option))
        with open(self._fname, 'wb') as f:
            self._parser.write(f)


class JSONRequestHandler(web.RequestHandler):
    def set_default_headers(self):
        self.set_header('Content-Type', 'application/json')

    def data_received(self, chunk):
        pass


class ApiUserOptionsHandler(JSONRequestHandler):
    # noinspection PyAttributeOutsideInit
    def initialize(self, **kwargs):
        self._options = kwargs.get('user_options')
        self._fn_on_save = kwargs.get('fn_on_save')

    def get(self):
        self._options.reload()
        self.write(json.dumps({s: self._options.get_options(s) for s in (self._options.list_sections())}))

    def post(self):
        data = json.loads(self.request.body)
        for section in data.keys():
            for key, value in data.get(section):
                self._options.set_option(section, key, value)
        if data:
            self._options.save()
            self._options.reload()
            self._fn_on_save()
        self.write(json.dumps(dict(message='ok')))


class JSONMethodDumpRequestHandler(JSONRequestHandler):
    # noinspection PyAttributeOutsideInit
    def initialize(self, **kwargs):
        self._method = kwargs.get('fn_method')

    def get(self):
        self.write(json.dumps(self._method()))
