#!/usr/bin/env python

import collections
import json
import logging
import os
import traceback
from ConfigParser import SafeConfigParser

import cv2
import numpy as np
from tornado import web, websocket

from byodr.utils import timestamp

logger = logging.getLogger(__name__)


class ControlServerSocket(websocket.WebSocketHandler):
    # There can be only one client in control at any time.
    connections = set()

    # noinspection PyAttributeOutsideInit
    def initialize(self, **kwargs):
        self._fn_control = kwargs.get('fn_control')

    def check_origin(self, origin):
        return True

    def data_received(self, chunk):
        pass

    def open(self, *args, **kwargs):
        logger.info("Control client connected.")
        self.connections.add(self)
        if len(self.connections) > 1:
            [c.close() for c in self.connections]

    def on_close(self):
        logger.info("Control client disconnected.")
        self.connections.remove(self)

    def on_message(self, json_message):
        msg = json.loads(json_message)
        msg['time'] = timestamp()
        self._fn_control(msg)
        try:
            self.write_message('{}')
        except websocket.WebSocketClosedError:
            pass


class MessageServerSocket(websocket.WebSocketHandler):
    # noinspection PyAttributeOutsideInit
    def initialize(self, **kwargs):
        self._fn_state = kwargs.get('fn_state')
        self._speed_scale = kwargs.get('speed_scale')

    def check_origin(self, origin):
        return True

    def data_received(self, chunk):
        pass

    def open(self, *args, **kwargs):
        logger.info("Log client connected.")

    def on_close(self):
        logger.info("Log client disconnected.")

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
            response = {
                'ctl': self._translate_driver(pilot, inference),
                'debug1': 0 if inference is None else inference.get('corridor'),
                'debug2': 0 if inference is None else inference.get('obstacle'),
                'debug3': 0 if inference is None else inference.get('penalty'),
                'debug4': 0 if inference is None else inference.get('surprise'),
                'debug5': 0 if inference is None else inference.get('critic'),
                'debug6': 0 if inference is None else inference.get('brake'),
                'debug7': 0 if inference is None else inference.get('entropy'),
                'rec_act': False if recorder is None else recorder.get('active'),
                'rec_mod': self._translate_recorder(recorder),
                'ste': 0 if pilot is None else pilot.get('steering'),
                'thr': 0 if pilot is None else pilot.get('throttle'),
                'rev': 0,
                'vel_y': 0 if vehicle is None else vehicle.get('velocity') * self._speed_scale,
                'x': 0 if vehicle is None else vehicle.get('x_coordinate'),
                'y': 0 if vehicle is None else vehicle.get('y_coordinate'),
                'speed': 0 if pilot is None else pilot.get('desired_speed') * self._speed_scale,
                'max_speed': 0 if pilot is None else pilot.get('cruise_speed') * self._speed_scale,
                'head': 0 if vehicle is None else vehicle.get('heading'),
                'route': None,
                'route_np': None,
                'route_np_sim': 0.,
                'route_np_debug1': 0.,
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
        self._capture = kwargs.get('fn_capture')
        self._black_img = np.zeros(shape=(240, 320, 3), dtype=np.uint8)

    def check_origin(self, origin):
        return True

    def data_received(self, chunk):
        pass

    def open(self, *args, **kwargs):
        logger.info("Camera client connected.")

    def on_close(self):
        logger.info("Camera client disconnected.")

    def on_message(self, message):
        try:
            request = json.loads(message)
            quality = request.get('quality', 90)
            display = request.get('display', 'HVGA').strip().upper()
            img = self._capture()
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
        self.write(json.dumps({s: self._options.get_options(s) for s in (self._options.list_sections())}))

    def post(self):
        data = json.loads(self.request.body)
        for section in data.keys():
            for key, value in data.get(section):
                self._options.set_option(section, key, value)
                logger.info("Save {} {} {}.".format(section, key, value))
        if data:
            self._options.save()
            self._options.reload()
            self._fn_on_save()
        self.write(json.dumps(dict(message='ok')))


class ApiSystemStateHandler(JSONRequestHandler):
    # noinspection PyAttributeOutsideInit
    def initialize(self, **kwargs):
        self._list_start_messages = kwargs.get('fn_list_start_messages')

    def get(self):
        messages = self._list_start_messages()
        self.write(json.dumps(messages))
