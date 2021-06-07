#!/usr/bin/env python

import collections
import datetime
import json
import logging
import os
import threading
import time
import traceback
from ConfigParser import SafeConfigParser

import cv2
import numpy as np
import tornado
from tornado import web, websocket

from byodr.utils import timestamp
from byodr.utils.navigate import AbstractRouteDataSource

logger = logging.getLogger(__name__)


class ControlServerSocket(websocket.WebSocketHandler):
    # There can be only one operator in control at any time.
    # Do not use a single variable since class attributes mutate to be instance attributes.
    operators = set()
    operator_access_control = collections.deque(maxlen=1)

    def _has_operators(self):
        return len(self.operators) > 0

    def _is_operator(self):
        return self in self.operators

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
        if self._is_operator():
            logger.info("Operator {} reconnected.".format(self.request.remote_ip))
        else:
            took_over = self._has_operators()
            self.operators.clear()
            self.operators.add(self)
            logger.info("Operator {} {}.".format(self.request.remote_ip, ('took over' if took_over else 'connected')))

    def on_close(self):
        if self._is_operator():
            self.operators.clear()
            logger.info("Operator {} disconnected.".format(self.request.remote_ip))
        else:
            logger.info("Viewer {} disconnected.".format(self.request.remote_ip))

    def on_message(self, json_message):
        _response = json.dumps(dict(control='viewer'))
        if self._is_operator():
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
            speed_scale = 1. if pilot is None else float(pilot.get('speed_scale', 1))
            pilot_navigation_active = 0 if pilot is None else int(pilot.get('navigation_active', False))
            pilot_match_image = -1 if pilot is None else pilot.get('navigation_match_image', -1)
            pilot_match_sim = 1 if pilot is None else pilot.get('navigation_match_distance', 1)
            pilot_match_point = '' if pilot is None else pilot.get('navigation_match_point', '')
            inference_current_image = -1 if inference is None else inference.get('navigation_image', -1)
            inference_current_sim = 1 if inference is None else inference.get('navigation_distance', 1)
            inference_command = 0 if inference is None else inference.get('navigation_command', 0)
            response = {
                'ctl': self._translate_driver(pilot, inference),
                'debug1': 0 if inference is None else inference.get('brake_critic_out'),
                'debug2': 0 if inference is None else inference.get('obstacle'),
                'debug3': 0 if inference is None else inference.get('penalty'),
                'debug4': 0 if inference is None else inference.get('surprise_out'),
                'debug5': 0 if inference is None else inference.get('critic_out'),
                'debug6': 0,
                'debug7': 0 if inference is None else inference.get('_fps'),
                'rec_act': False if recorder is None else recorder.get('active'),
                'rec_mod': self._translate_recorder(recorder),
                'ste': 0 if pilot is None else pilot.get('steering'),
                'thr': 0 if pilot is None else pilot.get('throttle'),
                'vel_y': 0 if vehicle is None else vehicle.get('velocity') * speed_scale,
                'x': 0 if vehicle is None else vehicle.get('x_coordinate'),
                'y': 0 if vehicle is None else vehicle.get('y_coordinate'),
                'speed': 0 if pilot is None else pilot.get('desired_speed') * speed_scale,
                'max_speed': 0 if pilot is None else pilot.get('cruise_speed') * speed_scale,
                'head': 0 if vehicle is None else vehicle.get('heading'),
                'nav_active': pilot_navigation_active,
                'nav_point': pilot_match_point,
                'nav_image': [pilot_match_image, inference_current_image],
                'nav_distance': [pilot_match_sim, inference_current_sim],
                'nav_command': inference_command,
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
            quality = int(request.get('quality', 90))
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
        self._jpeg_quality = 95

    def data_received(self, chunk):
        pass

    @tornado.web.asynchronous
    @tornado.gen.coroutine
    def get(self):
        try:
            image_id = int(self.get_query_argument('im'))
        except ValueError:
            image_id = -1
        image = self._fn_get_image(image_id)
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
        return [s for s in self._parser.sections() if s != 'inference']

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


class NavigationRequestError(Exception):
    def __init__(self, *args, **kwargs):
        super(NavigationRequestError, self).__init__(args, kwargs)


class IllegalActionNavigationRequestError(Exception):
    def __init__(self, *args, **kwargs):
        super(IllegalActionNavigationRequestError, self).__init__(args, kwargs)


class UnknownRouteNavigationRequestError(Exception):
    def __init__(self, *args, **kwargs):
        super(UnknownRouteNavigationRequestError, self).__init__(args, kwargs)


class UnknownPointNavigationRequestError(Exception):
    def __init__(self, *args, **kwargs):
        super(UnknownPointNavigationRequestError, self).__init__(args, kwargs)


class LocalRouteDataSource(AbstractRouteDataSource):
    def __init__(self, delegate):
        self._delegate = delegate
        self._q_navigation_requests = collections.deque(maxlen=1)

    def get_navigation_request(self):
        try:
            return self._q_navigation_requests.pop()
        except IndexError:
            return None

    def set_navigation_request(self, item):
        self._q_navigation_requests.append(item)

    def __len__(self):
        return len(self._delegate)

    def load_routes(self):
        return self._delegate.load_routes()

    def list_routes(self):
        return self._delegate.list_routes()

    def get_selected_route(self):
        return self._delegate.get_selected_route()

    def open(self, route_name=None):
        self._delegate.open(route_name)

    def delayed_open(self, route_name):
        if self.get_selected_route() != route_name:
            threading.Thread(target=self.open, args=(route_name,)).start()

    def is_open(self):
        return self._delegate.is_open()

    def close(self):
        self._delegate.close()

    def quit(self):
        self._delegate.quit()

    def list_navigation_points(self):
        return self._delegate.list_navigation_points()

    def has_navigation_point(self, route, point):
        return self._delegate.has_navigation_point(route, point)

    def list_all_images(self):
        return self._delegate.list_all_images()

    def get_image(self, image_id):
        return self._delegate.get_image(image_id)

    def get_image_navigation_point(self, idx):
        return self._delegate.get_image_navigation_point(idx)

    def get_image_navigation_point_id(self, idx):
        return self._delegate.get_image_navigation_point_id(idx)

    def get_instructions(self, point):
        return self._delegate.get_instructions(point)


class SimpleRequestNavigationHandler(web.RequestHandler):
    # noinspection PyAttributeOutsideInit
    def initialize(self, **kwargs):
        self._store = kwargs.get('route_store')

    def data_received(self, chunk):
        pass

    @tornado.web.asynchronous
    @tornado.gen.coroutine
    def get(self):
        nav_request = dict(
            action=self.get_query_argument('action', default=None),
            route=self.get_query_argument('route', default=None),
            point=self.get_query_argument('point', default=None),
            speed=self.get_query_argument('speed', default=None)
        )
        logger.info("{}".format(nav_request))
        response_code = 200
        blob = {}
        try:
            #   /navigate?action='halt'
            #   /navigate?action='halt'&route=''&point=''
            #   /navigate?action='resume'&route=''&speed=1
            action = nav_request.get('action', None)
            route = nav_request.get('route', None)
            point = nav_request.get('point', None)
            speed = nav_request.get('speed', None)
            # Only the latest non-executed request is to be processed.
            if action not in ('resume', 'halt'):
                raise IllegalActionNavigationRequestError()
            if route is not None and route not in self._store.list_routes():
                raise UnknownRouteNavigationRequestError()
            if action == 'halt' and route is not None and (point is None or not self._store.has_navigation_point(route, point)):
                raise UnknownPointNavigationRequestError()
            if speed is not None:
                _ = float(speed)
            # We are the authority on route state.
            if route is not None:
                self._store.delayed_open(route)
            self._store.set_navigation_request(nav_request)
            blob['message'] = 'Your request has been successfully completed'
        except IllegalActionNavigationRequestError:
            response_code = 404
            blob['message'] = 'The action is invalid'
        except UnknownRouteNavigationRequestError:
            response_code = 404
            blob['message'] = 'The route is invalid'
        except UnknownPointNavigationRequestError:
            response_code = 404
            blob['message'] = 'The navigation point is invalid'
        except StandardError as se:
            logger.info(se)
            response_code = 404
            blob['message'] = 'An illegal value error occurred'

        blob['status'] = 'ok' if response_code == 200 else 'error'
        blob['timestamp'] = datetime.datetime.utcnow().strftime('%b %d %H:%M:%S.%s UTC')
        message = json.dumps(blob)
        # self.set_header('Content-Type', 'text/plain; charset=UTF-8')
        self.set_header('Content-Type', 'application/json')
        self.set_header('Content-Length', len(message))
        self.set_status(response_code)
        self.write(message)
        yield tornado.gen.Task(self.flush)


class JSONNavigationHandler(JSONRequestHandler):
    # noinspection PyAttributeOutsideInit
    def initialize(self, **kwargs):
        self._store = kwargs.get('route_store')

    def get(self):
        action = self.get_query_argument('action')
        if action == 'list':
            _routes = self._store.list_routes()
            _selected = self._store.get_selected_route()
            _response = {'routes': sorted(_routes), 'selected': _selected}
            self.write(json.dumps(_response))
            threading.Thread(target=self._store.load_routes).start()
        else:
            self.write(json.dumps({}))

    def post(self):
        data = json.loads(self.request.body)
        action = data.get('action')
        selected_route = data.get('route')
        _active = len(self._store) > 0
        if action == 'start' or (action == 'toggle' and (not _active or self._store.get_selected_route() != selected_route)):
            self._store.delayed_open(selected_route)
        elif action in ('close', 'toggle'):
            self._store.close()
        self.write(json.dumps(dict(message='ok')))
