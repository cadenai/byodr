#!/usr/bin/env python
import collections
import json
import logging
import time
import traceback

import cv2
import numpy as np
from tornado import websocket

logger = logging.getLogger(__name__)


class ControlServerSocket(websocket.WebSocketHandler):
    """ Routes received commands directly to their respective topics. """

    # noinspection PyAttributeOutsideInit
    def initialize(self, **kwargs):
        self._fn_control = kwargs.get('fn_control')

    def check_origin(self, origin):
        return True

    def data_received(self, chunk):
        pass

    def open(self, *args, **kwargs):
        logger.info("Client connected.")

    def on_close(self):
        logger.info("Client disconnected.")

    def on_message(self, json_message):
        msg = json.loads(json_message)
        msg['time'] = time.time()
        self._fn_control(msg)


class MessageServerSocket(websocket.WebSocketHandler):

    # noinspection PyAttributeOutsideInit
    def initialize(self, **kwargs):
        self._fn_state = kwargs.get('fn_state')

    def check_origin(self, origin):
        return True

    def data_received(self, chunk):
        pass

    def open(self, *args, **kwargs):
        logger.info("Log client connected.")

    def on_close(self):
        logger.info("Log client disconnected.")

    def on_message(self, *args):
        try:
            state = self._fn_state()
            pilot = None if state is None else state[0]
            inference = None if state is None else state[1]
            vehicle = None
            response = {
                'ctl': 0,
                'debug1': 0 if inference is None else inference.get('corridor'),
                'debug2': 0 if inference is None else inference.get('obstacle'),
                'debug3': 0 if inference is None else inference.get('penalty'),
                'debug4': 0 if inference is None else inference.get('surprise'),
                'debug5': 0 if inference is None else inference.get('critic'),
                'debug6': 0 if inference is None else inference.get('brake'),
                'debug7': 0 if inference is None else inference.get('entropy'),
                'rec_act': False,
                'rec_mod': None,
                'ste': 0 if pilot is None else pilot.get('steering'),
                'thr': 0 if pilot is None else pilot.get('throttle'),
                'rev': 0,
                'vel_y': 0 if vehicle is None else vehicle.get('velocity'),
                'x': 0 if vehicle is None else vehicle.get('x_coordinate'),
                'y': 0 if vehicle is None else vehicle.get('y_coordinate'),
                'speed': 0.,
                'max_speed': 0.,
                'head': 0 if vehicle is None else vehicle.get('heading'),
                'route': None,
                'route_np': None,
                'route_np_sim': 0.,
                'route_np_debug1': 0.,
                'turn': 'intersection.ahead',
            }
            self.write_message(json.dumps(response))
        except Exception:
            logger.error("MessageServerSocket:on_message:{}".format(traceback.format_exc()))
            raise


def jpeg_encode(image, quality=95):
    """Higher quality leads to slightly more cpu load. Default cv jpeg quality is 95. """
    return cv2.imencode('.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), quality])[1]


class CameraServerSocket(websocket.WebSocketHandler):
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
            if img is not None:
                resolutions = CameraServerSocket._display_resolutions
                if display in resolutions.keys():
                    _width, _height = resolutions[display]
                    if np.prod(img.shape[:2]) > (_width * _height):
                        img = cv2.resize(img, (_width, _height))
                self.write_message(jpeg_encode(img, quality).tobytes(), binary=True)
        except Exception as e:
            logger.error("Camera socket@on_message: {} {}".format(e, traceback.format_exc(e)))
            logger.error("JSON message:---\n{}\n---".format(message))
