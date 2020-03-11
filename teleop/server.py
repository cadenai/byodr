#!/usr/bin/env python

import collections
import json
import logging
import threading
import time
import traceback
from struct import Struct

import cv2
import ffmpeg
import numpy as np
from tornado import websocket

from byodr.utils import timestamp

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
        msg['time'] = timestamp()
        self._fn_control(msg)
        self.write_message('{}')


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


class FFMPegThread(threading.Thread):
    def __init__(self, fn_capture, event):
        super(FFMPegThread, self).__init__()
        self._capture = fn_capture
        self._quit_event = event
        self.consumers = []

    def add_consumer(self, _fn):
        self.consumers.append(_fn)

    def remove_consumer(self, _fn):
        self.consumers.remove(_fn)

    def run(self):
        # Determine the image shape first.
        _shape = None
        while not self._quit_event.is_set() and _shape is None:
            img = self._capture()
            if img is not None:
                _shape = img.shape
            else:
                time.sleep(.100)
        height, width = _shape[:2]
        process = ffmpeg.input('pipe:', format='rawvideo', pix_fmt='bgr24', s='{}x{}'.format(width, height))
        process = process.output('pipe:', format='mpegts', vcodec='mpeg1video', s='480x320',
                                 video_bitrate='1024', tune='zerolatency', fflags='nobuffer')
        while not self._quit_event.is_set():
            try:
                img = self._capture()
                if img is not None and self.consumers:
                    frame, err = process.run(capture_stdout=True, capture_stderr=True, input=img.astype(np.uint8).tobytes())
                    [c(frame) for c in self.consumers]
            except StandardError as e:
                logger.error("Trace: {}".format(traceback.format_exc(e)))
                time.sleep(1)
        process.stdin.close()
        process.stdout.close()
        process.wait()


class MpegServerSocket(websocket.WebSocketHandler):
    # noinspection PyAttributeOutsideInit
    def initialize(self, **kwargs):
        self.shape = (320, 480, 3)
        self._registry = kwargs.get('registry')

    def _frame(self, b):
        try:
            self.write_message(b, binary=True)
        except websocket.WebSocketClosedError:
            pass

    def check_origin(self, origin):
        return True

    def data_received(self, chunk):
        pass

    def open(self, *args, **kwargs):
        logger.info("Mpeg client connected.")
        _header = Struct('>4sHH')
        _magic = b'jsmp'
        _height, _width = self.shape[:2]
        self.write_message(_header.pack(_magic, _width, _height), binary=True)
        self._registry.add_consumer(self._frame)

    def on_close(self):
        self._registry.remove_consumer(self._frame)
        logger.info("Mpeg client disconnected.")

    def on_message(self, message):
        pass
