#!/usr/bin/env python
import argparse
import collections
import json
import logging
import multiprocessing
import os
import signal
import threading
import time
import traceback

import cv2
import numpy as np
import zmq
# import rospy
# from std_msgs.msg import String as RosString
from tornado import web, websocket, ioloop

logger = logging.getLogger(__name__)

log_format = '%(levelname)s: %(filename)s %(funcName)s %(message)s'

io_loop = ioloop.IOLoop.instance()
signal.signal(signal.SIGINT, lambda sig, frame: io_loop.add_callback_from_signal(_interrupt))

quit_event = multiprocessing.Event()


def _interrupt():
    logger.info("Received interrupt, quitting.")
    quit_event.set()
    io_loop.stop()


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
        if not quit_event.is_set():
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
            pilot = self._fn_state()
            vehicle = None
            response = {
                'ctl': 0,
                'debug1': 0.,
                'debug2': 0.,
                'debug3': 0.,
                'debug4': 0.,
                'debug5': 0.,
                'debug6': 0.,
                'debug7': 0.,
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


# def _ros_init():
# Ros replaces the root logger - add a new handler after ros initialisation.
# rospy.init_node('teleop', disable_signals=False, anonymous=True, log_level=rospy.INFO)
# console_handler = logging.StreamHandler(stream=sys.stdout)
# console_handler.setFormatter(logging.Formatter(log_format))
# logging.getLogger().addHandler(console_handler)
# logging.getLogger().setLevel(logging.INFO)
# rospy.on_shutdown(lambda: quit_event.set())


def main():
    parser = argparse.ArgumentParser(description='Teleop sockets server.')
    parser.add_argument('--port', type=int, default=9100, help='Port number')
    args = parser.parse_args()

    # context = zmq.Context()
    publisher = zmq.Context().socket(zmq.PUB)
    publisher.bind('ipc:///tmp/byodr/teleop.sock')

    subscriber = zmq.Context().socket(zmq.SUB)
    subscriber.setsockopt(zmq.RCVHWM, 1)
    subscriber.setsockopt(zmq.RCVTIMEO, 20)
    subscriber.setsockopt(zmq.LINGER, 0)
    subscriber.connect('ipc:///tmp/byodr/pilot.sock')
    subscriber.setsockopt(zmq.SUBSCRIBE, b'aav/pilot/output')

    output_state = collections.deque(maxlen=1)

    def _receive():
        while not quit_event.is_set():
            try:
                output_state.appendleft(json.loads(subscriber.recv().split(':', 1)[1]))
            except zmq.Again:
                pass

    ipc_thread = threading.Thread(target=_receive)
    ipc_thread.start()

    def _send(data):
        publisher.send('aav/teleop/input:{}'.format(json.dumps(data)), zmq.NOBLOCK)

    # def _capture():
    #     img = None
    #     try:
    #         [_, md, data] = socket.recv_multipart(flags=zmq.NOBLOCK)
    #         md = json.loads(md)
    #         height, width, channels = md['shape']
    #         img = np.frombuffer(buffer(data), dtype=np.uint8)
    #         img = img.reshape((height, width, channels))
    #     except zmq.Again:
    #         pass
    #     return img
    #
    # publisher.send_multipart(['aav/teleop/input ', json.dumps(x)])))
    #
    #

    try:
        web_app = web.Application([
            (r"/ws/ctl", ControlServerSocket, dict(fn_control=_send)),
            (r"/ws/log", MessageServerSocket, dict(fn_state=(lambda: output_state[0] if bool(output_state) else None))),
            (r"/ws/cam", CameraServerSocket, dict(fn_capture=(lambda: None))),
            (r"/(.*)", web.StaticFileHandler, {
                'path': os.path.join(os.environ.get('TELEOP_HOME'), 'html'),
                'default_filename': 'index.htm'
            })
        ])
        port = args.port
        web_app.listen(port)
        logger.info("Web service starting on port {}.".format(port))
        io_loop.start()
    except KeyboardInterrupt:
        quit_event.set()

    # logger.info("Waiting on zmq to terminate.")
    # context.term()
    ipc_thread.join()


if __name__ == "__main__":
    logging.basicConfig(format=log_format)
    logging.getLogger().setLevel(logging.DEBUG)
    main()
