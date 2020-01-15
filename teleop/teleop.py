#!/usr/bin/env python
import argparse
import collections
import json
import logging
import multiprocessing
import os
import signal
import sys
import time
import traceback

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as RosImage
from std_msgs.msg import String as RosString
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


def jpeg_encode(image, quality=80):
    """Higher quality leads to slightly more cpu load. """
    return cv2.imencode('.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), quality])[1]


class ControlServerSocket(websocket.WebSocketHandler):
    """ Routes received commands directly to their respective topics. """

    # noinspection PyAttributeOutsideInit
    def initialize(self, **kwargs):
        self._control_topic = kwargs.get('control_topic')
        self._drive_topic = kwargs.get('drive_topic')

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
            cmd = {
                'time': time.time(),
                'steering': msg.get('steering', 0),
                'throttle': msg.get('throttle', 0)
            }
            self._drive_topic.publish(json.dumps(cmd))
            if bool(set(msg.keys()).difference({'steering', 'throttle'})):
                self._control_topic.publish(json_message)


class MessageServerSocket(websocket.WebSocketHandler):

    # noinspection PyAttributeOutsideInit
    def initialize(self, **kwargs):
        self.vehicle_state = kwargs.get('vehicle_state')
        self.pilot_state = kwargs.get('pilot_state')

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
            pilot = self.pilot_state[0] if bool(self.pilot_state) else None
            vehicle = self.vehicle_state[0] if bool(self.vehicle_state) else None
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
        self.camera = kwargs.get('camera')
        self.bridge = kwargs.get('bridge')

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
            data = self.camera[0] if bool(self.camera) else None
            if data is not None:
                img = self.bridge.imgmsg_to_cv2(data, "bgr8")
                resolutions = CameraServerSocket._display_resolutions
                if display in resolutions.keys():
                    _width, _height = resolutions[display]
                    if np.prod(img.shape[:2]) > (_width * _height):
                        img = cv2.resize(img, (_width, _height))
                self.write_message(jpeg_encode(img, quality).tobytes(), binary=True)
        except Exception as e:
            logger.error("Camera socket@on_message: {} {}".format(e, traceback.format_exc(e)))
            logger.error("JSON message:---\n{}\n---".format(message))


def _ros_init():
    # Ros replaces the root logger - add a new handler after ros initialisation.
    rospy.init_node('teleop', disable_signals=False, anonymous=True, log_level=rospy.DEBUG)
    console_handler = logging.StreamHandler(stream=sys.stdout)
    console_handler.setFormatter(logging.Formatter(log_format))
    console_handler.setLevel(logging.DEBUG)
    logging.getLogger().addHandler(console_handler)
    logging.getLogger().setLevel(logging.DEBUG)
    rospy.on_shutdown(lambda: quit_event.set())


def main():
    parser = argparse.ArgumentParser(description='Teleop sockets server.')
    parser.add_argument('--port', type=int, default=9100, help='Port number')
    args = parser.parse_args()
    try:
        _ros_init()
        # Setup topics and subscriptions once - the web application creates a handler instance per request.
        vehicle_state = collections.deque(maxlen=1)
        pilot_state = collections.deque(maxlen=1)
        camera0 = collections.deque(maxlen=1)
        rospy.Subscriber('aav/vehicle/state/blob', RosString, lambda x: vehicle_state.appendleft(json.loads(x.data)), queue_size=1)
        rospy.Subscriber('aav/pilot/command/blob', RosString, lambda x: pilot_state.appendleft(json.loads(x.data)), queue_size=1)
        rospy.Subscriber('aav/vehicle/camera/0', RosImage, lambda x: camera0.appendleft(x), queue_size=1)
        control_topic = rospy.Publisher('aav/teleop/input/control', RosString, queue_size=1)
        drive_topic = rospy.Publisher('aav/teleop/input/drive', RosString, queue_size=1)
        bridge = CvBridge()
        web_app = web.Application([
            (r"/ws/ctl", ControlServerSocket, dict(control_topic=control_topic, drive_topic=drive_topic)),
            (r"/ws/log", MessageServerSocket, dict(vehicle_state=vehicle_state, pilot_state=pilot_state)),
            (r"/ws/cam", CameraServerSocket, dict(camera=camera0, bridge=bridge)),
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
        pass


if __name__ == "__main__":
    logging.basicConfig(format=log_format)
    logging.getLogger().setLevel(logging.DEBUG)
    main()
