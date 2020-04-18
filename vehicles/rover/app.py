import Queue
import argparse
import glob
import logging
import math
import multiprocessing
import os
import signal
import sys
import threading
import time
import traceback
from ConfigParser import SafeConfigParser
from functools import partial

import cv2
import numpy as np
import requests
import rospy
from geometry_msgs.msg import Twist, TwistStamped
from requests.auth import HTTPDigestAuth

from byodr.utils import timestamp
from byodr.utils.ipc import ReceiverThread, JSONPublisher, ImagePublisher
from video import GstRawSource

logger = logging.getLogger(__name__)
log_format = '%(levelname)s: %(filename)s %(funcName)s %(message)s'

quit_event = multiprocessing.Event()

CH_NONE, CH_THROTTLE, CH_STEERING, CH_BOTH = (0, 1, 2, 3)
CTL_LAST = 0

# Safety - cap the range that is availble through user configuration.
VEHICLE_THROTTLE_SCALE = 0.133

signal.signal(signal.SIGINT, lambda sig, frame: _interrupt())
signal.signal(signal.SIGTERM, lambda sig, frame: _interrupt())


def _interrupt():
    logger.info("Received interrupt, quitting.")
    quit_event.set()


class RosGate(object):
    """
         rostopic hz /roy_teleop/sensor/odometer
         subscribed to [/roy_teleop/sensor/odometer]
         average rate: 81.741
    """

    def __init__(self, connect=True, wheel_radius_m=0.10, sensor_ticks_per_wheel_rotation=8):

        self._circum_m = 2 * math.pi * wheel_radius_m  # Convert to circumference.
        self._gear_ratio = sensor_ticks_per_wheel_rotation
        self._rps = 0  # Hall sensor revolutions per second.

        if connect:
            rospy.Subscriber("roy_teleop/sensor/odometer", TwistStamped, self._update_odometer)
            self._pub = rospy.Publisher('roy_teleop/command/drive', Twist, queue_size=1)

    def _update_odometer(self, message):
        # The odometer publishes revolutions per second.
        self._rps = float(message.twist.linear.y)

    def publish(self, channel=CH_NONE, throttle=0., steering=0., control=CTL_LAST, button=0):
        if not quit_event.is_set():
            # The button state does not need to go to ros but currently there is no separate state holder (server side)
            # for the button state per timestamp.
            twist = Twist()
            twist.angular.x, twist.angular.z, twist.linear.x, twist.linear.z = (channel, steering, control, throttle)
            twist.linear.y = button
            self._pub.publish(twist)

    def get_odometer_value(self):
        # Convert to travel speed in meters per second.
        return (self._rps / self._gear_ratio) * self._circum_m


class FakeGate(RosGate):
    def __init__(self):
        super(FakeGate, self).__init__(connect=False)

    def publish(self, channel=CH_NONE, throttle=0., steering=0., control=CTL_LAST, button=0):
        pass


class TwistHandler(object):
    def __init__(self, ros_gate, **kwargs):
        super(TwistHandler, self).__init__()
        self._gate = ros_gate
        self._steer_calibration_shift = None
        self._throttle_calibration_shift = None
        cfg = kwargs
        try:
            _steer_shift = float(cfg.get('calibrate.steer.shift'))
            _throttle_shift = float(cfg.get('calibrate.throttle.shift'))
            self._steer_calibration_shift = _steer_shift
            self._throttle_calibration_shift = _throttle_shift
            self._throttle_forward_scale = float(cfg.get('throttle.forward.scale'))
            self._throttle_forward_shift = float(cfg.get('throttle.forward.shift'))
            self._throttle_backward_shift = float(cfg.get('throttle.backward.shift'))
            self._throttle_backward_scale = float(cfg.get('throttle.backward.scale'))
            logger.info("Calibration steer, throttle is {:2.2f}, {:2.2f}.".format(_steer_shift, _throttle_shift))
        except TypeError as e:
            logger.error(traceback.format_exc(e))
            raise e

    def _scale(self, user_throttle, user_steering):
        # Separate capping the throttle range from calibration.
        _throttle = 0
        if user_throttle > 0:
            _throttle = user_throttle + self._throttle_forward_shift + (user_throttle * self._throttle_forward_scale)
        elif user_throttle < 0:
            _throttle = user_throttle + self._throttle_backward_shift + (user_throttle * self._throttle_backward_scale)
        servo_throttle = int(max(-1, min(1, (_throttle + self._throttle_calibration_shift) * VEHICLE_THROTTLE_SCALE)) * 90) + 90
        servo_steering = int(max(-1, min(1, user_steering + self._steer_calibration_shift)) * 90) + 90
        return servo_throttle, servo_steering

    def _drive(self, steering, throttle):
        try:
            throttle, steering = self._scale(throttle, steering)
            self._gate.publish(steering=steering, throttle=throttle)
        except Exception as e:
            logger.error("{}".format(e))

    def state(self):
        x, y = 0, 0
        return dict(x_coordinate=x,
                    y_coordinate=y,
                    heading=0,
                    velocity=self._gate.get_odometer_value(),
                    time=timestamp())

    def noop(self):
        self._drive(steering=0, throttle=0)

    def drive(self, cmd):
        if cmd is not None:
            self._drive(steering=cmd.get('steering'), throttle=cmd.get('throttle'))


class CameraPtzThread(threading.Thread):
    def __init__(self, event, server, user, password,
                 protocol='http', path='/ISAPI/PTZCtrl/channels/1',
                 preset_duration_sec=5.0, scale=100, speed=1., flip=(1, 1)):
        super(CameraPtzThread, self).__init__()
        self._quit_event = event
        self._preset_duration = preset_duration_sec
        self._scale = scale
        self._speed = speed
        self._flip = flip
        self._auth = HTTPDigestAuth(user, password)
        port = 80 if protocol == 'http' else 443
        self._url = '{protocol}://{server}:{port}{path}'.format(**dict(protocol=protocol, server=server, port=port, path=path))
        self._ptz_xml = """
        <PTZData version='2.0' xmlns='http://www.isapi.org/ver20/XMLSchema'>
            <pan>{pan}</pan>
            <tilt>{tilt}</tilt>
            <zoom>0</zoom>
        </PTZData>
        """
        self._queue = Queue.Queue(maxsize=1)
        self._previous = (0, 0)

    def _norm(self, value):
        return max(-self._scale, min(self._scale, int(self._scale * value * self._speed)))

    def _perform(self, operation):
        # Goto a preset position takes time.
        prev = self._previous
        if type(prev) == tuple and prev[0] == 'goto_home' and time.time() - prev[1] < self._preset_duration:
            pass
        elif operation != prev:
            if operation == 'set_home':
                logger.info("Saving ptz home position.")
                self._previous = operation
                r = requests.put(self._url + '/homeposition', auth=self._auth)
            elif operation == 'goto_home':
                logger.info("Activating ptz home position.")
                self._previous = ('goto_home', time.time())
                r = requests.put(self._url + '/homeposition/goto', auth=self._auth)
            else:
                pan, tilt = operation
                self._previous = operation
                r = requests.put(self._url + '/continuous', data=self._ptz_xml.format(**dict(pan=pan, tilt=tilt)), auth=self._auth)
            if r.status_code != 200:
                logger.warn("Got status {} on operation {}.".format(r.status_code, operation))

    def add(self, pilot, teleop):
        try:
            if pilot and pilot.get('driver') == 'driver_mode.teleop.direct' and teleop:
                self._queue.put_nowait(teleop)
        except Queue.Full:
            pass

    def run(self):
        while not self._quit_event.is_set():
            try:
                cmd = self._queue.get(block=True, timeout=0.050)
                operation = (0, 0)
                if all([cmd.get(k, 0) for k in ('button_left', 'button_right', 'button_b')]):
                    operation = 'set_home'
                elif any([cmd.get(k, 0) for k in ('button_y', 'button_center', 'button_a', 'button_x')]):
                    operation = 'goto_home'
                elif 'pan' in cmd and 'tilt' in cmd:
                    operation = (self._norm(cmd.get('pan')) * self._flip[0], self._norm(cmd.get('tilt')) * self._flip[1])
                self._perform(operation)
            except Queue.Empty:
                pass


def _gate_init(cfg):
    dry_run = bool(int(cfg.get('dry.run')))
    if dry_run:
        return FakeGate()
    else:
        # Ros replaces the root logger - add a new handler after ros initialisation.
        rospy.init_node('rover', disable_signals=False, anonymous=True, log_level=rospy.INFO)
        console_handler = logging.StreamHandler(stream=sys.stdout)
        console_handler.setFormatter(logging.Formatter(log_format))
        logging.getLogger().addHandler(console_handler)
        logging.getLogger().setLevel(logging.INFO)
        rospy.on_shutdown(lambda: quit_event.set())
        _wheel_radius = float(cfg.get('chassis.wheel.radius.meter'))
        _sensor_tick_ratio = int(cfg.get('chassis.hall.ticks.per.rotation'))
        logger.info("Starting ROS gate - wheel radius is {:2.2f}m and sensor tick ratio is {}.".format(_wheel_radius, _sensor_tick_ratio))
        return RosGate(wheel_radius_m=_wheel_radius, sensor_ticks_per_wheel_rotation=_sensor_tick_ratio)


def _gst_init(image_publisher, cfg):
    _camera_ip = str(cfg.get('camera.ip'))
    _camera_user = str(cfg.get('camera.user'))
    _camera_pass = str(cfg.get('camera.password'))
    _camera_rtsp_port = int(cfg.get('camera.rtsp.port'))
    _camera_rtsp_path = str(cfg.get('camera.rtsp.path'))
    _camera_img_wh = str(cfg.get('camera.image.shape'))
    _camera_img_flip = str(cfg.get('camera.image.flip'))
    _camera_shape = [int(x) for x in _camera_img_wh.split('x')]
    _camera_shape = (_camera_shape[1], _camera_shape[0], 3)
    _camera_rtsp_url = 'rtsp://{user}:{password}@{ip}:{port}{path}'.format(
        **dict(user=_camera_user, password=_camera_pass, ip=_camera_ip, port=_camera_rtsp_port, path=_camera_rtsp_path)
    )

    def _image(_b, flipcode=None):
        _img = np.fromstring(_b.extract_dup(0, _b.get_size()), dtype=np.uint8).reshape(_camera_shape)
        image_publisher.publish(cv2.flip(_img, flipcode) if flipcode else _img)

    _url = "rtspsrc " \
           "location={} " \
           "latency=0 drop-on-latency=true ! queue ! " \
           "rtph264depay ! h264parse ! queue ! avdec_h264 ! videoconvert ! " \
           "videoscale ! video/x-raw,format=BGR ! queue".format(_camera_rtsp_url)

    # flipcode = 0: flip vertically
    # flipcode > 0: flip horizontally
    # flipcode < 0: flip vertically and horizontally
    _flipcode = None
    if _camera_img_flip in ('both', 'vertical', 'horizontal'):
        _flipcode = 0 if _camera_img_flip == 'vertical' else 1 if _camera_img_flip == 'horizontal' else -1

    logger.info("Camera rtsp url = {}.".format(_camera_rtsp_url))
    logger.info("Using image flipcode={}".format(_flipcode))
    return GstRawSource(fn_callback=partial(_image, flipcode=_flipcode), command=_url)


def _ptz_init(cfg):
    _camera_ptz_enabled = bool(int(cfg.get('camera.ptz.enabled')))
    if not _camera_ptz_enabled:
        return None
    else:
        _camera_ip = str(cfg.get('camera.ip'))
        _camera_user = str(cfg.get('camera.user'))
        _camera_pass = str(cfg.get('camera.password'))
        _camera_ptz_protocol = str(cfg.get('camera.ptz.protocol'))
        _camera_ptz_path = str(cfg.get('camera.ptz.path'))
        _camera_ptz_flip = str(cfg.get('camera.ptz.flip'))
        _camera_ptz_speed = float(cfg.get('camera.ptz.speed'))
        _flipcode = [1, 1]
        if _camera_ptz_flip in ('pan', 'tilt', 'both'):
            _flipcode[0] = -1 if _camera_ptz_flip in ('pan', 'both') else 1
            _flipcode[1] = -1 if _camera_ptz_flip in ('tilt', 'both') else 1
        logger.info("Create PTZ thread for camera {}.".format(_camera_ip))
        return CameraPtzThread(event=quit_event,
                               server=_camera_ip,
                               user=_camera_user,
                               password=_camera_pass,
                               protocol=_camera_ptz_protocol,
                               path=_camera_ptz_path,
                               speed=_camera_ptz_speed,
                               flip=_flipcode)


def main():
    parser = argparse.ArgumentParser(description='Rover main.')
    parser.add_argument('--config', type=str, default='/config', help='Config directory path.')
    args = parser.parse_args()

    parser = SafeConfigParser()
    [parser.read(_f) for _f in ['config.ini'] + glob.glob(os.path.join(args.config, '*.ini'))]
    cfg = dict(parser.items('vehicle'))
    cfg.update(dict(parser.items('platform')))
    for key in sorted(cfg):
        logger.info("{} = {}".format(key, cfg[key]))

    _process_frequency = int(cfg.get('clock.hz'))
    _patience_micro = float(cfg.get('patience.ms', 200)) * 1000

    state_publisher = JSONPublisher(url='ipc:///byodr/vehicle.sock', topic='aav/vehicle/state')
    image_publisher = ImagePublisher(url='ipc:///byodr/camera.sock', topic='aav/camera/0')

    gst_source = _gst_init(image_publisher, cfg)
    gst_source.open()

    vehicle = TwistHandler(ros_gate=(_gate_init(cfg)), **cfg)
    camera_ptz = _ptz_init(cfg)

    threads = []
    pilot = ReceiverThread(url='ipc:///byodr/pilot.sock', topic=b'aav/pilot/output', event=quit_event)
    teleop = ReceiverThread(url='ipc:///byodr/teleop.sock', topic=b'aav/teleop/input', event=quit_event)
    threads.append(pilot)
    threads.append(teleop)
    if camera_ptz:
        threads.append(camera_ptz)
    [t.start() for t in threads]

    def _check_gst():
        if gst_source.is_open() and not gst_source.is_healthy(patience=0.50):
            gst_source.close()
        if gst_source.is_closed():
            gst_source.open()

    logger.info("Processing at {} Hz and a patience of {} ms.".format(_process_frequency, _patience_micro / 1000))
    _period = 1. / _process_frequency
    while not quit_event.is_set():
        command = pilot.get_latest()
        _command_time = 0 if command is None else command.get('time')
        _command_age = timestamp() - _command_time
        _on_time = _command_age < _patience_micro
        if _on_time:
            vehicle.drive(command)
        else:
            vehicle.noop()
        state_publisher.publish(vehicle.state())
        _check_gst()
        if camera_ptz:
            camera_ptz.add(command, teleop.get_latest())
        time.sleep(_period)

    logger.info("Waiting on stream source to close.")
    gst_source.close()

    logger.info("Waiting on threads to stop.")
    [t.join() for t in threads]


if __name__ == "__main__":
    logging.basicConfig(format=log_format)
    logging.getLogger().setLevel(logging.INFO)
    main()
