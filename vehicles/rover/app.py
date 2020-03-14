import argparse
import logging
import multiprocessing
import signal
import sys
import time
import traceback
from ConfigParser import SafeConfigParser

import numpy as np
import rospy
from geometry_msgs.msg import Twist, TwistStamped

from byodr.utils import timestamp
from byodr.utils.ipc import ReceiverThread, JSONPublisher, ImagePublisher
from video import GstRawSource

logger = logging.getLogger(__name__)
log_format = '%(levelname)s: %(filename)s %(funcName)s %(message)s'

quit_event = multiprocessing.Event()

CAMERA_SHAPE = (360, 640, 3)
CH_NONE, CH_THROTTLE, CH_STEERING, CH_BOTH = (0, 1, 2, 3)
CTL_LAST = 0

signal.signal(signal.SIGINT, lambda sig, frame: _interrupt())
signal.signal(signal.SIGTERM, lambda sig, frame: _interrupt())


def _interrupt():
    logger.info("Received interrupt, quitting.")
    quit_event.set()


class RosGate(object):
    """
    """

    def __init__(self, connect=True):
        """
        """
        self._circum_m = .30  # Meters.
        self._e_time = time.time()
        self._dt = 1e12

        if connect:
            rospy.Subscriber("roy_teleop/sensor/odometer", TwistStamped, self._update_odometer)
            self._pub = rospy.Publisher('roy_teleop/command/drive', Twist, queue_size=1)

    def _update_odometer(self, message):
        # Either zero or one for the hall effect.
        effect = int(message.twist.linear.y)
        if effect == 1:
            self._dt = time.time() - self._e_time
            self._e_time = time.time()

    def publish(self, channel=CH_NONE, throttle=0., steering=0., control=CTL_LAST, button=0):
        if not quit_event.is_set():
            # The button state does not need to go to ros but currently there is no separate state holder (server side)
            # for the button state per timestamp.
            twist = Twist()
            twist.angular.x, twist.angular.z, twist.linear.x, twist.linear.z = (channel, steering, control, throttle)
            twist.linear.y = button
            self._pub.publish(twist)

    def get_odometer_value(self):
        # Meters per second.
        return self._circum_m / self._dt


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
            self._throttle_backward_scale = float(cfg.get('throttle.backward.scale'))
            logger.info("Calibration steer, throttle is {:2.2f}, {:2.2f}.".format(_steer_shift, _throttle_shift))
        except TypeError as e:
            logger.error(traceback.format_exc(e))
            raise e

    def _scale(self, _throttle, _steering):
        # First shift.
        _steering += self._steer_calibration_shift
        _throttle += self._throttle_calibration_shift
        # Then scale and handle the dead-zone when forwards.
        _throttle = _throttle if _throttle < 0 else (_throttle + self._throttle_forward_shift)
        _throttle = (_throttle * self._throttle_backward_scale) if _throttle < 0 else (_throttle * self._throttle_forward_scale)
        # Protect boundaries.
        _steering = int(max(-1, min(1, _steering)) * 180 / 2 + 90)
        _throttle = int(max(-1, min(1, _throttle)) * 180 / 2 + 90)
        return _throttle, _steering

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


def _ros_init():
    # Ros replaces the root logger - add a new handler after ros initialisation.
    rospy.init_node('rover', disable_signals=False, anonymous=True, log_level=rospy.INFO)
    console_handler = logging.StreamHandler(stream=sys.stdout)
    console_handler.setFormatter(logging.Formatter(log_format))
    logging.getLogger().addHandler(console_handler)
    logging.getLogger().setLevel(logging.INFO)
    rospy.on_shutdown(lambda: quit_event.set())


def main():
    parser = argparse.ArgumentParser(description='Rover main.')
    parser.add_argument('--config', type=str, required=True, help='Config file location.')
    args = parser.parse_args()

    parser = SafeConfigParser()
    [parser.read(_f) for _f in args.config.split(',')]
    cfg = dict(parser.items('vehicle'))
    cfg.update(dict(parser.items('platform')))
    for key in sorted(cfg):
        logger.info("{} = {}".format(key, cfg[key]))

    _process_frequency = int(cfg.get('clock.hz'))
    _patience_micro = float(cfg.get('patience.ms')) * 1000
    logger.info("Processing at {} Hz and a patience of {} ms.".format(_process_frequency, _patience_micro / 1000))

    dry_run = bool(int(cfg.get('dry.run')))
    if dry_run:
        gate = FakeGate()
    else:
        _ros_init()
        gate = RosGate()
        logger.info("ROS gate started.")

    state_publisher = JSONPublisher(url='ipc:///byodr/vehicle.sock', topic='aav/vehicle/state')
    image_publisher = ImagePublisher(url='ipc:///byodr/camera.sock', topic='aav/camera/0')

    def _image(_b):
        image_publisher.publish(np.fromstring(_b.extract_dup(0, _b.get_size()), dtype=np.uint8).reshape(CAMERA_SHAPE))

    _url = "rtspsrc " \
           "location=rtsp://user1:HelloUser1@192.168.50.64:554/Streaming/Channels/102 " \
           "latency=0 drop-on-latency=true ! queue ! " \
           "rtph264depay ! h264parse ! queue ! avdec_h264 ! videoconvert ! " \
           "videoscale ! video/x-raw,format=BGR ! queue"
    gst_source = GstRawSource(fn_callback=_image, command=_url)
    gst_source.open()

    vehicle = TwistHandler(ros_gate=gate, **cfg)
    threads = []
    pilot = ReceiverThread(url='ipc:///byodr/pilot.sock', topic=b'aav/pilot/output', event=quit_event)
    threads.append(pilot)
    [t.start() for t in threads]

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
        time.sleep(_period)

    logger.info("Waiting on stream source to close.")
    gst_source.close()

    logger.info("Waiting on threads to stop.")
    [t.join() for t in threads]


if __name__ == "__main__":
    logging.basicConfig(format=log_format)
    logging.getLogger().setLevel(logging.DEBUG)
    main()
