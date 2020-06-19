import argparse
import httplib
import logging
import multiprocessing
import os
import signal
import socket
import sys
import xmlrpclib

import rospy
from geometry_msgs.msg import Twist
from gpiozero import AngularServo

logger = logging.getLogger(__name__)
log_format = '%(levelname)s: %(filename)s %(funcName)s %(message)s'

quit_event = multiprocessing.Event()
rospy.on_shutdown(lambda: quit_event.set())

signal.signal(signal.SIGINT, lambda sig, frame: _interrupt())
signal.signal(signal.SIGTERM, lambda sig, frame: _interrupt())


def _interrupt():
    logger.info("Received interrupt, quitting.")
    quit_event.set()


def _drive(servo_steering, servo_throttle, msg):
    # The calibration shifts are at the x values.
    throttle = msg.linear.x + msg.linear.y
    steering = msg.angular.x + msg.angular.y
    # Apply hard-coded ceilings as safety.
    throttle = -90 + (140 if throttle > 140 else 40 if throttle < 40 else throttle)
    if servo_throttle.angle != throttle:
        servo_throttle.angle = throttle
    steering = -90 + (180 if steering > 180 else 0 if steering < 0 else steering)
    if servo_steering.angle != steering:
        servo_steering.angle = steering


def main():
    parser = argparse.ArgumentParser(description='Pi servo control.')
    parser.add_argument('--servo_pin', type=int, default=12, help='Steering servo pin id')
    parser.add_argument('--motor_pin', type=int, default=13, help='Drive motor pin id')
    args = parser.parse_args()

    _steering_servo = AngularServo(pin=args.servo_pin, min_pulse_width=0.0005, max_pulse_width=0.0020, frame_width=0.013)
    _throttle_servo = AngularServo(pin=args.motor_pin)

    rospy.init_node('pi_servos', disable_signals=False, anonymous=True, log_level=rospy.INFO)
    console_handler = logging.StreamHandler(stream=sys.stdout)
    console_handler.setFormatter(logging.Formatter(log_format))
    logging.getLogger().addHandler(console_handler)
    logging.getLogger().setLevel(logging.INFO)
    _subscriber = rospy.Subscriber("roy_teleop/command/drive", Twist, lambda msg: _drive(_steering_servo, _throttle_servo, msg))

    try:
        # Recover from ros master restarts.
        rate = rospy.Rate(5)
        m_server = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
        while not rospy.is_shutdown() and not quit_event.is_set():
            logger.info("calling")
            m_server.getUri('servos_caller')
            rate.sleep()
    except (socket.error, httplib.HTTPException):
        # Let the process manager resume.
        exit(1)

    logger.info("End")


if __name__ == "__main__":
    logging.basicConfig(format=log_format)
    logging.getLogger().setLevel(logging.INFO)
    main()
