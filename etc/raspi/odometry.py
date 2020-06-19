import argparse
import httplib
import logging
import multiprocessing
import os
import signal
import socket
import sys
import time
import xmlrpclib

import rospy
from geometry_msgs.msg import TwistStamped

logger = logging.getLogger(__name__)
log_format = '%(levelname)s: %(filename)s %(funcName)s %(message)s'

quit_event = multiprocessing.Event()
rospy.on_shutdown(lambda: quit_event.set())

signal.signal(signal.SIGINT, lambda sig, frame: _interrupt())
signal.signal(signal.SIGTERM, lambda sig, frame: _interrupt())


def _interrupt():
    logger.info("Received interrupt, quitting.")
    quit_event.set()


def node(hz):
    rospy.init_node('pi_odometry', disable_signals=False, anonymous=True, log_level=rospy.INFO)
    console_handler = logging.StreamHandler(stream=sys.stdout)
    console_handler.setFormatter(logging.Formatter(log_format))
    logging.getLogger().addHandler(console_handler)
    logging.getLogger().setLevel(logging.INFO)
    _publisher = rospy.Publisher("roy_teleop/sensor/odometer", TwistStamped, queue_size=1)
    rate = rospy.Rate(hz)
    while not quit_event.is_set():
        odo_message = TwistStamped()
        """
            message.header.stamp = nodeHandle.now();  
            message.twist.linear.x = h_up;
            message.twist.linear.y = h_rps;
            message.twist.linear.z = analogRead(HALL_IN_PIN);
        """
        odo_message.twist.linear.x = 0
        odo_message.twist.linear.y = 0
        odo_message.twist.linear.z = 0
        _publisher.publish(odo_message)
        # Survive ros master restarts.
        rospy.get_master().getUri()
        rate.sleep()


def main():
    parser = argparse.ArgumentParser(description='Pi main.')
    parser.add_argument('--hz', type=int, default=25, help='Publish frequency')
    args = parser.parse_args()

    hz = args.hz
    while not quit_event.is_set():
        try:
            rospy.init_node('pi_odometry', disable_signals=False, anonymous=True, log_level=rospy.INFO)
            console_handler = logging.StreamHandler(stream=sys.stdout)
            console_handler.setFormatter(logging.Formatter(log_format))
            logging.getLogger().addHandler(console_handler)
            logging.getLogger().setLevel(logging.INFO)
            _publisher = rospy.Publisher("roy_teleop/sensor/odometer", TwistStamped, queue_size=1)
            rate = rospy.Rate(hz)
            m_server = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
            while not quit_event.is_set():
                odo_message = TwistStamped()
                """
                    message.header.stamp = nodeHandle.now();  
                    message.twist.linear.x = h_up;
                    message.twist.linear.y = h_rps;
                    message.twist.linear.z = analogRead(HALL_IN_PIN);
                """
                odo_message.twist.linear.x = 0
                odo_message.twist.linear.y = 0
                odo_message.twist.linear.z = 0
                _publisher.publish(odo_message)
                # Survive ros master restarts.
                m_server.getUri('odometry_caller')
                rate.sleep()
        except (socket.error, httplib.HTTPException):
            time.sleep(1)
        except rospy.ROSInterruptException:
            pass


if __name__ == "__main__":
    logging.basicConfig(format=log_format)
    logging.getLogger().setLevel(logging.INFO)
    main()
