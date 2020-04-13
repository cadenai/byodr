import argparse
import logging
import sys
import traceback

import rospy
from rosserial_python import SerialClient

logger = logging.getLogger(__name__)
log_format = '%(levelname)s: %(filename)s %(funcName)s %(message)s'

if __name__ == "__main__":
    """
        Adapted from http://docs.ros.org/kinetic/api/rosserial_python/html/serial__node_8py_source.html.
        Exit on any exception in order for an outside process manager to restart this process.
        Meant to be able to survive a serial device reconnect.
    """
    parser = argparse.ArgumentParser(description='Custom rosserial script.')
    parser.add_argument('--port', type=str, default='/dev/arduino', help='Port name.')
    parser.add_argument('--baud', type=int, default=57600, help='Baud rate.')
    args = parser.parse_args()

    logging.basicConfig(format=log_format)
    logging.getLogger().setLevel(logging.INFO)

    rospy.init_node('ros_serial', disable_signals=False, anonymous=True, log_level=rospy.INFO)
    console_handler = logging.StreamHandler(stream=sys.stdout)
    console_handler.setFormatter(logging.Formatter(log_format))
    logging.getLogger().addHandler(console_handler)
    logging.getLogger().setLevel(logging.INFO)

    port_name = args.port
    baud = args.baud
    while not rospy.is_shutdown():
        rospy.loginfo("Connecting to %s at %d baud" % (port_name, baud))
        try:
            client = SerialClient(port_name, baud, fix_pyserial_for_test=False)
            client.run()
        except KeyboardInterrupt:
            break
        except Exception as e:
            logger.warn(traceback.format_exc(e))
