import argparse
import glob
import logging
import multiprocessing
import os
import signal
import sys
import time
from ConfigParser import SafeConfigParser

import rospy

from byodr.utils import timestamp
from byodr.utils.ipc import ReceiverThread, JSONPublisher, ImagePublisher
from core import TwistHandler, PTZCamera, GstSource

logger = logging.getLogger(__name__)
log_format = '%(levelname)s: %(filename)s %(funcName)s %(message)s'

quit_event = multiprocessing.Event()

signal.signal(signal.SIGINT, lambda sig, frame: _interrupt())
signal.signal(signal.SIGTERM, lambda sig, frame: _interrupt())


def _interrupt():
    logger.info("Received interrupt, quitting.")
    quit_event.set()


def _latest_or_none(receiver, patience):
    candidate = receiver.get_latest()
    _time = 0 if candidate is None else candidate.get('time')
    _on_time = (timestamp() - _time) < patience
    return candidate if _on_time else None


def ros_init():
    # Ros replaces the root logger - add a new handler after ros initialisation.
    rospy.init_node('rover', disable_signals=False, anonymous=True, log_level=rospy.INFO)
    console_handler = logging.StreamHandler(stream=sys.stdout)
    console_handler.setFormatter(logging.Formatter(log_format))
    logging.getLogger().addHandler(console_handler)
    logging.getLogger().setLevel(logging.INFO)
    rospy.on_shutdown(lambda: quit_event.set())


def main():
    parser = argparse.ArgumentParser(description='Rover main.')
    parser.add_argument('--config', type=str, default='/config', help='Config directory path.')
    args = parser.parse_args()

    parser = SafeConfigParser()
    [parser.read(_f) for _f in ['config.ini'] + glob.glob(os.path.join(args.config, '*.ini'))]
    vehicle_cfg = dict(parser.items('vehicle'))
    camera_cfg = dict(parser.items('camera'))

    ros_init()
    state_publisher = JSONPublisher(url='ipc:///byodr/vehicle.sock', topic='aav/vehicle/state')
    image_publisher = ImagePublisher(url='ipc:///byodr/camera.sock', topic='aav/camera/0')

    vehicle = TwistHandler(**vehicle_cfg)
    ptz_camera = PTZCamera(**camera_cfg)
    gst_source = GstSource(image_publisher, **camera_cfg)

    threads = []
    pilot = ReceiverThread(url='ipc:///byodr/pilot.sock', topic=b'aav/pilot/output', event=quit_event)
    teleop = ReceiverThread(url='ipc:///byodr/teleop.sock', topic=b'aav/teleop/input', event=quit_event)
    threads.append(pilot)
    threads.append(teleop)
    [t.start() for t in threads]

    _process_frequency = int(vehicle_cfg.get('clock.hz'))
    _patience_micro = float(vehicle_cfg.get('patience.ms', 200)) * 1000

    _period = 1. / _process_frequency
    logger.info("Processing at {} Hz and a patience of {} ms.".format(_process_frequency, _patience_micro / 1000))
    while not quit_event.is_set():
        c_pilot = _latest_or_none(pilot, patience=_patience_micro)
        c_teleop = _latest_or_none(teleop, patience=_patience_micro)
        vehicle.drive(c_pilot, c_teleop)
        state_publisher.publish(vehicle.state())
        gst_source.check()
        ptz_camera.add(c_pilot, c_teleop)
        time.sleep(_period)

    logger.info("Waiting on handler to quit.")
    vehicle.quit()

    logger.info("Waiting on stream source to close.")
    gst_source.close()

    logger.info("Waiting on ptz camera to close.")
    ptz_camera.close()

    logger.info("Waiting on threads to stop.")
    [t.join() for t in threads]


if __name__ == "__main__":
    logging.basicConfig(format=log_format)
    logging.getLogger().setLevel(logging.INFO)
    main()
