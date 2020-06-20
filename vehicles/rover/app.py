import argparse
import glob
import logging
import multiprocessing
import os
import signal
import time
from ConfigParser import SafeConfigParser

from byodr.utils import timestamp
from byodr.utils.ipc import ReceiverThread, JSONPublisher, ImagePublisher, LocalIPCServer
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


class IPCServer(LocalIPCServer):
    def __init__(self, url, event, receive_timeout_ms=50):
        super(IPCServer, self).__init__('platform', url, event, receive_timeout_ms)

    def serve_local(self, message):
        return {}


def get_parser(config_dir):
    parser = SafeConfigParser()
    [parser.read(_f) for _f in ['config.ini'] + glob.glob(os.path.join(config_dir, '*.ini'))]
    return parser


def create_all(ipc_server, image_publisher, config_dir, previous=(None, None, None)):
    parser = get_parser(config_dir)
    vehicle_cfg = dict(parser.items('vehicle'))
    camera_cfg = dict(parser.items('camera'))
    vehicle, ptz_camera, gst_source = previous
    _configured = any(map(lambda _x: _x is None, previous))
    if vehicle is None:
        vehicle = TwistHandler(**vehicle_cfg)
    elif vehicle.is_reconfigured(**vehicle_cfg):
        vehicle.restart(**vehicle_cfg)
        _configured = True
    if ptz_camera is None:
        ptz_camera = PTZCamera(**camera_cfg)
    elif ptz_camera.is_reconfigured(**camera_cfg):
        ptz_camera.restart(**camera_cfg)
        _configured = True
    if gst_source is None:
        gst_source = GstSource(image_publisher, **camera_cfg)
    elif gst_source.is_reconfigured(**camera_cfg):
        gst_source.restart(**camera_cfg)
        _configured = True
    if _configured:
        errors = []
        [errors.extend(x.get_errors()) for x in (vehicle, ptz_camera, gst_source)]
        ipc_server.register_start(errors)
        _process_frequency = vehicle.get_process_frequency()
        _patience_micro = vehicle.get_patience_micro()
        logger.info("Processing at {} Hz and a patience of {} ms.".format(_process_frequency, _patience_micro / 1000))
    return vehicle, ptz_camera, gst_source


def main():
    parser = argparse.ArgumentParser(description='Rover main.')
    parser.add_argument('--config', type=str, default='/config', help='Config directory path.')
    args = parser.parse_args()

    state_publisher = JSONPublisher(url='ipc:///byodr/vehicle.sock', topic='aav/vehicle/state')
    image_publisher = ImagePublisher(url='ipc:///byodr/camera.sock', topic='aav/camera/0')

    pilot = ReceiverThread(url='ipc:///byodr/pilot.sock', topic=b'aav/pilot/output', event=quit_event)
    teleop = ReceiverThread(url='ipc:///byodr/teleop.sock', topic=b'aav/teleop/input', event=quit_event)
    ipc_chatter = ReceiverThread(url='ipc:///byodr/teleop.sock', topic=b'aav/teleop/chatter', event=quit_event)
    ipc_server = IPCServer(url='ipc:///byodr/vehicle_c.sock', event=quit_event)
    threads = [pilot, teleop, ipc_chatter, ipc_server]
    if quit_event.is_set():
        return 0

    vehicle, ptz_camera, gst_source = create_all(ipc_server, image_publisher, args.config)
    [t.start() for t in threads]
    _period = 1. / vehicle.get_process_frequency()
    while not quit_event.is_set():
        c_pilot = _latest_or_none(pilot, patience=vehicle.get_patience_micro())
        c_teleop = _latest_or_none(teleop, patience=vehicle.get_patience_micro())
        vehicle.drive(c_pilot, c_teleop)
        state_publisher.publish(vehicle.state())
        gst_source.check()
        ptz_camera.add(c_pilot, c_teleop)
        chat = ipc_chatter.pop_latest()
        if chat and chat.get('command') == 'restart':
            previous = (vehicle, ptz_camera, gst_source)
            vehicle, ptz_camera, gst_source = create_all(ipc_server, image_publisher, args.config, previous=previous)
            _period = 1. / vehicle.get_process_frequency()
        else:
            time.sleep(_period)

    logger.info("Waiting on handler to quit.")
    vehicle.quit()

    logger.info("Waiting on stream source to close.")
    gst_source.quit()

    logger.info("Waiting on ptz camera to close.")
    ptz_camera.quit()

    logger.info("Waiting on threads to stop.")
    [t.join() for t in threads]


if __name__ == "__main__":
    logging.basicConfig(format=log_format)
    logging.getLogger().setLevel(logging.INFO)
    main()
