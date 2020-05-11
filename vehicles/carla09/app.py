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
from byodr.utils.option import parse_option
from vehicle import CarlaHandler

logger = logging.getLogger(__name__)
quit_event = multiprocessing.Event()

signal.signal(signal.SIGINT, lambda sig, frame: _interrupt())
signal.signal(signal.SIGTERM, lambda sig, frame: _interrupt())


def _interrupt():
    logger.info("Received interrupt, quitting.")
    quit_event.set()


class IPCServer(LocalIPCServer):
    def __init__(self, url, event, receive_timeout_ms=50):
        super(IPCServer, self).__init__('platform', url, event, receive_timeout_ms)

    def serve_local(self, message):
        return {}


def _latest_or_none(receiver, patience):
    candidate = receiver.get_latest()
    _time = 0 if candidate is None else candidate.get('time')
    _on_time = (timestamp() - _time) < patience
    return candidate if _on_time else None


def main():
    parser = argparse.ArgumentParser(description='Carla vehicle client.')
    parser.add_argument('--config', type=str, default='/config', help='Config directory path.')
    args = parser.parse_args()

    parser = SafeConfigParser()
    [parser.read(_f) for _f in ['config.ini'] + glob.glob(os.path.join(args.config, '*.ini'))]
    cfg = dict(parser.items('vehicle'))
    cfg.update(dict(parser.items('platform')))

    _errors = []
    _process_frequency = parse_option('clock.hz', int, 10, _errors, **cfg)
    _patience_micro = parse_option('patience.ms', int, 200, _errors, **cfg) * 1000.
    logger.info("Processing at {} Hz and a patience of {} ms.".format(_process_frequency, _patience_micro / 1000))

    state_publisher = JSONPublisher(url='ipc:///byodr/vehicle.sock', topic='aav/vehicle/state')
    image_publisher = ImagePublisher(url='ipc:///byodr/camera.sock', topic='aav/camera/0')

    _remote = cfg.get('host.location')
    vehicle = CarlaHandler((lambda x: image_publisher.publish(x)), **cfg)
    vehicle.start()

    teleop = ReceiverThread(url='ipc:///byodr/teleop.sock', topic=b'aav/teleop/input', event=quit_event)
    pilot = ReceiverThread(url='ipc:///byodr/pilot.sock', topic=b'aav/pilot/output', event=quit_event)
    ipc_server = IPCServer(url='ipc:///byodr/vehicle_c.sock', event=quit_event)
    threads = [teleop, pilot, ipc_server]
    if quit_event.is_set():
        return 0

    [t.start() for t in threads]
    ipc_server.register_start(_errors)
    _period = 1. / _process_frequency
    while not quit_event.is_set():
        c_pilot = _latest_or_none(pilot, patience=_patience_micro)
        c_teleop = _latest_or_none(teleop, patience=_patience_micro)
        if c_teleop is not None and c_teleop.get('button_x', 0):
            vehicle.quit()
            vehicle.start()
        if c_pilot is not None:
            vehicle.drive(c_pilot)
        else:
            vehicle.noop()
        state_publisher.publish(vehicle.state())
        time.sleep(_period)

    logger.info("Waiting on threads to stop.")
    [t.join() for t in threads]

    logger.info("Waiting on carla to quit.")
    vehicle.quit()


if __name__ == "__main__":
    logging.basicConfig(format='%(levelname)s: %(filename)s %(funcName)s %(message)s')
    logging.getLogger().setLevel(logging.INFO)
    main()
