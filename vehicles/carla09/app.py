import argparse
import logging
import multiprocessing
import signal
import time

from byodr.utils.ipc import ReceiverThread, JSONPublisher, ImagePublisher
from vehicle import create_handler

logger = logging.getLogger(__name__)
quit_event = multiprocessing.Event()

signal.signal(signal.SIGINT, lambda sig, frame: _interrupt())
signal.signal(signal.SIGTERM, lambda sig, frame: _interrupt())


def _interrupt():
    logger.info("Received interrupt, quitting.")
    quit_event.set()


def main():
    parser = argparse.ArgumentParser(description='Carla vehicle client.')
    parser.add_argument('--remote', type=str, required=True, help='Carla server remote host:port')
    parser.add_argument('--clock', type=int, required=True, help='Clock frequency in hz.')
    parser.add_argument('--patience', type=float, default=.100, help='Maximum age of a command before it is considered stale.')
    args = parser.parse_args()

    state_publisher = JSONPublisher(url='ipc:///byodr/vehicle.sock', topic='aav/vehicle/state')
    image_publisher = ImagePublisher(url='ipc:///byodr/camera.sock', topic='aav/camera/0')

    vehicle = create_handler(remote=args.remote, on_image=(lambda x: image_publisher.publish(x)))
    vehicle.start()

    threads = []
    pilot = ReceiverThread(url='ipc:///byodr/pilot.sock', topic=b'aav/pilot/output', event=quit_event)
    threads.append(pilot)
    [t.start() for t in threads]

    _hz = args.clock
    _patience = args.patience
    _period = 1. / _hz
    while not quit_event.is_set():
        command = pilot.get_latest()
        _command_time = 0 if command is None else command.get('time')
        _command_age = time.time() - _command_time
        _on_time = _command_age < _patience
        if _on_time:
            vehicle.drive(command)
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
    logging.getLogger().setLevel(logging.DEBUG)
    main()
