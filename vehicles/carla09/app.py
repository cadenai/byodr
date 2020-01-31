import argparse
import json
import logging
import multiprocessing
import signal
import time

import numpy as np
import zmq

from byodr.utils.ipc import ReceiverThread
from vehicle import create_handler

logger = logging.getLogger(__name__)
quit_event = multiprocessing.Event()

signal.signal(signal.SIGINT, lambda sig, frame: _interrupt())
signal.signal(signal.SIGTERM, lambda sig, frame: _interrupt())


def _interrupt():
    logger.info("Received interrupt, quitting.")
    quit_event.set()


# noinspection PyUnresolvedReferences
class StatePublisher(object):
    def __init__(self):
        publisher = zmq.Context().socket(zmq.PUB)
        publisher.bind('ipc:///byodr/vehicle.sock')
        self._publisher = publisher

    def publish(self, data):
        self._publisher.send('aav/vehicle/state:{}'.format(json.dumps(data)), zmq.NOBLOCK)


# noinspection PyUnresolvedReferences
class ImagePublisher(object):
    def __init__(self):
        publisher = zmq.Context().socket(zmq.PUB)
        publisher.bind('ipc:///byodr/camera.sock')
        self._publisher = publisher

    def publish(self, _img):
        self._publisher.send_multipart(['aav/camera/0',
                                        json.dumps(dict(time=time.time(), shape=_img.shape)),
                                        np.ascontiguousarray(_img, dtype=np.uint8)],
                                       flags=zmq.NOBLOCK)


def main():
    parser = argparse.ArgumentParser(description='Carla vehicle client.')
    parser.add_argument('--remote', type=str, required=True, help='Carla server remote host:port')
    parser.add_argument('--clock', type=int, required=True, help='Clock frequency in hz.')
    args = parser.parse_args()

    state_publisher = StatePublisher()
    image_publisher = ImagePublisher()

    vehicle = create_handler(remote=args.remote, on_image=(lambda x: image_publisher.publish(x)))
    vehicle.start()

    threads = []
    pilot = ReceiverThread(url='ipc:///byodr/pilot.sock', topic=b'aav/pilot/output', event=quit_event)
    threads.append(pilot)
    [t.start() for t in threads]

    _hz = args.clock
    _period = 1. / _hz
    while not quit_event.is_set():
        command = pilot.get_latest()
        _command_time = 0 if command is None else command.get('time')
        _command_age = time.time() - _command_time
        _on_time = _command_age < (2 * _period)
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
