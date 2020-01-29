import argparse
import collections
import json
import logging
import multiprocessing
import signal
import threading
import time

import numpy as np
import zmq

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
        publisher.bind('ipc:///tmp/byodr/vehicle.sock')
        self._publisher = publisher

    def publish(self, data):
        self._publisher.send('aav/vehicle/state:{}'.format(json.dumps(data)), zmq.NOBLOCK)


# noinspection PyUnresolvedReferences
class ImagePublisher(object):
    def __init__(self):
        publisher = zmq.Context().socket(zmq.PUB)
        publisher.bind('ipc:///tmp/byodr/camera.sock')
        self._publisher = publisher

    def publish(self, _img):
        self._publisher.send_multipart(['aav/camera/0',
                                        json.dumps(dict(time=time.time(), shape=_img.shape)),
                                        np.ascontiguousarray(_img, dtype=np.uint8)],
                                       flags=zmq.NOBLOCK)


# noinspection PyUnresolvedReferences
class ReceiverThread(threading.Thread):
    def __init__(self, url, topic=''):
        super(ReceiverThread, self).__init__()
        subscriber = zmq.Context().socket(zmq.SUB)
        subscriber.setsockopt(zmq.RCVHWM, 1)
        subscriber.setsockopt(zmq.RCVTIMEO, 10)
        subscriber.setsockopt(zmq.LINGER, 0)
        subscriber.connect(url)
        subscriber.setsockopt(zmq.SUBSCRIBE, topic)
        self._subscriber = subscriber
        self._queue = collections.deque(maxlen=1)

    def get_latest(self):
        return self._queue[0] if bool(self._queue) else None

    def run(self):
        while not quit_event.is_set():
            try:
                self._queue.appendleft(json.loads(self._subscriber.recv().split(':', 1)[1]))
            except zmq.Again:
                pass


def main():
    parser = argparse.ArgumentParser(description='Carla vehicle client.')
    parser.add_argument('--remote', type=str, required=True, help='Carla server remote host:port')
    parser.add_argument('--clock', type=int, default=50, help='Clock frequency in hz.')
    args = parser.parse_args()

    state_publisher = StatePublisher()
    image_publisher = ImagePublisher()

    vehicle = create_handler(remote=args.remote, on_image=(lambda x: image_publisher.publish(x)))
    vehicle.start()

    threads = []
    pilot = ReceiverThread(url='ipc:///tmp/byodr/pilot.sock', topic=b'aav/pilot/output')
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
