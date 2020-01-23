import argparse
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
class DriveThread(threading.Thread):
    def __init__(self, vehicle):
        super(DriveThread, self).__init__()
        self._vehicle = vehicle
        subscriber = zmq.Context().socket(zmq.SUB)
        subscriber.setsockopt(zmq.RCVHWM, 1)
        subscriber.setsockopt(zmq.RCVTIMEO, 10)
        subscriber.setsockopt(zmq.LINGER, 0)
        subscriber.connect('ipc:///tmp/byodr/pilot.sock')
        subscriber.setsockopt(zmq.SUBSCRIBE, b'aav/pilot/output')
        self._subscriber = subscriber

    def run(self):
        while not quit_event.is_set():
            try:
                self._vehicle.drive(json.loads(self._subscriber.recv().split(':', 1)[1]))
            except zmq.Again:
                pass


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


def main():
    parser = argparse.ArgumentParser(description='Carla vehicle client.')
    parser.add_argument('--remote', type=str, required=True, help='Carla server remote host:port')
    args = parser.parse_args()

    state_publisher = StatePublisher()
    image_publisher = ImagePublisher()

    vehicle = create_handler(remote=args.remote, on_image=(lambda x: image_publisher.publish(x)))
    vehicle.start()

    drive_thread = DriveThread(vehicle=vehicle)
    drive_thread.start()

    while not quit_event.is_set():
        state_publisher.publish(vehicle.state())
        time.sleep(1. / 40)

    logger.info("Waiting on thread to stop.")
    drive_thread.join()

    logger.info("Waiting on carla to quit.")
    vehicle.quit()


if __name__ == "__main__":
    logging.basicConfig(format='%(levelname)s: %(filename)s %(funcName)s %(message)s')
    logging.getLogger().setLevel(logging.DEBUG)
    main()
