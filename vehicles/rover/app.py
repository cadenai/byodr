import argparse
import collections
import json
import logging
import multiprocessing
import signal
import threading
import time

import cv2
import numpy as np
import zmq

logger = logging.getLogger(__name__)
quit_event = multiprocessing.Event()

CAMERA_SHAPE = (240, 320, 3)

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


class CaptureThread(threading.Thread):
    def __init__(self, url, hz=50):
        super(CaptureThread, self).__init__()
        self._url = url
        self._capture = cv2.VideoCapture(url)
        self._sleep = 1. / hz
        self._publisher = ImagePublisher()

    def run(self):
        while not quit_event.is_set():
            _start = time.time()
            suc, img = self._capture.read()
            if suc:
                _height, _width, _ = CAMERA_SHAPE
                self._publisher.publish(cv2.resize(img, (_width, _height)))
            else:
                logger.info("Could not capture from '{}' - retrying in 100 ms.".format(self._url))
                time.sleep(.100)
                self._capture = cv2.VideoCapture(self._url)
            _duration = time.time() - _start
            time.sleep(max(0., self._sleep - _duration))


def main():
    parser = argparse.ArgumentParser(description='Rover main.')
    parser.add_argument('--clock', type=int, default=40, help='Main loop frequency in hz.')
    parser.add_argument('--fps', type=int, default=20, help='Camera capture frequency in hz.')

    args = parser.parse_args()

    state_publisher = StatePublisher()

    # vehicle = create_handler()
    # vehicle.start()

    threads = []
    pilot = ReceiverThread(url='ipc:///tmp/byodr/pilot.sock', topic=b'aav/pilot/output')
    _fps = args.fps
    capture = CaptureThread(url='rtsp://user1:HelloUser1@192.168.50.64:554/Streaming/Channels/102', hz=_fps)
    threads.append(pilot)
    threads.append(capture)
    [t.start() for t in threads]

    _hz = args.clock
    logger.info("Running at {} hz and a capture rate of {}.".format(_hz, _fps))
    while not quit_event.is_set():
        # state_publisher.publish(vehicle.state())
        time.sleep(1. / _hz)

    logger.info("Waiting on threads to stop.")
    [t.join() for t in threads]

    # logger.info("Waiting on vehicle to quit.")
    # vehicle.quit()


if __name__ == "__main__":
    logging.basicConfig(format='%(levelname)s: %(filename)s %(funcName)s %(message)s')
    logging.getLogger().setLevel(logging.DEBUG)
    main()
