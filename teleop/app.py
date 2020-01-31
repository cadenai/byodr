#!/usr/bin/env python
import argparse
import collections
import json
import logging
import multiprocessing
import os
import signal
import threading

import numpy as np
import zmq
from tornado import web, ioloop

from byodr.utils.ipc import ReceiverThread
from teleop import CameraServerSocket, ControlServerSocket, MessageServerSocket

logger = logging.getLogger(__name__)

log_format = '%(levelname)s: %(filename)s %(funcName)s %(message)s'

io_loop = ioloop.IOLoop.instance()
signal.signal(signal.SIGINT, lambda sig, frame: io_loop.add_callback_from_signal(_interrupt))
signal.signal(signal.SIGTERM, lambda sig, frame: io_loop.add_callback_from_signal(_interrupt))

quit_event = multiprocessing.Event()


def _interrupt():
    logger.info("Received interrupt, quitting.")
    quit_event.set()
    io_loop.stop()


# noinspection PyUnresolvedReferences
class CameraThread(threading.Thread):
    def __init__(self):
        super(CameraThread, self).__init__()
        subscriber = zmq.Context().socket(zmq.SUB)
        subscriber.setsockopt(zmq.RCVHWM, 1)
        subscriber.setsockopt(zmq.RCVTIMEO, 20)
        subscriber.setsockopt(zmq.LINGER, 0)
        subscriber.connect('ipc:///byodr/camera.sock')
        subscriber.setsockopt(zmq.SUBSCRIBE, b'aav/camera/0')
        self._subscriber = subscriber
        self._images = collections.deque(maxlen=1)

    def capture(self):
        return self._images[0] if bool(self._images) else None

    def run(self):
        while not quit_event.is_set():
            try:
                [_, md, data] = self._subscriber.recv_multipart()
                md = json.loads(md)
                height, width, channels = md['shape']
                img = np.frombuffer(buffer(data), dtype=np.uint8)
                img = img.reshape((height, width, channels))
                self._images.appendleft(img)
            except zmq.Again:
                pass


# noinspection PyUnresolvedReferences
class TeleopPublisher(object):
    def __init__(self):
        publisher = zmq.Context().socket(zmq.PUB)
        publisher.bind('ipc:///byodr/teleop.sock')
        self._publisher = publisher

    def publish(self, data):
        self._publisher.send('aav/teleop/input:{}'.format(json.dumps(data)), zmq.NOBLOCK)


def main():
    parser = argparse.ArgumentParser(description='Teleop sockets server.')
    parser.add_argument('--port', type=int, default=9100, help='Port number')
    args = parser.parse_args()

    threads = []
    publisher = TeleopPublisher()
    pilot = ReceiverThread(url='ipc:///byodr/pilot.sock', topic=b'aav/pilot/output', event=quit_event)
    vehicle = ReceiverThread(url='ipc:///byodr/vehicle.sock', topic=b'aav/vehicle/state', event=quit_event)
    inference = ReceiverThread(url='ipc:///byodr/inference.sock', topic=b'aav/inference/state', event=quit_event)
    recorder = ReceiverThread(url='ipc:///byodr/recorder.sock', topic=b'aav/recorder/state', event=quit_event)
    camera = CameraThread()
    threads.append(pilot)
    threads.append(vehicle)
    threads.append(inference)
    threads.append(recorder)
    threads.append(camera)
    [t.start() for t in threads]

    try:
        web_app = web.Application([
            (r"/ws/ctl", ControlServerSocket, dict(fn_control=(lambda x: publisher.publish(x)))),
            (r"/ws/log", MessageServerSocket, dict(fn_state=(lambda: (pilot.get_latest(),
                                                                      vehicle.get_latest(),
                                                                      inference.get_latest(),
                                                                      recorder.get_latest())))),
            (r"/ws/cam", CameraServerSocket, dict(fn_capture=(lambda: camera.capture()))),
            (r"/(.*)", web.StaticFileHandler, {
                'path': os.path.join(os.environ.get('TELEOP_HOME'), 'html'),
                'default_filename': 'index.htm'
            })
        ])
        port = args.port
        web_app.listen(port)
        logger.info("Web service starting on port {}.".format(port))
        io_loop.start()
    except KeyboardInterrupt:
        quit_event.set()

    logger.info("Waiting on threads to stop.")
    [t.join() for t in threads]


if __name__ == "__main__":
    logging.basicConfig(format=log_format)
    logging.getLogger().setLevel(logging.DEBUG)
    main()
