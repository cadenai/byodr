#!/usr/bin/env python
import argparse
import logging
import multiprocessing
import os
import signal

from tornado import web, ioloop

from byodr.utils.ipc import ReceiverThread, CameraThread, JSONPublisher
from server import CameraServerSocket, ControlServerSocket, MessageServerSocket

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


def main():
    parser = argparse.ArgumentParser(description='Teleop sockets server.')
    parser.add_argument('--port', type=int, default=9100, help='Port number')
    args = parser.parse_args()

    threads = []
    publisher = JSONPublisher(url='ipc:///byodr/teleop.sock', topic='aav/teleop/input')
    pilot = ReceiverThread(url='ipc:///byodr/pilot.sock', topic=b'aav/pilot/output', event=quit_event)
    vehicle = ReceiverThread(url='ipc:///byodr/vehicle.sock', topic=b'aav/vehicle/state', event=quit_event)
    inference = ReceiverThread(url='ipc:///byodr/inference.sock', topic=b'aav/inference/state', event=quit_event)
    recorder = ReceiverThread(url='ipc:///byodr/recorder.sock', topic=b'aav/recorder/state', event=quit_event)
    camera = CameraThread(url='ipc:///byodr/camera.sock', topic=b'aav/camera/0', event=quit_event)
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
            (r"/ws/cam", CameraServerSocket, dict(fn_capture=(lambda: camera.capture()[-1]))),
            (r"/(.*)", web.StaticFileHandler, {
                'path': os.path.join(os.environ.get('TELEOP_HOME'), 'html'),
                'default_filename': 'index.htm'
            })
        ])
        tmp_app = web.Application([
            (r"/ws/ctl", ControlServerSocket, dict(fn_control=(lambda x: publisher.publish(x)))),
            (r"/ws/log", MessageServerSocket, dict(fn_state=(lambda: (pilot.get_latest(),
                                                                      vehicle.get_latest(),
                                                                      inference.get_latest(),
                                                                      recorder.get_latest())))),
            (r"/ws/cam", CameraServerSocket, dict(fn_capture=(lambda: camera.capture()[-1]))),
            (r"/(.*)", web.StaticFileHandler, {
                'path': os.path.join(os.environ.get('TELEOP_HOME'), 'html2'),
                'default_filename': 'index.html'
            })
        ])
        port = args.port
        web_app.listen(port)
        tmp_app.listen(port + 1)
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
