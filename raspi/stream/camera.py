#!/usr/bin/env python
import argparse
import asyncio
import logging
import multiprocessing
import os
import signal
import threading
import time
from configparser import ConfigParser as SafeConfigParser

from tornado import web, ioloop
from tornado.platform.asyncio import AnyThreadEventLoopPolicy

from byodr.utils import Application
from byodr.utils.option import parse_option
from byodr.utils.video import create_rtsp_video_stream_source
from byodr.utils.websocket import HttpLivePlayerVideoSocket

logger = logging.getLogger(__name__)

log_format = '%(levelname)s: %(filename)s %(funcName)s %(message)s'

signal.signal(signal.SIGINT, lambda sig, frame: _interrupt())
signal.signal(signal.SIGTERM, lambda sig, frame: _interrupt())

quit_event = multiprocessing.Event()


def _interrupt():
    logger.info("Received interrupt, quitting.")
    quit_event.set()


class CameraApplication(Application):
    def __init__(self, stream, event):
        super(CameraApplication, self).__init__(quit_event=event, run_hz=2)
        self._stream = stream

    def setup(self):
        pass

    def step(self):
        self._stream.check()


def create_stream(config_file):
    parser = SafeConfigParser()
    parser.read(config_file)
    kwargs = dict(parser.items('camera'))
    _errors = []
    name = os.path.basename(os.path.splitext(config_file)[0])
    config = {
        'name': name,
        'uri': (parse_option('camera.uri', str, errors=_errors, **kwargs)),
        'ip': (parse_option('camera.ip', str, errors=_errors, **kwargs)),
        'user': (parse_option('camera.user', str, errors=_errors, **kwargs)),
        'password': (parse_option('camera.password', str, errors=_errors, **kwargs)),
        'shape': (parse_option('camera.input.shape', str, errors=_errors, **kwargs))
    }
    assert len(_errors) == 0
    return create_rtsp_video_stream_source(**config)


def main():
    parser = argparse.ArgumentParser(description='Camera web-socket server.')
    parser.add_argument('--config', type=str, default='/config/camera0.ini', help='Configuration file.')
    parser.add_argument('--port', type=int, default=9101, help='Socket port.')
    args = parser.parse_args()

    config_file = args.config
    if os.path.exists(config_file) and os.path.isfile(config_file):
        video_stream = create_stream(config_file)
        application = CameraApplication(stream=video_stream, event=quit_event)

        threads = [threading.Thread(target=application.run)]
        if quit_event.is_set():
            return 0

        [t.start() for t in threads]

        asyncio.set_event_loop_policy(AnyThreadEventLoopPolicy())
        asyncio.set_event_loop(asyncio.new_event_loop())

        io_loop = ioloop.IOLoop.instance()
        web_app = web.Application([(r"/", HttpLivePlayerVideoSocket, dict(video_source=video_stream, io_loop=io_loop))])
        rear_server = web.HTTPServer(web_app, xheaders=True)
        rear_server.bind(args.port)
        rear_server.start()
        io_loop.start()
        logger.info("Web service started on port {}.".format(args.port))

        logger.info("Waiting on threads to stop.")
        [t.join() for t in threads]
    else:
        while not quit_event.is_set():
            time.sleep(.2)


if __name__ == "__main__":
    logging.basicConfig(format=log_format)
    logging.getLogger().setLevel(logging.INFO)
    main()
