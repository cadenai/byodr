#!/usr/bin/env python
import argparse
import glob
import json
import logging
import multiprocessing
import os
import signal
import threading
import traceback
from ConfigParser import SafeConfigParser

from tornado import web, ioloop, websocket
from tornado.httpserver import HTTPServer

from byodr.utils import Application, Configurable
from byodr.utils.ipc import JSONReceiver, CollectorThread, LocalIPCServer
from byodr.utils.option import parse_option
from byodr.utils.video import GstRawSource

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


class GstSource(Configurable):
    def __init__(self, position='front'):
        super(GstSource, self).__init__()
        self._im_width, self._im_height = 640, 480
        self._position = position
        self._listeners = []
        self._source = None

    def get_image_width(self):
        return self._im_width

    def get_image_height(self):
        return self._im_height

    def is_open(self):
        return self._source is not None and self._source.is_open()

    def add_listener(self, listener):
        with self._lock:
            self._listeners.append(listener)

    def remove_listener(self, listener):
        with self._lock:
            self._listeners.remove(listener)

    def _publish(self, _b):
        with self._lock:
            map(lambda a: a(_b.extract_dup(0, _b.get_size())), self._listeners)

    def check(self):
        with self._lock:
            if self._source:
                self._source.check()

    def internal_quit(self, restarting=False):
        if self._source:
            self._source.close()

    def internal_start(self, **kwargs):
        _errors = []
        _server = parse_option(self._position + '.camera.ip', str, errors=_errors, **kwargs)
        _user = parse_option(self._position + '.camera.user', str, errors=_errors, **kwargs)
        _password = parse_option(self._position + '.camera.password', str, errors=_errors, **kwargs)
        _rtsp_port = parse_option(self._position + '.camera.rtsp.port', int, 0, errors=_errors, **kwargs)
        _rtsp_path = parse_option(self._position + '.camera.stream.path', str, errors=_errors, **kwargs)
        _img_wh = parse_option(self._position + '.camera.stream.shape', str, errors=_errors, **kwargs)
        self._im_width, self._im_height = [int(x) for x in _img_wh.split('x')]
        if self._source:
            self._source.close()
        if len(_errors) == 0:
            _rtsp_url = 'rtsp://{user}:{password}@{ip}:{port}{path}'.format(
                **dict(user=_user, password=_password, ip=_server, port=_rtsp_port, path=_rtsp_path)
            )
            _command = "rtspsrc " \
                       "location={url} " \
                       "latency=0 drop-on-latency=true do-retransmission=false ! queue ! " \
                       "rtph264depay ! h264parse ! queue ! video/x-h264,stream-format=\"byte-stream\" ! queue". \
                format(**dict(url=_rtsp_url))
            logger.info("Camera stream url = {}.".format(_rtsp_url))
            self._source = GstRawSource(name=self._position, fn_callback=self._publish, command=_command)
            self._source.open()
        return _errors


class H264StreamSocket(websocket.WebSocketHandler):
    def __init__(self, application, request, **kwargs):
        super(H264StreamSocket, self).__init__(application, request, **kwargs)
        self._lock = threading.Lock()
        self._streaming = False

    # noinspection PyAttributeOutsideInit
    def initialize(self, **kwargs):
        self._gst_source = kwargs.get('gst_source')
        self._image_width = kwargs.get('fn_image_width')()
        self._image_height = kwargs.get('fn_image_height')()

    def _stream(self, _bytes):
        with self._lock:
            if self._streaming:
                self.write_message(_bytes, binary=True)

    def check_origin(self, origin):
        return True

    def data_received(self, chunk):
        pass

    def open(self, *args, **kwargs):
        self._gst_source.add_listener(self._stream)
        self.write_message(json.dumps(dict(action='init', width=self._image_width, height=self._image_height)))

    def on_close(self):
        self._gst_source.remove_listener(self._stream)

    def on_message(self, message):
        try:
            with self._lock:
                self._streaming = 'REQUESTSTREAM' in message
                logger.info("On message - streaming = {}.".format(self._streaming))
        except Exception as e:
            logger.error("Stream socket@on_message: {} {}".format(e, traceback.format_exc(e)))
            logger.error("Input message:---\n{}\n---".format(message))


class CameraApplication(Application):
    def __init__(self, event, config_dir=os.getcwd()):
        super(CameraApplication, self).__init__(quit_event=event, run_hz=2)
        self._config_dir = config_dir
        self._stream = GstSource()
        self.ipc_server = None
        self.ipc_chatter = None

    def _config(self):
        parser = SafeConfigParser()
        [parser.read(_f) for _f in ['config.ini'] + glob.glob(os.path.join(self._config_dir, '*.ini'))]
        cfg = dict(parser.items('camera'))
        return cfg

    def get_camera(self):
        return self._stream

    def get_image_width(self):
        return self._stream.get_image_width()

    def get_image_height(self):
        return self._stream.get_image_height()

    def setup(self):
        if self.active():
            _config = self._config()
            if self._stream.is_open():
                self._stream.restart(**_config)
            else:
                self._stream.start(**_config)
            self.ipc_server.register_start(self._stream.get_errors())

    def step(self):
        self._stream.check()
        chat = self.ipc_chatter()
        if chat and chat.get('command') == 'restart':
            self.setup()


def main():
    parser = argparse.ArgumentParser(description='Camera web-socket server.')
    parser.add_argument('--config', type=str, default='/config', help='Config directory path.')
    args = parser.parse_args()

    ipc_chatter = JSONReceiver(url='ipc:///byodr/teleop_c.sock', topic=b'aav/teleop/chatter', pop=True)
    collector = CollectorThread(receivers=ipc_chatter, event=quit_event)

    application = CameraApplication(event=quit_event, config_dir=args.config)
    application.ipc_chatter = lambda: collector.get(0)
    application.ipc_server = LocalIPCServer(url='ipc:///byodr/camera_c.sock', name='camera_ws', event=quit_event)

    threads = [collector, (threading.Thread(target=application.run)), application.ipc_server]
    if quit_event.is_set():
        return 0

    [t.start() for t in threads]

    try:
        main_app = web.Application([
            (r"/", H264StreamSocket, dict(gst_source=application.get_camera(),
                                          fn_image_width=(lambda: application.get_image_width()),
                                          fn_image_height=(lambda: application.get_image_height())))
        ])
        http_server = HTTPServer(main_app, xheaders=True)
        http_server.bind(9101)
        http_server.start()
        logger.info("Web services started on port 9101.")
        io_loop.start()
    except KeyboardInterrupt:
        quit_event.set()

    logger.info("Waiting on threads to stop.")
    [t.join() for t in threads]


if __name__ == "__main__":
    logging.basicConfig(format=log_format)
    logging.getLogger().setLevel(logging.INFO)
    main()
