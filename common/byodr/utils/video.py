from __future__ import absolute_import

import json
import logging
import threading
import time
import traceback

import gi
from tornado import websocket

gi.require_version('Gst', '1.0')
from gi.repository import Gst

logger = logging.getLogger(__name__)


class GstRawSource(object):
    def __init__(self, name='raw', boot_time_seconds=20, fn_callback=None, command="videotestsrc ! decodebin ! videoconvert"):
        self.name = name
        self.boot_time_seconds = boot_time_seconds
        self.fn_callback = fn_callback
        Gst.init(None)
        if 'sink' in command:
            raise AssertionError("Command cannot contain a sink yet, must be added here.")
        self.command = command + " ! appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true"
        self.closed = True
        self._callback_time = None
        self.video_pipe = None

    def _setup(self):
        self.video_pipe = Gst.parse_launch(self.command)
        self.closed = True

    # noinspection PyUnusedLocal
    def _eos(self, bus, msg):
        logger.info(msg)
        self.close()

    # noinspection PyUnusedLocal
    def _error(self, bus, msg):
        logger.error(msg)
        self.close()

    def open(self):
        self._setup()
        self.video_pipe.set_state(Gst.State.PLAYING)
        video_sink = self.video_pipe.get_by_name('sink')
        video_sink.connect('new-sample', self._callback)
        bus = self.video_pipe.get_bus()
        bus.add_signal_watch()
        bus.connect('message::eos', self._eos)
        bus.connect('message::error', self._error)
        self.closed = False
        self._callback_time = time.time() + self.boot_time_seconds
        logger.info("Source {} opened.".format(self.name))

    def _callback(self, sink):
        sample = sink.emit('pull-sample')
        buf = sample.get_buffer()
        # caps = sample.get_caps()
        if self.fn_callback is not None:
            self.fn_callback(buf)
        self._callback_time = time.time()
        return Gst.FlowReturn.OK

    def is_healthy(self, patience):
        return self._callback_time and time.time() - self._callback_time < patience

    def is_closed(self):
        return self.closed

    def is_open(self):
        return not self.is_closed()

    def check(self, patience=0.50):
        if self.is_open() and not self.is_healthy(patience=patience):
            self.close()
        if self.is_closed():
            self.open()

    def close(self):
        if self.video_pipe is not None:
            self.video_pipe.set_state(Gst.State.NULL)
        self.closed = True
        logger.info("Source {} closed.".format(self.name))


class HttpLivePlayerVideoSocket(websocket.WebSocketHandler):
    def __init__(self, application, request, **kwargs):
        super(HttpLivePlayerVideoSocket, self).__init__(application, request, **kwargs)
        self._lock = threading.Lock()
        self._streaming = False

    # noinspection PyAttributeOutsideInit
    def initialize(self, **kwargs):
        self._video = kwargs.get('video_source')

    def _client(self, _bytes):
        with self._lock:
            if self._streaming:
                self.write_message(_bytes, binary=True)

    # noinspection PyUnusedLocal
    @staticmethod
    def check_origin(origin):
        return True

    def data_received(self, chunk):
        pass

    # noinspection PyUnusedLocal
    def open(self, *args, **kwargs):
        self._video.add_listener(self._client)
        self.write_message(json.dumps(dict(action='init', width=self._video.get_width(), height=self._video.get_height())))

    def on_close(self):
        self._video.remove_listener(self._client)

    def on_message(self, message):
        try:
            with self._lock:
                self._streaming = 'REQUESTSTREAM' in message
                logger.info("On message - streaming = {}.".format(self._streaming))
        except Exception as e:
            logger.error("Stream socket@on_message: {} {}".format(e, traceback.format_exc(e)))
            logger.error("Input message:---\n{}\n---".format(message))


class JMuxerVideoStreamSocket(websocket.WebSocketHandler):
    # noinspection PyAttributeOutsideInit
    def initialize(self, **kwargs):
        self._video = kwargs.get('video_source')

    def _client(self, _bytes):
        self.write_message(_bytes, binary=True)

    # noinspection PyUnusedLocal
    @staticmethod
    def check_origin(origin):
        return True

    def data_received(self, chunk):
        pass

    # noinspection PyUnusedLocal
    def open(self, *args, **kwargs):
        self._video.add_listener(self._client)

    def on_close(self):
        self._video.remove_listener(self._client)

    @staticmethod
    def on_message(message):
        logger.info("Unexpected message '{}' received.".format(message))
