from __future__ import absolute_import

import json
import logging
import threading
import traceback

import gi
from tornado import websocket

gi.require_version('Gst', '1.0')
from gi.repository import Gst

Gst.init(None)

logger = logging.getLogger(__name__)


class HttpLivePlayerVideoSocket(websocket.WebSocketHandler):
    def __init__(self, application, request, **kwargs):
        super(HttpLivePlayerVideoSocket, self).__init__(application, request, **kwargs)
        self._lock = threading.Lock()
        self._streaming = False

    # noinspection PyAttributeOutsideInit
    def initialize(self, **kwargs):
        self._video = kwargs.get('video_source')
        self._io_loop = kwargs.get('io_loop')

    def _push(self, _bytes):
        with self._lock:
            if self._streaming:
                self.write_message(_bytes, binary=True)

    def _client(self, _bytes):
        self._io_loop.add_callback(lambda: self._push(_bytes))

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
        self._io_loop = kwargs.get('io_loop')

    def _push(self, _bytes):
        self.write_message(_bytes, binary=True)

    def _client(self, _bytes):
        self._io_loop.add_callback(lambda: self._push(_bytes))

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
