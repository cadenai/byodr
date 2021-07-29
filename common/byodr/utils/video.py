from __future__ import absolute_import

import collections
import logging
import time

import gi
import numpy as np

gi.require_version('Gst', '1.0')
from gi.repository import Gst

Gst.init(None)

logger = logging.getLogger(__name__)


class RawGstSource(object):
    def __init__(self, name='app', boot_time_seconds=20, command="videotestsrc ! decodebin ! videoconvert"):
        if 'sink' in command:
            raise AssertionError("Command cannot contain a sink yet, must be added here.")
        self.name = name
        self.boot_time_seconds = boot_time_seconds
        self.command = command + " ! appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true"
        self._listeners = collections.deque()
        self._sample_time = None
        self.closed = True
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

    def _sample(self, sink):
        buffer = sink.emit('pull-sample').get_buffer()
        array = self.convert_buffer(buffer.extract_dup(0, buffer.get_size()))
        for listen in self._listeners:
            listen(array)
        self._sample_time = time.time()
        return Gst.FlowReturn.OK

    def convert_buffer(self, buffer):
        return buffer

    def add_listener(self, listener):
        self._listeners.append(listener)

    def remove_listener(self, listener):
        self._listeners.remove(listener)

    def open(self):
        self._setup()
        self.video_pipe.set_state(Gst.State.PLAYING)
        video_sink = self.video_pipe.get_by_name('sink')
        video_sink.connect('new-sample', self._sample)
        bus = self.video_pipe.get_bus()
        bus.add_signal_watch()
        bus.connect('message::eos', self._eos)
        bus.connect('message::error', self._error)
        self.closed = False
        self._sample_time = time.time() + self.boot_time_seconds
        logger.info("Source {} opened.".format(self.name))

    def is_healthy(self, patience):
        return self._sample_time and time.time() - self._sample_time < patience

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


class ImageGstSource(RawGstSource):
    def __init__(self, name, shape, command):
        super(ImageGstSource, self).__init__(name=name, command=command)
        self._shape = shape

    def convert_buffer(self, buffer):
        return np.fromstring(buffer, dtype=np.uint8).reshape(self._shape)


class VideoGstSource(RawGstSource):
    def __init__(self, name, shape, command):
        super(VideoGstSource, self).__init__(name=name, command=command)
        self._shape = shape

    def get_width(self):
        return self._shape[1]

    def get_height(self):
        return self._shape[0]

    def convert_buffer(self, buffer):
        return buffer


def create_rtsp_video_stream_source(name, uri, **kwargs):
    url = uri.format(**kwargs)
    width, height = [int(x) for x in kwargs['shape'].split('x')]
    command = "rtspsrc location={url} " \
              "latency=0 drop-on-latency=true do-retransmission=false ! queue ! " \
              "rtph264depay ! h264parse ! queue ! video/x-h264,stream-format=\"byte-stream\" ! queue". \
        format(**dict(url=url))
    logger.info("Gst rtsp '{}' url = {}.".format(name, url))
    return VideoGstSource(name, shape=(height, width, 3), command=command)


def create_rtsp_image_source(name, uri, **kwargs):
    url = uri.format(**kwargs)
    width, height = [int(x) for x in kwargs['shape'].split('x')]
    framerate = kwargs['framerate']
    command = "rtspsrc location={url} " \
              "latency=0 drop-on-latency=true do-retransmission=false ! queue ! " \
              "rtph264depay ! h264parse ! queue ! avdec_h264 ! videoconvert ! " \
              "videorate ! video/x-raw,framerate={framerate}/1 ! " \
              "videoscale ! video/x-raw,width={width},height={height},format=BGR ! queue". \
        format(**dict(url=url, height=height, width=width, framerate=framerate))
    logger.info("Gst rtsp '{}' url = {} image shape = {} and decode rate = {}.".format(name, url, (width, height), framerate))
    return ImageGstSource(name, shape=(height, width, 3), command=command)
