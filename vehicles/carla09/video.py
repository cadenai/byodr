#!/usr/bin/env python
import collections
import logging

import gi

from byodr.utils import Configurable
from byodr.utils.option import parse_option

gi.require_version('Gst', '1.0')

from gi.repository import GObject, Gst

GObject.threads_init()
Gst.init(None)

logger = logging.getLogger(__name__)


# noinspection PyUnusedLocal
def bus_message(bus, message, loop):
    m_type = message.type
    """
        Gstreamer Message Types and how to parse
        https://lazka.github.io/pgi-docs/Gst-1.0/flags.html#Gst.MessageType
    """
    if m_type == Gst.MessageType.EOS:
        logger.info("End of stream")
        loop.quit()
    elif m_type == Gst.MessageType.ERROR:
        err, debug = message.parse_error()
        logger.error("{} {}".format(err, debug))
        loop.quit()
    elif m_type == Gst.MessageType.WARNING:
        err, debug = message.parse_warning()
        logger.warning("{} {}".format(err, debug))
    return True


def parse_width_height(key, errors, **kwargs):
    return [int(x) for x in parse_option(key, str, errors=errors, **kwargs).split('x')]


class NumpyImageVideoSource(Configurable):
    def __init__(self, name='front'):
        super(NumpyImageVideoSource, self).__init__()
        self._stream_width, self._stream_height = 640, 480
        self._name = name
        self._listeners = collections.deque()
        self._pipeline = None
        self._source = None
        self._caps = None

    def get_width(self):
        # The lock is used at (re)configuration.
        with self._lock:
            return self._stream_width

    def get_height(self):
        # The lock is used at (re)configuration.
        with self._lock:
            return self._stream_height

    def add_listener(self, listener):
        self._listeners.append(listener)

    def remove_listener(self, listener):
        self._listeners.remove(listener)

    def _publish(self, sink):
        buffer = sink.emit('pull-sample').get_buffer()
        array = buffer.extract_dup(0, buffer.get_size())
        for listen in self._listeners:
            listen(array)
        return Gst.FlowReturn.OK

    def push(self, image):
        with self._lock:
            if self._source is not None:
                buffer = Gst.Buffer.new_wrapped(image.tobytes())
                sample = Gst.Sample.new(buffer, self._caps, None, None)
                self._source.emit('push-sample', sample)

    def _close(self):
        if self._source is not None:
            self._source.emit("end-of-stream")
        if self._pipeline is not None:
            self._pipeline.set_state(Gst.State.NULL)
        self._pipeline = None
        self._source = None
        self._caps = None

    def internal_quit(self, restarting=False):
        # This method runs under lock.
        self._close()

    def internal_start(self, **kwargs):
        # This method runs under lock.
        self._close()
        _errors = []
        input_width, input_height = parse_width_height('camera.image.input.shape', _errors, **kwargs)
        self._stream_width, self._stream_height = parse_width_height(self._name + '.video.output.shape', _errors, **kwargs)
        if len(_errors) == 0:
            _args = dict(input_width=input_width,
                         input_height=input_height,
                         stream_width=self._stream_width,
                         stream_height=self._stream_height)
            command = "appsrc name=source emit-signals=True is-live=True " \
                      "caps=video/x-raw,format=BGR,width={input_width},height={input_height} ! " \
                      "videoconvert ! queue ! " \
                      "videoscale ! video/x-raw,width={stream_width},height={stream_height} ! " \
                      "queue ! x264enc speed-preset=ultrafast tune=zerolatency ! " \
                      "video/x-h264,profile=baseline,stream-format=\"byte-stream\" ! queue ! " \
                      "appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true".format(**_args)
            pipeline = Gst.parse_launch(command)
            loop = GObject.MainLoop()
            bus = pipeline.get_bus()
            bus.add_signal_watch()
            bus.connect("message", bus_message, loop)
            src = pipeline.get_by_name('source')
            # src.set_property("format", Gst.Format.TIME)
            # src.set_property("block", True)
            src.set_property('format', 'time')
            src.set_property('do-timestamp', True)
            _caps = "video/x-raw,format=BGR,width={input_width},height={input_height},framerate={fps}/1".format(
                **dict(input_width=input_width, input_height=input_height, fps=30)
            )
            video_sink = pipeline.get_by_name('sink')
            video_sink.connect('new-sample', self._publish)
            pipeline.set_state(Gst.State.PLAYING)
            self._caps = Gst.Caps.from_string(_caps)
            self._pipeline = pipeline
            self._source = src
            logger.info("Setup video stream '{}' with input caps '{}'.".format(self._name, _caps))
        return _errors
