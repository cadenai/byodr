import logging
import time

import gi

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

    def _eos(self, bus, msg):
        logger.info(msg)
        self.close()

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
