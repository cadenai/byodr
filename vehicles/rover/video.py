import logging

import gi

gi.require_version('Gst', '1.0')
from gi.repository import Gst

logger = logging.getLogger(__name__)


class GstRawSource(object):
    def __init__(self, fn_callback=None, command="videotestsrc ! decodebin ! videoconvert"):
        self.fn_callback = fn_callback
        Gst.init(None)
        if 'sink' in command:
            raise AssertionError("Command cannot contain a sink yet, must be added here.")
        self.video_pipe = Gst.parse_launch(command + " ! appsink name=sink emit-signals=true sync=false max-buffers=2 drop=true")
        self.closed = True

    def _eos(self, bus, msg):
        logger.info(msg)
        self.close()

    def _error(self, bus, msg):
        logger.error(msg)
        self.close()

    def open(self):
        self.video_pipe.set_state(Gst.State.PLAYING)
        video_sink = self.video_pipe.get_by_name('sink')
        video_sink.connect('new-sample', self._callback)
        bus = self.video_pipe.get_bus()
        bus.add_signal_watch()
        bus.connect('message::eos', self._eos)
        bus.connect('message::error', self._error)
        self.closed = False
        logger.info("Source opened.")

    def _callback(self, sink):
        sample = sink.emit('pull-sample')
        buf = sample.get_buffer()
        # caps = sample.get_caps()
        if self.fn_callback is not None:
            self.fn_callback(buf)
        return Gst.FlowReturn.OK

    def is_closed(self):
        return self.closed

    def close(self):
        self.video_pipe.set_state(Gst.State.NULL)
        self.closed = True
        logger.info("Source closed.")
