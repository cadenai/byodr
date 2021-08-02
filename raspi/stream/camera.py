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
from byodr.utils.video import create_video_source
from byodr.utils.websocket import HttpLivePlayerVideoSocket, JMuxerVideoStreamSocket

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


gst_commands = {
    'h264/rtsp':
        "rtspsrc location=rtsp://{user}:{password}@{ip}:{port}{path} latency=0 drop-on-latency=true do-retransmission=false ! "
        "queue ! rtph264depay ! h264parse ! queue ! video/x-h264,stream-format=\"byte-stream\" ! queue ! appsink",
    # 'raw/usb': "v4l2src device={uri} ! video/x-raw,width={width},height={height} ! queue ! "
    #            "videoconvert ! queue ! omxh264enc target-bitrate={bitrate} control-rate=1 ! "
    #            "queue ! video/x-h264,profile=baseline,stream-format=\"byte-stream\" ! queue ! appsink",
    # 'raw/usb/h264/tcp':
    #     "v4l2src device={uri} ! video/x-raw,width={src_width},height={src_height} ! tee name=t "
    #     "t. ! queue ! videoconvert ! videoscale ! video/x-raw,width={tcp_width},height={tcp_height},format=I420 ! "
    #     "queue ! rawvideoparse use-sink-caps=true ! jpegenc ! queue ! "
    #     "tcpserversink host=0.0.0.0 port={tcp_port} sync=false async=false "
    #     "t. ! queue ! videoconvert ! videoscale ! video/x-raw,width={out_width},height={out_height} ! queue ! "
    #     "omxh264enc target-bitrate={bitrate} control-rate=1 ! "
    #     "queue ! video/x-h264,profile=baseline,stream-format=\"byte-stream\" ! queue ! appsink",
    'raw/usb/h264/udp':
        "v4l2src device={uri} ! video/x-raw,width={src_width},height={src_height} ! tee name=t "
        "t. ! queue ! videoconvert ! videoscale ! video/x-raw,width={udp_width},height={udp_height} ! queue ! "
        "omxh264enc target-bitrate={udp_bitrate} control-rate=1 interval-intraframes=50 ! queue ! "
        "video/x-h264, profile=baseline ! rtph264pay ! udpsink host={udp_host} port={udp_port} sync=false async=false "
        "t. ! queue ! videoconvert ! videoscale ! video/x-raw,width={out_width},height={out_height} ! queue ! "
        "omxh264enc target-bitrate={out_bitrate} control-rate=1 interval-intraframes=50 ! queue ! "
        "video/x-h264,profile=baseline,stream-format=\"byte-stream\" ! queue ! appsink"
}


def create_stream(config_file):
    parser = SafeConfigParser()
    parser.read(config_file)
    kwargs = dict(parser.items('camera'))
    _errors = []
    name = os.path.basename(os.path.splitext(config_file)[0])
    _type = parse_option('camera.type', str, errors=_errors, **kwargs)
    assert _type in gst_commands.keys(), "Unrecognized camera type '{}'.".format(_type)
    if _type == 'h264/rtsp':
        out_width, out_height = [int(x) for x in parse_option('camera.output.shape', str, errors=_errors, **kwargs).split('x')]
        config = {
            'ip': (parse_option('camera.ip', str, errors=_errors, **kwargs)),
            'port': (parse_option('camera.port', int, errors=_errors, **kwargs)),
            'user': (parse_option('camera.user', str, errors=_errors, **kwargs)),
            'password': (parse_option('camera.password', str, errors=_errors, **kwargs)),
            'path': (parse_option('camera.path', str, errors=_errors, **kwargs))
        }
    else:
        _type = 'raw/usb/h264/udp'
        src_width, src_height = [int(x) for x in parse_option('camera.source.shape', str, errors=_errors, **kwargs).split('x')]
        udp_width, udp_height = [int(x) for x in parse_option('camera.udp.shape', str, errors=_errors, **kwargs).split('x')]
        out_width, out_height = [int(x) for x in parse_option('camera.output.shape', str, errors=_errors, **kwargs).split('x')]
        config = {
            'uri': (parse_option('camera.uri', str, errors=_errors, **kwargs)),
            'src_width': src_width,
            'src_height': src_height,
            'udp_width': udp_width,
            'udp_height': udp_height,
            'udp_bitrate': (parse_option('camera.udp.bitrate', int, errors=_errors, **kwargs)),
            'udp_host': (parse_option('camera.udp.host', str, errors=_errors, **kwargs)),
            'udp_port': (parse_option('camera.udp.port', int, errors=_errors, **kwargs)),
            'out_width': out_width,
            'out_height': out_height,
            'out_bitrate': (parse_option('camera.output.bitrate', int, errors=_errors, **kwargs))
        }
    _command = gst_commands.get(_type).format(**config)
    _socket_ref = parse_option('camera.output.class', str, errors=_errors, **kwargs)
    logger.info("Socket '{}' ref '{}' gst command={}".format(name, _socket_ref, _command))
    return create_video_source(name, shape=(out_height, out_width, 3), command=_command), _socket_ref


def main():
    parser = argparse.ArgumentParser(description='Camera web-socket server.')
    parser.add_argument('--config', type=str, default='/config/stream.ini', help='Configuration file.')
    parser.add_argument('--port', type=int, default=9101, help='Socket port.')
    args = parser.parse_args()

    config_file = args.config
    if os.path.exists(config_file) and os.path.isfile(config_file):
        video_stream, socket_type = create_stream(config_file)
        application = CameraApplication(stream=video_stream, event=quit_event)

        threads = [threading.Thread(target=application.run)]
        if quit_event.is_set():
            return 0

        [t.start() for t in threads]

        asyncio.set_event_loop_policy(AnyThreadEventLoopPolicy())
        asyncio.set_event_loop(asyncio.new_event_loop())

        io_loop = ioloop.IOLoop.instance()
        class_ref = HttpLivePlayerVideoSocket if socket_type == 'http-live' else JMuxerVideoStreamSocket
        web_app = web.Application([(r"/", class_ref, dict(video_source=video_stream, io_loop=io_loop))])
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
