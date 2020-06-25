#!/usr/bin/env python
import argparse
import glob
import logging
import multiprocessing
import os
import signal
from ConfigParser import SafeConfigParser
from functools import partial

from tornado import web, ioloop

from byodr.utils import timestamp
from byodr.utils.ipc import ReceiverThread, CameraThread, JSONPublisher, JSONZmqClient
from server import CameraMJPegSocket, ControlServerSocket, MessageServerSocket, ApiUserOptionsHandler, UserOptions, ApiSystemStateHandler

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


def on_options_save(publisher):
    publisher.publish(dict(time=timestamp(), command='restart'), topic='aav/teleop/chatter')


def list_process_start_messages(client):
    return client.call(dict(request='system/startup/list'))


def main():
    parser = argparse.ArgumentParser(description='Teleop sockets server.')
    parser.add_argument('--port', type=int, default=9100, help='Port number')
    parser.add_argument('--config', type=str, default='/config', help='Config directory path.')
    args = parser.parse_args()

    # Read the internal configuration first.
    parser = SafeConfigParser()
    parser.read('config.ini')
    # One user configuration file is optional and can be used to persist settings.
    _pattern = os.path.join(args.config, '*.ini')
    _candidates = glob.glob(_pattern)
    if len(_candidates) == 0:
        user_file = os.path.join(args.config, 'config.ini')
    else:
        user_file = _candidates[0]
        parser.read(user_file)
        if len(_candidates) > 1:
            logger.warning("Found {} files for '{}' but using only one.".format(len(_candidates), _pattern))
    # Convert for ease of use.
    cfg = dict(parser.items('teleop'))

    _display_speed_scale = float(cfg.get('display.speed.scale'))

    pilot = ReceiverThread(url='ipc:///byodr/pilot.sock', topic=b'aav/pilot/output', event=quit_event)
    vehicle = ReceiverThread(url='ipc:///byodr/vehicle.sock', topic=b'aav/vehicle/state', event=quit_event)
    inference = ReceiverThread(url='ipc:///byodr/inference.sock', topic=b'aav/inference/state', event=quit_event)
    recorder = ReceiverThread(url='ipc:///byodr/recorder.sock', topic=b'aav/recorder/state', event=quit_event)
    camera = CameraThread(url='ipc:///byodr/camera.sock', topic=b'aav/camera/0', event=quit_event)
    threads = [pilot, vehicle, inference, recorder, camera]
    if quit_event.is_set():
        return 0

    [t.start() for t in threads]

    publisher = JSONPublisher(url='ipc:///byodr/teleop.sock', topic='aav/teleop/input')
    zm_client = JSONZmqClient(urls=['ipc:///byodr/pilot_c.sock',
                                    'ipc:///byodr/inference_c.sock',
                                    'ipc:///byodr/vehicle_c.sock',
                                    'ipc:///byodr/recorder_c.sock'])
    user_options = UserOptions(user_file)
    try:
        web_app = web.Application([
            (r"/ws/ctl", ControlServerSocket,
             dict(fn_control=(lambda x: publisher.publish(x)))),
            (r"/ws/log", MessageServerSocket,
             dict(speed_scale=_display_speed_scale,
                  fn_state=(lambda: (pilot.get_latest(),
                                     vehicle.get_latest(),
                                     inference.get_latest(),
                                     recorder.get_latest())))),
            (r"/ws/cam", CameraMJPegSocket,
             dict(fn_capture=(lambda: camera.capture()[-1]))),
            (r"/api/user/options", ApiUserOptionsHandler,
             dict(user_options=user_options, fn_on_save=partial(on_options_save, publisher=publisher))),
            (r"/api/system/state", ApiSystemStateHandler,
             dict(fn_list_start_messages=partial(list_process_start_messages, client=zm_client))),
            (r"/(.*)", web.StaticFileHandler, {
                'path': os.path.join(os.path.sep, 'app', 'htm'),
                'default_filename': 'index.htm'
            })
        ])
        port = args.port
        web_app.listen(port)
        logger.info("Web service starting on port {}.".format(port))
        io_loop.start()
    except KeyboardInterrupt:
        quit_event.set()

    logger.info("Waiting on threads to stop.")
    [t.join() for t in threads]


if __name__ == "__main__":
    logging.basicConfig(format=log_format)
    logging.getLogger().setLevel(logging.INFO)
    main()
