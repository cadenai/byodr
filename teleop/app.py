#!/usr/bin/env python
import argparse
import glob
import logging
import multiprocessing
import os
import signal
from ConfigParser import SafeConfigParser

from tornado import web, ioloop

from byodr.utils import Application
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


class TeleopApplication(Application):
    def __init__(self, event, config_dir=os.getcwd()):
        super(TeleopApplication, self).__init__(quit_event=event)
        self._config_dir = config_dir
        self._display_speed_scale = 0
        self._user_config_file = os.path.join(self._config_dir, 'config.ini')

    def _check_user_config(self):
        _candidates = glob.glob(os.path.join(self._config_dir, '*.ini'))
        if len(_candidates) > 0:
            self._user_config_file = _candidates[0]

    def _config(self):
        parser = SafeConfigParser()
        [parser.read(_f) for _f in ['config.ini'] + glob.glob(os.path.join(self._config_dir, '*.ini'))]
        return dict(parser.items('teleop'))

    def get_user_config_file(self):
        return self._user_config_file

    def get_display_speed_scale(self):
        return self._display_speed_scale

    def setup(self):
        if self.active():
            self._check_user_config()
            self._display_speed_scale = float(self._config().get('display.speed.scale'))
            self.logger.info("Speed scale = {}.".format(self._display_speed_scale))


def main():
    parser = argparse.ArgumentParser(description='Teleop sockets server.')
    parser.add_argument('--port', type=int, default=9100, help='Port number')
    parser.add_argument('--config', type=str, default='/config', help='Config directory path.')
    args = parser.parse_args()

    application = TeleopApplication(event=quit_event, config_dir=args.config)
    application.setup()

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

    def on_options_save():
        publisher.publish(dict(time=timestamp(), command='restart'), topic='aav/teleop/chatter')
        application.setup()

    def list_process_start_messages():
        return zm_client.call(dict(request='system/startup/list'))

    try:
        web_app = web.Application([
            (r"/ws/ctl", ControlServerSocket,
             dict(fn_control=(lambda x: publisher.publish(x)))),
            (r"/ws/log", MessageServerSocket,
             dict(speed_scale=application.get_display_speed_scale(),
                  fn_state=(lambda: (pilot.get_latest(),
                                     vehicle.get_latest(),
                                     inference.get_latest(),
                                     recorder.get_latest())))),
            (r"/ws/cam", CameraMJPegSocket, dict(fn_capture=(lambda: camera.capture()[-1]))),
            (r"/api/user/options", ApiUserOptionsHandler, dict(user_options=(UserOptions(application.get_user_config_file())),
                                                               fn_on_save=on_options_save)),
            (r"/api/system/state", ApiSystemStateHandler, dict(fn_list_start_messages=list_process_start_messages)),
            (r"/", web.RedirectHandler, dict(url='/index.htm?v=0.20.3', permanent=False)),
            (r"/(.*)", web.StaticFileHandler, {'path': os.path.join(os.path.sep, 'app', 'htm')})
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
