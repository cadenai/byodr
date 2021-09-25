import argparse
import glob
import logging
import multiprocessing
import os
import shutil
import signal
import threading

from ConfigParser import SafeConfigParser
from tornado import web, ioloop
from tornado.httpserver import HTTPServer
from vehicle import CarlaHandler
from video import NumpyImageVideoSource

from byodr.utils import Application
from byodr.utils import Configurable
from byodr.utils.ipc import JSONPublisher, ImagePublisher, LocalIPCServer, json_collector
from byodr.utils.option import parse_option
from byodr.utils.websocket import HttpLivePlayerVideoSocket

logger = logging.getLogger(__name__)

io_loop = ioloop.IOLoop.instance()
signal.signal(signal.SIGINT, lambda sig, frame: io_loop.add_callback_from_signal(_interrupt))
signal.signal(signal.SIGTERM, lambda sig, frame: io_loop.add_callback_from_signal(_interrupt))

quit_event = multiprocessing.Event()


def _interrupt():
    logger.info("Received interrupt, quitting.")
    quit_event.set()
    io_loop.stop()


class RoutingImagePublisher(object):
    def __init__(self, cameras, streams):
        self.front_camera = cameras[0]
        self.rear_camera = cameras[1]
        self.front_stream = streams[0]
        self.rear_stream = streams[1]

    def quit(self):
        self.front_stream.quit()
        self.rear_stream.quit()

    def restart(self, **kwargs):
        self.front_stream.restart(**kwargs)
        self.rear_stream.restart(**kwargs)
        return self.front_stream.get_errors() + self.rear_stream.get_errors()

    def publish(self, image, route=0):
        if route == 0:
            self.front_camera.publish(image)
            self.front_stream.push(image)
        else:
            self.rear_camera.publish(image)
            self.rear_stream.push(image)


class CarlaRunner(Configurable):
    def __init__(self, image_publisher):
        super(CarlaRunner, self).__init__()
        self._process_frequency = 10
        self._patience_micro = 1000.
        self._publisher = image_publisher
        self._vehicle = CarlaHandler((lambda img, route: image_publisher.publish(img, route=route)))

    def get_image_shape(self):
        return self._vehicle.get_image_shape()

    def get_process_frequency(self):
        with self._lock:
            return self._process_frequency

    def get_patience_micro(self):
        with self._lock:
            return self._patience_micro

    def internal_quit(self, restarting=False):
        if not restarting:
            self._vehicle.quit()
            self._publisher.quit()

    def internal_start(self, **kwargs):
        self._vehicle.restart(**kwargs)
        _errors = self._vehicle.get_errors()
        _errors += self._publisher.restart(**kwargs)
        self._process_frequency = parse_option('clock.hz', int, 100, _errors, **kwargs)
        self._patience_micro = parse_option('patience.ms', int, 200, _errors, **kwargs) * 1000.
        return _errors

    def reset_agent(self):
        self._vehicle.reset()

    def state(self):
        return self._vehicle.state()

    def noop(self):
        self._vehicle.noop()

    def drive(self, cmd):
        self._vehicle.drive(cmd)


class CarlaApplication(Application):
    def __init__(self, image_publisher, config_dir=os.getcwd()):
        super(CarlaApplication, self).__init__(quit_event=quit_event)
        self._runner = CarlaRunner(image_publisher)
        self._config_dir = config_dir
        self.publisher = None
        self.ipc_chatter = None
        self.pilot = None
        self.teleop = None
        self.ipc_server = None

    def _check_user_file(self):
        # One user configuration file is optional and can be used to persist settings.
        _candidates = glob.glob(os.path.join(self._config_dir, '*.ini'))
        if len(_candidates) == 0:
            shutil.copyfile('user.template.ini', os.path.join(self._config_dir, 'config.ini'))
            logger.info("Create a new user configuration file from template.")

    def _config(self):
        parser = SafeConfigParser()
        [parser.read(_f) for _f in glob.glob(os.path.join(self._config_dir, '*.ini'))]
        cfg = dict(parser.items('platform')) if parser.has_section('platform') else {}
        cfg.update(dict(parser.items('vehicle')) if parser.has_section('vehicle') else {})
        cfg.update(dict(parser.items('camera')) if parser.has_section('camera') else {})
        self.logger.info(cfg)
        return cfg

    @staticmethod
    def _capabilities():
        # The video dimensions are determined by the websocket services.
        return {
            'video': {
                'front': {'ptz': 0},
                'rear': {'ptz': 0}
            }
        }

    def setup(self):
        if self.active():
            self._check_user_file()
            _restarted = self._runner.restart(**self._config())
            if _restarted:
                self.ipc_server.register_start(self._runner.get_errors(), self._capabilities())
                _process_frequency = self._runner.get_process_frequency()
                _patience_micro = self._runner.get_patience_micro()
                self.set_hz(_process_frequency)
                self.logger.info("Processing at {} Hz and a patience of {} ms.".format(_process_frequency, _patience_micro / 1000))

    def finish(self):
        self._runner.quit()

    def step(self):
        runner, pilot, teleop, ipc_chatter, ipc_server = self._runner, self.pilot, self.teleop, self.ipc_chatter, self.ipc_server
        c_pilot = self._latest_or_none(pilot, patience=(runner.get_patience_micro()))
        c_teleop = self._latest_or_none(teleop, patience=(runner.get_patience_micro()))
        if c_teleop is not None and c_teleop.get('button_a', 0):
            runner.reset_agent()
        if c_pilot is not None:
            runner.drive(c_pilot)
        else:
            runner.noop()
        self.publisher.publish(runner.state())
        chat = self.ipc_chatter()
        if chat and chat.get('command') == 'restart':
            self.setup()


def main():
    parser = argparse.ArgumentParser(description='Carla vehicle client.')
    parser.add_argument('--name', type=str, default='none', help='Process name.')
    parser.add_argument('--config', type=str, default='/config', help='Config directory path.')
    args = parser.parse_args()

    front_camera = ImagePublisher(url='ipc:///byodr/camera_0.sock', topic='aav/camera/0')
    rear_camera = ImagePublisher(url='ipc:///byodr/camera_1.sock', topic='aav/camera/1')

    front_stream = NumpyImageVideoSource(name='front')
    rear_stream = NumpyImageVideoSource(name='rear')

    image_router = RoutingImagePublisher(cameras=(front_camera, rear_camera), streams=(front_stream, rear_stream))
    application = CarlaApplication(image_publisher=image_router, config_dir=args.config)

    pilot = json_collector(url='ipc:///byodr/pilot.sock', topic=b'aav/pilot/output', event=quit_event)
    teleop = json_collector(url='ipc:///byodr/teleop.sock', topic=b'aav/teleop/input', event=quit_event)
    ipc_chatter = json_collector(url='ipc:///byodr/teleop_c.sock', topic=b'aav/teleop/chatter', pop=True, event=quit_event)

    application.publisher = JSONPublisher(url='ipc:///byodr/vehicle.sock', topic='aav/vehicle/state')
    application.ipc_server = LocalIPCServer(url='ipc:///byodr/vehicle_c.sock', name='platform', event=quit_event)
    application.pilot = lambda: pilot.get()
    application.teleop = lambda: teleop.get()
    application.ipc_chatter = lambda: ipc_chatter.get()
    app_thread = threading.Thread(target=application.run)

    threads = [pilot, teleop, ipc_chatter, application.ipc_server, app_thread]
    if quit_event.is_set():
        return 0

    [t.start() for t in threads]

    try:
        class_ref = HttpLivePlayerVideoSocket
        front_app = web.Application([(r"/", class_ref, dict(video_source=front_stream, io_loop=io_loop))])
        front_server = HTTPServer(front_app, xheaders=True)
        front_server.bind(9101)
        front_server.start()
        rear_app = web.Application([(r"/", class_ref, dict(video_source=rear_stream, io_loop=io_loop))])
        rear_server = HTTPServer(rear_app, xheaders=True)
        rear_server.bind(9102)
        rear_server.start()
        logger.info("Video streams started on port 9101 and 9102.")
        io_loop.start()
    except KeyboardInterrupt:
        quit_event.set()

    logger.info("Waiting on threads to stop.")
    [t.join() for t in threads]


if __name__ == "__main__":
    logging.basicConfig(format='%(levelname)s: %(filename)s %(funcName)s %(message)s')
    logging.getLogger().setLevel(logging.INFO)
    main()
