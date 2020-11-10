#!/usr/bin/env python
import argparse
import glob
import json
import logging
import multiprocessing
import os
import signal
import threading
from ConfigParser import SafeConfigParser

import cv2
import numpy as np
from tornado import web, ioloop

from byodr.utils import Application, hash_dict
from byodr.utils import timestamp
from byodr.utils.ipc import CameraThread, JSONPublisher, JSONZmqClient, JSONReceiver, CollectorThread
from byodr.utils.navigate import FileSystemRouteDataSource
from server import CameraMJPegSocket, ControlServerSocket, MessageServerSocket, ApiUserOptionsHandler, UserOptions, \
    JSONMethodDumpRequestHandler, NavImageHandler, JSONRequestHandler

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
        self._config_hash = -1
        self._lock = threading.Lock()

    def _check_user_config(self):
        _candidates = glob.glob(os.path.join(self._config_dir, '*.ini'))
        if len(_candidates) > 0:
            self._user_config_file = _candidates[0]

    def _config(self):
        parser = SafeConfigParser()
        [parser.read(_f) for _f in ['config.ini'] + glob.glob(os.path.join(self._config_dir, '*.ini'))]
        cfg = dict(parser.items('teleop'))
        return cfg

    def get_user_config_file(self):
        return self._user_config_file

    def get_display_speed_scale(self):
        with self._lock:
            return self._display_speed_scale

    def setup(self):
        if self.active():
            self._check_user_config()
            _config = self._config()
            _hash = hash_dict(**_config)
            if _hash != self._config_hash:
                self._config_hash = _hash
                _scale = float(_config.get('display.speed.scale'))
                self.logger.info("Speed scale = {}.".format(_scale))
                with self._lock:
                    self._display_speed_scale = _scale


def _load_nav_image(fname):
    image = cv2.imread(fname)
    image = cv2.resize(image, (160, 120))
    image = image.astype(np.uint8)
    return image


class Navigator(object):
    def __init__(self, route_store):
        self._store = route_store
        self._lock = threading.Lock()

    def reload(self):
        self._store.load_routes()

    def open_route(self, name):
        with self._lock:
            self._store.open(name)

    def get_navigation_image(self, image_id):
        image_id = -1 if image_id is None else image_id
        with self._lock:
            images = self._store.list_all_images()
            return images[image_id] if len(images) > image_id >= 0 else None

    def list_routes(self):
        return self._store.list_routes()


class NavigationHandler(JSONRequestHandler):
    # noinspection PyAttributeOutsideInit
    def initialize(self, **kwargs):
        self._navigator = kwargs.get('navigator')
        self.fn_publish = kwargs.get('fn_publish')

    def get(self):
        action = self.get_query_argument('action')
        if action == 'list':
            routes = self._navigator.list_routes()
            self.write(json.dumps(routes))
        else:
            self.write(json.dumps({}))

    def post(self):
        data = json.loads(self.request.body)
        action = data.get('action')
        selected_route = data.get('route')
        if action == 'start':
            self.fn_publish(dict(time=timestamp(), navigator={'action': 'start', 'route': selected_route}))
            threading.Thread(target=self._navigator.open_route, args=(selected_route,)).start()
        elif action == 'stop':
            self.fn_publish(dict(time=timestamp(), navigator={'action': 'stop'}))
        self.write(json.dumps(dict(message='ok')))


def main():
    parser = argparse.ArgumentParser(description='Teleop sockets server.')
    parser.add_argument('--port', type=int, default=9100, help='Port number')
    parser.add_argument('--config', type=str, default='/config', help='Config directory path.')
    parser.add_argument('--routes', type=str, default='/routes', help='Directory with the navigation routes.')
    args = parser.parse_args()

    route_store = FileSystemRouteDataSource(directory=args.routes, fn_load_image=_load_nav_image, load_instructions=False)
    navigator = Navigator(route_store)
    navigator.reload()

    application = TeleopApplication(event=quit_event, config_dir=args.config)
    application.setup()

    camera_front = CameraThread(url='ipc:///byodr/camera_0.sock', topic=b'aav/camera/0', event=quit_event)
    camera_rear = CameraThread(url='ipc:///byodr/camera_1.sock', topic=b'aav/camera/1', event=quit_event, receive_timeout_ms=100)
    pilot = JSONReceiver(url='ipc:///byodr/pilot.sock', topic=b'aav/pilot/output')
    vehicle = JSONReceiver(url='ipc:///byodr/vehicle.sock', topic=b'aav/vehicle/state')
    inference = JSONReceiver(url='ipc:///byodr/inference.sock', topic=b'aav/inference/state')
    recorder = JSONReceiver(url='ipc:///byodr/recorder.sock', topic=b'aav/recorder/state')
    collector = CollectorThread(receivers=(pilot, vehicle, inference, recorder), event=quit_event)

    threads = [camera_front, camera_rear, collector]
    if quit_event.is_set():
        return 0

    [t.start() for t in threads]

    publisher = JSONPublisher(url='ipc:///byodr/teleop.sock', topic='aav/teleop/input')
    chatter = JSONPublisher(url='ipc:///byodr/teleop_c.sock', topic='aav/teleop/chatter')
    zm_client = JSONZmqClient(urls=['ipc:///byodr/pilot_c.sock',
                                    'ipc:///byodr/inference_c.sock',
                                    'ipc:///byodr/vehicle_c.sock',
                                    'ipc:///byodr/relay_c.sock',
                                    'ipc:///byodr/recorder_c.sock'])

    def on_options_save():
        chatter.publish(dict(time=timestamp(), command='restart'))
        application.setup()

    def list_process_start_messages():
        return zm_client.call(dict(request='system/startup/list'))

    def list_service_capabilities():
        return zm_client.call(dict(request='system/service/capabilities'))

    def get_navigation_image():
        inf_state = collector.get(2)
        image_id = None if inf_state is None else inf_state.get('navigation_image')
        return navigator.get_navigation_image(image_id)

    try:
        web_app = web.Application([
            (r"/ws/ctl", ControlServerSocket,
             dict(fn_control=(lambda x: publisher.publish(x)))),
            (r"/ws/log", MessageServerSocket,
             dict(fn_speed_scale=(lambda: application.get_display_speed_scale()),
                  fn_state=(lambda: (collector.get(0),
                                     collector.get(1),
                                     collector.get(2),
                                     collector.get(3))))),
            (r"/ws/cam", CameraMJPegSocket, dict(capture_front=(lambda: camera_front.capture()[-1]),
                                                 capture_rear=(lambda: camera_rear.capture()[-1]))),
            (r'/ws/nav', NavImageHandler, dict(fn_get_image=(lambda: get_navigation_image()))),
            (r"/api/user/options", ApiUserOptionsHandler, dict(user_options=(UserOptions(application.get_user_config_file())),
                                                               fn_on_save=on_options_save)),
            (r"/api/system/state", JSONMethodDumpRequestHandler, dict(fn_method=list_process_start_messages)),
            (r"/api/system/capabilities", JSONMethodDumpRequestHandler, dict(fn_method=list_service_capabilities)),
            (r"/api/navigation/routes", NavigationHandler, dict(navigator=navigator, fn_publish=(lambda x: chatter.publish(x)))),
            (r"/", web.RedirectHandler, dict(url='/index.htm?v=0.40.3', permanent=False)),
            (r"/(.*)", web.StaticFileHandler, {'path': os.path.join(os.path.sep, 'app', 'htm')})
        ])
        port = args.port
        web_app.listen(port)
        logger.info("Web service starting on port {}.".format(port))
        io_loop.start()
    except KeyboardInterrupt:
        quit_event.set()

    route_store.close()

    logger.info("Waiting on threads to stop.")
    [t.join() for t in threads]


if __name__ == "__main__":
    logging.basicConfig(format=log_format)
    logging.getLogger().setLevel(logging.INFO)
    main()
