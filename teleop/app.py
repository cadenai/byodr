#!/usr/bin/env python
import argparse
import glob
import logging
import multiprocessing
import os
import signal

import cv2
import numpy as np
from ConfigParser import SafeConfigParser
from server import CameraMJPegSocket, ControlServerSocket, MessageServerSocket, ApiUserOptionsHandler, UserOptions, \
    JSONMethodDumpRequestHandler, NavImageHandler, JSONNavigationHandler, SimpleRequestNavigationHandler
from tornado import web, ioloop
from tornado.httpserver import HTTPServer

from byodr.utils import Application, hash_dict
from byodr.utils import timestamp
from byodr.utils.ipc import CameraThread, JSONPublisher, JSONZmqClient, json_collector
from byodr.utils.navigate import FileSystemRouteDataSource, ReloadableDataSource

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


def _load_nav_image(fname):
    image = cv2.imread(fname)
    image = cv2.resize(image, (160, 120))
    image = image.astype(np.uint8)
    return image


class TeleopApplication(Application):
    def __init__(self, event, config_dir=os.getcwd()):
        super(TeleopApplication, self).__init__(quit_event=event)
        self._config_dir = config_dir
        self._user_config_file = os.path.join(self._config_dir, 'config.ini')
        self._config_hash = -1

    def _check_user_config(self):
        _candidates = glob.glob(os.path.join(self._config_dir, '*.ini'))
        if len(_candidates) > 0:
            self._user_config_file = _candidates[0]

    def _config(self):
        parser = SafeConfigParser()
        [parser.read(_f) for _f in glob.glob(os.path.join(self._config_dir, '*.ini'))]
        cfg = dict(parser.items('teleop')) if parser.has_section('teleop') else {}
        return cfg

    def get_user_config_file(self):
        return self._user_config_file

    def setup(self):
        if self.active():
            self._check_user_config()
            _config = self._config()
            _hash = hash_dict(**_config)
            if _hash != self._config_hash:
                self._config_hash = _hash


def main():
    parser = argparse.ArgumentParser(description='Teleop sockets server.')
    parser.add_argument('--name', type=str, default='none', help='Process name.')
    parser.add_argument('--config', type=str, default='/config', help='Config directory path.')
    parser.add_argument('--routes', type=str, default='/routes', help='Directory with the navigation routes.')
    args = parser.parse_args()

    route_store = ReloadableDataSource(
        FileSystemRouteDataSource(
            directory=args.routes,
            fn_load_image=_load_nav_image,
            load_instructions=False)
    )
    route_store.load_routes()

    application = TeleopApplication(event=quit_event, config_dir=args.config)
    application.setup()

    camera_front = CameraThread(url='ipc:///byodr/camera_0.sock', topic=b'aav/camera/0', event=quit_event)
    camera_rear = CameraThread(url='ipc:///byodr/camera_1.sock', topic=b'aav/camera/1', event=quit_event)
    pilot = json_collector(url='ipc:///byodr/pilot.sock', topic=b'aav/pilot/output', event=quit_event)
    vehicle = json_collector(url='ipc:///byodr/vehicle.sock', topic=b'aav/vehicle/state', event=quit_event)
    inference = json_collector(url='ipc:///byodr/inference.sock', topic=b'aav/inference/state', event=quit_event)
    recorder = json_collector(url='ipc:///byodr/recorder.sock', topic=b'aav/recorder/state', event=quit_event)

    threads = [camera_front, camera_rear, pilot, vehicle, inference, recorder]
    if quit_event.is_set():
        return 0

    [t.start() for t in threads]

    teleop_publisher = JSONPublisher(url='ipc:///byodr/teleop.sock', topic='aav/teleop/input')
    external_publisher = JSONPublisher(url='ipc:///byodr/external.sock', topic='aav/external/input')
    chatter = JSONPublisher(url='ipc:///byodr/teleop_c.sock', topic='aav/teleop/chatter')
    zm_client = JSONZmqClient(urls=['ipc:///byodr/pilot_c.sock',
                                    'ipc:///byodr/inference_c.sock',
                                    'ipc:///byodr/vehicle_c.sock',
                                    'ipc:///byodr/relay_c.sock',
                                    'ipc:///byodr/recorder_c.sock',
                                    'ipc:///byodr/camera_c.sock'])

    def on_options_save():
        chatter.publish(dict(time=timestamp(), command='restart'))
        application.setup()

    def list_process_start_messages():
        return zm_client.call(dict(request='system/startup/list'))

    def list_service_capabilities():
        return zm_client.call(dict(request='system/service/capabilities'))

    def get_navigation_image(image_id):
        return route_store.get_image(image_id)

    def teleop_publish(cmd):
        # We are the authority on route state.
        cmd['navigator'] = dict(route=route_store.get_selected_route())
        teleop_publisher.publish(cmd)

    def override_publish(nav_request):
        # We are the authority on route state.
        if nav_request is not None:
            external_publisher.publish(nav_request)

    try:
        main_redirect_url = '/index.htm?v=0.50.1'
        main_app = web.Application([
            (r"/ws/ctl", ControlServerSocket, dict(fn_control=teleop_publish)),
            (r"/ws/log", MessageServerSocket,
             dict(fn_state=(lambda: (pilot.get(),
                                     vehicle.get(),
                                     inference.get(),
                                     recorder.get())))),
            (r"/ws/cam", CameraMJPegSocket, dict(capture_front=(lambda: camera_front.capture()[-1]),
                                                 capture_rear=(lambda: camera_rear.capture()[-1]))),
            (r'/ws/nav', NavImageHandler, dict(fn_get_image=(lambda image_id: get_navigation_image(image_id)))),
            (r"/api/user/options", ApiUserOptionsHandler, dict(user_options=(UserOptions(application.get_user_config_file())),
                                                               fn_on_save=on_options_save)),
            (r"/api/system/state", JSONMethodDumpRequestHandler, dict(fn_method=list_process_start_messages)),
            (r"/api/system/capabilities", JSONMethodDumpRequestHandler, dict(fn_method=list_service_capabilities)),
            (r"/api/navigation/routes", JSONNavigationHandler, dict(route_store=route_store)),
            (r"/ext/v10/direct/navigate", SimpleRequestNavigationHandler, dict(route_store=route_store, fn_override=override_publish)),
            (r"/", web.RedirectHandler, dict(url=main_redirect_url, permanent=False)),
            (r"/(.*)", web.StaticFileHandler, {'path': os.path.join(os.path.sep, 'app', 'htm')})
        ])
        http_server = HTTPServer(main_app, xheaders=True)
        http_server.bind(8080)
        http_server.start()
        logger.info("Web services started on port 8080.")
        io_loop.start()
    except KeyboardInterrupt:
        quit_event.set()

    route_store.quit()

    logger.info("Waiting on threads to stop.")
    [t.join() for t in threads]


if __name__ == "__main__":
    logging.basicConfig(format=log_format)
    logging.getLogger().setLevel(logging.INFO)
    main()
