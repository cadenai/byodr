from __future__ import absolute_import

import argparse
import glob
import logging
import os
import sys
import threading
from functools import partial

import cv2
import numpy as np
# For operators see: https://github.com/glenfletcher/Equation/blob/master/Equation/equation_base.py
from Equation import Expression
from scipy.cluster.vq import vq

from byodr.utils import timestamp, Configurable, Application
from byodr.utils.ipc import CameraThread, JSONPublisher, LocalIPCServer, JSONReceiver, CollectorThread
from byodr.utils.navigate import FileSystemRouteDataSource, ReloadableDataSource
from byodr.utils.option import parse_option, PropertyError
from .image import get_registered_function
from .inference import DynamicMomentum, TRTDriver, maneuver_intention

if sys.version_info > (3,):
    from configparser import ConfigParser as SafeConfigParser
else:
    from six.moves.configparser import SafeConfigParser

logger = logging.getLogger(__name__)


def _norm_scale(v, min_=0., max_=1.):
    """Zero values below the minimum but let values larger than the maximum be scaled up. """
    return abs(max(0., v - min_) / (max_ - min_))


class Navigator(object):
    def __init__(self, model_directories, routes_directory):
        self._model_directories = model_directories
        self._routes_directory = routes_directory
        self._lock = threading.Lock()
        self._quit_event = threading.Event()
        self._eps = 1e-6
        self._fn_dave_image = None
        self._fn_alex_image = None
        self._network = None
        self._store = None
        self._nav_point_id = None
        self._code_points = None  # The index is the image id and the value is the navigation point id.
        self._code_book = None
        self._distances = None

    def _pull_image_features(self, image):
        return self._network.forward(dave_image=self._fn_dave_image(image, dtype=np.float32),
                                     alex_image=self._fn_alex_image(image, dtype=np.float32))[-2:]

    def _route_open(self, route):
        # This may take a while.
        if not self._quit_event.is_set():
            with self._lock:
                if route != self._store.get_selected_route():
                    self._nav_point_id = None
                    self._store.open(route)
                    if len(self._store) > 0:
                        _images = self._store.list_all_images()
                        _points, _features, _distances = [], [], []
                        for im_id in range(len(_images)):
                            _a, _b = self._pull_image_features(_images[im_id])
                            _points.append(self._store.get_image_navigation_point_id(im_id))
                            _features.append(_a)
                            _distances.append(_b)
                        self._code_points = np.array(_points)
                        self._code_book = np.array(_features)
                        self._distances = np.reshape(np.array(_distances), [-1, 8, 3])

    def _check_state(self, route=None):
        if route is None:
            self._store.close()
        elif route not in self._store.list_routes():
            threading.Thread(target=self._store.load_routes).start()
        elif route != self._store.get_selected_route():
            threading.Thread(target=self._route_open, args=(route,)).start()

    def restart(self, fn_dave_image, fn_alex_image, gpu_id=0):
        self._quit_event.clear()
        with self._lock:
            _load_image = (lambda fname: self._fn_alex_image(cv2.imread(fname)))
            _store = FileSystemRouteDataSource(self._routes_directory, fn_load_image=_load_image, load_instructions=False)
            self._fn_dave_image = fn_dave_image
            self._fn_alex_image = fn_alex_image
            self._store = ReloadableDataSource(_store)
            self._network = TRTDriver(self._model_directories, gpu_id=gpu_id)
            self._network.activate()
            self._store.load_routes()
            self._nav_point_id = None
            self._code_points = None
            self._code_book = None
            self._distances = None

    def forward(self, image, intention, route=None):
        # This runs at the service process frequency.
        self._check_state(route)
        _dave_img = self._fn_dave_image(image, dtype=np.float32)
        _alex_img = self._fn_alex_image(image, dtype=np.float32)
        _out = self._network.forward(dave_image=_dave_img,
                                     alex_image=_alex_img,
                                     maneuver_command=(maneuver_intention(intention)))
        action_out, critic_out, surprise_out, gumbel_out, brake_out, brake_critic_out, features_out, distance_out = _out
        nav_point_id, nav_image_id, nav_distance = None, None, None
        _acquired = self._lock.acquire(False)
        try:
            if _acquired and self._store.is_open() and self._code_book is not None:
                nav_point_id = self._nav_point_id
                distortions = vq(self._code_book, np.reshape(np.array([features_out]), [1, -1]))[-1]
                # Bring the expected navigation images closer.
                scaled = np.clip((distortions - np.std(distortions)) / (self._eps + np.mean(distortions)), 0, 1)
                # Determine the next navigation point based on the scaled distance in case we missed one.
                nav_image_id = np.argmin(scaled)
                nav_distance = scaled[nav_image_id]
                if nav_distance < 0.100:
                    nav_point_id = self._code_points[nav_image_id]
                    self._nav_point_id = nav_point_id
                # Navigate on the expected images and discard past navigation images.
                if nav_point_id is not None:
                    _cp = self._code_points
                    next_point_id = (nav_point_id + 1) % len(self._store)
                    _cross = np.reshape(np.cross(np.reshape(distance_out, [8, 3]), self._distances, axis=-1), [-1, 24])
                    signs = np.sign(np.sum(_cross, axis=-1))
                    mask = np.logical_and(signs > 0, np.logical_or(_cp == nav_point_id, _cp == next_point_id))
                    if any(mask):
                        expected = np.where(mask, scaled, 1)
                        nav_image_id = np.argmin(expected)
                        nav_distance = expected[nav_image_id]
                    else:
                        nav_image_id, nav_distance = None, None
        finally:
            if _acquired:
                self._lock.release()
        return action_out, critic_out, surprise_out, brake_out, brake_critic_out, nav_point_id, nav_image_id, nav_distance

    def quit(self):
        # Store and network are thread-safe.
        self._quit_event.set()
        if self._store is not None:
            self._store.quit()
        if self._network is not None:
            self._network.deactivate()


class TFRunner(Configurable):
    def __init__(self, model_directories, routes_directory=None):
        super(TFRunner, self).__init__()
        self._navigator = Navigator(model_directories, routes_directory)
        self._gpu_id = 0
        self._process_frequency = 10
        self._steering_scale_left = 1
        self._steering_scale_right = 1
        self._penalty_filter = None
        self._debug_filter = None
        self._fn_obstacle_norm = None
        self._fn_brake_critic_norm = None
        self._fn_corridor_norm = None
        self._fn_corridor_penalty = None

    def get_gpu(self):
        return self._gpu_id

    def get_frequency(self):
        return self._process_frequency

    def internal_quit(self, restarting=False):
        self._navigator.quit()

    def internal_start(self, **kwargs):
        _errors = []
        self._gpu_id = parse_option('gpu.id', int, 0, _errors, **kwargs)
        self._process_frequency = parse_option('clock.hz', int, 10, _errors, **kwargs)
        self._steering_scale_left = parse_option('driver.dnn.steering.scale.left', lambda x: abs(float(x)), 0, _errors, **kwargs)
        self._steering_scale_right = parse_option('driver.dnn.steering.scale.right', float, 0, _errors, **kwargs)
        _penalty_up_momentum = parse_option('driver.autopilot.filter.momentum.up', float, 0, _errors, **kwargs)
        _penalty_down_momentum = parse_option('driver.autopilot.filter.momentum.down', float, 0, _errors, **kwargs)
        _penalty_ceiling = parse_option('driver.autopilot.filter.ceiling', float, 0, _errors, **kwargs)
        self._penalty_filter = DynamicMomentum(up=_penalty_up_momentum, down=_penalty_down_momentum, ceiling=_penalty_ceiling)
        self._debug_filter = DynamicMomentum(up=_penalty_up_momentum, down=_penalty_down_momentum, ceiling=_penalty_ceiling)
        _brake_scale_max = parse_option('driver.dnn.obstacle.scale.max', float, 1e-6, _errors, **kwargs)
        _brake_critic_scale_max = parse_option('driver.dnn.brake.critic.scale.max', float, 1e-6, _errors, **kwargs)
        _corridor_equation_key = 'driver.dnn.steer.corridor.equation'
        _corridor_penalty_eq = parse_option(_corridor_equation_key, str, "e ** (critic + surprise)", _errors, **kwargs)
        try:
            self._fn_corridor_penalty = Expression(_corridor_penalty_eq)
            self._fn_corridor_penalty(surprise=0, critic=0)
        except (TypeError, IndexError, ZeroDivisionError) as te:
            _errors.append(PropertyError(_corridor_equation_key, str(te)))
            self._fn_corridor_penalty = lambda surprise, critic: 100
        self._fn_obstacle_norm = partial(_norm_scale, min_=0, max_=_brake_scale_max)
        self._fn_brake_critic_norm = partial(_norm_scale, min_=0, max_=_brake_critic_scale_max)
        self._fn_corridor_norm = (lambda v: v)
        _fn_dave_image = get_registered_function('dnn.image.transform.dave', _errors, **kwargs)
        _fn_alex_image = get_registered_function('dnn.image.transform.alex', _errors, **kwargs)
        self._navigator.restart(fn_dave_image=_fn_dave_image, fn_alex_image=_fn_alex_image, gpu_id=self._gpu_id)
        return _errors

    def _dnn_steering(self, raw):
        return raw * (self._steering_scale_left if raw < 0 else self._steering_scale_right)

    def forward(self, image, intention, route=None):
        _out = self._navigator.forward(image, intention, route)
        action_out, critic_out, surprise_out, brake_out, brake_critic_out, nav_point_id, nav_image_id, nav_distance = _out

        critic = self._fn_corridor_norm(critic_out)
        surprise = self._fn_corridor_norm(surprise_out)
        _corridor_penalty = max(0, self._fn_corridor_penalty(surprise=surprise, critic=critic))

        # Penalties to decrease desired speed.
        _obstacle_penalty = self._fn_obstacle_norm(brake_out) + self._fn_brake_critic_norm(brake_critic_out)
        _total_penalty = max(0, min(1, self._penalty_filter.calculate(_corridor_penalty + _obstacle_penalty)))

        return dict(time=timestamp(),
                    action=float(self._dnn_steering(action_out)),
                    corridor=float(self._debug_filter.calculate(_corridor_penalty)),
                    surprise_out=float(surprise_out),
                    critic_out=float(critic_out),
                    dagger=int(0),
                    obstacle=float(_obstacle_penalty),
                    penalty=float(_total_penalty),
                    internal=[float(0)],
                    navigation_point=int(-1 if nav_point_id is None else nav_point_id),
                    navigation_image=int(-1 if nav_image_id is None else nav_image_id),
                    navigation_distance=float(1 if nav_distance is None else nav_distance)
                    )


class InferenceApplication(Application):
    def __init__(self, runner=None, config_dir=os.getcwd(), internal_models=os.getcwd(), user_models=None, navigation_routes=None):
        super(InferenceApplication, self).__init__()
        self._config_dir = config_dir
        self._internal_models = internal_models
        self._user_models = user_models
        self._runner = TFRunner([user_models, internal_models], routes_directory=navigation_routes) if runner is None else runner
        self.publisher = None
        self.camera = None
        self.ipc_server = None
        self.teleop = None
        self.pilot = None
        self.ipc_chatter = None

    @staticmethod
    def _glob(directory, pattern):
        return glob.glob(os.path.join(directory, pattern))

    def _config(self):
        parser = SafeConfigParser()
        # The end-user config overrides come last so all settings are modifiable.
        [parser.read(_f) for _f in ['config.ini'] + self._glob(self._internal_models, '*.ini') + self._glob(self._config_dir, '*.ini')]
        cfg = dict(parser.items('inference'))
        return cfg

    def get_process_frequency(self):
        return self._runner.get_frequency()

    def setup(self):
        if self.active():
            _restarted = self._runner.restart(**self._config())
            if _restarted:
                self.ipc_server.register_start(self._runner.get_errors())
                _frequency = self._runner.get_frequency()
                self.set_hz(_frequency)
                self.logger.info("Processing at {} Hz on gpu {}.".format(_frequency, self._runner.get_gpu()))

    def finish(self):
        self._runner.quit()

    def step(self):
        # Leave the state as is on empty teleop state.
        c_teleop = self.teleop()
        c_pilot = self.pilot()
        image = self.camera.capture()[-1]
        if image is not None:
            instruction = 'intersection.ahead' if c_pilot is None else c_pilot.get('instruction')
            c_route = None if c_teleop is None else c_teleop.get('navigator').get('route')
            state = self._runner.forward(image=image, intention=instruction, route=c_route)
            state['_fps'] = self.get_actual_hz()
            self.publisher.publish(state)
        chat = self.ipc_chatter()
        if chat is not None:
            if chat.get('command') == 'restart':
                self.setup()


def main():
    parser = argparse.ArgumentParser(description='Inference server.')
    parser.add_argument('--config', type=str, default='/config', help='Config directory path.')
    parser.add_argument('--internal', type=str, default='/models', help='Directory with the default inference models.')
    parser.add_argument('--user', type=str, default='/user_models', help='Directory with the user inference models.')
    parser.add_argument('--routes', type=str, default='/routes', help='Directory with the navigation routes.')
    args = parser.parse_args()

    application = InferenceApplication(config_dir=args.config,
                                       internal_models=args.internal,
                                       user_models=args.user,
                                       navigation_routes=args.routes)
    quit_event = application.quit_event

    teleop = JSONReceiver(url='ipc:///byodr/teleop.sock', topic=b'aav/teleop/input')
    pilot = JSONReceiver(url='ipc:///byodr/pilot.sock', topic=b'aav/pilot/output')
    ipc_chatter = JSONReceiver(url='ipc:///byodr/teleop_c.sock', topic=b'aav/teleop/chatter', pop=True)
    collector = CollectorThread(receivers=(teleop, pilot, ipc_chatter), event=quit_event)

    application.publisher = JSONPublisher(url='ipc:///byodr/inference.sock', topic='aav/inference/state')
    application.camera = CameraThread(url='ipc:///byodr/camera_0.sock', topic=b'aav/camera/0', event=quit_event)
    application.ipc_server = LocalIPCServer(url='ipc:///byodr/inference_c.sock', name='inference', event=quit_event)
    application.teleop = lambda: collector.get(0)
    application.pilot = lambda: collector.get(1)
    application.ipc_chatter = lambda: collector.get(2)
    threads = [collector, application.camera, application.ipc_server]
    if quit_event.is_set():
        return 0

    [t.start() for t in threads]
    application.run()

    logger.info("Waiting on threads to stop.")
    [t.join() for t in threads]


if __name__ == "__main__":
    logging.basicConfig(format='%(levelname)s: %(filename)s %(funcName)s %(message)s')
    logging.getLogger().setLevel(logging.INFO)
    main()
