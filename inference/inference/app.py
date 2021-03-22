from __future__ import absolute_import

import argparse
import glob
import logging
import os
import sys
import threading
import time
from functools import partial

import cv2
import numpy as np
# For operators see: https://github.com/glenfletcher/Equation/blob/master/Equation/equation_base.py
from Equation import Expression
from scipy.special import softmax
from six.moves import range

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


class RouteMemory(object):
    def __init__(self):
        self._recognition_threshold = 0
        self._num_points = 0
        self._num_codes = 0
        self._navigation_point = None
        self._beliefs = None
        self._evidence = None
        # Image id index to navigation point id.
        self._code_points = None
        # Image id index to features.
        self._code_book = None
        self._code_diagonal = None
        self._destination_keys = None
        self._destination_values = None

    def _evidence_reset(self):
        self._evidence = np.ones(self._num_codes, dtype=np.float32)

    def _belief_reset(self):
        self._beliefs = np.zeros(self._num_codes, dtype=np.float32)

    def set_threshold(self, value):
        self._recognition_threshold = value

    def reset(self, n_points=0, code_points=None, coordinates=None, keys=None, values=None):
        self._navigation_point = None
        self._num_points = n_points
        self._num_codes = 0 if code_points is None else len(code_points)
        self._code_points = None if code_points is None else np.array(code_points)
        self._code_book = None if coordinates is None else np.array(coordinates)
        self._code_diagonal = None if self._code_book is None else np.diag(np.dot(self._code_book, self._code_book.T))
        self._destination_keys = None if keys is None else np.array(keys)
        self._destination_values = None if values is None else np.array(values)
        self._evidence_reset()
        self._belief_reset()

    def is_open(self):
        return self._code_book is not None

    def match(self, features, query):
        code_points = self._code_points
        _uniform_prior = 1. / self._num_codes
        self._beliefs = np.maximum(_uniform_prior, self._beliefs)
        _before_match = self._navigation_point is None
        _point, _previous, _next = (-1, -1, -1) if _before_match else self._navigation_point
        _threshold = self._recognition_threshold

        # The beliefs incorporate local information through the network probabilities.
        _p_out = softmax(np.matmul(query.reshape([1, -1]), self._destination_keys.T)).flatten()
        _dot_product = np.dot(self._code_book, np.reshape(features, [1, -1]).T).flatten()
        _errors = (_dot_product - self._code_diagonal) ** 2
        _errors = np.minimum(1, _errors)
        self._beliefs = (self._beliefs * np.exp(-np.maximum(0, _errors)) * _p_out)
        self._evidence = np.minimum(self._evidence, _errors)

        # Select the destination from the next expected navigation point.
        _image = self._beliefs.argmax() if _before_match else np.where(code_points == _next, self._beliefs, -1).argmax()

        # Allow for a better match in case it is tracking the wrong image.
        _competitor = np.where(np.logical_or.reduce([code_points == _point, code_points == _previous]), -1, self._beliefs).argmax()

        _match = None
        if self._evidence[_image] < _threshold and _errors[_image] > np.power(self._evidence[_image], .75):
            _match = code_points[_image]
        elif self._evidence[_competitor] < _threshold and _errors[_competitor] > np.power(self._evidence[_competitor], .75):
            _image = _competitor
            _match = code_points[_competitor]

        if _match is not None and _point != _match:
            logger.info("Match {} error {:.2f} evidence {:.2f}".format(_match, _errors[_image], self._evidence[_image]))
            n_points = self._num_points
            self._navigation_point = _match, ((_match - 1) % n_points), ((_match + 1) % n_points)
            self._evidence_reset()

        _distance = _errors[_image]
        _destination = self._destination_values[_image]
        return _match, _image, _distance, _destination


class Navigator(object):
    def __init__(self, user_directory, internal_directory, routes_directory):
        self._model_directories = [user_directory, internal_directory]
        self._routes_directory = routes_directory
        self._lock = threading.Lock()
        self._quit_event = threading.Event()
        self._memory = RouteMemory()
        self._network = None
        self._store = None
        self._fn_dave_image = None
        self._fn_alex_image = None
        self._gumbel = None
        self._destination = None

    def _create_network(self, gpu_id=0, runtime_compilation=1):
        cache_directory, internal_directory = self._model_directories
        network = TRTDriver(cache_directory, internal_directory, gpu_id=gpu_id, runtime_compilation=runtime_compilation)
        return network

    def _pull_image_features(self, image):
        return self._network.features(dave_image=self._fn_dave_image(image, dtype=np.float32),
                                      alex_image=self._fn_alex_image(image, dtype=np.float32))

    def _route_open(self, route):
        # This may take a while.
        if not self._quit_event.is_set():
            with self._lock:
                if route != self._store.get_selected_route():
                    self._memory.reset()
                    self._gumbel = None
                    self._destination = None
                    self._store.open(route)
                    num_points = len(self._store)
                    if num_points > 0:
                        _images = self._store.list_all_images()
                        _codes, _coordinates, _keys, _values = [], [], [], []
                        for im_id in range(len(_images)):
                            _codes.append(self._store.get_image_navigation_point_id(im_id))
                            _c, _k, _v = self._pull_image_features(_images[im_id])
                            _coordinates.append(_c)
                            _keys.append(_k)
                            _values.append(_v)
                        self._memory.reset(num_points, _codes, _coordinates, _keys, _values)

    def _check_state(self, route=None):
        if route is None:
            self._store.close()
        elif route not in self._store.list_routes():
            threading.Thread(target=self._store.load_routes).start()
        elif route != self._store.get_selected_route():
            threading.Thread(target=self._route_open, args=(route,)).start()

    def recompile(self):
        with self._lock:
            if self._network is not None and self._network.will_compile():
                self._network.reactivate()

    def restart(self, fn_dave_image, fn_alex_image, recognition_threshold=0, gpu_id=0, runtime_compilation=1):
        self._quit_event.clear()
        with self._lock:
            _load_image = (lambda fname: self._fn_alex_image(cv2.imread(fname)))
            _store = FileSystemRouteDataSource(self._routes_directory, fn_load_image=_load_image, load_instructions=False)
            self._store = ReloadableDataSource(_store)
            self._fn_dave_image = fn_dave_image
            self._fn_alex_image = fn_alex_image
            if self._network is not None:
                self._network.deactivate()
            self._network = self._create_network(gpu_id, runtime_compilation)
            self._network.activate()
            self._store.load_routes()
            self._memory.reset()
            self._memory.set_threshold(recognition_threshold)
            self._gumbel = None
            self._destination = None

    def forward(self, image, route=None):
        # This runs at the service process frequency.
        self._check_state(route)
        _dave_img = self._fn_dave_image(image, dtype=np.float32)
        _alex_img = self._fn_alex_image(image, dtype=np.float32)
        _gumbel = self._gumbel
        _destination = self._destination
        _command = maneuver_intention() if _gumbel is None else _gumbel
        _out = self._network.forward(dave_image=_dave_img,
                                     alex_image=_alex_img,
                                     maneuver_command=_command,
                                     destination=_destination)
        action_out, critic_out, surprise_out, gumbel_out, brake_out, brake_critic_out, coord_out, query_out = _out

        nav_point_id, nav_image_id, nav_distance, _destination = None, None, None, None
        _acquired = self._lock.acquire(False)
        try:
            if _acquired and self._store.is_open() and self._memory.is_open():
                nav_point_id, nav_image_id, nav_distance, _destination = self._memory.match(coord_out, query_out)
        finally:
            if _acquired:
                self._lock.release()

        self._gumbel = None if _destination is None else gumbel_out
        self._destination = _destination

        return action_out, critic_out, surprise_out, brake_out, brake_critic_out, nav_point_id, nav_image_id, nav_distance, _command

    def quit(self):
        # Store and network are thread-safe.
        self._quit_event.set()
        if self._store is not None:
            self._store.quit()
        if self._network is not None:
            self._network.deactivate()


def _norm_scale(v, min_=0., max_=1.):
    """Zero values below the minimum but let values larger than the maximum be scaled up. """
    return abs(max(0., v - min_) / (max_ - min_))


class TFRunner(Configurable):
    def __init__(self, navigator):
        super(TFRunner, self).__init__()
        self._gpu_id = 0
        self._navigator = navigator
        self._process_frequency = 10
        self._steering_scale_left = 1
        self._steering_scale_right = 1
        self._penalty_filter = None
        self._debug_filter = None
        self._corridor_shift = 0
        self._obstruction_shift = 0
        self._fn_obstacle_norm = None
        self._fn_corridor_norm = None
        self._fn_corridor_penalty = None

    def get_gpu(self):
        return self._gpu_id

    def get_frequency(self):
        return self._process_frequency

    def recompile(self):
        self._navigator.recompile()

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

        self._corridor_shift = parse_option('driver.dnn.steer.corridor.shift', float, 0, _errors, **kwargs)
        self._obstruction_shift = parse_option('driver.dnn.obstacle.corridor.shift', float, 0, _errors, **kwargs)

        _brake_scale_max = parse_option('driver.dnn.obstacle.scale.max', float, 1e-6, _errors, **kwargs)

        _corridor_equation_key = 'driver.dnn.steer.corridor.equation'
        _corridor_penalty_eq = parse_option(_corridor_equation_key, str, "e ** (critic + surprise)", _errors, **kwargs)
        try:
            self._fn_corridor_penalty = Expression(_corridor_penalty_eq)
            self._fn_corridor_penalty(surprise=0, critic=0)
        except (TypeError, IndexError, ZeroDivisionError) as te:
            _errors.append(PropertyError(_corridor_equation_key, str(te)))
            self._fn_corridor_penalty = lambda surprise, critic: 100
        self._fn_obstacle_norm = partial(_norm_scale, min_=0, max_=_brake_scale_max)
        self._fn_corridor_norm = (lambda v: v)
        _fn_dave_image = get_registered_function('dnn.image.transform.dave', _errors, **kwargs)
        _fn_alex_image = get_registered_function('dnn.image.transform.alex', _errors, **kwargs)
        _nav_threshold = parse_option('navigator.point.recognition.threshold', float, 0, _errors, **kwargs)
        _rt_compile = parse_option('runtime.graph.compilation', int, 1, _errors, **kwargs)
        self._navigator.restart(fn_dave_image=_fn_dave_image,
                                fn_alex_image=_fn_alex_image,
                                recognition_threshold=_nav_threshold,
                                gpu_id=self._gpu_id,
                                runtime_compilation=_rt_compile)
        return _errors

    def _dnn_steering(self, raw):
        return raw * (self._steering_scale_left if raw < 0 else self._steering_scale_right)

    def forward(self, image, route=None):
        _out = self._navigator.forward(image, route)
        action_out, critic_out, surprise_out, brake_out, brake_critic_out, nav_point_id, nav_image_id, nav_distance, _command = _out

        _corridor_penalty = max(0,
                                self._fn_corridor_penalty(
                                    surprise=(max(0, self._fn_corridor_norm(surprise_out) + self._corridor_shift)),
                                    critic=(max(0, self._fn_corridor_norm(critic_out) + self._corridor_shift)))
                                )

        # Penalties to decrease desired speed.
        _obstacle_penalty = self._fn_obstacle_norm(brake_out) + max(0, brake_critic_out + self._obstruction_shift)
        _total_penalty = max(0, min(1, self._penalty_filter.calculate(_corridor_penalty + _obstacle_penalty)))

        _command_index = int(np.argmax(_command))

        return dict(time=timestamp(),
                    action=float(self._dnn_steering(action_out)),
                    corridor=float(self._debug_filter.calculate(_corridor_penalty)),
                    surprise_out=float(surprise_out),
                    critic_out=float(critic_out),
                    brake_critic_out=float(brake_critic_out),
                    dagger=int(0),
                    obstacle=float(brake_out),
                    penalty=float(_total_penalty),
                    internal=[float(0)],
                    navigation_point=int(-1 if nav_point_id is None else nav_point_id),
                    navigation_image=int(-1 if nav_image_id is None else nav_image_id),
                    navigation_distance=float(1 if nav_distance is None else nav_distance),
                    navigation_command=float(_command_index - 1) + _command[_command_index]
                    )


class InferenceApplication(Application):
    def __init__(self, runner=None, config_dir=os.getcwd(), internal_models=os.getcwd(), user_models=None, navigation_routes=None):
        super(InferenceApplication, self).__init__()
        self._config_dir = config_dir
        self._internal_models = internal_models
        self._user_models = user_models
        if user_models is not None and not os.path.exists(user_models):
            _mask = os.umask(000)
            os.makedirs(user_models, mode=0o775)
            os.umask(_mask)
        if runner is None:
            runner = TFRunner(navigator=Navigator(user_models, internal_models, navigation_routes))
        self._runner = runner
        self.publisher = None
        self.camera = None
        self.ipc_server = None
        self.teleop = None
        self.pilot = None
        self.ipc_chatter = None
        self._last_known_active_time = timestamp()

    @staticmethod
    def _glob(directory, pattern):
        return glob.glob(os.path.join(directory, pattern))

    def _config(self):
        parser = SafeConfigParser()
        # Inference overrides are only allowed internally - ignore user settings. Overrides come last.
        [parser.read(_f) for _f in ['config.ini'] + self._glob(self._internal_models, '*.ini')]
        cfg = dict(parser.items('inference'))
        logger.info(cfg)
        return cfg

    def _touch(self, c_pilot):
        # Keep track of the activity to have the network update in case it has a new model and the robot is not in use.
        if c_pilot is not None and (abs(c_pilot.get('steering', 0)) > 1e-3 or abs(c_pilot.get('throttle', 0)) > 1e-3):
            self._last_known_active_time = timestamp()

    def get_process_frequency(self):
        return self._runner.get_frequency()

    def recompile(self, seconds=300):
        _sleeping = (timestamp() - self._last_known_active_time) > seconds * 1e6
        if _sleeping:
            self._runner.recompile()

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
        self._touch(c_pilot)
        image = self.camera.capture()[-1]
        if image is not None:
            c_route = None if c_teleop is None else c_teleop.get('navigator').get('route')
            state = self._runner.forward(image=image, route=c_route)
            state['_fps'] = self.get_actual_hz()
            self.publisher.publish(state)
        chat = self.ipc_chatter()
        if chat is not None:
            if chat.get('command') == 'restart':
                self.setup()


class RecompilationThread(threading.Thread):
    def __init__(self, application, sleep_seconds=600):
        super(RecompilationThread, self).__init__()
        self._app = application
        self._sleep = sleep_seconds
        self._quit_event = threading.Event()

    def quit(self):
        self._quit_event.set()

    def is_running(self):
        return not self._quit_event.is_set()

    def run(self):
        while self.is_running():
            time.sleep(self._sleep)
            self._app.recompile()


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

    recompilation = RecompilationThread(application)

    threads = [collector, application.camera, application.ipc_server, recompilation]
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
