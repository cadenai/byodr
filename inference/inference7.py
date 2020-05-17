import glob
import logging
import multiprocessing
import os
import time
from threading import Thread, Semaphore

import numpy as np
import tensorflow as tf
from tensorflow.python.framework.errors_impl import CancelledError, FailedPreconditionError

logger = logging.getLogger(__name__)


class DynamicMomentum(object):
    """Low-pass filter with separate acceleration and deceleration momentum."""

    def __init__(self, up=.1, down=.9, ceiling=1.):
        self._previous_value = 0
        self._acceleration = up
        self._deceleration = down
        self._ceiling = ceiling

    def calculate(self, value):
        _momentum = self._acceleration if value > self._previous_value else self._deceleration
        _new_value = min(self._ceiling, _momentum * value + (1. - _momentum) * self._previous_value)
        self._previous_value = _new_value
        return _new_value


class Barrier(object):
    def __init__(self, parties):
        self.n = parties
        self.count = 0
        self.mutex = Semaphore(1)
        self.barrier = Semaphore(0)

    def wait(self):
        self.mutex.acquire()
        self.count = self.count + 1
        self.mutex.release()
        if self.count == self.n:
            self.barrier.release()
        self.barrier.acquire()
        self.barrier.release()


def _create_input_nodes():
    placeholder = tf.placeholder
    input_dave = placeholder(dtype=tf.uint8, shape=[None, 3, 66, 200], name='input/dave_image')
    input_alex = placeholder(dtype=tf.uint8, shape=[None, 227, 227, 3], name='input/alex_image')
    maneuver_command = tf.placeholder(dtype=tf.float32, shape=[None, 4], name='input/maneuver_command')
    fallback_command = tf.placeholder(dtype=tf.float32, shape=[None, 4], name='input/fallback_command')
    speed_command = tf.placeholder(dtype=tf.float32, shape=[None, 3], name='input/command')
    input_task = placeholder(dtype=tf.float32, shape=[None, 2], name='input/task')
    input_use_dropout = placeholder(tf.bool, shape=(), name='input/use_dropout')
    input_p_dropout = placeholder(tf.float32, shape=(), name='input/p_dropout')
    return (input_dave, maneuver_command, fallback_command, input_task, input_use_dropout, input_p_dropout), (input_alex, speed_command)


def _newest_file(path, pattern):
    files = [] if path is None else glob.glob(os.path.join(os.path.expanduser(path), pattern))
    match = max(files, key=os.path.getctime) if len(files) > 0 else None
    logger.info("Located file '{}' for directory '{}' and pattern '{}'.".format(match, path, pattern))
    return match


def _load_definition(f_name):
    if f_name is None:
        return None
    graph_def = tf.GraphDef()
    with tf.gfile.GFile(f_name, 'rb') as f:
        graph_def.ParseFromString(f.read())
    return graph_def


def _maneuver_intention(turn='general.fallback', dtype=np.float32):
    command = np.zeros(4, dtype=dtype)
    _options = {'general.fallback': 0, 'intersection.left': 1, 'intersection.ahead': 2, 'intersection.right': 3}
    command[_options[turn]] = 1
    return command


def _speed_intention(turn='intersection.ahead', dtype=np.float32):
    command = np.zeros(3, dtype=dtype)
    _options = {'intersection.left': 0, 'intersection.ahead': 1, 'intersection.right': 2}
    command[_options[turn]] = 1
    return command


class TFDriver(object):
    def __init__(self, model_directory, gpu_id=0, p_conv_dropout=0):
        os.environ["CUDA_VISIBLE_DEVICES"] = "{}".format(gpu_id)
        self.p_conv_dropout = p_conv_dropout
        self.model_directory = model_directory
        self._lock = multiprocessing.Lock()
        self.input_dave = None
        self.input_alex = None
        self.maneuver_cmd = None
        self.fallback_cmd = None
        self.speed_cmd = None
        self.input_task = None
        self.input_udr = None
        self.input_pdr = None
        self.p_steering = None
        self.p_critic = None
        self.p_surprise = None
        # self.f_steering = None
        # self.f_critic = None
        # self.f_surprise = None
        self.tf_entropy = None
        self.tf_brake = None
        self.sess = None
        self.maneuver_graph_def = None
        self.speed_graph_def = None

    def set_conv_dropout_probability(self, p):
        self.p_conv_dropout = p

    def _create_session(self, graph, barrier):
        with graph.as_default():
            config = tf.ConfigProto()
            config.gpu_options.allow_growth = True
            self.sess = tf.Session(config=config, graph=graph)
        barrier.wait()

    def _load_maneuver_graph_def(self, barrier):
        self.maneuver_graph_def = _load_definition(_newest_file(self.model_directory, 'steer*.pb'))
        barrier.wait()

    def _load_speed_graph_def(self, barrier):
        self.speed_graph_def = _load_definition(_newest_file(self.model_directory, 'speed*.pb'))
        barrier.wait()

    def activate(self):
        with self._lock:
            _start = time.time()
            graph = tf.Graph()
            _threads = []
            _barrier = Barrier(parties=3)
            _threads.append(Thread(target=self._create_session, args=(graph, _barrier)))
            _threads.append(Thread(target=self._load_maneuver_graph_def, args=(_barrier,)))
            _threads.append(Thread(target=self._load_speed_graph_def, args=(_barrier,)))
            for thr in _threads:
                thr.start()
            for thr in _threads:
                thr.join(timeout=30)
            logger.info("Loaded session and graph definitions in {:2.2f} seconds.".format(time.time() - _start))
            if None in (self.maneuver_graph_def, self.speed_graph_def):
                logger.warning("Cannot create the graph when one of the parts is missing.")
                return
            with graph.as_default():
                nodes_tuple = _create_input_nodes()
                self.input_dave, self.maneuver_cmd, self.fallback_cmd, self.input_task, self.input_udr, self.input_pdr = nodes_tuple[0]
                self.input_alex, self.speed_cmd = nodes_tuple[-1]
                _input_maneuver = {
                    'input/dave_image': self.input_dave,
                    'input/maneuver_command': self.maneuver_cmd,
                    # 'input/fallback_command': self.fallback_cmd,
                    'input/use_dropout': self.input_udr,
                    'input/p_dropout': self.input_pdr
                }
                _input_speed = {
                    'input/alex_image': self.input_alex,
                    'input/command': self.speed_cmd
                }
                tf.import_graph_def(self.maneuver_graph_def, input_map=_input_maneuver, name='fm')
                tf.import_graph_def(self.speed_graph_def, input_map=_input_speed, name='fs')
                self.p_steering = graph.get_tensor_by_name('fm/output/p_steering:0')
                self.p_critic = graph.get_tensor_by_name('fm/output/p_critic:0')
                self.p_surprise = graph.get_tensor_by_name('fm/output/p_surprise:0')
                # self.f_steering = graph.get_tensor_by_name('fm/output/f_steering:0')
                # self.f_critic = graph.get_tensor_by_name('fm/output/f_critic:0')
                # self.f_surprise = graph.get_tensor_by_name('fm/output/f_surprise:0')
                self.tf_entropy = graph.get_tensor_by_name('fm/output/entropy:0')
                self.tf_brake = graph.get_tensor_by_name('fs/output/brake:0')

    def deactivate(self):
        with self._lock:
            if self.sess is not None:
                self.sess.close()
                self.sess = None

    def forward(self, dave_image, alex_image, turn, dagger=False):
        with self._lock:
            assert self.sess is not None, "There is no session - run activation prior to calling this method."
            _ops = [self.p_steering,
                    self.p_critic,
                    self.p_surprise,
                    # self.f_steering,
                    # self.f_critic,
                    # self.f_surprise,
                    self.tf_brake,
                    self.tf_entropy]
            _ret = (0, 1, 1, 0, 1, 1, 1, [0, 0, 0])
            if None in _ops:
                return _ret
            with self.sess.graph.as_default():
                feeder = {
                    self.input_dave: [dave_image],
                    self.input_alex: [alex_image],
                    self.input_udr: dagger,
                    self.input_pdr: self.p_conv_dropout if dagger else 0,
                    self.maneuver_cmd: [_maneuver_intention(turn=turn)],
                    # self.fallback_cmd: [_maneuver_intention()],
                    self.speed_cmd: [_speed_intention(turn=turn)],
                    self.input_task: [[0, 0]]
                }
                try:
                    p_action, p_critic, p_surprise, _brake, _entropy = self.sess.run(_ops, feed_dict=feeder)
                    return p_action, p_critic, p_surprise, max(0, _brake), _entropy
                except (StandardError, CancelledError, FailedPreconditionError) as e:
                    if isinstance(e, FailedPreconditionError):
                        logger.warning('FailedPreconditionError')
                    else:
                        logger.warning(e)
                    return _ret
