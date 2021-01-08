from __future__ import absolute_import

import glob
import logging
import multiprocessing
import os
import time
from threading import Thread, Semaphore

import numpy as np
import tensorflow as tf
import tensorflow.contrib.tensorrt as trt
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
    input_dave = placeholder(dtype=tf.float32, shape=[1, 3, 66, 200], name='input/dave_image')
    input_alex = placeholder(dtype=tf.float32, shape=[1, 227, 227, 3], name='input/alex_image')
    maneuver_command = tf.placeholder(dtype=tf.float32, shape=[1, 4], name='input/maneuver_command')
    return input_dave, input_alex, maneuver_command


def _newest_file(paths, pattern):
    if isinstance(paths, tuple) or isinstance(paths, list):
        _result = None
        for path in paths:
            _result = _newest_file(path, pattern)
            if _result is not None:
                break
        return _result
    else:
        path = paths
        files = [] if path is None else glob.glob(os.path.join(os.path.expanduser(path), pattern))
        match = max(files, key=os.path.getmtime) if len(files) > 0 else None
        logger.info("Located file '{}'.".format(match))
        return match


def _load_definition(f_name):
    if f_name is None:
        return None
    graph_def = tf.GraphDef()
    with tf.gfile.GFile(f_name, 'rb') as f:
        graph_def.ParseFromString(f.read())
    return graph_def


def maneuver_index(turn='general.fallback'):
    _options = {'general.fallback': 0, 'intersection.left': 1, 'intersection.ahead': 2, 'intersection.right': 3}
    return _options[turn]


def maneuver_intention(turn='general.fallback', dtype=np.float32):
    command = np.zeros(4, dtype=dtype)
    command[maneuver_index(turn=turn)] = 1
    return command


def get_frozen_graph(_file):
    with tf.gfile.GFile(_file, "rb") as f:
        graph_def = tf.GraphDef()
        graph_def.ParseFromString(f.read())
    return graph_def


def image_standardization(img):
    # Mimic the tensorflow operation.
    # The op computes (x - mean) / adjusted_stddev, where mean is the average of all values in image,
    # and adjusted_stddev = max(stddev, 1.0 / sqrt(image.NumElements())).
    return (img - np.mean(img)) / max(np.std(img), (1. / np.sqrt(img.size)))


def l2_normalize(features):
    return features / np.sqrt(np.sum(features ** 2))


class TRTDriver(object):
    def __init__(self, model_directories, gpu_id=0):
        os.environ["CUDA_VISIBLE_DEVICES"] = "{}".format(gpu_id)
        _list = (isinstance(model_directories, tuple) or isinstance(model_directories, list))
        self.model_directories = model_directories if _list else [model_directories]
        self._lock = multiprocessing.Lock()
        self.input_dave = None
        self.input_alex = None
        self.tf_maneuver_cmd = None
        self.tf_steering = None
        self.tf_critic = None
        self.tf_surprise = None
        self.tf_brake = None
        self.tf_brake_critic = None
        self.tf_features1 = None
        self.tf_features2 = None
        self.tf_features3 = None
        self.sess = None
        self.graph_def = None
        self._fallback_intention = maneuver_intention()

    def _create_session(self, graph, barrier):
        with graph.as_default():
            config = tf.ConfigProto()
            config.gpu_options.allow_growth = True
            self.sess = tf.Session(config=config, graph=graph)
        barrier.wait()

    def _load_graph_def(self, f_definition, barrier):
        self.graph_def = _load_definition(f_definition)
        barrier.wait()

    def activate(self):
        with self._lock:
            _start = time.time()
            # Use the cached version of the most recent graph.
            f_optimized = _newest_file(self.model_directories, '*.optimized.pb')
            if f_optimized is None or not os.path.isfile(f_optimized):
                logger.warning("Cannot load from a missing graph.")
                return
            # The tensor runtime engine graph is device specific - if necessary remove the compiled engine and rebuild on-device.
            # The compile step may have previously been interrupted due to memory constraints resulting in an under optimized graph.
            # Determine whether to rebuild by file size ratio.
            f_runtime = os.path.splitext(os.path.splitext(f_optimized)[0])[0] + '.trt.pb'
            f_optimized_size = float(os.stat(f_optimized).st_size)
            f_runtime_size = float(os.stat(f_runtime).st_size) if os.path.isfile(f_runtime) else f_optimized_size
            _size_ratio = f_runtime_size / f_optimized_size
            if _size_ratio < 1.3:
                trt_graph = trt.create_inference_graph(
                    get_frozen_graph(f_optimized),
                    ['output/steer/steering',
                     'output/steer/critic',
                     'output/steer/surprise',
                     'output/speed/brake',
                     'output/speed/critic',
                     'output/posor/features1',
                     'output/posor/features2',
                     'output/posor/coordinates'],
                    max_batch_size=1,
                    is_dynamic_op=False,
                    precision_mode='FP32')
                with open(f_runtime, 'wb') as output_file:
                    output_file.write(trt_graph.SerializeToString())
            # Proceed to load the engine.
            graph = tf.Graph()
            _threads = []
            _barrier = Barrier(parties=2)
            _threads.append(Thread(target=self._create_session, args=(graph, _barrier)))
            _threads.append(Thread(target=self._load_graph_def, args=(f_runtime, _barrier,)))
            for thr in _threads:
                thr.start()
            for thr in _threads:
                thr.join(timeout=30)
            logger.info("Loaded '{}' in {:2.2f} seconds.".format(f_runtime, time.time() - _start))
            with graph.as_default():
                self.input_dave, self.input_alex, self.tf_maneuver_cmd = _create_input_nodes()
                _inputs = {
                    'input/dave_image': self.input_dave,
                    'input/alex_image': self.input_alex,
                    'input/maneuver_command': self.tf_maneuver_cmd
                }
                tf.import_graph_def(self.graph_def, input_map=_inputs, name='m')
                self.tf_steering = graph.get_tensor_by_name('m/output/steer/steering:0')
                self.tf_critic = graph.get_tensor_by_name('m/output/steer/critic:0')
                self.tf_surprise = graph.get_tensor_by_name('m/output/steer/surprise:0')
                self.tf_brake = graph.get_tensor_by_name('m/output/speed/brake:0')
                self.tf_brake_critic = graph.get_tensor_by_name('m/output/speed/critic:0')
                self.tf_features1 = graph.get_tensor_by_name('m/output/posor/features1:0')
                self.tf_features2 = graph.get_tensor_by_name('m/output/posor/features2:0')
                self.tf_features3 = graph.get_tensor_by_name('m/output/posor/coordinates:0')

    def deactivate(self):
        with self._lock:
            if self.sess is not None:
                self.sess.close()
                self.sess = None

    def forward(self, dave_image, alex_image, turn, fallback=False):
        with self._lock:
            assert self.sess is not None, "There is no session - run activation prior to calling this method."
            _ops = [self.tf_steering,
                    self.tf_critic,
                    self.tf_surprise,
                    self.tf_brake,
                    self.tf_brake_critic,
                    self.tf_features1,
                    self.tf_features2,
                    self.tf_features3
                    ]
            _ret = (0, 1, 1, 1, 1, [0], [0], [0])
            if None in _ops:
                return _ret
            with self.sess.graph.as_default():
                # Copy the trainer behavior.
                dave_image = image_standardization(dave_image / 255.)
                alex_image = image_standardization(alex_image)  # Not scaled since squeeze-net was trained like that.
                feeder = {
                    self.input_dave: [dave_image],
                    self.input_alex: [alex_image],
                    self.tf_maneuver_cmd: [self._fallback_intention if fallback else maneuver_intention(turn=turn)]
                }
                try:
                    _action, _critic, _surprise, _brake, _br_critic, _fe1, _fe2, _fe3 = self.sess.run(_ops, feed_dict=feeder)
                    _fe1 = l2_normalize(_fe1.flatten())
                    _fe2 = l2_normalize(_fe2.flatten())
                    _fe3 = l2_normalize(_fe3.flatten())
                    return _action[0][0], _critic[0][0], _surprise[0][0], max(0, _brake[0][0]), _br_critic[0][0], _fe1, _fe2, _fe3
                except (CancelledError, FailedPreconditionError) as e:
                    if isinstance(e, FailedPreconditionError):
                        logger.warning('FailedPreconditionError')
                    else:
                        logger.warning(e)
                    return _ret
