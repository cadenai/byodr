from __future__ import absolute_import

import glob
import logging
import multiprocessing
import os
import time
from io import open
from threading import Semaphore

import numpy as np
import tensorflow as tf
import tensorflow.contrib.tensorrt as trt

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
    input_dave = tf.placeholder(dtype=tf.float32, shape=[1, 66, 200, 3], name='input/dave_image')
    input_alex = tf.placeholder(dtype=tf.float32, shape=[1, 100, 200, 3], name='input/alex_image')
    input_command = tf.placeholder(dtype=tf.float32, shape=[1, 4], name='input/maneuver_command')
    # input_source = tf.placeholder(dtype=tf.float32, shape=[1, 100], name='input/current_source')
    input_destination = tf.placeholder(dtype=tf.float32, shape=[1, 150], name='input/current_destination')
    return input_dave, input_alex, input_command, input_destination


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


class TRTDriver(object):
    def __init__(self, cache_directory, internal_directory, gpu_id=0):
        os.environ["CUDA_VISIBLE_DEVICES"] = "{}".format(gpu_id)
        self._gpu_id = gpu_id
        self.model_directories = [cache_directory, internal_directory]
        self._lock = multiprocessing.Lock()
        self._zero_vector = np.zeros(shape=(150,), dtype=np.float32)
        self.input_dave = None
        self.input_alex = None
        self.input_command = None
        self.input_destination = None
        self.tf_steering = None
        self.tf_critic = None
        self.tf_surprise = None
        self.tf_gumbel = None
        self.tf_brake = None
        self.tf_brake_critic = None
        self.tf_coordinate = None
        self.tf_query = None
        self.tf_key = None
        self.tf_value = None
        self.sess = None

    def _compile(self):
        # Find the most recent optimized graph to compile.
        f_optimized = _newest_file(self.model_directories, 'runtime*.optimized.pb')
        if f_optimized is None or not os.path.isfile(f_optimized):
            logger.warning("Missing optimized graph.")
            return

        # The tensor runtime engine graph is device specific - if necessary remove the compiled engine and rebuild on-device.
        # The compile step may have previously been interrupted due to memory constraints resulting in an under optimized graph.
        # Use the internal directory to store the compiled graphs.
        # The compilation is gpu specific.
        # The engine plan file is generated on an incompatible device, expecting compute x.x got compute y.y, please rebuild.)
        file_name, dir_name = '{}_{}'.format(self._gpu_id, os.path.basename(f_optimized)), self.model_directories[-1]
        f_runtime = os.path.join(dir_name, os.path.splitext(os.path.splitext(file_name)[0])[0] + '.trt.pb')
        f_optimized_size = float(os.stat(f_optimized).st_size)
        f_runtime_size = float(os.stat(f_runtime).st_size) if os.path.isfile(f_runtime) else f_optimized_size
        # Determine whether to rebuild by file size ratio.
        _size_ratio = f_runtime_size / f_optimized_size
        if _size_ratio < 1.5:
            trt_graph = trt.create_inference_graph(
                get_frozen_graph(f_optimized),
                ['output/steer/steering',
                 'output/steer/critic',
                 'output/steer/surprise',
                 'output/steer/gumbel',
                 'output/speed/brake',
                 'output/speed/brake_critic',
                 'output/posor/coordinate',
                 'output/posor/query',
                 'output/posor/key',
                 'output/posor/value'
                 ],
                max_batch_size=1,
                is_dynamic_op=False,
                precision_mode='FP16',
                minimum_segment_size=5
            )
            with open(f_runtime, 'wb') as output_file:
                output_file.write(trt_graph.SerializeToString())
        return f_runtime

    def activate(self):
        with self._lock:
            _start = time.time()
            graph = tf.Graph()
            with graph.as_default():
                config = tf.ConfigProto()
                # config.gpu_options.allow_growth = True
                config.gpu_options.per_process_gpu_memory_fraction = 0.125
                self.sess = tf.Session(config=config, graph=graph)
                # Compile after session creation for tf memory management.
                f_runtime = self._compile()
                _nodes = _create_input_nodes()
                self.input_dave, self.input_alex, self.input_command, self.input_destination = _nodes
                _inputs = {
                    'input/dave_image': self.input_dave,
                    'input/alex_image': self.input_alex,
                    'input/maneuver_command': self.input_command,
                    'input/current_destination': self.input_destination
                }
                tf.import_graph_def(_load_definition(f_runtime), input_map=_inputs, name='m')
                logger.info("Loaded '{}' in {:2.2f} seconds.".format(f_runtime, time.time() - _start))
                self.tf_steering = graph.get_tensor_by_name('m/output/steer/steering:0')
                self.tf_critic = graph.get_tensor_by_name('m/output/steer/critic:0')
                self.tf_surprise = graph.get_tensor_by_name('m/output/steer/surprise:0')
                self.tf_gumbel = graph.get_tensor_by_name('m/output/steer/gumbel:0')
                self.tf_brake = graph.get_tensor_by_name('m/output/speed/brake:0')
                self.tf_brake_critic = graph.get_tensor_by_name('m/output/speed/brake_critic:0')
                self.tf_coordinate = graph.get_tensor_by_name('m/output/posor/coordinate:0')
                self.tf_query = graph.get_tensor_by_name('m/output/posor/query:0')
                self.tf_key = graph.get_tensor_by_name('m/output/posor/key:0')
                self.tf_value = graph.get_tensor_by_name('m/output/posor/value:0')

    def deactivate(self):
        with self._lock:
            if self.sess is not None:
                self.sess.close()
                self.sess = None

    def features(self, dave_image, alex_image):
        with self._lock:
            assert self.sess is not None, "There is no session - run activation prior to calling this method."
            _ops = [self.tf_coordinate, self.tf_key, self.tf_value]
            with self.sess.graph.as_default():
                # Copy the trainer behavior.
                dave_image = image_standardization(dave_image / 255.)
                alex_image = image_standardization(alex_image / 255.)
                feed = {
                    self.input_dave: [dave_image],
                    self.input_alex: [alex_image],
                    self.input_command: [maneuver_intention()],
                    self.input_destination: [self._zero_vector]
                }
                _out = [x.flatten() for x in self.sess.run(_ops, feed_dict=feed)]
                return _out

    def forward(self, dave_image, alex_image, maneuver_command=maneuver_intention(), destination=None):
        with self._lock:
            assert self.sess is not None, "There is no session - run activation prior to calling this method."
            _ops = [self.tf_steering,
                    self.tf_critic,
                    self.tf_surprise,
                    self.tf_gumbel,
                    self.tf_brake,
                    self.tf_brake_critic,
                    self.tf_coordinate,
                    self.tf_query
                    ]
            destination = self._zero_vector if destination is None else destination
            with self.sess.graph.as_default():
                # Copy the trainer behavior.
                dave_image = image_standardization(dave_image / 255.)
                alex_image = image_standardization(alex_image / 255.)
                feed = {
                    self.input_dave: [dave_image],
                    self.input_alex: [alex_image],
                    self.input_command: [maneuver_command],
                    self.input_destination: [destination]
                }
                _out = [x.flatten() for x in self.sess.run(_ops, feed_dict=feed)]
                _action, _critic, _surprise, _gumbel, _brake, _br_critic, _coord, _query = _out
                _gumbel = np.insert(_gumbel, 0, values=0, axis=0)
                return _action, _critic, _surprise, _gumbel, _brake, _br_critic, _coord, _query
