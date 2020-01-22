import argparse
import glob
import json
import logging
import multiprocessing
import os
import sys
import time
from threading import Thread, Semaphore

import cv2
import numpy as np
import rospy
import tensorflow as tf
from std_msgs.msg import String as RosString
from tensorflow.python.framework.errors_impl import CancelledError, FailedPreconditionError

logger = logging.getLogger(__name__)
log_format = '%(levelname)s: %(filename)s %(funcName)s %(message)s'

quit_event = multiprocessing.Event()

_optimization_transforms = [
    # 'add_default_attributes',
    # 'remove_nodes(op=Identity)', We currently use identity nodes in the runtime graph.
    'merge_duplicate_nodes',
    'strip_unused_nodes',
    'fold_constants(ignore_errors=true)',
    'fold_batch_norms',
    'fold_old_batch_norms'
    # 'obfuscate_names',
    # 'round_weights(num_steps=256)'
    # 'quantize_nodes',  Current implementation only supports equal length strides in the row and column dimensions.
    # 'quantize_weights'  Worse inference performance on jetson.
    # 'sort_by_execution_order'
]


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
    input_dave_image = tf.placeholder(dtype=tf.uint8, shape=[None, 3, 66, 200], name='input/dave_image')
    input_speed_image = tf.placeholder(dtype=tf.uint8, shape=[None, 227, 227, 3], name='input/alex_image')
    input_command = tf.placeholder(dtype=tf.float32, shape=[None, 3], name='input/command')
    input_use_dropout = tf.placeholder(tf.bool, shape=(), name='input/use_dropout')
    input_p_dropout = tf.placeholder(tf.float32, shape=(), name='input/p_dropout')
    return (input_dave_image, input_command, input_use_dropout, input_p_dropout), (input_speed_image, input_command)


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


def drive_cmd_vector(turn='intersection.ahead', dtype=np.float32):
    command = np.zeros(3, dtype=dtype)
    _maneuvers = {'intersection.left': 0, 'intersection.ahead': 1, 'intersection.right': 2}
    command[_maneuvers[turn]] = 1
    return command


class TFDriver(object):
    def __init__(self, gpu_id=0, p_conv_dropout=0):
        os.environ["CUDA_VISIBLE_DEVICES"] = "{}".format(gpu_id)
        self.p_conv_dropout = p_conv_dropout
        self.maneuver_env = "CADEN_DNN_STEER" if os.environ.get("CADEN_DNN", None) is None else "CADEN_DNN"
        self.speed_env = "CADEN_DNN_SPEED" if os.environ.get("CADEN_DNN", None) is None else "CADEN_DNN"
        self.posor_env = "CADEN_DNN_NAV" if os.environ.get("CADEN_DNN", None) is None else "CADEN_DNN"
        self._lock = multiprocessing.Lock()
        self.input_dave_image = None
        self.input_alex_image = None
        self.input_posor_image = None
        self.input_command = None
        self.input_use_dropout = None
        self.input_p_dropout = None
        self.tf_steering = None
        self.tf_surprise = None
        self.tf_critic = None
        self.tf_entropy = None
        self.tf_brake = None
        self.tf_features = None
        self.sess = None
        self.maneuver_graph_def = None
        self.speed_graph_def = None
        self.posor_graph_def = None

    def set_conv_dropout_probability(self, p):
        self.p_conv_dropout = p

    def _create_session(self, graph, barrier):
        with graph.as_default():
            config = tf.ConfigProto()
            config.gpu_options.allow_growth = True
            self.sess = tf.Session(config=config, graph=graph)
        barrier.wait()

    def _load_maneuver_graph_def(self, barrier):
        self.maneuver_graph_def = _load_definition(_newest_file(os.environ.get(self.maneuver_env, None), 'steer*.pb'))
        barrier.wait()

    def _load_speed_graph_def(self, barrier):
        self.speed_graph_def = _load_definition(_newest_file(os.environ.get(self.speed_env, None), 'speed*.pb'))
        barrier.wait()

    def _load_posor_graph_def(self, barrier):
        self.posor_graph_def = _load_definition(_newest_file(os.environ.get(self.posor_env, None), 'posor*.pb'))
        barrier.wait()

    def activate(self):
        with self._lock:
            _start = time.time()
            graph = tf.Graph()
            _threads = []
            _barrier = Barrier(parties=4)
            _threads.append(Thread(target=self._create_session, args=(graph, _barrier)))
            _threads.append(Thread(target=self._load_maneuver_graph_def, args=(_barrier,)))
            _threads.append(Thread(target=self._load_speed_graph_def, args=(_barrier,)))
            _threads.append(Thread(target=self._load_posor_graph_def, args=(_barrier,)))
            for thr in _threads:
                thr.start()
            for thr in _threads:
                thr.join(timeout=5)
            logger.info("Loaded session and graph definitions in {:2.2f} seconds.".format(time.time() - _start))
            if None in (self.maneuver_graph_def, self.speed_graph_def, self.posor_graph_def):
                logger.warning("Cannot create the graph when one of the parts is missing.")
                return
            with graph.as_default():
                nodes_tuple = _create_input_nodes()
                self.input_dave_image, _, self.input_use_dropout, self.input_p_dropout = nodes_tuple[0]
                self.input_alex_image, self.input_command = nodes_tuple[-1]
                self.input_posor_image = tf.placeholder(dtype=tf.uint8, shape=[1, 227, 227, 3])
                _input_maneuver = {
                    'input/dave_image': self.input_dave_image,
                    'input/command': self.input_command,
                    'input/use_dropout': self.input_use_dropout,
                    'input/p_dropout': self.input_p_dropout
                }
                _input_speed = {
                    'input/alex_image': self.input_alex_image,
                    'input/command': self.input_command
                }
                _input_posor = {
                    'input/posor_image':
                        self.input_posor_image
                }
                tf.import_graph_def(self.maneuver_graph_def, input_map=_input_maneuver, name='fm')
                tf.import_graph_def(self.speed_graph_def, input_map=_input_speed, name='fs')
                tf.import_graph_def(self.posor_graph_def, input_map=_input_posor, name='fp')
                self.tf_steering = graph.get_tensor_by_name('fm/output/steering:0')
                self.tf_surprise = graph.get_tensor_by_name('fm/output/surprise:0')
                self.tf_critic = graph.get_tensor_by_name('fm/output/critic:0')
                self.tf_entropy = graph.get_tensor_by_name('fm/output/entropy:0')
                self.tf_brake = graph.get_tensor_by_name('fs/output/brake:0')
                self.tf_features = graph.get_tensor_by_name('fp/output/features:0')

    def deactivate(self):
        with self._lock:
            if self.sess is not None:
                self.sess.close()
                self.sess = None

    def forward(self, dave_image, alex_image, posor_image, turn, dagger=False):
        assert self.sess is not None, "There is no session - run activation prior to calling this method."
        _ops = [self.tf_steering, self.tf_brake, self.tf_surprise, self.tf_critic, self.tf_entropy, self.tf_features]
        _ret = (0, 1, 1, 1, 0, [0])
        if None in _ops:
            return _ret
        with self.sess.graph.as_default():
            feeder = {
                self.input_dave_image: [dave_image],
                self.input_alex_image: [alex_image],
                self.input_posor_image: [posor_image],
                self.input_use_dropout: dagger,
                self.input_p_dropout: self.p_conv_dropout if dagger else 0,
                self.input_command: [drive_cmd_vector(turn=turn)]
            }
            try:
                _steer, _brake, _surprise, _critic, _entropy, _features = self.sess.run(_ops, feed_dict=feeder)
                return _steer, max(0, _brake), _surprise, max(0, _critic), _entropy, _features.ravel()
            except (StandardError, CancelledError, FailedPreconditionError) as e:
                if isinstance(e, FailedPreconditionError):
                    logger.warning('FailedPreconditionError')
                else:
                    logger.warning(e)
                return _ret

    def features(self, image):
        assert self.sess is not None, "There is no session - run activation prior to calling this method."
        _ret = [0]
        if self.tf_features is None:
            return _ret
        with self.sess.graph.as_default():
            feeder = {self.input_posor_image: [image]}
            try:
                return self.sess.run(self.tf_features, feed_dict=feeder).ravel()
            except (StandardError, CancelledError) as e:
                logger.warning(e)
                return _ret


def _ros_init():
    # Ros replaces the root logger - add a new handler after ros initialisation.
    rospy.init_node('pilot', disable_signals=False, anonymous=True, log_level=rospy.INFO)
    console_handler = logging.StreamHandler(stream=sys.stdout)
    console_handler.setFormatter(logging.Formatter(log_format))
    logging.getLogger().addHandler(console_handler)
    logging.getLogger().setLevel(logging.INFO)
    rospy.on_shutdown(lambda: quit_event.set())


def hwc_bgr_to_yuv(img):
    return cv2.cvtColor(img, cv2.COLOR_BGR2YUV)


def hwc_to_chw(img):
    # Switch to C,H,W.
    return None if img is None else img.transpose((2, 0, 1))


def hwc_alexnet(image):
    image = cv2.resize(image, (227, 227))
    image = image.astype(np.uint8)
    return image


def caffe_dave_200_66(image, resize_wh=None, crop=(0, 0, 0, 0), dave=True, yuv=True, chw=True):
    # If resize is not the first operation, then resize the incoming image to the start of the data pipeline persistent images.
    image = image if resize_wh is None else cv2.resize(image, resize_wh)
    top, right, bottom, left = crop
    image = image[top:image.shape[0] - bottom, left:image.shape[1] - right]
    image = cv2.resize(image, (200, 66)) if dave else image
    image = hwc_bgr_to_yuv(image) if yuv else image
    image = hwc_to_chw(image) if chw else image
    image = image.astype(np.uint8)
    return image


def main():
    parser = argparse.ArgumentParser(description='Inference server.')
    parser.add_argument('--gpu', type=int, default=0, help='GPU number')
    args = parser.parse_args()

    # context = zmq.Context()
    # socket = context.socket(zmq.SUB)
    # socket.setsockopt(zmq.RCVHWM, 1)
    # socket.setsockopt(zmq.RCVTIMEO, 20)  # ms
    capture = cv2.VideoCapture('/dev/video5')

    try:
        # socket.connect('ipc:///tmp/byodr/zmq.sock')
        # socket.setsockopt(zmq.SUBSCRIBE, b'topic/image')

        _driver = TFDriver(gpu_id=args.gpu, p_conv_dropout=0)
        _driver.activate()

        _ros_init()
        dnn_topic = rospy.Publisher('aav/inference/state/blob', RosString, queue_size=1)

        def _camera(_msg):
            try:
                # [_, md, data] = socket.recv_multipart(flags=0, copy=True, track=False)
                # md = json.loads(md)
                # height, width, channels = md['shape']
                # img = np.frombuffer(buffer(data), dtype=np.uint8)
                # img = img.reshape((height, width, channels))
                suc, img = capture.read()
                _dave_img = caffe_dave_200_66(img)
                _alex_img = hwc_alexnet(img)
                action_out, brake_out, surprise_out, critic_out, entropy_out, conv5_out = \
                    _driver.forward(dave_image=_dave_img,
                                    alex_image=_alex_img,
                                    posor_image=_alex_img,
                                    turn='intersection.ahead',
                                    dagger=False)
                dnn_topic.publish(json.dumps(dict(action=float(action_out))))
            except Exception as e:
                logger.warning(e)

        rospy.Subscriber('aav/vehicle/camera/0', RosString, _camera, queue_size=1)
        rospy.spin()
    except KeyboardInterrupt:
        quit_event.set()

    # logger.info("Waiting on zmq to terminate.")
    # socket.close()
    # context.term()
    capture.release()


if __name__ == "__main__":
    logging.basicConfig(format=log_format)
    logging.getLogger().setLevel(logging.DEBUG)
    main()
