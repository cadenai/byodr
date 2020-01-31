import argparse
import collections
import json
import logging
import multiprocessing
import signal
import threading
import time
import traceback
from functools import partial

import numpy as np
import zmq
from jsoncomment import JsonComment

from image import get_registered_function
from inference import TFDriver, DynamicMomentum

logger = logging.getLogger(__name__)
quit_event = multiprocessing.Event()

signal.signal(signal.SIGINT, lambda sig, frame: _interrupt())
signal.signal(signal.SIGTERM, lambda sig, frame: _interrupt())


def _interrupt():
    logger.info("Received interrupt, quitting.")
    quit_event.set()


# noinspection PyUnresolvedReferences
class ReceiverThread(threading.Thread):
    def __init__(self, url, topic=''):
        super(ReceiverThread, self).__init__()
        subscriber = zmq.Context().socket(zmq.SUB)
        subscriber.setsockopt(zmq.RCVHWM, 1)
        subscriber.setsockopt(zmq.RCVTIMEO, 10)
        subscriber.setsockopt(zmq.LINGER, 0)
        subscriber.connect(url)
        subscriber.setsockopt(zmq.SUBSCRIBE, topic)
        self._subscriber = subscriber
        self._queue = collections.deque(maxlen=1)

    def get_latest(self):
        return self._queue[0] if bool(self._queue) else None

    def run(self):
        while not quit_event.is_set():
            try:
                self._queue.appendleft(json.loads(self._subscriber.recv().split(':', 1)[1]))
            except zmq.Again:
                pass


# noinspection PyUnresolvedReferences
class CameraThread(threading.Thread):
    def __init__(self):
        super(CameraThread, self).__init__()
        subscriber = zmq.Context().socket(zmq.SUB)
        subscriber.setsockopt(zmq.RCVHWM, 1)
        subscriber.setsockopt(zmq.RCVTIMEO, 20)
        subscriber.setsockopt(zmq.LINGER, 0)
        subscriber.connect('ipc:///byodr/camera.sock')
        subscriber.setsockopt(zmq.SUBSCRIBE, b'aav/camera/0')
        self._subscriber = subscriber
        self._images = collections.deque(maxlen=1)

    def capture(self):
        return self._images[0] if bool(self._images) else None

    def run(self):
        while not quit_event.is_set():
            try:
                [_, md, data] = self._subscriber.recv_multipart()
                md = json.loads(md)
                height, width, channels = md['shape']
                img = np.frombuffer(buffer(data), dtype=np.uint8)
                img = img.reshape((height, width, channels))
                self._images.appendleft(img)
            except zmq.Again:
                pass


# noinspection PyUnresolvedReferences
class Publisher(object):
    def __init__(self):
        publisher = zmq.Context().socket(zmq.PUB)
        publisher.bind('ipc:///byodr/inference.sock')
        self._publisher = publisher

    def publish(self, data):
        self._publisher.send('aav/inference/state:{}'.format(json.dumps(data)), zmq.NOBLOCK)


class TFRunner(object):
    def __init__(self, **kwargs):
        self._lock = multiprocessing.Lock()
        self._dagger = False
        self._steering_scale_left = abs(float(kwargs['driver.dnn.steering.scale.left']))
        self._steering_scale_right = float(kwargs['driver.dnn.steering.scale.right'])
        _penalty_up_momentum = float(kwargs['driver.autopilot.filter.momentum.up'])
        _penalty_down_momentum = float(kwargs['driver.autopilot.filter.momentum.down'])
        _penalty_ceiling = float(kwargs['driver.autopilot.filter.ceiling'])
        self._penalty_filter = DynamicMomentum(up=_penalty_up_momentum,
                                               down=_penalty_down_momentum,
                                               ceiling=_penalty_ceiling)
        _brake_scale_min = float(kwargs['driver.dnn.obstacle.scale.min'])
        _brake_scale_max = float(kwargs['driver.dnn.obstacle.scale.max'])
        _corridor_scale_min = float(kwargs['driver.dnn.steer.corridor.scale.min'])
        _corridor_scale_max = float(kwargs['driver.dnn.steer.corridor.scale.max'])
        self._fn_obstacle_norm = partial(self._norm_scale, min_=_brake_scale_min, max_=_brake_scale_max)
        self._fn_corridor_norm = partial(self._norm_scale, min_=_corridor_scale_min, max_=_corridor_scale_max)
        p_conv_dropout = float(kwargs['driver.dnn.dagger.conv.dropout'])
        self._fn_dave_image = get_registered_function(kwargs['dnn.image.transform.dave'])
        self._fn_alex_image = get_registered_function(kwargs['dnn.image.transform.alex'])
        self._driver = TFDriver(model_directory=kwargs['model_directory'], gpu_id=kwargs['gpu_id'], p_conv_dropout=p_conv_dropout)
        self._driver.activate()

    def quit(self):
        self._driver.deactivate()

    def set_dagger(self, value):
        with self._lock:
            self._dagger = value

    def _dnn_steering(self, raw):
        return raw * (self._steering_scale_left if raw < 0 else self._steering_scale_right)

    @staticmethod
    def _norm_scale(v, min_=0., max_=1.):
        """Zero values below the minimum but let values larger than the maximum be scaled up. """
        return abs(max(0., v - min_) / (max_ - min_))

    def forward(self, image, turn='intersection.ahead'):
        with self._lock:
            dagger = self._dagger
        _dave_img = self._fn_dave_image(image)
        _alex_img = self._fn_alex_image(image)
        action_out, brake_out, surprise_out, critic_out, entropy_out = \
            self._driver.forward(dave_image=_dave_img,
                                 alex_image=_alex_img,
                                 posor_image=_alex_img,
                                 turn=turn,
                                 dagger=dagger)

        # The critic is a good indicator at inference time which is why the difference between the two values does not work.
        # Both surprise and critic are standard deviations.
        # Using the geometric mean would lessen the impact of large differences between the values.
        _corridor_uncertainty = np.mean([surprise_out, critic_out])
        _corridor_penalty = self._fn_corridor_norm(_corridor_uncertainty)

        # Penalties to decrease desired speed.
        _obstacle_penalty = self._fn_obstacle_norm(brake_out)
        # _obstacle_penalty += _obstacle_penalty * _norm_speed * self._brake_speed_multiplier
        _total_penalty = max(0, min(1, self._penalty_filter.calculate(_corridor_penalty + _obstacle_penalty)))

        return dict(action=float(self._dnn_steering(action_out)),
                    brake=float(brake_out),
                    corridor=float(_corridor_penalty),
                    critic=float(critic_out),
                    dagger=int(dagger),
                    entropy=float(entropy_out),
                    obstacle=float(_obstacle_penalty),
                    penalty=float(_total_penalty),
                    surprise=float(surprise_out),
                    time=time.time()
                    )


def main():
    parser = argparse.ArgumentParser(description='Inference server.')
    parser.add_argument('--models', type=str, required=True, help='Directory with the inference models.')
    parser.add_argument('--config', type=str, required=True, help='Config file location.')
    parser.add_argument('--clock', type=int, required=True, help='Clock frequency in hz.')
    parser.add_argument('--gpu', type=int, default=0, help='GPU number')
    args = parser.parse_args()

    with open(args.config, 'r') as cfg_file:
        cfg = JsonComment(json).loads(cfg_file.read())
    for key in sorted(cfg):
        logger.info("{} = {}".format(key, cfg[key]))

    _process_frequency = args.clock
    logger.info("Processing at {} Hz.".format(_process_frequency))
    max_duration = 1. / _process_frequency

    threads = []
    teleop = ReceiverThread(url='ipc:///byodr/teleop.sock', topic=b'aav/teleop/input')
    camera = CameraThread()
    threads.append(teleop)
    threads.append(camera)
    [t.start() for t in threads]
    publisher = Publisher()
    runner = TFRunner(model_directory=args.models, gpu_id=args.gpu, **cfg)
    try:
        while not quit_event.is_set():
            proc_start = time.time()
            command = teleop.get_latest()
            # Leave the value intact unless the control is activated.
            if command is not None and any([k in command.keys() for k in ('button_y', 'button_b', 'button_a', 'button_x')]):
                runner.set_dagger(command.get('button_x', 0) == 1)
            image = camera.capture()
            if image is not None:
                publisher.publish(runner.forward(image=image))
            _proc_sleep = max_duration - (time.time() - proc_start)
            if _proc_sleep < 0:
                logger.warning("Cannot maintain {} Hz.".format(_process_frequency))
            time.sleep(max(0, _proc_sleep))
    except KeyboardInterrupt:
        quit_event.set()
    except StandardError as e:
        logger.error("{}".format(traceback.format_exc(e)))
        quit_event.set()

    logger.info("Waiting on threads to stop.")
    [t.join() for t in threads]

    logger.info("Waiting on runner to quit.")
    runner.quit()


if __name__ == "__main__":
    logging.basicConfig(format='%(levelname)s: %(filename)s %(funcName)s %(message)s')
    logging.getLogger().setLevel(logging.DEBUG)
    main()
